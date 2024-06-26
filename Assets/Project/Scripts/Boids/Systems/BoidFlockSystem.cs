using Boids.Components;
using Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using UnityEngine;

namespace Boids.Systems
{
    public partial struct BoidFlockSystem : ISystem
    {
        private EntityQuery _boidQuery;
        private NativeArray<float3> _sphereRaysArray;

        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsWorldSingleton>();
            state.RequireForUpdate<BoidSetting>();
            _boidQuery = state.GetEntityQuery(new EntityQueryDesc()
            {
                All = new[]
                {
                    ComponentType.ReadOnly<Boid>(),
                    ComponentType.ReadWrite<LocalTransform>(),
                    ComponentType.ReadOnly<LocalToWorld>(),
                    ComponentType.ReadWrite<Velocity>(),
                }
            });

            _sphereRaysArray = BoidHelper.GetSphereRaysNativeArray(Allocator.Persistent);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var dependency = state.Dependency;
            int boidCount = _boidQuery.CalculateEntityCount();
            
            if(boidCount == 0) return;

            PhysicsWorldSingleton physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
            
            BoidSetting boidSetting = SystemAPI.GetSingleton<BoidSetting>();

            //Get Boids Velocity
            NativeArray<Velocity> velocityArr =
                CollectionHelper.CreateNativeArray<Velocity>(boidCount, state.WorldUpdateAllocator);
            var getVelocityJobHandle = new GetVelocityComponentNativeArrayJob()
            {
                array = velocityArr
            }.ScheduleParallel(_boidQuery, dependency);

            //Get Boids Transform
            NativeArray<LocalTransform> boidTransformArr = CollectionHelper.CreateNativeArray<LocalTransform>(boidCount,
                state.WorldUpdateAllocator);
            var getTransformJobHandle = new GetLocalTransformNativeArrayJob()
            {
                array = boidTransformArr
            }.ScheduleParallel(_boidQuery, dependency);
            
            JobHandle.CombineDependencies(getVelocityJobHandle, getTransformJobHandle).Complete();
            
            //Create different acceleration results to run rules job in parallel
            
            NativeArray<float3> cohesionDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            NativeArray<float3> alignmentDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            NativeArray<float3> separationDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            NativeArray<float3> collisionAvoidanceDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            
            NativeArray<float3> finalDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);

            //This job implement rule cohesion, steer toward flock center
            var boidCohesionJobHandle = new BoidCohesionJob()
            {
                boidTransformArr = boidTransformArr,
                velocityArr = velocityArr,
                boidSetting = boidSetting,
                desiredAccelerationArr = cohesionDesiredAccelArr
            }.ScheduleParallel(boidCount, 64, dependency);

            //This job implement rule alignment, steer to flock general direction
            var boidAlignmentJobHandle = new BoidAlignmentJob()
            {
                boidTransformArr = boidTransformArr,
                velocityArr = velocityArr,
                boidSetting = boidSetting,
                desiredAccelerationArr = alignmentDesiredAccelArr
            }.ScheduleParallel(boidCount, 64, dependency);

            //This job implement rule separation, if too close to each other, steer the hell away
            var boidSeparationJobHandle = new BoidSeparationJob()
            {
                boidTransformArr = boidTransformArr,
                velocityArr = velocityArr,
                boidSetting = boidSetting,
                desiredAccelerationArr = separationDesiredAccelArr
            }.ScheduleParallel(boidCount, 64, dependency);

            //This job is extra, detect collision and heavily influence direction to steer away obstacle
            var boidCollisionAvoidJobHandle = new BoidCollisionAvoidJob()
            {
                boidSetting = boidSetting,
                transformArr = boidTransformArr,
                desiredAccelArr = collisionAvoidanceDesiredAccelArr,
                velocityArr = velocityArr,
                sphereRaysArr = _sphereRaysArray,
                physicsWorld = physicsWorld.PhysicsWorld
            }.ScheduleParallel(boidCount, 64, dependency);

            NativeArray<JobHandle> jobHandles =
                CollectionHelper.CreateNativeArray<JobHandle>(4, state.WorldUpdateAllocator);
            jobHandles[0] = boidCohesionJobHandle;
            jobHandles[1] = boidAlignmentJobHandle;
            jobHandles[2] = boidSeparationJobHandle;
            jobHandles[3] = boidCollisionAvoidJobHandle;

            dependency = JobHandle.CombineDependencies(jobHandles);
            dependency.Complete();

            //Sums all the desired accels to get the general accel
            for (int i = 0; i < boidCount; i++)
            {
                var cohesionAccel = cohesionDesiredAccelArr[i] * boidSetting.cohesionWeight;
                var alignmentAccel = alignmentDesiredAccelArr[i] * boidSetting.alignmentWeight;
                var separationAccel = separationDesiredAccelArr[i] * boidSetting.separationWeight;
                var collisionAvoidanceAccel = collisionAvoidanceDesiredAccelArr[i] * boidSetting.collisionWeight;

                finalDesiredAccelArr[i] += cohesionAccel;
                finalDesiredAccelArr[i] += alignmentAccel;
                finalDesiredAccelArr[i] += separationAccel;
                finalDesiredAccelArr[i] += collisionAvoidanceAccel;
            }

            dependency = new BoidTranslateJob()
            {
                accelArr = finalDesiredAccelArr,
                boidSetting = boidSetting,
                dt = SystemAPI.Time.DeltaTime
            }.ScheduleParallel(_boidQuery, dependency);

            state.Dependency = dependency;
        }

        public void OnDestroy(ref SystemState state)
        {
            _sphereRaysArray.Dispose();
        }
    }

    [BurstCompile]
    public struct BoidCohesionJob : IJobFor
    {
        [ReadOnly] public NativeArray<LocalTransform> boidTransformArr;
        [ReadOnly] public NativeArray<Velocity> velocityArr;
        [ReadOnly] public BoidSetting boidSetting;

        [NativeDisableParallelForRestriction] public NativeArray<float3> desiredAccelerationArr;

        public void Execute(int index)
        {
            int current = index;

            desiredAccelerationArr[current] = new float3(0);
            LocalTransform transform = boidTransformArr[current];
            Velocity velocity = velocityArr[current];
            float3 center = new float3(0);
            int cellCount = 0;

            for (int i = 0; i < boidTransformArr.Length; i++)
            {
                if (i != current)
                {
                    LocalTransform target = boidTransformArr[i];

                    if (math.distancesq(target.Position, transform.Position) <=
                        boidSetting.perceptionRange * boidSetting.perceptionRange)
                    {
                        center += target.Position;
                        cellCount++;
                    }
                }
            }

            center /= cellCount;
            desiredAccelerationArr[current] =
                BoidHelper.SteerToward(velocity.value, (center-transform.Position), boidSetting.maxSpeed);
        }
    }

    [BurstCompile]
    public struct BoidAlignmentJob : IJobFor
    {
        [ReadOnly] public NativeArray<LocalTransform> boidTransformArr;
        [ReadOnly] public BoidSetting boidSetting;
        [ReadOnly] public NativeArray<Velocity> velocityArr;

        [NativeDisableParallelForRestriction] public NativeArray<float3> desiredAccelerationArr;

        public void Execute(int index)
        {
            int current = index;
            desiredAccelerationArr[current] = new float3(0);
            var velocity = velocityArr[current];
            float3 flockVelocity = new float3(0);
            int cellCount = 0;

            for (int i = 0; i < boidTransformArr.Length; i++)
            {
                if (current != i)
                {
                    LocalTransform target = boidTransformArr[i];
                    LocalTransform transform = boidTransformArr[current];

                    if (math.distancesq(target.Position, transform.Position) <= boidSetting.perceptionRange * boidSetting.perceptionRange)
                    {
                        flockVelocity += velocityArr[i].value;
                    }
                }
            }

            desiredAccelerationArr[current] = BoidHelper.SteerToward(velocity.value, flockVelocity, boidSetting.maxSpeed);
        }
    }

    [BurstCompile]
    public struct BoidSeparationJob : IJobFor
    {
        [ReadOnly] public NativeArray<LocalTransform> boidTransformArr;
        [ReadOnly] public NativeArray<Velocity> velocityArr;
        [ReadOnly] public BoidSetting boidSetting;

        [NativeDisableParallelForRestriction] public NativeArray<float3> desiredAccelerationArr;

        public void Execute(int index)
        {
            int current = index;
            desiredAccelerationArr[current] = new float3(0);
            Velocity velocity = velocityArr[current];
            float3 direction = new float3(0);

            for (int i = 0; i < boidTransformArr.Length; i++)
            {
                if (current != i)
                {
                    LocalTransform target = boidTransformArr[i];
                    LocalTransform transform = boidTransformArr[current];

                    float distance = math.distance(target.Position, transform.Position);

                    if (distance <= boidSetting.separationRange)
                    {
                        direction += (transform.Position - target.Position) * (1f / distance);
                    }
                }
            }

            desiredAccelerationArr[current] = BoidHelper.SteerToward(velocity.value, direction, boidSetting.maxSpeed);
        }
    }
    
    [BurstCompile]
    public struct BoidCollisionAvoidJob : IJobFor
    {
        [ReadOnly] public PhysicsWorld physicsWorld;
        [ReadOnly] public BoidSetting boidSetting;
        [ReadOnly] public NativeArray<LocalTransform> transformArr;
        [ReadOnly] public NativeArray<Velocity> velocityArr;
        [ReadOnly] public NativeArray<float3> sphereRaysArr;

        [NativeDisableParallelForRestriction] public NativeArray<float3> desiredAccelArr;
        
        public void Execute(int index)
        {
            LocalTransform transform = transformArr[index];
            Velocity velocity = velocityArr[index];

            desiredAccelArr[index] = new float3(0);
            
            if (physicsWorld.SphereCast(transform.Position, boidSetting.collisionRange, float3.zero, boidSetting.collisionRange, CollisionFilter.Default))
            {
                bool foundDesireDirection = false;
                float3 desireDirection = default;

                for (int i = 0; i < sphereRaysArr.Length; i++)
                {
                    float3 rayDir = transform.TransformDirection(math.normalizesafe(sphereRaysArr[i]));

                    RaycastInput rayInput = new RaycastInput()
                    {
                        Start = transform.Position,
                        End = transform.Position + rayDir * boidSetting.collisionRange,
                        Filter = CollisionFilter.Default
                    };
                        
                    if (!physicsWorld.CastRay(rayInput))
                    {
                        foundDesireDirection = true;
                        desireDirection = rayDir;
                        break;
                    }
                }

                if (foundDesireDirection)
                {
                    desiredAccelArr[index] =
                        BoidHelper.SteerToward(velocity.value, desireDirection, boidSetting.maxSpeed);
                }
            }
        }
    }

    [BurstCompile]
    public partial struct BoidTranslateJob : IJobEntity
    {
        public NativeArray<float3> accelArr;

        public BoidSetting boidSetting;

        public float dt;

        [BurstCompile]
        public void Execute(
            ref LocalTransform localTransform,
            ref Velocity velocity,
            [EntityIndexInQuery] int index
            )
        {
            var acceleration = accelArr[index];

            velocity.value += acceleration * dt;

            float mag = math.length(velocity.value);
            float3 dir = velocity.value / mag;

            mag = math.clamp(mag, boidSetting.minSpeed, boidSetting.maxSpeed);
            velocity.value = mag * dir;

            localTransform.Position += velocity.value * dt;
            localTransform.Rotation = quaternion.LookRotation(dir, math.up());
        }
    }
    
    
}