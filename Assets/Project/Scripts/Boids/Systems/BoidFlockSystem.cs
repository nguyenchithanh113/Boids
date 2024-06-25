using Boids.Components;
using Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Boids.Systems
{
    public partial struct BoidFlockSystem : ISystem
    {
        private EntityQuery _boidQuery;

        public void OnCreate(ref SystemState state)
        {
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
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var dependency = state.Dependency;
            int boidCount = _boidQuery.CalculateEntityCount();
            BoidSetting boidSetting = SystemAPI.GetSingleton<BoidSetting>();

            NativeArray<Velocity> velocityArr =
                CollectionHelper.CreateNativeArray<Velocity>(boidCount, state.WorldUpdateAllocator);
            var getVelocityJobHandle = new GetVelocityComponentNativeArrayJob()
            {
                array = velocityArr
            }.ScheduleParallel(_boidQuery, dependency);
            getVelocityJobHandle.Complete();

            NativeArray<LocalTransform> boidTransformArr = CollectionHelper.CreateNativeArray<LocalTransform>(boidCount,
                state.WorldUpdateAllocator);
            
            NativeArray<float3> cohesionDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            NativeArray<float3> alignmentDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            NativeArray<float3> separationDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);
            
            NativeArray<float3> finalDesiredAccelArr = CollectionHelper.CreateNativeArray<float3>(boidCount,
                state.WorldUpdateAllocator);


            var boidCohesionJobHandle = new BoidCohesionJob()
            {
                boidTransformArr = boidTransformArr,
                velocityArr = velocityArr,
                boidSetting = boidSetting,
                desiredAccelerationArr = cohesionDesiredAccelArr
            }.ScheduleParallel(boidCount, 64, dependency);

            var boidAlignmentJobHandle = new BoidAlignmentJob()
            {
                boidTransformArr = boidTransformArr,
                velocityArr = velocityArr,
                boidSetting = boidSetting,
                desiredAccelerationArr = alignmentDesiredAccelArr
            }.ScheduleParallel(boidCount, 64, dependency);

            var boidSeparationJobHandle = new BoidSeparationJob()
            {
                boidTransformArr = boidTransformArr,
                velocityArr = velocityArr,
                boidSetting = boidSetting,
                desiredAccelerationArr = separationDesiredAccelArr
            }.ScheduleParallel(boidCount, 64, dependency);

            dependency = JobHandle.CombineDependencies(boidCohesionJobHandle, boidAlignmentJobHandle,
                boidSeparationJobHandle);
            dependency.Complete();

            for (int i = 0; i < boidCount; i++)
            {
                var cohesionAccel = cohesionDesiredAccelArr[i] * boidSetting.cohesionWeight;
                var alignmentAccel = alignmentDesiredAccelArr[i] * boidSetting.alignmentWeight;
                var separationAccel = separationDesiredAccelArr[i] * boidSetting.separationWeight;

                finalDesiredAccelArr[i] += cohesionAccel;
                finalDesiredAccelArr[i] += alignmentAccel;
                finalDesiredAccelArr[i] += separationAccel;
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
                        boidSetting.collisionRange * boidSetting.collisionRange)
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
            float3 direction = new float3(0);

            for (int i = 0; i < boidTransformArr.Length; i++)
            {
                if (current != i)
                {
                    LocalTransform target = boidTransformArr[i];
                    LocalTransform transform = boidTransformArr[current];

                    if (math.distancesq(target.Position, transform.Position) <= boidSetting.perceptionRange * boidSetting.perceptionRange)
                    {
                        direction += target.Forward();
                    }
                }
            }

            desiredAccelerationArr[current] = BoidHelper.SteerToward(velocity.value, direction, boidSetting.maxSpeed);
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

    public partial struct BoidTranslateJob : IJobEntity
    {
        public NativeArray<float3> accelArr;

        public BoidSetting boidSetting;

        public float dt;

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