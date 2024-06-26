using Boids.Components;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Random = Unity.Mathematics.Random;

namespace Boids.Systems
{
    public partial struct BoidSpawningSystem : ISystem, ISystemStartStop
    {
        private Random rand;
        
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<BoidSetting>();

            rand = new Random((uint)UnityEngine.Random.Range(0, 10000));
        }
        
        [BurstCompile]
        public void OnStartRunning(ref SystemState state)
        {
            BoidSetting boidSetting = SystemAPI.GetSingleton<BoidSetting>();
            
            int totalBoids = boidSetting.totalBoids;

            EntityCommandBuffer ecb = new EntityCommandBuffer(state.WorldUpdateAllocator);

            NativeArray<float3> randPositionArr =
                CollectionHelper.CreateNativeArray<float3>(totalBoids, state.WorldUpdateAllocator);
            NativeArray<Velocity> randVelocityArr =
                CollectionHelper.CreateNativeArray<Velocity>(totalBoids, state.WorldUpdateAllocator);

            for (int i = 0; i < totalBoids; i++)
            {
                float3 randPosition = rand.NextFloat3() * 5;
                float3 randVelocity = math.normalizesafe(rand.NextFloat3()) * boidSetting.minSpeed;

                randPositionArr[i] = randPosition;
                randVelocityArr[i] = new Velocity()
                {
                    value = randVelocity
                };
            }


            state.Dependency = new SpawnBoidJob()
            {
                positionArr = randPositionArr,
                velocityArr = randVelocityArr,
                boidPrefab = boidSetting.boidPrefab,
                ecbWrite = ecb.AsParallelWriter()
            }.ScheduleParallel(totalBoids, 64, state.Dependency);
            
            state.Dependency.Complete();
            
            ecb.Playback(state.EntityManager);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            
        }


        public void OnDestroy(ref SystemState state)
        {

        }

        public void OnStopRunning(ref SystemState state)
        {

        }
        
        public struct SpawnBoidJob : IJobFor
        {
            public NativeArray<float3> positionArr;
            public NativeArray<Velocity> velocityArr;

            public EntityCommandBuffer.ParallelWriter ecbWrite;

            [ReadOnly] public Entity boidPrefab;
            
            public void Execute(int index)
            {
                var position = positionArr[index];
                var velocity = velocityArr[index];

                var spawn = ecbWrite.Instantiate(index, boidPrefab);
                
                quaternion rot = quaternion.identity;

                if (!velocity.value.Equals(float3.zero))
                {
                    rot = quaternion.LookRotationSafe(velocity.value, math.up());
                }
                
                ecbWrite.SetComponent(index,spawn, new LocalTransform()
                {
                    Position = position,
                    Scale = 1,
                    Rotation = rot
                });
                
                ecbWrite.SetComponent(index, spawn, new Velocity()
                {
                    value = velocity.value
                });
                
            }
        }
    }
}