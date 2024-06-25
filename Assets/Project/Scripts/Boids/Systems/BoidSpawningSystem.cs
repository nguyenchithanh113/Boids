using Boids.Components;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Boids.Systems
{
    public partial struct BoidSpawningSystem : ISystem, ISystemStartStop
    {
        
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<BoidSetting>();
        }
        
        [BurstCompile]
        public void OnStartRunning(ref SystemState state)
        {
            
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

                var spawn = ecbWrite.Instantiate(index, boidPrefab);
                
                ecbWrite.AddComponent(index,spawn, new LocalTransform()
                {
                    Position = position,
                    Rotation = quaternion.identity
                });
                
            }
        }
    }
}