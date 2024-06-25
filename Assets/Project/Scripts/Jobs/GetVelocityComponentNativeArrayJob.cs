using Boids.Components;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace Jobs
{
    public partial struct GetVelocityComponentNativeArrayJob : IJobEntity
    {
        [NativeDisableParallelForRestriction] public NativeArray<Velocity> array;
        public void Execute(
            in Velocity velocity,
            [EntityIndexInQuery] int index)
        {
            array[index] = velocity;
        }
    }
}