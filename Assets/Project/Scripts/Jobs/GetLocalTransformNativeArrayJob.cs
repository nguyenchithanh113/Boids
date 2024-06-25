using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace Jobs
{
    [BurstCompile]
    public partial struct GetLocalTransformNativeArrayJob : IJobEntity
    {
        [NativeDisableParallelForRestriction] public NativeArray<LocalTransform> array;
        public void Execute(
            in LocalTransform localTransform,
            [EntityIndexInQuery] int index)
        {
            array[index] = localTransform;
        }
    }
}