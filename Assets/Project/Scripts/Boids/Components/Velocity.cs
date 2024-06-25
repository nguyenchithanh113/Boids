using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Components
{
    public struct Velocity : IComponentData
    {
        public float3 value;
    }
}