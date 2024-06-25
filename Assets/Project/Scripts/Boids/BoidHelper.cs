using Unity.Mathematics;

namespace Boids
{
    public static class BoidHelper
    {
        public static float3 SteerToward(float3 velocity, float3 direction, float maxSpeex)
        {
            return math.normalizesafe(direction) * maxSpeex - velocity;
        }
    }
}