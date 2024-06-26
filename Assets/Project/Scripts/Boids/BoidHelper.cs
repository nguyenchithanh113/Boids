using Unity.Collections;
using Unity.Mathematics;

namespace Boids
{
    public static class BoidHelper
    {
        const int rayCount = 300;
        public static readonly float3[] sphereRays;

        static BoidHelper () {
            sphereRays = new float3[rayCount];

            float goldenRatio = (1 + math.sqrt (5)) / 2;
            float angleIncrement = math.PI * 2 * goldenRatio;

            for (int i = 0; i < rayCount; i++) {
                float t = (float) i / rayCount;
                float inclination = math.acos(1 - 2 * t);
                float azimuth = angleIncrement * i;

                float x = math.sin (inclination) * math.cos (azimuth);
                float y = math.sin (inclination) * math.sin (azimuth);
                float z = math.cos (inclination);
                sphereRays[i] = new float3 (x, y, z);
            }
        }

        public static NativeArray<float3> GetSphereRaysNativeArray(Allocator allocator)
        {
            NativeArray<float3> array = new NativeArray<float3>(rayCount, allocator);
            
            for (int i = 0; i < rayCount; i++)
            {
                array[i] = sphereRays[i];
            }

            return array;
        }
        
        public static float3 SteerToward(float3 velocity, float3 direction, float maxSpeex)
        {
            return math.normalizesafe(direction) * maxSpeex - velocity;
        }
    }
}