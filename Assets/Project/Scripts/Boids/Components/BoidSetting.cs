using Unity.Entities;

namespace Boids.Components
{
    public struct BoidSetting : IComponentData
    {
        public float maxSpeed;
        public float minSpeed;

        public float perceptionRange;
        public float separationRange;
        public float collisionRange;
        
        public float alignmentWeight;
        public float cohesionWeight;
        public float separationWeight;

        public Entity boidPrefab;
    }
}