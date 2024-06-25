using Boids.Components;
using Unity.Entities;
using UnityEngine;

namespace Boids.Authorings
{
    public class BoidSettingAuthoring : MonoBehaviour
    {
        public float maxSpeed = 8;
        public float minSpeed = 4;

        public float perceptionRange = 4;
        public float separationRange = 2;
        public float collisionRange = 2;
        
        public float alignmentWeight = 2;
        public float cohesionWeight = 1;
        public float separationWeight = 3;
        
        
        private class BoidSettingAuthoringBaker : Baker<BoidSettingAuthoring>
        {
            public override void Bake(BoidSettingAuthoring authoring)
            {
                Entity entity = GetEntity(TransformUsageFlags.None);
                
                AddComponent(entity, new BoidSetting()
                {
                    maxSpeed = authoring.maxSpeed,
                    minSpeed = authoring.minSpeed,
                    perceptionRange = authoring.perceptionRange,
                    separationRange = authoring.separationRange,
                    collisionRange = authoring.collisionRange,
                    alignmentWeight = authoring.alignmentWeight,
                    cohesionWeight = authoring.cohesionWeight,
                    separationWeight = authoring.cohesionWeight,
                });
            }
        }
    }
}