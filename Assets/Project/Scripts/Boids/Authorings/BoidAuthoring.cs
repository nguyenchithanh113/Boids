using Boids.Components;
using Unity.Entities;
using UnityEngine;

namespace Boids.Authorings
{
    public class BoidAuthoring : MonoBehaviour
    {
        private class BoidAuthoringBaker : Baker<BoidAuthoring>
        {
            public override void Bake(BoidAuthoring authoring)
            {
                Entity entity = GetEntity(TransformUsageFlags.Dynamic);
                
                AddComponent(entity, new Boid());
            }
        }
    }
}