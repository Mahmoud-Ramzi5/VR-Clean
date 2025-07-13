using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


public class CollisionJobManager : MonoBehaviour
{
    private NativeArray<SpringPointData> springPoints;
    //private NativeArray<SpringConnectionData> springConnections;

    private JobHandle collisionJobHandle;

    public struct BodyBounds
    {
        public float3 min;
        public float3 max;
    }

    /// <summary>
    /// Initializes the collision system with spring points.
    /// NOTE: This class will NOT take ownership or dispose of the springPoints array.
    /// </summary>
    public void InitializeArrays(NativeArray<SpringPointData> springPoints)
    {
        this.springPoints = springPoints;
    }

    [BurstCompile]
    struct GroundCollisionJob : IJobParallelFor
    {
        public NativeArray<SpringPointData> springPoints;

        [ReadOnly] public float groundLevel;
        [ReadOnly] public float groundBounce;
        [ReadOnly] public float groundFriction;

        public void Execute(int i)
        {
            SpringPointData point = springPoints[i];
            float3 pointPosition = point.position;

            float combinedBounce = (point.bounciness + groundBounce) * 0.5f;
            float combinedFriction = math.sqrt(point.friction * groundFriction);

            if (pointPosition.y < groundLevel)
            {
                point.position = new float3(
                    pointPosition.x,
                    groundLevel,
                    pointPosition.z
                );

                point.velocity = new float3(
                    point.velocity.x * combinedFriction,
                    -point.velocity.y * combinedBounce,
                    point.velocity.z * combinedFriction
                );

                point.acc = float3.zero;
            }

            springPoints[i] = point;
        }
    }

    [BurstCompile]
    public struct BroadPhaseJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<BodyBounds> bounds;
        public NativeList<int2>.ParallelWriter collisionPairs;

        public void Execute(int index)
        {
            BodyBounds a = bounds[index];
            for (int j = index + 1; j < bounds.Length; j++)
            {
                BodyBounds b = bounds[j];
                if (BoundsIntersect(a, b))
                {
                    collisionPairs.AddNoResize(new int2(index, j));
                }
            }
        }

        private bool BoundsIntersect(BodyBounds a, BodyBounds b)
        {
            return math.all(a.max >= b.min) && math.all(a.min <= b.max);
        }
    }

    [BurstCompile]
    public struct ElasticCollisionJob : IJobParallelFor
    {
        public NativeArray<SpringPointData> springPoints;
        [ReadOnly] public float restitution;
        [ReadOnly] public float contactRadius;

        public void Execute(int index)
        {
            SpringPointData pointA = springPoints[index];
            if (pointA.isFixed != 0) return;

            for (int j = 0; j < springPoints.Length; j++)
            {
                if (index == j) continue;

                SpringPointData pointB = springPoints[j];
                if (pointB.isFixed != 0) continue;

                float3 delta = pointB.position - pointA.position;
                float dist = math.length(delta);

                if (dist < contactRadius && dist > 0.0001f)
                {
                    float3 normal = delta / dist;

                    float3 relVel = pointA.velocity - pointB.velocity;
                    float velAlongNormal = math.dot(relVel, normal);

                    if (velAlongNormal > 0f) continue; // moving apart

                    // Masses
                    float m1 = math.max(pointA.mass, 1e-3f);
                    float m2 = math.max(pointB.mass, 1e-3f);

                    // Impulse scalar
                    float jImpulse = -(1 + restitution) * velAlongNormal;
                    jImpulse /= (1 / m1 + 1 / m2);

                    float3 impulse = jImpulse * normal;

                    // Apply impulse
                    pointA.velocity += impulse / m1;
                    pointB.velocity -= impulse / m2;

                    springPoints[index] = pointA;
                    springPoints[j] = pointB;
                }
            }
        }
    }


    public void ScheduleGroundCollisionJobs(float groundLevel, float groundBounce, float groundFriction)
    {
        var groundJob = new GroundCollisionJob
        {
            springPoints = springPoints,

            groundLevel = groundLevel,
            groundBounce = groundBounce,
            groundFriction = groundFriction
        };

        collisionJobHandle = groundJob.Schedule(springPoints.Length, 64);
    }

    public void ScheduleElasticCollision(float contactRadius = 1.0f, float restitution = 1.0f)
    {
        var collisionJob = new ElasticCollisionJob
        {
            springPoints = springPoints,
            restitution = restitution,
            contactRadius = contactRadius,
        };

        collisionJobHandle = collisionJob.Schedule(springPoints.Length, 64, collisionJobHandle);
    }


    public void CompleteAllJobsAndApply()
    {
        collisionJobHandle.Complete();
    }

    private void OnDestroy()
    {
        //if (springPoints.IsCreated) springPoints.Dispose();
        //if (springConnections.IsCreated) springConnections.Dispose();
    }
}