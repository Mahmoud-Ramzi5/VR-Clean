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
            }

            springPoints[i] = point;
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

//testing

//public class CollisionJobManager : MonoBehaviour
//{
//    [Header("Collision Settings")]
//    public float penetrationSlop = 0.01f;
//    public float restitution = 0.6f;
//    public float friction = 0.4f;
//    public int maxContactsPerPair = 8;

//    // Core collision data
//    private NativeArray<SpringPointData> springPoints;
//    private NativeArray<ObjectCollisionData> collisionObjects;
//    private NativeList<CollisionPair> collisionPairs;
//    private NativeList<ContactPoint> contactPoints;

//    // Job handles
//    private JobHandle collisionJobHandle;

//    public void Initialize(NativeArray<SpringPointData> springPointsRef)
//    {
//        this.springPoints = springPointsRef;

//        // The only job of Initialize is now to ensure the native containers exist.
//        // The actual data is populated by UpdateCollisionObjects.
//        if (!collisionPairs.IsCreated)
//        {
//            collisionPairs = new NativeList<CollisionPair>(Allocator.Persistent);
//        }
//        if (!contactPoints.IsCreated)
//        {
//            contactPoints = new NativeList<ContactPoint>(Allocator.Persistent);
//        }
//    }

//    public void UpdateCollisionObjects(NativeArray<ObjectCollisionData> newObjects)
//    {
//        // If the list has changed size, we must reallocate the internal array.
//        if (!collisionObjects.IsCreated || collisionObjects.Length != newObjects.Length)
//        {
//            if (collisionObjects.IsCreated) collisionObjects.Dispose();
//            collisionObjects = new NativeArray<ObjectCollisionData>(newObjects.Length, Allocator.Persistent);
//        }

//        // Copy the latest data from the source array to our internal array.
//        newObjects.CopyTo(collisionObjects);
//    }

//    public void ScheduleCollisionJobs()
//    {
//        collisionPairs.Clear();
//        contactPoints.Clear();

//        // Phase 1: Broad Phase
//        var broadPhaseJob = new BroadPhaseCollisionJob
//        {
//            collisionObjects = this.collisionObjects,
//            collisionPairs = this.collisionPairs.AsParallelWriter()
//        };
//        var broadPhaseHandle = broadPhaseJob.Schedule(collisionObjects.Length, 32);

//        // Phase 2: Narrow Phase
//        var narrowPhaseJob = new NarrowPhaseCollisionJob
//        {
//            springPoints = this.springPoints,
//            collisionObjects = this.collisionObjects,
//            collisionPairs = this.collisionPairs.AsDeferredJobArray(),
//            contactPoints = this.contactPoints.AsParallelWriter(),
//            penetrationSlop = this.penetrationSlop
//        };
//        var narrowPhaseHandle = narrowPhaseJob.Schedule(collisionPairs, 1, broadPhaseHandle);

//        // Phase 3: Collision Response
//        var responseJob = new CollisionResponseJob
//        {
//            springPoints = this.springPoints,
//            contactPoints = this.contactPoints.AsDeferredJobArray(),
//            restitution = this.restitution,
//            friction = this.friction
//        };
//        collisionJobHandle = responseJob.Schedule(contactPoints, 32, narrowPhaseHandle);
//    }

//    public void CompleteCollisionJobs()
//    {
//        collisionJobHandle.Complete();
//    }

//    private void OnDestroy()
//    {
//        collisionJobHandle.Complete();
//        if (springPoints.IsCreated) springPoints.Dispose();
//        if (collisionObjects.IsCreated) collisionObjects.Dispose();
//        if (collisionPairs.IsCreated) collisionPairs.Dispose();
//        if (contactPoints.IsCreated) contactPoints.Dispose();
//    }

//    #region Jobs

//    [BurstCompile]
//    private struct BroadPhaseCollisionJob : IJobParallelFor
//    {
//        [ReadOnly] public NativeArray<ObjectCollisionData> collisionObjects;
//        [WriteOnly] public NativeList<CollisionPair>.ParallelWriter collisionPairs;

//        public void Execute(int i)
//        {
//            for (int j = i + 1; j < collisionObjects.Length; j++)
//            {
//                var objA = collisionObjects[i];
//                var objB = collisionObjects[j];

//                if (objA.isActive == 0 || objB.isActive == 0) continue;
//                if (!objA.layerMask.CanCollideWith(objB.layerMask)) continue;

//                if (objA.boundingVolume.Intersects(objB.boundingVolume))
//                {
//                    collisionPairs.AddNoResize(new CollisionPair { objectA = i, objectB = j });
//                }
//            }
//        }
//    }

//    [BurstCompile]
//    private struct NarrowPhaseCollisionJob : IJobParallelForDefer
//    {
//        [ReadOnly] public NativeArray<SpringPointData> springPoints;
//        [ReadOnly] public NativeArray<ObjectCollisionData> collisionObjects;
//        [ReadOnly] public NativeArray<CollisionPair> collisionPairs;
//        [WriteOnly] public NativeList<ContactPoint>.ParallelWriter contactPoints;
//        [ReadOnly] public float penetrationSlop;

//        public void Execute(int i)
//        {
//            var pair = collisionPairs[i];
//            var objA = collisionObjects[pair.objectA];
//            var objB = collisionObjects[pair.objectB];

//            for (int pA_idx = objA.surfacePointStartIndex; pA_idx < objA.surfacePointStartIndex + objA.surfacePointCount; pA_idx++)
//            {
//                var pointA = springPoints[pA_idx];
//                for (int pB_idx = objB.surfacePointStartIndex; pB_idx < objB.surfacePointStartIndex + objB.surfacePointCount; pB_idx++)
//                {
//                    var pointB = springPoints[pB_idx];

//                    float3 diff = pointA.position - pointB.position;
//                    float distSq = math.lengthsq(diff);
//                    float radiusSum = 0.1f; // Simplified radius

//                    if (distSq < radiusSum * radiusSum)
//                    {
//                        float dist = math.sqrt(distSq);
//                        float penetration = radiusSum - dist;
//                        if (penetration > penetrationSlop)
//                        {
//                            contactPoints.AddNoResize(new ContactPoint
//                            {
//                                objectA = pair.objectA,
//                                objectB = pair.objectB,
//                                pointIndexA = pA_idx,
//                                pointIndexB = pB_idx,
//                                normal = dist > 0 ? diff / dist : new float3(0, 1, 0),
//                                penetrationDepth = penetration,
//                                restitution = (pointA.bounciness + pointB.bounciness) * 0.5f,
//                                friction = math.sqrt(pointA.friction * pointB.friction)
//                            });
//                        }
//                    }
//                }
//            }
//        }
//    }

//    [BurstCompile]
//    private struct CollisionResponseJob : IJobParallelForDefer
//    {
//        public NativeArray<SpringPointData> springPoints;
//        [ReadOnly] public NativeArray<ContactPoint> contactPoints;
//        [ReadOnly] public float restitution;
//        [ReadOnly] public float friction;

//        public void Execute(int i)
//        {
//            var contact = contactPoints[i];
//            var pointA = springPoints[contact.pointIndexA];
//            var pointB = springPoints[contact.pointIndexB];

//            // Positional Correction
//            float invMassA = pointA.mass > 0 ? 1.0f / pointA.mass : 0;
//            float invMassB = pointB.mass > 0 ? 1.0f / pointB.mass : 0;
//            float totalInvMass = invMassA + invMassB;

//            if (totalInvMass > 0)
//            {
//                float3 correction = contact.normal * (contact.penetrationDepth / totalInvMass);
//                if (pointA.isFixed == 0) pointA.position += correction * invMassA;
//                if (pointB.isFixed == 0) pointB.position -= correction * invMassB;
//            }

//            // Impulse Resolution
//            float3 relativeVel = pointA.velocity - pointB.velocity;
//            float velAlongNormal = math.dot(relativeVel, contact.normal);

//            if (velAlongNormal < 0)
//            {
//                float e = math.min(pointA.bounciness, pointB.bounciness);
//                float j = -(1 + e) * velAlongNormal;
//                j /= totalInvMass;

//                float3 impulse = j * contact.normal;
//                if (pointA.isFixed == 0) pointA.velocity += impulse * invMassA;
//                if (pointB.isFixed == 0) pointB.velocity -= impulse * invMassB;

//                // Friction
//                float3 tangent = relativeVel - velAlongNormal * contact.normal;
//                if (math.lengthsq(tangent) > 0.0001f)
//                {
//                    tangent = math.normalize(tangent);
//                    float jt = -math.dot(relativeVel, tangent);
//                    jt /= totalInvMass;

//                    float mu = math.sqrt(pointA.friction * pointB.friction);
//                    float3 frictionImpulse = tangent * math.min(math.abs(jt), j * mu);

//                    if (pointA.isFixed == 0) pointA.velocity += frictionImpulse * invMassA;
//                    if (pointB.isFixed == 0) pointB.velocity -= frictionImpulse * invMassB;
//                }
//            }

//            springPoints[contact.pointIndexA] = pointA;
//            springPoints[contact.pointIndexB] = pointB;
//        }
//    }
//    #endregion