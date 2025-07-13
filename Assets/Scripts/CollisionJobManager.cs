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

    [BurstCompile]
    struct RoomCollisionJob : IJobParallelFor
    {
        public NativeArray<SpringPointData> springPoints;

        [ReadOnly] public float3 minBounds;
        [ReadOnly] public float3 maxBounds;

        [ReadOnly] public float bounce;
        [ReadOnly] public float friction;

        public void Execute(int i)
        {
            SpringPointData point = springPoints[i];
            float3 pos = point.position;
            float3 vel = point.velocity;

            bool collided = false;

            // Check X axis
            if (pos.x < minBounds.x)
            {
                pos.x = minBounds.x;
                vel.x = -vel.x * bounce;
                vel.y *= friction;
                vel.z *= friction;
                collided = true;
            }
            else if (pos.x > maxBounds.x)
            {
                pos.x = maxBounds.x;
                vel.x = -vel.x * bounce;
                vel.y *= friction;
                vel.z *= friction;
                collided = true;
            }

            // Check Y axis
            if (pos.y < minBounds.y)
            {
                pos.y = minBounds.y;
                vel.y = -vel.y * bounce;
                vel.x *= friction;
                vel.z *= friction;
                collided = true;
            }
            else if (pos.y > maxBounds.y)
            {
                pos.y = maxBounds.y;
                vel.y = -vel.y * bounce;
                vel.x *= friction;
                vel.z *= friction;
                collided = true;
            }

            // Check Z axis
            if (pos.z < minBounds.z)
            {
                pos.z = minBounds.z;
                vel.z = -vel.z * bounce;
                vel.x *= friction;
                vel.y *= friction;
                collided = true;
            }
            else if (pos.z > maxBounds.z)
            {
                pos.z = maxBounds.z;
                vel.z = -vel.z * bounce;
                vel.x *= friction;
                vel.y *= friction;
                collided = true;
            }

            if (collided)
            {
                point.position = pos;
                point.velocity = vel;
                springPoints[i] = point;
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

    public void ScheduleRoomCollisionJob(float3 minBounds, float3 maxBounds, float bounce, float friction)
    {
        var roomCollisionJob = new RoomCollisionJob
        {
            springPoints = springPoints,
            minBounds = minBounds,
            maxBounds = maxBounds,
            bounce = bounce,
            friction = friction
        };

        collisionJobHandle = roomCollisionJob.Schedule(springPoints.Length, 64);
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