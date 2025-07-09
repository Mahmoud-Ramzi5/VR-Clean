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
        if (springPoints.IsCreated) springPoints.Dispose();
        //if (springConnections.IsCreated) springConnections.Dispose();
    }
}