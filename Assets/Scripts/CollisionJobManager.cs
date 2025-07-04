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
    private NativeArray<float3> velocities;
    private NativeArray<float3> positions;

    private JobHandle collisionJobHandle;

    // Reference to the parent system
    private OctreeSpringFiller parentSystem;

    public void InitializeArrays(OctreeSpringFiller parent, int pointCount, int connectionCount)
    {
        parentSystem = parent;

        velocities = new NativeArray<float3>(pointCount, Allocator.Persistent);
        positions = new NativeArray<float3>(pointCount, Allocator.Persistent);
    }

    [BurstCompile]
    struct GroundCollisionJob : IJobParallelFor
    {
        public NativeArray<float3> velocities;
        public NativeArray<float3> positions;

        [ReadOnly] public float groundLevel;
        [ReadOnly] public float groundBounce;
        [ReadOnly] public float groundFriction;

        public void Execute(int i)
        {
            if (positions[i].y < groundLevel)
            {
                positions[i] = new float3(positions[i].x, groundLevel, positions[i].z);
                velocities[i] = new float3(
                    velocities[i].x * groundFriction,
                    -velocities[i].y * groundBounce,
                    velocities[i].z * groundFriction
                );
            }
        }
    }

    public void ScheduleGroundCollisionJobs(float groundLevel, float groundBounce, float groundFriction)
    {
        for (int i = 0; i < parentSystem.allSpringPoints.Count; i++)
        {
            // Update positions and velocities
            velocities[i] = parentSystem.allSpringPoints[i].velocity;
            positions[i] = parentSystem.allSpringPoints[i].position;
        }

        var groundJob = new GroundCollisionJob
        {
            positions = positions,
            velocities = velocities,
            groundLevel = groundLevel,
            groundBounce = groundBounce,
            groundFriction = groundFriction
        };

        collisionJobHandle = groundJob.Schedule(positions.Length, 64);
    }

    public void CompleteAllJobsAndApply()
    {
        // Complete All jobs (for now only one)
        collisionJobHandle.Complete();

        // Apply forces to SpringPointTest objects
        for (int i = 0; i < parentSystem.allSpringPoints.Count; i++)
        {
            var velocityX = velocities[i].x;
            var velocityY = velocities[i].y;
            var velocityZ = velocities[i].z;
            Vector3 velocityVector = new Vector3(velocityX, velocityY, velocityZ);
            parentSystem.allSpringPoints[i].velocity = velocityVector;

            var positionX = positions[i].x;
            var positionY = positions[i].y;
            var positionZ = positions[i].z;
            Vector3 positionVector = new Vector3(positionX, positionY, positionZ);
            parentSystem.allSpringPoints[i].position = positionVector;
        }
    }

    private void OnDestroy()
    {
        if (positions.IsCreated) positions.Dispose();
        if (velocities.IsCreated) velocities.Dispose();
    }
}