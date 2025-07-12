using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


public class RigidJobManager : MonoBehaviour
{
    private NativeArray<SpringPointData> springPoints;
    private NativeArray<SpringConnectionData> springConnections;

    private JobHandle gravityJobHandle;
    private JobHandle rigidJobHandle;
    private JobHandle pointJobHandle;

    /// <summary>
    /// Initializes the collision system with spring points.
    /// NOTE: This class will NOT take ownership or dispose of the springPoints array.
    /// </summary>
    public void InitializeArrays(NativeArray<SpringPointData> springPoints, NativeArray<SpringConnectionData> springConnections)
    {
        this.springPoints = springPoints;
        this.springConnections = springConnections;
    }

    [BurstCompile]
    public struct GravityJob : IJobParallelFor
    {
        [ReadOnly] public float3 gravity;
        [ReadOnly] public bool applyGravity;
        public NativeArray<SpringPointData> springPoints;

        public void Execute(int index)
        {
            SpringPointData point = springPoints[index];
            if (applyGravity && point.isFixed == 0)
            {
                // Add gravity force to each point
                point.force += gravity * point.mass;
            }
            springPoints[index] = point;
        }
    }

    [BurstCompile]
    struct IntegrateForcesJob : IJobParallelFor
    {
        public NativeArray<SpringPointData> springPoints;

        [ReadOnly] public float deltaTime;

        public void Execute(int index)
        {
            SpringPointData point = springPoints[index];
            if (point.isFixed != 0) return;

            // --- NaN/Origin Checks ---
            float3 position = point.predictedPosition;
            if (math.any(math.isnan(position)))
            {
                point.force = float3.zero;
                point.velocity = float3.zero;
                springPoints[index] = point;
                return;
            }

            // Prevent division by zero
            float mass = math.max(point.mass, 1f);

            // --- Force/Velocity Validation ---
            float3 force = point.force;
            if (!math.any(math.isnan(force)))
            {
                float3 acceleration = force / mass;
                float3 velocity = point.velocity + (acceleration * deltaTime);

                // More conservative velocity clamping (50 units/s squared)
                if (math.lengthsq(velocity) > 2500f)
                {
                    velocity = math.normalize(velocity) * 50f;
                }

                point.velocity = velocity;
            }

            float3 predictedPosition = position + (point.velocity * deltaTime);
            if (!math.any(math.isnan(predictedPosition)) && math.length(predictedPosition) < 100000f)
            {
                point.predictedPosition = predictedPosition;
            }
            else
            {
                point.velocity = float3.zero;
            }

            // Reset force for next integration
            point.force = float3.zero;

            springPoints[index] = point;
        }
    }

    [BurstCompile]
    struct RigidConstraintJob : IJob    // No ParallelFor
    {
        public NativeArray<SpringPointData> springPoints;
        public NativeArray<SpringConnectionData> springConnections;

        [ReadOnly] public float relaxation;

        public void Execute()
        {
            for (int i = 0; i < springConnections.Length; i++)
            {
                var conn = springConnections[i];
                var pointA = springPoints[conn.pointA];
                var pointB = springPoints[conn.pointB];

                float3 direction = pointB.predictedPosition - pointA.predictedPosition;

                float distance = math.length(direction);
                if (distance < 1e-6f || float.IsNaN(distance)) continue;

                direction = direction / distance;
                float stretch = distance - conn.restLength;
                float3 correction = direction * (stretch * 0.5f) * relaxation;

                // Add corrections with conditions
                if (pointA.isFixed == 0) pointA.predictedPosition += correction;
                if (pointB.isFixed == 0) pointB.predictedPosition -= correction;

                springPoints[conn.pointA] = pointA;
                springPoints[conn.pointB] = pointB;
            }
        }
    }

    [BurstCompile]
    struct UpdatePointJob : IJobParallelFor
    {
        public NativeArray<SpringPointData> springPoints;

        [ReadOnly] public float deltaTime;

        public void Execute(int index)
        {
            SpringPointData point = springPoints[index];
            if (point.isFixed != 0) return; // skip fixed

            // Update point velocity
            float3 velocity = (point.predictedPosition - point.position) / deltaTime;
            velocity *= 0.98f; // 0.98f ~ 2% damping

            // More conservative velocity clamping (50 units/s squared)
            if (math.lengthsq(velocity) > 2500f)
            {
                velocity = math.normalize(velocity) * 50f;
            }

            point.velocity = velocity;

            // Update point position
            point.position = point.predictedPosition;

            // Reset force for next frame
            point.force = float3.zero;

            springPoints[index] = point;
        }
    }

    public void ScheduleGravityJobs(float3 gravity, bool applyGravity)
    {
        var gravityJob = new GravityJob
        {
            gravity = gravity,
            applyGravity = applyGravity,
            springPoints = springPoints,
        };

        gravityJobHandle = gravityJob.Schedule(springPoints.Length, 64);
    }

    public void ScheduleRigidJobs(int checkIterations, float relaxation, float deltaTime)
    {
        var integrateJob = new IntegrateForcesJob
        {
            springPoints = springPoints,
            deltaTime = deltaTime,
        };

        rigidJobHandle = integrateJob.Schedule(springPoints.Length, 64, gravityJobHandle);

        // Predict positions and end Jobs
        JobHandle.CombineDependencies(gravityJobHandle, rigidJobHandle).Complete();

        // Correct predicted positions
        var constraintJob = new RigidConstraintJob
        {
            relaxation = relaxation,
            springPoints = springPoints,
            springConnections = springConnections,
        };

        JobHandle iterationHandle = new JobHandle();
        for (int i = 0; i < checkIterations; i++)   // Try 3-10 iterations
        {
            iterationHandle = constraintJob.Schedule(iterationHandle);
        }
        iterationHandle.Complete();

        var updatePointJob = new UpdatePointJob
        {
            springPoints = springPoints,
            deltaTime = deltaTime
        };

        pointJobHandle = updatePointJob.Schedule(springPoints.Length, 64);
    }

    public void CompleteAllJobsAndApply()
    {
        // Complete Last job
        pointJobHandle.Complete();
    }

    private void OnDestroy()
    {
        //if (springPoints.IsCreated) springPoints.Dispose();
        //if (springConnections.IsCreated) springConnections.Dispose();
    }
}
