using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


public class SpringJobManager : MonoBehaviour
{
    // NativeMultiHashMap was not found
    private NativeParallelMultiHashMap<int, float3> forceMap;

    private NativeArray<SpringPointData> springPoints;
    private NativeArray<SpringConnectionData> springConnections;

    private JobHandle gravityJobHandle;
    private JobHandle springJobHandle;
    private JobHandle pointJobHandle;

    /// <summary>
    /// Initializes the collision system with spring points.
    /// NOTE: This class will NOT take ownership or dispose of the springPoints array.
    /// </summary>
    public void InitializeArrays(NativeArray<SpringPointData> springPoints, NativeArray<SpringConnectionData> springConnections)
    {
        this.springPoints = springPoints;
        this.springConnections = springConnections;

        // Initialize force map with estimated capacity
        int estimatedForceCount = springConnections.Length * 2 + springPoints.Length;
        forceMap = new NativeParallelMultiHashMap<int, float3>(estimatedForceCount, Allocator.Persistent);
    }

    [BurstCompile]
    public struct GravityJob : IJobParallelFor
    {
        public NativeParallelMultiHashMap<int, float3>.ParallelWriter forceMap;

        [ReadOnly] public float3 gravity;
        [ReadOnly] public bool applyGravity;
        [ReadOnly] public NativeArray<SpringPointData> springPoints;

        public void Execute(int index)
        {
            SpringPointData point = springPoints[index];
            if (applyGravity && point.isFixed == 0)
            {
                // Add gravity force to each point
                forceMap.Add(index, gravity * point.mass);
            }
        }
    }

    [BurstCompile]
    struct CalculateForcesJob : IJobParallelFor
    {
        public NativeParallelMultiHashMap<int, float3>.ParallelWriter forceMap;

        [ReadOnly] public NativeArray<SpringPointData> springPoints;
        [ReadOnly] public NativeArray<SpringConnectionData> springConnections;

        public void Execute(int connectionIndex)
        {
            var conn = springConnections[connectionIndex];
            var pointA = springPoints[conn.pointA];
            var pointB = springPoints[conn.pointB];

            float3 direction = pointB.position - pointA.position;
            float distance = math.length(direction);
            if (distance > 0f)
            {
                direction = direction / distance;
                // Calculate spring force using Hooke's Law
                float stretch = distance - conn.restLength;
                float3 springForce = conn.springConstant * stretch * direction;

                // Apply damping to prevent sliding at higher speeds
                float3 relativeVel = pointB.velocity - pointA.velocity;
                float velocityAlongSpring = math.dot(relativeVel, direction);
                float3 dampingForce = conn.damperConstant * velocityAlongSpring * direction;

                // Combine forces
                float3 netForce = springForce + dampingForce;

                // Add forces to the map
                forceMap.Add(conn.pointA, netForce);
                forceMap.Add(conn.pointB, -netForce);
            }
        }
    }

    [BurstCompile]
    struct AccumulateForcesJob : IJobParallelFor
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, float3> forceMap;

        public NativeArray<SpringPointData> springPoints;

        public void Execute(int index)
        {
            float3 totalForce = float3.zero;
            SpringPointData point = springPoints[index];
            if (forceMap.TryGetFirstValue(index, out float3 force, out var it))
            {
                do
                {
                    totalForce += force;
                } while (forceMap.TryGetNextValue(out force, ref it));
            }

            point.force = totalForce;
            springPoints[index] = point;
        }
    }

    [BurstCompile]
    struct UpdatePointJob : IJobParallelFor
    {
        public NativeArray<SpringPointData> springPoints;

        [ReadOnly] public float deltaTime;

        public void Execute(int index)
        {
            var point = springPoints[index];
            if (point.isFixed != 0) return;

            // --- NaN/Origin Checks ---
            float3 position = point.position;
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
                point.acc = acceleration;
            }

            // --- Position Update ---
            float3 newPosition = position + (point.velocity * deltaTime);
            if (!math.any(math.isnan(newPosition)) && math.length(newPosition) < 100000f)
            {
                point.position = newPosition;
            }
            else
            {
                point.velocity = float3.zero;
            }

            // Reset force for next frame
            point.force = float3.zero;
            springPoints[index] = point;
        }
    }

    public void ScheduleGravityJobs(float3 gravity, bool applyGravity)
    {
        // Clear the force map
        forceMap.Clear();

        var gravityJob = new GravityJob
        {
            forceMap = forceMap.AsParallelWriter(),

            gravity = gravity,
            applyGravity = applyGravity,
            springPoints = springPoints,
        };

        gravityJobHandle = gravityJob.Schedule(springPoints.Length, 64);
    }

    public void ScheduleSpringJobs(float deltaTime)
    {
        var calculateJob = new CalculateForcesJob
        {
            forceMap = forceMap.AsParallelWriter(),

            springPoints = springPoints,
            springConnections = springConnections,
        };

        var accumulateJob = new AccumulateForcesJob
        {
            forceMap = forceMap,
            springPoints = springPoints
        };

        var updatePointJob = new UpdatePointJob
        {
            springPoints = springPoints,
            deltaTime = deltaTime,
        };

        // Schedule with dependency chain
        springJobHandle = calculateJob.Schedule(springConnections.Length, 64, gravityJobHandle);
        springJobHandle = accumulateJob.Schedule(springPoints.Length, 64, springJobHandle);
        pointJobHandle = updatePointJob.Schedule(springPoints.Length, 64, springJobHandle);
    }

    public void CompleteAllJobsAndApply()
    {
        JobHandle.CombineDependencies(gravityJobHandle, springJobHandle, pointJobHandle).Complete();
        forceMap.Clear();
    }

    private void OnDestroy()
    {
        if (forceMap.IsCreated) forceMap.Dispose();
        //if (springPoints.IsCreated) springPoints.Dispose();
        //if (springConnections.IsCreated) springConnections.Dispose();
    }
}