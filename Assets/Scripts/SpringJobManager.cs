using System;
using System.Collections.Generic;
using System.Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


public class SpringJobManager : MonoBehaviour
{
    // NativeMultiHashMap was not found
    private NativeParallelMultiHashMap<int, float3> forceMap;

    private NativeArray<float3> velocities;
    private NativeArray<float3> positions;
    private NativeArray<bool> isFixed;
    private NativeArray<float> masses;

    private NativeArray<int2> connections;
    private NativeArray<float> springConstants;
    private NativeArray<float> damperConstants;
    private NativeArray<float> restLengths;

    // Double buffered forces
    private NativeArray<float3> forcesBufferA;
    private NativeArray<float3> forcesBufferB;
    private bool usingForceBufferA;

    // Double buffered velocities
    private NativeArray<float3> velocitiesBufferA;
    private NativeArray<float3> velocitiesBufferB;
    private bool usingVelocityBufferA;

    // Double buffered positions
    private NativeArray<float3> positionsBufferA;
    private NativeArray<float3> positionsBufferB;
    private bool usingPositionBufferA;

    private JobHandle gravityJobHandle;
    private JobHandle springJobHandle;
    private JobHandle pointJobHandle;

    // Reference to the parent system
    private OctreeSpringFiller parentSystem;

    public void InitializeArrays(OctreeSpringFiller parent, int pointCount, int connectionCount)
    {
        parentSystem = parent;

        velocities = new NativeArray<float3>(pointCount, Allocator.Persistent);
        positions = new NativeArray<float3>(pointCount, Allocator.Persistent);
        isFixed = new NativeArray<bool>(pointCount, Allocator.Persistent);
        masses = new NativeArray<float>(pointCount, Allocator.Persistent);

        connections = new NativeArray<int2>(connectionCount, Allocator.Persistent);
        springConstants = new NativeArray<float>(connectionCount, Allocator.Persistent);
        damperConstants = new NativeArray<float>(connectionCount, Allocator.Persistent);
        restLengths = new NativeArray<float>(connectionCount, Allocator.Persistent);

        // Initialize both force buffers
        forcesBufferA = new NativeArray<float3>(pointCount, Allocator.Persistent);
        forcesBufferB = new NativeArray<float3>(pointCount, Allocator.Persistent);
        usingForceBufferA = true;

        // Initialize both velocity buffers
        velocitiesBufferA = new NativeArray<float3>(pointCount, Allocator.Persistent);
        velocitiesBufferB = new NativeArray<float3>(pointCount, Allocator.Persistent);
        usingVelocityBufferA = true;

        // Initialize both position buffers
        positionsBufferA = new NativeArray<float3>(pointCount, Allocator.Persistent);
        positionsBufferB = new NativeArray<float3>(pointCount, Allocator.Persistent);
        usingPositionBufferA = true;

        // Fill fixed and mass
        for (int i = 0; i < parent.allSpringPoints.Count; i++)
        {
            isFixed[i] = parent.allSpringPoints[i].isFixed;
            masses[i] = parent.allSpringPoints[i].mass;
        }

        // Initialize force map with estimated capacity
        int estimatedForceCount = connectionCount * 2 + pointCount;
        forceMap = new NativeParallelMultiHashMap<int, float3>(estimatedForceCount, Allocator.Persistent);
    }

    [BurstCompile]
    public struct GravityJob : IJobParallelFor
    {
        public NativeParallelMultiHashMap<int, float3>.ParallelWriter forceMap;

        [ReadOnly] public float3 gravity;
        [ReadOnly] public bool applyGravity;
        [ReadOnly] public NativeArray<float> masses;

        public void Execute(int index)
        {
            if (applyGravity)
            {
                // Add gravity force to each point
                forceMap.Add(index, gravity * masses[index]);
            }
        }
    }

    [BurstCompile]
    struct CalculateForcesJob : IJobParallelFor
    {
        public NativeParallelMultiHashMap<int, float3>.ParallelWriter forceMap;

        [ReadOnly] public NativeArray<float3> velocities;
        [ReadOnly] public NativeArray<float3> positions;

        [ReadOnly] public NativeArray<int2> connections;
        [ReadOnly] public NativeArray<float> springConstants;
        [ReadOnly] public NativeArray<float> damperConstants;
        [ReadOnly] public NativeArray<float> restLengths;

        public void Execute(int connectionIndex)
        {
            int2 points = connections[connectionIndex];
            float3 position1 = positions[points.x];
            float3 position2 = positions[points.y];

            float3 direction = position2 - position1;
            float distance = math.length(direction);
            if (distance > 0)
            {
                direction = direction / distance;
                // Calculate spring force using Hooke's Law
                float stretch = distance - restLengths[connectionIndex];
                float3 springForce = springConstants[connectionIndex] * stretch * direction;

                // Apply damping to prevent sliding at higher speeds
                float3 relativeVel = velocities[points.y] - velocities[points.x];
                float velocityAlongSpring = math.dot(relativeVel, direction);
                float3 dampingForce = damperConstants[connectionIndex] * velocityAlongSpring * direction;

                // Combine forces
                float3 netForce = springForce + dampingForce;

                // Add forces to the map
                forceMap.Add(points.x, netForce);
                forceMap.Add(points.y, -netForce);
            }
        }
    }

    [BurstCompile]
    struct AccumulateForcesJob : IJobParallelFor
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, float3> forceMap;
        [WriteOnly] public NativeArray<float3> accumulatedForces;

        public void Execute(int pointIndex)
        {
            float3 totalForce = float3.zero;

            if (forceMap.TryGetFirstValue(pointIndex, out float3 force, out var it))
            {
                do
                {
                    totalForce += force;
                } while (forceMap.TryGetNextValue(out force, ref it));
            }

            accumulatedForces[pointIndex] = totalForce;
        }
    }

    [BurstCompile]
    struct UpdatePointJob : IJobParallelFor
    {
        public NativeArray<float3> accumulatedForces;
        public NativeArray<float3> velocities;
        public NativeArray<float3> positions;
        public NativeArray<float> masses;
        public NativeArray<bool> isFixed;

        [ReadOnly] public float deltaTime;

        public void Execute(int pointIndex)
        {
            if (isFixed[pointIndex]) return;

            // --- NaN/Origin Checks ---
            float3 position = positions[pointIndex];
            if (math.any(math.isnan(position)))
            {
                accumulatedForces[pointIndex] = float3.zero;
                velocities[pointIndex] = float3.zero;
                return;
            }

            // Prevent division by zero
            float mass = math.max(masses[pointIndex], 1f);

            // --- Force/Velocity Validation ---
            float3 force = accumulatedForces[pointIndex];
            if (!math.any(math.isnan(force)))
            {
                float3 acceleration = force / mass;
                float3 velocity = velocities[pointIndex] + (acceleration * deltaTime);

                // More conservative velocity clamping (50 units/s squared)
                if (math.lengthsq(velocity) > 2500f)
                {
                    velocity = math.normalize(velocity) * 50f;
                }

                velocities[pointIndex] = velocity;
            }

            // --- Position Update ---
            float3 newPosition = position + (velocities[pointIndex] * deltaTime);
            if (!math.any(math.isnan(newPosition)) && math.length(newPosition) < 100000f)
            {
                positions[pointIndex] = newPosition;
            }
            else
            {
                velocities[pointIndex] = float3.zero;
            }

            // Reset force for next frame
            accumulatedForces[pointIndex] = float3.zero;
        }
    }

    public void ScheduleGravityJobs(float3 gravity, bool applyGravity)
    {
        // Clear the force map
        forceMap.Clear();

        var gravityJob = new GravityJob
        {
            forceMap = forceMap.AsParallelWriter(),

            masses = masses,
            gravity = gravity,
            applyGravity = applyGravity
        };

        gravityJobHandle = gravityJob.Schedule(masses.Length, 64);
    }

    public void ScheduleSpringJobs(float deltaTime)
    {
        // Clear the force buffer by setting each element to zero
        var currentForceBuffer = usingForceBufferA ? forcesBufferA : forcesBufferB; 
        for (int i = 0; i < currentForceBuffer.Length; i++)
        {
            currentForceBuffer[i] = float3.zero;
        }

        // Update positions and velocities from parent system
        var currentVelocityBuffer = usingVelocityBufferA ? velocitiesBufferA : velocitiesBufferB;
        var currentPositionBuffer = usingPositionBufferA ? positionsBufferA : positionsBufferB;
        for (int i = 0; i < parentSystem.allSpringPoints.Count; i++)
        {
            // Update positions and velocities
            velocities[i] = parentSystem.allSpringPoints[i].velocity;
            positions[i] = parentSystem.allSpringPoints[i].position;

            // Update all buffers to maintain consistency
            forcesBufferA[i] = (float3)parentSystem.allSpringPoints[i].force;
            forcesBufferB[i] = (float3)parentSystem.allSpringPoints[i].force;
            velocitiesBufferA[i] = (float3)parentSystem.allSpringPoints[i].velocity;
            velocitiesBufferB[i] = (float3)parentSystem.allSpringPoints[i].velocity;
            positionsBufferA[i] = (float3)parentSystem.allSpringPoints[i].position;
            positionsBufferB[i] = (float3)parentSystem.allSpringPoints[i].position;
        }

        var calculateJob = new CalculateForcesJob
        {
            forceMap = forceMap.AsParallelWriter(),

            velocities = currentVelocityBuffer,
            positions = currentPositionBuffer,

            connections = connections,
            restLengths = restLengths,

            springConstants = springConstants,
            damperConstants = damperConstants,
        };

        var accumulateJob = new AccumulateForcesJob
        {
            forceMap = forceMap,
            accumulatedForces = currentForceBuffer
        };


        var updatePointJob = new UpdatePointJob
        {
            accumulatedForces = currentForceBuffer,
            velocities = currentVelocityBuffer,
            positions = currentPositionBuffer,

            masses = masses,
            isFixed = isFixed,
            deltaTime = deltaTime,
        };

        // Schedule with dependency chain
        springJobHandle = calculateJob.Schedule(connections.Length, 64, gravityJobHandle);
        springJobHandle = accumulateJob.Schedule(parentSystem.allSpringPoints.Count, 64, springJobHandle);
        pointJobHandle = updatePointJob.Schedule(parentSystem.allSpringPoints.Count, 64, springJobHandle);
        
    }

    public void CompleteAllJobsAndApply()
    {
        // Complete All jobs
        JobHandle.CombineDependencies(gravityJobHandle, springJobHandle, pointJobHandle).Complete();

        // Get all buffers we finished writing to
        var completedForceBuffer = usingForceBufferA ? forcesBufferA : forcesBufferB;
        var completedVelocityBuffer = usingVelocityBufferA ? velocitiesBufferA : velocitiesBufferB;
        var completedPositionBuffer = usingPositionBufferA ? positionsBufferA : positionsBufferB;

        // Apply forces to SpringPointTest objects
        for (int i = 0; i < parentSystem.allSpringPoints.Count; i++)
        {
            // Convert float3 to Vector3
            var forceX = completedForceBuffer[i].x;
            var forceY = completedForceBuffer[i].y;
            var forceZ = completedForceBuffer[i].z;
            Vector3 forceVector = new Vector3(forceX, forceY, forceZ);
            parentSystem.allSpringPoints[i].force = forceVector;

            if (!parentSystem.allSpringPoints[i].isFixed)
            {
                var velocityX = completedVelocityBuffer[i].x;
                var velocityY = completedVelocityBuffer[i].y;
                var velocityZ = completedVelocityBuffer[i].z;
                Vector3 velocityVector = new Vector3(velocityX, velocityY, velocityZ);
                parentSystem.allSpringPoints[i].velocity = velocityVector;

                var positionX = completedPositionBuffer[i].x;
                var positionY = completedPositionBuffer[i].y;
                var positionZ = completedPositionBuffer[i].z;
                Vector3 positionVector = new Vector3(positionX, positionY, positionZ);
                parentSystem.allSpringPoints[i].position = positionVector;
            }
        }

        // Clear forces for next frame
        forceMap.Clear();

        // Switch buffers for next frame
        usingForceBufferA = !usingForceBufferA;
        usingVelocityBufferA = !usingVelocityBufferA;
        usingPositionBufferA = !usingPositionBufferA;
    }

    public void UpdateConnectionData(List<SpringConnection> connections)
    {
        // Update connection indices and rest lengths
        for (int i = 0; i < connections.Count; i++)
        {
            int index1 = parentSystem.allSpringPoints.IndexOf(connections[i].point1);
            int index2 = parentSystem.allSpringPoints.IndexOf(connections[i].point2);
            this.connections[i] = new int2(index1, index2);
            springConstants[i] = connections[i].springConstant;
            damperConstants[i] = connections[i].damperConstant;
            restLengths[i] = connections[i].restLength;
        }
    }

    private void OnDestroy()
    {
        if (forceMap.IsCreated) forceMap.Dispose();
        if (positions.IsCreated) positions.Dispose();
        if (velocities.IsCreated) velocities.Dispose();

        if (connections.IsCreated) connections.Dispose();
        if (springConstants.IsCreated) springConstants.Dispose();
        if (damperConstants.IsCreated) damperConstants.Dispose();
        if (restLengths.IsCreated) restLengths.Dispose();

        if (isFixed.IsCreated) isFixed.Dispose();
        if (masses.IsCreated) masses.Dispose();

        if (forcesBufferA.IsCreated) forcesBufferA.Dispose();
        if (forcesBufferB.IsCreated) forcesBufferB.Dispose();

        if (velocitiesBufferA.IsCreated) velocitiesBufferA.Dispose();
        if (velocitiesBufferB.IsCreated) velocitiesBufferB.Dispose();

        if (positionsBufferA.IsCreated) positionsBufferA.Dispose();
        if (positionsBufferB.IsCreated) positionsBufferB.Dispose();
    }
}
