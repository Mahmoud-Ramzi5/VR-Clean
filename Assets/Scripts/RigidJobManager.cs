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
    // NativeMultiHashMap was not found
    private NativeArray<float3> forces;
    private NativeArray<float3> velocities;
    private NativeArray<float3> positions;
    private NativeArray<float3> predictedPositions;

    private NativeArray<float> masses;
    private NativeArray<bool> isFixed;

    private NativeArray<int2> connections;
    private NativeArray<float> restLengths;

    private JobHandle gravityJobHandle;
    private JobHandle rigidJobHandle;
    private JobHandle pointJobHandle;

    // Reference to the parent system
    private OctreeSpringFiller parentSystem;

    public void InitializeArrays(OctreeSpringFiller parent, int pointCount, int connectionCount)
    {
        parentSystem = parent;

        forces = new NativeArray<float3>(pointCount, Allocator.Persistent);
        velocities = new NativeArray<float3>(pointCount, Allocator.Persistent);
        positions = new NativeArray<float3>(pointCount, Allocator.Persistent);
        predictedPositions = new NativeArray<float3>(pointCount, Allocator.Persistent);

        masses = new NativeArray<float>(pointCount, Allocator.Persistent);
        isFixed = new NativeArray<bool>(pointCount, Allocator.Persistent);

        connections = new NativeArray<int2>(connectionCount, Allocator.Persistent);
        restLengths = new NativeArray<float>(connectionCount, Allocator.Persistent);

        // Fill fixed and mass
        for (int i = 0; i < parent.allSpringPoints.Count; i++)
        {
            isFixed[i] = parent.allSpringPoints[i].isFixed;
            masses[i] = parent.allSpringPoints[i].mass;
        }
    }

    [BurstCompile]
    public struct GravityJob : IJobParallelFor
    {
        [ReadOnly] public float3 gravity;
        [ReadOnly] public bool applyGravity;
        [ReadOnly] public NativeArray<float> masses;
        [WriteOnly] public NativeArray<float3> forces;

        public void Execute(int index)
        {
            if (applyGravity)
            {
                // Add gravity force to each point
                forces[index] = gravity * masses[index];
            }
        }
    }

    [BurstCompile]
    struct IntegrateForcesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> forces;
        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public NativeArray<bool> isFixed;
        [ReadOnly] public float deltaTime;

        public NativeArray<float3> velocities;
        public NativeArray<float3> predictedPositions;


        public void Execute(int pointIndex)
        {
            if (isFixed[pointIndex]) return; // Skip fixed points

            // --- NaN/Origin Checks ---
            float3 position = predictedPositions[pointIndex];
            if (math.any(math.isnan(position)))
            {
                forces[pointIndex] = float3.zero;
                velocities[pointIndex] = float3.zero;
                return;
            }

            // Prevent division by zero
            float mass = math.max(masses[pointIndex], 1f);

            // --- Force/Velocity Validation ---
            float3 force = forces[pointIndex];
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

            float3 predictedPosition = position + (velocities[pointIndex] * deltaTime);
            if (!math.any(math.isnan(predictedPosition)) && math.length(predictedPosition) < 100000f)
            {
                predictedPositions[pointIndex] = predictedPosition;
            }
            else
            {
                velocities[pointIndex] = float3.zero;
            }
        }
    }

    [BurstCompile]
    struct RigidConstraintJob : IJob    // No ParallelFor
    {
        [ReadOnly] public NativeArray<bool> isFixed;
        [ReadOnly] public NativeArray<int2> connections;
        [ReadOnly] public NativeArray<float> restLengths;

        [ReadOnly] public float relaxation;
        public NativeArray<float3> predictedPositions;

        public void Execute()
        {
            for (int i = 0; i < connections.Length; i++)
            {
                int2 points = connections[i];
                float3 position1 = predictedPositions[points.x];
                float3 position2 = predictedPositions[points.y];

                float3 direction = position2 - position1;
                float distance = math.length(direction);

                if (distance < 1e-6f || float.IsNaN(distance)) return;

                direction = direction / distance;
                float stretch = distance - restLengths[i];
                float3 correction = direction * (stretch * 0.5f) * relaxation;

                // Add corrections with conditions
                if (!isFixed[points.x]) predictedPositions[points.x] += correction;
                if (!isFixed[points.y]) predictedPositions[points.y] -= correction;
            }
        }
    }

    [BurstCompile]
    struct UpdatePointJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> predictedPositions;
        public NativeArray<float3> originalPositions;
        public NativeArray<float3> velocities;

        [ReadOnly] public NativeArray<bool> isFixed;
        [ReadOnly] public float deltaTime;

        public void Execute(int pointIndex)
        {
            if (isFixed[pointIndex]) return; // Skip fixed points

            // Update point velocity
            float3 velocity = (predictedPositions[pointIndex] - originalPositions[pointIndex]) / deltaTime;
            velocity *= 0.98f; // 0.98f ~ 2% damping

            // More conservative velocity clamping (50 units/s squared)
            if (math.lengthsq(velocity) > 2500f)
            {
                velocity = math.normalize(velocity) * 50f;
            }

            velocities[pointIndex] = velocity;

            // Update point position
            originalPositions[pointIndex] = predictedPositions[pointIndex];
        }
    }

    public void ScheduleGravityJobs(float3 gravity, bool applyGravity)
    {
        // Clear the forces
        for (int i = 0; i < forces.Length; i++)
        {
            forces[i] = float3.zero;
        }

        var gravityJob = new GravityJob
        {
            forces = forces,
            masses = masses,
            gravity = gravity,
            applyGravity = applyGravity
        };

        gravityJobHandle = gravityJob.Schedule(masses.Length, 64);
    }

    public void ScheduleRigidJobs(int checkIterations, float relaxation, float deltaTime)
    {
        for (int i = 0; i < parentSystem.allSpringPoints.Count; i++)
        {
            // Initialize positions and velocities
            positions[i] = parentSystem.allSpringPoints[i].position;
            velocities[i] = parentSystem.allSpringPoints[i].velocity;
            predictedPositions[i] = positions[i]; // Critical initialization
        }

        var integrateJob = new IntegrateForcesJob
        {
            forces = forces,
            masses = masses,
            isFixed = isFixed,
            deltaTime = deltaTime,
            velocities = velocities,
            predictedPositions = predictedPositions,
        };

        rigidJobHandle = integrateJob.Schedule(parentSystem.allSpringPoints.Count, 64, gravityJobHandle);

        // Predict positions and end Jobs
        JobHandle.CombineDependencies(gravityJobHandle, rigidJobHandle).Complete();

        // Correct predicted positions
        var constraintJob = new RigidConstraintJob
        {
            isFixed = isFixed,
            connections = connections,
            restLengths = restLengths,

            relaxation = relaxation,
            predictedPositions = predictedPositions,
        };

        JobHandle iterationHandle = new JobHandle();
        for (int i = 0; i < checkIterations; i++)   // Try 3-10 iterations
        {
            iterationHandle = constraintJob.Schedule(iterationHandle);
        }
        iterationHandle.Complete();

        var updatePointJob = new UpdatePointJob
        {
            predictedPositions = predictedPositions,
            originalPositions = positions,
            velocities = velocities,
            isFixed = isFixed,
            deltaTime = deltaTime
        };

        pointJobHandle = updatePointJob.Schedule(parentSystem.allSpringPoints.Count, 64);
    }

    public void CompleteAllJobsAndApply()
    {
        // Complete Last job
        pointJobHandle.Complete();

        // Apply forces to SpringPointTest objects
        for (int i = 0; i < parentSystem.allSpringPoints.Count; i++)
        {
            // Convert float3 to Vector3
            var forceX = forces[i].x;
            var forceY = forces[i].y;
            var forceZ = forces[i].z;
            Vector3 forceVector = new Vector3(forceX, forceY, forceZ);
            parentSystem.allSpringPoints[i].force = forceVector;

            if (!parentSystem.allSpringPoints[i].isFixed)
            {
                if (!math.any(math.isnan(velocities[i])))
                {
                    var velocityX = velocities[i].x;
                    var velocityY = velocities[i].y;
                    var velocityZ = velocities[i].z;
                    Vector3 velocityVector = new Vector3(velocityX, velocityY, velocityZ);
                    parentSystem.allSpringPoints[i].velocity = velocityVector;
                }
                if (!math.any(math.isnan(positions[i])))
                {
                    var positionX = positions[i].x;
                    var positionY = positions[i].y;
                    var positionZ = positions[i].z;
                    Vector3 positionVector = new Vector3(positionX, positionY, positionZ);
                    parentSystem.allSpringPoints[i].position = positionVector;
                }
            }
        }
    }

    public void UpdateConnectionData(List<SpringConnection> connections)
    {
        // Update connection indices and rest lengths
        for (int i = 0; i < connections.Count; i++)
        {
            int index1 = parentSystem.allSpringPoints.IndexOf(connections[i].point1);
            int index2 = parentSystem.allSpringPoints.IndexOf(connections[i].point2);
            this.connections[i] = new int2(index1, index2);
            restLengths[i] = connections[i].restLength;
        }
    }

    private void OnDestroy()
    {
        if (forces.IsCreated) forces.Dispose();
        if (velocities.IsCreated) velocities.Dispose();
        if (positions.IsCreated) positions.Dispose();
        if (predictedPositions.IsCreated) predictedPositions.Dispose();

        if (connections.IsCreated) connections.Dispose();
        if (restLengths.IsCreated) restLengths.Dispose();

        if (isFixed.IsCreated) isFixed.Dispose();
        if (masses.IsCreated) masses.Dispose();
    }
}
