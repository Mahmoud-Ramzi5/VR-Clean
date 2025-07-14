using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


public class MeshJobManagerCPU : MonoBehaviour
{
    private NativeArray<SpringPointData> springPoints;

    private NativeList<SpringPointData> surfaceSpringPoints;
    private NativeList<float3> surfacePointsLocalSpace;

    private NativeParallelHashMap<int, int> VertexPointMap;
    private NativeArray<float3> meshVerticesNative;
    private NativeArray<int> meshTrianglesNative;
    private float surfaceDetectionThreshold;

    private NativeArray<float3> positions;
    private JobHandle jobHandle;

    /// <summary>
    /// Initializes the collision system with spring points.
    /// NOTE: This class will NOT take ownership or dispose of the springPoints array.
    /// </summary>
    public void Initialize(Vector3[] meshVertices, int[] meshTriangles, NativeArray<SpringPointData> springPoints,
        NativeList<SpringPointData> surfaceSpringPoints, NativeList<float3> surfacePointsLocalSpace, float surfaceDetectionThreshold)
    {
        this.springPoints = springPoints;
        this.surfaceSpringPoints = surfaceSpringPoints;
        this.surfacePointsLocalSpace = surfacePointsLocalSpace;
        this.surfaceDetectionThreshold = surfaceDetectionThreshold;

        // Initialize surface point containers
        //surfaceSpringPoints = new NativeList<SpringPointData>(Allocator.Persistent);
        //surfacePointsLocalSpace = new NativeList<float3>(Allocator.Persistent);

        VertexPointMap = new NativeParallelHashMap<int, int>(meshVertices.Length * 2, Allocator.Persistent);
        meshVerticesNative = new NativeArray<float3>(meshVertices.Length *200, Allocator.Persistent);
        meshTrianglesNative = new NativeArray<int>(meshTriangles.Length * 200, Allocator.Persistent);

        for (int i = 0; i < meshVertices.Length; i++)
        {
            meshVerticesNative[i] = meshVertices[i];
        }
        for (int i = 0; i < meshTriangles.Length; i++)
        {
            meshTrianglesNative[i] = meshTriangles[i];
        }
    }
    // Burst-compiled job
    [BurstCompile]
    struct GetPositionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<SpringPointData> springPoints;
        [WriteOnly] public NativeArray<float3> positions;

        public void Execute(int i)
        {
            positions[i] = springPoints[i].position;
        }
    }

    [BurstCompile]
    struct IdentifySurfacePointsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<SpringPointData> allSpringPoints;
        [ReadOnly] public NativeArray<float3> meshVertices;

        [ReadOnly] public NativeArray<int> meshTriangles;
        [ReadOnly] public float surfaceDetectionThreshold;
        [ReadOnly] public float angleThreshold; // New: Added angle threshold (e.g., 45 degrees)
        [ReadOnly] public float4x4 worldToLocalMatrix;

        [WriteOnly] public NativeList<SpringPointData>.ParallelWriter surfacePoints;

        public void Execute(int index)
        {
            SpringPointData point = allSpringPoints[index];
            float3 localPoint = math.transform(worldToLocalMatrix, point.position);

            // Find nearest triangle
            int nearestTri = FindNearestTriangle(localPoint);
            if (nearestTri == -1) return;

            // Get triangle normal
            float3 triNormal = GetTriangleNormal(nearestTri);

            // Vector from point to triangle center
            float3 triCenter = GetTriangleCenter(nearestTri);
            float3 pointToTri = math.normalize(triCenter - localPoint);

            // Compare angles (dot product)
            float angle = math.degrees(math.acos(math.dot(triNormal, pointToTri)));

            // If angle is large, it's likely a surface point
            if (angle > angleThreshold)
            {
                point.isMeshVertex = 1;
                surfacePoints.AddNoResize(point);
            }
        }

        private int FindNearestTriangle(float3 point)
        {
            int nearestTriangle = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < meshTriangles.Length / 3; i++)
            {
                float3 v0 = meshVertices[meshTriangles[i * 3]];
                float3 v1 = meshVertices[meshTriangles[i * 3 + 1]];
                float3 v2 = meshVertices[meshTriangles[i * 3 + 2]];
                float3 center = (v0 + v1 + v2) / 3f;

                float distance = math.distance(point, center);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestTriangle = i;
                }
            }

            return nearestTriangle;
        }

        private float3 GetTriangleNormal(int triangleIndex)
        {
            float3 v0 = meshVertices[meshTriangles[triangleIndex * 3]];
            float3 v1 = meshVertices[meshTriangles[triangleIndex * 3 + 1]];
            float3 v2 = meshVertices[meshTriangles[triangleIndex * 3 + 2]];

            float3 edge1 = v1 - v0;
            float3 edge2 = v2 - v0;

            return math.normalize(math.cross(edge1, edge2));
        }

        private float3 GetTriangleCenter(int triangleIndex)
        {
            float3 v0 = meshVertices[meshTriangles[triangleIndex * 3]];
            float3 v1 = meshVertices[meshTriangles[triangleIndex * 3 + 1]];
            float3 v2 = meshVertices[meshTriangles[triangleIndex * 3 + 2]];

            return (v0 + v1 + v2) / 3f;
        }
    }


    //[BurstCompile]
    //struct GenerateLocalSurfacePointsJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeList<SpringPointData> surfacePoints;
    //    [ReadOnly] public float4x4 worldToLocalMatrix;

    //    [WriteOnly] public NativeList<float3>.ParallelWriter localPositions;

    //    public void Execute(int index)
    //    {
    //        float3 worldInitial = surfacePoints[index].initialPosition;
    //        float3 local = math.transform(worldToLocalMatrix, worldInitial);
    //        localPositions.AddNoResize(local); // Requires enough capacity
    //    }
    //}

    //[BurstCompile]
    //public struct TransformToLocalSurfacePointsJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeList<SpringPointData> surfacePoints;
    //    [ReadOnly] public float4x4 worldToLocal;

    //    [WriteOnly] public NativeArray<float3> localPositions;

    //    public void Execute(int index)
    //    {
    //        localPositions[index] = math.transform(worldToLocal, surfacePoints[index].position);
    //    }
    //}

    [BurstCompile]
    struct FindVertexClosestPointJob : IJobParallelFor
    {
        public NativeParallelHashMap<int, int>.ParallelWriter VertexPointMap;

        [ReadOnly] public NativeArray<SpringPointData> allSpringPoints;
        [ReadOnly] public NativeArray<float3> meshVertices;
        [ReadOnly] public float4x4 localToWorldMatrix;

        public void Execute(int index)
        {
            float3 worldVertex = math.transform(localToWorldMatrix, meshVertices[index]);
            int closestPointIndex = FindClosestPointIndex(worldVertex);
            VertexPointMap.TryAdd(index, closestPointIndex);
        }

        private int FindClosestPointIndex(float3 worldPos)
        {
            int closestIndex = 0;
            float minDistanceSqr = float.MaxValue;

            for (int i = 0; i < allSpringPoints.Length; i++)
            {
                float distSqr = math.distancesq(worldPos, allSpringPoints[i].position);
                if (distSqr < minDistanceSqr)
                {
                    minDistanceSqr = distSqr;
                    closestIndex = i;
                }
            }

            return closestIndex;
        }
    }

    [BurstCompile]
    struct UpdateMeshVerticesJob : IJobParallelFor
    {
        [WriteOnly] public NativeList<SpringPointData>.ParallelWriter surfacePoints;
        [WriteOnly] public NativeArray<float3> meshVertices;

        [ReadOnly] public NativeParallelHashMap<int, int> VertexPointMap;
        [ReadOnly] public NativeArray<SpringPointData> springPoints;
        [ReadOnly] public float4x4 localToWorldMatrix;
        [ReadOnly] public float4x4 worldToLocalMatrix;

        public void Execute(int index)
        {
            if (VertexPointMap.TryGetValue(index, out int pointIndex))
            {
                if (pointIndex >= 0 && pointIndex < springPoints.Length)
                {
                    SpringPointData closestPoint = springPoints[pointIndex];
                    surfacePoints.AddNoResize(closestPoint);

                    // Defensive check
                    if (closestPoint.mass > 0 || closestPoint.isFixed == 0)
                    {
                        meshVertices[index] = math.transform(worldToLocalMatrix, closestPoint.position);
                    }
                }
            }
        }
    }

    public void IdentifySurfacePoints(Vector3[] meshVerts, int[] meshTris, Matrix4x4 worldToLocal)
    {
        surfaceSpringPoints.Clear();
        surfacePointsLocalSpace.Clear();

        // Set capacity based on expected maximum size
        surfaceSpringPoints.Capacity = springPoints.Length;
        surfacePointsLocalSpace.Capacity = springPoints.Length;

        for (int i = 0; i < meshVerts.Length; i++) meshVerticesNative[i] = (float3)meshVerts[i];
        for (int i = 0; i < meshTris.Length; i++) meshTrianglesNative[i] = meshTris[i];

        new IdentifySurfacePointsJob
        {
            allSpringPoints = springPoints,
            meshVertices = meshVerticesNative,
            meshTriangles = meshTrianglesNative,
            surfaceDetectionThreshold = surfaceDetectionThreshold,
            angleThreshold = 40f, // Configurable angle threshold (e.g., 45 degrees)
            worldToLocalMatrix = worldToLocal,
            surfacePoints = surfaceSpringPoints.AsParallelWriter()
        }.Schedule(springPoints.Length, 64).Complete();
    }

    public void ScheduleMeshVerticesUpdateJobs(Vector3[] meshVerts, int[] meshTris, Matrix4x4 localToWorld, Matrix4x4 worldToLocal)
    {
        VertexPointMap.Clear();
        surfaceSpringPoints.Clear();

        // Set capacity based on expected maximum size
        surfaceSpringPoints.Capacity = math.max(springPoints.Length, meshVerticesNative.Length);

        // Get only positions from SpringPointData structs
        positions = new(springPoints.Length, Allocator.TempJob);
        new GetPositionsJob
        {
            springPoints = springPoints,
            positions = positions,
        }.Schedule(springPoints.Length, 64).Complete();

        // Calculate average position
        float3 averagePos = float3.zero;
        foreach (var position in positions)
        {
            averagePos += position;
        }
        averagePos /= positions.Length;

        transform.position = averagePos;

        for (int i = 0; i < meshVerts.Length; i++) meshVerticesNative[i] = (float3)meshVerts[i];
        for (int i = 0; i < meshTris.Length; i++) meshTrianglesNative[i] = meshTris[i];

        var findJob = new FindVertexClosestPointJob
        {
            VertexPointMap = VertexPointMap.AsParallelWriter(),

            allSpringPoints = springPoints,
            meshVertices = meshVerticesNative,
            localToWorldMatrix = localToWorld,
        };

        var updateJob = new UpdateMeshVerticesJob
        {
            surfacePoints = surfaceSpringPoints.AsParallelWriter(),
            meshVertices = meshVerticesNative,

            springPoints = springPoints,
            VertexPointMap = VertexPointMap,
            localToWorldMatrix = localToWorld,
            worldToLocalMatrix = worldToLocal,
        };

        jobHandle = findJob.Schedule(meshVerticesNative.Length, 64);
        jobHandle = updateJob.Schedule(meshVerticesNative.Length, 64, jobHandle);
    }

    public void CompleteAllJobsAndApply(Vector3[] meshVertices, int[] meshTriangles, Mesh targetMesh, List<SpringPointData> surfacePoints)
    {
        jobHandle.Complete();

        // update points
        surfacePoints.Clear();
        for (int i = 0; i < surfaceSpringPoints.Length; i++)
            surfacePoints.Add(surfaceSpringPoints[i]);

        // Copy NativeArray to regular array
        Vector3[] newVertices = new Vector3[meshVerticesNative.Length];
        for (int i = 0; i < meshVerticesNative.Length; i++)
        {
            newVertices[i] = (Vector3)meshVerticesNative[i];
        }

        // Ensure triangles array is valid for new vertex count
        if (meshTrianglesNative.Length > 0)
        {
            // Validate triangle indices
            int maxIndex = newVertices.Length - 1;
            for (int i = 0; i < meshTrianglesNative.Length; i++)
            {
                if (meshTrianglesNative[i] > maxIndex)
                {
                    meshTrianglesNative[i] = maxIndex;
                }
            }
        }

        int[] newTriangles = new int[meshTrianglesNative.Length];
        for (int i = 0; i < meshTrianglesNative.Length; i++)
        {
            newTriangles[i] = (int)meshTrianglesNative[i];
        }

        targetMesh.vertices = newVertices;
        targetMesh.triangles = newTriangles;

        targetMesh.RecalculateNormals();
        targetMesh.RecalculateBounds();

        // Update cached references
        meshVertices = newVertices;
        meshTriangles = newTriangles;

        if (positions.IsCreated)
            positions.Dispose();
    }

    private void OnDestroy()
    {
        // Complete any pending jobs
        jobHandle.Complete();

        //if (springPoints.IsCreated) springPoints.Dispose
        //if (surfaceSpringPoints.IsCreated) surfaceSpringPoints.Clear();
        //if (surfacePointsLocalSpace.IsCreated) surfacePointsLocalSpace.Clear();

        if (VertexPointMap.IsCreated) VertexPointMap.Dispose();
        if (meshVerticesNative.IsCreated) meshVerticesNative.Dispose();
        if (meshTrianglesNative.IsCreated) meshTrianglesNative.Dispose();
    }
}