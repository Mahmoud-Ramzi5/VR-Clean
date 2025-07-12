using System;
using System.Collections.Generic;
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

    // Track original sizes for validation
    private int originalVertexCount;
    private int originalTriangleCount;

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

        // Store original counts
        originalVertexCount = meshVertices.Length;
        originalTriangleCount = meshTriangles.Length;

        Debug.Log($"MeshJobManagerCPU Initialize: {originalVertexCount} vertices, {originalTriangleCount} triangle indices, {springPoints.Length} spring points");

        // Initialize with extra capacity for subdivision
        int maxVertices = meshVertices.Length * 4; // Allow for subdivision growth
        int maxTriangles = meshTriangles.Length * 4;

        VertexPointMap = new NativeParallelHashMap<int, int>(maxVertices, Allocator.Persistent);
        meshVerticesNative = new NativeArray<float3>(maxVertices, Allocator.Persistent);
        meshTrianglesNative = new NativeArray<int>(maxTriangles, Allocator.Persistent);

        // Copy initial data
        for (int i = 0; i < meshVertices.Length; i++)
        {
            meshVerticesNative[i] = meshVertices[i];
        }
        for (int i = 0; i < meshTriangles.Length; i++)
        {
            meshTrianglesNative[i] = meshTriangles[i];
        }

        // Clear unused portions
        for (int i = meshVertices.Length; i < maxVertices; i++)
        {
            meshVerticesNative[i] = float3.zero;
        }
        for (int i = meshTriangles.Length; i < maxTriangles; i++)
        {
            meshTrianglesNative[i] = 0;
        }
    }

    public void UpdateMeshData(Vector3[] newVertices, int[] newTriangles)
    {
        if (newVertices == null || newTriangles == null)
        {
            Debug.LogError("UpdateMeshData called with null arrays");
            return;
        }

        // Validate triangle array
        if (newTriangles.Length % 3 != 0)
        {
            Debug.LogError($"Invalid triangle array length: {newTriangles.Length} (should be multiple of 3)");
            return;
        }

        // Check if we need to resize arrays
        if (newVertices.Length > meshVerticesNative.Length)
        {
            Debug.Log($"Resizing vertex array from {meshVerticesNative.Length} to {newVertices.Length * 2}");

            // Dispose old arrays
            if (meshVerticesNative.IsCreated) meshVerticesNative.Dispose();
            if (VertexPointMap.IsCreated) VertexPointMap.Dispose();

            // Create larger arrays
            meshVerticesNative = new NativeArray<float3>(newVertices.Length * 2, Allocator.Persistent);
            VertexPointMap = new NativeParallelHashMap<int, int>(newVertices.Length * 2, Allocator.Persistent);
        }

        if (newTriangles.Length > meshTrianglesNative.Length)
        {
            Debug.Log($"Resizing triangle array from {meshTrianglesNative.Length} to {newTriangles.Length * 2}");

            if (meshTrianglesNative.IsCreated) meshTrianglesNative.Dispose();
            meshTrianglesNative = new NativeArray<int>(newTriangles.Length * 2, Allocator.Persistent);
        }

        // Validate triangle indices
        for (int i = 0; i < newTriangles.Length; i++)
        {
            if (newTriangles[i] < 0 || newTriangles[i] >= newVertices.Length)
            {
                Debug.LogError($"Invalid triangle index at position {i}: {newTriangles[i]} (vertex count: {newVertices.Length})");
                return;
            }
        }

        // Update data
        for (int i = 0; i < newVertices.Length; i++)
        {
            meshVerticesNative[i] = newVertices[i];
        }
        for (int i = 0; i < newTriangles.Length; i++)
        {
            meshTrianglesNative[i] = newTriangles[i];
        }

        // Update tracking variables
        originalVertexCount = newVertices.Length;
        originalTriangleCount = newTriangles.Length;

        Debug.Log($"MeshJobManagerCPU updated: {originalVertexCount} vertices, {originalTriangleCount} triangle indices");
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
        [ReadOnly] public NativeSlice<float3> meshVertices; // Use slice for safety
        [ReadOnly] public NativeSlice<int> meshTriangles;   // Use slice for safety
        [ReadOnly] public int triangleCount;
        [ReadOnly] public float surfaceDetectionThreshold;
        [ReadOnly] public float angleThreshold;
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
            float angle = math.degrees(math.acos(math.clamp(math.dot(triNormal, pointToTri), -1f, 1f)));

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

            for (int i = 0; i < triangleCount; i++)
            {
                int baseIdx = i * 3;
                if (baseIdx + 2 >= meshTriangles.Length) continue;

                int v0Idx = meshTriangles[baseIdx];
                int v1Idx = meshTriangles[baseIdx + 1];
                int v2Idx = meshTriangles[baseIdx + 2];

                if (v0Idx >= meshVertices.Length || v1Idx >= meshVertices.Length || v2Idx >= meshVertices.Length)
                    continue;

                float3 v0 = meshVertices[v0Idx];
                float3 v1 = meshVertices[v1Idx];
                float3 v2 = meshVertices[v2Idx];
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
            int baseIdx = triangleIndex * 3;
            if (baseIdx + 2 >= meshTriangles.Length) return float3.zero;

            int v0Idx = meshTriangles[baseIdx];
            int v1Idx = meshTriangles[baseIdx + 1];
            int v2Idx = meshTriangles[baseIdx + 2];

            if (v0Idx >= meshVertices.Length || v1Idx >= meshVertices.Length || v2Idx >= meshVertices.Length)
                return float3.zero;

            float3 v0 = meshVertices[v0Idx];
            float3 v1 = meshVertices[v1Idx];
            float3 v2 = meshVertices[v2Idx];

            float3 edge1 = v1 - v0;
            float3 edge2 = v2 - v0;

            return math.normalize(math.cross(edge1, edge2));
        }

        private float3 GetTriangleCenter(int triangleIndex)
        {
            int baseIdx = triangleIndex * 3;
            if (baseIdx + 2 >= meshTriangles.Length) return float3.zero;

            int v0Idx = meshTriangles[baseIdx];
            int v1Idx = meshTriangles[baseIdx + 1];
            int v2Idx = meshTriangles[baseIdx + 2];

            if (v0Idx >= meshVertices.Length || v1Idx >= meshVertices.Length || v2Idx >= meshVertices.Length)
                return float3.zero;

            float3 v0 = meshVertices[v0Idx];
            float3 v1 = meshVertices[v1Idx];
            float3 v2 = meshVertices[v2Idx];

            return (v0 + v1 + v2) / 3f;
        }
    }

    [BurstCompile]
    struct FindVertexClosestPointJob : IJobParallelFor
    {
        public NativeParallelHashMap<int, int>.ParallelWriter VertexPointMap;

        [ReadOnly] public NativeArray<SpringPointData> allSpringPoints;
        [ReadOnly] public NativeSlice<float3> meshVertices;
        [ReadOnly] public int vertexCount;
        [ReadOnly] public float4x4 localToWorldMatrix;

        public void Execute(int index)
        {
            if (index >= vertexCount || index >= meshVertices.Length) return;

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
        public NativeSlice<float3> meshVertices;

        [ReadOnly] public NativeParallelHashMap<int, int> VertexPointMap;
        [ReadOnly] public NativeArray<SpringPointData> springPoints;
        [ReadOnly] public int vertexCount;
        [ReadOnly] public float4x4 localToWorldMatrix;
        [ReadOnly] public float4x4 worldToLocalMatrix;

        public void Execute(int index)
        {
            if (index >= vertexCount || index >= meshVertices.Length) return;

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

    // NEW: Improved job for updating mesh from spring points
    [BurstCompile]
    struct UpdateMeshVerticesFromSpringPointsJob : IJobParallelFor
    {
        // Output: Updated mesh vertices
        public NativeSlice<float3> meshVertices;

        // Input: Spring point data
        [ReadOnly] public NativeArray<SpringPointData> springPoints;
        [ReadOnly] public int vertexCount;
        [ReadOnly] public float4x4 worldToLocalMatrix;
        [ReadOnly] public float influenceRadius;

        public void Execute(int vertexIndex)
        {
            if (vertexIndex >= vertexCount || vertexIndex >= meshVertices.Length) return;

            // Get current vertex position in world space
            float3 currentLocalPos = meshVertices[vertexIndex];
            float3 currentWorldPos = math.transform(math.inverse(worldToLocalMatrix), currentLocalPos);

            // Find closest spring point
            int closestSpringIndex = -1;
            float minDistanceSqr = float.MaxValue;

            for (int i = 0; i < springPoints.Length; i++)
            {
                float distSqr = math.distancesq(currentWorldPos, springPoints[i].position);
                if (distSqr < minDistanceSqr)
                {
                    minDistanceSqr = distSqr;
                    closestSpringIndex = i;
                }
            }

            // Update vertex position if close spring point found
            if (closestSpringIndex >= 0 && math.sqrt(minDistanceSqr) < influenceRadius)
            {
                SpringPointData closestSpring = springPoints[closestSpringIndex];
                float3 newLocalPos = math.transform(worldToLocalMatrix, closestSpring.position);

                // Only update if the change is significant
                if (math.distance(currentLocalPos, newLocalPos) > 0.001f)
                {
                    meshVertices[vertexIndex] = newLocalPos;
                }
            }
        }
    }

    public void IdentifySurfacePoints(Vector3[] meshVerts, int[] meshTris, Matrix4x4 worldToLocal)
    {
        if (meshVerts == null || meshTris == null)
        {
            Debug.LogError("IdentifySurfacePoints called with null arrays");
            return;
        }

        // Update mesh data first
        UpdateMeshData(meshVerts, meshTris);

        surfaceSpringPoints.Clear();
        surfacePointsLocalSpace.Clear();

        // Set capacity based on expected maximum size
        surfaceSpringPoints.Capacity = springPoints.Length;
        surfacePointsLocalSpace.Capacity = springPoints.Length;

        int triangleCount = meshTris.Length / 3;
        Debug.Log($"IdentifySurfacePoints: Processing {triangleCount} triangles with {springPoints.Length} spring points");

        var meshVerticesSlice = new NativeSlice<float3>(meshVerticesNative, 0, meshVerts.Length);
        var meshTrianglesSlice = new NativeSlice<int>(meshTrianglesNative, 0, meshTris.Length);

        new IdentifySurfacePointsJob
        {
            allSpringPoints = springPoints,
            meshVertices = meshVerticesSlice,
            meshTriangles = meshTrianglesSlice,
            triangleCount = triangleCount,
            surfaceDetectionThreshold = surfaceDetectionThreshold,
            angleThreshold = 45f,
            worldToLocalMatrix = worldToLocal,
            surfacePoints = surfaceSpringPoints.AsParallelWriter()
        }.Schedule(springPoints.Length, 64).Complete();

        Debug.Log($"IdentifySurfacePoints: Found {surfaceSpringPoints.Length} surface points");
    }

    public void ScheduleMeshVerticesUpdateJobs(Vector3[] meshVerts, int[] meshTris, Matrix4x4 localToWorld, Matrix4x4 worldToLocal)
    {
        if (meshVerts == null || meshTris == null)
        {
            Debug.LogError("ScheduleMeshVerticesUpdateJobs called with null arrays");
            return;
        }

        // Update mesh data first
        UpdateMeshData(meshVerts, meshTris);

        VertexPointMap.Clear();
        surfaceSpringPoints.Clear();

        // Set capacity based on expected maximum size
        surfaceSpringPoints.Capacity = math.max(springPoints.Length, meshVerts.Length);

        // Get only positions from SpringPointData structs
        if (positions.IsCreated) positions.Dispose();
        positions = new NativeArray<float3>(springPoints.Length, Allocator.TempJob);

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
        if (positions.Length > 0)
        {
            averagePos /= positions.Length;
            transform.position = averagePos;
        }

        var meshVerticesSlice = new NativeSlice<float3>(meshVerticesNative, 0, meshVerts.Length);

        var findJob = new FindVertexClosestPointJob
        {
            VertexPointMap = VertexPointMap.AsParallelWriter(),
            allSpringPoints = springPoints,
            meshVertices = meshVerticesSlice,
            vertexCount = meshVerts.Length,
            localToWorldMatrix = localToWorld,
        };

        var updateJob = new UpdateMeshVerticesJob
        {
            surfacePoints = surfaceSpringPoints.AsParallelWriter(),
            meshVertices = meshVerticesSlice,
            springPoints = springPoints,
            VertexPointMap = VertexPointMap,
            vertexCount = meshVerts.Length,
            localToWorldMatrix = localToWorld,
            worldToLocalMatrix = worldToLocal,
        };

        jobHandle = findJob.Schedule(meshVerts.Length, 64);
        jobHandle = updateJob.Schedule(meshVerts.Length, 64, jobHandle);
    }

    // NEW: Improved method for updating mesh from spring points
    public void UpdateMeshFromSpringPoints(Vector3[] meshVerts, Matrix4x4 worldToLocal, float influenceRadius)
    {
        if (meshVerts == null || !springPoints.IsCreated)
        {
            Debug.LogError("UpdateMeshFromSpringPoints called with invalid data");
            return;
        }

        // Ensure our native array is up to date
        for (int i = 0; i < math.min(meshVerts.Length, meshVerticesNative.Length); i++)
        {
            meshVerticesNative[i] = meshVerts[i];
        }

        var meshVerticesSlice = new NativeSlice<float3>(meshVerticesNative, 0, meshVerts.Length);

        var updateJob = new UpdateMeshVerticesFromSpringPointsJob
        {
            meshVertices = meshVerticesSlice,
            springPoints = springPoints,
            vertexCount = meshVerts.Length,
            worldToLocalMatrix = worldToLocal,
            influenceRadius = influenceRadius
        };

        jobHandle = updateJob.Schedule(meshVerts.Length, 32);
    }

    public void CompleteAllJobsAndApply(Vector3[] meshVertices, int[] meshTriangles, Mesh targetMesh, List<SpringPointData> surfacePoints)
    {
        if (meshVertices == null || meshTriangles == null || targetMesh == null || surfacePoints == null)
        {
            Debug.LogError("CompleteAllJobsAndApply called with null parameters");
            return;
        }

        jobHandle.Complete();

        // Update surface points list
        surfacePoints.Clear();
        for (int i = 0; i < surfaceSpringPoints.Length; i++)
            surfacePoints.Add(surfaceSpringPoints[i]);

        // Validate and copy updated mesh vertices
        int vertexCount = math.min(originalVertexCount, meshVerticesNative.Length);

        Vector3[] newVertices = new Vector3[vertexCount];
        bool hasChanges = false;

        for (int i = 0; i < vertexCount; i++)
        {
            Vector3 newPos = (Vector3)meshVerticesNative[i];
            if (i < meshVertices.Length && Vector3.Distance(newPos, meshVertices[i]) > 0.001f)
            {
                hasChanges = true;
            }
            newVertices[i] = newPos;
        }

        // Only update mesh if there were actual changes
        if (hasChanges)
        {
            try
            {
                targetMesh.vertices = newVertices;
                targetMesh.RecalculateNormals();
                targetMesh.RecalculateBounds();

                Debug.Log($"Applied mesh update with changes: {newVertices.Length} vertices");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error applying mesh update: {e.Message}");
            }
        }

        // Update the input array reference
        System.Array.Copy(newVertices, meshVertices, math.min(newVertices.Length, meshVertices.Length));

        // Clean up temp resources
        if (positions.IsCreated)
            positions.Dispose();
    }

    private void OnDestroy()
    {
        // Complete any pending jobs
        if (jobHandle.IsCompleted == false)
            jobHandle.Complete();

        if (VertexPointMap.IsCreated) VertexPointMap.Dispose();
        if (meshVerticesNative.IsCreated) meshVerticesNative.Dispose();
        if (meshTrianglesNative.IsCreated) meshTrianglesNative.Dispose();
        if (positions.IsCreated) positions.Dispose();
    }
}