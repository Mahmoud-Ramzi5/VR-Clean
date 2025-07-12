using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;


public class MeshJobManagerGPU : MonoBehaviour
{
    [System.Serializable]

    public struct WeightedPoint
    {
        public int index;
        public float weight;
    }

    [System.Serializable]
    public struct VertexWeightBinding
    {
        public WeightedPoint wp0;
        public WeightedPoint wp1;
        public WeightedPoint wp2;
    }

    private NativeArray<SpringPointData> springPoints;
    public VertexWeightBinding[] vertexWeightBindings;

    private ComputeShader meshUpdater;
    private int kernelId;

    private ComputeBuffer[] vertexBuffers = new ComputeBuffer[2];   // RWStructuredBuffer<float3>
    private int currentBufferIndex = 0;
    private int frameCount = 0;

    private ComputeBuffer PointPosBuffer;   // StructuredBuffer<float3>
    private ComputeBuffer weightMapBuffer;  // StructuredBuffer<VertexWeightBinding>

    private NativeList<SpringPointData> surfaceSpringPoints;
    private NativeList<float3> surfacePointsLocalSpace;
    private NativeArray<float3> meshVerticesNative;
    private NativeArray<int> meshTrianglesNative;
    private float surfaceDetectionThreshold;
    private int originalVertexCount;
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

        meshUpdater = Resources.Load<ComputeShader>("MeshUpdater");
        kernelId = meshUpdater.FindKernel("CSMain");
        PrecomputeVertexMapping(meshVertices);

        vertexBuffers[0] = new ComputeBuffer(meshVertices.Length, sizeof(float) * 3);
        vertexBuffers[1] = new ComputeBuffer(meshVertices.Length, sizeof(float) * 3);

        PointPosBuffer = new ComputeBuffer(springPoints.Length, sizeof(float) * 3);
        weightMapBuffer = new ComputeBuffer(vertexWeightBindings.Length, 3 * (sizeof(int) + sizeof(float)));


        // Initialize surface point containers
        //surfaceSpringPoints = new NativeList<SpringPointData>(Allocator.Persistent);
        //surfacePointsLocalSpace = new NativeList<float3>(Allocator.Persistent);
        meshVerticesNative = new NativeArray<float3>(meshVertices.Length * 2, Allocator.Persistent);
        meshTrianglesNative = new NativeArray<int>(meshTriangles.Length * 2, Allocator.Persistent);

        originalVertexCount = meshVertices.Length;
        for (int i = 0; i < originalVertexCount; i++)
        {
            meshVerticesNative[i] = meshVertices[i];
        }
    }

    public void PrecomputeVertexMapping(Vector3[] vertices)
    {
        int vertexCount = vertices.Length;
        vertexWeightBindings = new VertexWeightBinding[vertexCount];

        for (int i = 0; i < vertexCount; i++)
        {
            // Transform vertex to world space
            Vector3 worldPos = transform.TransformPoint(vertices[i]);

            // Find closest spring points and their squared distances
            List<(int index, float distSq)> closest = new List<(int, float)>();
            for (int j = 0; j < springPoints.Length; j++)
            {
                float distSq = math.lengthsq((float3)worldPos - springPoints[j].position);
                closest.Add((j, distSq));
            }

            // Sort by distance ascending
            closest.Sort((a, b) => a.distSq.CompareTo(b.distSq));

            // We want exactly 3 entries per vertex. If less, fill with dummy entries.
            // Compute total inverse distance weight for normalization
            float totalWeight = 0f;
            int limit = math.min(3, closest.Count);

            float[] invDistances = new float[3];
            int[] indices = new int[3];

            for (int k = 0; k < limit; k++)
            {
                invDistances[k] = 1f / (closest[k].distSq + 1e-4f);
                indices[k] = closest[k].index;
                totalWeight += invDistances[k];
            }

            // Fill missing entries with index -1 and zero weight
            for (int k = limit; k < 3; k++)
            {
                invDistances[k] = 0f;
                indices[k] = -1; // invalid index
            }

            // Normalize weights and assign to the struct
            vertexWeightBindings[i] = new VertexWeightBinding
            {
                wp0 = new WeightedPoint
                {
                    index = indices[0],
                    weight = (totalWeight > 0f) ? invDistances[0] / totalWeight : 0f
                },
                wp1 = new WeightedPoint
                {
                    index = indices[1],
                    weight = (totalWeight > 0f) ? invDistances[1] / totalWeight : 0f
                },
                wp2 = new WeightedPoint
                {
                    index = indices[2],
                    weight = (totalWeight > 0f) ? invDistances[2] / totalWeight : 0f
                }
            };
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
            angleThreshold = 45f, // Configurable angle threshold (e.g., 45 degrees)
            worldToLocalMatrix = worldToLocal,
            surfacePoints = surfaceSpringPoints.AsParallelWriter()
        }.Schedule(springPoints.Length, 64).Complete();
    }

    public void DispatchMeshUpdate(Vector3[] meshVerts, Matrix4x4 worldToLocal, Mesh targetMesh)
    {
        NativeArray<float3> positions = new(springPoints.Length, Allocator.TempJob);

        new GetPositionsJob
        {
            springPoints = springPoints,
            positions = positions,
        }.Schedule(springPoints.Length, 64).Complete();

        ComputeBuffer currentVertexBuffer = vertexBuffers[currentBufferIndex];

        currentVertexBuffer.SetData(meshVerts);
        PointPosBuffer.SetData(positions);
        weightMapBuffer.SetData(vertexWeightBindings);

        meshUpdater.SetBuffer(kernelId, "Vertices", currentVertexBuffer);
        meshUpdater.SetBuffer(kernelId, "PointsPositions", PointPosBuffer);
        meshUpdater.SetBuffer(kernelId, "VertexBindings", weightMapBuffer);

        meshUpdater.SetMatrix("worldToLocal", worldToLocal);
        meshUpdater.Dispatch(kernelId, Mathf.CeilToInt(meshVerts.Length / 64f), 1, 1);

        // Request readback for PREVIOUS frame's buffer 
        int readbackIndex = (currentBufferIndex + 1) % 2;

        // if available
        if (frameCount > 0)
        {
            AsyncGPUReadback.Request(vertexBuffers[readbackIndex], (request) =>
            {
                if (!request.hasError)
                {
                    if (targetMesh != null)  // check if mesh still exists
                    {
                        var data = request.GetData<float3>();

                        // Apply updated vertices
                        meshVerticesNative = data;
                        targetMesh.SetVertices(data);
                        targetMesh.RecalculateBounds();
                        targetMesh.RecalculateNormals();
                    }
                }
            });
        }

        // Switch to next buffer for next frame
        currentBufferIndex = readbackIndex;
        frameCount++;

        //// Read back
        //vertexBuffer.GetData(meshVerts);

        //// Apply to mesh
        //targetMesh.vertices = meshVerts;
        //targetMesh.RecalculateBounds();
        //targetMesh.RecalculateNormals();

        if (positions.IsCreated)
            positions.Dispose();
    }

    private void OnDestroy()
    {
        // Complete any pending jobs
        jobHandle.Complete();

        if (vertexBuffers[0] != null) vertexBuffers[0].Release();
        if (vertexBuffers[1] != null) vertexBuffers[1].Release();

        if (PointPosBuffer != null) PointPosBuffer.Release();
        if (weightMapBuffer != null) weightMapBuffer.Release();

        //if (springPoints.IsCreated) springPoints.Dispose();
        //if (surfaceSpringPoints.IsCreated) surfaceSpringPoints.Clear();
        //if (surfacePointsLocalSpace.IsCreated) surfacePointsLocalSpace.Clear();

        if (meshVerticesNative.IsCreated) meshVerticesNative.Dispose();
        if (meshTrianglesNative.IsCreated) meshTrianglesNative.Dispose();
    }
}