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
    private VertexWeightBinding[] vertexWeightBindings;

    private ComputeShader meshUpdater;
    private int kernelId;

    private ComputeBuffer[] vertexBuffers = new ComputeBuffer[2];   // RWStructuredBuffer<float3>
    private int currentBufferIndex = 0;
    private int frameCount = 0;

    private ComputeBuffer PointPosBuffer;   // StructuredBuffer<float3>
    private ComputeBuffer weightMapBuffer;  // StructuredBuffer<VertexWeightBinding>


    private NativeList<SpringPointData> surfaceSpringPoints;
    private NativeList<float3> surfacePointsLocalSpace;
    private NativeArray<float3> meshVerticesNative; // Cache the mesh vertices
    private float surfaceDetectionThreshold = 0.01f; // Adjust as needed
    private int originalVertexCount;
    private JobHandle jobHandle;

    public void Initialize(Vector3[] meshVertices, NativeArray<SpringPointData> springPoints, NativeList<SpringPointData> surfaceSpringPoints, NativeList<float3> surfacePointsLocalSpace)
    {
        this.springPoints = springPoints;
        this.surfaceSpringPoints = surfaceSpringPoints;
        this.surfacePointsLocalSpace = surfacePointsLocalSpace;

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
        meshVerticesNative = new NativeArray<float3>(meshVertices.Length, Allocator.Persistent);

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
                surfacePoints.AddNoResize(point);
            }
        }

        private int FindNearestTriangle(float3 point)
        {
            int nearestTri = -1;
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
                    nearestTri = i;
                }
            }

            return nearestTri;
        }

        private float3 GetTriangleNormal(int triIndex)
        {
            float3 v0 = meshVertices[meshTriangles[triIndex * 3]];
            float3 v1 = meshVertices[meshTriangles[triIndex * 3 + 1]];
            float3 v2 = meshVertices[meshTriangles[triIndex * 3 + 2]];

            float3 edge1 = v1 - v0;
            float3 edge2 = v2 - v0;
            return math.normalize(math.cross(edge1, edge2));
        }

        private float3 GetTriangleCenter(int triIndex)
        {
            float3 v0 = meshVertices[meshTriangles[triIndex * 3]];
            float3 v1 = meshVertices[meshTriangles[triIndex * 3 + 1]];
            float3 v2 = meshVertices[meshTriangles[triIndex * 3 + 2]];
            return (v0 + v1 + v2) / 3f;
        }
    }


    [BurstCompile]
    struct GenerateLocalSurfacePointsJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<SpringPointData> surfacePoints;
        [ReadOnly] public float4x4 worldToLocalMatrix;

        [WriteOnly] public NativeList<float3>.ParallelWriter localPositions;

        public void Execute(int index)
        {
            float3 worldInitial = surfacePoints[index].initialPosition;
            float3 local = math.transform(worldToLocalMatrix, worldInitial);
            localPositions.AddNoResize(local); // Requires enough capacity
        }
    }

    [BurstCompile]
    public struct TransformToLocalSurfacePointsJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<SpringPointData> surfacePoints;
        [ReadOnly] public float4x4 worldToLocal;

        [WriteOnly] public NativeArray<float3> localPositions;

        public void Execute(int index)
        {
            localPositions[index] = math.transform(worldToLocal, surfacePoints[index].position);
        }
    }

    [BurstCompile]
    public struct GenerateSurfaceTrianglesJob : IJob
    {
        [ReadOnly] public NativeList<float3> surfaceLocalPoints;
        [ReadOnly] public NativeArray<float3> inVertices;
        [ReadOnly] public NativeArray<int> inTriangles;
        [ReadOnly] public int originalVertexCount;

        [WriteOnly] public NativeArray<int3> outTriangles; // 3 triangles per point -> 3x count

        public void Execute()
        {
            for (int index = 0; index < surfaceLocalPoints.Length; index++)
            {
                float3 surfacePoint = surfaceLocalPoints[index];

                // Find nearest triangle
                float closestDistance = float.MaxValue;
                int closestTriangleIndex = -1;

                // Find Closest Triangle To the Point
                for (int i = 0; i < inTriangles.Length; i += 3)
                {
                    float3 v1 = inVertices[inTriangles[i]];
                    float3 v2 = inVertices[inTriangles[i + 1]];
                    float3 v3 = inVertices[inTriangles[i + 2]];

                    float3 centerTriangle = (v1 + v2 + v3) / 3f;
                    float distance = math.length(surfacePoint - centerTriangle);
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestTriangleIndex = i;
                    }
                }

                int v0 = originalVertexCount + index;

                if (closestTriangleIndex >= 0)
                {
                    int t0 = inTriangles[closestTriangleIndex];
                    int t1 = inTriangles[closestTriangleIndex + 1];
                    int t2 = inTriangles[closestTriangleIndex + 2];

                    outTriangles[index * 3 + 0] = new int3(v0, t0, t1);
                    outTriangles[index * 3 + 1] = new int3(v0, t1, t2);
                    outTriangles[index * 3 + 2] = new int3(v0, t2, t0);
                }
                else
                {
                    // Fallback: Find 3 nearest base vertices
                    float min1 = float.MaxValue, min2 = float.MaxValue, min3 = float.MaxValue;
                    float3 d1 = float3.zero, d2 = float3.zero, d3 = float3.zero;
                    int i1 = -1, i2 = -1, i3 = -1;

                    for (int i = 0; i < originalVertexCount; i++)
                    {
                        float dist = math.length(surfacePoint - inVertices[i]);
                        if (dist < min1)
                        {
                            min3 = min2; i3 = i2;
                            min2 = min1; i2 = i1;
                            min1 = dist; i1 = i;
                        }
                    }

                    outTriangles[index * 3 + 0] = new int3(v0, i1, i2);
                    outTriangles[index * 3 + 1] = new int3(v0, i2, i3);
                    outTriangles[index * 3 + 2] = new int3(v0, i3, i1);
                }
            }
        }
    }

    public void ScheduleSurfacePointsJobs(Vector3[] meshVerts, int[] meshTris, Matrix4x4 worldToLocal)
    {
        surfaceSpringPoints.Clear();
        surfacePointsLocalSpace.Clear();

        // Convert mesh data to NativeArrays for the job
        NativeArray<float3> nativeVerts = new NativeArray<float3>(meshVerts.Length, Allocator.TempJob);
        NativeArray<int> nativeTris = new NativeArray<int>(meshTris.Length, Allocator.TempJob);

        for (int i = 0; i < meshVerts.Length; i++) nativeVerts[i] = meshVerts[i];
        for (int i = 0; i < meshTris.Length; i++) nativeTris[i] = meshTris[i];

        var identifyJob = new IdentifySurfacePointsJob
        {
            allSpringPoints = springPoints,
            meshVertices = nativeVerts,
            meshTriangles = nativeTris,
            surfaceDetectionThreshold = surfaceDetectionThreshold,
            angleThreshold = 45f, // Configurable angle threshold (e.g., 45 degrees)
            worldToLocalMatrix = worldToLocal,
            surfacePoints = surfaceSpringPoints.AsParallelWriter()
        };

        jobHandle = identifyJob.Schedule(springPoints.Length, 64);

        // Don't forget to dispose temporary NativeArrays later!
        nativeVerts.Dispose(jobHandle);
        nativeTris.Dispose(jobHandle);
    }

    public (Vector3[], int[]) ApplyMeshSubdivisionJobs(int[] meshTriangles, Matrix4x4 worldToLocal)
    {
        NativeArray<int> baseTriangles = new(meshTriangles.Length, Allocator.TempJob);
        NativeArray<int3> generatedTriangles = new(surfaceSpringPoints.Length * 3, Allocator.TempJob);
        NativeArray<float3> baseVertices = new(meshVerticesNative.Length + surfaceSpringPoints.Length, Allocator.TempJob);

        // Update mesh vertices
        for (int i = 0; i < meshVerticesNative.Length; i++)
        {
            baseVertices[i] = meshVerticesNative[i];
        }

        // Update mesh triangles
        for (int i = 0; i < meshTriangles.Length; i++)
        {
            baseTriangles[i] = meshTriangles[i];
        }

        // Copy to NativeArray
        NativeArray<float3> localPositions = surfacePointsLocalSpace.AsArray();

        var transformToLocalJob = new TransformToLocalSurfacePointsJob
        {
            surfacePoints = surfaceSpringPoints,
            worldToLocal = worldToLocal,

            localPositions = localPositions,
        };
        transformToLocalJob.Schedule(surfaceSpringPoints.Length, 64).Complete();

        // Copy back to NativeList
        for (int i = 0; i < localPositions.Length; i++)
        {
            surfacePointsLocalSpace[i] = localPositions[i];
        }

        // Append surface vertices
        for (int i = 0; i < surfaceSpringPoints.Length; i++)
            baseVertices[originalVertexCount + i] = surfacePointsLocalSpace[i];

        var generateTrianglesJob = new GenerateSurfaceTrianglesJob
        {
            surfaceLocalPoints = surfacePointsLocalSpace,
            inVertices = meshVerticesNative,
            inTriangles = baseTriangles,
            originalVertexCount = originalVertexCount,
            outTriangles = generatedTriangles
        };
        generateTrianglesJob.Schedule().Complete();

        // Merge data
        Vector3[] finalVertices = new Vector3[baseVertices.Length];
        for (int i = 0; i < finalVertices.Length; i++)
            finalVertices[i] = baseVertices[i];

        List<int> finalTriangles = new List<int>(baseTriangles.Length + surfaceSpringPoints.Length * 3 * 3);
        finalTriangles.AddRange(meshTriangles);

        for (int i = 0; i < generatedTriangles.Length; i++)
        {
            int3 tri = generatedTriangles[i];
            finalTriangles.Add(tri.x);
            finalTriangles.Add(tri.y);
            finalTriangles.Add(tri.z);
        }

        localPositions.Dispose();

        baseVertices.Dispose();
        baseTriangles.Dispose();
        generatedTriangles.Dispose();

        return (finalVertices, finalTriangles.ToArray());
    }

    public void CompleteAllJobsAndApply()
    {
        jobHandle.Complete();
    }

    public void DispatchMeshUpdate(Vector3[] meshVerts, Matrix4x4 worldToLocal, Mesh targetMesh)
    {
        NativeArray<float3> positions = new(springPoints.Length, Allocator.Persistent);

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
    }

    private void OnDestroy()
    {
        vertexBuffers[0].Dispose();
        vertexBuffers[1].Dispose();

        PointPosBuffer.Dispose();
        weightMapBuffer.Dispose();
    }
}