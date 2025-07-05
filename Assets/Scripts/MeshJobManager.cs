using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


public class MeshJobManager : MonoBehaviour
{
    public struct WeightedPoint
    {
        public int index;
        public float weight;
    }

    private NativeParallelMultiHashMap<int, WeightedPoint> vertexPointMap;

    private NativeArray<float3> vertices;
    private NativeArray<SpringPointData> springPoints;

    private JobHandle meshUpdateJobHandle;

    public void InitializeArrays(Vector3[] meshVertices, NativeArray<SpringPointData> springPoints)
    {
        vertices = new NativeArray<float3>(meshVertices.Length, Allocator.Persistent);
        this.springPoints = springPoints;

        for (int i = 0; i < meshVertices.Length; i++)
        {
            // Update mesh vertices
            vertices[i] = (float3)meshVertices[i];
        }

        // Build the vertex - to - point relationship map once at startup
        vertexPointMap = new NativeParallelMultiHashMap<int, WeightedPoint>(meshVertices.Length * 3, Allocator.Persistent);
        PrecomputeVertexMapping();
    }

    public void PrecomputeVertexMapping()
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            // Transform vertex to world space
            Vector3 worldPos = transform.TransformPoint(vertices[i]);

            // Find closest spring point
            List<(int index, float distSq)> closest = new();
            for (int j = 0; j < springPoints.Length; j++)
            {
                float distSq = math.lengthsq((float3)worldPos - springPoints[j].position);
                closest.Add((j, distSq));
            }

            // Sort by squared distance and take top 3
            closest.Sort((a, b) => a.distSq.CompareTo(b.distSq));
            float totalWeight = 0f;

            for (int k = 0; k < math.min(3, closest.Count); k++)
            {
                totalWeight += 1f / (closest[k].distSq + 1e-4f); // Avoid division by zero
            }

            // Store relationships
            for (int k = 0; k < math.min(3, closest.Count); k++)
            {
                float invDist = 1f / (closest[k].distSq + 1e-4f);
                float weight = invDist / totalWeight;

                vertexPointMap.Add(i, new WeightedPoint
                {
                    index = closest[k].index,
                    weight = weight
                });
            }
        }
    }


    // Burst-compiled job
    [BurstCompile]
    struct MeshUpdateJob : IJobParallelFor
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, WeightedPoint> vertexMap;
        [ReadOnly] public NativeArray<SpringPointData> springPoints;

        [ReadOnly] public float4x4 localToWorld;
        [ReadOnly] public float4x4 worldToLocal;

        [WriteOnly] public NativeArray<float3> vertices;

        public void Execute(int i)
        {
            float3 blended = float3.zero;
            float totalWeight = 0f;
            if (vertexMap.TryGetFirstValue(i, out WeightedPoint wp, out var it))
            {
                do
                {
                    blended += springPoints[wp.index].position * wp.weight;
                    totalWeight += wp.weight; // calculate the total weight
                } while (vertexMap.TryGetNextValue(out wp, ref it));

                if (totalWeight > 0f)
                {
                    blended /= totalWeight;
                    vertices[i] = math.transform(worldToLocal, blended);
                }
            }
        }
    }

    public void ScheduleMeshUpdateJobs(Vector3[] meshVertices, Matrix4x4 localToWorldMatrix, Matrix4x4 worldToLocalMatrix)
    {
        if (vertices.Length != meshVertices.Length)
        {
            if (vertices.IsCreated) vertices.Dispose();
            vertices = new NativeArray<float3>(meshVertices.Length, Allocator.Persistent);
        }

        for (int i = 0; i < meshVertices.Length; i++)
        {
            // Update mesh vertices
            vertices[i] = (float3)meshVertices[i];
        }

        var meshUpdateJob = new MeshUpdateJob
        {
            vertices = vertices,
            vertexMap = vertexPointMap,
            springPoints = springPoints,
            localToWorld = localToWorldMatrix,
            worldToLocal = worldToLocalMatrix
        };

        meshUpdateJobHandle = meshUpdateJob.Schedule(meshVertices.Length, 64);
    }

    public void CompleteAllJobsAndApply(Mesh targetMesh)
    {
        // Complete All jobs (for now only one)
        meshUpdateJobHandle.Complete();

        // Check if mesh exists and is writable
        if (targetMesh == null)
        {
            Debug.LogError("Target mesh is null!");
            return;
        }

        try
        {
            // Copy NativeArray to regular array
            Vector3[] newVertices = new Vector3[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                newVertices[i] = (Vector3)vertices[i];
            }

            // Apply updated vertices to the mesh
            targetMesh.MarkDynamic();
            targetMesh.vertices = newVertices;
            targetMesh.UploadMeshData(false);
            targetMesh.RecalculateNormals();
            targetMesh.RecalculateBounds();
        }
        catch (Exception e)
        {
            Debug.LogError($"Mesh update failed: {e.Message}");
        }
    }

    private void OnDestroy()
    {
        if (vertices.IsCreated) vertices.Dispose();
        if (springPoints.IsCreated) springPoints.Dispose();
        if (vertexPointMap.IsCreated) vertexPointMap.Dispose();
    }
}