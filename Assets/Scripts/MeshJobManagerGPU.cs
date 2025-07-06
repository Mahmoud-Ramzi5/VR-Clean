using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;


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

    private NativeArray<float3> vertices;
    private NativeArray<SpringPointData> springPoints;

    private ComputeShader meshCompute;
    private int kernelId;

    private ComputeBuffer vertexBuffer;         // RWStructuredBuffer<float3>
    private ComputeBuffer PointPosBuffer;    // StructuredBuffer<float3>
    private ComputeBuffer weightMapBuffer;      // StructuredBuffer<VertexWeightBinding>
    private VertexWeightBinding[] vertexWeightBindings;

    public void Initialize(Vector3[] meshVertices, NativeArray<SpringPointData> springPoints)
    {
        vertices = new NativeArray<float3>(meshVertices.Length, Allocator.Persistent);
        this.springPoints = springPoints;

        for (int i = 0; i < meshVertices.Length; i++)
        {
            // Update mesh vertices
            vertices[i] = (float3)meshVertices[i];
        }

        meshCompute = Resources.Load<ComputeShader>("MeshUpdater");
        kernelId = meshCompute.FindKernel("CSMain");
        PrecomputeVertexMapping();
    }

    public void PrecomputeVertexMapping()
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

    public void DispatchMeshUpdate(Vector3[] meshVerts, Matrix4x4 worldToLocal, Mesh targetMesh)
    {
        NativeArray<float3> positions = new(springPoints.Length, Allocator.Persistent);

        new GetPositionsJob {
            springPoints = springPoints,
            positions = positions,
        }.Schedule(springPoints.Length, 64).Complete();

        int count = meshVerts.Length;
        vertexBuffer = new ComputeBuffer(count, sizeof(float) * 3);
        PointPosBuffer = new ComputeBuffer(positions.Length, sizeof(float) * 3);
        weightMapBuffer = new ComputeBuffer(vertexWeightBindings.Length, 3 * (sizeof(int) + sizeof(float)));

        vertexBuffer.SetData(meshVerts);
        PointPosBuffer.SetData(positions);
        weightMapBuffer.SetData(vertexWeightBindings);

        meshCompute.SetBuffer(kernelId, "Vertices", vertexBuffer);
        meshCompute.SetBuffer(kernelId, "PointsPositions", PointPosBuffer);
        meshCompute.SetBuffer(kernelId, "VertexBindings", weightMapBuffer);

        meshCompute.SetMatrix("worldToLocal", worldToLocal);
        meshCompute.Dispatch(kernelId, Mathf.CeilToInt(count / 64f), 1, 1);

        UnityEngine.Rendering.AsyncGPUReadback.Request(vertexBuffer, (request) =>
        {
            if (!request.hasError)
            {
                if (targetMesh != null)  // check if mesh still exists
                {
                    var data = request.GetData<Vector3>();
                    data.CopyTo(meshVerts);
                    targetMesh.vertices = meshVerts;
                    targetMesh.RecalculateBounds();
                    targetMesh.RecalculateNormals();
                }
            }
        });

        //// Read back
        //vertexBuffer.GetData(meshVerts);

        //// Apply to mesh
        //targetMesh.vertices = meshVerts;
        //targetMesh.RecalculateBounds();
        //targetMesh.RecalculateNormals();


        // Cleanup
        vertexBuffer.Dispose();
        PointPosBuffer.Dispose();
        weightMapBuffer.Dispose();
    }
}