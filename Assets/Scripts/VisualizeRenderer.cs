using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

public class VisualizeRenderer
{
    // SpringPoint
    private Mesh pointMesh;
    private Material pointMaterial;
    private Matrix4x4[] pointMatrices;
    private const int maxBatchSize = 1023;

    // Spring Connection
    private ComputeBuffer connectionBuffer;
    private Material connectionMaterial;
    private int connectionCount;

    public void CreatePointMeshAndMaterial()
    {
        // === 1. Generate sphere mesh (default sphere) ===
        GameObject tempSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        pointMesh = Object.Instantiate(tempSphere.GetComponent<MeshFilter>().sharedMesh);
        Object.Destroy(tempSphere); // Clean up temporary object

        // === 2. Create simple instanced material ===
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");

        if (shader == null)
        {
            Debug.LogError("URP/Lit shader not found.");
            return;
        }

        pointMaterial = new Material(shader);
        pointMaterial.enableInstancing = true;
    }

    public void CreateConnectionMaterial()
    {
        if (connectionMaterial == null)
        {
            Shader connectionShader = Shader.Find("Custom/InstancedLineShader");
            //Shader connectionShader = Resources.Load<Shader>("InstancedLineShader");
            if (connectionShader != null)
            {
                connectionMaterial = new Material(connectionShader);
                connectionMaterial.enableInstancing = true;
            }
            else
            {
                Debug.LogError("Shader not found!");
            }
        }
    }

    public void DrawInstancedPoints(bool visualize, NativeArray<SpringPointData> springPoints)
    {
        if (!visualize || pointMesh == null || pointMaterial == null || springPoints == null)
        {
            return;
        }


        int count = springPoints.Length;
        if (count == 0) return; // no points

        if (pointMatrices == null || pointMatrices.Length != count)
        {
            pointMatrices = new Matrix4x4[count];
        }

        // Build transformation matrix per point
        for (int i = 0; i < count; i++)
        {
            Vector3 pos = springPoints[i].position;
            pointMatrices[i] = Matrix4x4.TRS(pos, Quaternion.identity, Vector3.one * 0.05f);
        }

        // Batch draw in groups of 1023
        for (int i = 0; i < count; i += maxBatchSize)
        {
            int len = Mathf.Min(maxBatchSize, count - i);
            Graphics.DrawMeshInstanced(pointMesh, 0, pointMaterial, pointMatrices, len, null, UnityEngine.Rendering.ShadowCastingMode.Off, false);
        }
    }

    public void UploadConnectionsToGPU(NativeArray<SpringPointData> springPoints, NativeArray<SpringConnectionData> springConnections)
    {
        if (springPoints == null || springConnections == null || springPoints.Length == 0 || springConnections.Length == 0)
        {
            return;
        }

        connectionCount = springConnections.Length;
        int totalPoints = connectionCount * 2;

        Vector3[] positions = new Vector3[totalPoints];
        for (int i = 0; i < connectionCount; i++)
        {
            positions[i * 2] = springPoints[springConnections[i].pointA].position;
            positions[i * 2 + 1] = springPoints[springConnections[i].pointB].position;
        }

        // Dispose old buffer if needed
        if (connectionBuffer == null || connectionBuffer.count != totalPoints)
        {
            if (connectionBuffer != null) connectionBuffer.Release();
            connectionBuffer = new ComputeBuffer(totalPoints, sizeof(float) * 3);
        }
        connectionBuffer.SetData(positions);
    }

    public void DrawInstancedConnections(bool visualize, Vector3 centerPosition, NativeArray<SpringPointData> springPoints, NativeArray<SpringConnectionData> springConnections)
    {
        if (!visualize || connectionBuffer == null || connectionMaterial == null || connectionCount == 0 || springPoints == null || springConnections == null)
        {
            return;
        }

        // Update connection data if changed
        if (connectionCount != springConnections.Length)
        {
            UploadConnectionsToGPU(springPoints, springConnections);
        }

        // Set material properties
        connectionMaterial.SetBuffer("_LineBuffer", connectionBuffer);
        connectionMaterial.SetColor("_Color", Color.white);
        connectionMaterial.SetFloat("_Thickness", 0.05f);

        // renders a number of line primitives directly on the GPU
        // using procedural geometry — meaning no mesh is needed.
        Graphics.DrawProcedural(
            connectionMaterial, // The material that contains the shader
            new Bounds(centerPosition, Vector3.one * 100f), // Large bounds
            MeshTopology.Lines, // for rendering lines
            connectionCount * 2, // 2 vertices per line
            1, // Instance count (not instancing)
            null, // Camera (null = current)
            null, // Material properties
            ShadowCastingMode.Off,
            false // Receive shadows
        );
    }

    public void Dispose()
    {
        if (pointMaterial != null)
        {
            Object.Destroy(pointMaterial);
            pointMaterial = null;
        }

        if (connectionMaterial != null)
        {
            Object.Destroy(connectionMaterial);
            connectionMaterial = null;
        }

        if (connectionBuffer != null)
        {
            connectionBuffer.Release();
        }
    }
}