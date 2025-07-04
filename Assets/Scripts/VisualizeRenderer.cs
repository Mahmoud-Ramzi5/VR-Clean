using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class VisualizeRenderer: MonoBehaviour
{
    // SpringPoint
    private Mesh pointMesh;
    private Material pointMaterial;
    private Matrix4x4[] pointMatrices;
    private const int maxBatchSize = 2047;

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
            Debug.LogError("URP/Lit shader not found. Make sure URP is installed and active.");
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

    public void UploadConnectionsToGPU(List<SpringConnection> springConnections)
    {
        if (springConnections == null || springConnections.Count == 0)
        {
            return;
        }

        connectionCount = springConnections.Count;
        int totalPoints = connectionCount * 2;

        Vector3[] positions = new Vector3[totalPoints];
        for (int i = 0; i < connectionCount; i++)
        {
            positions[i * 2] = springConnections[i].point1.position;
            positions[i * 2 + 1] = springConnections[i].point2.position;
        }

        // Dispose old buffer if needed
        if (connectionBuffer != null)
        {  
            connectionBuffer.Release();
        }

        connectionBuffer = new ComputeBuffer(totalPoints, sizeof(float) * 3);
        connectionBuffer.SetData(positions);
    }


    public void DrawInstancedPoints(bool visualize, List<SpringPoint> springPoints)
    {
        if (!visualize || pointMesh == null || pointMaterial == null || springPoints == null)
        {
            return;
        }


        int count = springPoints.Count;
        if (count == 0) return; // no points

        if (pointMatrices == null || pointMatrices.Length != count)
        {
            pointMatrices = new Matrix4x4[count];
        }

        // Build transformation matrix per point
        for (int i = 0; i < count; i++)
        {
            float radius = springPoints[i].radius;
            Vector3 pos = springPoints[i].position;
            pointMatrices[i] = Matrix4x4.TRS(pos, Quaternion.identity, Vector3.one * radius);
        }

        // Batch draw in groups of 1023
        for (int i = 0; i < count; i += maxBatchSize)
        {
            int len = Mathf.Min(maxBatchSize, count - i);
            Graphics.DrawMeshInstanced(pointMesh, 0, pointMaterial, pointMatrices, len, null, UnityEngine.Rendering.ShadowCastingMode.Off, false);
        }
    }

    public void DrawInstancedConnections(bool visualize, List<SpringConnection> springConnections, Vector3 centerPosition)
    {
        if (!visualize || connectionBuffer == null || connectionMaterial == null || connectionCount == 0 || springConnections == null)
        {
            return;
        }

        // Update connection data if changed
        if (connectionCount != springConnections.Count)
        {
            UploadConnectionsToGPU(springConnections);
        }

        // Set material properties
        connectionMaterial.SetBuffer("_LineBuffer", connectionBuffer);
        connectionMaterial.SetColor("_Color", Color.green);
        connectionMaterial.SetFloat("_Thickness", 0.02f);

        // Draw all lines in one call
        Graphics.DrawProcedural(
            connectionMaterial,
            new Bounds(centerPosition, Vector3.one * 100f), // Large bounds
            MeshTopology.Lines,
            connectionCount * 2, // 2 vertices per line
            1, // Instance count
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