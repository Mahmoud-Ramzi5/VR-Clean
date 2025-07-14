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
        // Too high & expensive (515 vertices, 768 triangles)

        //GameObject tempSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //pointMesh = Object.Instantiate(tempSphere.GetComponent<MeshFilter>().sharedMesh);
        //Object.Destroy(tempSphere); // Clean up temporary object

        pointMesh = new Mesh();

        // === 1. Generate Icosahedron Mesh ===
        // Medium (12 vertices, 20 triangles)

        //// Golden ratio
        //float t = (1f + Mathf.Sqrt(5f)) / 2f;

        //// 12 vertices of an icosahedron
        //Vector3[] vertices = {
        //    new Vector3(-1f,  t, 0f).normalized,
        //    new Vector3( 1f,  t, 0f).normalized,
        //    new Vector3(-1f, -t, 0f).normalized,
        //    new Vector3( 1f, -t, 0f).normalized,

        //    new Vector3(0f, -1f,  t).normalized,
        //    new Vector3(0f,  1f,  t).normalized,
        //    new Vector3(0f, -1f, -t).normalized,
        //    new Vector3(0f,  1f, -t).normalized,

        //    new Vector3( t, 0f, -1f).normalized,
        //    new Vector3( t, 0f,  1f).normalized,
        //    new Vector3(-t, 0f, -1f).normalized,
        //    new Vector3(-t, 0f,  1f).normalized,
        //};

        //// 20 triangular faces
        //int[] triangles = {
        //    0, 11, 5,    0, 5, 1,    0, 1, 7,    0, 7, 10,   0, 10, 11,
        //    1, 5, 9,     5, 11, 4,   11, 10, 2,  10, 7, 6,    7, 1, 8,
        //    3, 9, 4,     3, 4, 2,    3, 2, 6,    3, 6, 8,     3, 8, 9,
        //    4, 9, 5,     2, 4, 11,   6, 2, 10,   8, 6, 7,     9, 8, 1,
        //};

        // === 1. Generate Octahedron Mesh ===
        // Lowst possible (6 vertices, 8 triangles)

        // 6 vertices of an octahedron
        Vector3[] vertices = {
            new Vector3( 1,  0,  0),
            new Vector3(-1,  0,  0),
            new Vector3( 0,  1,  0),
            new Vector3( 0, -1,  0),
            new Vector3( 0,  0,  1),
            new Vector3( 0,  0, -1),
        };

        // 8 triangular faces
        int[] triangles = {
            4, 0, 2,   4, 2, 1,   4, 1, 3,   4, 3, 0,
            5, 2, 0,   5, 1, 2,   5, 3, 1,   5, 0, 3,
        };

        pointMesh.vertices = vertices;
        pointMesh.triangles = triangles;
        pointMesh.RecalculateNormals();

        // === 2. Create simple instanced material ===
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");

        if (shader == null)
        {
            // Debug.LogError("URP/Lit shader not found.");
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
                // Debug.LogError("Shader not found!");
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
        // using procedural geometry � meaning no mesh is needed.
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
        // Clean up materials
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

        // Clean up mesh
        if (pointMesh != null)
        {
            Object.Destroy(pointMesh);
            pointMesh = null;
        }

        // Clean up compute buffer
        if (connectionBuffer != null)
        {
            connectionBuffer.Release();
            connectionBuffer = null;
        }
    }
}