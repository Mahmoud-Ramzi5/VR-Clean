using SimpleFileBrowser; // Required for file browser
using Assimp.Unmanaged;  // For AssimpContext
using Assimp;           // For PostProcessSteps, Mesh, etc.
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;

public class Menu7 : MenuDataBinderBase
{
    public GameObject panel;
    public OctreeSpringFiller target;
    public Button cubeButton;
    public Button sphereButton;
    public Button cylinderButton;
    public Button capsuleButton;
    public Button uploadButton;
    public MeshFilter targetMeshFilter;
    public UnityEngine.Mesh cubeMesh;
    public UnityEngine.Mesh sphereMesh;
    public UnityEngine.Mesh cylinderMesh;
    public UnityEngine.Mesh capsuleMesh;

    private UnityEngine.Mesh selectedMesh;  // Unity's mesh
    private Assimp.Mesh assimpMesh;        // Assimp's mesh

    void Start()
    {
        // Set up button listeners
        cubeButton.onClick.AddListener(() => SelectMesh(cubeMesh));
        sphereButton.onClick.AddListener(() => SelectMesh(sphereMesh));
        cylinderButton.onClick.AddListener(() => SelectMesh(cylinderMesh));
        capsuleButton.onClick.AddListener(() => SelectMesh(capsuleMesh));
        uploadButton.onClick.AddListener(OpenFileBrowser);

        // Configure file browser (run once at start)
        FileBrowser.SetFilters(true, new FileBrowser.Filter("3D Models", ".obj"));
        FileBrowser.SetDefaultFilter(".obj");
        FileBrowser.SetExcludedExtensions(".lnk", ".tmp", ".zip", ".rar", ".exe");
    }

    void OpenFileBrowser()
    {
        FileBrowser.SetFilters(true, new FileBrowser.Filter("3D Models", ".fbx", ".obj"));
        FileBrowser.ShowLoadDialog(
            onSuccess: (paths) => LoadFBXModel(paths[0]),
            onCancel: () => Debug.Log("Cancelled"),
            FileBrowser.PickMode.Files,
            title: "Select FBX/OBJ"
        );
    }

    void LoadFBXModel(string filePath)
    {
        // Load FBX using Assimp
        var assimpContext = new AssimpContext();
        var scene = assimpContext.ImportFile(filePath, PostProcessSteps.Triangulate);

        if (scene.MeshCount == 0)
        {
            Debug.LogError("No meshes found in FBX!");
            return;
        }

        // Convert Assimp mesh to Unity mesh
        UnityEngine.Mesh unityMesh = ConvertAssimpMeshToUnityMesh(scene.Meshes[0]);
        unityMesh.name = Path.GetFileNameWithoutExtension(filePath);
        SelectMesh(unityMesh);
    }

    UnityEngine.Mesh ConvertAssimpMeshToUnityMesh(Assimp.Mesh assimpMesh)
    {
        UnityEngine.Mesh unityMesh = new UnityEngine.Mesh();

        // Vertices
        Vector3[] vertices = new Vector3[assimpMesh.VertexCount];
        for (int i = 0; i < assimpMesh.VertexCount; i++)
        {
            vertices[i] = new Vector3(
                assimpMesh.Vertices[i].X,
                assimpMesh.Vertices[i].Y,
                assimpMesh.Vertices[i].Z
            );
        }
        unityMesh.vertices = vertices;

        // Triangles (indices)
        int[] triangles = new int[assimpMesh.FaceCount * 3];
        for (int i = 0; i < assimpMesh.FaceCount; i++)
        {
            var face = assimpMesh.Faces[i];
            triangles[i * 3] = face.Indices[0];
            triangles[i * 3 + 1] = face.Indices[1];
            triangles[i * 3 + 2] = face.Indices[2];
        }
        unityMesh.triangles = triangles;

        // Normals (if available)
        if (assimpMesh.HasNormals)
        {
            Vector3[] normals = new Vector3[assimpMesh.VertexCount];
            for (int i = 0; i < assimpMesh.VertexCount; i++)
            {
                normals[i] = new Vector3(
                    assimpMesh.Normals[i].X,
                    assimpMesh.Normals[i].Y,
                    assimpMesh.Normals[i].Z
                );
            }
            unityMesh.normals = normals;
        }
        else
        {
            unityMesh.RecalculateNormals();
        }

        // UVs (if available)
        if (assimpMesh.HasTextureCoords(0))
        {
            Vector2[] uvs = new Vector2[assimpMesh.VertexCount];
            for (int i = 0; i < assimpMesh.VertexCount; i++)
            {
                uvs[i] = new Vector2(
                    assimpMesh.TextureCoordinateChannels[0][i].X,
                    assimpMesh.TextureCoordinateChannels[0][i].Y
                );
            }
            unityMesh.uv = uvs;
        }

        unityMesh.RecalculateBounds();
        return unityMesh;
    }

    void SelectMesh(UnityEngine.Mesh newMesh)
    {
        selectedMesh = newMesh;
        if (selectedMesh != null && targetMeshFilter != null)
        {
            // Fix: Avoid type mismatch by clearing sharedMesh first
            targetMeshFilter.sharedMesh = null;
            targetMeshFilter.mesh = selectedMesh;

            if (target != null)
            {
                target.targetMesh = selectedMesh;
                panel.SetActive(false);
            }

            Debug.Log("Mesh loaded and assigned successfully.");
        }
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        // Your existing logic
    }
}
//public static class OBJLoader
//{
//    public static UnityEngine.Mesh ParseOBJ(string objFileContent)
//    {
//        List<Vector3> vertices = new List<Vector3>();
//        List<Vector2> uvs = new List<Vector2>();
//        List<Vector3> normals = new List<Vector3>();
//        List<int> triangles = new List<int>();

//        // Temporary lists for face data
//        List<Vector3> finalVertices = new List<Vector3>();
//        List<Vector2> finalUVs = new List<Vector2>();
//        List<Vector3> finalNormals = new List<Vector3>();
//        List<int> finalTriangles = new List<int>();

//        using (StringReader reader = new StringReader(objFileContent))
//        {
//            string line;
//            while ((line = reader.ReadLine()) != null)
//            {
//                line = line.Trim();
//                if (string.IsNullOrEmpty(line)) continue;

//                string[] parts = line.Split(new[] { ' ' }, System.StringSplitOptions.RemoveEmptyEntries);
//                if (parts.Length == 0) continue;

//                switch (parts[0])
//                {
//                    case "v": // Vertex
//                        if (parts.Length >= 4)
//                        {
//                            vertices.Add(new Vector3(
//                                float.Parse(parts[1]),
//                                float.Parse(parts[2]),
//                                float.Parse(parts[3])
//                            ));
//                        }
//                        break;

//                    case "vt": // Texture coordinate
//                        if (parts.Length >= 3)
//                        {
//                            uvs.Add(new Vector2(
//                                float.Parse(parts[1]),
//                                float.Parse(parts[2])
//                            ));
//                        }
//                        break;

//                    case "vn": // Normal
//                        if (parts.Length >= 4)
//                        {
//                            normals.Add(new Vector3(
//                                float.Parse(parts[1]),
//                                float.Parse(parts[2]),
//                                float.Parse(parts[3])
//                            ));
//                        }
//                        break;

//                    case "f": // Face
//                        if (parts.Length >= 4)
//                        {
//                            // For each face-vertex (triangles)
//                            for (int i = 1; i <= 3; i++)
//                            {
//                                string[] faceParts = parts[i].Split('/');

//                                // Parse vertex index (required)
//                                int vertexIndex = int.Parse(faceParts[0]);
//                                if (vertexIndex < 0) vertexIndex = vertices.Count + vertexIndex;
//                                else vertexIndex--;

//                                // Parse UV index (if exists)
//                                int uvIndex = -1;
//                                if (faceParts.Length > 1 && !string.IsNullOrEmpty(faceParts[1]))
//                                {
//                                    uvIndex = int.Parse(faceParts[1]);
//                                    if (uvIndex < 0) uvIndex = uvs.Count + uvIndex;
//                                    else uvIndex--;
//                                }

//                                // Parse normal index (if exists)
//                                int normalIndex = -1;
//                                if (faceParts.Length > 2 && !string.IsNullOrEmpty(faceParts[2]))
//                                {
//                                    normalIndex = int.Parse(faceParts[2]);
//                                    if (normalIndex < 0) normalIndex = normals.Count + normalIndex;
//                                    else normalIndex--;
//                                }

//                                // Add vertex data
//                                finalVertices.Add(vertices[vertexIndex]);

//                                // Add UV if available
//                                if (uvIndex >= 0 && uvIndex < uvs.Count)
//                                    finalUVs.Add(uvs[uvIndex]);
//                                else
//                                    finalUVs.Add(Vector2.zero); // Default UV

//                                // Add normal if available
//                                if (normalIndex >= 0 && normalIndex < normals.Count)
//                                    finalNormals.Add(normals[normalIndex]);
//                                else
//                                    finalNormals.Add(Vector3.up); // Default normal

//                                // Add triangle index
//                                finalTriangles.Add(finalVertices.Count - 1);
//                            }
//                        }
//                        break;
//                }
//            }
//        }

//        UnityEngine.Mesh mesh = new UnityEngine.Mesh();
//        mesh.vertices = finalVertices.ToArray();
//        mesh.triangles = finalTriangles.ToArray();

//        // Apply UVs if any were found
//        if (finalUVs.Count > 0)
//            mesh.uv = finalUVs.ToArray();

//        // Apply normals if any were found
//        if (finalNormals.Count > 0)
//            mesh.normals = finalNormals.ToArray();
//        else
//            mesh.RecalculateNormals();

//        mesh.RecalculateBounds();
//        return mesh;
//    }
//}