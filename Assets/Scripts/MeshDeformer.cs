using System;
using System.Collections.Generic;
using UnityEngine;

public class MeshDeformer : MonoBehaviour
{
    public OctreeSpringFiller springFiller;
    public MeshFilter meshFilter;
    public int maxSubdivisionLevel = 3;
    public float influenceRadius = 1.0f;
    public bool showWeights = false;
    public bool logSubdividedTriangles = true;

    private Mesh originalMesh;
    private Mesh workingMesh;
    private List<WeightedInfluence>[] vertexInfluences;
    private Vector3[] baseVertices;
    private Vector3[] currentVertices;
    private int[] baseTriangles;
    private int[] currentTriangles;

    private struct TriangleData
    {
        public int originalIndex;
        public int subdivisionLevel;
        public bool canSubdivide;
    }
    private List<TriangleData> triangleDataList;
    public bool isInitialized = false;
    
    private bool deformationActive = true;
    private float lastDeformationUpdate = 0f;
    private const float DEFORMATION_UPDATE_INTERVAL = 1f / 60f; // 60fps updates
    private Dictionary<int, int> vertexToSpringPointMap = new Dictionary<int, int>();

    // Add these methods to your MeshDeformer class:

    // Method to force immediate deformation update
    public void ForceDeformationUpdate()
    {
        UpdateDeformation();
    }

    // Public method to enable/disable deformation
    public void SetDeformationActive(bool active)
    {
        deformationActive = active;
        Debug.Log($"Mesh deformation {(active ? "enabled" : "disabled")}");
    }
    void Start()
    {
        if (!meshFilter) meshFilter = GetComponent<MeshFilter>();
        originalMesh = meshFilter.mesh;
        springFiller = GetComponent<OctreeSpringFiller>();

        InitializeDeformationMesh();
        BuildInfluenceMapping();
        BuildVertexToSpringPointMapping();
        isInitialized = true;
    }

    void InitializeDeformationMesh()
    {
        workingMesh = Instantiate(originalMesh); // Create a copy to avoid modifying original
        meshFilter.mesh = workingMesh; // Assign the copy to the mesh filter

        baseVertices = workingMesh.vertices;
        currentVertices = (Vector3[])baseVertices.Clone();
        baseTriangles = workingMesh.triangles;
        currentTriangles = (int[])baseTriangles.Clone();

        triangleDataList = new List<TriangleData>();
        int triangleCount = baseTriangles.Length / 3;

        Debug.Log($"Initializing {triangleCount} triangles from {baseTriangles.Length} triangle indices");

        for (int i = 0; i < triangleCount; i++)
        {
            triangleDataList.Add(new TriangleData
            {
                originalIndex = i,
                subdivisionLevel = 0,
                canSubdivide = true
            });
        }

        Debug.Log($"Triangle data list initialized with {triangleDataList.Count} entries");
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            HandleMouseClick();
        }
    }

    private void HandleMouseClick()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        if (FindClosestTriangle(ray, out int triangleIndex, out Vector3 hitPoint))
        {
            Vector3 localPoint = transform.InverseTransformPoint(hitPoint);
            SubdivideSingleTriangle(triangleIndex, localPoint);
        }
    }

    private bool FindClosestTriangle(Ray ray, out int closestTriangleIndex, out Vector3 hitPoint)
    {
        closestTriangleIndex = -1;
        hitPoint = Vector3.zero;
        float closestDistance = Mathf.Infinity;

        Matrix4x4 localToWorld = transform.localToWorldMatrix;
        Vector3[] vertices = workingMesh.vertices;
        int[] triangles = workingMesh.triangles;

        for (int i = 0; i < triangles.Length - 2; i += 3)
        {
            Vector3 v0 = localToWorld.MultiplyPoint3x4(vertices[triangles[i]]);
            Vector3 v1 = localToWorld.MultiplyPoint3x4(vertices[triangles[i + 1]]);
            Vector3 v2 = localToWorld.MultiplyPoint3x4(vertices[triangles[i + 2]]);

            if (RayIntersectsTriangle(ray, v0, v1, v2, out Vector3 intersectPoint))
            {
                float distance = Vector3.Distance(ray.origin, intersectPoint);
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestTriangleIndex = i / 3;
                    hitPoint = intersectPoint;
                }
            }
        }

        return closestTriangleIndex != -1;
    }

    private bool RayIntersectsTriangle(Ray ray, Vector3 v0, Vector3 v1, Vector3 v2, out Vector3 intersectPoint)
    {
        intersectPoint = Vector3.zero;
        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;
        Vector3 h = Vector3.Cross(ray.direction, e2);
        float a = Vector3.Dot(e1, h);

        if (a > -Mathf.Epsilon && a < Mathf.Epsilon)
            return false;

        float f = 1.0f / a;
        Vector3 s = ray.origin - v0;
        float u = f * Vector3.Dot(s, h);

        if (u < 0.0 || u > 1.0)
            return false;

        Vector3 q = Vector3.Cross(s, e1);
        float v = f * Vector3.Dot(ray.direction, q);

        if (v < 0.0 || u + v > 1.0)
            return false;

        float t = f * Vector3.Dot(e2, q);
        if (t > Mathf.Epsilon)
        {
            intersectPoint = ray.origin + ray.direction * t;
            return true;
        }

        return false;
    }

    private void SubdivideSingleTriangle(int triangleIndex, Vector3 localPoint)
    {
        // Validate triangle index
        if (triangleIndex < 0 || triangleIndex >= triangleDataList.Count)
        {
            Debug.LogError($"Invalid triangle index: {triangleIndex}. Triangle data list has {triangleDataList.Count} entries.");
            return;
        }

        TriangleData data = triangleDataList[triangleIndex];
        if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel)
            return;

        if (logSubdividedTriangles)
        {
            Debug.Log($"Subdividing triangle {triangleIndex} at position {localPoint}");
        }

        SubdivideSelectedTriangles(new List<int> { triangleIndex });
        BuildInfluenceMapping();
    }

    public void HandleCollisionPoints(List<Vector3> collisionPoints)
    {
        if (collisionPoints == null || collisionPoints.Count == 0)
        {
            Debug.LogWarning("HandleCollisionPoints called with null or empty collision points list");
            return;
        }

        foreach (Vector3 point in collisionPoints)
        {
            Vector3 localPoint = transform.InverseTransformPoint(point);
            FindAndSubdivideAffectedTriangles(localPoint);
        }

        NotifyMeshChanged();
    }

    private void FindAndSubdivideAffectedTriangles(Vector3 localPoint)
    {
        Vector3[] vertices = workingMesh.vertices;
        int[] triangles = workingMesh.triangles;
        List<int> trianglesToSubdivide = new List<int>();

        // Ensure we have valid triangles array
        if (triangles == null || triangles.Length < 3)
        {
            Debug.LogError("Invalid triangles array in FindAndSubdivideAffectedTriangles");
            return;
        }

        int triangleCount = triangles.Length / 3;
        Debug.Log($"Checking {triangleCount} triangles against {triangleDataList.Count} triangle data entries");

        for (int i = 0; i < triangles.Length - 2; i += 3)
        {
            int triangleIndex = i / 3;

            // Bounds check for triangle data list
            if (triangleIndex >= triangleDataList.Count)
            {
                Debug.LogError($"Triangle index {triangleIndex} exceeds triangle data list size {triangleDataList.Count}");
                continue;
            }

            TriangleData data = triangleDataList[triangleIndex];

            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel)
                continue;

            // Bounds check for vertex indices
            if (triangles[i] >= vertices.Length || triangles[i + 1] >= vertices.Length || triangles[i + 2] >= vertices.Length)
            {
                Debug.LogError($"Triangle {triangleIndex} has invalid vertex indices: {triangles[i]}, {triangles[i + 1]}, {triangles[i + 2]}. Vertex count: {vertices.Length}");
                continue;
            }

            Vector3 v1 = vertices[triangles[i]];
            Vector3 v2 = vertices[triangles[i + 1]];
            Vector3 v3 = vertices[triangles[i + 2]];

            if (IsPointNearTriangle(localPoint, v1, v2, v3))
            {
                trianglesToSubdivide.Add(triangleIndex);
            }
        }

        if (trianglesToSubdivide.Count > 0)
        {
            if (logSubdividedTriangles)
            {
                Debug.Log($"Subdividing {trianglesToSubdivide.Count} triangles at indices: " +
                         string.Join(", ", trianglesToSubdivide));
            }

            SubdivideSelectedTriangles(trianglesToSubdivide);
            BuildInfluenceMapping();
        }
    }

    private bool IsPointNearTriangle(Vector3 point, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        Vector3 centroid = (v1 + v2 + v3) / 3f;
        float distance = Vector3.Distance(point, centroid);
        return distance < influenceRadius;
    }

    private struct Edge : IEquatable<Edge>
    {
        public readonly int v1, v2;
        public readonly Vector3 pos1, pos2;
        private readonly int hashCode;

        public Edge(int a, int b, Vector3[] vertices)
        {
            v1 = Mathf.Min(a, b);
            v2 = Mathf.Max(a, b);
            pos1 = vertices[v1];
            pos2 = vertices[v2];
            hashCode = pos1.GetHashCode() ^ pos2.GetHashCode();
        }

        public bool Equals(Edge other)
        {
            return (pos1.Equals(other.pos1) && pos2.Equals(other.pos2)) ||
                   (pos1.Equals(other.pos2) && pos2.Equals(other.pos1));
        }

        public override bool Equals(object obj) => obj is Edge e && Equals(e);
        public override int GetHashCode() => hashCode;
    }

    private int GetMidpoint(int a, int b, Vector3[] verts, List<Vector3> newVerts, Dictionary<Edge, int> midpoints, bool create = true)
    {
        Edge edge = new Edge(a, b, verts);
        if (midpoints.TryGetValue(edge, out int index)) return index;

        Vector3 mid = (verts[a] + verts[b]) * 0.5f;
        newVerts.Add(mid);
        int newIndex = newVerts.Count - 1;
        midpoints.Add(edge, newIndex);

        if (create && springFiller != null)
        {
            Vector3 worldPos = transform.TransformPoint(mid);
            springFiller.AddSpringPointAtPosition(worldPos);
        }

        return newIndex;
    }

    void BuildInfluenceMapping()
    {
        if (baseVertices == null || baseVertices.Length == 0)
        {
            Debug.LogWarning("BuildInfluenceMapping called with null or empty baseVertices");
            return;
        }

        if (springFiller == null || springFiller.allSpringPoints.Length == 0)
        {
            Debug.LogWarning("BuildInfluenceMapping called with null springFiller or empty spring points");
            return;
        }

        vertexInfluences = new List<WeightedInfluence>[baseVertices.Length];

        for (int i = 0; i < baseVertices.Length; i++)
        {
            vertexInfluences[i] = new List<WeightedInfluence>();
            Vector3 vertexWorld = transform.TransformPoint(baseVertices[i]);

            foreach (SpringPointData sp in springFiller.allSpringPoints)
            {
                float distance = Vector3.Distance(vertexWorld, sp.position);
                if (distance < influenceRadius)
                {
                    float weight = Mathf.Exp(-distance * distance);
                    vertexInfluences[i].Add(new WeightedInfluence
                    {
                        springPoint = sp,
                        weight = weight
                    });
                }
            }
        }
    }

    void LateUpdate()
    {
        if (deformationActive && Time.time - lastDeformationUpdate >= DEFORMATION_UPDATE_INTERVAL)
        {
            UpdateDeformation();
            lastDeformationUpdate = Time.time;
        }
    }

    public void OnMeshTopologyChanged()
    {
        Debug.Log("Mesh topology changed, rebuilding mappings...");

        // Update cached mesh data
        if (workingMesh != null)
        {
            baseVertices = workingMesh.vertices;
            currentVertices = (Vector3[])baseVertices.Clone();
            baseTriangles = workingMesh.triangles;
            currentTriangles = (int[])baseTriangles.Clone();
        }

        // Rebuild mappings
        BuildInfluenceMapping();
        BuildVertexToSpringPointMapping();
    }

    void UpdateDeformation()
    {
        if (!deformationActive || currentVertices == null || baseVertices == null)
            return;

        if (springFiller == null || !springFiller.allSpringPoints.IsCreated || springFiller.allSpringPoints.Length == 0)
            return;

        bool meshChanged = false;

        // Method 1: Use vertex to spring point mapping (primary method)
        if (vertexToSpringPointMap.Count > 0)
        {
            foreach (var kvp in vertexToSpringPointMap)
            {
                int vertexIndex = kvp.Key;
                int springPointIndex = kvp.Value;

                if (vertexIndex >= 0 && vertexIndex < currentVertices.Length &&
                    springPointIndex >= 0 && springPointIndex < springFiller.allSpringPoints.Length)
                {
                    SpringPointData springPoint = springFiller.allSpringPoints[springPointIndex];
                    Vector3 newLocalPos = transform.InverseTransformPoint(springPoint.position);

                    // Check if vertex needs updating
                    if (Vector3.Distance(currentVertices[vertexIndex], newLocalPos) > 0.001f)
                    {
                        currentVertices[vertexIndex] = newLocalPos;
                        meshChanged = true;
                    }
                }
            }
        }
        else
        {
            // Method 2: Fallback using influence mapping
            if (vertexInfluences != null)
            {
                for (int i = 0; i < currentVertices.Length; i++)
                {
                    if (i >= vertexInfluences.Length) continue;

                    Vector3 newPos = Vector3.zero;
                    float totalWeight = 0f;
                    int influenceCount = 0;

                    foreach (var inf in vertexInfluences[i])
                    {
                        Vector3 springWorldPos = inf.springPoint.position;
                        Vector3 initialWorldPos = inf.springPoint.initialPosition;
                        Vector3 displacement = springWorldPos - initialWorldPos;

                        newPos += displacement * inf.weight;
                        totalWeight += inf.weight;
                        influenceCount++;
                    }

                    if (totalWeight > 0.01f && influenceCount > 0)
                    {
                        Vector3 finalPos = baseVertices[i] + newPos / totalWeight;

                        if (Vector3.Distance(currentVertices[i], finalPos) > 0.001f)
                        {
                            currentVertices[i] = finalPos;
                            meshChanged = true;
                        }
                    }
                }
            }
        }

        // Apply mesh changes if any occurred
        if (meshChanged)
        {
            ApplyMeshDeformation();
        }
    }

    void ApplyMeshDeformation()
    {
        if (workingMesh == null || currentVertices == null)
            return;

        try
        {
            workingMesh.vertices = currentVertices;
            workingMesh.RecalculateNormals();
            workingMesh.RecalculateBounds();

            // Update base vertices for next frame
            baseVertices = (Vector3[])currentVertices.Clone();

            NotifyMeshChanged();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error applying mesh deformation: {e.Message}");
        }
    }

    void BuildVertexToSpringPointMapping()
    {
        if (springFiller == null || !springFiller.allSpringPoints.IsCreated || springFiller.allSpringPoints.Length == 0)
        {
            Debug.LogWarning("Cannot build vertex mapping: spring filler not ready");
            return;
        }

        vertexToSpringPointMap.Clear();

        if (baseVertices == null || baseVertices.Length == 0)
        {
            Debug.LogWarning("Cannot build vertex mapping: no base vertices");
            return;
        }

        Debug.Log($"Building vertex to spring point mapping for {baseVertices.Length} vertices and {springFiller.allSpringPoints.Length} spring points");

        for (int vertexIndex = 0; vertexIndex < baseVertices.Length; vertexIndex++)
        {
            Vector3 worldVertexPos = transform.TransformPoint(baseVertices[vertexIndex]);

            int closestSpringPointIndex = -1;
            float minDistance = float.MaxValue;

            // Find closest spring point
            for (int springIndex = 0; springIndex < springFiller.allSpringPoints.Length; springIndex++)
            {
                float distance = Vector3.Distance(worldVertexPos, springFiller.allSpringPoints[springIndex].position);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestSpringPointIndex = springIndex;
                }
            }

            if (closestSpringPointIndex >= 0)
            {
                vertexToSpringPointMap[vertexIndex] = closestSpringPointIndex;
            }
        }

        Debug.Log($"Mapped {vertexToSpringPointMap.Count} vertices to spring points");
    }

    public void NotifyMeshChanged()
    {
        if (springFiller != null)
        {
            // Mesh change notification logic here
        }
    }

    void OnDrawGizmosSelected()
    {
        if (!showWeights || vertexInfluences == null || currentVertices == null) return;

        Gizmos.color = Color.cyan;
        for (int i = 0; i < currentVertices.Length; i++)
        {
            if (i >= vertexInfluences.Length) continue;

            Vector3 worldPos = transform.TransformPoint(currentVertices[i]);
            foreach (var inf in vertexInfluences[i])
            {
                Gizmos.DrawLine(worldPos, inf.springPoint.position);
            }
        }
    }

    public void SubdivideMeshWithPoints(List<SpringPointData> newPoints)
    {
        if (newPoints == null || newPoints.Count == 0)
        {
            Debug.LogWarning("SubdivideMeshWithPoints called with null or empty points list");
            return;
        }

        if (currentVertices == null || currentTriangles == null)
        {
            Debug.LogError("SubdivideMeshWithPoints called with null mesh data");
            return;
        }

        Vector3[] worldVertices = new Vector3[currentVertices.Length];
        for (int i = 0; i < currentVertices.Length; i++)
        {
            worldVertices[i] = transform.TransformPoint(currentVertices[i]);
        }

        List<int> trianglesToSubdivide = new List<int>();
        int triangleCount = currentTriangles.Length / 3;

        Debug.Log($"SubdivideMeshWithPoints: Checking {triangleCount} triangles against {triangleDataList.Count} triangle data entries");

        for (int i = 0; i < currentTriangles.Length - 2; i += 3)
        {
            int triangleIndex = i / 3;

            // Bounds check
            if (triangleIndex >= triangleDataList.Count)
            {
                Debug.LogError($"Triangle index {triangleIndex} exceeds triangle data list size {triangleDataList.Count}");
                continue;
            }

            TriangleData data = triangleDataList[triangleIndex];
            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel) continue;

            // Vertex bounds check
            if (currentTriangles[i] >= worldVertices.Length ||
                currentTriangles[i + 1] >= worldVertices.Length ||
                currentTriangles[i + 2] >= worldVertices.Length)
            {
                Debug.LogError($"Triangle {triangleIndex} has invalid vertex indices");
                continue;
            }

            Vector3 v0 = worldVertices[currentTriangles[i]];
            Vector3 v1 = worldVertices[currentTriangles[i + 1]];
            Vector3 v2 = worldVertices[currentTriangles[i + 2]];
            Vector3 centroid = (v0 + v1 + v2) / 3f;

            int springPointCount = 0;
            foreach (SpringPointData pointData in newPoints)
            {
                if (Vector3.Distance(pointData.position, centroid) < influenceRadius)
                {
                    springPointCount++;
                    if (springPointCount > 3) break;
                }
            }

            if (springPointCount > 3)
            {
                trianglesToSubdivide.Add(triangleIndex);
            }
        }

        if (trianglesToSubdivide.Count > 0)
        {
            SubdivideAllTriangles(false);
            // Recursive call - be careful of infinite recursion
            if (trianglesToSubdivide.Count < triangleCount) // Only recurse if we're making progress
            {
                SubdivideMeshWithPoints(newPoints);
            }
        }
    }

    public void SubdivideAllTriangles(bool create = true)
    {
        if (currentTriangles == null || currentTriangles.Length == 0)
        {
            Debug.LogError("SubdivideAllTriangles called with null or empty triangles");
            return;
        }

        List<int> allTriangleIndices = new List<int>();
        int triangleCount = currentTriangles.Length / 3;

        for (int i = 0; i < triangleCount; i++)
        {
            allTriangleIndices.Add(i);
        }

        Debug.Log($"SubdivideAllTriangles: Processing {allTriangleIndices.Count} triangles");
        SubdivideSelectedTriangles(allTriangleIndices, create);
    }

    private void SubdivideSelectedTriangles(List<int> triangleIndices, bool create = true)
    {
        if (triangleIndices == null || triangleIndices.Count == 0)
        {
            Debug.LogWarning("SubdivideSelectedTriangles called with null or empty triangle indices");
            return;
        }

        if (workingMesh == null)
        {
            Debug.LogError("SubdivideSelectedTriangles called with null working mesh");
            return;
        }

        Vector3[] oldVertices = workingMesh.vertices;
        int[] oldTriangles = workingMesh.triangles;

        if (oldVertices == null || oldTriangles == null)
        {
            Debug.LogError("SubdivideSelectedTriangles: Mesh has null vertices or triangles");
            return;
        }

        Debug.Log($"SubdivideSelectedTriangles: Starting with {oldVertices.Length} vertices, {oldTriangles.Length / 3} triangles, {triangleDataList.Count} triangle data entries");

        List<Vector3> newVertices = new List<Vector3>(oldVertices);

        // Validate all triangle indices before processing
        foreach (int triangleIndex in triangleIndices)
        {
            if (triangleIndex < 0 || triangleIndex >= triangleDataList.Count)
            {
                Debug.LogError($"Invalid triangle index in subdivision: {triangleIndex}. Valid range: 0-{triangleDataList.Count - 1}");
                return;
            }

            int baseIdx = triangleIndex * 3;
            if (baseIdx + 2 >= oldTriangles.Length)
            {
                Debug.LogError($"Triangle index {triangleIndex} points to invalid triangle array position {baseIdx}. Array length: {oldTriangles.Length}");
                return;
            }
        }

        // Build vertex-to-triangles mapping
        Dictionary<int, List<int>> vertexToTriangles = new Dictionary<int, List<int>>();
        for (int i = 0; i < oldTriangles.Length; i++)
        {
            int vertexIndex = oldTriangles[i];
            if (vertexIndex >= 0 && vertexIndex < oldVertices.Length)
            {
                if (!vertexToTriangles.ContainsKey(vertexIndex))
                {
                    vertexToTriangles[vertexIndex] = new List<int>();
                }
                vertexToTriangles[vertexIndex].Add(i / 3);
            }
        }

        // Build edge-to-triangles mapping
        Dictionary<Edge, List<int>> edgeToTriangles = new Dictionary<Edge, List<int>>();
        for (int i = 0; i < oldTriangles.Length - 2; i += 3)
        {
            int triangleIdx = i / 3;
            int i0 = oldTriangles[i];
            int i1 = oldTriangles[i + 1];
            int i2 = oldTriangles[i + 2];

            // Validate vertex indices
            if (i0 >= 0 && i0 < oldVertices.Length &&
                i1 >= 0 && i1 < oldVertices.Length &&
                i2 >= 0 && i2 < oldVertices.Length)
            {
                AddEdgeToMap(new Edge(i0, i1, oldVertices), triangleIdx, edgeToTriangles);
                AddEdgeToMap(new Edge(i1, i2, oldVertices), triangleIdx, edgeToTriangles);
                AddEdgeToMap(new Edge(i2, i0, oldVertices), triangleIdx, edgeToTriangles);
            }
        }

        // Find all triangles to subdivide using BFS
        HashSet<int> trianglesToSubdivide = new HashSet<int>();
        Queue<int> trianglesToProcess = new Queue<int>(triangleIndices);

        while (trianglesToProcess.Count > 0)
        {
            int currentTri = trianglesToProcess.Dequeue();

            if (currentTri < 0 || currentTri >= triangleDataList.Count)
            {
                Debug.LogWarning($"Invalid triangle index during BFS: {currentTri}");
                continue;
            }

            if (trianglesToSubdivide.Contains(currentTri)) continue;

            TriangleData data = triangleDataList[currentTri];
            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel) continue;

            trianglesToSubdivide.Add(currentTri);

            // Get vertices of this triangle
            int baseIdx = currentTri * 3;
            if (baseIdx + 2 < oldTriangles.Length)
            {
                int v0 = oldTriangles[baseIdx];
                int v1 = oldTriangles[baseIdx + 1];
                int v2 = oldTriangles[baseIdx + 2];

                // Find connected triangles through shared vertices
                foreach (int vertexIndex in new[] { v0, v1, v2 })
                {
                    if (vertexToTriangles.TryGetValue(vertexIndex, out List<int> connectedTris))
                    {
                        foreach (int connectedTri in connectedTris)
                        {
                            if (!trianglesToSubdivide.Contains(connectedTri))
                            {
                                trianglesToProcess.Enqueue(connectedTri);
                            }
                        }
                    }
                }
            }
        }

        // Create midpoints for edges of triangles to be subdivided
        Dictionary<Edge, int> edgeMidpoints = new Dictionary<Edge, int>();
        foreach (int triIdx in trianglesToSubdivide)
        {
            int baseIdx = triIdx * 3;
            if (baseIdx + 2 < oldTriangles.Length)
            {
                int i0 = oldTriangles[baseIdx];
                int i1 = oldTriangles[baseIdx + 1];
                int i2 = oldTriangles[baseIdx + 2];

                if (i0 < oldVertices.Length && i1 < oldVertices.Length && i2 < oldVertices.Length)
                {
                    GetMidpoint(i0, i1, oldVertices, newVertices, edgeMidpoints, create);
                    GetMidpoint(i1, i2, oldVertices, newVertices, edgeMidpoints, create);
                    GetMidpoint(i2, i0, oldVertices, newVertices, edgeMidpoints, create);
                }
            }
        }

        // Rebuild triangles
        List<int> newTriangles = new List<int>();
        List<TriangleData> newTriangleData = new List<TriangleData>();

        for (int i = 0; i < oldTriangles.Length - 2; i += 3)
        {
            int originalTriangleIndex = i / 3;

            if (originalTriangleIndex >= triangleDataList.Count)
            {
                Debug.LogError($"Original triangle index {originalTriangleIndex} out of range");
                continue;
            }

            TriangleData originalData = triangleDataList[originalTriangleIndex];

            int i0 = oldTriangles[i];
            int i1 = oldTriangles[i + 1];
            int i2 = oldTriangles[i + 2];

            // Validate vertex indices
            if (i0 >= oldVertices.Length || i1 >= oldVertices.Length || i2 >= oldVertices.Length)
            {
                Debug.LogError($"Invalid vertex indices in triangle {originalTriangleIndex}: {i0}, {i1}, {i2}");
                continue;
            }

            bool has_m01 = edgeMidpoints.TryGetValue(new Edge(i0, i1, oldVertices), out int m01);
            bool has_m12 = edgeMidpoints.TryGetValue(new Edge(i1, i2, oldVertices), out int m12);
            bool has_m20 = edgeMidpoints.TryGetValue(new Edge(i2, i0, oldVertices), out int m20);

            int splitEdgeCount = (has_m01 ? 1 : 0) + (has_m12 ? 1 : 0) + (has_m20 ? 1 : 0);

            switch (splitEdgeCount)
            {
                case 3:
                    int newLevel = originalData.subdivisionLevel + 1;
                    newTriangles.AddRange(new[] { i0, m01, m20 });
                    newTriangles.AddRange(new[] { m01, i1, m12 });
                    newTriangles.AddRange(new[] { m20, m12, i2 });
                    newTriangles.AddRange(new[] { m01, m12, m20 });

                    for (int j = 0; j < 4; j++)
                    {
                        newTriangleData.Add(new TriangleData
                        {
                            originalIndex = originalData.originalIndex,
                            subdivisionLevel = newLevel,
                            canSubdivide = newLevel < maxSubdivisionLevel
                        });
                    }
                    break;

                case 2:
                    if (has_m01 && has_m12)
                    {
                        newTriangles.AddRange(new[] { i0, m01, m12 });
                        newTriangles.AddRange(new[] { i0, m12, i2 });
                        newTriangles.AddRange(new[] { m01, i1, m12 });
                    }
                    else if (has_m12 && has_m20)
                    {
                        newTriangles.AddRange(new[] { i1, m12, m20 });
                        newTriangles.AddRange(new[] { i1, m20, i0 });
                        newTriangles.AddRange(new[] { m12, i2, m20 });
                    }
                    else
                    {
                        newTriangles.AddRange(new[] { i2, m20, m01 });
                        newTriangles.AddRange(new[] { i2, m01, i1 });
                        newTriangles.AddRange(new[] { m20, i0, m01 });
                    }

                    for (int j = 0; j < 3; j++)
                    {
                        newTriangleData.Add(new TriangleData
                        {
                            originalIndex = originalData.originalIndex,
                            subdivisionLevel = originalData.subdivisionLevel,
                            canSubdivide = originalData.canSubdivide
                        });
                    }
                    break;

                case 1:
                    if (has_m01)
                    {
                        newTriangles.AddRange(new[] { i2, i0, m01 });
                        newTriangles.AddRange(new[] { i2, m01, i1 });
                    }
                    else if (has_m12)
                    {
                        newTriangles.AddRange(new[] { i0, i1, m12 });
                        newTriangles.AddRange(new[] { i0, m12, i2 });
                    }
                    else
                    {
                        newTriangles.AddRange(new[] { i1, i2, m20 });
                        newTriangles.AddRange(new[] { i1, m20, i0 });
                    }

                    for (int j = 0; j < 2; j++)
                    {
                        newTriangleData.Add(new TriangleData
                        {
                            originalIndex = originalData.originalIndex,
                            subdivisionLevel = originalData.subdivisionLevel,
                            canSubdivide = originalData.canSubdivide
                        });
                    }
                    break;

                case 0:
                default:
                    newTriangles.AddRange(new[] { i0, i1, i2 });
                    newTriangleData.Add(originalData);
                    break;
            }
        }

        Debug.Log($"SubdivideSelectedTriangles: Final result - {newVertices.Count} vertices, {newTriangles.Count / 3} triangles, {newTriangleData.Count} triangle data entries");

        // Validate final arrays before applying
        if (newTriangles.Count % 3 != 0)
        {
            Debug.LogError($"Invalid triangle count: {newTriangles.Count} (should be multiple of 3)");
            return;
        }

        // Validate all triangle indices
        for (int i = 0; i < newTriangles.Count; i++)
        {
            if (newTriangles[i] < 0 || newTriangles[i] >= newVertices.Count)
            {
                Debug.LogError($"Invalid triangle vertex index: {newTriangles[i]} (vertex count: {newVertices.Count})");
                return;
            }
        }

        // Update mesh data
        workingMesh.Clear();
        workingMesh.vertices = newVertices.ToArray();
        workingMesh.triangles = newTriangles.ToArray();
        workingMesh.RecalculateNormals();
        workingMesh.RecalculateBounds();

        triangleDataList = newTriangleData;

        baseVertices = workingMesh.vertices;
        currentVertices = (Vector3[])baseVertices.Clone();
        baseTriangles = workingMesh.triangles;
        currentTriangles = (int[])baseTriangles.Clone();

        Debug.Log($"Mesh subdivision complete. New mesh: {baseVertices.Length} vertices, {baseTriangles.Length / 3} triangles");
    }

    private void AddEdgeToMap(Edge edge, int triangleIdx, Dictionary<Edge, List<int>> edgeToTriangles)
    {
        if (!edgeToTriangles.ContainsKey(edge))
        {
            edgeToTriangles[edge] = new List<int>();
        }
        edgeToTriangles[edge].Add(triangleIdx);
    }

    private struct WeightedInfluence
    {
        public SpringPointData springPoint;
        public float weight;
    }

    // Additional validation methods
    private bool ValidateMeshData()
    {
        if (workingMesh == null)
        {
            Debug.LogError("Working mesh is null");
            return false;
        }

        Vector3[] vertices = workingMesh.vertices;
        int[] triangles = workingMesh.triangles;

        if (vertices == null || vertices.Length == 0)
        {
            Debug.LogError("Mesh has no vertices");
            return false;
        }

        if (triangles == null || triangles.Length == 0)
        {
            Debug.LogError("Mesh has no triangles");
            return false;
        }

        if (triangles.Length % 3 != 0)
        {
            Debug.LogError($"Triangle array length {triangles.Length} is not divisible by 3");
            return false;
        }

        // Check triangle indices
        for (int i = 0; i < triangles.Length; i++)
        {
            if (triangles[i] < 0 || triangles[i] >= vertices.Length)
            {
                Debug.LogError($"Triangle index {i} has invalid vertex reference: {triangles[i]} (vertex count: {vertices.Length})");
                return false;
            }
        }

        int expectedTriangleDataCount = triangles.Length / 3;
        if (triangleDataList.Count != expectedTriangleDataCount)
        {
            Debug.LogError($"Triangle data count mismatch: {triangleDataList.Count} vs expected {expectedTriangleDataCount}");
            return false;
        }

        return true;
    }

    private void RepairMeshData()
    {
        Debug.Log("Attempting to repair mesh data...");

        if (workingMesh == null)
        {
            Debug.LogError("Cannot repair null working mesh");
            return;
        }

        Vector3[] vertices = workingMesh.vertices;
        int[] triangles = workingMesh.triangles;

        if (vertices == null || vertices.Length == 0)
        {
            Debug.LogError("Cannot repair mesh with no vertices");
            return;
        }

        if (triangles == null)
        {
            triangles = new int[0];
        }

        // Remove invalid triangles
        List<int> validTriangles = new List<int>();
        for (int i = 0; i < triangles.Length - 2; i += 3)
        {
            if (triangles[i] >= 0 && triangles[i] < vertices.Length &&
                triangles[i + 1] >= 0 && triangles[i + 1] < vertices.Length &&
                triangles[i + 2] >= 0 && triangles[i + 2] < vertices.Length)
            {
                validTriangles.Add(triangles[i]);
                validTriangles.Add(triangles[i + 1]);
                validTriangles.Add(triangles[i + 2]);
            }
        }

        // Update mesh
        workingMesh.triangles = validTriangles.ToArray();

        // Rebuild triangle data list
        int triangleCount = validTriangles.Count / 3;
        triangleDataList.Clear();
        for (int i = 0; i < triangleCount; i++)
        {
            triangleDataList.Add(new TriangleData
            {
                originalIndex = i,
                subdivisionLevel = 0,
                canSubdivide = true
            });
        }

        // Update cached arrays
        baseVertices = workingMesh.vertices;
        currentVertices = (Vector3[])baseVertices.Clone();
        baseTriangles = workingMesh.triangles;
        currentTriangles = (int[])baseTriangles.Clone();

        Debug.Log($"Mesh repair complete: {vertices.Length} vertices, {triangleCount} triangles");
    }

    // Call this method periodically or when errors occur
    public void ValidateAndRepairMesh()
    {
        if (!ValidateMeshData())
        {
            RepairMeshData();
        }
    }
}