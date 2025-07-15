using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using Unity.Collections;
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
    [SerializeField] private int maxSubdivisionsPerFrame = 50;  // Tune based on perf


    private struct TriangleData
    {
        public int originalIndex;
        public int subdivisionLevel;
        public bool canSubdivide;
    }
    private List<TriangleData> triangleDataList;
    public bool isInitialized = false;

    void Start()
    {
        if (!meshFilter) meshFilter = GetComponent<MeshFilter>();
        originalMesh = meshFilter.mesh;
        springFiller = GetComponent<OctreeSpringFiller>();

        InitializeDeformationMesh();
        BuildInfluenceMapping();
        isInitialized = true;
    }

    void InitializeDeformationMesh()
    {
        workingMesh = originalMesh;

        baseVertices = workingMesh.vertices;
        currentVertices = baseVertices.Clone() as Vector3[];
        baseTriangles = workingMesh.triangles;
        currentTriangles = baseTriangles.Clone() as int[];

        triangleDataList = new List<TriangleData>();
        int triangleCount = workingMesh.triangles.Length / 3;
        for (int i = 0; i < triangleCount; i++)
        {
            triangleDataList.Add(new TriangleData
            {
                originalIndex = i,
                subdivisionLevel = 0,
                canSubdivide = true
            });
        }
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

        for (int i = 0; i < triangles.Length; i += 3)
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
        TriangleData data = triangleDataList[triangleIndex];
        if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel)
            return;

        if (logSubdividedTriangles)
        {
            // Debug.Log($"Subdividing triangle {triangleIndex} at position {localPoint}");
        }

        SubdivideSelectedTriangles(new List<int> { triangleIndex });
        BuildInfluenceMapping();
    }

    public void HandleCollisionPoints(List<Vector3> collisionPoints)
    {
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

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int triangleIndex = i / 3;
            TriangleData data = triangleDataList[triangleIndex];

            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel)
                continue;

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
                // Debug.Log($"Subdividing {trianglesToSubdivide.Count} triangles at indices: " +
                //         string.Join(", ", trianglesToSubdivide));
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

        Vector3 worldPos = transform.TransformPoint(mid);
        if (create)
        {
            springFiller.AddSpringPointAtPosition(worldPos);
        }

        return newIndex;
    }

    void BuildInfluenceMapping()
    {
        //Vector3[] currentMeshVertices = workingMesh.vertices;
        //vertexInfluences = new List<WeightedInfluence>[currentMeshVertices.Length];
        //for (int i = 0; i < currentMeshVertices.Length; i++)
        //{
        //    vertexInfluences[i] = new List<WeightedInfluence>();
        //    Vector3 vertexWorld = transform.TransformPoint(currentMeshVertices[i]);
        //    float totalWeight = 0f;
        //    int maxInfluences = 4;  // Limit for perf

        //    // Sort springs by distance and take top N
        //    var closestSprings = springFiller.allSpringPoints
        //        .Select((sp, idx) => new { Dist = Vector3.Distance(vertexWorld, sp.position), Sp = sp, Idx = idx })
        //        .OrderBy(x => x.Dist)
        //        .Take(maxInfluences);

        //    foreach (var cs in closestSprings)
        //    {
        //        if (cs.Dist < influenceRadius)
        //        {
        //            float weight = 1f / (cs.Dist + 0.01f);  // Inverse distance weighting
        //            totalWeight += weight;
        //            vertexInfluences[i].Add(new WeightedInfluence { springPoint = cs.Sp, weight = weight });
        //        }
        //    }

        //    // Normalize weights
        //    for (int j = 0; j < vertexInfluences[i].Count; j++)
        //    {
        //        var inf = vertexInfluences[i][j];
        //        inf.weight /= totalWeight;
        //        vertexInfluences[i][j] = inf;
        //    }
        //}
    }

    void LateUpdate()
    {
        //UpdateDeformation();
    }

    void UpdateDeformation()
    {
        for (int i = 0; i < currentVertices.Length; i++)
        {
            Vector3 displacement = Vector3.zero;
            float totalWeight = 0f;
            foreach (var inf in vertexInfluences[i])
            {
                Vector3 disp = inf.springPoint.position - inf.springPoint.initialPosition;
                displacement += disp * inf.weight;
                totalWeight += inf.weight;
            }
            currentVertices[i] = baseVertices[i] + (totalWeight > 0 ? displacement / totalWeight : Vector3.zero);
        }
        NotifyMeshChanged();
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
        if (!showWeights || vertexInfluences == null) return;

        Gizmos.color = Color.cyan;
        for (int i = 0; i < currentVertices.Length; i++)
        {
            Vector3 worldPos = transform.TransformPoint(currentVertices[i]);
            foreach (var inf in vertexInfluences[i])
            {
                Gizmos.DrawLine(worldPos, inf.springPoint.position);
            }
        }
    }

    public void SubdivideMeshWithPoints(NativeList<SpringPointData> newPoints)
    {
        Stopwatch sw = Stopwatch.StartNew();
        //StringBuilder debugLog = new StringBuilder("\n--- SubdivideMeshWithPoints() ---\n");

        if (newPoints.Length == 0)
        {
            //debugLog.Append("No new points - exiting.\n");
            //// Debug.Log(debugLog);
            return;
        }

        // Log initial input
        //debugLog.Append($"Input points:/ {newPoints.Length}\n");

        // Step 1: Add half of input (with ceiling division)
        int afterAddingHalf = newPoints.Length + CeilDivide(newPoints.Length, 2);
        //debugLog.Append($"After adding half: {newPoints.Length} + Ceil({newPoints.Length}/2) = {afterAddingHalf}\n");

        // Step 2: First division by 3 (ceiling)
        int afterFirstDivision = afterAddingHalf / 3;
        //debugLog.Append($"After first /3: Ceil({afterAddingHalf}/3) = {afterFirstDivision}\n");

        // Step 3: Second division by 3 (ceiling)
        int finalNumber = afterFirstDivision / 3;
        //debugLog.Append($"After second /3: Ceil({afterFirstDivision}/3) = {finalNumber}\n");


        int originalTriangleCount = originalMesh.triangles.Length / 3;
        //debugLog.Append($"Original triangles: {originalTriangleCount}\n");

        finalNumber = CeilDivide(finalNumber, 4);
        if (finalNumber == 0)
        {
            //debugLog.Append("After dividing by 4, finalNumber is zero, skipping subdivision to avoid division by zero.\n");
            // Debug.Log(debugLog.ToString());
            return;
        }

        int divisionRatio = originalTriangleCount / finalNumber;
        //debugLog.Append($"Check {originalTriangleCount}/{finalNumber} == 3: {divisionRatio}\n");

        if (divisionRatio % 3 == 0)
        {
            //debugLog.Append("CONDITIONS MET - Subdividing mesh!\n");
            //SubdivideAllTriangles(false);
            for (int i = 0; i < (divisionRatio); i++)
            {
                SubdivideAllTriangles(false);
            }

            return;
        }
        else
        {
            //debugLog.Append($"Division ratio {divisionRatio} != 3 - skipping subdivision.\n");
        }



        sw.Stop();
        UnityEngine.Debug.Log($"[MeshDeformer] SubdivideMeshWithPoints took {sw.ElapsedMilliseconds}ms for {newPoints.Length} new points");
    }
    private int CeilDivide(int a, int b)
    {
        return (a + b - 1) / b;  // Ensures rounding UP
    }
    public void SubdivideAllTriangles(bool create = true)
    {
        List<int> allTriangleIndices = new List<int>();
        for (int i = 0; i < currentTriangles.Length / 3; i++)
        {
            allTriangleIndices.Add(i);
        }
        UnityEngine.Debug.Log("yo");
        SubdivideSelectedTriangles(allTriangleIndices, create);
    }
    private void SubdivideSelectedTriangles(List<int> triangleIndices, bool create = true)
    {
        Stopwatch sw = Stopwatch.StartNew();
        Vector3[] oldVertices = workingMesh.vertices;
        int[] oldTriangles = workingMesh.triangles;
        List<Vector3> newVertices = new List<Vector3>(oldVertices);

        // Build vertex-to-triangles mapping (required for original stitching behavior)
        Dictionary<int, List<int>> vertexToTriangles = new Dictionary<int, List<int>>();
        for (int i = 0; i < oldTriangles.Length; i++)
        {
            int vertexIndex = oldTriangles[i];
            if (!vertexToTriangles.ContainsKey(vertexIndex))
            {
                vertexToTriangles[vertexIndex] = new List<int>();
            }
            vertexToTriangles[vertexIndex].Add(i / 3);
        }

        // Build edge-to-triangles mapping with position-aware edges (critical for stitching)
        Dictionary<Edge, List<int>> edgeToTriangles = new Dictionary<Edge, List<int>>();
        for (int i = 0; i < oldTriangles.Length; i += 3)
        {
            int triangleIdx = i / 3;
            int i0 = oldTriangles[i];
            int i1 = oldTriangles[i + 1];
            int i2 = oldTriangles[i + 2];

            AddEdgeToMap(new Edge(i0, i1, oldVertices), triangleIdx, edgeToTriangles);
            AddEdgeToMap(new Edge(i1, i2, oldVertices), triangleIdx, edgeToTriangles);
            AddEdgeToMap(new Edge(i2, i0, oldVertices), triangleIdx, edgeToTriangles);
        }

        // Find all triangles to subdivide using original BFS approach
        HashSet<int> trianglesToSubdivide = new HashSet<int>();
        Queue<int> trianglesToProcess = new Queue<int>(triangleIndices);

        while (trianglesToProcess.Count > 0)
        {
            int currentTri = trianglesToProcess.Dequeue();

            if (trianglesToSubdivide.Contains(currentTri)) continue;

            TriangleData data = triangleDataList[currentTri];
            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel) continue;

            trianglesToSubdivide.Add(currentTri);

            // Get all vertices of this triangle
            int baseIdx = currentTri * 3;
            int v0 = oldTriangles[baseIdx];
            int v1 = oldTriangles[baseIdx + 1];
            int v2 = oldTriangles[baseIdx + 2];

            // Find all triangles sharing these vertices (including on other faces)
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

        // Create midpoints for all edges of trfiangles to be subdivided
        Dictionary<Edge, int> edgeMidpoints = new Dictionary<Edge, int>();
        foreach (int triIdx in trianglesToSubdivide)
        {
            int i0 = oldTriangles[triIdx * 3];
            int i1 = oldTriangles[triIdx * 3 + 1];
            int i2 = oldTriangles[triIdx * 3 + 2];

            GetMidpoint(i0, i1, oldVertices, newVertices, edgeMidpoints, create);
            GetMidpoint(i1, i2, oldVertices, newVertices, edgeMidpoints, create);
            GetMidpoint(i2, i0, oldVertices, newVertices, edgeMidpoints, create);
        }

        // Rebuild all triangles using original splitting logic
        List<int> newTriangles = new List<int>();
        List<TriangleData> newTriangleData = new List<TriangleData>();

        for (int i = 0; i < oldTriangles.Length; i += 3)
        {
            int originalTriangleIndex = i / 3;
            TriangleData originalData = triangleDataList[originalTriangleIndex];

            int i0 = oldTriangles[i];
            int i1 = oldTriangles[i + 1];
            int i2 = oldTriangles[i + 2];

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

        // Update mesh data
        workingMesh.Clear();
        workingMesh.vertices = newVertices.ToArray();
        workingMesh.triangles = newTriangles.ToArray();
        workingMesh.RecalculateNormals();
        workingMesh.RecalculateBounds();

        triangleDataList = newTriangleData;
        baseVertices = workingMesh.vertices;
        currentVertices = baseVertices.Clone() as Vector3[];
        sw.Stop();
        UnityEngine.Debug.Log($"[MeshDeformer] BuildInfluenceMapping took {sw.ElapsedMilliseconds}ms for {workingMesh.vertices.Length} vertices and {springFiller.allSpringPoints.Length} springs");
    }

    private void AddEdgeToMap(Edge edge, int triangleIdx, Dictionary<Edge, List<int>> edgeToTriangles)
    {
        if (!edgeToTriangles.ContainsKey(edge))
        {
            edgeToTriangles[edge] = new List<int>();
        }
        edgeToTriangles[edge].Add(triangleIdx);
    }

    public void UpdateMeshWithPoints(NativeList<SpringPointData> newPoints)
    {
        if (workingMesh.vertexCount < newPoints.Length)
        {
            SubdivideMeshWithPoints(newPoints);
        }
        else
        {
            UnityEngine.Debug.Log("merge");
            MergeMeshWithPoints(newPoints);
        }
    }

    public int MergeMeshWithPoints(NativeList<SpringPointData> newPoints)
    {
        Stopwatch sw = Stopwatch.StartNew();
        if (newPoints.Length == 0)
        {
            return 0;
        }
        int first = meshFilter.mesh.vertices.Length - CeilDivide(meshFilter.mesh.vertices.Length, 3);
        // Step 1: Calculate the intermediate values in reverse order
        int second = CeilDivide(first, newPoints.Length);

        for (int i = 0; i < second; i++)
        {
            MergeAllTriangles(false);
        }

        sw.Stop();
        UnityEngine.Debug.Log($"[MeshDeformer] MergeMeshWithPoints took {sw.ElapsedMilliseconds}ms for {newPoints.Length} points");
        return 0;
    }

    public void MergeAllTriangles(bool create = true)
    {
        // Get current mesh data
        Vector3[] vertices = workingMesh.vertices;
        int[] triangles = workingMesh.triangles;
        Vector3[] normals = workingMesh.normals;

        // Use a tolerance based on mesh size for vertex welding
        float weldTolerance = workingMesh.bounds.size.magnitude * 0.001f;

        // Weld nearby vertices
        List<Vector3> weldedVertices = new List<Vector3>();
        Dictionary<int, int> vertexMapping = new Dictionary<int, int>();

        for (int i = 0; i < vertices.Length; i++)
        {
            bool found = false;
            for (int j = 0; j < weldedVertices.Count; j++)
            {
                if (Vector3.Distance(vertices[i], weldedVertices[j]) < weldTolerance)
                {
                    vertexMapping[i] = j;
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                vertexMapping[i] = weldedVertices.Count;
                weldedVertices.Add(vertices[i]);
            }
        }

        // Rebuild triangles with new vertex indices
        List<int> weldedTriangles = new List<int>();
        for (int i = 0; i < triangles.Length; i++)
        {
            weldedTriangles.Add(vertexMapping[triangles[i]]);
        }

        // Remove degenerate triangles (where all 3 vertices are the same)
        List<int> finalTriangles = new List<int>();
        for (int i = 0; i < weldedTriangles.Count; i += 3)
        {
            int v1 = weldedTriangles[i];
            int v2 = weldedTriangles[i + 1];
            int v3 = weldedTriangles[i + 2];

            // Only keep triangles with distinct vertices
            if (v1 != v2 && v2 != v3 && v1 != v3)
            {
                finalTriangles.Add(v1);
                finalTriangles.Add(v2);
                finalTriangles.Add(v3);
            }
        }

        // Update mesh data
        workingMesh.Clear();
        workingMesh.vertices = weldedVertices.ToArray();
        workingMesh.triangles = finalTriangles.ToArray();
        workingMesh.RecalculateNormals();
        workingMesh.RecalculateBounds();

        // Reset triangle data
        triangleDataList = new List<TriangleData>();
        int triangleCount = finalTriangles.Count / 3;
        for (int i = 0; i < triangleCount; i++)
        {
            triangleDataList.Add(new TriangleData
            {
                originalIndex = i,
                subdivisionLevel = 0,
                canSubdivide = true
            });
        }

        // Update cached references
        baseVertices = workingMesh.vertices;
        currentVertices = baseVertices.Clone() as Vector3[];
        baseTriangles = workingMesh.triangles;
        currentTriangles = baseTriangles.Clone() as int[];
    }


    // ... Existing variables remain unchanged ...

    private void MergeSelectedTriangles(List<int> triangleIndices)
    {
        if (triangleIndices.Count == 0) return;

        // Step 1: Build connected components of triangles
        List<HashSet<int>> components = FindConnectedComponents(new HashSet<int>(triangleIndices));

        // Collect all triangles to be replaced
        HashSet<int> allTrianglesToRemove = new HashSet<int>();
        foreach (var comp in components)
        {
            allTrianglesToRemove.UnionWith(comp);
        }

        // Create new triangle list and triangle data list
        List<int> newTriangleList = new List<int>();
        List<TriangleData> newTriangleDataList = new List<TriangleData>();

        // Add unaffected triangles
        for (int i = 0; i < triangleDataList.Count; i++)
        {
            if (!allTrianglesToRemove.Contains(i))
            {
                int baseIdx = i * 3;
                newTriangleList.Add(currentTriangles[baseIdx]);
                newTriangleList.Add(currentTriangles[baseIdx + 1]);
                newTriangleList.Add(currentTriangles[baseIdx + 2]);
                newTriangleDataList.Add(triangleDataList[i]);
            }
        }

        // Process each component
        foreach (var component in components)
        {
            // Collect edges and count occurrences
            Dictionary<Edge, int> edgeCounts = new Dictionary<Edge, int>();
            foreach (int triIdx in component)
            {
                int baseIdx = triIdx * 3;
                int i0 = currentTriangles[baseIdx];
                int i1 = currentTriangles[baseIdx + 1];
                int i2 = currentTriangles[baseIdx + 2];

                Edge e0 = new Edge(i0, i1, baseVertices);
                Edge e1 = new Edge(i1, i2, baseVertices);
                Edge e2 = new Edge(i2, i0, baseVertices);

                edgeCounts.TryGetValue(e0, out int count0);
                edgeCounts[e0] = count0 + 1;

                edgeCounts.TryGetValue(e1, out int count1);
                edgeCounts[e1] = count1 + 1;

                edgeCounts.TryGetValue(e2, out int count2);
                edgeCounts[e2] = count2 + 1;
            }

            // Identify boundary edges (only used by one triangle)
            List<Edge> boundaryEdges = new List<Edge>();
            foreach (var kvp in edgeCounts)
            {
                if (kvp.Value == 1)
                {
                    boundaryEdges.Add(kvp.Key);
                }
            }

            // Build boundary polygon
            List<int> boundaryPolygon = BuildBoundaryPolygon(boundaryEdges);
            if (boundaryPolygon.Count < 3) continue;

            // Triangulate the polygon
            List<int> newTris = TriangulateConvexPolygon(boundaryPolygon);

            // Determine new triangle data properties
            int minLevel = component.Min(triIdx => triangleDataList[triIdx].subdivisionLevel);
            int minOriginalIndex = component.Min(triIdx => triangleDataList[triIdx].originalIndex);
            int newLevel = Mathf.Max(0, minLevel - 1);
            bool canSubdivide = newLevel < maxSubdivisionLevel;

            // Add new triangles
            newTriangleList.AddRange(newTris);
            for (int i = 0; i < newTris.Count / 3; i++)
            {
                newTriangleDataList.Add(new TriangleData
                {
                    originalIndex = minOriginalIndex,
                    subdivisionLevel = newLevel,
                    canSubdivide = canSubdivide
                });
            }
        }

        // Update mesh data
        currentTriangles = newTriangleList.ToArray();
        triangleDataList = newTriangleDataList;
        workingMesh.triangles = currentTriangles;
        workingMesh.RecalculateNormals();
        workingMesh.RecalculateBounds();
    }

    private List<HashSet<int>> FindConnectedComponents(HashSet<int> trianglesToMerge)
    {
        List<HashSet<int>> components = new List<HashSet<int>>();
        HashSet<int> visited = new HashSet<int>();
        Dictionary<Edge, List<int>> edgeToTriangles = new Dictionary<Edge, List<int>>();

        // Build edge mapping
        foreach (int tri in trianglesToMerge)
        {
            int baseIdx = tri * 3;
            int i0 = currentTriangles[baseIdx];
            int i1 = currentTriangles[baseIdx + 1];
            int i2 = currentTriangles[baseIdx + 2];

            Edge e0 = new Edge(i0, i1, baseVertices);
            Edge e1 = new Edge(i1, i2, baseVertices);
            Edge e2 = new Edge(i2, i0, baseVertices);

            AddEdgeToMap(e0, tri, edgeToTriangles);
            AddEdgeToMap(e1, tri, edgeToTriangles);
            AddEdgeToMap(e2, tri, edgeToTriangles);
        }

        // Build adjacency
        Dictionary<int, List<int>> adj = new Dictionary<int, List<int>>();
        foreach (var kvp in edgeToTriangles)
        {
            List<int> tris = kvp.Value;
            if (tris.Count == 2)
            {
                int t1 = tris[0];
                int t2 = tris[1];
                if (!adj.ContainsKey(t1)) adj[t1] = new List<int>();
                if (!adj.ContainsKey(t2)) adj[t2] = new List<int>();
                adj[t1].Add(t2);
                adj[t2].Add(t1);
            }
        }

        // Find components using BFS
        foreach (int tri in trianglesToMerge)
        {
            if (visited.Contains(tri)) continue;

            HashSet<int> component = new HashSet<int>();
            Queue<int> queue = new Queue<int>();
            queue.Enqueue(tri);
            visited.Add(tri);

            while (queue.Count > 0)
            {
                int current = queue.Dequeue();
                component.Add(current);

                if (adj.TryGetValue(current, out List<int> neighbors))
                {
                    foreach (int neighbor in neighbors)
                    {
                        if (!visited.Contains(neighbor))
                        {
                            visited.Add(neighbor);
                            queue.Enqueue(neighbor);
                        }
                    }
                }
            }
            components.Add(component);
        }

        return components;
    }

    private List<int> BuildBoundaryPolygon(List<Edge> boundaryEdges)
    {
        Dictionary<int, List<int>> graph = new Dictionary<int, List<int>>();
        foreach (Edge edge in boundaryEdges)
        {
            if (!graph.ContainsKey(edge.v1)) graph[edge.v1] = new List<int>();
            if (!graph.ContainsKey(edge.v2)) graph[edge.v2] = new List<int>();
            graph[edge.v1].Add(edge.v2);
            graph[edge.v2].Add(edge.v1);
        }

        List<int> polygon = new List<int>();
        if (graph.Count == 0) return polygon;

        int start = graph.Keys.First();
        int current = start;
        int prev = -1;

        do
        {
            polygon.Add(current);
            List<int> neighbors = graph[current];
            int next = neighbors[0] == prev ? neighbors[1] : neighbors[0];
            prev = current;
            current = next;
        } while (current != start && polygon.Count <= graph.Count);

        return polygon;
    }

    private List<int> TriangulateConvexPolygon(List<int> vertices)
    {
        List<int> triangles = new List<int>();
        if (vertices.Count < 3) return triangles;

        for (int i = 1; i < vertices.Count - 1; i++)
        {
            triangles.Add(vertices[0]);
            triangles.Add(vertices[i]);
            triangles.Add(vertices[i + 1]);
        }

        return triangles;
    }

    // ... Rest of the class remains unchanged ...

    private struct WeightedInfluence
    {
        public SpringPointData springPoint;
        public float weight;
    }
}