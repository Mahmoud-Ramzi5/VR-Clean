using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class MeshDeformer : MonoBehaviour
{
    // Public fields remain unchanged for external configuration
    public OctreeSpringFiller springFiller;
    public MeshFilter meshFilter;
    public int maxSubdivisionLevel = 3;
    public float influenceRadius = 1.0f;
    public bool showWeights = false;
    public bool logSubdividedTriangles = true;

    // Private fields optimized for memory and performance
    private Mesh originalMesh;
    private Mesh workingMesh;
    private List<WeightedInfluence>[] vertexInfluences;
    private Vector3[] baseVertices;
    private Vector3[] currentVertices;
    private int[] baseTriangles;
    private List<TriangleData> triangleDataList;
    public bool isInitialized = false;

    // Simplified TriangleData struct
    private struct TriangleData
    {
        public int originalIndex;
        public byte subdivisionLevel; // Using byte to save memory since levels are small
        public bool canSubdivide;
    }

    // Optimized Edge struct with better hash distribution
    private readonly struct Edge
    {
        public readonly int v1, v2;
        public readonly Vector3 pos1, pos2;

        public Edge(int a, int b, Vector3[] vertices)
        {
            v1 = Mathf.Min(a, b);
            v2 = Mathf.Max(a, b);
            pos1 = vertices[v1];
            pos2 = vertices[v2];
        }

        public override int GetHashCode() => (v1 * 397) ^ v2;
        public override bool Equals(object obj) => obj is Edge other && v1 == other.v1 && v2 == other.v2;
    }

    // Simplified WeightedInfluence struct
    private struct WeightedInfluence
    {
        public SpringPointData springPoint;
        public float weight;
    }

    void Start()
    {
        // Initialize required components
        if (!meshFilter) meshFilter = GetComponent<MeshFilter>();
        if (!springFiller) springFiller = GetComponent<OctreeSpringFiller>();

        originalMesh = meshFilter.mesh;
        InitializeDeformationMesh();
        BuildInfluenceMapping();
        isInitialized = true;
    }

    void InitializeDeformationMesh()
    {
        workingMesh = Instantiate(originalMesh); // Create a copy to work with

        baseVertices = workingMesh.vertices;
        currentVertices = (Vector3[])baseVertices.Clone();
        baseTriangles = workingMesh.triangles;

        // Initialize triangle data
        int triangleCount = baseTriangles.Length / 3;
        triangleDataList = new List<TriangleData>(triangleCount);

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
            SubdivideSingleTriangle(triangleIndex, transform.InverseTransformPoint(hitPoint));
        }
    }

    private bool FindClosestTriangle(Ray ray, out int closestTriangleIndex, out Vector3 hitPoint)
    {
        closestTriangleIndex = -1;
        hitPoint = Vector3.zero;
        float closestDistance = float.MaxValue;

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
                float distance = (ray.origin - intersectPoint).sqrMagnitude;
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
            Debug.Log($"Subdividing triangle {triangleIndex} at position {localPoint}");
        }

        SubdivideSelectedTriangles(new List<int> { triangleIndex });
        BuildInfluenceMapping();
    }

    public void HandleCollisionPoints(List<Vector3> collisionPoints)
    {
        foreach (Vector3 point in collisionPoints)
        {
            FindAndSubdivideAffectedTriangles(transform.InverseTransformPoint(point));
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
                Debug.Log($"Subdividing {trianglesToSubdivide.Count} triangles");
            }

            SubdivideSelectedTriangles(trianglesToSubdivide);
            BuildInfluenceMapping();
        }
    }

    private bool IsPointNearTriangle(Vector3 point, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        Vector3 centroid = (v1 + v2 + v3) / 3f;
        return (point - centroid).sqrMagnitude < influenceRadius * influenceRadius;
    }

    private int GetMidpoint(int a, int b, Vector3[] verts, List<Vector3> newVerts, Dictionary<Edge, int> midpoints, bool create = true)
    {
        Edge edge = new Edge(a, b, verts);
        if (midpoints.TryGetValue(edge, out int index)) return index;

        Vector3 mid = (verts[a] + verts[b]) * 0.5f;
        newVerts.Add(mid);
        int newIndex = newVerts.Count - 1;
        midpoints.Add(edge, newIndex);

        if (create)
        {
            springFiller.AddSpringPointAtPosition(transform.TransformPoint(mid));
        }

        return newIndex;
    }

    void BuildInfluenceMapping()
    {
        //vertexInfluences = new List<WeightedInfluence>[baseVertices.Length];

        //for (int i = 0; i < baseVertices.Length; i++)
        //{
        //    vertexInfluences[i] = new List<WeightedInfluence>();
        //    Vector3 vertexWorld = transform.TransformPoint(baseVertices[i]);

        //    foreach (SpringPointData sp in springFiller.allSpringPoints)
        //    {
        //        float sqrDistance = (vertexWorld - sp.position).sqrMagnitude;
        //        if (sqrDistance < influenceRadius * influenceRadius)
        //        {
        //            vertexInfluences[i].Add(new WeightedInfluence
        //            {
        //                springPoint = sp,
        //                weight = Mathf.Exp(-sqrDistance)
        //            });
        //        }
        //    }
        //}
    }

    public void SubdivideMeshWithPoints(List<SpringPointData> newPoints)
    {
        if (newPoints == null || newPoints.Count == 0)
        {
            Debug.LogWarning("No new points supplied for subdivision.");
            return;
        }

        // Optimization 1: Precompute world vertices once
        Vector3[] worldVertices = new Vector3[currentVertices.Length];
        for (int i = 0; i < currentVertices.Length; i++)
        {
            worldVertices[i] = transform.TransformPoint(currentVertices[i]);
        }

        // Optimization 2: Create spatial index for spring points
        Bounds bounds = new Bounds(newPoints[0].position, Vector3.zero);
        foreach (var point in newPoints) bounds.Encapsulate(point.position);

        float cellSize = Mathf.Max(0.1f, influenceRadius * 0.5f);
        SpatialGrid<SpringPointData> springGrid = new SpatialGrid<SpringPointData>(bounds, cellSize);
        foreach (var point in newPoints) springGrid.Add(point.position, point);

        List<int> trianglesToSubdivide = new List<int>();
        float sqrInfluence = influenceRadius * influenceRadius;

        // Optimization 3: Process triangles in parallel
        Parallel.For(0, baseTriangles.Length / 3, i =>
        {
            int triangleIndex = i;
            if (triangleIndex >= triangleDataList.Count) return;

            TriangleData data = triangleDataList[triangleIndex];
            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel)
                return;

            int idx0 = baseTriangles[i * 3];
            int idx1 = baseTriangles[i * 3 + 1];
            int idx2 = baseTriangles[i * 3 + 2];

            Vector3 centroid = (worldVertices[idx0] + worldVertices[idx1] + worldVertices[idx2]) / 3f;

            // Optimization 4: Use spatial query instead of brute-force
            int springPointCount = 0;
            var nearbyPoints = springGrid.Query(centroid, influenceRadius);

            foreach (var pointData in nearbyPoints)
            {
                Vector3 pointPos = pointData.position; // Implicit conversion from float3 to Vector3
                if ((pointPos - centroid).sqrMagnitude < sqrInfluence)
                {
                    if (++springPointCount > 3) break;
                }
            }

            if (springPointCount > 3)
            {
                lock (trianglesToSubdivide)
                {
                    trianglesToSubdivide.Add(triangleIndex);
                }
            }
        });

        if (trianglesToSubdivide.Count > 0)
        {
            SubdivideSelectedTriangles(trianglesToSubdivide, false);
            BuildInfluenceMapping();
        }
    }

    // Spatial partitioning grid implementation
    public class SpatialGrid<T>
    {
        private readonly Dictionary<Vector3Int, List<T>> grid = new Dictionary<Vector3Int, List<T>>();
        private readonly Bounds bounds;
        private readonly float cellSize;

        public SpatialGrid(Bounds bounds, float cellSize)
        {
            this.bounds = bounds;
            this.cellSize = cellSize;
        }

        public void Add(Vector3 position, T item)
        {
            Vector3Int cell = WorldToGrid(position);
            if (!grid.TryGetValue(cell, out var list))
            {
                list = new List<T>();
                grid[cell] = list;
            }
            list.Add(item);
        }

        public IEnumerable<T> Query(Vector3 position, float radius)
        {
            int cells = Mathf.CeilToInt(radius / cellSize);
            Vector3Int centerCell = WorldToGrid(position);

            for (int x = -cells; x <= cells; x++)
            {
                for (int y = -cells; y <= cells; y++)
                {
                    for (int z = -cells; z <= cells; z++)
                    {
                        Vector3Int cell = new Vector3Int(
                            centerCell.x + x,
                            centerCell.y + y,
                            centerCell.z + z
                        );

                        if (grid.TryGetValue(cell, out var list))
                        {
                            foreach (var item in list)
                            {
                                yield return item;
                            }
                        }
                    }
                }
            }
        }

        private Vector3Int WorldToGrid(Vector3 position)
        {
            return new Vector3Int(
                Mathf.FloorToInt((position.x - bounds.min.x) / cellSize),
                Mathf.FloorToInt((position.y - bounds.min.y) / cellSize),
                Mathf.FloorToInt((position.z - bounds.min.z) / cellSize)
            );
        }
    }

    private void SubdivideSelectedTriangles(List<int> triangleIndices, bool create = true)
    {
        Vector3[] oldVertices = workingMesh.vertices;
        int[] oldTriangles = workingMesh.triangles;
        List<Vector3> newVertices = new List<Vector3>(oldVertices);
        Dictionary<Edge, int> edgeMidpoints = new Dictionary<Edge, int>();

        // First pass: find all triangles to subdivide and their midpoints
        foreach (int triIdx in triangleIndices)
        {
            if (triIdx >= triangleDataList.Count) continue;

            TriangleData data = triangleDataList[triIdx];
            if (!data.canSubdivide || data.subdivisionLevel >= maxSubdivisionLevel)
                continue;

            int i0 = oldTriangles[triIdx * 3];
            int i1 = oldTriangles[triIdx * 3 + 1];
            int i2 = oldTriangles[triIdx * 3 + 2];

            GetMidpoint(i0, i1, oldVertices, newVertices, edgeMidpoints, create);
            GetMidpoint(i1, i2, oldVertices, newVertices, edgeMidpoints, create);
            GetMidpoint(i2, i0, oldVertices, newVertices, edgeMidpoints, create);
        }

        // Second pass: rebuild triangles
        List<int> newTriangles = new List<int>(oldTriangles.Length * 4); // Pre-allocate
        List<TriangleData> newTriangleData = new List<TriangleData>();

        for (int i = 0; i < oldTriangles.Length; i += 3)
        {
            int originalTriIdx = i / 3;
            TriangleData originalData = triangleDataList[originalTriIdx];

            int i0 = oldTriangles[i];
            int i1 = oldTriangles[i + 1];
            int i2 = oldTriangles[i + 2];

            bool has_m01 = edgeMidpoints.TryGetValue(new Edge(i0, i1, oldVertices), out int m01);
            bool has_m12 = edgeMidpoints.TryGetValue(new Edge(i1, i2, oldVertices), out int m12);
            bool has_m20 = edgeMidpoints.TryGetValue(new Edge(i2, i0, oldVertices), out int m20);

            int splitEdgeCount = (has_m01 ? 1 : 0) + (has_m12 ? 1 : 0) + (has_m20 ? 1 : 0);

            switch (splitEdgeCount)
            {
                case 3: // Full subdivision
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
                            subdivisionLevel = (byte)newLevel,
                            canSubdivide = newLevel < maxSubdivisionLevel
                        });
                    }
                    break;

                case 2: // Two edges split
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
                        newTriangleData.Add(originalData);
                    }
                    break;

                case 1: // One edge split
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
                        newTriangleData.Add(originalData);
                    }
                    break;

                default: // No subdivision
                    newTriangles.AddRange(new[] { i0, i1, i2 });
                    newTriangleData.Add(originalData);
                    break;
            }
        }

        // Apply changes to mesh
        workingMesh.Clear();
        workingMesh.vertices = newVertices.ToArray();
        workingMesh.triangles = newTriangles.ToArray();
        workingMesh.RecalculateNormals();
        workingMesh.RecalculateBounds();

        // Update references
        triangleDataList = newTriangleData;
        baseVertices = workingMesh.vertices;
        currentVertices = (Vector3[])baseVertices.Clone();
        baseTriangles = workingMesh.triangles;
    }

    public void NotifyMeshChanged()
    {
        // Placeholder for mesh change notifications
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
}