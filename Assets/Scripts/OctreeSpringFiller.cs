using NUnit.Framework.Internal;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

public class OctreeSpringFiller : MonoBehaviour
{
    [Header("Filling Settings")]
    public float minNodeSize = 0.5f;
    public float PointSpacing = 0.5f;
    public bool isFilled = true;
    public bool isRigid = true;

    public int maxConnectionsPerPoint = 18;

    [Header("Spring Settings")]
    [Header("Spring Settings / Layer 1")]
    public float springConstantL1 = 100f;
    public float damperConstantL1 = 0.6f;
    public float connectionRadiusL1 = 1f;
    public float maxRestLengthL1 = 2f;
    [Header("Spring Settings / Layer 2")]
    public float springConstantL2 = 60f;
    public float damperConstantL2 = 0.5f;
    public float connectionRadiusL2 = 2f;
    public float maxRestLengthL2 = 2.5f;
    [Header("Spring Settings / Layer 3")]
    public float springConstantL3 = 40f;
    public float damperConstantL3 = 0.4f;
    public float connectionRadiusL3 = 3f;
    public float maxRestLengthL3 = 3f;


    [Header("Mesh Settings")]
    public float totalMass = 100f;

    public float bounciness = 0.5f;
    public float friction = 0.8f;

    private Mesh targetMesh;
    private Bounds meshBounds;
    private Vector3[] meshVertices;
    private int[] meshTriangles;
    private Vector3 lastPos;

    [Header("Gravity Settings")]
    public bool applyGravity = true;
    public Vector3 gravity => new Vector3(0, -9.81f, 0);

    [Header("Collision Settings")]
    public float groundLevel = 0f;       // Y-position of the ground plane
    public float groundBounce = 0.5f;   // Bounce coefficient (0 = no bounce, 1 = full bounce)
    public float groundFriction = 0.8f; // Friction coefficient (0 = full stop, 1 = no friction)
    public bool applyGroundCollision = true;

    [Header("Visualize Settings")]
    public bool visualizeSpringPoints = true;
    public bool visualizeSpringConnections = true;
    private VisualizeRenderer visualizeRenderer;

    // Lists
    private List<Vector3> allPointPositions = new List<Vector3>();

    private NativeList<SpringPointData> tempPoints = new NativeList<SpringPointData>(Allocator.Persistent);
    private NativeList<SpringConnectionData> tempConnections = new NativeList<SpringConnectionData>(Allocator.Persistent);

    public NativeArray<SpringPointData> allSpringPoints;
    private NativeArray<SpringConnectionData> allSpringConnections;

    // Jobs
    private CollisionJobManager collisionJobManager;
    private SpringJobManager springJobManager;
    private RigidJobManager rigidJobManager;
    private MeshJobManagerGPU meshJobManager;


    [Header("External Systems")]
    public CollisionManager collisionManager;

    // NEW: Surface point tracking
    private NativeList<SpringPointData> surfaceSpringPoints2;
    private NativeList<float3> surfacePointsLocalSpace2;
    private JobHandle meshJobHandle;


    private List<SpringPointData> surfaceSpringPoints = new List<SpringPointData>();
    public List<Vector3> convexHullVertices;
    public List<Vector3> surfacePointsLocalSpace;
    Dictionary<int, int> surfacePointToVertexIndex = new Dictionary<int, int>();
    private int originalVertexCount;
    // NEW: Surface integration settings
    [Header("Surface Integration")]
    public float surfaceDetectionThreshold = 0.2f;
    public bool enableMeshSubdivision = true;
    public bool autoUpdateMeshFromSurface = true;

    public bool applyVelocity = false;

    public Bounds boundingVolume;
    public List<SpringPointData> SurfacePoints => surfaceSpringPoints;


    private void Awake()
    {
        //Application.targetFrameRate = 60;
        Time.fixedDeltaTime = 1f / 30f;

        visualizeRenderer = new VisualizeRenderer();
        visualizeRenderer.CreatePointMeshAndMaterial();
        visualizeRenderer.CreateConnectionMaterial();
    }

    void Start()
    {
        // save transform
        lastPos = transform.position;

        // Get mesh
        targetMesh = GetComponent<MeshFilter>().mesh;
        targetMesh.MarkDynamic();   // mesh will be updated frequently at runtime.

        meshBounds = targetMesh.bounds;
        meshVertices = targetMesh.vertices;
        meshTriangles = targetMesh.triangles;

        // NEW: Store original vertex count
        originalVertexCount = meshVertices.Length;

        // Initialize from MaterialManager if exists
        var materialManager = GetComponent<MaterialManager>();
        if (materialManager != null)
        {
            var preset = materialManager.GetMaterialProperties();
            if (preset != null)
            {
                // Apply preset properties
                springConstantL1 = preset.springConstantL1;
                damperConstantL1 = preset.damperConstantL1;
                connectionRadiusL1 = preset.connectionRadiusL1;
                maxRestLengthL1 = preset.maxRestLengthL1;
                springConstantL2 = preset.springConstantL2;
                damperConstantL2 = preset.damperConstantL2;
                connectionRadiusL2 = preset.connectionRadiusL2;
                maxRestLengthL2 = preset.maxRestLengthL2;
                springConstantL3 = preset.springConstantL3;
                damperConstantL3 = preset.damperConstantL3;
                connectionRadiusL3 = preset.connectionRadiusL3;
                maxRestLengthL3 = preset.maxRestLengthL3;
                isRigid = preset.isRigid;
                totalMass = preset.totalMass;
                bounciness = preset.bounciness;
                friction = preset.friction;
            }
        }

        FillObjectWithSpringPoints();

        //RebuildSurfaceRepresentation();
        //// NEW: After filling, identify surface points and subdivide mesh
        //if (enableMeshSubdivision)
        //{
        //    SubdivideMeshWithSurfacePoints();
        //}

        // Update positions and bounds on start
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            SpringPointData point = allSpringPoints[i]; // get copy
            point.mass = totalMass / allSpringPoints.Length;
            allSpringPoints[i] = point; // write back modified copy

            //    Vector3 moveStep = transform.position - lastPos;
            //    point.UpdateBounds(moveStep);
        }


        // Use Jobs to calculate physics on GPU threads
        // Parallelizing calculations improves performance

        // Mesh
        surfaceSpringPoints2 = new NativeList<SpringPointData>(allSpringPoints.Length, Allocator.Persistent);
        surfacePointsLocalSpace2 = new NativeList<float3>(allSpringPoints.Length, Allocator.Persistent);

        meshJobManager = gameObject.AddComponent<MeshJobManagerGPU>();
        meshJobManager.Initialize(meshVertices, allSpringPoints, surfaceSpringPoints2, surfacePointsLocalSpace2);

        // Spring
        springJobManager = gameObject.AddComponent<SpringJobManager>();
        springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        // Rigid
        rigidJobManager = gameObject.AddComponent<RigidJobManager>();
        rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        // Collision
        collisionJobManager = gameObject.AddComponent<CollisionJobManager>();
        collisionJobManager.InitializeArrays(allSpringPoints);


        //meshJobManager.ScheduleSurfacePointsJobs(meshVertices, transform.worldToLocalMatrix);
        //meshJobManager.CompleteAllJobsAndApply();

        for (int i = 0; i < surfaceSpringPoints2.Length; i++)
        {
            SpringPointData point = surfaceSpringPoints2[i];
            surfaceSpringPoints.Add(point);
        }

        for (int i = 0; i < surfacePointsLocalSpace2.Length; i++)
        {
            float3 point = surfacePointsLocalSpace2[i];
            surfacePointsLocalSpace.Add(new Vector3(point.x, point.y, point.z));
        }


        RebuildSurfaceRepresentation();
        // NEW: After filling, identify surface points and subdivide mesh
        if (enableMeshSubdivision)
        {
            SubdivideMeshWithSurfacePoints();

            //(Vector3[] finalVertices, int[] finalTriangles) = meshJobManager.ApplyMeshSubdivisionJobs(meshTriangles, transform.worldToLocalMatrix);
            //UpdateMeshGeometry(finalVertices, finalTriangles);
        }

        if (collisionManager == null)
        {
            collisionManager = FindObjectOfType<CollisionManager>();
            if (collisionManager == null)
            {
                Debug.LogWarning($"{gameObject.name}: No CollisionManager found in scene!");
            }
        }
    }

    //private void Update()
    //{
    //    if (Time.frameCount % 5 == 0)
    //    {
    //        // Run every 5th frame
    //        // Spread out expensive operations
    //        foreach (SpringPoint point in allSpringPoints)
    //        {
    //            if (transform.position != lastPos)
    //            {
    //                Vector3 moveStep = transform.position - lastPos;
    //                if (moveStep.magnitude > 0.001f)
    //                {
    //                    point.UpdateBounds(moveStep);
    //                }
    //            }
    //        }
    //        if (transform.position != lastPos)
    //        {
    //            lastPos = transform.position;
    //        }
    //    }
    //}

    void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;

        if (isRigid)
        {
            // ----- RIGID MODE -----
            // 1. Schedule gravity job
            rigidJobManager.ScheduleGravityJobs(gravity, applyGravity);

            // 2. Schedule spring jobs
            rigidJobManager.ScheduleRigidJobs(10, 0.5f, deltaTime);

            // 3. Complete all jobs and apply results
            rigidJobManager.CompleteAllJobsAndApply();

            //// 1. Initialize predicted positions
            //foreach (var point in allSpringPoints)
            //{
            //    point.predictedPosition = point.position;
            //    point.force = Vector3.zero; // Clear forces
            //}

            //// 2. Apply gravity and other forces directly (skip jobs for rigid mode)
            //if (applyGravity)
            //{
            //    foreach (var point in allSpringPoints)
            //    {
            //        if (!point.isFixed)
            //            point.force += gravity * point.mass;
            //    }
            //}

            //// 3. Apply spring forces as rigid constraints (not as forces)
            //// (We don't use ScheduleSpringOrRigidJobs in rigid mode)

            //// 4. Integrate forces to get predicted positions
            //foreach (var point in allSpringPoints)
            //{
            //    if (!point.isFixed)
            //    {
            //        Vector3 acceleration = point.force / point.mass;
            //        point.velocity += acceleration * deltaTime;
            //        point.predictedPosition += point.velocity * deltaTime;
            //    }
            //}

            //// 5. Solve constraints (multiple iterations)
            //for (int i = 0; i < 5; i++)  // Try 3-10 iterations
            //{
            //    foreach (var connection in allSpringConnections)
            //    {
            //        connection.EnforceRigidConstraint();
            //    }
            //}

            //// 6. Update velocities and positions
            //foreach (var point in allSpringPoints)
            //{
            //    if (!point.isFixed)
            //    {
            //        point.velocity = (point.predictedPosition - point.position) / deltaTime;
            //        point.position = point.predictedPosition;
            //    }
            //}
        }
        else
        {
            // ----- SOFT BODY MODE -----
            // 1. Schedule gravity job
            springJobManager.ScheduleGravityJobs(gravity, applyGravity);

            // 2. Schedule spring jobs
            springJobManager.ScheduleSpringJobs(deltaTime);

            // 3. Complete all jobs and apply results
            springJobManager.CompleteAllJobsAndApply();

            // This next section was moved to Jobs

            //// Update springs
            //foreach (var connection in allSpringConnections)
            //{
            //    connection.CalculateAndApplyForces();
            //}
            //// Update points normally
            //foreach (var point in allSpringPoints)
            //{
            //    point.UpdatePoint(deltaTime);
            //}
        }

        // ----- COMMON OPERATIONS -----
        // Handle collisions
        if (applyGroundCollision)
        {
            collisionJobManager.ScheduleGroundCollisionJobs(
                groundLevel, groundBounce, groundFriction
            );

            collisionJobManager.CompleteAllJobsAndApply();
        }

        if (collisionManager != null)
        {
            collisionManager.ResolveInterObjectCollisions();
        }
        else
        {
            // NEW DEBUG LOG: This will alert you if the manager isn't assigned.
            if (enableMeshSubdivision) Debug.LogWarning($"{gameObject.name}: CollisionManager is not assigned in the inspector!", this);
        }

        if (Time.frameCount % 3 == 0) // Every 3 frames
        {
            UpdateSurfacePointsInMesh();
        }

        // Handle Mesh Update
        UpdateMeshFromPoints();
        //meshJobManager.DispatchMeshUpdate(meshVertices, transform.worldToLocalMatrix, targetMesh);
        //meshJobManager.ScheduleMeshUpdateJobs(
        //    meshVertices, transform.localToWorldMatrix, transform.worldToLocalMatrix
        //);

        //meshJobManager.CompleteAllJobsAndApply(targetMesh);
    }

    void LateUpdate()
    {
        visualizeRenderer.DrawInstancedPoints(visualizeSpringPoints, allSpringPoints);

        visualizeRenderer.UploadConnectionsToGPU(allSpringPoints, allSpringConnections);
        visualizeRenderer.DrawInstancedConnections(visualizeSpringConnections,
            transform.position, allSpringPoints, allSpringConnections
        );
    }

    private void OnEnable()
    {
        if (!CollisionManager.AllSoftBodies.Contains(this))
        {
            CollisionManager.AllSoftBodies.Add(this);
            // NEW DEBUG LOG
            Debug.Log($"{gameObject.name} added to CollisionManager. Total bodies: {CollisionManager.AllSoftBodies.Count}", this);
        }
    }

    private void OnDisable()
    {
        if (CollisionManager.AllSoftBodies.Contains(this))
        {
            CollisionManager.AllSoftBodies.Remove(this);
            // NEW DEBUG LOG
            Debug.Log($"{gameObject.name} removed from CollisionManager. Total bodies: {CollisionManager.AllSoftBodies.Count}", this);
        }
    }

    public void FillObjectWithSpringPoints()
    {
        // NEW: Clear surface point data
        surfaceSpringPoints.Clear();
        surfacePointToVertexIndex.Clear();

        // Recalculate accurate world-space bounds
        if (meshVertices.Length <= 0)
        {
            Debug.Log("Vertices Error");
            return;
        }

        // Abort if complexity too high
        float complexityEstimate = meshBounds.size.magnitude / PointSpacing;
        if (complexityEstimate > 100000)
        {
            Debug.LogError($"Aborted: Excessive complexity ({complexityEstimate})");
            return;
        }

        // TransformPoint converts the local mesh vertice dependent on the transform
        // position, scale and orientation into a global position
        Vector3 min = transform.TransformPoint(meshVertices[0]);
        Vector3 max = min;

        // Iterate through all vertices
        // except first one
        for (var i = 1; i < meshVertices.Length; i++)
        {
            var V = transform.TransformPoint(meshVertices[i]);

            // Go through X,Y and Z of the Vector3
            for (var n = 0; n < 3; n++)
            {
                max = Vector3.Max(V, max);
                min = Vector3.Min(V, min);
            }
        }

        Bounds worldBounds = new Bounds();
        worldBounds.SetMinMax(min, max);

        // Fill Object using Octree Algorithms
        OctreeNode rootNode = new OctreeNode(worldBounds, meshBounds);
        int total_nodes = BuildOctree(rootNode);
        CreateSpringConnections();

        // Copy to native arrays
        allSpringPoints = tempPoints.AsArray();
        allSpringConnections = tempConnections.AsArray();

        if (applyVelocity)
            for (int i = 0; i < allSpringPoints.Length; i++)
            {
                SpringPointData p = allSpringPoints[i];
                p.velocity = new Vector3(2f, 0, 0);
                allSpringPoints[i] = p;
            }

        // Some logs
        Debug.Log($"Octree Nodes: {total_nodes}");
        Debug.Log($"Created {allSpringPoints.Length} spring points.");
        Debug.Log($"Created {allSpringConnections.Length} spring connections.");
    }

    int BuildOctree(OctreeNode node)
    {
        int total_nodes = 0;

        if (node.isDivided || node.Divide(minNodeSize))
        {
            total_nodes += node.children.Length;
            foreach (var child in node.children)
            {
                int childCount = BuildOctree(child);
                if (childCount > 0)
                {
                    total_nodes += childCount;
                    total_nodes -= 1;
                }
            }
        }
        else
        {
            if (NodeIntersectsMesh(node))
            {
                FillNodeVertices(node);
            }
            else
            {
                if (isFilled)
                {
                    FillNodeWithSpringPoints(node);
                }
            }
        }

        return total_nodes;
    }

    bool NodeIntersectsMesh(OctreeNode node)
    {
        Bounds localBounds = node.localBounds;

        foreach (var vertex in meshVertices)
        {
            if (localBounds.Contains(vertex))
            {
                return true;
            }
        }

        return false;
    }

    void FillNodeVertices(OctreeNode node)
    {
        Bounds localBounds = node.localBounds;

        foreach (var vertex in meshVertices)
        {
            if (localBounds.Contains(vertex))
            {
                Vector3 worldPos = transform.TransformPoint(vertex);

                // Use approximate comparison instead of exact Contains
                bool alreadyExists = allPointPositions.Any(p =>
                    Vector3.Distance(p, worldPos) < PointSpacing * 0.5f);

                if (!alreadyExists)
                {
                    if (IsPointInsideMesh(worldPos))
                    {
                        allPointPositions.Add(worldPos);
                        CreateSpringPoint(worldPos, node.worldBounds, true);
                    }
                }
            }
        }
    }

    void FillNodeWithSpringPoints(OctreeNode node)
    {
        Bounds localBounds = node.localBounds;
        int stepsX = Mathf.Max(2, Mathf.FloorToInt(localBounds.size.x / PointSpacing));
        int stepsY = Mathf.Max(2, Mathf.FloorToInt(localBounds.size.y / PointSpacing));
        int stepsZ = Mathf.Max(2, Mathf.FloorToInt(localBounds.size.z / PointSpacing));

        for (int x = 0; x < stepsX; x++)
        {
            for (int y = 0; y < stepsY; y++)
            {
                for (int z = 0; z < stepsZ; z++)
                {
                    // Calculate normalized position in grid (0-1 range)
                    Vector3 normalizedPos = new Vector3(
                        stepsX > 1 ? x / (float)(stepsX - 1) : 0.5f,
                        stepsY > 1 ? y / (float)(stepsY - 1) : 0.5f,
                        stepsZ > 1 ? z / (float)(stepsZ - 1) : 0.5f
                    );

                    // Calculate position in local space (within bounds)
                    Vector3 localPos = new Vector3(
                        Mathf.Lerp(localBounds.min.x, localBounds.max.x, normalizedPos.x),
                        Mathf.Lerp(localBounds.min.y, localBounds.max.y, normalizedPos.y),
                        Mathf.Lerp(localBounds.min.z, localBounds.max.z, normalizedPos.z)
                    );

                    // Convert to world space
                    Vector3 worldPos = transform.TransformPoint(localPos);

                    // Use approximate comparison instead of exact Contains
                    bool alreadyExists = allPointPositions.Any(p =>
                        Vector3.Distance(p, worldPos) < PointSpacing * 0.5f);

                    if (!alreadyExists)
                    {
                        if (IsPointInsideMesh(worldPos))
                        {
                            allPointPositions.Add(worldPos);
                            CreateSpringPoint(worldPos, node.worldBounds, false);
                        }
                    }
                }
            }
        }
    }

    public SpringPointData CreateSpringPoint(Vector3 worldPos, Bounds bounds, bool isMeshVertex)
    {
        float3 center = (float3)bounds.center;
        float3 extents = (float3)bounds.extents;

        SpringPointData point = new SpringPointData(
            position: worldPos,
            velocity: new float3(0, 0, 0),
            mass: 1.0f,
            isFixed: 0,
            bounciness: bounciness,
            friction: friction,
            boundsMin: center - extents,
            boundsMax: center + extents,
            triangleIndex: -1,
            isMeshVertex: isMeshVertex ? 1 : 0
        );

        tempPoints.Add(point);
        return point;
    }

    // Before: O(n^2) complexity - 1,000 points = 500,000 distance checks (two for loops)
    // After: O(n * 27 * k) complexity - 1,000 points, approx: 27,000 checks(with k = 1)
    int3 GetCell(float3 position, float size)
    {
        return new int3(
            (int)math.floor(position.x / size),
            (int)math.floor(position.y / size),
            (int)math.floor(position.z / size)
        );
    }

    ulong HashPair(int a, int b)
    {
        uint min = (uint)math.min(a, b);
        uint max = (uint)math.max(a, b);
        return ((ulong)min << 32) | max;
    }

    void CreateSpringConnections()
    {
        tempConnections.Clear();
        if (tempPoints.Length == 0) return;

        // Calculate maximum connection radius
        float maxRadius = math.max(math.max(connectionRadiusL3, connectionRadiusL2), connectionRadiusL1) * PointSpacing;
        if (maxRadius <= 0) return;

        // Initialize spatial grid
        float cellSize = maxRadius;
        Dictionary<int3, List<int>> grid = new Dictionary<int3, List<int>>();

        var connectedPairs = new HashSet<ulong>();

        // Assign points to grid cells
        for (int i = 0; i < tempPoints.Length; i++)
        {
            int3 cell = GetCell(tempPoints[i].position, cellSize);
            if (!grid.TryGetValue(cell, out List<int> list))
            {
                list = new List<int>();
                grid.Add(cell, list);
            }
            list.Add(i);
        }

        // Create connections using spatial queries
        for (int i = 0; i < tempPoints.Length; i++)
        {
            float3 posA = tempPoints[i].position;
            int3 cell = GetCell(posA, cellSize);

            List<(int, float)> candidates = new List<(int, float)>();
            // Check 3x3x3 neighbor cells
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int z = -1; z <= 1; z++)
                    {
                        int3 neighborCell = cell + new int3(x, y, z);
                        if (!grid.TryGetValue(neighborCell, out List<int> points))
                            continue;

                        foreach (int j in points)
                        {
                            // Ensure each pair is only processed once (i < j)
                            if (j <= i || IsConnected(i, j)) continue;

                            ulong pair = HashPair(i, j);
                            if (connectedPairs.Contains(pair)) continue;

                            float3 posB = tempPoints[j].position;
                            float dist = math.distance(posA, posB);
                            if (dist <= maxRadius)
                                candidates.Add((j, dist));
                        }
                    }
                }
            }

            // Sort by distance, prioritize closest connections
            candidates.Sort((a, b) => a.Item2.CompareTo(b.Item2));

            int added = 0;
            foreach (var (j, dist) in candidates)
            {
                if (added >= maxConnectionsPerPoint) break;

                float threshold1 = connectionRadiusL1 * PointSpacing;
                float threshold2 = connectionRadiusL2 * PointSpacing;
                float threshold3 = connectionRadiusL3 * PointSpacing;

                if (dist <= threshold1)
                {
                    AddConnection(i, j, dist, maxRestLengthL1, springConstantL1, damperConstantL1);
                }
                else if (dist <= threshold2)
                {
                    AddConnection(i, j, dist, maxRestLengthL2, springConstantL2, damperConstantL2);
                }
                else if (dist <= threshold3)
                {
                    AddConnection(i, j, dist, maxRestLengthL3, springConstantL3, damperConstantL3);
                }
                else
                {
                    continue;
                }

                connectedPairs.Add(HashPair(i, j));
                added++;
            }
        }
    }

    void CreateSpringConnectionsForPoint(int pointIndex, SpringPointData pointData)
    {
        if (tempPoints.Length == 0) return;

        // Calculate maximum connection radius
        float maxRadius = math.max(math.max(connectionRadiusL3, connectionRadiusL2), connectionRadiusL1) * PointSpacing;
        if (maxRadius <= 0) return;

        // Initialize spatial grid
        float cellSize = maxRadius;
        Dictionary<int3, List<int>> grid = new Dictionary<int3, List<int>>();

        var connectedPairs = new HashSet<ulong>();

        // Assign points to grid cells
        for (int i = 0; i < tempPoints.Length; i++)
        {
            int3 cell = GetCell(tempPoints[i].position, cellSize);
            if (!grid.TryGetValue(cell, out List<int> list))
            {
                list = new List<int>();
                grid.Add(cell, list);
            }
            list.Add(i);
        }

        // Get the position of our target point
        float3 posA = pointData.position;
        int3 wantedCell = GetCell(posA, cellSize);

        List<(int, float)> candidates = new List<(int, float)>();

        // Check 3x3x3 neighbor cells (assuming grid is accessible)
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                for (int z = -1; z <= 1; z++)
                {
                    int3 neighborCell = wantedCell + new int3(x, y, z);
                    if (!grid.TryGetValue(neighborCell, out List<int> points))
                        continue;

                    foreach (int j in points)
                    {
                        // Skip self and already connected points
                        if (j == pointIndex || IsConnected(pointIndex, j)) continue;

                        float3 posB = tempPoints[j].position;
                        float dist = math.distance(posA, posB);
                        if (dist <= maxRadius)
                            candidates.Add((j, dist));
                    }
                }
            }
        }

        // Sort by distance, prioritize closest connections
        candidates.Sort((a, b) => a.Item2.CompareTo(b.Item2));

        int added = 0;
        foreach (var (j, dist) in candidates)
        {
            if (added >= maxConnectionsPerPoint) break;

            float threshold1 = connectionRadiusL1 * PointSpacing;
            float threshold2 = connectionRadiusL2 * PointSpacing;
            float threshold3 = connectionRadiusL3 * PointSpacing;

            if (dist <= threshold1)
            {
                AddConnection(pointIndex, j, dist, maxRestLengthL1, springConstantL1, damperConstantL1);
            }
            else if (dist <= threshold2)
            {
                AddConnection(pointIndex, j, dist, maxRestLengthL2, springConstantL2, damperConstantL2);
            }
            else if (dist <= threshold3)
            {
                AddConnection(pointIndex, j, dist, maxRestLengthL3, springConstantL3, damperConstantL3);
            }
            else
            {
                continue;
            }

            added++;
        }
    }

    void AddConnection(int i, int j, float distance, float maxRestLength, float springConst, float damperConst)
    {
        float restLength = math.clamp(distance, 0.5f, maxRestLength);
        tempConnections.Add(new SpringConnectionData(i, j, restLength, springConst, damperConst));
    }

    bool IsConnected(int point1Index, int point2Index)
    {
        foreach (var conn in tempConnections)
        {
            if ((conn.pointA == point1Index && conn.pointB == point2Index) ||
                (conn.pointA == point2Index && conn.pointB == point1Index))
                return true;
        }
        return false;
    }

    // Check if SpringPoint is in Mesh
    bool IsPointInsideMesh(Vector3 point)
    {
        // Transform point to mesh's local space
        Vector3 localPoint = transform.InverseTransformPoint(point);

        // 1. Fast bounding box check - if outside, definitely outside mesh
        if (!meshBounds.Contains(localPoint))
            return false;

        // 2. Use multiple ray directions to increase reliability
        Vector3[] baseDirections = {
            Vector3.left,
            Vector3.right,
            Vector3.forward,
            Vector3.back,
            Vector3.up,
            Vector3.down
        };

        float originOffset = 1e-9f; // very Small offset
        float jitterAmount = 1e-6f; // Small angle jitter
        int len = baseDirections.Length;
        Vector3[] testDirections = new Vector3[len * 2];
        for (int i = 0; i < len; i++)
        {
            // Create small random jitter vector
            Vector3 jitter_negative = new Vector3(-jitterAmount, -jitterAmount, -jitterAmount);
            Vector3 jitter_positive = new Vector3(jitterAmount, jitterAmount, jitterAmount);

            // Add jitter and normalize to keep direction unit length
            // reduces the edges or vertices where the ray barely grazes the mesh
            testDirections[i] = (baseDirections[i] + jitter_negative).normalized;
            testDirections[len + i] = (baseDirections[i] + jitter_positive).normalized;
        }

        // If *any* direction ray test says point is inside (odd intersections), we say inside
        foreach (Vector3 direction in testDirections)
        {
            // Nudge the ray origin a bit forward, to avoid self-intersections
            Vector3 rayOrigin = localPoint + direction * originOffset;

            // Check intersections
            int intersections = CountRayIntersections(rayOrigin, direction);
            if (intersections % 2 == 1) // odd = inside
                return true; // point is inside mesh
        }

        return false; // All tests say outside
    }

    int CountRayIntersections(Vector3 origin, Vector3 direction)
    {
        int count = 0;

        for (int i = 0; i < meshTriangles.Length; i += 3)
        {
            Vector3 v1 = meshVertices[meshTriangles[i]];
            Vector3 v2 = meshVertices[meshTriangles[i + 1]];
            Vector3 v3 = meshVertices[meshTriangles[i + 2]];

            if (RayTriangleIntersection(origin, direction, v1, v2, v3))
                count++;
        }

        return count;
    }

    bool RayTriangleIntersection(Vector3 origin, Vector3 direction, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        Vector3 e1 = v2 - v1;
        Vector3 e2 = v3 - v1;
        Vector3 p = Vector3.Cross(direction, e2);
        float det = Vector3.Dot(e1, p);

        float epsilon = 1e-3f;
        // If determinant is near zero, ray is parallel
        if (Mathf.Abs(det) < epsilon) // Ray parallel to triangle plane
            return false;

        float invDet = 1.0f / det;
        Vector3 t = origin - v1;
        float u = Vector3.Dot(t, p) * invDet;
        if (u < 0f || u > 1f)
            return false;

        Vector3 q = Vector3.Cross(t, e1);
        float v = Vector3.Dot(direction, q) * invDet;
        if (v < 0f || u + v > 1f)
            return false;

        float dist = Vector3.Dot(e2, q) * invDet;
        return dist >= -epsilon;
    }
    //  

    // Debug
    private void DrawDebugConnections()
    {
        if (!visualizeSpringConnections) return;

        foreach (var conn in allSpringConnections)
        {

            Debug.DrawLine(allSpringPoints[conn.pointA].position, allSpringPoints[conn.pointB].position, Color.white);
        }
    }

    private void OnDestroy()
    {
        tempPoints.Clear();
        tempConnections.Clear();
        allPointPositions.Clear();

        if (meshJobManager) Destroy(meshJobManager);
        if (rigidJobManager) Destroy(rigidJobManager);
        if (springJobManager) Destroy(springJobManager);
        if (collisionJobManager) Destroy(collisionJobManager);

        if (tempPoints.IsCreated) tempPoints.Dispose();
        if (tempConnections.IsCreated) tempConnections.Dispose();

        if (allSpringPoints.IsCreated) allSpringPoints.Dispose();
        if (allSpringConnections.IsCreated) allSpringConnections.Dispose();

        visualizeRenderer.Dispose();

        Resources.UnloadUnusedAssets();
        System.GC.Collect();
    }
    //



    public void AddSpringPointAtPosition(Vector3 worldPosition)
    {
        Vector3 worldCenter = transform.TransformPoint(meshBounds.center);
        Vector3 worldSize = Vector3.Scale(meshBounds.size, transform.lossyScale);
        Bounds worldBounds = new Bounds(worldCenter, worldSize);

        var newPoint = CreateSpringPoint(worldPosition, worldBounds, false);
        allSpringPoints = tempPoints.AsArray();

        int newPointIndex = tempPoints.Length - 1;
        CreateSpringConnectionsForPoint(newPointIndex, newPoint);
        allSpringConnections = tempConnections.AsArray();

        // Update positions and bounds on start
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            SpringPointData point = allSpringPoints[i]; // get copy
            point.mass = totalMass / allSpringPoints.Length;
            allSpringPoints[i] = point; // write back modified copy
        }

        UpdateMeshDataWithNewPoint(worldPosition);

        surfaceSpringPoints2 = new NativeList<SpringPointData>(allSpringPoints.Length, Allocator.Persistent);
        surfacePointsLocalSpace2 = new NativeList<float3>(allSpringPoints.Length, Allocator.Persistent);

        meshJobManager.Initialize(meshVertices, allSpringPoints, surfaceSpringPoints2, surfacePointsLocalSpace2);

        rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        collisionJobManager.InitializeArrays(allSpringPoints);

        meshJobManager.ScheduleSurfacePointsJobs(meshVertices, transform.worldToLocalMatrix);
        meshJobManager.CompleteAllJobsAndApply();
    }


    public float GetObjectRadius()
    {
        if (allSpringPoints.Length == 0) return 1f;

        Vector3 center = transform.position;
        float maxDistance = 0f;

        foreach (SpringPointData point in allSpringPoints)
        {
            float distance = Vector3.Distance(point.position, center);
            if (distance > maxDistance)
            {
                maxDistance = distance;
            }
        }

        return maxDistance;
    }


    void UpdateMeshFromPoints()
    {
        if (allSpringPoints == null || allSpringPoints.Length == 0 || meshVertices == null) return;

        Vector3[] vertices = new Vector3[meshVertices.Length];

        // --- 1. Find average position of all spring points (in world space) ---
        Vector3 averagePos = Vector3.zero;
        foreach (var point in allSpringPoints)
        {
            averagePos += (Vector3)point.position;
        }
        averagePos /= allSpringPoints.Length;

        // --- 2. Move the transform to the center of the point cloud ---
        transform.position = averagePos;

        // --- 3. For each original mesh vertex, find the closest spring point ---
        for (int i = 0; i < meshVertices.Length; i++)
        {
            Vector3 worldVertex = transform.TransformPoint(meshVertices[i]);
            SpringPointData closestPoint = FindClosestPoint(worldVertex);

            // Defensive check
            if (closestPoint.mass > 0 || closestPoint.isFixed == 0)
            {
                vertices[i] = transform.InverseTransformPoint(closestPoint.position);
            }
            else
            {
                // Fallback to keeping the original vertex
                vertices[i] = meshVertices[i];
            }
        }

        // --- 4. Update mesh vertices and recalculate bounds ---
        targetMesh.vertices = vertices;
        targetMesh.RecalculateNormals();
        targetMesh.RecalculateBounds();
    }


    SpringPointData FindClosestPoint(Vector3 worldPos)
    {
        SpringPointData closest = default;
        float minDist = float.MaxValue;

        foreach (var point in allSpringPoints)
        {
            float dist = Vector3.Distance(worldPos, point.position);
            if (dist < minDist)
            {
                minDist = dist;
                closest = point;
            }
        }

        return closest;
    }

    private void UpdateMeshDataWithNewPoint(Vector3 newWorldPosition)
    {
        Vector3 newLocalPosition = transform.InverseTransformPoint(newWorldPosition);

        Vector3[] newVertices = new Vector3[meshVertices.Length + 1];
        int[] newTriangles = new int[meshTriangles.Length + 3];

        System.Array.Copy(meshVertices, newVertices, meshVertices.Length);
        System.Array.Copy(meshTriangles, newTriangles, meshTriangles.Length);

        newVertices[meshVertices.Length] = newLocalPosition;

        int closest1 = 0;
        int closest2 = 1;
        float minDist1 = float.MaxValue;
        float minDist2 = float.MaxValue;

        for (int i = 0; i < meshVertices.Length; i++)
        {
            float dist = Vector3.Distance(newLocalPosition, meshVertices[i]);
            if (dist < minDist1)
            {
                minDist2 = minDist1;
                closest2 = closest1;
                minDist1 = dist;
                closest1 = i;
            }
            else if (dist < minDist2)
            {
                minDist2 = dist;
                closest2 = i;
            }
        }

        newTriangles[meshTriangles.Length] = closest1;
        newTriangles[meshTriangles.Length + 1] = closest2;
        newTriangles[meshTriangles.Length + 2] = meshVertices.Length;

        meshVertices = newVertices;
        meshTriangles = newTriangles;

        targetMesh.vertices = newVertices;
        targetMesh.triangles = newTriangles;
        targetMesh.RecalculateNormals();
        targetMesh.RecalculateBounds();
    }

    //public void UpdateBoundingVolume()
    //{
    //    Vector3 center = transform.position;
    //    float radius = GetObjectRadius();
    //    boundingVolume = new Bounds(center, Vector3.one * (radius * 2f));
    //}

    public void UpdateBoundingVolume()
    {
        if (allSpringPoints.Length == 0) return;

        Vector3 min = allSpringPoints[0].position;
        Vector3 max = allSpringPoints[0].position;

        foreach (SpringPointData p in allSpringPoints)
        {
            min = Vector3.Min(min, p.position);
            max = Vector3.Max(max, p.position);
        }

        boundingVolume = new Bounds((min + max) * 0.5f, max - min);
    }

    public void SubdivideMeshWithSurfacePoints()
    {
        if (surfaceSpringPoints.Count == 0)
        {
            Debug.LogWarning("No surface spring points found for subdivision");
            return;
        }
        Debug.Log($"Subdividing mesh with {surfaceSpringPoints.Count} surface points");
        List<Vector3> newVertices = new List<Vector3>(meshVertices);

        for (int i = 0; i < surfaceSpringPoints.Count; i++)
        {
            SpringPointData surfacePoint = surfaceSpringPoints[i];
            Vector3 localPos = transform.InverseTransformPoint(surfacePoint.position);
            newVertices.Add(localPos);
            int newVertexIndex = newVertices.Count - 1;
            surfacePointToVertexIndex[i] = newVertexIndex; // i = index in surfaceSpringPoints
        }
        List<int> newTriangles = new List<int>(meshTriangles);
        CreateTrianglesForSurfacePoints(newVertices, newTriangles);
        UpdateMeshGeometry(newVertices.ToArray(), newTriangles.ToArray());
        Debug.Log($"Mesh subdivision complete. Vertices: {meshVertices.Length}, Triangles: {meshTriangles.Length / 3}");
    }

    private void CreateTrianglesForSurfacePoints(List<Vector3> vertices, List<int> triangles)
    {
        foreach (var kvp in surfacePointToVertexIndex)
        {
            SpringPointData surfacePoint = allSpringPoints[kvp.Key];
            int surfaceVertexIndex = kvp.Value;
            Vector3 surfaceLocalPos = vertices[surfaceVertexIndex];
            int closestTriangleIndex = FindClosestTriangleToPoint(surfaceLocalPos);

            if (closestTriangleIndex >= 0)
            {
                int baseIndex = closestTriangleIndex * 3;
                int v1 = meshTriangles[baseIndex];
                int v2 = meshTriangles[baseIndex + 1];
                int v3 = meshTriangles[baseIndex + 2];
                triangles.AddRange(new[] { surfaceVertexIndex, v1, v2 });
                triangles.AddRange(new[] { surfaceVertexIndex, v2, v3 });
                triangles.AddRange(new[] { surfaceVertexIndex, v3, v1 });
            }
            else
            {
                List<int> nearestVertices = FindNearestVertices(surfaceLocalPos, vertices, 3);
                if (nearestVertices.Count >= 3)
                {
                    triangles.AddRange(new[] { surfaceVertexIndex, nearestVertices[0], nearestVertices[1] });
                    triangles.AddRange(new[] { surfaceVertexIndex, nearestVertices[1], nearestVertices[2] });
                    triangles.AddRange(new[] { surfaceVertexIndex, nearestVertices[2], nearestVertices[0] });
                }
            }
        }
    }

    private int FindClosestTriangleToPoint(Vector3 localPoint)
    {
        float closestDistance = float.MaxValue;
        int closestTriangle = -1;
        for (int i = 0; i < meshTriangles.Length; i += 3)
        {
            Vector3 v1 = meshVertices[meshTriangles[i]];
            Vector3 v2 = meshVertices[meshTriangles[i + 1]];
            Vector3 v3 = meshVertices[meshTriangles[i + 2]];
            Vector3 triangleCenter = (v1 + v2 + v3) / 3f;
            float distance = Vector3.Distance(localPoint, triangleCenter);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestTriangle = i / 3;
            }
        }
        return closestTriangle;
    }

    private List<int> FindNearestVertices(Vector3 position, List<Vector3> vertices, int count)
    {
        var vertexDistances = new List<(int index, float distance)>();
        for (int i = 0; i < originalVertexCount; i++)
        {
            float distance = Vector3.Distance(position, vertices[i]);
            vertexDistances.Add((i, distance));
        }
        return vertexDistances.OrderBy(x => x.distance).Take(count).Select(x => x.index).ToList();
    }

    private void UpdateMeshGeometry(Vector3[] newVertices, int[] newTriangles)
    {
        meshVertices = newVertices;
        meshTriangles = newTriangles;

        targetMesh.Clear();
        targetMesh.vertices = newVertices;
        targetMesh.triangles = newTriangles;

        targetMesh.RecalculateNormals();
        targetMesh.RecalculateBounds();
        meshBounds = targetMesh.bounds;
    }

    private void UpdateSurfacePointsInMesh()
    {
        if (!autoUpdateMeshFromSurface || surfacePointToVertexIndex.Count == 0) return;

        bool meshChanged = false;
        meshVertices = targetMesh.vertices;

        foreach (var kvp in surfacePointToVertexIndex)
        {
            int pointIndex = kvp.Key;
            int vertexIndex = kvp.Value;

            if (pointIndex < allSpringPoints.Length && vertexIndex < meshVertices.Length)
            {
                SpringPointData surfacePoint = allSpringPoints[pointIndex];
                Vector3 newLocalPos = transform.InverseTransformPoint(surfacePoint.position);
                Vector3 oldLocalPos = meshVertices[vertexIndex];
                if (Vector3.Distance(newLocalPos, oldLocalPos) > 0.001f)
                {
                    meshVertices[vertexIndex] = newLocalPos;
                    meshChanged = true;
                }
            }
        }
        if (meshChanged)
        {
            targetMesh.vertices = meshVertices;
            targetMesh.RecalculateNormals();
            targetMesh.RecalculateBounds();


            // Update SpringPointData from mesh vertex changes
            for (int i = 0; i < surfaceSpringPoints.Count; i++)
            {
                int springPointIndex = surfacePointToVertexIndex.Keys.ElementAt(i);
                int vertexIndex = surfacePointToVertexIndex.Values.ElementAt(i);

                if (springPointIndex < surfaceSpringPoints.Count && vertexIndex < meshVertices.Length)
                {
                    Vector3 worldPosition = transform.TransformPoint(meshVertices[vertexIndex]);
                    SpringPointData sp = surfaceSpringPoints[springPointIndex];
                    sp.position = worldPosition;
                    surfaceSpringPoints[springPointIndex] = sp;
                }
            }

        }
    }

    // NEW Gizmos for debugging surface points
    private void OnDrawGizmosSelected()
    {
        if (surfaceSpringPoints != null && surfaceSpringPoints.Count > 0)
        {
            Gizmos.color = Color.yellow;
            foreach (var sp in surfaceSpringPoints)
            {
                Gizmos.DrawSphere(sp.position, 0.05f);
            }
        }
    }

    // TODO:change
    public bool IsPointInside(Vector3 localPoint)
    {
        if (!meshBounds.Contains(localPoint)) return false;
        return IsPointInsideMesh(transform.TransformPoint(localPoint));
    }

    public void HandleCollisionResponse(CollisionInfo info, OctreeSpringFiller other)
    {
        for (int i = 0; i < SurfacePoints.Count; i++)
        {
            SpringPointData point = SurfacePoints[i];
            Vector3 otherLocal = other.transform.InverseTransformPoint(point.position);

            if (other.IsPointInside(otherLocal))
            {
                // Position correction
                point.position += (float3)info.Normal * (info.Depth + 0.01f);

                // Velocity response
                float velAlongNormal = Vector3.Dot(point.velocity, info.Normal);
                if (velAlongNormal < 0)
                {
                    point.velocity -= (float3)(1 + groundBounce) * velAlongNormal * info.Normal;
                    Vector3 tangentVel = point.velocity - velAlongNormal * (float3)info.Normal;
                    point.velocity = tangentVel * (1 - groundFriction);
                }
            }
            SurfacePoints[i] = point;
        }
    }

    public List<Vector3> GetSurfacePointsInCollision(OctreeSpringFiller other)
    {
        List<Vector3> points = new List<Vector3>();
        foreach (var point in SurfacePoints)
        {
            if (other.IsPointInside(other.transform.InverseTransformPoint(point.position)))
            {
                points.Add(point.position);
            }
        }
        return points;
    }


    private bool IsPointOnSurface(Vector3 worldPoint)
    {
        Vector3 localPoint = transform.InverseTransformPoint(worldPoint);
        float closestDistance = float.MaxValue;
        for (int i = 0; i < originalVertexCount; i++)
        {
            float distance = Vector3.Distance(localPoint, meshVertices[i]);
            if (distance < closestDistance)
            {
                closestDistance = distance;
            }
        }
        return closestDistance <= surfaceDetectionThreshold;
    }
    private void IdentifySurfacePoints()
    {
        surfaceSpringPoints.Clear();
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            SpringPointData point = allSpringPoints[i];
            if (IsPointOnSurface(point.position))
            {
                surfaceSpringPoints.Add(point);
            }
        }
        Debug.Log($"Found {surfaceSpringPoints.Count} surface spring points out of {allSpringPoints.Length} total points");
    }

    private void GenerateLocalSurfacePoints()
    {
        surfacePointsLocalSpace = new List<Vector3>();
        if (surfaceSpringPoints == null || surfaceSpringPoints.Count == 0)
        {
            Debug.LogWarning("No surface points found to generate GJK shape.", this);
            return;
        }
        for (int i = 0; i < surfaceSpringPoints.Count; i++)
        {
            SpringPointData sp = surfaceSpringPoints[i];
            surfacePointsLocalSpace.Add(transform.InverseTransformPoint(sp.initialPosition));
        }
    }

    // NEW Public Method to be called by MeshDeformer
    public void RebuildSurfaceRepresentation()
    {
        IdentifySurfacePoints();
        GenerateLocalSurfacePoints();
        Debug.Log($"{gameObject.name}: Surface representation rebuilt. New surface point count: {surfaceSpringPoints.Count}", this);
    }
}
