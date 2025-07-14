using System;
using System.Collections;
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
    public bool isFixCorners = false;
    public bool skipPhysicsFromMaterial = false;

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


    [SerializeField] private Vector3 roomMinBounds = new Vector3(-9.5f, 0f, -1f);
    [SerializeField] private Vector3 roomMaxBounds = new Vector3(9.5f, 10f, 9.5f);

    [SerializeField] public float roomBounce = 0.5f;
    [SerializeField] public float roomFriction = 0.8f;


    [Header("Mesh Settings")]
    public float totalMass = 100f;

    public float bounciness = 0.5f;
    public float friction = 0.8f;

    public Vector3 velocity = Vector3.zero;
    public bool applyVelocity = false;

    private Mesh targetMesh;
    private Bounds meshBounds;
    private Vector3[] meshVertices;
    private int[] meshTriangles;
    private Vector3 lastPos;

    [Header("Gravity Settings")]
    public bool applyGravity = true;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    [Header("Collision Settings")]
    public Bounds boundingVolume;

    public float groundLevel = 0f;       // Y-position of the ground plane
    public float groundBounce = 0.5f;   // Bounce coefficient (0 = no bounce, 1 = full bounce)
    public float groundFriction = 0.8f; // Friction coefficient (0 = full stop, 1 = no friction)
    public bool applyGroundCollision = true;
    public bool applyRoomCollision = true;

    [Header("External Systems")]
    public CollisionManager collisionManager;
    public MeshDeformer meshDeformer;

    [Header("Collision Layer")]
    [SerializeField] public CollisionLayer collisionLayer;

    // Layer presets for easy setup
    [Header("Layer Presets")]
    [SerializeField] private CollisionLayerPreset layerPreset = CollisionLayerPreset.Default;


    [Header("Visualize Settings")]
    public bool visualizeSpringPoints = true;
    public bool visualizeSpringConnections = true;
    private VisualizeRenderer visualizeRenderer;

    // Lists
    private List<Vector3> allPointPositions = new List<Vector3>();

    private NativeList<SpringPointData> tempPoints = new NativeList<SpringPointData>(Allocator.Persistent);
    private NativeList<SpringConnectionData> tempConnections = new NativeList<SpringConnectionData>(Allocator.Persistent);

    public NativeArray<SpringPointData> allSpringPoints;
    public NativeArray<SpringConnectionData> allSpringConnections;

    // Jobs
    private CollisionJobManager collisionJobManager;
    private SpringJobManager springJobManager;
    private RigidJobManager rigidJobManager;
    private MeshJobManagerCPU meshJobManager;

    // Surface point tracking
    public NativeList<SpringPointData> surfaceSpringPoints2;
    private NativeList<float3> surfacePointsLocalSpace;



    // Surface integration settings
    [Header("Surface Integration")]
    public bool enableMeshSubdivision = true;
    public bool autoUpdateMeshFromSurface = true;
    public float surfaceDetectionThreshold = 0.2f;


    Dictionary<int, int> surfacePointToVertexIndex = new Dictionary<int, int>();


    // changes for the new connections
    public enum ConnectionType
    {
        Structure,  // 6 cardinal directions (axis-aligned)
        Shear,      // 12 edge diagonals (45° to axes)
        Bend        // 26 face diagonals (next layer)
    }

    // Directions normalized to unit length
    private static readonly float3[] _structureDirections = {
        new float3(1, 0, 0), new float3(-1, 0, 0),
        new float3(0, 1, 0), new float3(0, -1, 0),
        new float3(0, 0, 1), new float3(0, 0, -1)
    };

    private static readonly float3[] _shearDirections = {
        new float3(1, 1, 0), new float3(1, -1, 0),
        new float3(-1, 1, 0), new float3(-1, -1, 0),
        new float3(1, 0, 1), new float3(1, 0, -1),
        new float3(-1, 0, 1), new float3(-1, 0, -1),
        new float3(0, 1, 1), new float3(0, 1, -1),
        new float3(0, -1, 1), new float3(0, -1, -1)
    };

    private static readonly float3[] _bendDirections = {
        new float3(1, 0, 0), new float3(-1, 0, 0),
        new float3(0, 1, 0), new float3(0, -1, 0),
        new float3(0, 0, 1), new float3(0, 0, -1),

        new float3(1, 1, 1),    new float3(1, 1, -1),
        new float3(1, -1, 1),   new float3(1, -1, -1),
        new float3(-1, 1, 1),   new float3(-1, 1, -1),
        new float3(-1, -1, 1),  new float3(-1, -1, -1),

        // Secondary bend directions (optional, for larger deformations)
        new float3(1, 1, 0), new float3(1, -1, 0),
        new float3(-1, 1, 0), new float3(-1, -1, 0),
        new float3(1, 0, 1), new float3(1, 0, -1),
        new float3(-1, 0, 1), new float3(-1, 0, -1),
        new float3(0, 1, 1), new float3(0, 1, -1),
        new float3(0, -1, 1), new float3(0, -1, -1)
    };

    public enum CollisionLayerPreset
    {
        Default,
        Rubber,
        Metal,
        Plastic,
        Gel,
        Custom
    }

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

        // Initialize from MaterialManager if exists
        if (!skipPhysicsFromMaterial)
        {
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
        }

        FillObjectWithSpringPoints();

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
        surfacePointsLocalSpace = new NativeList<float3>(allSpringPoints.Length, Allocator.Persistent);

        meshJobManager = gameObject.AddComponent<MeshJobManagerCPU>();
        meshJobManager.Initialize(meshVertices, meshTriangles, allSpringPoints, surfaceSpringPoints2, surfacePointsLocalSpace, surfaceDetectionThreshold);

        // Spring
        springJobManager = gameObject.AddComponent<SpringJobManager>();
        springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        // Rigid
        rigidJobManager = gameObject.AddComponent<RigidJobManager>();
        rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        // Collision
        collisionJobManager = gameObject.AddComponent<CollisionJobManager>();
        collisionJobManager.InitializeArrays(allSpringPoints);

        // Calculate surface points
        meshJobManager.IdentifySurfacePoints(
            meshVertices,
            meshTriangles,
            transform.worldToLocalMatrix
        );

        // copy data 
        //for (int i = 0; i < surfaceSpringPoints2.Length; i++)
        //{
        //    SpringPointData point = surfaceSpringPoints2[i];
        //    surfaceSpringPoints.Add(point);
        //}

        for (int i = 0; i < surfacePointsLocalSpace.Length; i++)
        {
            float3 point = surfacePointsLocalSpace[i];
        }

        // RebuildSurfaceRepresentation();
        if (meshDeformer == null)
        {
            meshDeformer = GetComponent<MeshDeformer>();
        }


        // NEW: After filling, identify surface points and subdivide mesh
        if (enableMeshSubdivision)
        {
            StartCoroutine(WaitForMeshDeformerInitialization());
        }

        if (collisionManager == null)
        {
            collisionManager = FindObjectOfType<CollisionManager>();
            if (collisionManager == null)
            {
                // Debug.LogWarning($"{gameObject.name}: No CollisionManager found in scene!");
            }
        }

        if (isFixCorners)
        {
            FixCorners();
        }

        if (applyVelocity)
            for (int i = 0; i < allSpringPoints.Length; i++)
            {
                SpringPointData p = allSpringPoints[i];
                p.velocity = velocity;
                allSpringPoints[i] = p;
            }

        if (collisionLayer == null)
        {
            InitializeCollisionLayer();
        }

        // Apply layer preset if selected
        ApplyLayerPreset();
    }


    public void OverrideSpringData(NativeArray<SpringPointData> sharedPoints, NativeArray<SpringConnectionData> sharedConnections)
    {
        allSpringPoints = new NativeArray<SpringPointData>(sharedPoints, Allocator.Persistent);
        allSpringConnections = new NativeArray<SpringConnectionData>(sharedConnections, Allocator.Persistent);

        // Reinitialize jobs and surface detection
        springJobManager = gameObject.AddComponent<SpringJobManager>();
        springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        rigidJobManager = gameObject.AddComponent<RigidJobManager>();
        rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        collisionJobManager = gameObject.AddComponent<CollisionJobManager>();
        collisionJobManager.InitializeArrays(allSpringPoints);

        // Skip FillObjectWithSpringPoints(), because data is already set
    }

    private void InitializeCollisionLayer()
    {
        collisionLayer = new CollisionLayer();
        collisionLayer.layerName = gameObject.name + "_Layer";
        collisionLayer.layerIndex = 0; // Default layer
    }

    private void ApplyLayerPreset()
    {
        switch (layerPreset)
        {
            case CollisionLayerPreset.Default:
                // Already initialized with default values
                break;

            case CollisionLayerPreset.Rubber:
                collisionLayer.density = 1200f;
                collisionLayer.restitution = 0.9f;
                collisionLayer.friction = 0.8f;
                collisionLayer.youngsModulus = 0.01e6f;
                collisionLayer.poissonRatio = 0.49f;
                break;

            case CollisionLayerPreset.Metal:
                collisionLayer.density = 7800f;
                collisionLayer.restitution = 0.3f;
                collisionLayer.friction = 0.6f;
                collisionLayer.youngsModulus = 200e9f;
                collisionLayer.poissonRatio = 0.27f;
                break;

            case CollisionLayerPreset.Plastic:
                collisionLayer.density = 1400f;
                collisionLayer.restitution = 0.5f;
                collisionLayer.friction = 0.4f;
                collisionLayer.youngsModulus = 3e9f;
                collisionLayer.poissonRatio = 0.35f;
                break;

            case CollisionLayerPreset.Gel:
                collisionLayer.density = 1000f;
                collisionLayer.restitution = 0.1f;
                collisionLayer.friction = 0.2f;
                collisionLayer.youngsModulus = 0.001e6f;
                collisionLayer.poissonRatio = 0.45f;
                collisionLayer.dampingFactor = 0.5f;
                break;
        }
    }

    // Public methods for collision layer system
    public bool CanCollideWith(OctreeSpringFiller other)
    {
        if (collisionLayer == null || other.collisionLayer == null)
            return true; // Default behavior if layers not set

        return collisionLayer.CanCollideWith(other.collisionLayer.layerIndex);
    }

    public CollisionLayer GetCollisionLayer()
    {
        return collisionLayer;
    }

    public void SetCollisionLayer(CollisionLayer newLayer)
    {
        collisionLayer = newLayer;
        ApplyMaterialProperties();
    }

    public void SetLayerIndex(int newIndex)
    {
        if (collisionLayer != null)
        {
            collisionLayer.layerIndex = newIndex;
        }
    }

    private void ApplyMaterialProperties()
    {
        if (collisionLayer == null) return;

        // NEW: For jelly/gel, reduce stiffness near surface
        if (collisionLayer.poissonRatio > 0.4f) // Incompressible like jelly
        {
            for (int i = 0; i < allSpringPoints.Length; i++)
            {
                var p = allSpringPoints[i];
                if (p.isMeshVertex == 1) p.mass *= 0.8f; // Softer surface
                allSpringPoints[i] = p;
            }
        }

        // Update collision manager defaults (used only as fallback)
        if (collisionManager != null)
        {
            collisionManager.defaultCoefficientOfRestitution = collisionLayer.restitution;
            collisionManager.defaultCoefficientOfFriction = collisionLayer.friction;
        }
    }

    private IEnumerator WaitForMeshDeformerInitialization()
    {
        while (!meshDeformer.isInitialized)
        {
            // Debug.Log("Waiting for MeshDeformer to initialize...");
            yield return null; // Wait one frame
        }

        // Debug.Log("MeshDeformer is ready, proceeding with subdivision...");
        meshDeformer.SubdivideMeshWithPoints(surfaceSpringPoints2);
        Debug.Log(meshDeformer.meshFilter.mesh.vertices.Length);
        Debug.Log(meshDeformer.meshFilter.mesh.triangles.Length);
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
        if (surfaceSpringPoints2.IsCreated)
        {
            // Calculate surface points
            meshJobManager.IdentifySurfacePoints(
                meshVertices,
                meshTriangles,
                transform.worldToLocalMatrix
            );
        }
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
            collisionJobManager.ScheduleGroundCollisionJobs(groundLevel, groundBounce, groundFriction);
            collisionJobManager.CompleteAllJobsAndApply();
        }

        if (applyRoomCollision)
        {
            collisionJobManager.ScheduleRoomCollisionJob(
                roomMinBounds, roomMaxBounds,
                roomBounce, roomFriction
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
            // if (enableMeshSubdivision)
            //     Debug.LogWarning($"{gameObject.name}: CollisionManager is not assigned in the inspector!", this);
        }

        if (Time.frameCount % 3 == 0) // Every 3 frames
        {
            //meshJobManager.UpdateSurfacePointsInMesh(meshVertices, meshTriangles,
            //    transform.localToWorldMatrix, transform.worldToLocalMatrix,
            //    targetMesh, autoUpdateMeshFromSurface);
            UpdateSurfacePointsInMesh();
        }

        // Handle Mesh Update
        UpdateMeshFromPoints();
        //meshJobManager.DispatchMeshUpdate(meshVertices, transform.worldToLocalMatrix, targetMesh, transform);
        //meshJobManager.ScheduleMeshVerticesUpdateJobs(meshVertices, meshTriangles, transform.localToWorldMatrix, transform.worldToLocalMatrix);
        //meshJobManager.CompleteAllJobsAndApply(meshVertices, meshTriangles, targetMesh, surfacePoints);
    }
    void LateUpdate()
    {
        if (visualizeRenderer == null || !allSpringPoints.IsCreated || !allSpringConnections.IsCreated)
        {
            return;  // Skip rendering if not initialized
        }

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
            // Debug.Log($"{gameObject.name} added to CollisionManager. Total bodies: {CollisionManager.AllSoftBodies.Count}", this);
        }
    }

    private void OnDisable()
    {
        if (CollisionManager.AllSoftBodies.Contains(this))
        {
            CollisionManager.AllSoftBodies.Remove(this);
            // NEW DEBUG LOG
            // Debug.Log($"{gameObject.name} removed from CollisionManager. Total bodies: {CollisionManager.AllSoftBodies.Count}", this);
        }
    }

    public void FillObjectWithSpringPoints()
    {
        // NEW: Clear surface point data
        if (surfaceSpringPoints2.IsCreated)
        {
            surfaceSpringPoints2.Clear();
            surfacePointToVertexIndex.Clear();
        }

        // Recalculate accurate world-space bounds
        if (meshVertices.Length <= 0)
        {
            // Debug.Log("Vertices Error");
            return;
        }

        // Abort if complexity too high
        float complexityEstimate = meshBounds.size.magnitude / PointSpacing;
        if (complexityEstimate > 100000)
        {
            // Debug.LogError($"Aborted: Excessive complexity ({complexityEstimate})");
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
        CreateSpringConnectionsForAllPoints();

        // Copy to native arrays
        allSpringPoints = tempPoints.AsArray();
        allSpringConnections = tempConnections.AsArray();


        // Some logs
        // Debug.Log($"Octree Nodes: {total_nodes}");
        // Debug.Log($"Created {allSpringPoints.Length} spring points.");
        // Debug.Log($"Created {allSpringConnections.Length} spring connections.");
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
                FillNodeWithSpringPoints(node);
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

                    // Convert to world space - this is how worldPos is obtained
                    Vector3 worldPos = transform.TransformPoint(localPos);

                    // Use approximate comparison instead of exact Contains
                    bool alreadyExists = allPointPositions.Any(p =>
                        Vector3.Distance(p, worldPos) < PointSpacing * 0.5f);

                    if (!alreadyExists)
                    {
                        // Your line here: Adaptive spacing based on surface proximity
                        float currentSpacing = IsPointNearSurface(worldPos) ? PointSpacing * 0.5f : PointSpacing;
                        // Note: In practice, you'd use currentSpacing to adjust step sizes or skip additions,
                        // but for simplicity, it can influence whether to add the point or not.

                        if (isFilled)
                        {
                            if (IsPointInsideMesh(worldPos))
                            {
                                allPointPositions.Add(worldPos);
                                CreateSpringPoint(worldPos, node.worldBounds, false);
                            }
                        }
                        else
                        {
                            if (IsPointOnMeshSurface(worldPos))
                            {
                                allPointPositions.Add(worldPos);
                                CreateSpringPoint(worldPos, node.worldBounds, false);
                            }
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
    ulong HashPair(int a, int b)
    {
        uint min = (uint)math.min(a, b);
        uint max = (uint)math.max(a, b);
        return ((ulong)min << 32) | max;
    }

    void CreateSpringConnectionsForAllPoints()
    {
        tempConnections.Clear();
        if (tempPoints.Length == 0) return;

        // Build a spatial hash for fast radius queries
        float maxRadius = math.max(connectionRadiusL1, math.max(connectionRadiusL2, connectionRadiusL3));
        var spatialHash = new SpatialHash(maxRadius * 1.1f); // Slightly larger cell size for bend connections
        for (int i = 0; i < tempPoints.Length; i++)
        {
            spatialHash.Add(tempPoints[i].position, i);
        }

        var connectedPairs = new HashSet<ulong>();
        for (int i = 0; i < tempPoints.Length; i++)
        {
            float3 posA = tempPoints[i].position;

            // Precompute neighbor lists ONCE per connection type
            var structNeighbors = spatialHash.Query(posA, connectionRadiusL1);
            var shearNeighbors = spatialHash.Query(posA, connectionRadiusL2);
            var bendNeighbors = spatialHash.Query(posA, connectionRadiusL3);

            // Structure connections (6)
            FindAndConnectNeighbors(i, posA, _structureDirections,
                /*PointSpacing * 1.5f*/ connectionRadiusL1, 15f, ConnectionType.Structure,
                structNeighbors, connectedPairs);

            // Shear connections (12)
            FindAndConnectNeighbors(i, posA, _shearDirections,
                /*PointSpacing * 3.5f*/connectionRadiusL2, 35f, ConnectionType.Shear,
                shearNeighbors, connectedPairs);

            // Bend connections (26)
            FindAndConnectNeighbors(i, posA, _bendDirections,
                /*PointSpacing * 5f*/connectionRadiusL3, 50f, ConnectionType.Bend,
                bendNeighbors, connectedPairs);
        }
    }

    void CreateSpringConnectionsForPoint(int pointIndex, SpringPointData pointData)
    {
        if (tempPoints.Length == 0) return;

        // Build a spatial hash for fast radius queries
        float maxRadius = math.max(connectionRadiusL1, math.max(connectionRadiusL2, connectionRadiusL3));
        var spatialHash = new SpatialHash(maxRadius * 1.1f); // Slightly larger cell size for bend connections
        for (int i = 0; i < tempPoints.Length; i++)
        {
            spatialHash.Add(tempPoints[i].position, i);
        }

        var connectedPairs = new HashSet<ulong>();

        // Precompute neighbor lists ONCE per connection type
        var structNeighbors = spatialHash.Query(pointData.position, connectionRadiusL1);
        var shearNeighbors = spatialHash.Query(pointData.position, connectionRadiusL2);
        var bendNeighbors = spatialHash.Query(pointData.position, connectionRadiusL3);

        // Structure connections (6)
        FindAndConnectNeighbors(pointIndex, pointData.position, _structureDirections,
            /*PointSpacing * 1.5f*/ connectionRadiusL1, 15f, ConnectionType.Structure,
            structNeighbors, connectedPairs);

        // Shear connections (12)
        FindAndConnectNeighbors(pointIndex, pointData.position, _shearDirections,
            /*PointSpacing * 3.5f*/connectionRadiusL2, 35f, ConnectionType.Shear,
            shearNeighbors, connectedPairs);

        // Bend connections (26)
        FindAndConnectNeighbors(pointIndex, pointData.position, _bendDirections,
            /*PointSpacing * 5f*/connectionRadiusL3, 50f, ConnectionType.Bend,
            bendNeighbors, connectedPairs);
    }

    void FindAndConnectNeighbors(int pointIndex, float3 position, float3[] directions,
        float maxDistance, float maxAngleDegrees, ConnectionType type,
        IEnumerable<int> neighbors, HashSet<ulong> connectedPairs)
    {
        foreach (var dir in directions)
        {
            // Find nearest neighbor in this direction
            int bestNeighbor = -1;
            float bestDot = -1f; // Direction alignment score (-1 to 1)

            foreach (int j in neighbors/*spatialHash.Query(position, maxDistance)*/)
            {
                if (j == pointIndex) continue;

                float3 toNeighbor = math.normalize(tempPoints[j].position - position);
                float dot = math.dot(dir, toNeighbor);

                // Check if neighbor is roughly in the target direction
                if (dot > math.cos(math.radians(maxAngleDegrees)))
                {
                    if (dot > bestDot)
                    {
                        bestDot = dot;
                        bestNeighbor = j;
                    }
                }
            }

            // Create connection if valid neighbor found
            if (bestNeighbor != -1)
            {
                ulong pair = HashPair(pointIndex, bestNeighbor);
                if (!connectedPairs.Contains(pair))
                {
                    float dist = math.distance(position, tempPoints[bestNeighbor].position);
                    switch (type)
                    {
                        case ConnectionType.Structure:
                            if (!IsConnected(pointIndex, bestNeighbor))
                            {
                                AddConnection(pointIndex, bestNeighbor, dist,
                                maxRestLengthL1, springConstantL1, damperConstantL1);
                            }
                            break;
                        case ConnectionType.Shear:
                            if (!IsConnected(pointIndex, bestNeighbor))
                            {
                                AddConnection(pointIndex, bestNeighbor, dist,
                                maxRestLengthL2, springConstantL2, damperConstantL2);
                            }
                            break;
                        case ConnectionType.Bend:
                            if (!IsConnected(pointIndex, bestNeighbor))
                            {
                                AddConnection(pointIndex, bestNeighbor, dist,
                                maxRestLengthL3, springConstantL3, damperConstantL3);
                            }
                            break;
                    }
                    connectedPairs.Add(pair);
                }
            }
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


    public void FixCorners()
    {
        if (allSpringPoints.Length == 0) return;

        // Find the min and max bounds of all points
        Vector3 min = allSpringPoints[0].position;
        Vector3 max = allSpringPoints[0].position;

        for (int i = 1; i < allSpringPoints.Length; i++)
        {
            min = Vector3.Min(min, allSpringPoints[i].position);
            max = Vector3.Max(max, allSpringPoints[i].position);
        }

        // Tolerance for corner detection (1% of the smallest dimension)
        float tolerance = Vector3.Min(max, min).magnitude * 0.01f;

        // Mark points near the corners as fixed
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            SpringPointData point = allSpringPoints[i];
            Vector3 pos = point.position;

            // Check if this point is near any corner (within tolerance)
            if ((pos.x <= min.x + tolerance || pos.x >= max.x - tolerance) &&
                (pos.y <= min.y + tolerance || pos.y >= max.y - tolerance) &&
                (pos.z <= min.z + tolerance || pos.z >= max.z - tolerance))
            {
                point.isFixed = 1; // Mark as fixed
                point.velocity = float3.zero; // Reset velocity
                allSpringPoints[i] = point;
            }
        }

        // Debug.Log($"Fixed {allSpringPoints.Count(p => p.isFixed == 1)} corner points");
    }

    bool IsPointNearSurface(Vector3 worldPos, float distanceThreshold = 0.1f)
    {
        // Simple: check distance to nearest mesh vertex
        float minDist = float.MaxValue;
        foreach (var v in meshVertices)
        {
            minDist = Mathf.Min(minDist, Vector3.Distance(worldPos, transform.TransformPoint(v)));
        }
        return minDist < distanceThreshold;
    }

    /// <summary>
    /// Checks if a point is on the surface of the given mesh within a specified tolerance.
    /// </summary>
    bool IsPointOnMeshSurface(Vector3 point, float tolerance = 0.0001f)
    {
        Vector3[] vertices = meshVertices;
        int[] triangles = meshTriangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 p0 = transform.TransformPoint(vertices[triangles[i]]);
            Vector3 p1 = transform.TransformPoint(vertices[triangles[i + 1]]);
            Vector3 p2 = transform.TransformPoint(vertices[triangles[i + 2]]);

            if (IsPointOnTriangle(point, p0, p1, p2, tolerance))
                return true;
        }

        return false;
    }

    /// <summary>
    /// Checks if a point is on a specific triangle using barycentric coordinates and distance.
    /// </summary>
    bool IsPointOnTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c, float tolerance)
    {
        // Compute normal
        Vector3 normal = Vector3.Cross(b - a, c - a).normalized;

        // Project point onto triangle plane
        float distanceToPlane = Vector3.Dot(normal, point - a);
        if (Mathf.Abs(distanceToPlane) > tolerance)
            return false;

        // Closest point on plane
        Vector3 projectedPoint = point - distanceToPlane * normal;

        // Compute barycentric coordinates
        Vector3 v0 = b - a;
        Vector3 v1 = c - a;
        Vector3 v2 = projectedPoint - a;

        float d00 = Vector3.Dot(v0, v0);
        float d01 = Vector3.Dot(v0, v1);
        float d11 = Vector3.Dot(v1, v1);
        float d20 = Vector3.Dot(v2, v0);
        float d21 = Vector3.Dot(v2, v1);

        float denom = d00 * d11 - d01 * d01;
        if (Mathf.Abs(denom) < Mathf.Epsilon)
            return false;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        // Check if point is inside triangle using barycentric coordinates
        return (u >= -tolerance && v >= -tolerance && w >= -tolerance);
    }

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
        allPointPositions.Clear();

        if (meshJobManager) Destroy(meshJobManager);
        if (rigidJobManager) Destroy(rigidJobManager);
        if (springJobManager) Destroy(springJobManager);
        if (collisionJobManager) Destroy(collisionJobManager);

        if (tempPoints.IsCreated) tempPoints.Dispose();
        if (tempConnections.IsCreated) tempConnections.Dispose();

        if (allSpringPoints.IsCreated) allSpringPoints.Dispose();
        if (allSpringConnections.IsCreated) allSpringConnections.Dispose();

        if (surfaceSpringPoints2.IsCreated) surfaceSpringPoints2.Dispose();
        if (surfacePointsLocalSpace.IsCreated) surfacePointsLocalSpace.Dispose();

        if (visualizeRenderer != null)
        {
            visualizeRenderer.Dispose();
            visualizeRenderer = null;
        }

        Resources.UnloadUnusedAssets();
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

        // Dispose previous instances before reassigning
        if (surfaceSpringPoints2.IsCreated) surfaceSpringPoints2.Dispose();
        if (surfacePointsLocalSpace.IsCreated) surfacePointsLocalSpace.Dispose();

        surfaceSpringPoints2 = new NativeList<SpringPointData>(allSpringPoints.Length, Allocator.Persistent);
        surfacePointsLocalSpace = new NativeList<float3>(allSpringPoints.Length, Allocator.Persistent);

        meshJobManager.Initialize(meshVertices, meshTriangles, allSpringPoints, surfaceSpringPoints2, surfacePointsLocalSpace, surfaceDetectionThreshold);

        rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        collisionJobManager.InitializeArrays(allSpringPoints);

        meshJobManager.IdentifySurfacePoints(
            meshVertices,
            meshTriangles,
            transform.worldToLocalMatrix
        );
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

    // Add this dictionary as a class member to cache closest points
    private Dictionary<int, int> vertexToClosestPointMap = new Dictionary<int, int>();

    void UpdateMeshFromPoints()
    {
        if (allSpringPoints == null || allSpringPoints.Length == 0) return;

        // Get current mesh data directly from the mesh
        Vector3[] currentVertices = targetMesh.vertices;
        int[] currentTriangles = targetMesh.triangles;

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
        // Initialize the dictionary if it's empty or if vertex count changed
        if (vertexToClosestPointMap.Count == 0 || vertexToClosestPointMap.Count != currentVertices.Length)
        {
            vertexToClosestPointMap.Clear();
            for (int i = 0; i < currentVertices.Length; i++)
            {
                Vector3 worldVertex = transform.TransformPoint(currentVertices[i]);
                int closestPointIndex = FindClosestPointIndex(worldVertex);
                vertexToClosestPointMap[i] = closestPointIndex;
            }
        }

        Vector3[] newVertices = new Vector3[currentVertices.Length];
        for (int i = 0; i < currentVertices.Length; i++)
        {
            if (vertexToClosestPointMap.TryGetValue(i, out int pointIndex) &&
                pointIndex >= 0 && pointIndex < allSpringPoints.Length)
            {
                SpringPointData closestPoint = allSpringPoints[pointIndex];

                // Defensive check
                if (closestPoint.mass > 0 || closestPoint.isFixed == 0)
                {
                    newVertices[i] = transform.InverseTransformPoint(closestPoint.position);
                }
                else
                {
                    // Fallback to keeping the original vertex
                    newVertices[i] = currentVertices[i];
                }
            }
            else
            {
                // Fallback if cache is invalid
                Vector3 worldVertex = transform.TransformPoint(currentVertices[i]);
                SpringPointData closestPoint = FindClosestPoint(worldVertex);
                newVertices[i] = transform.InverseTransformPoint(closestPoint.position);
            }
        }

        // --- 4. Update mesh vertices and recalculate bounds ---
        targetMesh.vertices = newVertices;

        // Ensure triangles array is valid for new vertex count
        if (currentTriangles.Length > 0)
        {
            // Validate triangle indices
            int maxIndex = newVertices.Length - 1;
            for (int i = 0; i < currentTriangles.Length; i++)
            {
                if (currentTriangles[i] > maxIndex)
                {
                    currentTriangles[i] = maxIndex;
                }
            }
            targetMesh.triangles = currentTriangles;
        }

        targetMesh.RecalculateNormals();
        targetMesh.RecalculateBounds();

        // Update cached references
        meshVertices = newVertices;
        meshTriangles = currentTriangles;
    }

    // Helper method to find the index of the closest point
    private int FindClosestPointIndex(Vector3 worldPos)
    {
        int closestIndex = 0;
        float minDist = float.MaxValue;

        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            float dist = Vector3.Distance(worldPos, allSpringPoints[i].position);
            if (dist < minDist)
            {
                minDist = dist;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    // Clear the cache when points change (call this when adding/removing points)
    public void ClearVertexPointCache()
    {
        vertexToClosestPointMap.Clear();
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
            for (int i = 0; i < surfaceSpringPoints2.Length; i++)
            {
                int springPointIndex = surfacePointToVertexIndex.Keys.ElementAt(i);
                int vertexIndex = surfacePointToVertexIndex.Values.ElementAt(i);

                if (springPointIndex < surfaceSpringPoints2.Length && vertexIndex < meshVertices.Length)
                {
                    Vector3 worldPosition = transform.TransformPoint(meshVertices[vertexIndex]);
                    SpringPointData sp = surfaceSpringPoints2[springPointIndex];
                    sp.position = worldPosition;
                    surfaceSpringPoints2[springPointIndex] = sp;
                }
            }

        }
    }

    // NEW Gizmos for debugging surface points
    private void OnDrawGizmosSelected()
    {
        if (surfaceSpringPoints2.IsCreated && surfaceSpringPoints2.Length > 0)
        {
            Gizmos.color = Color.yellow;
            foreach (var sp in surfaceSpringPoints2)
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
        for (int i = 0; i < surfaceSpringPoints2.Length; i++)
        {
            SpringPointData point = surfaceSpringPoints2[i];
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
            surfaceSpringPoints2[i] = point;
        }
    }

    public List<Vector3> GetSurfacePointsInCollision(OctreeSpringFiller other)
    {
        List<Vector3> points = new List<Vector3>();
        foreach (var point in surfaceSpringPoints2)
        {
            if (other.IsPointInside(other.transform.InverseTransformPoint(point.position)))
            {
                points.Add(point.position);
            }
        }
        return points;
    }

    public float GetTotalKineticEnergy()
    {
        float ke = 0f;
        foreach (var p in allSpringPoints) ke += 0.5f * p.mass * math.lengthsq(p.velocity);
        return ke;
    }

    public void DampenVelocities(float factor)
    {
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            var p = allSpringPoints[i];
            p.velocity *= factor;
            allSpringPoints[i] = p;
        }
    }

}