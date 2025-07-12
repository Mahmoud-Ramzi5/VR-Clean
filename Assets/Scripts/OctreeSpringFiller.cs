using NUnit.Framework.Internal;
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
    public Bounds boundingVolume;

    public float groundLevel = 0f;       // Y-position of the ground plane
    public float groundBounce = 0.5f;   // Bounce coefficient (0 = no bounce, 1 = full bounce)
    public float groundFriction = 0.8f; // Friction coefficient (0 = full stop, 1 = no friction)
    public bool applyGroundCollision = true;

    [Header("External Systems")]
    public CollisionManager collisionManager;
    public MeshDeformer meshDeformer;

    [Header("Collision Layer System")]
    [SerializeField] private CollisionLayer collisionLayer;

    // Layer presets for easy setup
    [Header("Layer Presets")]
    [SerializeField] private CollisionLayerPreset layerPreset = CollisionLayerPreset.Default;

    [Header("Mesh Deformation")]
    public bool meshDeformationEnabled = true;
    public float influenceRadius = 2.0f;
    private float lastMeshUpdate = 0f;
    private const float MESH_UPDATE_INTERVAL = 1f / 30f;



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
    private MeshJobManagerCPU meshJobManager;

    // Surface point tracking
    private NativeList<SpringPointData> surfaceSpringPoints2;
    private NativeList<float3> surfacePointsLocalSpace;

    // Surface integration settings
    [Header("Surface Integration")]
    public bool enableMeshSubdivision = true;
    public bool autoUpdateMeshFromSurface = true;
    public float surfaceDetectionThreshold = 0.2f;

    public bool applyVelocity = false;

    private List<SpringPointData> surfaceSpringPoints = new List<SpringPointData>();
    Dictionary<int, int> surfacePointToVertexIndex = new Dictionary<int, int>();
    public List<SpringPointData> SurfacePoints => surfaceSpringPoints;
    public List<SpringPointData> surfacePoints = new List<SpringPointData>();


    //Track mesh state
    private int lastMeshVertexCount = 0;
    private int lastMeshTriangleCount = 0;
    private bool meshNeedsUpdate = false;


    // changes for the new connections
    public enum ConnectionType
    {
        Structure,  // 6 cardinal directions (axis-aligned)
        Shear,      // 12 edge diagonals (45° to axes)
        Bend        // 8 face diagonals (next layer)
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

        if (surfaceSpringPoints2.IsCreated && surfaceSpringPoints2.Length > 0)
        {
            // Copy data from NativeList to regular List (existing code)
            for (int i = 0; i < surfaceSpringPoints2.Length; i++)
            {
                SpringPointData point = surfaceSpringPoints2[i];
                surfaceSpringPoints.Add(point);
            }

            // NEW: Build the surface point to vertex mapping
            BuildInitialSurfacePointMapping();
        }

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
        for (int i = 0; i < surfaceSpringPoints2.Length; i++)
        {
            SpringPointData point = surfaceSpringPoints2[i];
            surfaceSpringPoints.Add(point);
        }

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
                Debug.LogWarning($"{gameObject.name}: No CollisionManager found in scene!");
            }
        }

        if (collisionLayer == null)
        {
            InitializeCollisionLayer();
        }


        // Apply layer preset if selected
        ApplyLayerPreset();

        ValidateAndSyncMesh();
    }

    // Add this method to validate and sync mesh data
    private void ValidateAndSyncMesh()
    {
        if (targetMesh == null)
        {
            Debug.LogError("Target mesh is null!");
            return;
        }

        Vector3[] currentVertices = targetMesh.vertices;
        int[] currentTriangles = targetMesh.triangles;

        if (currentVertices == null || currentVertices.Length == 0)
        {
            Debug.LogError("Mesh has no vertices!");
            return;
        }

        if (currentTriangles == null || currentTriangles.Length == 0)
        {
            Debug.LogError("Mesh has no triangles!");
            return;
        }

        if (currentTriangles.Length % 3 != 0)
        {
            Debug.LogError($"Invalid triangle count: {currentTriangles.Length} (should be multiple of 3)");
            return;
        }

        // Validate triangle indices
        for (int i = 0; i < currentTriangles.Length; i++)
        {
            if (currentTriangles[i] < 0 || currentTriangles[i] >= currentVertices.Length)
            {
                Debug.LogError($"Invalid triangle index: {currentTriangles[i]} at position {i} (vertex count: {currentVertices.Length})");
                return;
            }
        }

        // Update cached data
        meshVertices = currentVertices;
        meshTriangles = currentTriangles;
        lastMeshVertexCount = currentVertices.Length;
        lastMeshTriangleCount = currentTriangles.Length;

        Debug.Log($"Mesh validated: {lastMeshVertexCount} vertices, {lastMeshTriangleCount / 3} triangles");
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

        // Apply density to total mass
        totalMass = CalculateVolumeFromMesh() * collisionLayer.density;

        // Apply material properties to spring constants
        CalculateRealisticSpringConstants();

        // Apply to collision properties
        collisionManager.coefficientOfRestitution = collisionLayer.restitution;
        collisionManager.coefficientOfFriction = collisionLayer.friction;
    }

    private float CalculateVolumeFromMesh()
    {
        if (meshBounds.size.magnitude == 0) return 1f;

        // Simple volume calculation - can be improved with actual mesh volume calculation
        return meshBounds.size.x * meshBounds.size.y * meshBounds.size.z;
    }

    private void CalculateRealisticSpringConstants()
    {
        if (collisionLayer == null) return;

        float volume = CalculateVolumeFromMesh();
        float avgSpacing = PointSpacing;

        // Convert Young's modulus to spring constant
        float area = avgSpacing * avgSpacing;
        float length = avgSpacing;

        springConstantL1 = (collisionLayer.youngsModulus * area) / length;
        springConstantL2 = springConstantL1 * 0.6f;
        springConstantL3 = springConstantL1 * 0.4f;

        // Calculate damping for critical damping
        float avgMass = totalMass / (allSpringPoints.IsCreated ? allSpringPoints.Length : 100);
        float baseDamping = 2.0f * Mathf.Sqrt(springConstantL1 * avgMass);

        damperConstantL1 = baseDamping * collisionLayer.dampingFactor;
        damperConstantL2 = baseDamping * collisionLayer.dampingFactor * 0.8f;
        damperConstantL3 = baseDamping * collisionLayer.dampingFactor * 0.6f;
    }

    // Modified WaitForMeshDeformerInitialization with better error handling
    private System.Collections.IEnumerator WaitForMeshDeformerInitialization()
    {
        int waitFrames = 0;
        const int maxWaitFrames = 300; // 5 seconds at 60fps

        while (!meshDeformer.isInitialized && waitFrames < maxWaitFrames)
        {
            Debug.Log($"Waiting for MeshDeformer to initialize... ({waitFrames}/{maxWaitFrames})");
            waitFrames++;
            yield return null;
        }

        if (waitFrames >= maxWaitFrames)
        {
            Debug.LogError("MeshDeformer failed to initialize within timeout!");
            yield break;
        }

        Debug.Log("MeshDeformer is ready, proceeding with subdivision...");

        // Validate mesh before subdivision
        ValidateAndSyncMesh();

        // Validate surface points
        if (surfaceSpringPoints == null || surfaceSpringPoints.Count == 0)
        {
            Debug.LogWarning("No surface spring points found for subdivision");
            yield break;
        }

        try
        {
            // Validate MeshDeformer state
            if (meshDeformer != null)
            {
                meshDeformer.ValidateAndRepairMesh();
            }

            meshDeformer.SubdivideMeshWithPoints(surfaceSpringPoints);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error during mesh subdivision: {e.Message}");
            Debug.LogError($"Stack trace: {e.StackTrace}");
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

        // Check if mesh needs validation and synchronization
        if (targetMesh != null)
        {
            Vector3[] currentVertices = targetMesh.vertices;
            int[] currentTriangles = targetMesh.triangles;

            // Detect mesh changes and validate
            if (currentVertices.Length != lastMeshVertexCount ||
                currentTriangles.Length != lastMeshTriangleCount)
            {
                Debug.Log($"Mesh changed detected: vertices {lastMeshVertexCount} -> {currentVertices.Length}, triangles {lastMeshTriangleCount} -> {currentTriangles.Length}");
                ValidateAndSyncMesh();

                // Reinitialize mesh job manager with new data
                if (meshJobManager != null)
                {
                    try
                    {
                        meshJobManager.UpdateMeshData(meshVertices, meshTriangles);
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogError($"Error updating mesh job manager: {e.Message}");
                    }
                }
            }
        }

        // Validate spring points array before physics operations
        if (!allSpringPoints.IsCreated || allSpringPoints.Length == 0)
        {
            Debug.LogWarning("Spring points array is not created or empty, skipping physics update");
            return;
        }

        // ----- PHYSICS SIMULATION -----
        if (isRigid)
        {
            // ----- RIGID MODE -----
            try
            {
                // 1. Schedule gravity job
                rigidJobManager.ScheduleGravityJobs(gravity, applyGravity);

                // 2. Schedule spring jobs
                rigidJobManager.ScheduleRigidJobs(10, 0.5f, deltaTime);

                // 3. Complete all jobs and apply results
                rigidJobManager.CompleteAllJobsAndApply();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error in rigid body simulation: {e.Message}");
            }
        }
        else
        {
            // ----- SOFT BODY MODE -----
            try
            {
                // 1. Schedule gravity job
                springJobManager.ScheduleGravityJobs(gravity, applyGravity);

                // 2. Schedule spring jobs
                springJobManager.ScheduleSpringJobs(deltaTime);

                // 3. Complete all jobs and apply results
                springJobManager.CompleteAllJobsAndApply();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error in soft body simulation: {e.Message}");
            }
        }

        // ----- COLLISION HANDLING -----

        // Ground collision
        if (applyGroundCollision)
        {
            try
            {
                collisionJobManager.ScheduleGroundCollisionJobs(
                    groundLevel, groundBounce, groundFriction
                );
                collisionJobManager.CompleteAllJobsAndApply();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error in ground collision: {e.Message}");
            }
        }

        // Inter-object collisions
        if (collisionManager != null)
        {
            try
            {
                collisionManager.ResolveInterObjectCollisions();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error in inter-object collisions: {e.Message}");
            }
        }
        else if (enableMeshSubdivision)
        {
            // Only warn if mesh subdivision is enabled and we need collision manager
            if (Time.frameCount % 300 == 0) // Warn every 5 seconds at 60fps
            {
                Debug.LogWarning($"{gameObject.name}: CollisionManager is not assigned but mesh subdivision is enabled!");
            }
        }

        // ----- SURFACE POINT UPDATES -----

        // Update surface points periodically for performance
        if (Time.frameCount % 3 == 0) // Every 3 frames
        {
            try
            {
                UpdateSurfacePointsInMesh();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error updating surface points: {e.Message}");
            }
        }

        // ----- MESH DEFORMATION UPDATES -----

        // Trigger mesh deformation update if enabled
        if (meshDeformationEnabled && meshDeformer != null && meshDeformer.isInitialized)
        {
            try
            {
                // Force mesh deformation update periodically
                if (Time.frameCount % 2 == 0) // Every 2 frames for responsive deformation
                {
                    meshDeformer.ForceDeformationUpdate();
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error in mesh deformation update: {e.Message}");
            }
        }

        // ----- MESH UPDATES -----

        // Handle Mesh Update with comprehensive error handling
        try
        {
            if (meshJobManager != null && meshVertices != null && meshTriangles != null && targetMesh != null)
            {
                // Validate mesh data before processing
                if (meshVertices.Length == 0 || meshTriangles.Length == 0)
                {
                    Debug.LogWarning("Skipping mesh update: empty mesh data");
                    return;
                }

                if (meshTriangles.Length % 3 != 0)
                {
                    Debug.LogError($"Invalid triangle array length: {meshTriangles.Length} (should be multiple of 3)");
                    ValidateAndSyncMesh(); // Try to repair
                    return;
                }

                // Validate triangle indices
                bool hasInvalidIndices = false;
                for (int i = 0; i < meshTriangles.Length; i++)
                {
                    if (meshTriangles[i] < 0 || meshTriangles[i] >= meshVertices.Length)
                    {
                        Debug.LogError($"Invalid triangle index: {meshTriangles[i]} at position {i} (vertex count: {meshVertices.Length})");
                        hasInvalidIndices = true;
                        break;
                    }
                }

                if (hasInvalidIndices)
                {
                    ValidateAndSyncMesh(); // Try to repair
                    return;
                }

                // Schedule mesh update jobs
                meshJobManager.ScheduleMeshVerticesUpdateJobs(
                    meshVertices, meshTriangles,
                    transform.localToWorldMatrix, transform.worldToLocalMatrix);

                // Complete and apply mesh updates
                meshJobManager.CompleteAllJobsAndApply(
                    meshVertices, meshTriangles, targetMesh, surfacePoints);
            }
            else
            {
                // Log missing components for debugging
                if (meshJobManager == null)
                    Debug.LogWarning("MeshJobManager is null");
                if (meshVertices == null)
                    Debug.LogWarning("MeshVertices is null");
                if (meshTriangles == null)
                    Debug.LogWarning("MeshTriangles is null");
                if (targetMesh == null)
                    Debug.LogWarning("TargetMesh is null");
            }
        }
        catch (System.ArgumentOutOfRangeException e)
        {
            Debug.LogError($"Argument out of range in mesh update: {e.Message}");
            Debug.LogError($"Stack trace: {e.StackTrace}");

            // Attempt recovery
            Debug.Log("Attempting mesh data recovery...");
            ValidateAndSyncMesh();

            // If mesh deformer exists, try to validate it too
            if (meshDeformer != null)
            {
                meshDeformer.ValidateAndRepairMesh();
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Unexpected error in mesh update: {e.Message}");
            Debug.LogError($"Stack trace: {e.StackTrace}");

            // Attempt basic recovery
            ValidateAndSyncMesh();
        }

        // ----- POSITION TRACKING -----

        // Update transform position tracking for next frame
        Vector3 currentPos = transform.position;
        if (Vector3.Distance(currentPos, lastPos) > 0.001f)
        {
            lastPos = currentPos;

            // Optional: Update spring point bounds if object moved significantly
            if (Time.frameCount % 30 == 0) // Every 30 frames
            {
                try
                {
                    UpdateBoundingVolume();
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"Error updating bounding volume: {e.Message}");
                }
            }
        }

        // ----- DEBUGGING AND VALIDATION -----

        // Periodic validation check in debug builds
#if UNITY_EDITOR || DEVELOPMENT_BUILD
        if (Time.frameCount % 600 == 0) // Every 10 seconds at 60fps
        {
            try
            {
                ValidateMeshState();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error in periodic validation: {e.Message}");
            }
        }
#endif
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
        CreateSpringConnectionsForAllPoints();

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

    //void CreateSpringConnectionsForAllPoints()
    //{
    //    tempConnections.Clear();
    //    if (tempPoints.Length == 0) return;

    //    // Calculate maximum connection radius
    //    float maxRadius = math.max(math.max(connectionRadiusL3, connectionRadiusL2), connectionRadiusL1) * PointSpacing;
    //    if (maxRadius <= 0) return;

    //    // Initialize spatial grid
    //    float cellSize = maxRadius;
    //    Dictionary<int3, List<int>> grid = new Dictionary<int3, List<int>>();

    //    var connectedPairs = new HashSet<ulong>();

    //    // Assign points to grid cells
    //    for (int i = 0; i < tempPoints.Length; i++)
    //    {
    //        int3 cell = GetCell(tempPoints[i].position, cellSize);
    //        if (!grid.TryGetValue(cell, out List<int> list))
    //        {
    //            list = new List<int>();
    //            grid.Add(cell, list);
    //        }
    //        list.Add(i);
    //    }

    //    // Create connections using spatial queries
    //    for (int i = 0; i < tempPoints.Length; i++)
    //    {
    //        float3 posA = tempPoints[i].position;
    //        int3 cell = GetCell(posA, cellSize);

    //        List<(int, float)> candidates = new List<(int, float)>();
    //        // Check 3x3x3 neighbor cells
    //        for (int x = -1; x <= 1; x++)
    //        {
    //            for (int y = -1; y <= 1; y++)
    //            {
    //                for (int z = -1; z <= 1; z++)
    //                {
    //                    int3 neighborCell = cell + new int3(x, y, z);
    //                    if (!grid.TryGetValue(neighborCell, out List<int> points))
    //                        continue;

    //                    foreach (int j in points)
    //                    {
    //                        // Ensure each pair is only processed once (i < j)
    //                        if (j <= i || IsConnected(i, j)) continue;

    //                        ulong pair = HashPair(i, j);
    //                        if (connectedPairs.Contains(pair)) continue;

    //                        float3 posB = tempPoints[j].position;
    //                        float dist = math.distance(posA, posB);
    //                        if (dist <= maxRadius)
    //                            candidates.Add((j, dist));
    //                    }
    //                }
    //            }
    //        }

    //        // Sort by distance, prioritize closest connections
    //        candidates.Sort((a, b) => a.Item2.CompareTo(b.Item2));

    //        int added = 0;
    //        foreach (var (j, dist) in candidates)
    //        {
    //            if (added >= maxConnectionsPerPoint) break;

    //            float threshold1 = connectionRadiusL1 * PointSpacing;
    //            float threshold2 = connectionRadiusL2 * PointSpacing;
    //            float threshold3 = connectionRadiusL3 * PointSpacing;

    //            if (dist <= threshold1)
    //            {
    //                AddConnection(i, j, dist, maxRestLengthL1, springConstantL1, damperConstantL1);
    //            }
    //            else if (dist <= threshold2)
    //            {
    //                AddConnection(i, j, dist, maxRestLengthL2, springConstantL2, damperConstantL2);
    //            }
    //            else if (dist <= threshold3)
    //            {
    //                AddConnection(i, j, dist, maxRestLengthL3, springConstantL3, damperConstantL3);
    //            }
    //            else
    //            {
    //                continue;
    //            }

    //            connectedPairs.Add(HashPair(i, j));
    //            added++;
    //        }
    //    }
    //}


    // testing new connect function
    void CreateSpringConnectionsForAllPoints()
    {
        tempConnections.Clear();
        if (tempPoints.Length == 0) return;

        // Build a spatial hash for fast radius queries
        var spatialHash = new SpatialHash(PointSpacing * 10f); // Larger cell size for bend connections
        for (int i = 0; i < tempPoints.Length; i++)
        {
            spatialHash.Add(tempPoints[i].position, i);
        }

        var connectedPairs = new HashSet<ulong>();

        for (int i = 0; i < tempPoints.Length; i++)
        {
            float3 posA = tempPoints[i].position;

            // Structure connections (6)
            FindAndConnectNeighbors(i, posA, _structureDirections,
                PointSpacing * 1.5f, 15f, ConnectionType.Structure,
                spatialHash, connectedPairs);

            // Shear connections (12)
            FindAndConnectNeighbors(i, posA, _shearDirections,
                PointSpacing * 4f, 35f, ConnectionType.Shear,
                spatialHash, connectedPairs);

            // Bend connections (8)
            FindAndConnectNeighbors(i, posA, _bendDirections,
                PointSpacing * 10f, 50f, ConnectionType.Bend,
                spatialHash, connectedPairs);
        }
    }

    void FindAndConnectNeighbors(int pointIndex, float3 position, float3[] directions,
        float maxDistance, float maxAngleDegrees, ConnectionType type,
        SpatialHash spatialHash, HashSet<ulong> connectedPairs)
    {
        foreach (var dir in directions)
        {
            // Find nearest neighbor in this direction
            int bestNeighbor = -1;
            float bestDot = -1f; // Direction alignment score (-1 to 1)

            foreach (int j in spatialHash.Query(position, maxDistance))
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
                            AddConnection(pointIndex, bestNeighbor, dist,
                                maxRestLengthL1, springConstantL1, damperConstantL1);
                            break;
                        case ConnectionType.Shear:
                            AddConnection(pointIndex, bestNeighbor, dist,
                                maxRestLengthL2, springConstantL2, damperConstantL2);
                            break;
                        case ConnectionType.Bend:
                            AddConnection(pointIndex, bestNeighbor, dist,
                                maxRestLengthL3, springConstantL3, damperConstantL3);
                            break;
                    }
                    connectedPairs.Add(pair);
                }
            }
        }
    }

    // Fast spatial partitioning for radius queries
    public class SpatialHash
    {
        private readonly float _cellSize;
        private readonly Dictionary<int3, List<int>> _cells = new Dictionary<int3, List<int>>();

        public SpatialHash(float cellSize) { _cellSize = cellSize; }

        public void Add(float3 position, int index)
        {
            int3 cell = new int3(
                (int)math.floor(position.x / _cellSize),
                (int)math.floor(position.y / _cellSize),
                (int)math.floor(position.z / _cellSize)
            );
            if (!_cells.TryGetValue(cell, out var list))
            {
                list = new List<int>();
                _cells.Add(cell, list);
            }
            list.Add(index);
        }

        public IEnumerable<int> Query(float3 position, float radius)
        {
            int3 minCell = GetCell(position - radius, _cellSize);
            int3 maxCell = GetCell(position + radius, _cellSize);

            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    for (int z = minCell.z; z <= maxCell.z; z++)
                    {
                        if (_cells.TryGetValue(new int3(x, y, z), out var list))
                        {
                            foreach (int idx in list) yield return idx;
                        }
                    }
                }
            }
        }
        int3 GetCell(float3 position, float size)
        {
            return new int3(
                (int)math.floor(position.x / size),
                (int)math.floor(position.y / size),
                (int)math.floor(position.z / size)
            );
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

    // Modified AddSpringPointAtPosition with better mesh handling
    public void AddSpringPointAtPosition(Vector3 worldPosition)
    {
        try
        {
            Vector3 worldCenter = transform.TransformPoint(meshBounds.center);
            Vector3 worldSize = Vector3.Scale(meshBounds.size, transform.lossyScale);
            Bounds worldBounds = new Bounds(worldCenter, worldSize);

            var newPoint = CreateSpringPoint(worldPosition, worldBounds, false);
            allSpringPoints = tempPoints.AsArray();

            int newPointIndex = tempPoints.Length - 1;
            CreateSpringConnectionsForPoint(newPointIndex, newPoint);
            allSpringConnections = tempConnections.AsArray();

            // Update mass distribution
            float newMass = totalMass / allSpringPoints.Length;
            for (int i = 0; i < allSpringPoints.Length; i++)
            {
                SpringPointData point = allSpringPoints[i];
                point.mass = newMass;
                allSpringPoints[i] = point;
            }

            // Update mesh data safely
            UpdateMeshDataWithNewPoint(worldPosition);

            // Reinitialize job managers with updated data
            ValidateAndSyncMesh();
            ReinitializeJobManagers();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error adding spring point: {e.Message}");
        }
    }

    private void ReinitializeJobManagers()
    {
        try
        {
            // Dispose and recreate surface point containers
            if (surfaceSpringPoints2.IsCreated) surfaceSpringPoints2.Clear();
            if (surfacePointsLocalSpace.IsCreated) surfacePointsLocalSpace.Clear();

            surfaceSpringPoints2.Capacity = allSpringPoints.Length;
            surfacePointsLocalSpace.Capacity = allSpringPoints.Length;

            // Reinitialize mesh job manager
            if (meshJobManager != null)
            {
                meshJobManager.Initialize(meshVertices, meshTriangles, allSpringPoints,
                    surfaceSpringPoints2, surfacePointsLocalSpace, surfaceDetectionThreshold);
            }

            // Reinitialize other job managers
            if (rigidJobManager != null)
                rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

            if (springJobManager != null)
                springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

            if (collisionJobManager != null)
                collisionJobManager.InitializeArrays(allSpringPoints);

            // Identify surface points
            if (meshJobManager != null)
            {
                meshJobManager.IdentifySurfacePoints(
                    meshVertices, meshTriangles, transform.worldToLocalMatrix);
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error reinitializing job managers: {e.Message}");
        }
    }

    private void UpdateMeshDataWithNewPoint(Vector3 newWorldPosition)
    {
        try
        {
            Vector3 newLocalPosition = transform.InverseTransformPoint(newWorldPosition);

            // Create new arrays with increased size
            Vector3[] newVertices = new Vector3[meshVertices.Length + 1];
            int[] newTriangles = new int[meshTriangles.Length + 3];

            // Copy existing data
            System.Array.Copy(meshVertices, newVertices, meshVertices.Length);
            System.Array.Copy(meshTriangles, newTriangles, meshTriangles.Length);

            // Add new vertex
            newVertices[meshVertices.Length] = newLocalPosition;

            // Find two closest existing vertices for triangle creation
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

            // Add new triangle
            newTriangles[meshTriangles.Length] = closest1;
            newTriangles[meshTriangles.Length + 1] = closest2;
            newTriangles[meshTriangles.Length + 2] = meshVertices.Length; // new vertex index

            // Update cached arrays
            meshVertices = newVertices;
            meshTriangles = newTriangles;

            // Update mesh
            targetMesh.vertices = newVertices;
            targetMesh.triangles = newTriangles;
            targetMesh.RecalculateNormals();
            targetMesh.RecalculateBounds();

            // Update tracking
            lastMeshVertexCount = newVertices.Length;
            lastMeshTriangleCount = newTriangles.Length;

            Debug.Log($"Added new mesh vertex: {newVertices.Length} vertices, {newTriangles.Length / 3} triangles");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error updating mesh with new point: {e.Message}");
        }
    }

    // Add validation method for debugging
    public void ValidateMeshState()
    {
        Debug.Log("=== MESH STATE VALIDATION ===");
        Debug.Log($"Target Mesh: {(targetMesh != null ? "Valid" : "NULL")}");

        if (targetMesh != null)
        {
            Debug.Log($"Mesh Vertices: {targetMesh.vertexCount}");
            Debug.Log($"Mesh Triangles: {targetMesh.triangles.Length / 3}");
        }

        Debug.Log($"Cached Vertices: {(meshVertices != null ? meshVertices.Length.ToString() : "NULL")}");
        Debug.Log($"Cached Triangles: {(meshTriangles != null ? (meshTriangles.Length / 3).ToString() : "NULL")}");
        Debug.Log($"Spring Points: {(allSpringPoints.IsCreated ? allSpringPoints.Length.ToString() : "Not Created")}");
        Debug.Log($"Surface Points: {surfaceSpringPoints.Count}");
        Debug.Log("=============================");
    }

    // Call this in OnValidate for editor debugging
    private void OnValidate()
    {
        if (Application.isPlaying)
        {
            ValidateMeshState();
        }
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

    private void UpdateSurfacePointsInMesh()
    {
        if (!autoUpdateMeshFromSurface) return;

        if (!allSpringPoints.IsCreated || allSpringPoints.Length == 0)
        {
            return;
        }

        if (targetMesh == null)
        {
            Debug.LogWarning("Target mesh is null in UpdateSurfacePointsInMesh");
            return;
        }

        bool meshChanged = false;
        meshVertices = targetMesh.vertices;

        // Method 1: Use direct spring point to mesh vertex mapping
        if (TryUpdateMeshFromSpringPoints(ref meshChanged))
        {
            // Successfully updated using direct mapping
        }
        else
        {
            // Fallback: Build mapping on the fly and update
            BuildRealTimeVertexMapping();
            TryUpdateMeshFromSpringPoints(ref meshChanged);
        }

        if (meshChanged)
        {
            try
            {
                targetMesh.vertices = meshVertices;
                targetMesh.RecalculateNormals();
                targetMesh.RecalculateBounds();

                Debug.Log($"Mesh updated from spring points: {meshVertices.Length} vertices");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error applying mesh changes: {e.Message}");
            }
        }
    }

    // Build vertex mapping in real-time when needed
    private void BuildRealTimeVertexMapping()
    {
        surfacePointToVertexIndex.Clear();

        if (!allSpringPoints.IsCreated || meshVertices == null)
        {
            return;
        }

        // For each mesh vertex, find the closest spring point
        for (int vertexIndex = 0; vertexIndex < meshVertices.Length; vertexIndex++)
        {
            Vector3 vertexWorldPos = transform.TransformPoint(meshVertices[vertexIndex]);

            int closestSpringIndex = -1;
            float minDistance = float.MaxValue;

            // Find closest spring point to this vertex
            for (int springIndex = 0; springIndex < allSpringPoints.Length; springIndex++)
            {
                SpringPointData springPoint = allSpringPoints[springIndex];
                float distance = Vector3.Distance(vertexWorldPos, springPoint.position);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestSpringIndex = springIndex;
                }
            }

            // Only map if we found a reasonably close spring point
            if (closestSpringIndex >= 0 && minDistance < influenceRadius)
            {
                // Map spring point index to vertex index
                if (!surfacePointToVertexIndex.ContainsKey(closestSpringIndex))
                {
                    surfacePointToVertexIndex[closestSpringIndex] = vertexIndex;
                }
            }
        }

        Debug.Log($"Built real-time vertex mapping: {surfacePointToVertexIndex.Count} mappings");
    }

    // Enhanced method to initialize surface point mapping during startup
    private void BuildInitialSurfacePointMapping()
    {
        surfacePointToVertexIndex.Clear();

        if (!allSpringPoints.IsCreated || meshVertices == null)
        {
            Debug.LogWarning("Cannot build surface point mapping: missing data");
            return;
        }

        // Method 1: Map surface spring points to vertices
        if (surfaceSpringPoints != null && surfaceSpringPoints.Count > 0)
        {
            for (int i = 0; i < surfaceSpringPoints.Count; i++)
            {
                SpringPointData surfacePoint = surfaceSpringPoints[i];

                // Find this surface point in the main spring points array
                int mainArrayIndex = FindSpringPointInMainArray(surfacePoint);
                if (mainArrayIndex >= 0)
                {
                    // Find closest vertex to this surface point
                    int closestVertexIndex = FindClosestVertexToPoint(surfacePoint.position);
                    if (closestVertexIndex >= 0)
                    {
                        surfacePointToVertexIndex[mainArrayIndex] = closestVertexIndex;
                    }
                }
            }
        }
        else
        {
            // Method 2: Fallback - use all spring points and find surface ones
            BuildRealTimeVertexMapping();
        }

        Debug.Log($"Built initial surface point mapping: {surfacePointToVertexIndex.Count} surface points mapped to vertices");
    }

    // Helper method to find a surface point in the main spring points array
    private int FindSpringPointInMainArray(SpringPointData targetPoint)
    {
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            SpringPointData point = allSpringPoints[i];

            // Match by position (since structs can't be compared by reference)
            if (Vector3.Distance(point.position, targetPoint.position) < 0.001f)
            {
                return i;
            }
        }
        return -1;
    }

    // Helper method to find closest vertex to a world position
    private int FindClosestVertexToPoint(Vector3 worldPosition)
    {
        if (meshVertices == null || meshVertices.Length == 0)
            return -1;

        int closestIndex = -1;
        float minDistance = float.MaxValue;
        Vector3 localPosition = transform.InverseTransformPoint(worldPosition);

        for (int i = 0; i < meshVertices.Length; i++)
        {
            float distance = Vector3.Distance(localPosition, meshVertices[i]);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }

        return minDistance < influenceRadius ? closestIndex : -1;
    }

    // Method to update mesh using spring point positions
    private bool TryUpdateMeshFromSpringPoints(ref bool meshChanged)
    {
        if (surfacePointToVertexIndex.Count == 0)
        {
            return false; // No mapping available
        }

        foreach (var kvp in surfacePointToVertexIndex)
        {
            int springPointIndex = kvp.Key;
            int vertexIndex = kvp.Value;

            // Validate indices
            if (springPointIndex < 0 || springPointIndex >= allSpringPoints.Length ||
                vertexIndex < 0 || vertexIndex >= meshVertices.Length)
            {
                continue;
            }

            // Get spring point data (this is a copy, but we only need to read from it)
            SpringPointData springPoint = allSpringPoints[springPointIndex];

            // Convert spring point world position to local mesh space
            Vector3 newLocalPos = transform.InverseTransformPoint(springPoint.position);
            Vector3 oldLocalPos = meshVertices[vertexIndex];

            // Check if vertex needs updating (threshold to avoid unnecessary updates)
            if (Vector3.Distance(newLocalPos, oldLocalPos) > 0.001f)
            {
                meshVertices[vertexIndex] = newLocalPos;
                meshChanged = true;
            }
        }

        return true;
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

    public enum CollisionLayerPreset
    {
        Default,
        Rubber,
        Metal,
        Plastic,
        Gel,
        Custom
    }
}
