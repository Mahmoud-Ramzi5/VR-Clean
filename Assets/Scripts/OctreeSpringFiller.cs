using NUnit.Framework.Internal;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;


public struct SpringPointData
{
    public float3 position;
    public float3 velocity;
    public float3 force;

    public float mass;
    public int isFixed;       // 0 = false, 1 = true
    //public float radius;

    // Collision
    public float bounciness;
    public float friction;

    // Bounds (just min/max; no UnityEngine.Bounds)
    public float3 boundsMin;
    public float3 boundsMax;

    // Mesh
    public int isMeshVertex;  // 0 = false, 1 = true
    public int triangleIndex;

    // other data
    public float3 initialPosition;
    public float3 predictedPosition;

    public SpringPointData(
    float3 position,
    float3 velocity,
    float mass,
    int isFixed,
    float bounciness,
    float friction,
    float3 boundsMin,
    float3 boundsMax,
    int triangleIndex,
    int isMeshVertex
)
    {
        this.position = position;
        this.velocity = velocity;
        this.force = float3.zero;

        this.mass = mass;
        this.isFixed = isFixed;

        this.bounciness = bounciness;
        this.friction = friction;

        this.boundsMin = boundsMin;
        this.boundsMax = boundsMax;

        this.triangleIndex = triangleIndex;
        this.isMeshVertex = isMeshVertex;

        this.initialPosition = position;
        this.predictedPosition = position;
    }
}

public struct SpringConnectionData
{
    public int pointA;         // index into SpringPointData array
    public int pointB;         // index into SpringPointData array
    public float restLength;
    public float springConstant;
    public float damperConstant;

    public SpringConnectionData(int a, int b, float restLength, float springConstant, float damperConstant)
    {
        this.pointA = a;
        this.pointB = b;
        this.restLength = restLength;
        this.springConstant = springConstant;
        this.damperConstant = damperConstant;
    }
}


public class OctreeSpringFiller : MonoBehaviour
{
    [Header("Filling Settings")]
    public float minNodeSize = 0.5f;
    public float PointSpacing = 0.5f;
    public bool isFilled = true;
    public bool isRigid = true;

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
    public bool applyGravity = true;
    public Vector3 gravity => new Vector3(0, -9.81f, 0);

    private Mesh targetMesh;
    private Bounds meshBounds;
    private Vector3[] meshVertices;
    private int[] meshTriangles;
    private Vector3 lastPos;

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

    private NativeArray<SpringPointData> allSpringPoints;
    private NativeArray<SpringConnectionData> allSpringConnections;

    // Jobs
    private CollisionJobManager collisionJobManager;
    private SpringJobManager springJobManager;
    private RigidJobManager rigidJobManager;
    private MeshJobManager meshJobManager;

    private void Awake()
    {
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
        targetMesh.RecalculateBounds();

        meshBounds = targetMesh.bounds;
        meshVertices = targetMesh.vertices;
        meshTriangles = targetMesh.triangles;

        FillObjectWithSpringPoints();

        // Update positions and bounds on start
        for (int i = 0; i < allSpringPoints.Length; i++)
        {
            SpringPointData point = allSpringPoints[i]; // get copy
            point.mass = totalMass / allSpringPoints.Length;
            allSpringPoints[i] = point; // write back modified copy
        }

        //foreach (SpringPoint point in allSpringPoints)
        //{
        //    point.mass = totalMass / allSpringPoints.Count;
        //    Vector3 moveStep = transform.position - lastPos;
        //    point.UpdateBounds(moveStep);
        //}


        // Use Jobs to calculate physics on GPU threads
        // Parallelizing calculations improves performance

        // Mesh
        meshJobManager = gameObject.AddComponent<MeshJobManager>();
        meshJobManager.InitializeArrays(meshVertices, allSpringPoints);

        // Spring
        springJobManager = gameObject.AddComponent<SpringJobManager>();
        springJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        // Rigid
        rigidJobManager = gameObject.AddComponent<RigidJobManager>();
        rigidJobManager.InitializeArrays(allSpringPoints, allSpringConnections);

        // Collision
        collisionJobManager = gameObject.AddComponent<CollisionJobManager>();
        collisionJobManager.InitializeArrays(allSpringPoints);
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
            rigidJobManager.ScheduleRigidJobs(10, 0.1f, deltaTime);

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

        // Handle Mesh Update
        meshJobManager.ScheduleMeshUpdateJobs(
            meshVertices, transform.localToWorldMatrix, transform.worldToLocalMatrix
        );

        meshJobManager.CompleteAllJobsAndApply(targetMesh);
    }

    void LateUpdate()
    {
        visualizeRenderer.DrawInstancedPoints(visualizeSpringPoints, allSpringPoints);

        visualizeRenderer.UploadConnectionsToGPU(allSpringPoints, allSpringConnections);
        visualizeRenderer.DrawInstancedConnections(visualizeSpringConnections, 
            transform.position, allSpringPoints, allSpringConnections
        );
    }

    public void FillObjectWithSpringPoints()
    {
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

    void CreateSpringPoint(Vector3 worldPos, Bounds bounds, bool isMeshVertex)
    {
        float3 center = (float3)bounds.center;
        float3 extents = (float3)bounds.extents;

        SpringPointData point = new SpringPointData(
            position: worldPos,
            velocity: new float3(0, 0, 0),
            mass: 1.0f,
            isFixed: 0,
            bounciness: 0.5f,
            friction: 0.8f,
            boundsMin: center - extents,
            boundsMax: center + extents,
            triangleIndex: -1,
            isMeshVertex: isMeshVertex? 1 : 0
        );

        //if (isMeshVertex)
        //{
        //    point.isMeshVertex = true;
        //}
        //else
        //{
        //    Vector3 meshPoint = point.FindClosestMeshPoint(targetMesh, transform);

        //    // Find the index of the triangle that contains the meshPoint
        //    int triangleIndex = FindTriangleIndex(targetMesh, meshPoint);

        //    if (triangleIndex != -1)
        //    {
        //        point.triangleIndex = triangleIndex;
        //    }
        //    else
        //    {
        //        point.triangleIndex = -1;
        //    }
        //}

        tempPoints.Add(point);

        // Function to find the index of the triangle that contains a point
        int FindTriangleIndex(Mesh mesh, Vector3 point)
        {
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v1 = vertices[triangles[i]];
                Vector3 v2 = vertices[triangles[i + 1]];
                Vector3 v3 = vertices[triangles[i + 2]];

                // Check if the point is inside the triangle
                if (IsPointInTriangle(point, v1, v2, v3))
                {
                    return i / 3; // Return the triangle index
                }
            }

            return -1; // Return -1 if the point is not inside any triangle
        }

        // Function to check if a point is inside a triangle using Barycentric coordinates
        bool IsPointInTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
        {
            // Compute the vectors for the triangle edges
            Vector3 v0 = b - a;
            Vector3 v1 = c - a;
            Vector3 v2 = point - a;

            // Compute dot products
            float dot00 = Vector3.Dot(v0, v0);
            float dot01 = Vector3.Dot(v0, v1);
            float dot02 = Vector3.Dot(v0, v2);
            float dot11 = Vector3.Dot(v1, v1);
            float dot12 = Vector3.Dot(v1, v2);

            // Compute Barycentric coordinates
            float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            // Check if point is in triangle
            return (u >= 0) && (v >= 0) && (u + v < 1);
        }
    }

    void CreateSpringConnections()
    {
        // Clear existing connections
        tempConnections.Clear();

        // For each point, find nearby points and create connections
        for (int i = 0; i < tempPoints.Length; i++)
        {
            SpringPointData currentPoint = tempPoints[i];

            for (int j = i + 1; j < tempPoints.Length; j++)
            {
                SpringPointData otherPoint = tempPoints[j];
                float distance = math.distance(currentPoint.position, otherPoint.position);
                // Connect if within radius and not already connected
                if (distance <= connectionRadiusL1 * PointSpacing && !IsConnected(i, j))
                {
                    // Clamp rest length to reasonable values
                    float restLength = Mathf.Clamp(distance, 0.5f, maxRestLengthL1);

                    SpringConnectionData c = new SpringConnectionData(i, j, restLength, springConstantL1, damperConstantL1);
                    tempConnections.Add(c);
                }
                else if (distance <= connectionRadiusL2 * PointSpacing && !IsConnected(i, j))
                {
                    // Clamp rest length to reasonable values
                    float restLength = Mathf.Clamp(distance, 0.5f, maxRestLengthL2);

                    SpringConnectionData c = new SpringConnectionData(i, j, restLength, springConstantL2, damperConstantL2);
                    tempConnections.Add(c);
                }
                else if (distance <= connectionRadiusL3 * PointSpacing && !IsConnected(i, j))
                {
                    // Clamp rest length to reasonable values
                    float restLength = Mathf.Clamp(distance, 0.5f, maxRestLengthL3);

                    SpringConnectionData c = new SpringConnectionData(i, j, restLength, springConstantL3, damperConstantL3);
                    tempConnections.Add(c);
                }
                else
                {
                    continue;
                }
            }
        }
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
}
