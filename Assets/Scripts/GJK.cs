using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor.Rendering.LookDev;
using UnityEngine;


public struct SoftBodyData
{
    public float3 worldPosition;
    public NativeList<SpringPointData> surfacePoints;

    public SoftBodyData(float3 worldPosition, NativeList<SpringPointData> surfacePoints)
    {
        this.worldPosition = worldPosition;
        this.surfacePoints = surfacePoints;
    }
}

/// <summary>
/// Contains the result of a GJK/EPA collision query.
/// </summary>
public struct CollisionInfo
{
    public bool DidCollide;
    public float3 Normal;
    public float Depth;
}

/// <summary>
/// A static class providing a full GJK and EPA implementation for convex shapes.
/// </summary>
public static class GJK
{
    private const int MaxGJKIterations = 32;
    private const int MaxEPAIterations = 32;
    private const float Epsilon = 0.0001f;

    /// <summary>
    /// Detects collision between two soft bodies using the GJK algorithm.
    /// If a collision occurs, it uses EPA to find the penetration normal and depth.
    /// </summary>
    [BurstCompile]
    public struct GJKCollisionJob : IJob
    {
        [ReadOnly] public SoftBodyData bodyA;
        [ReadOnly] public SoftBodyData bodyB;
        public NativeArray<CollisionInfo> result; // length 1

        [ReadOnly] public int MaxGJKIterations;
        [ReadOnly] public int MaxEPAIterations;
        [ReadOnly] public float Epsilon;

        public void Execute()
        {
            CollisionInfo info = new CollisionInfo();
            FixedList4096Bytes<float3> simplex = default;

            float3 direction = bodyA.worldPosition - bodyB.worldPosition;
            if (math.lengthsq(direction) < 1e-6f)
                direction = new float3(1, 0, 0); // right

            // Initial support point
            float3 support = Support(bodyA, bodyB, direction);
            simplex.Add(support);
            direction = -support;

            // GJK Main Loop
            for (int i = 0; i < MaxGJKIterations; i++)
            {
                support = Support(bodyA, bodyB, direction);
                if (math.dot(support, direction) < 0)
                {
                    // No collision
                    info.DidCollide = false;
                    //Debug.Log($"GJK End: No Collision found after {i} iterations!"); // DEBUG
                    result[0] = info;
                    return;
                }

                simplex.Add(support);

                if (HandleSimplex(ref simplex, ref direction))
                {
                    // Collision found, proceed to EPA
                    info.DidCollide = true;
                    //Debug.Log($"GJK End: Collision found after {i} iterations! Proceeding to EPA."); // DEBUG
                    EPA(ref simplex, bodyA, bodyB, out info.Normal, out info.Depth);
                    result[0] = info;
                    return;
                }
            }

            // Max iterations reached, likely no collision
            info.DidCollide = false;
            result[0] = info;
        }
    }

    public static void DetectCollision(ref SoftBodyData obj1, ref SoftBodyData obj2, out CollisionInfo info)
    {
        NativeArray<CollisionInfo> arr = new NativeArray<CollisionInfo>(1, Allocator.TempJob);
        new GJKCollisionJob
        {
            bodyA = obj1,
            bodyB = obj2,
            result = arr,

            MaxGJKIterations = MaxGJKIterations,
            MaxEPAIterations = MaxEPAIterations,
            Epsilon = Epsilon,

        }.Schedule().Complete();

        info = arr[0];
        arr.Dispose();
    }

    /// <summary>
    /// The Support function for the Minkowski Difference of two soft bodies.
    /// </summary>
    public static float3 Support(in SoftBodyData a, in SoftBodyData b, float3 direction)
    {
        float3 p1 = FindFurthestPoint(a, direction);
        float3 p2 = FindFurthestPoint(b, -direction);

        // DEBUG: Draw a line between the two support points in world space.
        //Debug.DrawLine(p1, p2, Color.magenta, 0.1f);

        return p1 - p2;
    }

    /// <summary>
    /// Finds the point on a soft body's surface furthest in a given world-space direction.
    /// This is the corrected version that uses the current deformed shape.
    /// </summary>
    public static float3 FindFurthestPoint(in SoftBodyData body, float3 direction)
    {
        float maxDot = float.NegativeInfinity;
        float3 furthestPoint = float3.zero;

        if (body.surfacePoints.Length == 0)
        {
            //Debug.LogWarning($"No surface points for GJK calculation!");
            return body.worldPosition;
        }

        for (int i = 0; i < body.surfacePoints.Length; i++)
        {
            float3 point = body.surfacePoints[i].position;
            float dot = math.dot(point, direction);
            if (dot > maxDot)
            {
                maxDot = dot;
                furthestPoint = point;
            }
        }

        // DEBUG: Visualize the search direction on the body and the resulting furthest point.
        //Debug.DrawRay(body.worldPosition, math.normalize(direction) * 2, Color.green, 0.1f);
        //Debug.DrawLine(body.worldPosition, furthestPoint, Color.yellow, 0.1f);

        return furthestPoint;
    }

    /// <summary>
    /// Processes the current simplex to see if it contains the origin.
    /// If not, it updates the simplex and the search direction.
    /// </summary>
    private static bool HandleSimplex(ref FixedList4096Bytes<float3> simplex, ref float3 direction)
    {
        if (simplex.Length == 2) return Line(ref simplex, ref direction);
        if (simplex.Length == 3) return Triangle(ref simplex, ref direction);
        if (simplex.Length == 4) return Tetrahedron(ref simplex, ref direction);
        return false;
    }

    private static bool Line(ref FixedList4096Bytes<float3> simplex, ref float3 direction)
    {
        float3 a = simplex[simplex.Length - 1];
        float3 b = simplex[simplex.Length - 2];
        float3 ab = b - a;
        float3 ao = -a;

        if (math.dot(ab, ao) > 0)
        {
            direction = math.cross(math.cross(ab, ao), ab);
        }
        else
        {
            simplex.Clear();
            simplex.Add(a);
            direction = ao;
        }
        return false;
    }

    private static bool Triangle(ref FixedList4096Bytes<float3> simplex, ref float3 direction)
    {
        float3 a = simplex[simplex.Length - 1];
        float3 b = simplex[simplex.Length - 2];
        float3 c = simplex[simplex.Length - 3];

        float3 ab = b - a;
        float3 ac = c - a;
        float3 ao = -a;

        float3 abc = math.cross(ab, ac);

        if (math.dot(math.cross(abc, ac), ao) > 0)
        {
            if (math.dot(ac, ao) > 0)
            {
                simplex.RemoveAt(simplex.Length - 2);
                direction = math.cross(math.cross(ac, ao), ac);
            }
            else
            {
                return Line(ref simplex, ref direction);
            }
        }
        else
        {
            if (math.dot(math.cross(ab, abc), ao) > 0)
            {
                return Line(ref simplex, ref direction);
            }
            else
            {
                if (math.dot(abc, ao) > 0)
                {
                    direction = abc;
                }
                else
                {
                    direction = -abc;
                }
            }
        }
        return false;
    }

    private static bool Tetrahedron(ref FixedList4096Bytes<float3> simplex, ref float3 direction)
    {
        float3 a = simplex[3];
        float3 b = simplex[2];
        float3 c = simplex[1];
        float3 d = simplex[0];

        float3 ao = -a;
        float3 ab = b - a;
        float3 ac = c - a;
        float3 ad = d - a;

        float3 abc = math.cross(ab, ac);
        float3 acd = math.cross(ac, ad);
        float3 adb = math.cross(ad, ab);

        if (math.dot(abc, ao) > 0)
        {
            simplex.Clear();
            simplex.Add(c);
            simplex.Add(b);
            simplex.Add(a);
            return Triangle(ref simplex, ref direction);
        }
        if (math.dot(acd, ao) > 0)
        {
            simplex.Clear();
            simplex.Add(d);
            simplex.Add(c);
            simplex.Add(a);
            return Triangle(ref simplex, ref direction);
        }
        if (math.dot(adb, ao) > 0)
        {
            simplex.Clear();
            simplex.Add(b);
            simplex.Add(d);
            simplex.Add(a);
            return Triangle(ref simplex, ref direction);
        }
        return true; // Origin is enclosed
    }

    /// <summary>
    /// Expanding Polytope Algorithm. Calculates the penetration depth and normal.
    /// </summary>
    private static void EPA(ref FixedList4096Bytes<float3> simplex, in SoftBodyData bodyA, in SoftBodyData bodyB, out float3 normal, out float depth)
    {
        FixedList4096Bytes<float3> polytope = simplex;
        FixedList4096Bytes<int> faces = new FixedList4096Bytes<int>();
        faces.Add(0); faces.Add(1); faces.Add(2);
        faces.Add(0); faces.Add(3); faces.Add(1);
        faces.Add(0); faces.Add(2); faces.Add(3);
        faces.Add(1); faces.Add(3); faces.Add(2);

        for (int i = 0; i < MaxEPAIterations; i++)
        {
            // Find face closest to origin
            float minDistance = float.MaxValue;
            float3 minNormal = float3.zero;
            int minFaceIndex = 0;

            for (int j = 0; j < faces.Length; j += 3)
            {
                float3 a = polytope[faces[j]];
                float3 b = polytope[faces[j + 1]];
                float3 c = polytope[faces[j + 2]];

                float3 n = math.cross(b - a, c - a);
                n = math.normalize(n);

                float dist = math.dot(n, a);

                if (dist < minDistance)
                {
                    minDistance = dist;
                    minNormal = n;
                    minFaceIndex = j;
                }
            }

            float3 support = Support(bodyA, bodyB, minNormal);
            float sDist = math.dot(minNormal, support);

            if (math.abs(sDist - minDistance) < Epsilon)
            {
                // Convergence
                normal = minNormal;
                depth = sDist;
                return;
            }

            // Expand polytope
            FixedList4096Bytes<int2> edges = new FixedList4096Bytes<int2>();
            FixedList4096Bytes<int> newFaces = new FixedList4096Bytes<int>();

            int newPointIndex = polytope.Length;
            polytope.Add(support);

            for (int j = 0; j < faces.Length; j += 3)
            {
                int i1 = faces[j];
                int i2 = faces[j + 1];
                int i3 = faces[j + 2];

                float3 a = polytope[i1];
                float3 b = polytope[i2];
                float3 c = polytope[i3];

                if (math.dot(math.cross(b - a, c - a), support - a) < 0)
                {
                    // Face is visible from the new point, add its edges to the list
                    AddEdge(ref edges, i1, i2);
                    AddEdge(ref edges, i2, i3);
                    AddEdge(ref edges, i3, i1);
                }
                else
                {
                    // Face is not visible, keep it
                    newFaces.Add(i1);
                    newFaces.Add(i2);
                    newFaces.Add(i3);
                }
            }

            // Create new faces from the silhouette edges to the new point
            for (int j = 0; j < edges.Length; j++)
            {
                newFaces.Add(edges[j].x);
                newFaces.Add(edges[j].y);
                newFaces.Add(newPointIndex);
            }

            faces = newFaces;
        }

        // Max iterations reached, return best guess
        FindClosestFace(polytope, faces, out normal, out depth);
    }

    private static void AddEdge(ref FixedList4096Bytes<int2> edges, int a, int b)
    {
        var reverse = new int2(b, a);
        for (int i = 0; i < edges.Length; i++)
        {
            if (edges[i].Equals(reverse))
            {
                edges.RemoveAt(i);
                return;
            }
        }
        edges.Add(new int2(a, b));
    }


    private static void FindClosestFace(
        in FixedList4096Bytes<float3> polytope,
        in FixedList4096Bytes<int> faces,
        out float3 normal,
        out float depth)
    {
        float minDistance = float.MaxValue;
        normal = new float3(0, 1, 0); // up
        depth = 0f;

        for (int i = 0; i < faces.Length; i += 3)
        {
            float3 a = polytope[faces[i]];
            float3 b = polytope[faces[i + 1]];
            float3 c = polytope[faces[i + 2]];

            float3 n = math.cross(b - a, c - a);
            n = math.normalize(n);
            float dist = math.dot(n, a);

            if (dist < minDistance)
            {
                minDistance = dist;
                normal = n;
            }
        }
        depth = minDistance;
    }
}