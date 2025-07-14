using UnityEngine;
using System.Collections.Generic;
using Unity.Mathematics;

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
    public static bool DetectCollision(OctreeSpringFiller bodyA, OctreeSpringFiller bodyB, out CollisionInfo info)
    {
        info = new CollisionInfo();
        List<float3> simplex = new List<float3>();
        float3 direction = (float3)(bodyA.transform.position - bodyB.transform.position);
        if (math.all(direction == float3.zero))
        {
            direction = new float3(1, 0, 0); // right
        }

        // Initial support point
        float3 support = Support(bodyA, bodyB, direction);
        simplex.Add(support);
        direction = -support;

        // GJK Main Loop
        for (int i = 0; i < MaxGJKIterations; i++)
        {
            // DEBUG: Visualize the current search direction from the origin of the Minkowski Difference.
            Debug.DrawRay(float3.zero, math.normalize(direction) * 2, Color.cyan, 0.1f);

            support = Support(bodyA, bodyB, direction);

            if (math.dot(support, direction) < 0f)
            {
                // No collision
                info.DidCollide = false;
                // Debug.Log($"GJK End: No Collision found after {i} iterations!"); // DEBUG
                return false;
            }

            simplex.Add(support);

            if (HandleSimplex(ref simplex, ref direction))
            {
                // Collision found, proceed to EPA
                info.DidCollide = true;
                // Debug.Log($"GJK End: Collision found after {i} iterations! Proceeding to EPA."); // DEBUG
                EPA(simplex, bodyA, bodyB, out info.Normal, out info.Depth);
                return true;
            }
        }

        // Max iterations reached, likely no collision
        info.DidCollide = false;
        return false;
    }

    /// <summary>
    /// The Support function for the Minkowski Difference of two soft bodies.
    /// </summary>
    private static float3 Support(OctreeSpringFiller bodyA, OctreeSpringFiller bodyB, float3 direction)
    {
        float3 p1 = FindFurthestPoint(bodyA, direction);
        float3 p2 = FindFurthestPoint(bodyB, -direction);

        // DEBUG: Draw a line between the two support points in world space.
        Debug.DrawLine(p1, p2, Color.magenta, 0.1f);

        return p1 - p2;
    }

    /// <summary>
    /// Finds the point on a soft body's surface furthest in a given world-space direction.
    /// This is the corrected version that uses the current deformed shape.
    /// </summary>
    private static float3 FindFurthestPoint(OctreeSpringFiller body, float3 worldDirection)
    {
        float3 furthestPoint = float3.zero;
        float maxDot = float.NegativeInfinity;

        if (body.surfaceSpringPoints2.Length == 0)
        {
            // Debug.LogWarning($"{body.name} has no surface points for GJK calculation!", body);
            return body.transform.position;
        }

        // Initialize with the first point to ensure we have a valid starting point
        furthestPoint = body.surfaceSpringPoints2[0].position;
        maxDot = math.dot(furthestPoint, worldDirection);

        for (int i = 0; i < body.surfaceSpringPoints2.Length; i++)
        {
            SpringPointData sp = body.surfaceSpringPoints2[i];
            float dot = math.dot(sp.position, worldDirection);
            if (dot > maxDot)
            {
                maxDot = dot;
                furthestPoint = sp.position;
            }
        }

        // DEBUG: Visualize the search direction on the body and the resulting furthest point.
        float3 origin = (float3)body.transform.position;
        Debug.DrawRay(origin, math.normalize(worldDirection) * 2, Color.green, 0.1f);
        Debug.DrawLine(origin, furthestPoint, Color.yellow, 0.1f);

        return furthestPoint;
    }

    /// <summary>
    /// Processes the current simplex to see if it contains the origin.
    /// If not, it updates the simplex and the search direction.
    /// </summary>
    private static bool HandleSimplex(ref List<float3> simplex, ref float3 direction)
    {
        if (simplex.Count == 2) return Line(ref simplex, ref direction);
        if (simplex.Count == 3) return Triangle(ref simplex, ref direction);
        if (simplex.Count == 4) return Tetrahedron(ref simplex, ref direction);
        return false;
    }

    private static bool Line(ref List<float3> simplex, ref float3 direction)
    {
        float3 a = simplex[1];
        float3 b = simplex[0];
        float3 ab = b - a;
        float3 ao = -a;

        if (math.dot(ab, ao) > 0)
        {
            direction = math.cross(math.cross(ab, ao), ab);
        }
        else
        {
            simplex = new List<float3> { a };
            direction = ao;
        }
        return false;
    }

    private static bool Triangle(ref List<float3> simplex, ref float3 direction)
    {
        float3 a = simplex[2];
        float3 b = simplex[1];
        float3 c = simplex[0];

        float3 ab = b - a;
        float3 ac = c - a;
        float3 ao = -a;

        float3 abc = math.cross(ab, ac);

        if (math.dot(math.cross(abc, ac), ao) > 0)
        {
            if (math.dot(ac, ao) > 0)
            {
                simplex = new List<float3> { c, a };
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
                    // Swap winding order for EPA
                    simplex = new List<float3> { b, c, a };
                }
            }
        }
        return false;
    }

    private static bool Tetrahedron(ref List<float3> simplex, ref float3 direction)
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
            simplex = new List<float3> { c, b, a };
            return Triangle(ref simplex, ref direction);
        }
        if (math.dot(acd, ao) > 0)
        {
            simplex = new List<float3> { d, c, a };
            return Triangle(ref simplex, ref direction);
        }
        if (math.dot(adb, ao) > 0)
        {
            simplex = new List<float3> { b, d, a };
            return Triangle(ref simplex, ref direction);
        }
        return true; // Origin is enclosed
    }

    /// <summary>
    /// Expanding Polytope Algorithm. Calculates the penetration depth and normal.
    /// </summary>
    private static void EPA(List<float3> simplex, OctreeSpringFiller bodyA, OctreeSpringFiller bodyB, out float3 normal, out float depth)
    {
        List<float3> polytope = new List<float3>(simplex);
        List<int> faces = new List<int>
        {
            0, 1, 2,
            0, 3, 1,
            0, 2, 3,
            1, 3, 2
        };

        for (int i = 0; i < MaxEPAIterations; i++)
        {
            // Find face closest to origin
            int minFaceIndex = 0;
            float minDistance = float.MaxValue;
            float3 minNormal = float3.zero;

            for (int j = 0; j < faces.Count / 3; j++)
            {
                float3 a = polytope[faces[j * 3]];
                float3 b = polytope[faces[j * 3 + 1]];
                float3 c = polytope[faces[j * 3 + 2]];

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
            List<int> newFaces = new List<int>();
            List<int2> edges = new List<int2>();
            int newPointIndex = polytope.Count;
            polytope.Add(support);

            for (int j = 0; j < faces.Count / 3; j++)
            {
                int i1 = faces[j * 3];
                int i2 = faces[j * 3 + 1];
                int i3 = faces[j * 3 + 2];

                float3 a = polytope[i1];
                float3 b = polytope[i2];
                float3 c = polytope[i3];

                if (math.dot(math.cross(b - a, c - a), support - a) < 0)
                {
                    // Face is visible from the new point, add its edges to the list
                    AddEdge(edges, i1, i2);
                    AddEdge(edges, i2, i3);
                    AddEdge(edges, i3, i1);
                }
                else
                {
                    // Face is not visible, keep it
                    newFaces.AddRange(new int[] { i1, i2, i3 });
                }
            }

            // Create new faces from the silhouette edges to the new point
            foreach (var edge in edges)
            {
                newFaces.AddRange(new int[] { edge.x, edge.y, newPointIndex });
            }
            faces = newFaces;
        }

        // Max iterations reached, return best guess
        FindClosestFace(polytope, faces, out normal, out depth);
    }

    private static void AddEdge(List<int2> edges, int a, int b)
    {
        var reverse = new int2(b, a);
        if (edges.Contains(reverse))
        {
            edges.Remove(reverse);
        }
        else
        {
            edges.Add(new int2(a, b));
        }
    }

    private static void FindClosestFace(List<float3> polytope, List<int> faces, out float3 normal, out float depth)
    {
        float minDistance = float.MaxValue;
        normal = new float3(0, 1, 0); // up

        for (int i = 0; i < faces.Count / 3; i++)
        {
            float3 a = polytope[faces[i * 3]];
            float3 b = polytope[faces[i * 3 + 1]];
            float3 c = polytope[faces[i * 3 + 2]];

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