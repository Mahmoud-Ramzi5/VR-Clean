using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Contains the result of a GJK/EPA collision query.
/// </summary>
public struct CollisionInfo
{
    public bool DidCollide;
    public Vector3 Normal;
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
        List<Vector3> simplex = new List<Vector3>();
        Vector3 direction = bodyA.transform.position - bodyB.transform.position;
        if (direction == Vector3.zero)
        {
            direction = Vector3.right;
        }

        // Initial support point
        Vector3 support = Support(bodyA, bodyB, direction);
        simplex.Add(support);
        direction = -support;

        // GJK Main Loop
        for (int i = 0; i < MaxGJKIterations; i++)
        {
            // DEBUG: Visualize the current search direction from the origin of the Minkowski Difference.
            Debug.DrawRay(Vector3.zero, direction.normalized * 2, Color.cyan, 0.1f);

            support = Support(bodyA, bodyB, direction);

            if (Vector3.Dot(support, direction) < 0)
            {
                // No collision
                info.DidCollide = false;
                return false;
            }

            simplex.Add(support);

            if (HandleSimplex(ref simplex, ref direction))
            {
                // Collision found, proceed to EPA
                info.DidCollide = true;
                Debug.Log($"GJK End: Collision found after {i} iterations! Proceeding to EPA."); // DEBUG
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
    private static Vector3 Support(OctreeSpringFiller bodyA, OctreeSpringFiller bodyB, Vector3 direction)
    {
        Vector3 p1 = FindFurthestPoint(bodyA, direction);
        Vector3 p2 = FindFurthestPoint(bodyB, -direction);

        // DEBUG: Draw a line between the two support points in world space.
        Debug.DrawLine(p1, p2, Color.magenta, 0.1f);

        return p1 - p2;
    }

    /// <summary>
    /// Finds the point on a soft body's surface furthest in a given world-space direction.
    /// This is the corrected version that uses the current deformed shape.
    /// </summary>
    private static Vector3 FindFurthestPoint(OctreeSpringFiller body, Vector3 worldDirection)
    {
        Vector3 furthestPoint = Vector3.zero;
        float maxDot = float.NegativeInfinity;

        if (body.SurfacePoints.Count == 0)
        {
            Debug.LogWarning($"{body.name} has no surface points for GJK calculation!", body);
            return body.transform.position;
        }

        // Initialize with the first point to ensure we have a valid starting point
        furthestPoint = body.SurfacePoints[0].position;
        maxDot = Vector3.Dot(furthestPoint, worldDirection);

        foreach (SpringPointData sp in body.SurfacePoints)
        {
            float dot = Vector3.Dot(sp.position, worldDirection);
            if (dot > maxDot)
            {
                maxDot = dot;
                furthestPoint = sp.position;
            }
        }

        // DEBUG: Visualize the search direction on the body and the resulting furthest point.
        Vector3 origin = body.transform.position;
        Debug.DrawRay(origin, worldDirection.normalized * 2, Color.green, 0.1f);
        Debug.DrawLine(origin, furthestPoint, Color.yellow, 0.1f);

        return furthestPoint;
    }

    /// <summary>
    /// Processes the current simplex to see if it contains the origin.
    /// If not, it updates the simplex and the search direction.
    /// </summary>
    private static bool HandleSimplex(ref List<Vector3> simplex, ref Vector3 direction)
    {
        if (simplex.Count == 2) return Line(ref simplex, ref direction);
        if (simplex.Count == 3) return Triangle(ref simplex, ref direction);
        if (simplex.Count == 4) return Tetrahedron(ref simplex, ref direction);
        return false;
    }

    private static bool Line(ref List<Vector3> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[1];
        Vector3 b = simplex[0];
        Vector3 ab = b - a;
        Vector3 ao = -a;

        if (Vector3.Dot(ab, ao) > 0)
        {
            direction = Vector3.Cross(Vector3.Cross(ab, ao), ab);
        }
        else
        {
            simplex = new List<Vector3> { a };
            direction = ao;
        }
        return false;
    }

    private static bool Triangle(ref List<Vector3> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[2];
        Vector3 b = simplex[1];
        Vector3 c = simplex[0];

        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ao = -a;

        Vector3 abc = Vector3.Cross(ab, ac);

        if (Vector3.Dot(Vector3.Cross(abc, ac), ao) > 0)
        {
            if (Vector3.Dot(ac, ao) > 0)
            {
                simplex = new List<Vector3> { c, a };
                direction = Vector3.Cross(Vector3.Cross(ac, ao), ac);
            }
            else
            {
                return Line(ref simplex, ref direction);
            }
        }
        else
        {
            if (Vector3.Dot(Vector3.Cross(ab, abc), ao) > 0)
            {
                return Line(ref simplex, ref direction);
            }
            else
            {
                if (Vector3.Dot(abc, ao) > 0)
                {
                    direction = abc;
                }
                else
                {
                    direction = -abc;
                    // Swap winding order for EPA
                    simplex = new List<Vector3> { b, c, a };
                }
            }
        }
        return false;
    }

    private static bool Tetrahedron(ref List<Vector3> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[3];
        Vector3 b = simplex[2];
        Vector3 c = simplex[1];
        Vector3 d = simplex[0];

        Vector3 ao = -a;
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ad = d - a;

        Vector3 abc = Vector3.Cross(ab, ac);
        Vector3 acd = Vector3.Cross(ac, ad);
        Vector3 adb = Vector3.Cross(ad, ab);

        if (Vector3.Dot(abc, ao) > 0)
        {
            simplex = new List<Vector3> { c, b, a };
            return Triangle(ref simplex, ref direction);
        }
        if (Vector3.Dot(acd, ao) > 0)
        {
            simplex = new List<Vector3> { d, c, a };
            return Triangle(ref simplex, ref direction);
        }
        if (Vector3.Dot(adb, ao) > 0)
        {
            simplex = new List<Vector3> { b, d, a };
            return Triangle(ref simplex, ref direction);
        }
        return true; // Origin is enclosed
    }

    /// <summary>
    /// Expanding Polytope Algorithm. Calculates the penetration depth and normal.
    /// </summary>
    private static void EPA(List<Vector3> simplex, OctreeSpringFiller bodyA, OctreeSpringFiller bodyB, out Vector3 normal, out float depth)
    {
        List<Vector3> polytope = new List<Vector3>(simplex);
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
            Vector3 minNormal = Vector3.zero;

            for (int j = 0; j < faces.Count / 3; j++)
            {
                Vector3 a = polytope[faces[j * 3]];
                Vector3 b = polytope[faces[j * 3 + 1]];
                Vector3 c = polytope[faces[j * 3 + 2]];

                Vector3 n = Vector3.Cross(b - a, c - a);
                n.Normalize();
                float dist = Vector3.Dot(n, a);

                if (dist < minDistance)
                {
                    minDistance = dist;
                    minNormal = n;
                    minFaceIndex = j;
                }
            }

            Vector3 support = Support(bodyA, bodyB, minNormal);
            float sDist = Vector3.Dot(minNormal, support);

            if (Mathf.Abs(sDist - minDistance) < Epsilon)
            {
                // Convergence
                normal = minNormal;
                depth = sDist;
                return;
            }

            // Expand polytope
            List<int> newFaces = new List<int>();
            List<Vector2Int> edges = new List<Vector2Int>();
            int newPointIndex = polytope.Count;
            polytope.Add(support);

            for (int j = 0; j < faces.Count / 3; j++)
            {
                int i1 = faces[j * 3];
                int i2 = faces[j * 3 + 1];
                int i3 = faces[j * 3 + 2];

                Vector3 a = polytope[i1];
                Vector3 b = polytope[i2];
                Vector3 c = polytope[i3];

                if (Vector3.Dot(Vector3.Cross(b - a, c - a), support - a) < 0)
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

    private static void AddEdge(List<Vector2Int> edges, int a, int b)
    {
        var reverse = new Vector2Int(b, a);
        if (edges.Contains(reverse))
        {
            edges.Remove(reverse);
        }
        else
        {
            edges.Add(new Vector2Int(a, b));
        }
    }

    private static void FindClosestFace(List<Vector3> polytope, List<int> faces, out Vector3 normal, out float depth)
    {
        float minDistance = float.MaxValue;
        normal = Vector3.up;

        for (int i = 0; i < faces.Count / 3; i++)
        {
            Vector3 a = polytope[faces[i * 3]];
            Vector3 b = polytope[faces[i * 3 + 1]];
            Vector3 c = polytope[faces[i * 3 + 2]];

            Vector3 n = Vector3.Cross(b - a, c - a).normalized;
            float dist = Vector3.Dot(n, a);

            if (dist < minDistance)
            {
                minDistance = dist;
                normal = n;
            }
        }
        depth = minDistance;
    }
}