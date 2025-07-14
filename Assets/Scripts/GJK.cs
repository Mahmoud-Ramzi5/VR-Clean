// GJK.cs (Updated with Larger Limits and Safety Checks)

using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;

public struct CollisionInfo
{
    public bool DidCollide;
    public float3 Normal;
    public float Depth;
}

public static class GJK
{
    private const int MaxGJKIterations = 32;
    private const int MaxEPAIterations = 32;
    private const float Epsilon = 0.0001f;
    private const int MaxSimplexSize = 4;
    private const int MaxPolytopeSize = 128;  // Increased from 64
    private const int MaxFacesSize = 384;     // 128 faces * 3 indices
    private const int MaxEdgesSize = 384;     // Increased conservative

    [BurstCompile]
    public static bool DetectCollision(NativeSlice<float3> pointsA, NativeSlice<float3> pointsB, out CollisionInfo info)
    {
        info = new CollisionInfo();

        NativeArray<float3> simplex = new NativeArray<float3>(MaxSimplexSize, Allocator.Temp);
        int simplexCount = 0;

        float3 direction = new float3(1, 0, 0);  // Arbitrary initial direction

        // Initial support
        float3 support = Support(pointsA, pointsB, direction);
        simplex[simplexCount++] = support;
        direction = -support;

        for (int i = 0; i < MaxGJKIterations; i++)
        {
            support = Support(pointsA, pointsB, direction);

            if (math.dot(support, direction) < 0f)
            {
                info.DidCollide = false;
                return false;
            }

            simplex[simplexCount++] = support;

            if (HandleSimplex(simplex, ref simplexCount, ref direction))
            {
                info.DidCollide = true;
                EPA(simplex, simplexCount, pointsA, pointsB, out info.Normal, out info.Depth);
                return true;
            }
        }

        info.DidCollide = false;
        return false;
    }

    [BurstCompile]
    private static float3 Support(NativeSlice<float3> pointsA, NativeSlice<float3> pointsB, float3 direction)
    {
        float3 p1 = FindFurthestPoint(pointsA, direction);
        float3 p2 = FindFurthestPoint(pointsB, -direction);
        return p1 - p2;
    }

    [BurstCompile]
    private static float3 FindFurthestPoint(NativeSlice<float3> points, float3 direction)
    {
        if (points.Length == 0) return float3.zero;

        float3 furthest = points[0];
        float maxDot = math.dot(furthest, direction);

        for (int i = 1; i < points.Length; i++)
        {
            float dot = math.dot(points[i], direction);
            if (dot > maxDot)
            {
                maxDot = dot;
                furthest = points[i];
            }
        }
        return furthest;
    }

    [BurstCompile]
    private static bool HandleSimplex(NativeArray<float3> simplex, ref int count, ref float3 direction)
    {
        if (count == 2) return Line(simplex, ref count, ref direction);
        if (count == 3) return Triangle(simplex, ref count, ref direction);
        if (count == 4) return Tetrahedron(simplex, ref count, ref direction);
        return false;
    }

    [BurstCompile]
    private static bool Line(NativeArray<float3> simplex, ref int count, ref float3 direction)
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
            simplex[0] = a;
            count = 1;
            direction = ao;
        }
        return false;
    }

    [BurstCompile]
    private static bool Triangle(NativeArray<float3> simplex, ref int count, ref float3 direction)
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
                simplex[0] = a;
                simplex[1] = c;
                count = 2;
                direction = math.cross(math.cross(ac, ao), ac);
            }
            else
            {
                simplex[0] = a;
                simplex[1] = b;
                count = 2;
                return Line(simplex, ref count, ref direction);
            }
        }
        else
        {
            if (math.dot(math.cross(ab, abc), ao) > 0)
            {
                simplex[0] = a;
                simplex[1] = b;
                count = 2;
                return Line(simplex, ref count, ref direction);
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
                    // Swap winding
                    float3 temp = simplex[1];
                    simplex[1] = simplex[0];
                    simplex[0] = temp;
                }
            }
        }
        return false;
    }

    [BurstCompile]
    private static bool Tetrahedron(NativeArray<float3> simplex, ref int count, ref float3 direction)
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
            simplex[0] = a;
            simplex[1] = b;
            simplex[2] = c;
            count = 3;
            return Triangle(simplex, ref count, ref direction);
        }
        if (math.dot(acd, ao) > 0)
        {
            simplex[0] = a;
            simplex[1] = c;
            simplex[2] = d;
            count = 3;
            return Triangle(simplex, ref count, ref direction);
        }
        if (math.dot(adb, ao) > 0)
        {
            simplex[0] = a;
            simplex[1] = d;
            simplex[2] = b;
            count = 3;
            return Triangle(simplex, ref count, ref direction);
        }
        return true; // Origin enclosed
    }

    [BurstCompile]
    private static void EPA(NativeArray<float3> initialSimplex, int initialCount, NativeSlice<float3> pointsA, NativeSlice<float3> pointsB, out float3 normal, out float depth)
    {
        NativeArray<float3> polytope = new NativeArray<float3>(MaxPolytopeSize, Allocator.Temp);
        int polytopeCount = initialCount;
        for (int i = 0; i < initialCount; i++) polytope[i] = initialSimplex[i];

        NativeArray<int> faces = new NativeArray<int>(MaxFacesSize, Allocator.Temp);
        int faceCount = 12;
        faces[0] = 0; faces[1] = 1; faces[2] = 2;
        faces[3] = 0; faces[4] = 3; faces[5] = 1;
        faces[6] = 0; faces[7] = 2; faces[8] = 3;
        faces[9] = 1; faces[10] = 3; faces[11] = 2;

        for (int iter = 0; iter < MaxEPAIterations; iter++)
        {
            // Find closest face
            int minFaceIndex = 0;
            float minDistance = float.MaxValue;
            float3 minNormal = float3.zero;

            for (int j = 0; j < faceCount / 3; j++)
            {
                int idx = j * 3;
                float3 a = polytope[faces[idx]];
                float3 b = polytope[faces[idx + 1]];
                float3 c = polytope[faces[idx + 2]];

                float3 n = math.normalize(math.cross(b - a, c - a));
                float dist = math.dot(n, a);

                if (dist < minDistance)
                {
                    minDistance = dist;
                    minNormal = n;
                    minFaceIndex = j;
                }
            }

            float3 support = Support(pointsA, pointsB, minNormal);
            float sDist = math.dot(minNormal, support);

            if (math.abs(sDist - minDistance) < Epsilon)
            {
                normal = minNormal;
                depth = sDist;
                polytope.Dispose();
                faces.Dispose();
                return;
            }

            // Expand: Collect edges of visible faces
            NativeArray<int2> edges = new NativeArray<int2>(MaxEdgesSize, Allocator.Temp);
            int edgeCount = 0;
            int newFaceCount = 0;
            NativeArray<int> newFaces = new NativeArray<int>(MaxFacesSize, Allocator.Temp);

            for (int j = 0; j < faceCount / 3; j++)
            {
                int idx = j * 3;
                int i1 = faces[idx];
                int i2 = faces[idx + 1];
                int i3 = faces[idx + 2];

                float3 pa = polytope[i1];
                float3 pb = polytope[i2];
                float3 pc = polytope[i3];

                if (math.dot(math.cross(pb - pa, pc - pa), support - pa) < 0)
                {
                    AddEdge(edges, ref edgeCount, i1, i2);
                    AddEdge(edges, ref edgeCount, i2, i3);
                    AddEdge(edges, ref edgeCount, i3, i1);
                }
                else
                {
                    // Keep face
                    newFaces[newFaceCount] = i1;
                    newFaces[newFaceCount + 1] = i2;
                    newFaces[newFaceCount + 2] = i3;
                    newFaceCount += 3;
                }
            }

            // Add new point
            int newPointIndex = polytopeCount;
            polytope[polytopeCount++] = support;

            // Safety check for overflow
            if (polytopeCount >= MaxPolytopeSize || (newFaceCount + edgeCount * 3) >= MaxFacesSize)
            {
                FindClosestFace(polytope, polytopeCount, faces, faceCount, out normal, out depth);
                polytope.Dispose();
                faces.Dispose();
                edges.Dispose();
                newFaces.Dispose();
                return;
            }

            // Create new faces from edges
            for (int e = 0; e < edgeCount; e++)
            {
                newFaces[newFaceCount] = edges[e].x;
                newFaces[newFaceCount + 1] = edges[e].y;
                newFaces[newFaceCount + 2] = newPointIndex;
                newFaceCount += 3;
            }

            // Copy back to faces
            for (int f = 0; f < newFaceCount; f++)
            {
                faces[f] = newFaces[f];
            }
            faceCount = newFaceCount;

            edges.Dispose();
            newFaces.Dispose();
        }

        // Fallback
        FindClosestFace(polytope, polytopeCount, faces, faceCount, out normal, out depth);

        polytope.Dispose();
        faces.Dispose();
    }

    [BurstCompile]
    private static void AddEdge(NativeArray<int2> edges, ref int count, int a, int b)
    {
        int2 rev = new int2(b, a);
        for (int i = 0; i < count; i++)
        {
            if (edges[i].Equals(rev))
            {
                // Remove by swapping with last
                edges[i] = edges[--count];
                return;
            }
        }
        if (count < MaxEdgesSize)
        {
            edges[count++] = new int2(a, b);
        }
    }

    [BurstCompile]
    private static void FindClosestFace(NativeArray<float3> polytope, int polytopeCount, NativeArray<int> faces, int faceCount, out float3 normal, out float depth)
    {
        float minDistance = float.MaxValue;
        normal = new float3(0, 1, 0);
        depth = 0f;

        for (int i = 0; i < faceCount / 3; i++)
        {
            int idx = i * 3;
            float3 a = polytope[faces[idx]];
            float3 b = polytope[faces[idx + 1]];
            float3 c = polytope[faces[idx + 2]];

            float3 n = math.normalize(math.cross(b - a, c - a));
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