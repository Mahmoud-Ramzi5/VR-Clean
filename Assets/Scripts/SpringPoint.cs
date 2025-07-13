using Unity.Mathematics;

public struct SpringPointData
{
    public float3 position;
    public float3 velocity;
    public float3 force;
    public float3 acc;

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
        this.acc = float3.zero;

        this.mass = mass;
        this.isFixed = isFixed;

        this.bounciness = bounciness;
        this.friction = friction;

        this.boundsMin = boundsMin;
        this.boundsMax = boundsMax;

        this.triangleIndex = triangleIndex;
        this.isMeshVertex = 0;

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