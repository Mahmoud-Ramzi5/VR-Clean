using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class CollisionManager : MonoBehaviour
{
    public static List<OctreeSpringFiller> AllSoftBodies = new List<OctreeSpringFiller>();

    [Header("Inter-Object Collision Settings")]
    public bool enableInterObjectCollision = true;

    [Header("Collision Response")]
    [Range(0f, 1f)]
    public float coefficientOfRestitution = 0.6f;
    [Range(0f, 1f)]
    public float coefficientOfFriction = 0.4f; // Note: Friction not implemented in this GJK response
    public float penetrationCorrectionFactor = 0.6f; // How strongly to push objects apart (0-1)
    public float penetrationSlop = 0.01f; // A small allowance for penetration to prevent jitter

    // Collision statistics
    [Header("Debug Info")]
    public int totalCollisionsThisFrame = 0;
    public bool showCollisionGizmos = false;
    private List<CollisionInfo> lastFrameCollisions = new List<CollisionInfo>();


    /// <summary>
    /// This is now the primary collision resolution method called in FixedUpdate.
    /// </summary>
    public void ResolveInterObjectCollisions()
    {
        for (int i = 0; i < AllSoftBodies.Count; i++)
        {
            OctreeSpringFiller obj1 = AllSoftBodies[i];
            obj1.UpdateBoundingVolume();

            for (int j = i + 1; j < AllSoftBodies.Count; j++)
            {
                OctreeSpringFiller obj2 = AllSoftBodies[j];
                obj2.UpdateBoundingVolume();

                if (!obj1.boundingVolume.Intersects(obj2.boundingVolume)) continue;

                if (GJK.DetectCollision(obj1, obj2, out CollisionInfo info))
                {
                    HandleCollisionPair(obj1, obj2, info);
                }
            }
        }
    }


    private void TriggerMeshDeformation(OctreeSpringFiller obj1, OctreeSpringFiller obj2)
    {
        if (obj1.TryGetComponent<MeshDeformer>(out var deformer1))
        {
            deformer1.HandleCollisionPoints(obj1.GetSurfacePointsInCollision(obj2));
        }
        if (obj2.TryGetComponent<MeshDeformer>(out var deformer2))
        {
            deformer2.HandleCollisionPoints(obj2.GetSurfacePointsInCollision(obj1));
        }
    }

    /// <summary>
    /// Handles the physics response for two colliding objects using the data from GJK/EPA.
    /// </summary>
    private void HandleCollisionPair(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Apply the actual physics response (this was commented out)
        HandleGJKCollisionResponse(obj1, obj2, info);

        // Also apply the per-point collision response
        obj1.HandleCollisionResponse(info, obj2);
        obj2.HandleCollisionResponse(new CollisionInfo
        {
            Normal = -info.Normal,
            Depth = info.Depth,
            DidCollide = true
        }, obj1);

        // Optional: Trigger mesh deformation
        TriggerMeshDeformation(obj1, obj2);
    }

    // Make this method private (it was already correct, just noting the visibility)
    private void HandleGJKCollisionResponse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // --- 1. Calculate Relative Velocity ---
        Vector3 avgVelocity1 = GetAverageVelocity(obj1);
        Vector3 avgVelocity2 = GetAverageVelocity(obj2);
        Vector3 relativeVelocity = avgVelocity1 - avgVelocity2;
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, info.Normal);

        // Don't resolve if velocities are already separating
        if (velocityAlongNormal > 0) return;

        // --- 2. Calculate Impulse (Velocity Change) ---
        float totalMass1 = obj1.totalMass;
        float totalMass2 = obj2.totalMass;
        float invMass1 = (totalMass1 > 0) ? 1.0f / totalMass1 : 0.0f;
        float invMass2 = (totalMass2 > 0) ? 1.0f / totalMass2 : 0.0f;

        // Use the smaller coefficient of restitution
        float e = coefficientOfRestitution;

        // Impulse scalar formula
        float j = -(1 + e) * velocityAlongNormal;
        j /= (invMass1 + invMass2);

        // Apply impulse to each object
        Vector3 impulse = j * info.Normal;
        Vector3 velocityChange1 = impulse * invMass1;
        Vector3 velocityChange2 = -impulse * invMass2;

        // Distribute velocity change across all points of the soft body
        for (int i=0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj1.allSpringPoints[i];
            point.velocity += (float3)velocityChange1;
            obj1.allSpringPoints[i] = point;
        }
        for (int i = 0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj2.allSpringPoints[i];
            point.velocity += (float3)velocityChange1;
            obj1.allSpringPoints[i] = point;
        }

        // --- 3. Positional Correction (Resolve Penetration) ---
        Vector3 correction = Mathf.Max(info.Depth - penetrationSlop, 0.0f) / (invMass1 + invMass2) * penetrationCorrectionFactor * info.Normal;
        Vector3 posChange1 = correction * invMass1;
        Vector3 posChange2 = -correction * invMass2;

        // Distribute position change across all points
        for (int i = 0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj1.allSpringPoints[i];
            point.position += (float3)posChange1;
            obj1.allSpringPoints[i] = point;
        }
        for (int i = 0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj2.allSpringPoints[i];
            point.position += (float3)posChange2;
            obj1.allSpringPoints[i] = point;
        }
    }

    private Vector3 GetAverageVelocity(OctreeSpringFiller obj)
    {
        if (obj.allSpringPoints.Length == 0) return Vector3.zero;
        Vector3 totalVelocity = Vector3.zero;
        for (int i=0; i < obj.allSpringPoints.Length; i++)
        {
            SpringPointData p = obj.allSpringPoints[i];
            totalVelocity += (Vector3)p.velocity;
            obj.allSpringPoints[i] = p;
        }
        return totalVelocity / obj.allSpringPoints.Length;
    }

    private void OnDrawGizmos()
    {
        if (!showCollisionGizmos || !Application.isPlaying || lastFrameCollisions == null)
            return;

        // Draw the collision normal for each detected collision this frame
        Gizmos.color = Color.red;
        foreach (var info in lastFrameCollisions)
        {
            // Find a representative center point for the gizmo
            Vector3 center = Vector3.Lerp(info.Normal * -info.Depth, Vector3.zero, 0.5f); // Simplified center
            Gizmos.DrawRay(center, info.Normal * 2.0f); // Draw the normal
            Gizmos.DrawWireSphere(center, 0.2f); // Mark the approximate collision spot
        }
    }
}