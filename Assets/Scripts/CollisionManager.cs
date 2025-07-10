using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class CollisionManager : MonoBehaviour
{
    public static List<OctreeSpringFiller> AllSoftBodies = new List<OctreeSpringFiller>();

    [Header("Inter-Object Collision Settings")]
    public bool enableInterObjectCollision = true;

    [Header("Contact-Point Response")]
    public bool enableContactPointResponse = true;
    public float contactRadius = 1.0f;              // How far from contact point to apply forces
    public float contactInfluenceFalloff = 2.0f;    // How quickly influence drops with distance (higher = sharper falloff)
    public int maxContactsPerCollision = 8;         // Limit contacts for performance


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

    // Debug visualization
    [Header("Debug Visualization")]
    public bool showContactPoints = true;
    public bool showInfluenceRadius = true;
    public bool logCollisionDetails = false;

    private List<ContactPoint> currentFrameContacts = new List<ContactPoint>();

    private struct ContactPoint
    {
        public Vector3 worldPosition;
        public Vector3 normal;
        public float penetrationDepth;
        public float impactVelocity;
        public int point1Index;  // Index of point from object 1 involved in contact
        public int point2Index;   // Index of point from object 2 involved in contact
        public float influenceRadius;
    }

    /// <summary>
    /// This is now the primary collision resolution method called in FixedUpdate.
    /// </summary>
    public void ResolveInterObjectCollisions()
    {
        if (!enableInterObjectCollision) return;

        totalCollisionsThisFrame = 0;
        lastFrameCollisions.Clear();
        currentFrameContacts.Clear();

        for (int i = 0; i < AllSoftBodies.Count; i++)
        {
            OctreeSpringFiller obj1 = AllSoftBodies[i];
            obj1.UpdateBoundingVolume();

            for (int j = i + 1; j < AllSoftBodies.Count; j++)
            {
                OctreeSpringFiller obj2 = AllSoftBodies[j];
                obj2.UpdateBoundingVolume();

                // CHECK COLLISION LAYERS FIRST
                if (!obj1.CanCollideWith(obj2))
                {
                    continue; // Skip collision if layers don't interact
                }

                if (!obj1.boundingVolume.Intersects(obj2.boundingVolume)) continue;

                if (GJK.DetectCollision(obj1, obj2, out CollisionInfo info))
                {
                    totalCollisionsThisFrame++;
                    lastFrameCollisions.Add(info);

                    // Apply layer-specific collision properties
                    ApplyLayerSpecificCollision(obj1, obj2, info);

                    if (enableContactPointResponse)
                    {
                        HandleContactPointCollision(obj1, obj2, info);
                    }
                    else
                    {
                        HandleGlobalCollision(obj1, obj2, info);
                    }
                }
            }
        }
    }

    private void ApplyLayerSpecificCollision(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        var layer1 = obj1.GetCollisionLayer();
        var layer2 = obj2.GetCollisionLayer();

        if (layer1 != null && layer2 != null)
        {
            // Blend material properties for collision response
            coefficientOfRestitution = (layer1.restitution + layer2.restitution) * 0.5f;
            coefficientOfFriction = (layer1.friction + layer2.friction) * 0.5f;
        }
    }

    private void HandleContactPointCollision(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Find all contact points between the two objects
        List<ContactPoint> contacts = FindContactPoints(obj1, obj2, info);

        if (logCollisionDetails)
        {
            Debug.Log($"Collision between {obj1.name} and {obj2.name}: {contacts.Count} contact points found");
        }

        // Apply response at each contact point
        foreach (var contact in contacts)
        {
            ApplyContactPointResponse(obj1, obj2, contact);
            currentFrameContacts.Add(contact); // For debug visualization
        }
    }

    private List<ContactPoint> FindContactPoints(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        List<ContactPoint> contacts = new List<ContactPoint>();

        // Method 1: Use EPA result for primary contact (most accurate)
        Vector3 primaryContactPos = GetPrimaryContactPosition(obj1, obj2, info);
        ContactPoint primaryContact = CreatePrimaryContact(obj1, obj2, info, primaryContactPos);
        if (primaryContact.point1Index >= 0 && primaryContact.point2Index >= 0)
        {
            contacts.Add(primaryContact);
        }

        // Method 2: Find additional penetrating surface points for multiple contact points
        List<ContactPoint> penetratingContacts = FindPenetratingContacts(obj1, obj2, info);

        // Merge and filter contacts to avoid duplicates
        foreach (var contact in penetratingContacts)
        {
            if (!IsContactDuplicate(contact, contacts))
            {
                contacts.Add(contact);
            }
        }

        // Limit number of contacts for performance and stability
        if (contacts.Count > maxContactsPerCollision)
        {
            contacts = SelectBestContacts(contacts);
        }

        return contacts;
    }

    private Vector3 GetPrimaryContactPosition(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Use EPA penetration vector to estimate contact location
        // The contact should be roughly at the midpoint of the penetration
        Vector3 obj1Center = GetObjectCenter(obj1);
        Vector3 obj2Center = GetObjectCenter(obj2);

        // Move from obj1 center along the collision normal by half the penetration depth
        Vector3 contactPos = obj1Center + info.Normal * (info.Depth * 0.5f);

        return contactPos;
    }

    private ContactPoint CreatePrimaryContact(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info, Vector3 contactWorldPos)
    {
        // Find closest surface points to the estimated contact location
        int closest1 = FindClosestSurfacePointIndex(obj1, contactWorldPos);
        int closest2 = FindClosestSurfacePointIndex(obj2, contactWorldPos);

        ContactPoint contact = new ContactPoint();

        if (closest1 >= 0 && closest2 >= 0)
        {
            SpringPointData point1 = obj1.SurfacePoints[closest1];
            SpringPointData point2 = obj2.SurfacePoints[closest2];

            Vector3 relativeVel = point1.velocity - point2.velocity;
            float impactSpeed = Vector3.Dot(relativeVel, info.Normal);

            contact = new ContactPoint
            {
                worldPosition = Vector3.Lerp(point1.position, point2.position, 0.5f),
                normal = info.Normal, // Use EPA normal for accuracy
                penetrationDepth = info.Depth, // Use EPA depth for accuracy
                impactVelocity = impactSpeed,
                point1Index = closest1,
                point2Index = closest2,
                influenceRadius = contactRadius
            };
        }
        else
        {
            // Fallback if surface points aren't found
            contact.point1Index = -1;
            contact.point2Index = -1;
        }

        return contact;
    }

    private List<ContactPoint> FindPenetratingContacts(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        List<ContactPoint> penetratingContacts = new List<ContactPoint>();

        // Find surface points of obj1 that are penetrating obj2
        for (int i = 0; i < obj1.SurfacePoints.Count; i++)
        {
            SpringPointData point1 = obj1.SurfacePoints[i];
            Vector3 localPoint = obj2.transform.InverseTransformPoint(point1.position);

            if (obj2.IsPointInside(localPoint))
            {
                ContactPoint contact = CreateContactFromPenetration(obj1, obj2, i, true, info);
                if (contact.point2Index >= 0)
                {
                    penetratingContacts.Add(contact);
                }
            }
        }

        // Find surface points of obj2 that are penetrating obj1
        for (int i = 0; i < obj2.SurfacePoints.Count; i++)
        {
            SpringPointData point2 = obj2.SurfacePoints[i];
            Vector3 localPoint = obj1.transform.InverseTransformPoint(point2.position);

            if (obj1.IsPointInside(localPoint))
            {
                ContactPoint contact = CreateContactFromPenetration(obj1, obj2, i, false, info);
                if (contact.point1Index >= 0)
                {
                    penetratingContacts.Add(contact);
                }
            }
        }

        return penetratingContacts;
    }

    private ContactPoint CreateContactFromPenetration(OctreeSpringFiller obj1, OctreeSpringFiller obj2,
                                                      int penetratingPointIndex, bool isObj1Penetrating, CollisionInfo info)
    {
        ContactPoint contact = new ContactPoint();

        if (isObj1Penetrating)
        {
            // obj1's point is penetrating obj2
            SpringPointData point1 = obj1.SurfacePoints[penetratingPointIndex];
            int closestIndex = FindClosestSurfacePointIndex(obj2, point1.position);

            if (closestIndex >= 0)
            {
                SpringPointData closestPoint2 = obj2.SurfacePoints[closestIndex];
                float penetration = Vector3.Distance(point1.position, closestPoint2.position);

                // Use a blend between local normal and EPA normal for better stability
                Vector3 localNormal = (point1.position - closestPoint2.position);
                Vector3 blendedNormal = Vector3.Slerp(localNormal, info.Normal, 0.7f).normalized;

                Vector3 relativeVel = point1.velocity - closestPoint2.velocity;
                float impactSpeed = Vector3.Dot(relativeVel, blendedNormal);

                contact = new ContactPoint
                {
                    worldPosition = Vector3.Lerp(point1.position, closestPoint2.position, 0.5f),
                    normal = blendedNormal,
                    penetrationDepth = penetration,
                    impactVelocity = impactSpeed,
                    point1Index = penetratingPointIndex,
                    point2Index = closestIndex,
                    influenceRadius = contactRadius
                };
            }
            else
            {
                contact.point2Index = -1;
            }
        }
        else
        {
            // obj2's point is penetrating obj1
            SpringPointData point2 = obj2.SurfacePoints[penetratingPointIndex];
            int closestIndex = FindClosestSurfacePointIndex(obj1, point2.position);

            if (closestIndex >= 0)
            {
                SpringPointData closestPoint1 = obj1.SurfacePoints[closestIndex];
                float penetration = Vector3.Distance(point2.position, closestPoint1.position);

                // Use a blend between local normal and EPA normal for better stability
                Vector3 localNormal = (closestPoint1.position - point2.position);
                Vector3 blendedNormal = Vector3.Slerp(localNormal, info.Normal, 0.7f).normalized;

                Vector3 relativeVel = closestPoint1.velocity - point2.velocity;
                float impactSpeed = Vector3.Dot(relativeVel, blendedNormal);

                contact = new ContactPoint
                {
                    worldPosition = Vector3.Lerp(closestPoint1.position, point2.position, 0.5f),
                    normal = blendedNormal,
                    penetrationDepth = penetration,
                    impactVelocity = impactSpeed,
                    point1Index = closestIndex,
                    point2Index = penetratingPointIndex,
                    influenceRadius = contactRadius
                };
            }
            else
            {
                contact.point1Index = -1;
            }
        }

        return contact;
    }

    private bool IsContactDuplicate(ContactPoint newContact, List<ContactPoint> existingContacts)
    {
        foreach (var existing in existingContacts)
        {
            // Check both position and point indices for duplicates
            float positionDistance = Vector3.Distance(existing.worldPosition, newContact.worldPosition);
            bool samePoints = (existing.point1Index == newContact.point1Index && existing.point2Index == newContact.point2Index) ||
                             (existing.point1Index == newContact.point2Index && existing.point2Index == newContact.point1Index);

            if (positionDistance < 0.1f || samePoints)
            {
                return true;
            }
        }
        return false;
    }

    private List<ContactPoint> SelectBestContacts(List<ContactPoint> contacts)
    {
        // Sort contacts by multiple criteria for best selection
        contacts.Sort((a, b) => {
            // Primary: Highest impact velocity (most significant collision)
            int impactComparison = b.impactVelocity.CompareTo(a.impactVelocity);
            if (impactComparison != 0) return impactComparison;

            // Secondary: Highest penetration depth (most urgent to resolve)
            int penetrationComparison = b.penetrationDepth.CompareTo(a.penetrationDepth);
            if (penetrationComparison != 0) return penetrationComparison;

            // Tertiary: Spread contacts spatially for stability
            return 0;
        });

        // Take the top contacts, but ensure spatial distribution
        List<ContactPoint> selectedContacts = new List<ContactPoint>();
        selectedContacts.Add(contacts[0]); // Always take the most significant contact

        for (int i = 1; i < contacts.Count && selectedContacts.Count < maxContactsPerCollision; i++)
        {
            ContactPoint candidate = contacts[i];
            bool tooClose = false;

            // Ensure minimum distance between selected contacts for stability
            foreach (var selected in selectedContacts)
            {
                if (Vector3.Distance(candidate.worldPosition, selected.worldPosition) < contactRadius * 0.5f)
                {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose)
            {
                selectedContacts.Add(candidate);
            }
        }

        return selectedContacts;
    }

    private Vector3 GetObjectCenter(OctreeSpringFiller obj)
    {
        // Calculate center of mass or geometric center
        Vector3 center = Vector3.zero;
        float totalMass = 0f;

        foreach (var point in obj.allSpringPoints)
        {
            center += (Vector3)point.position * point.mass;
            totalMass += point.mass;
        }

        return totalMass > 0 ? center / totalMass : obj.transform.position;
    }

    private int FindClosestSurfacePointIndex(OctreeSpringFiller body, Vector3 worldPosition)
    {
        int closestIndex = -1;
        float minDistance = float.MaxValue;

        for (int i = 0; i < body.SurfacePoints.Count; i++)
        {
            float distance = Vector3.Distance(body.SurfacePoints[i].position, worldPosition);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    private void ApplyContactPointResponse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        SpringPointData point1 = obj1.SurfacePoints[contact.point1Index];
        SpringPointData point2 = obj2.SurfacePoints[contact.point2Index];

        // Calculate impulse magnitude for this contact
        float impulseMagnitude = CalculateContactImpulse(obj1, obj2, contact);
        Vector3 impulse = contact.normal * impulseMagnitude;

        if (point1.isFixed == 0)
        {
            point1.velocity += (float3)impulse / point1.mass;
            obj1.SurfacePoints[contact.point1Index] = point1;
        }

        if (point2.isFixed == 0)
        {
            point2.velocity -= (float3)impulse / point2.mass;
            obj2.SurfacePoints[contact.point2Index] = point2;
        }

        // Apply impulse to nearby points in both objects
        ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, impulse, contact.influenceRadius);
        ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -impulse, contact.influenceRadius);

        // Apply friction if enabled
        if (coefficientOfFriction > 0.001f)
        {
            Vector3 frictionImpulse = CalculateFrictionImpulse(obj1, obj2, contact, impulseMagnitude);
            ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, frictionImpulse, contact.influenceRadius);
            ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -frictionImpulse, contact.influenceRadius);
        }

        // Position correction to resolve penetration
        ApplyPositionCorrection(obj1, obj2, contact);
    }

    private float CalculateContactImpulse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        SpringPointData point1 = obj1.SurfacePoints[contact.point1Index];
        SpringPointData point2 = obj2.SurfacePoints[contact.point2Index];

        // Calculate relative velocity at contact point
        Vector3 relativeVelocity = point1.velocity - point2.velocity;
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, contact.normal);

        // Don't resolve if already separating
        if (velocityAlongNormal > 0) return 0f;

        float invMass1 = point1.mass > 0 ? 1.0f / point1.mass : 0.0f;
        float invMass2 = point2.mass > 0 ? 1.0f / point2.mass : 0.0f;

        // Calculate impulse magnitude
        float j = -(1 + coefficientOfRestitution) * velocityAlongNormal;
        j /= (invMass1 + invMass2);

        return j;
    }

    private Vector3 CalculateFrictionImpulse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact, float normalImpulseMagnitude)
    {
        SpringPointData point1 = obj1.SurfacePoints[contact.point1Index];
        SpringPointData point2 = obj2.SurfacePoints[contact.point2Index];

        // Get tangential velocity
        Vector3 relativeVelocity = point1.velocity - point2.velocity;
        Vector3 tangentialVelocity = relativeVelocity - Vector3.Dot(relativeVelocity, contact.normal) * contact.normal;

        if (tangentialVelocity.magnitude < 0.001f) return Vector3.zero;

        Vector3 tangentDirection = tangentialVelocity.normalized;
        float frictionImpulse = coefficientOfFriction * normalImpulseMagnitude;

        // Clamp friction to not exceed available tangential velocity
        float maxFrictionImpulse = tangentialVelocity.magnitude * (point1.mass + point2.mass) * 0.5f;
        frictionImpulse = Mathf.Min(frictionImpulse, maxFrictionImpulse);

        return -tangentDirection * frictionImpulse;
    }

    private void ApplyImpulseToNearbyPoints(OctreeSpringFiller body, Vector3 contactPoint, Vector3 impulse, float influenceRadius)
    {
        for (int i = 0; i < body.allSpringPoints.Length; i++)
        {
            SpringPointData point = body.allSpringPoints[i];
            float distance = Vector3.Distance(point.position, contactPoint);

            if (distance < influenceRadius && point.isFixed == 0)
            {
                // Calculate influence based on distance with falloff
                float normalizedDistance = distance / influenceRadius;
                float influence = CalculateInfluenceFalloff(normalizedDistance);
                Vector3 velocityChange = impulse * influence / point.mass;
                point.velocity += (float3)velocityChange;
                body.allSpringPoints[i] = point;

                if (logCollisionDetails && influence > 0.1f)
                {
                    Debug.Log($"Applied impulse {velocityChange.magnitude:F2} to point at distance {distance:F2} (influence: {influence:F2})");
                }
            }
        }
    }

    private float CalculateInfluenceFalloff(float normalizedDistance)
    {
        // Different falloff curves - you can experiment with these
        switch ((int)contactInfluenceFalloff)
        {
            case 1: // Linear falloff
                return 1.0f - normalizedDistance;

            case 2: // Quadratic falloff (default)
                return (1.0f - normalizedDistance) * (1.0f - normalizedDistance);

            case 3: // Cubic falloff (sharp)
                float invDist = 1.0f - normalizedDistance;
                return invDist * invDist * invDist;

            case 4: // Smooth step
                return Mathf.SmoothStep(1.0f, 0.0f, normalizedDistance);

            default:
                return Mathf.Pow(1.0f - normalizedDistance, contactInfluenceFalloff);
        }
    }

    private void ApplyPositionCorrection(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        if (contact.penetrationDepth <= penetrationSlop) return;

        SpringPointData point1 = obj1.SurfacePoints[contact.point1Index];
        SpringPointData point2 = obj2.SurfacePoints[contact.point2Index];

        // Calculate correction amount
        float correctionAmount = (contact.penetrationDepth - penetrationSlop) * penetrationCorrectionFactor;
        Vector3 correction = contact.normal * correctionAmount;

        // Calculate mass ratio for correction distribution
        float totalMass = point1.mass + point2.mass;
        float mass1Ratio = point2.mass / totalMass; // obj1 gets more correction if obj2 is heavier
        float mass2Ratio = point1.mass / totalMass; // obj2 gets more correction if obj1 is heavier

        Vector3 correction1 = correction * mass1Ratio;
        Vector3 correction2 = -correction * mass2Ratio;

        // Apply position correction to nearby points
        ApplyPositionCorrectionToNearbyPoints(obj1, contact.worldPosition, correction1, contact.influenceRadius);
        ApplyPositionCorrectionToNearbyPoints(obj2, contact.worldPosition, correction2, contact.influenceRadius);
    }

    private void ApplyPositionCorrectionToNearbyPoints(OctreeSpringFiller body, Vector3 contactPoint, Vector3 correction, float influenceRadius)
    {
        for (int i = 0; i < body.allSpringPoints.Length; i++)
        {
            SpringPointData point = body.allSpringPoints[i];
            float distance = Vector3.Distance(point.position, contactPoint);

            if (distance < influenceRadius && point.isFixed == 0)
            {
                float normalizedDistance = distance / influenceRadius;
                float influence = CalculateInfluenceFalloff(normalizedDistance);
                point.position += (float3)correction * influence;
                body.allSpringPoints[i] = point;
            }
        }
    }

    // Fallback to original global collision response
    private void HandleGlobalCollision(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Your original collision response code here
        obj1.HandleCollisionResponse(info, obj2);
        obj2.HandleCollisionResponse(new CollisionInfo
        {
            Normal = -info.Normal,
            Depth = info.Depth,
            DidCollide = true
        }, obj1);
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

        // Apply velocity changes to obj1
        for (int i = 0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj1.allSpringPoints[i];
            point.velocity += (float3)velocityChange1;
            obj1.allSpringPoints[i] = point;
        }

        // Apply velocity changes to obj2  
        for (int i = 0; i < obj2.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj2.allSpringPoints[i];
            point.velocity += (float3)velocityChange2;
            obj2.allSpringPoints[i] = point;
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

    private float3 GetAverageVelocity(OctreeSpringFiller obj)
    {
        if (obj.allSpringPoints.Length == 0) return float3.zero;

        float3 totalVelocity = float3.zero;
        int movablePoints = 0;

        for (int i = 0; i < obj.allSpringPoints.Length; i++)
        {
            SpringPointData p = obj.allSpringPoints[i];
            if (p.isFixed == 0)
            {
                totalVelocity += p.velocity;
                movablePoints++;
            }
        }

        return movablePoints > 0 ? totalVelocity / movablePoints : float3.zero;
    }

    private void DebugDrawBounds(Bounds bounds, Color color)
    {
        Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z), new Vector3(bounds.max.x, bounds.min.y, bounds.min.z), color);
        Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z), new Vector3(bounds.min.x, bounds.max.y, bounds.min.z), color);
        Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z), new Vector3(bounds.min.x, bounds.min.y, bounds.max.z), color);
        Debug.DrawLine(new Vector3(bounds.max.x, bounds.max.y, bounds.max.z), new Vector3(bounds.min.x, bounds.max.y, bounds.max.z), color);
        Debug.DrawLine(new Vector3(bounds.max.x, bounds.max.y, bounds.max.z), new Vector3(bounds.max.x, bounds.min.y, bounds.max.z), color);
        Debug.DrawLine(new Vector3(bounds.max.x, bounds.max.y, bounds.max.z), new Vector3(bounds.max.x, bounds.max.y, bounds.min.z), color);
    }

    private void OnDrawGizmos()
    {
        if (!showCollisionGizmos || !Application.isPlaying || !enableInterObjectCollision || lastFrameCollisions == null)
            return;

        if (showContactPoints)
        {
            Gizmos.color = Color.red;
            foreach (var contact in currentFrameContacts)
            {
                Gizmos.DrawSphere(contact.worldPosition, 0.05f);
                Gizmos.DrawRay(contact.worldPosition, contact.normal * 0.5f);
            }
        }

        if (showInfluenceRadius)
        {
            Gizmos.color = Color.yellow;
            foreach (var contact in currentFrameContacts)
            {
                Gizmos.DrawWireSphere(contact.worldPosition, contact.influenceRadius);
            }
        }

        Gizmos.color = Color.cyan;
        foreach (var info in lastFrameCollisions)
        {
            Vector3 center = Vector3.Lerp(info.Normal * -info.Depth, Vector3.zero, 0.5f);
            Gizmos.DrawRay(center, info.Normal * 2.0f);
            Gizmos.DrawWireSphere(center, 0.2f);
        }
    }

    void OnGUI()
    {
        GUILayout.Label($"Collisions: {totalCollisionsThisFrame}");
        GUILayout.Label($"Bodies: {AllSoftBodies.Count}");

        foreach (var body in AllSoftBodies)
        {
            GUILayout.Label($"{body.name}: Surface={body.SurfacePoints.Count}");
        }
    }
}