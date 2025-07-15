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
    public float baseContactRadius = 1.0f;              // Base radius for force application
    public float contactInfluenceFalloff = 2.0f;    // Sharpness of falloff (higher = sharper)
    public int maxContactsPerCollision = 8;         // Limit contacts for performance
    public float radiusDepthScale = 0.5f;           // How much penetration scales the radius

    [Header("Collision Response")]
    public float contactStiffness = 500f;  // Stiffness for penalty repulsion (N/m)
    [Range(0f, 2f)]
    public float contactDampingRatio = 0.8f;  // Damping ratio (1.0 = critical damping)
    [Range(0f, 1f)]
    public float viscousFrictionFactor = 0.5f;  // Scales tangential damping relative to friction coeff
    [Range(0f, 1f)]
    public float defaultCoefficientOfRestitution = 0.6f;  // Default if points don't have values
    [Range(0f, 1f)]
    public float defaultCoefficientOfFriction = 0.4f;     // Default if points don't have values
    public float penetrationCorrectionFactor = 0.3f; // How strongly to push objects apart (0-1)
    public float penetrationSlop = 0.02f; // A small allowance for penetration to prevent jitter

    // Collision statistics
    [Header("Debug Info")]
    public int totalCollisionsThisFrame = 0;
    public bool showCollisionGizmos = false;
    private List<CollisionInfo> lastFrameCollisions = new List<CollisionInfo>();
    private List<ContactPoint> currentFrameContacts = new List<ContactPoint>();

    // Debug visualization
    [Header("Debug Visualization")]
    public bool showContactPoints = true;
    public bool showInfluenceRadius = true;
    public bool logCollisionDetails = false;


    private struct ContactPoint
    {
        public Vector3 worldPosition;
        public Vector3 normal;

        public float penetrationDepth;
        public float impactVelocity;

        public int point1Index;  // Index of point from object 1 involved in contact
        public int point2Index;   // Index of point from object 2 involved in contact

        public float influenceRadius;
        public float restitution;  // Blended restitution for this contact
        public float friction;     // Blended friction for this contact
    }

    private struct MergeSpring
    {
        public OctreeSpringFiller body1;
        public OctreeSpringFiller body2;
        public int point1Index;
        public int point2Index;
        public float restLength;
        public float springConstant;
        public float damperConstant;
    }

    private List<MergeSpring> tempMergeSprings = new List<MergeSpring>();

    // NEW: Spatial Hash for broad-phase optimization
    private SpatialHash<OctreeSpringFiller> bodySpatialHash;
    private float hashCellSize = 10f; // Adjust based on typical body size; larger for bigger scenes
    void Awake()
    {
        // Initialize spatial hash with a reasonable cell size
        bodySpatialHash = new SpatialHash<OctreeSpringFiller>(hashCellSize);
    }

    void FixedUpdate()
    {
        // Remove temporary merge springs that are no longer needed
        for (int i = tempMergeSprings.Count - 1; i >= 0; i--)
        {
            MergeSpring mergeSpring = tempMergeSprings[i];
            Vector3 pos1 = mergeSpring.body1.allSpringPoints[mergeSpring.point1Index].position;
            Vector3 pos2 = mergeSpring.body2.allSpringPoints[mergeSpring.point2Index].position;
            float currentDist = Vector3.Distance(pos1, pos2);

            if (currentDist > mergeSpring.restLength * 1.5f) // Separated
            {
                tempMergeSprings.RemoveAt(i);
            }
        }
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

        // NEW: Clear and rebuild spatial hash each frame (since bodies move)
        bodySpatialHash.Clear();
        foreach (var body in AllSoftBodies)
        {
            body.UpdateBoundingVolume(); // Ensure bounds are up-to-date
            bodySpatialHash.Add(body.boundingVolume.center, body);
        }

        // NEW: Use spatial hash to find potential pairs instead of O(n^2) loop
        HashSet<KeyValuePair<int, int>> checkedPairs = new HashSet<KeyValuePair<int, int>>(); // Avoid duplicates
        foreach (var body1 in AllSoftBodies)
        {
            // Query nearby bodies using the bounding volume extent
            float queryRadius = body1.boundingVolume.extents.magnitude * 2f; // Conservative radius
            var nearbyBodies = bodySpatialHash.Query(body1.boundingVolume.center, queryRadius);

            foreach (var body2 in nearbyBodies)
            {
                if (body1 == body2) continue;

                // Ensure we don't check pairs twice (i < j)
                int id1 = AllSoftBodies.IndexOf(body1);
                int id2 = AllSoftBodies.IndexOf(body2);
                if (id1 > id2) continue; // Skip if already would have been checked from the other side

                var pair = new KeyValuePair<int, int>(Mathf.Min(id1, id2), Mathf.Max(id1, id2));
                if (checkedPairs.Contains(pair)) continue;
                checkedPairs.Add(pair);

                // CHECK COLLISION LAYERS FIRST
                if (!body1.CanCollideWith(body2))
                {
                    continue; // Skip collision if layers don't interact
                }

                if (!body1.boundingVolume.Intersects(body2.boundingVolume)) continue;

                if (GJK.DetectCollision(body1, body2, out CollisionInfo info))
                {
                    totalCollisionsThisFrame++;
                    lastFrameCollisions.Add(info);

                    if (enableContactPointResponse)
                    {
                        HandleContactPointCollision(body1, body2, info);
                    }
                    else
                    {
                        HandleGlobalCollision(body1, body2, info);
                    }
                }
            }
        }
    }

    private void HandleContactPointCollision(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Find all contact points between the two objects
        List<ContactPoint> contacts = FindContactPoints(obj1, obj2, info);

        if (logCollisionDetails)
        {
            // Debug.Log($"Collision between {obj1.name} and {obj2.name}: {contacts.Count} contact points found");
        }

        // Calculate pre-collision KE before applying responses
        float preKE1 = obj1.GetTotalKineticEnergy();
        float preKE2 = obj2.GetTotalKineticEnergy();

        if (ShouldMerge(obj1, obj2, info))
        {
            CreateTemporaryMergeSprings(obj1, obj2, contacts);
        }

        // Apply response at each contact point
        foreach (var contact in contacts)
        {
            GG(obj1, obj2, contact);
            //ApplyContactPointResponse(obj1, obj2, contact);
            currentFrameContacts.Add(contact); // For debug visualization
        }

        VerifyEnergyConservation(obj1, obj2, contacts, preKE1, preKE2);
    }

    private void CreateTemporaryMergeSprings(OctreeSpringFiller obj1, OctreeSpringFiller obj2, List<ContactPoint> contacts)
    {
        foreach (var contact in contacts)
        {
            // Connect closest points across bodies with weak springs
            int p1 = contact.point1Index;
            int p2 = contact.point2Index;

            float dist = Vector3.Distance(obj1.allSpringPoints[p1].position, obj2.allSpringPoints[p2].position);

            tempMergeSprings.Add(new MergeSpring
            {
                body1 = obj1,
                body2 = obj2,
                point1Index = p1,
                point2Index = p2,
                restLength = dist,
                springConstant = 10f,
                damperConstant = 0.1f
            });
        }
    }

    private bool ShouldMerge(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Check material softness (e.g., low Young's modulus from layer)
        return obj1.collisionLayer.youngsModulus < 1e6f && // Soft threshold
               obj2.collisionLayer.youngsModulus < 1e6f &&
               info.Depth > 0.1f; // Deep penetration
    }

    private void VerifyEnergyConservation(OctreeSpringFiller obj1, OctreeSpringFiller obj2, List<ContactPoint> contacts, float preKE1, float preKE2)
    {
        // After applying impulses, get post KE
        float postKE1 = obj1.GetTotalKineticEnergy();
        float postKE2 = obj2.GetTotalKineticEnergy();

        // If post > pre (energy gain), dissipate excess proportionally
        float excess = Mathf.Max(0, (postKE1 + postKE2) - (preKE1 + preKE2));
        if (excess > 0)
        {
            float dampFactor = Mathf.Pow(0.95f, excess / (preKE1 + preKE2)); // Exponential damp
            obj1.DampenVelocities(dampFactor);
            obj2.DampenVelocities(dampFactor);
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
        Vector3 contactPos = obj1Center + (Vector3)info.Normal * (info.Depth * 0.5f);

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
            SpringPointData point1 = obj1.surfaceSpringPoints2[closest1];
            SpringPointData point2 = obj2.surfaceSpringPoints2[closest2];

            Vector3 relativeVel = point1.velocity - point2.velocity;
            float impactSpeed = Vector3.Dot(relativeVel, info.Normal);

            // Dynamic radius based on penetration depth
            float dynamicRadius = baseContactRadius * (1f + info.Depth * radiusDepthScale);

            // Blend material properties from both contact points
            float blendedRestitution = (point1.bounciness + point2.bounciness) * 0.5f;
            float blendedFriction = (point1.friction + point2.friction) * 0.5f;

            contact = new ContactPoint
            {
                worldPosition = Vector3.Lerp(point1.position, point2.position, 0.5f),
                normal = info.Normal, // Use EPA normal for accuracy
                penetrationDepth = info.Depth, // Use EPA depth for accuracy
                impactVelocity = impactSpeed,
                point1Index = closest1,
                point2Index = closest2,
                influenceRadius = dynamicRadius,
                restitution = blendedRestitution,
                friction = blendedFriction
            };
        }
        else
        {
            // Fallback if surface points aren't found
            contact.point1Index = -1;
            contact.point2Index = -1;
            contact.restitution = defaultCoefficientOfRestitution;
            contact.friction = defaultCoefficientOfFriction;
        }

        return contact;
    }

    private List<ContactPoint> FindPenetratingContacts(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        List<ContactPoint> penetratingContacts = new List<ContactPoint>();

        // Find surface points of obj1 that are penetrating obj2
        for (int i = 0; i < obj1.surfaceSpringPoints2.Length; i++)
        {
            SpringPointData point1 = obj1.surfaceSpringPoints2[i];
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
        for (int i = 0; i < obj2.surfaceSpringPoints2.Length; i++)
        {
            SpringPointData point2 = obj2.surfaceSpringPoints2[i];
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
            SpringPointData point1 = obj1.surfaceSpringPoints2[penetratingPointIndex];
            int closestIndex = FindClosestSurfacePointIndex(obj2, point1.position);

            if (closestIndex >= 0)
            {
                SpringPointData closestPoint2 = obj2.surfaceSpringPoints2[closestIndex];
                float penetration = Vector3.Distance(point1.position, closestPoint2.position);

                // Use a blend between local normal and EPA normal for better stability
                Vector3 localNormal = (point1.position - closestPoint2.position);
                Vector3 blendedNormal = Vector3.Slerp(localNormal, info.Normal, 0.7f).normalized;

                Vector3 relativeVel = point1.velocity - closestPoint2.velocity;
                float impactSpeed = Vector3.Dot(relativeVel, blendedNormal);

                // Dynamic radius
                float dynamicRadius = baseContactRadius * (1f + penetration * radiusDepthScale);

                // Blend material properties
                float blendedRestitution = (point1.bounciness + closestPoint2.bounciness) * 0.5f;
                float blendedFriction = (point1.friction + closestPoint2.friction) * 0.5f;

                contact = new ContactPoint
                {
                    worldPosition = Vector3.Lerp(point1.position, closestPoint2.position, 0.5f),
                    normal = blendedNormal,
                    penetrationDepth = penetration,
                    impactVelocity = impactSpeed,
                    point1Index = penetratingPointIndex,
                    point2Index = closestIndex,
                    influenceRadius = dynamicRadius,
                    restitution = blendedRestitution,
                    friction = blendedFriction
                };
            }
            else
            {
                contact.point2Index = -1;
                contact.restitution = defaultCoefficientOfRestitution;
                contact.friction = defaultCoefficientOfFriction;
            }
        }
        else
        {
            // obj2's point is penetrating obj1
            SpringPointData point2 = obj2.surfaceSpringPoints2[penetratingPointIndex];
            int closestIndex = FindClosestSurfacePointIndex(obj1, point2.position);

            if (closestIndex >= 0)
            {
                SpringPointData closestPoint1 = obj1.surfaceSpringPoints2[closestIndex];
                float penetration = Vector3.Distance(point2.position, closestPoint1.position);

                // Use a blend between local normal and EPA normal for better stability
                Vector3 localNormal = (closestPoint1.position - point2.position);
                Vector3 blendedNormal = Vector3.Slerp(localNormal, info.Normal, 0.7f).normalized;

                Vector3 relativeVel = closestPoint1.velocity - point2.velocity;
                float impactSpeed = Vector3.Dot(relativeVel, blendedNormal);

                // Dynamic radius
                float dynamicRadius = baseContactRadius * (1f + penetration * radiusDepthScale);

                // Blend material properties
                float blendedRestitution = (closestPoint1.bounciness + point2.bounciness) * 0.5f;
                float blendedFriction = (closestPoint1.friction + point2.friction) * 0.5f;

                contact = new ContactPoint
                {
                    worldPosition = Vector3.Lerp(closestPoint1.position, point2.position, 0.5f),
                    normal = blendedNormal,
                    penetrationDepth = penetration,
                    impactVelocity = impactSpeed,
                    point1Index = closestIndex,
                    point2Index = penetratingPointIndex,
                    influenceRadius = dynamicRadius,
                    restitution = blendedRestitution,
                    friction = blendedFriction
                };
            }
            else
            {
                contact.point1Index = -1;
                contact.restitution = defaultCoefficientOfRestitution;
                contact.friction = defaultCoefficientOfFriction;
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
        contacts.Sort((a, b) =>
        {
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
                if (Vector3.Distance(candidate.worldPosition, selected.worldPosition) < candidate.influenceRadius * 0.5f)
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

        for (int i = 0; i < body.surfaceSpringPoints2.Length; i++)
        {
            float distance = Vector3.Distance(body.surfaceSpringPoints2[i].position, worldPosition);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    private void GG(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        SpringPointData point1 = obj1.surfaceSpringPoints2[contact.point1Index];
        SpringPointData point2 = obj2.surfaceSpringPoints2[contact.point2Index];

        if (contact.normal == Vector3.zero) return; // Simple Unnecessary Check

        float impulseMagnitude = CalculateContactImpulse(obj1, obj2, contact);
        Vector3 impulse = contact.normal * impulseMagnitude;

        if (point1.isFixed == 0)
        {
            point1.velocity += (float3)(impulse / point1.mass);
            obj1.surfaceSpringPoints2[contact.point1Index] = point1;
        }

        if (point2.isFixed == 0)
        {
            point2.velocity -= (float3)(impulse / point2.mass);
            obj2.surfaceSpringPoints2[contact.point2Index] = point2;
        }

        // Apply impulse to nearby points in both objects
        ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, impulse, contact.influenceRadius, contact.normal, contact.restitution);
        ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -impulse, contact.influenceRadius, -contact.normal, contact.restitution);

        // Apply friction if the contact friction is significant
        if (contact.friction > 0.001f)
        {
            Vector3 frictionImpulse = CalculateFrictionImpulse(obj1, obj2, contact, impulseMagnitude);
            ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, frictionImpulse, contact.influenceRadius, contact.normal, contact.friction);
            ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -frictionImpulse, contact.influenceRadius, -contact.normal, contact.friction);
        }
        if (contact.penetrationDepth <= penetrationSlop) return;
        float correctionAmount = (contact.penetrationDepth - penetrationSlop) * penetrationCorrectionFactor;
        Vector3 correction = contact.normal * correctionAmount;


        // Compute effective invMass based on nearby points
        float effectiveInvMass1 = CalculateEffectiveInvMass(obj1, contact.worldPosition, contact.influenceRadius, contact.normal);
        float effectiveInvMass2 = CalculateEffectiveInvMass(obj2, contact.worldPosition, contact.influenceRadius, -contact.normal);
        float totalEffectiveInvMass = effectiveInvMass1 + effectiveInvMass2;

        if (totalEffectiveInvMass == 0f) return;

        Vector3 correction1 = -correction * (effectiveInvMass1 / totalEffectiveInvMass);
        Vector3 correction2 = correction * (effectiveInvMass2 / totalEffectiveInvMass);

        ApplyPositionCorrectionToNearbyPoints(obj1, contact.worldPosition, correction1, contact.influenceRadius, contact.normal);
        ApplyPositionCorrectionToNearbyPoints(obj2, contact.worldPosition, correction2, contact.influenceRadius, -contact.normal);
    }

    private void ApplyContactPointResponse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        SpringPointData point1 = obj1.surfaceSpringPoints2[contact.point1Index];
        SpringPointData point2 = obj2.surfaceSpringPoints2[contact.point2Index];

        // Calculate impulse magnitude for this contact using per-point material properties
        float impulseMagnitude = CalculateContactImpulse(obj1, obj2, contact);
        Vector3 impulse = contact.normal * impulseMagnitude;

        if (point1.isFixed == 0)
        {
            point1.velocity += (float3)(impulse / point1.mass);
            obj1.surfaceSpringPoints2[contact.point1Index] = point1;
        }

        if (point2.isFixed == 0)
        {
            point2.velocity -= (float3)(impulse / point2.mass);
            obj2.surfaceSpringPoints2[contact.point2Index] = point2;
        }

        // Apply impulse to nearby points in both objects
        ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, impulse, contact.influenceRadius, contact.normal, contact.restitution);
        ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -impulse, contact.influenceRadius, -contact.normal, contact.restitution);

        // Apply friction if the contact friction is significant
        if (contact.friction > 0.001f)
        {
            Vector3 frictionImpulse = CalculateFrictionImpulse(obj1, obj2, contact, impulseMagnitude);
            ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, frictionImpulse, contact.influenceRadius, contact.normal, contact.friction);
            ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -frictionImpulse, contact.influenceRadius, -contact.normal, contact.friction);
        }

        // Position correction to resolve penetration
        ApplyPositionCorrection(obj1, obj2, contact);
        
        // Continuous penalty forces and damping for sustained contacts
        ApplyContinuousContactForces(obj1, obj2, contact);

        // Position correction to resolve penetration (existing)
        ApplyPositionCorrection(obj1, obj2, contact);
    }

    private void ApplyContinuousContactForces(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        SpringPointData point1 = obj1.surfaceSpringPoints2[contact.point1Index];
        SpringPointData point2 = obj2.surfaceSpringPoints2[contact.point2Index];

        Vector3 relativeVelocity = point1.velocity - point2.velocity;
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, contact.normal);

        float penetration = Mathf.Max(contact.penetrationDepth - penetrationSlop, 0f);
        if (penetration <= 0f) return;  // No force if not penetrating

        // Effective inverse masses for adaptive stiffness/damping
        float effectiveInvMass1 = CalculateEffectiveInvMass(obj1, contact.worldPosition, contact.influenceRadius, contact.normal);
        float effectiveInvMass2 = CalculateEffectiveInvMass(obj2, contact.worldPosition, contact.influenceRadius, -contact.normal);
        float effectiveInvMass = effectiveInvMass1 + effectiveInvMass2;
        if (effectiveInvMass <= 0f) return;
        float effectiveMass = 1f / effectiveInvMass;

        // Blend stiffness with restitution (softer for bouncier materials)
        float e = contact.restitution;
        float blendedStiffness = contactStiffness * (1f - e * 0.5f);  // Less stiff for high e


        // Normal penalty force magnitude (Kelvin-Voigt)
        float repulsionMag = blendedStiffness * penetration;
        float criticalDamping = 2f * Mathf.Sqrt(blendedStiffness * effectiveMass);
        float dampingMag = contactDampingRatio * criticalDamping * Mathf.Max(-velocityAlongNormal, 0f);  // Damp only closing to avoid sticking
        float keAlongNormal = 0.5f * effectiveMass * velocityAlongNormal * velocityAlongNormal;

        // Dissipate excess energy (e.g., for inelastic)
        float dissipation = (1 - contact.restitution) * keAlongNormal * 0.1f; // Tune factor
        dampingMag += dissipation / Time.fixedDeltaTime;
        float normalForceMag = repulsionMag + dampingMag;

        // Clamp to prevent negative (tensile) forces
        normalForceMag = Mathf.Max(0f, normalForceMag);

        Vector3 normalForceImpulse = contact.normal * normalForceMag * Time.fixedDeltaTime;

        // Apply normal force impulse
        ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, normalForceImpulse, contact.influenceRadius, contact.normal, e);
        ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -normalForceImpulse, contact.influenceRadius, -contact.normal, e);

        // Tangential viscous damping for smoother sliding (supplements impulsive friction)
        if (contact.friction > 0.001f)
        {
            Vector3 tangentialVelocity = relativeVelocity - velocityAlongNormal * contact.normal;
            float tangentialMag = tangentialVelocity.magnitude;

            if (tangentialMag > 0.001f)
            {
                Vector3 tangentDir = tangentialVelocity.normalized;
                float viscousMag = contact.friction * viscousFrictionFactor * normalForceMag; // Proportional to normal force
                Vector3 viscousImpulse = -tangentDir * viscousMag * Time.fixedDeltaTime;

                ApplyImpulseToNearbyPoints(obj1, contact.worldPosition, viscousImpulse, contact.influenceRadius, contact.normal, contact.friction);
                ApplyImpulseToNearbyPoints(obj2, contact.worldPosition, -viscousImpulse, contact.influenceRadius, -contact.normal, contact.friction);
            }
        }
    }

    private float CalculateContactImpulse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        SpringPointData point1 = obj1.surfaceSpringPoints2[contact.point1Index];
        SpringPointData point2 = obj2.surfaceSpringPoints2[contact.point2Index];

        // Calculate relative velocity at contact point
        Vector3 relativeVelocity = point1.velocity - point2.velocity;
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, contact.normal);

        // Don't resolve if already separating
        if (velocityAlongNormal > 0) return 0f;

        float invMass1 = point1.mass > 0 ? 1.0f / point1.mass : 0.0f;
        float invMass2 = point2.mass > 0 ? 1.0f / point2.mass : 0.0f;
        // NEW: Use effective (reduced) mass for conservation
        float reducedMass = 1f / (invMass1 + invMass2 + 1e-6f);
        float e = contact.restitution;

        // NEW: Impulse with momentum conservation
        float j = -(1 + e) * velocityAlongNormal * reducedMass;
        // Clamp j to prevent energy gain (e.g., if damping insufficient)
        j = Mathf.Max(j, -Mathf.Abs(velocityAlongNormal) * reducedMass * 0.95f); // Dissipate 5%

        return j;
    }

    private Vector3 CalculateFrictionImpulse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact, float normalImpulseMagnitude)
    {

        SpringPointData point1 = obj1.surfaceSpringPoints2[contact.point1Index];
        SpringPointData point2 = obj2.surfaceSpringPoints2[contact.point2Index];

        // Calculate relative velocity at contact point
        Vector3 relativeVelocity = point1.velocity - point2.velocity;

        // Project onto normal to get normal component
        float velAlongNormal = Vector3.Dot(relativeVelocity, contact.normal);

        // Tangential velocity (perpendicular to normal)
        Vector3 tangentialVelocity = relativeVelocity - (velAlongNormal * contact.normal);

        // Early out if no tangential motion
        float vtMag = tangentialVelocity.magnitude;
        if (vtMag < 0.001f) return Vector3.zero;

        // Tangent direction (in direction of tangential vel)
        Vector3 tangentDirection = tangentialVelocity.normalized;

        // Relative tangential speed (positive)
        float vr_t = Vector3.Dot(relativeVelocity, tangentDirection);

        // Inverse masses (handle fixed/infinite mass)
        float invMass1 = (point1.mass > 0) ? 1f / point1.mass : 0f;
        float invMass2 = (point2.mass > 0) ? 1f / point2.mass : 0f;
        float denom = invMass1 + invMass2;
        if (denom <= 0.0001f) return Vector3.zero;  // Both fixed, no impulse

        float staticThreshold = 0.01f; // Small velocity = static
        float mu_s = contact.friction * 1.2f; // Static coef > dynamic
        float mu_d = contact.friction;

        float mu = (vtMag < staticThreshold) ? mu_s : mu_d;
        float maxFriction = mu * normalImpulseMagnitude;

        // Trial friction impulse scalar (negative to oppose motion)
        float j_f = -vr_t / denom;

        // Clamp to friction cone
        if (Mathf.Abs(j_f) > maxFriction)
        {
            j_f = -Mathf.Sign(vr_t) * maxFriction;
        }

        // Friction impulse vector
        return j_f * tangentDirection;
    }

    private void ApplyImpulseToNearbyPoints(OctreeSpringFiller body, Vector3 contactPoint, Vector3 impulse,
                                           float influenceRadius, Vector3 contactNormal, float materialProperty)
    {
        for (int i = 0; i < body.allSpringPoints.Length; i++)
        {
            SpringPointData point = body.allSpringPoints[i];

            Vector3 toPoint = (Vector3)point.position - contactPoint;
            float distance = toPoint.magnitude;

            if (distance < influenceRadius && point.isFixed == 0)
            {
                // Directional bias - only apply if point is "behind" the contact surface
                float dot = Vector3.Dot(toPoint.normalized, contactNormal);
                if (dot < -0.5f) continue;  // Skip if opposite side

                // Calculate influence based on distance with Gaussian falloff for realism
                float normalizedDistance = distance / influenceRadius;
                float influence = Mathf.Exp(-(normalizedDistance * normalizedDistance) * contactInfluenceFalloff);

                // Bias for surface points
                if (point.isMeshVertex == 0)
                {  // Internal point
                    influence *= 0.6f;  // Weaker influence inside
                }

                // Scale influence by the material property (bounciness or friction)
                // This allows points with different material properties to respond differently
                float materialScale = Mathf.Lerp(0.5f, 1.5f, materialProperty);
                influence *= materialScale;

                Vector3 velocityChange = impulse * influence / point.mass;
                point.velocity += (float3)velocityChange;
                body.allSpringPoints[i] = point;

                if (logCollisionDetails && influence > 0.1f)
                {
                    // Debug.Log($"Applied impulse {velocityChange.magnitude:F2} to point at distance {distance:F2} (influence: {influence:F2}, material: {materialProperty:F2})");
                }
            }
        }
    }

    private void ApplyPositionCorrection(OctreeSpringFiller obj1, OctreeSpringFiller obj2, ContactPoint contact)
    {
        if (contact.penetrationDepth <= penetrationSlop) return;
        float correctionAmount = (contact.penetrationDepth - penetrationSlop) * penetrationCorrectionFactor;
        Vector3 correction = contact.normal * correctionAmount;

        int iterations = 3;
        for (int iter = 0; iter < iterations; iter++)
        {
            // Compute effective invMass based on nearby points
            float effectiveInvMass1 = CalculateEffectiveInvMass(obj1, contact.worldPosition, contact.influenceRadius, contact.normal);
            float effectiveInvMass2 = CalculateEffectiveInvMass(obj2, contact.worldPosition, contact.influenceRadius, -contact.normal);
            float totalEffectiveInvMass = effectiveInvMass1 + effectiveInvMass2;

            if (totalEffectiveInvMass == 0f) return;

            Vector3 correction1 = -correction * (effectiveInvMass1 / totalEffectiveInvMass) / iterations;
            Vector3 correction2 = correction * (effectiveInvMass2 / totalEffectiveInvMass) / iterations;

            ApplyPositionCorrectionToNearbyPoints(obj1, contact.worldPosition, correction1, contact.influenceRadius, contact.normal);
            ApplyPositionCorrectionToNearbyPoints(obj2, contact.worldPosition, correction2, contact.influenceRadius, -contact.normal);
        }
    }

    private float CalculateEffectiveInvMass(OctreeSpringFiller body, Vector3 contactPoint, float influenceRadius, Vector3 contactNormal)
    {
        float effectiveInvMass = 0f;
        for (int i = 0; i < body.allSpringPoints.Length; i++)
        {
            SpringPointData point = body.allSpringPoints[i];
            Vector3 toPoint = (Vector3)point.position - contactPoint;
            float distance = toPoint.magnitude;

            if (distance < influenceRadius && point.isFixed == 0)
            {
                float dot = Vector3.Dot(toPoint.normalized, contactNormal);
                if (dot < -0.1f) continue;

                float normalizedDistance = distance / influenceRadius;
                float influence = CalculateInfluenceFalloff(normalizedDistance);

                if (point.isMeshVertex == 0) influence *= 0.6f;

                effectiveInvMass += influence / point.mass;  // Weighted inv mass
            }
        }
        return effectiveInvMass;
    }

    private void ApplyPositionCorrectionToNearbyPoints(OctreeSpringFiller body, Vector3 contactPoint, Vector3 correction, float influenceRadius, Vector3 contactNormal)
    {
        for (int i = 0; i < body.allSpringPoints.Length; i++)
        {
            SpringPointData point = body.allSpringPoints[i];
            Vector3 toPoint = (Vector3)point.position - contactPoint;
            float distance = toPoint.magnitude;

            if (distance < influenceRadius && point.isFixed == 0)
            {
                // Directional bias for correction
                float dot = Vector3.Dot(toPoint.normalized, contactNormal);
                if (dot < -0.1f) continue;

                float normalizedDistance = distance / influenceRadius;
                float influence = CalculateInfluenceFalloff(normalizedDistance);

                // Surface bias
                if (point.isMeshVertex == 0)
                {
                    influence *= 0.6f;
                }

                point.position += (float3)(correction * influence);
                body.allSpringPoints[i] = point;
            }
        }
    }

    private float CalculateInfluenceFalloff(float normalizedDistance)
    {
        // Gaussian falloff for realistic response
        return Mathf.Exp(-(normalizedDistance * normalizedDistance) * contactInfluenceFalloff);
    }

    // Fallback to original global collision response
    private void HandleGlobalCollision(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Use the GJK collision response when contact point response is disabled
        HandleGJKCollisionResponse(obj1, obj2, info);

        // Also apply the per-point collision response
        obj1.HandleCollisionResponse(info, obj2);
        obj2.HandleCollisionResponse(new CollisionInfo
        {
            Normal = -info.Normal,
            Depth = info.Depth,
            DidCollide = true
        }, obj1);
    }

    // This method is now used only when contact point response is disabled
    private void HandleGJKCollisionResponse(OctreeSpringFiller obj1, OctreeSpringFiller obj2, CollisionInfo info)
    {
        // Calculate average material properties from all points
        float avgRestitution1 = GetAverageMaterialProperty(obj1, p => p.bounciness);
        float avgRestitution2 = GetAverageMaterialProperty(obj2, p => p.bounciness);
        float avgFriction1 = GetAverageMaterialProperty(obj1, p => p.friction);
        float avgFriction2 = GetAverageMaterialProperty(obj2, p => p.friction);

        // Blend the material properties
        float blendedRestitution = (avgRestitution1 + avgRestitution2) * 0.5f;
        float blendedFriction = (avgFriction1 + avgFriction2) * 0.5f;

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

        // Use the blended coefficient of restitution
        float e = blendedRestitution;

        // Impulse scalar formula
        float j = -(1 + e) * velocityAlongNormal;
        j /= (invMass1 + invMass2);

        // Apply impulse to each object
        Vector3 impulse = j * info.Normal;
        Vector3 velocityChange1 = impulse * invMass1;
        Vector3 velocityChange2 = -impulse * invMass2;

        // Apply velocity changes to obj1 with per-point material scaling
        for (int i = 0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj1.allSpringPoints[i];
            if (point.isFixed == 0)
            {
                // Scale velocity change by point's bounciness relative to average
                float bounceScale = (avgRestitution1 > 0) ? point.bounciness / avgRestitution1 : 1f;
                point.velocity += (float3)(velocityChange1 * bounceScale);
                obj1.allSpringPoints[i] = point;
            }
        }

        // Apply velocity changes to obj2 with per-point material scaling
        for (int i = 0; i < obj2.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj2.allSpringPoints[i];
            if (point.isFixed == 0)
            {
                // Scale velocity change by point's bounciness relative to average
                float bounceScale = (avgRestitution2 > 0) ? point.bounciness / avgRestitution2 : 1f;
                point.velocity += (float3)(velocityChange2 * bounceScale);
                obj2.allSpringPoints[i] = point;
            }
        }

        // --- 3. Apply Friction ---
        if (blendedFriction > 0.001f && velocityAlongNormal < 0)
        {
            // Calculate tangential velocity
            Vector3 tangentialVelocity = relativeVelocity - (Vector3)(velocityAlongNormal * info.Normal);
            float tangentialMagnitude = tangentialVelocity.magnitude;

            if (tangentialMagnitude > 0.001f)
            {
                Vector3 tangentDirection = tangentialVelocity.normalized;

                // Calculate friction impulse
                float frictionImpulseMagnitude = blendedFriction * Mathf.Abs(j);
                float maxFrictionImpulse = tangentialMagnitude / (invMass1 + invMass2);

                // Clamp friction to prevent reversal
                frictionImpulseMagnitude = Mathf.Min(frictionImpulseMagnitude, maxFrictionImpulse);

                Vector3 frictionImpulse = -tangentDirection * frictionImpulseMagnitude;
                Vector3 frictionChange1 = frictionImpulse * invMass1;
                Vector3 frictionChange2 = -frictionImpulse * invMass2;

                // Apply friction to obj1
                for (int i = 0; i < obj1.allSpringPoints.Length; i++)
                {
                    SpringPointData point = obj1.allSpringPoints[i];
                    if (point.isFixed == 0)
                    {
                        float frictionScale = (avgFriction1 > 0) ? point.friction / avgFriction1 : 1f;
                        point.velocity += (float3)(frictionChange1 * frictionScale);
                        obj1.allSpringPoints[i] = point;
                    }
                }

                // Apply friction to obj2
                for (int i = 0; i < obj2.allSpringPoints.Length; i++)
                {
                    SpringPointData point = obj2.allSpringPoints[i];
                    if (point.isFixed == 0)
                    {
                        float frictionScale = (avgFriction2 > 0) ? point.friction / avgFriction2 : 1f;
                        point.velocity += (float3)(frictionChange2 * frictionScale);
                        obj2.allSpringPoints[i] = point;
                    }
                }
            }
        }

        // --- 4. Positional Correction (Resolve Penetration) ---
        Vector3 correction = Mathf.Max(info.Depth - penetrationSlop, 0.0f) / (invMass1 + invMass2) * penetrationCorrectionFactor * info.Normal;
        Vector3 posChange1 = correction * invMass1;
        Vector3 posChange2 = -correction * invMass2;

        // Distribute position change across all points
        for (int i = 0; i < obj1.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj1.allSpringPoints[i];
            if (point.isFixed == 0)
            {
                point.position += (float3)posChange1;
                obj1.allSpringPoints[i] = point;
            }
        }

        for (int i = 0; i < obj2.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj2.allSpringPoints[i];
            if (point.isFixed == 0)
            {
                point.position += (float3)posChange2;
                obj2.allSpringPoints[i] = point;
            }
        }
    }

    private Vector3 GetAverageVelocity(OctreeSpringFiller obj)
    {
        if (obj.allSpringPoints.Length == 0) return Vector3.zero;

        Vector3 totalVelocity = Vector3.zero;
        float totalMass = 0f;

        for (int i = 0; i < obj.allSpringPoints.Length; i++)
        {
            SpringPointData p = obj.allSpringPoints[i];
            if (p.isFixed == 0)
            {
                totalVelocity += (Vector3)p.velocity * p.mass;
                totalMass += p.mass;
            }
        }

        return totalMass > 0 ? totalVelocity / totalMass : Vector3.zero;
    }

    private float GetAverageMaterialProperty(OctreeSpringFiller obj, System.Func<SpringPointData, float> propertySelector)
    {
        if (obj.allSpringPoints.Length == 0) return 0f;

        float total = 0f;
        int count = 0;

        for (int i = 0; i < obj.allSpringPoints.Length; i++)
        {
            SpringPointData point = obj.allSpringPoints[i];
            if (point.isFixed == 0)
            {
                total += propertySelector(point);
                count++;
            }
        }

        return count > 0 ? total / count : (propertySelector(obj.allSpringPoints[0]));
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
            if (body.surfaceSpringPoints2.IsCreated)
                GUILayout.Label($"{body.name}: Surface={body.surfaceSpringPoints2.Length}");
        }
    }

    private class SpatialHash<T> where T : class
    {
        private readonly Dictionary<Vector3Int, List<T>> buckets = new Dictionary<Vector3Int, List<T>>();
        private readonly float cellSize;

        public SpatialHash(float cellSize)
        {
            this.cellSize = cellSize;
        }

        public void Clear()
        {
            buckets.Clear();
        }

        public void Add(Vector3 position, T item)
        {
            Vector3Int cell = GetCell(position);
            if (!buckets.TryGetValue(cell, out var list))
            {
                list = new List<T>();
                buckets[cell] = list;
            }
            list.Add(item);
        }

        public IEnumerable<T> Query(Vector3 position, float radius)
        {
            List<T> results = new List<T>();
            Vector3Int centerCell = GetCell(position);
            int cellRadius = Mathf.CeilToInt(radius / cellSize) + 1; // Conservative

            for (int x = -cellRadius; x <= cellRadius; x++)
            {
                for (int y = -cellRadius; y <= cellRadius; y++)
                {
                    for (int z = -cellRadius; z <= cellRadius; z++)
                    {
                        Vector3Int neighborCell = centerCell + new Vector3Int(x, y, z);
                        if (buckets.TryGetValue(neighborCell, out var list))
                        {
                            foreach (var item in list)
                            {
                                results.Add(item);
                            }
                        }
                    }
                }
            }

            return results;
        }

        private Vector3Int GetCell(Vector3 position)
        {
            return new Vector3Int(
                Mathf.FloorToInt(position.x / cellSize),
                Mathf.FloorToInt(position.y / cellSize),
                Mathf.FloorToInt(position.z / cellSize)
            );
        }
    }

    // NEW: Simple Vector3Int struct (since Unity doesn't have one built-in)
    private struct Vector3Int
    {
        public int x, y, z;

        public Vector3Int(int x, int y, int z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vector3Int operator +(Vector3Int a, Vector3Int b) => new Vector3Int(a.x + b.x, a.y + b.y, a.z + b.z);

        public override int GetHashCode() => (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);

        public override bool Equals(object obj) => obj is Vector3Int other && x == other.x && y == other.y && z == other.z;
    }
}