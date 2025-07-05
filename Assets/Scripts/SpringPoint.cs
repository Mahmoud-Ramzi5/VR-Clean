using System.Collections.Generic;
using UnityEngine;

public class SpringConnection
{
    public SpringPoint point1, point2;
    public float springConstant = 100f;
    public float damperConstant = 0.5f;
    public float restLength = 1f;

    public SpringConnection(SpringPoint point1, SpringPoint point2, float restLength, float springConstant, float damperConstant)
    {
        this.point1 = point1;
        this.point2 = point2;
        this.restLength = restLength;
        this.springConstant = springConstant;
        this.damperConstant = damperConstant;
    }

    // this no longer being called 
    // the logic has been moved to Jobs
    private void EnforceRigidConstraint(float relaxation = 0.5f)
    {
        if (point1 == null || point2 == null || point1 == point2)
            return;

        Vector3 direction = point2.predictedPosition - point1.predictedPosition;
        float distance = direction.magnitude;

        if (distance == 0 || float.IsNaN(distance)) return;

        float stretch = distance - restLength;
        Vector3 correction = direction.normalized * (stretch * 0.5f) * relaxation;

        // Fixed point support
        if (!point1.isFixed && !point2.isFixed)
        {
            point1.predictedPosition += correction;
            point2.predictedPosition -= correction;
        }
        else if (!point1.isFixed)
        {
            point1.predictedPosition += correction;
        }
        else if (!point2.isFixed)
        {
            point2.predictedPosition -= correction;
        }
    }

    // this no longer being called 
    // the logic has been moved to Jobs
    private void CalculateAndApplyForces()
    {
        // --- NaN/Zero Distance Check --- 
        if (point1 == null || point2 == null || point1.position == point2.position)
            return;

        Vector3 direction = point2.position - point1.position;
        float dist = direction.magnitude;

        if (dist == 0 || float.IsNaN(dist)) return;

        // Hooke's Law
        // Calculate spring force using Hooke's Law
        float stretch = dist - restLength;
        Vector3 springForce = springConstant * stretch * direction.normalized; // Normalized direction

        // Damping
        // Apply damping to prevent sliding at higher speeds
        Vector3 relativeVelocity = point2.velocity - point1.velocity;
        float velocityAlongSpring = Vector3.Dot(relativeVelocity, direction.normalized);
        Vector3 dampingForce = damperConstant * velocityAlongSpring * direction.normalized;

        // Combine forces
        Vector3 netForce = springForce + dampingForce;

        // Apply forces
        point1.force += netForce;
        point2.force -= netForce;
    }
}

public class SpringPoint
{
    public float mass = 1f;
    public float radius = 0.5f;

    public Vector3 force;
    public Vector3 velocity;
    public Vector3 position;
    public bool isFixed = false;
    public Vector3 initialPosition;
    public Vector3 predictedPosition;

    [Header("Collision")]
    public float bounciness = 0.2f;
    public float friction = 0.1f;

    [Header("Bounds")]
    public Bounds nodeBounds;
    public Vector3 boundsMin = new Vector3(0, 0, 0);
    public Vector3 boundsMax = new Vector3(0, 0, 0);

    [Header("Mesh")]
    public bool isMeshVertex = false;
    public int triangleIndex;

    public SpringPoint(Vector3 position)
    {
        this.position = position;
        initialPosition = position;
    }

    // this no longer being called 
    // the logic has been moved to Jobs
    private void UpdatePoint(float deltaTime)
    {
        if (isFixed) return;

        // --- NaN/Origin Checks ---
        if (float.IsNaN(position.x) || float.IsNaN(position.y) || float.IsNaN(position.z))
        {
            Debug.LogWarning($"NaN. Resetting.");
            velocity = Vector3.zero;
            force = Vector3.zero;
            return;
        }

        // Prevent division by zero
        if (mass <= 0) mass = 1f;

        // --- Force/Velocity Validation ---
        if (!float.IsNaN(force.x) && !float.IsNaN(force.y) && !float.IsNaN(force.z))
        {
            Vector3 acceleration = force / mass;
            velocity += acceleration * deltaTime;

            // Clamp velocity to prevent explosions
            //if (velocity.magnitude > 100f)
            //{
            //    velocity = velocity.normalized * 100f;
            //}
            // More conservative velocity clamping
            if (velocity.sqrMagnitude > 2500f) // 50 units/s squared
            {
                velocity = velocity.normalized * 50f;
            }
        }

        // --- Position Update ---
        Vector3 newPosition = position + (velocity * deltaTime);
        if (!float.IsNaN(newPosition.x) && !float.IsNaN(newPosition.y) && !float.IsNaN(newPosition.z) && newPosition.magnitude < 100000f)
        {
            position = newPosition;
        }
        else
        {
            velocity = Vector3.zero;
        }

        force = Vector3.zero;
    }

    private void HandleBoundaryBox()
    {
        Vector3 pos = position;

        for (int i = 0; i < 3; i++)
        {
            if (pos[i] - radius < boundsMin[i])
            {
                pos[i] = boundsMin[i] + radius;
                velocity[i] *= -bounciness;
                velocity *= (1f - friction);
            }
            else if (pos[i] + radius > boundsMax[i])
            {
                pos[i] = boundsMax[i] - radius;
                velocity[i] *= -bounciness;
                velocity *= (1f - friction);
            }
        }

        position = pos;
    }

    public void UpdateBounds(Vector3 moveStep)
    {
        Vector3 newCenter = nodeBounds.center + moveStep;
        nodeBounds = new Bounds(newCenter, nodeBounds.size);

        boundsMin = newCenter - nodeBounds.extents;
        boundsMax = newCenter + nodeBounds.extents;
    }

    public void DrawBoundingBox()
    {
        Vector3 center = (boundsMin + boundsMax) * 0.5f;
        Vector3 size = boundsMax - boundsMin;

        Gizmos.color = Color.magenta;
        Gizmos.DrawWireCube(center, size);
    }
}