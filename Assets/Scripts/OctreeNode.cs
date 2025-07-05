using System.Collections.Generic;
using UnityEngine;

public class OctreeNode
{
    public Bounds worldBounds, localBounds;
    public OctreeNode[] children;
    public bool isDivided;

    public OctreeNode(Bounds nodeWorldBounds, Bounds nodeLocalbounds)
    {
        worldBounds = nodeWorldBounds;
        localBounds = nodeLocalbounds;
        isDivided = false;
    }

    public bool Divide(float minSize)
    {
        // If the node size is already small enough, do not subdivide further
        if (worldBounds.size.x <= minSize && worldBounds.size.y <= minSize && worldBounds.size.z <= minSize)
            return false;

        // Initialize the array of 8 children nodes
        children = new OctreeNode[8];

        // Start from this node's center
        Vector3 worldCenter = worldBounds.center;
        Vector3 localCenter = localBounds.center;

        // Each child node will be half the size of this node
        Vector3 worldChildSize = worldBounds.size * 0.5f;
        Vector3 localChildSize = localBounds.size * 0.5f;

        // Calculate offset to place child nodes relative to this node's center
        Vector3 worldQuarter = worldBounds.size * 0.25f;
        Vector3 localQuarter = localBounds.size * 0.25f;

        // Loop through each of the 8 octants (children)
        for (int i = 0; i < 8; i++)
        {
            // Calculate the offset for each child on x, y, and z axes
            // The bitwise checks on 'i' determine which side along each axis:
            // - If bit is 0, offset is negative (left/down/back)
            // - If bit is 1, offset is positive (right/up/front)

            Vector3 worldOffset = new Vector3(
                ((i & 1) == 0 ? -1 : 1) * worldQuarter.x,  // Check bit 0 for x-axis
                ((i & 2) == 0 ? -1 : 1) * worldQuarter.y,  // Check bit 1 for y-axis
                ((i & 4) == 0 ? -1 : 1) * worldQuarter.z   // Check bit 2 for z-axis
            );

            Vector3 localOffset = new Vector3(
                ((i & 1) == 0 ? -1 : 1) * localQuarter.x,  // Check bit 0 for x-axis
                ((i & 2) == 0 ? -1 : 1) * localQuarter.y,  // Check bit 1 for y-axis
                ((i & 4) == 0 ? -1 : 1) * localQuarter.z   // Check bit 2 for z-axis
            );

            // Create a new child node with the calculated center and size
            Bounds childWorldBounds = new Bounds(worldCenter + worldOffset, worldChildSize);
            Bounds childLocalBounds = new Bounds(localCenter + localOffset, localChildSize);
            children[i] = new OctreeNode(childWorldBounds, childLocalBounds);
        }

        // Mark this node as divided
        isDivided = true;

        return true;
    }

    public void DrawGizmos(Color leafColor, Color branchColor)
    {
        Gizmos.color = isDivided ? branchColor : leafColor;
        Gizmos.DrawWireCube(worldBounds.center, worldBounds.size);

        if (isDivided && children != null)
        {
            foreach (var child in children)
            {
                if (child != null)
                    child.DrawGizmos(leafColor, branchColor);
            }
        }
    }
}
