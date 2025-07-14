// Attach this to your MAIN panel (parent of all other panels)
using UnityEngine;

public class MainPanelReference : MonoBehaviour
{
    // The single OctreeSpringFiller reference for all child panels
    public OctreeSpringFiller springFillerRef;

    void Awake()
    {
        // Optional: Auto-find it if not assigned
        if (springFillerRef == null)
        {
            springFillerRef = GetComponentInChildren<OctreeSpringFiller>(true);
            // if (springFillerRef == null)
                // Debug.LogError("No OctreeSpringFiller found in hierarchy!");
        }
    }
}