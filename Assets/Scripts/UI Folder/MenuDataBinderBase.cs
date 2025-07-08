using UnityEngine;

public abstract class MenuDataBinderBase : MonoBehaviour
{
    /// <summary>
    /// Implement this in each binder script to apply its UI data
    /// to your simulation controller (e.g., OctreeSpringFiller).
    /// </summary>
    public abstract void ApplyTo(OctreeSpringFiller target);
}
