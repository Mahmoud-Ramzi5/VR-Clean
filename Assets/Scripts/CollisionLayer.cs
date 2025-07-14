using UnityEngine;

[System.Serializable]
public class CollisionLayer
{
    [Header("Layer Configuration")]
    public string layerName = "Default";

    [Range(0, 31)]
    public int layerIndex = 0;

    [Header("Collision Matrix")]
    [Tooltip("Check which layers this layer can collide with")]
    public bool[] collidesWith = new bool[32];

    [Header("Material Properties")]
    public float density = 1000f;        // kg/m^3
    public float restitution = 0.6f;     // 0.0-1.0 (bounciness)
    public float friction = 0.4f;        // 0.0-1.0+ (surface friction)

    [Header("Advanced Properties")]
    public float youngsModulus = 1e6f;    // Pa (material stiffness)
    public float poissonRatio = 0.3f;     // 0.0-0.5 (volume preservation)
    public float dampingFactor = 0.1f;    // Internal damping

    public CollisionLayer()
    {
        // Default: collide with everything
        for (int i = 0; i < collidesWith.Length; i++)
        {
            collidesWith[i] = true;
        }
    }

    public bool CanCollideWith(int otherLayerIndex)
    {
        if (otherLayerIndex < 0 || otherLayerIndex >= collidesWith.Length)
            return false;

        return collidesWith[otherLayerIndex];
    }
}