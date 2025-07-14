using UnityEngine;

public enum MaterialType
{
    Glass,
    Wood,
    Stone,
    Metal,
    Rubber,
    Plastic,
    Custom,
    WoodRoom,
    StoneRoom,
    MetalRoom,
    RubberRoom,
    PlasticRoom,
}

[ExecuteAlways] // Allows OnValidate to run in edit mode
public class MaterialManager : MonoBehaviour
{
    public bool skipApplyPhysicsOnStart = false;
    public MaterialType materialType;
    public MaterialDatabase materialDatabase;
    private GameObject currentObject;

    private void Start()
    {
        currentObject = gameObject;

        if (!skipApplyPhysicsOnStart)
        {
            ApplyMaterial(currentObject, materialType);
        }
    }

    private void OnValidate()
    {
        if (!skipApplyPhysicsOnStart)
        {
            ApplyMaterial(gameObject, materialType);
        }
    }

    public void ApplyMaterial(GameObject parent, MaterialType type)
    {
        var preset = materialDatabase.GetPreset(type);
        // Debug.LogWarning($"type {type}");
        // Debug.LogWarning($"Preset {preset}");

        if (preset != null && parent != null)
        {
            // ProcessChildren
            if (parent.transform.childCount > 0)
            {
                // Debug.LogWarning("Children found in " + parent.name);
                foreach (Transform child in parent.transform)
                {
                    // Process each child
                    var renderer = child.GetComponent<Renderer>();
                    if (preset != null && renderer != null)
                        renderer.sharedMaterial = preset.material;
                }
            } 
            else
            {
                // Debug.LogWarning("No children found in " + parent.name);
                var renderer = parent.GetComponent<Renderer>();
                if (preset != null && renderer != null)
                    renderer.sharedMaterial = preset.material;
            }
            // Apply physics properties to OctreeSpringFiller
            ApplyPhysicsProperties(preset);
        }
    }


    public void ApplyMaterial(GameObject parent, MaterialPreset preset)
    {
        if (preset != null && parent != null)
        {
            // ProcessChildren
            if (parent.transform.childCount > 0)
            {
                // Debug.LogWarning("Children found in " + parent.name);
                foreach (Transform child in parent.transform)
                {
                    // Process each child
                    var renderer = child.GetComponent<Renderer>();
                    if (preset != null && renderer != null)
                        renderer.sharedMaterial = preset.material;
                }
            }
            else
            {
                // Debug.LogWarning("No children found in " + parent.name);
                var renderer = parent.GetComponent<Renderer>();
                if (preset != null && renderer != null)
                    renderer.sharedMaterial = preset.material;
            }
            // Apply physics properties to OctreeSpringFiller
            ApplyPhysicsProperties(preset);
        }
    }

    public MaterialPreset GetMaterialProperties()
    {
        return materialDatabase.GetPreset(materialType);
    }

    private void ApplyPhysicsProperties(MaterialPreset preset)
    {
        var springFiller = GetComponent<OctreeSpringFiller>();
        if (springFiller == null) return;

        springFiller.springConstantL1 = preset.springConstantL1;
        springFiller.damperConstantL1 = preset.damperConstantL1;
        springFiller.connectionRadiusL1 = preset.connectionRadiusL1;
        springFiller.maxRestLengthL1 = preset.maxRestLengthL1;

        springFiller.springConstantL2 = preset.springConstantL2;
        springFiller.damperConstantL2 = preset.damperConstantL2;
        springFiller.connectionRadiusL2 = preset.connectionRadiusL2;
        springFiller.maxRestLengthL2 = preset.maxRestLengthL2;

        springFiller.springConstantL3 = preset.springConstantL3;
        springFiller.damperConstantL3 = preset.damperConstantL3;
        springFiller.connectionRadiusL3 = preset.connectionRadiusL3;
        springFiller.maxRestLengthL3 = preset.maxRestLengthL3;

        springFiller.totalMass = preset.totalMass;
        springFiller.bounciness = preset.bounciness;
        springFiller.friction = preset.friction;
        springFiller.isRigid = preset.isRigid;
    }

    // Combined Friction √(f1 × f2);
    // Effective Bounciness (b1 + b2)/2;

    /*
    private void OnValidate()
    {
        SetDefaultsForMaterial(materialProperties.materialType);

        // Only apply in editor mode when values change
        if (objectRenderer == null)
            objectRenderer = GetComponent<Renderer>();

        if (objectRenderer != null)
            ApplySelectedMaterial(materialProperties.materialType);
    }

    public MaterialProperties GetMaterialProperties()
    {
        return materialProperties;
    }

    private void SetDefaultsForMaterial(MaterialType type)
    {
        switch (type)
        {
            case MaterialType.Glass:
                materialProperties.bounciness = 0.3f;
                materialProperties.friction = 0.2f;
                break;
            case MaterialType.Wood:
                materialProperties.bounciness = 0.2f;
                materialProperties.friction = 0.7f;
                break;
            case MaterialType.Metal:
                materialProperties.bounciness = 0.1f;
                materialProperties.friction = 0.1f;
                break;
            case MaterialType.Stone:
                materialProperties.bounciness = 0.0f;
                materialProperties.friction = 0.9f;
                break;
            case MaterialType.Rubber:
                materialProperties.bounciness = 0.8f;
                materialProperties.friction = 0.4f;
                break;
            case MaterialType.Plastic:
                materialProperties.bounciness = 0.4f;
                materialProperties.friction = 0.3f;
                break;
            case MaterialType.Custom:
                break;
        }
    }

    private void ApplySelectedMaterial(MaterialType type)
    {
        switch (type)
        {
            case MaterialType.Glass:
                if (MaterialReferences.GlassMaterial != null) objectRenderer.material = MaterialReferences.GlassMaterial;
                break;
            case MaterialType.Wood:
                if (MaterialReferences.WoodMaterial != null) objectRenderer.material = MaterialReferences.WoodMaterial;
                break;
            case MaterialType.Stone:
                if (MaterialReferences.StoneMaterial != null) objectRenderer.material = MaterialReferences.StoneMaterial;
                break;
            case MaterialType.Metal:
                if (MaterialReferences.MetalMaterial != null) objectRenderer.material = MaterialReferences.MetalMaterial;
                break;
            case MaterialType.Rubber:
                if (MaterialReferences.RubberMaterial != null) objectRenderer.material = MaterialReferences.RubberMaterial;
                break;
            case MaterialType.Plastic:
                if (MaterialReferences.PlasticMaterial != null) objectRenderer.material = MaterialReferences.PlasticMaterial;
                break;
            case MaterialType.Custom:
                if (MaterialReferences.CustomMaterial != null) 
                    objectRenderer.material = MaterialReferences.CustomMaterial;
                else
                    objectRenderer.material = null;
                break;
        }
    } */
}