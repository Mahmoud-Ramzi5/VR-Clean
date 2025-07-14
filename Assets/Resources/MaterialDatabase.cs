using UnityEngine;
using System.Collections.Generic;
using Unity.VisualScripting;

[CreateAssetMenu(fileName = "MaterialDatabase", menuName = "Materials/Material Database")]
public class MaterialDatabase : ScriptableObject
{
    public List<MaterialPreset> presets;
    public List<MaterialType> types;

    public MaterialPreset GetPreset(MaterialType type)
    {
        // Debug.LogWarning($"{type}");
        
        return presets.Find(p => p.Type == type);
    }

}