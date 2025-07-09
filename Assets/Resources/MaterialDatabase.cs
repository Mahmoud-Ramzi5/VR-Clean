using UnityEngine;
using System.Collections.Generic;

[CreateAssetMenu(fileName = "MaterialDatabase", menuName = "Materials/Material Database")]
public class MaterialDatabase : ScriptableObject
{
    public List<MaterialPreset> presets;

    public MaterialPreset GetPreset(MaterialType type)
    {
        Debug.LogWarning($"{type}");

        return presets.Find(p => p.Type == type);
    }
}