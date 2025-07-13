using System;
using System.Collections.Generic;
using System.Reflection;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using static UnityEngine.GraphicsBuffer;

public class MaterialMenu : MonoBehaviour
{
    public TMP_Dropdown materialDropdown;
    public Material wood;
    public Material metal;
    public Material plastic;
    public Material rubber;
    public Material stone;
    [Header("Simulation")]
    public GameObject[] objects; // Now an array of scripts
    private Material[] materials;

    void Start()
    {
        // Initialize material presets array
        materials = new Material [] { wood, metal, plastic, rubber, stone };
        // Populate dropdown
        materialDropdown.ClearOptions();
        List<string> options = new List<string>();
        foreach (var item in materials)
        {
            options.Add(item.name);
        }
        materialDropdown.AddOptions(options);
        materialDropdown.onValueChanged.AddListener(ChangeMaterial);
        // Apply font size to all options
        ApplyDropdownFontSize();
    }

    void ApplyDropdownFontSize()
    {
        // Get the dropdown's template which contains the item list
        var itemList = materialDropdown.template.GetComponentInChildren<TMP_Text>();

        if (itemList != null)
        {
            // This affects the font size of items in the dropdown list
            itemList.fontSize = 12;
        }

        // This affects the currently selected item display
        var captionText = materialDropdown.captionText;
        if (captionText != null)
        {
            captionText.fontSize = 12;
        }
    }

    void ChangeMaterial(int index)
    {
        if (index < 0 || index >= materials.Length) return;

        foreach (var objectt in objects)
        {
            if (objectt == null) continue;

            // Apply to self
            var renderer = objectt.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.sharedMaterial = materials[index];
            }

            // Or apply to children
            foreach (var child in objectt.GetComponentsInChildren<Renderer>())
            {
                child.sharedMaterial = materials[index];
            }

            Debug.Log("Directly applied material: " + materials[index].name);
        }
    }
}

//using System;
//using System.Collections.Generic;
//using System.Reflection;
//using TMPro;
//using UnityEngine;
//using UnityEngine.UI;
//using static UnityEngine.GraphicsBuffer;

//public class MaterialMenu : MonoBehaviour
//{
//    public TMP_Dropdown materialDropdown;
//    public MaterialPreset wood;
//    public MaterialPreset metal;
//    public MaterialPreset plastic;
//    public MaterialPreset rubber;
//    public MaterialPreset stone;
//    [Header("Simulation")]
//    public OctreeSpringFiller[] simulationScripts; // Now an array of scripts
//    private MaterialPreset[] materialPresets;

//    void Start()
//    {
//        // Initialize material presets array
//        materialPresets = new MaterialPreset[] { wood, metal, plastic, rubber, stone };
//        // Populate dropdown
//        materialDropdown.ClearOptions();
//        List<string> options = new List<string>();
//        foreach (var item in materialPresets)
//        {
//            options.Add(item.name);
//        }
//        materialDropdown.AddOptions(options);
//        materialDropdown.onValueChanged.AddListener(ChangeMaterial);
//        // Apply font size to all options
//        ApplyDropdownFontSize();
//    }

//    void ApplyDropdownFontSize()
//    {
//        // Get the dropdown's template which contains the item list
//        var itemList = materialDropdown.template.GetComponentInChildren<TMP_Text>();

//        if (itemList != null)
//        {
//            // This affects the font size of items in the dropdown list
//            itemList.fontSize = 12;
//        }

//        // This affects the currently selected item display
//        var captionText = materialDropdown.captionText;
//        if (captionText != null)
//        {
//            captionText.fontSize = 12;
//        }
//    }

//    void ChangeMaterial(int index)
//    {
//        foreach (var simulationScript in simulationScripts)
//        {
//            if (simulationScript == null) continue;

//            Debug.Log($"Applied settings to {simulationScripts.Length} simulation objects");
//            if (index >= 0 && index < 5)
//            {
//                var game_object = simulationScript.gameObject;
//                var materialManager = game_object.GetComponent<MaterialManager>();
//                materialManager.ApplyMaterial(game_object, materialPresets[index]);
//                materialManager.materialType = materialPresets[index].Type;
//                Debug.Log("Applied preset: " + materialPresets[index].name);
//            }
//        }
//    }
//}