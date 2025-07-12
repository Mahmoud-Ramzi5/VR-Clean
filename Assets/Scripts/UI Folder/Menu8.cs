using Unity.Multiplayer.Center.Common;
using UnityEditor.Presets;
using UnityEngine;
using UnityEngine.UI;

public class Menu8 : MenuDataBinderBase
{
    public Button woodButton;
    public Button metalButton;
    public Button plasticButton;
    public Button rubberButton;
    public Button stoneButton;
    public MaterialPreset wood;
    public MaterialPreset metal;
    public MaterialPreset plastic;
    public MaterialPreset rubber;
    public MaterialPreset stone;

    private MaterialPreset selectedpreset; // Stores chosen mesh until Apply

    void Start()
    {
        // Set up button click listeners
        woodButton.onClick.AddListener(() => SelectMatirial(wood));
        metalButton.onClick.AddListener(() => SelectMatirial(metal));
        plasticButton.onClick.AddListener(() => SelectMatirial(plastic));
        rubberButton.onClick.AddListener(() => SelectMatirial(rubber));
        stoneButton.onClick.AddListener(() => SelectMatirial(stone));
    }

    void SelectMatirial(MaterialPreset preset)
    {
        selectedpreset = preset;
        Debug.Log("preset queued: " + preset.name);
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        // Apply mesh change if one was selected
        if (selectedpreset != null)
        {
            var game_object = target.gameObject;
            var materialManager = game_object.GetComponent<MaterialManager>();
            materialManager.ApplyMaterial(game_object, selectedpreset);
            materialManager.materialType = selectedpreset.Type;
            Debug.Log("Applied preset: " + selectedpreset.name);
        }
    }
}