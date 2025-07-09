using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MaterialPreset))]
public class MaterialPresetEditor : Editor
{
    public override void OnInspectorGUI()
    {
        MaterialPreset preset = (MaterialPreset)target;

        EditorGUI.BeginDisabledGroup(true);
        EditorGUILayout.EnumPopup("Type", preset.Type); // Grayed out
        EditorGUI.EndDisabledGroup();

        DrawDefaultInspector(); // Draws the rest normally except for 'type'
    }
}