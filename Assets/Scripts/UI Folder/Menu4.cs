using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Menu4 : MenuDataBinderBase
{
    public TMP_InputField Layer2_spring_constant;
    public TMP_InputField Layer2_spring_damper;
    public TMP_InputField Layer2_connection_radius;
    public TMP_InputField Layer2_max_rest_length;

    void Start()
    {
        // Get the main panel reference
        MainPanelReference mainPanel = GetComponentInParent<MainPanelReference>(true);

        if (mainPanel != null && mainPanel.springFillerRef != null)
        {
            Initializethings(mainPanel.springFillerRef);
        }
        else
        {
            // Debug.LogError("MainPanelReference or OctreeSpringFiller not found!");
        }
    }

    // Initialize toggle states based on target values
    public void Initializethings(OctreeSpringFiller target)
    {
        if (target == null) return;

        Layer2_spring_constant.text = target.springConstantL2.ToString("F2"); // "F2" = 2 decimal places
        Layer2_spring_damper.text = target.damperConstantL2.ToString("F2");
        Layer2_connection_radius.text = target.connectionRadiusL2.ToString("F2");
        Layer2_max_rest_length.text = target.maxRestLengthL2.ToString("F2");
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        if (Layer2_spring_constant != null && !string.IsNullOrWhiteSpace(Layer2_spring_constant.text))
            float.TryParse(Layer2_spring_constant.text, out target.springConstantL2);

        if (Layer2_spring_damper != null && !string.IsNullOrWhiteSpace(Layer2_spring_damper.text))
            float.TryParse(Layer2_spring_damper.text, out target.damperConstantL2);

        if (Layer2_connection_radius != null && !string.IsNullOrWhiteSpace(Layer2_connection_radius.text))
            float.TryParse(Layer2_connection_radius.text, out target.connectionRadiusL2);

        if (Layer2_max_rest_length != null && !string.IsNullOrWhiteSpace(Layer2_max_rest_length.text))
            float.TryParse(Layer2_max_rest_length.text, out target.maxRestLengthL2);
    }
}
