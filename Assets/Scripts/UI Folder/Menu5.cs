using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Menu5 : MenuDataBinderBase
{
    public TMP_InputField Layer3_spring_constant;
    public TMP_InputField Layer3_spring_damper;
    public TMP_InputField Layer3_connection_radius;
    public TMP_InputField Layer3_max_rest_length;

    void Start()
    {
        OctreeSpringFiller springFiller = FindObjectOfType<OctreeSpringFiller>();
        if (springFiller != null)
        {
            Initializethings(springFiller);
        }
    }

    // Initialize toggle states based on target values
    public void Initializethings(OctreeSpringFiller target)
    {
        if (target == null) return;

        Layer3_spring_constant.text = target.springConstantL1.ToString("F2"); // "F2" = 2 decimal places
        Layer3_spring_damper.text = target.damperConstantL1.ToString("F2");
        Layer3_connection_radius.text = target.connectionRadiusL1.ToString("F2");
        Layer3_max_rest_length.text = target.maxRestLengthL1.ToString("F2");
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        if (Layer3_spring_constant != null && !string.IsNullOrWhiteSpace(Layer3_spring_constant.text))
            float.TryParse(Layer3_spring_constant.text, out target.springConstantL3);

        if (Layer3_spring_damper != null && !string.IsNullOrWhiteSpace(Layer3_spring_damper.text))
            float.TryParse(Layer3_spring_damper.text, out target.damperConstantL3);

        if (Layer3_connection_radius != null && !string.IsNullOrWhiteSpace(Layer3_connection_radius.text))
            float.TryParse(Layer3_connection_radius.text, out target.connectionRadiusL3);

        if (Layer3_max_rest_length != null && !string.IsNullOrWhiteSpace(Layer3_max_rest_length.text))
            float.TryParse(Layer3_max_rest_length.text, out target.maxRestLengthL3);
    }
}
