using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Menu1 : MenuDataBinderBase
{
    public TMP_InputField distribution;

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

        distribution.text = target.minNodeSize.ToString("F2"); // "F2" = 2 decimal places
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        float O = 1.0f;
        if (distribution != null && !string.IsNullOrWhiteSpace(distribution.text))
            float.TryParse(distribution.text, out O);

        target.minNodeSize = O;
        target.PointSpacing = O;
    }
}
