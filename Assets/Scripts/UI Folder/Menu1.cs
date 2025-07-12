using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Menu1 : MenuDataBinderBase
{
    public TMP_InputField Min_node;
    public TMP_InputField Point;

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

        Min_node.text = target.minNodeSize.ToString("F2"); // "F2" = 2 decimal places
        Point.text = target.PointSpacing.ToString("F2"); // "F2" = 2 decimal places
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        if (Min_node != null && !string.IsNullOrWhiteSpace(Min_node.text))
            float.TryParse(Min_node.text, out target.minNodeSize);

        if (Point != null && !string.IsNullOrWhiteSpace(Point.text))
            float.TryParse(Point.text, out target.PointSpacing);

    }
}
