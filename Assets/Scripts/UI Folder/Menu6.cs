using UnityEngine;
using UnityEngine.UI;

public class Menu6 : MenuDataBinderBase
{
    public Toggle visualizeSpringPointsToggle;
    public Toggle visualizeSpringConnectionsToggle;

    public override void ApplyTo(OctreeSpringFiller target)
    {
        target.visualizeSpringPoints = visualizeSpringPointsToggle.isOn;
        target.visualizeSpringConnections = visualizeSpringConnectionsToggle.isOn;
    }
}
