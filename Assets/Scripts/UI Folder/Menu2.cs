using UnityEngine;
using UnityEngine.UI;
using static UnityEngine.GraphicsBuffer;

public class Menu2 : MenuDataBinderBase
{
    public Toggle Apply_ground_collision;

    public Toggle Is_filled;
    public Toggle Is_rigid;

    public Toggle Show_points;
    public Toggle Show_connections;


    // Option 1: Initialize when UI becomes active (recommended)
    void Start()
    {
        // Get the main panel reference
        MainPanelReference mainPanel = GetComponentInParent<MainPanelReference>(true);

        if (mainPanel != null && mainPanel.springFillerRef != null)
        {
            InitializeToggles(mainPanel.springFillerRef);
        }
        else
        {
            // Debug.LogError("MainPanelReference or OctreeSpringFiller not found!");
        }
    }

    // Initialize toggle states based on target values
    public void InitializeToggles(OctreeSpringFiller target)
    {
        if (target == null) return;

        Apply_ground_collision.isOn = target.applyGroundCollision;

        Is_rigid.isOn = target.isRigid;
        Is_filled.isOn = target.isFilled;

        Show_points.isOn = target.visualizeSpringPoints;
        Show_connections.isOn = target.visualizeSpringConnections;
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        target.applyGroundCollision = Apply_ground_collision.isOn;

        target.isFilled = Is_filled.isOn;
        target.isRigid = Is_rigid.isOn;

        target.visualizeSpringPoints = Show_points.isOn;
        target.visualizeSpringConnections = Show_connections.isOn;
    }
}
