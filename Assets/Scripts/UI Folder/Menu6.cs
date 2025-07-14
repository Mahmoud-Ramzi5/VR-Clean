using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Menu6 : MenuDataBinderBase
{
    public TMP_InputField Mass;
    public TMP_InputField Gravity;
    public Toggle Apply_Gravity;
    public TMP_InputField X_Velocity;
    public TMP_InputField Y_Velocity;
    public TMP_InputField Z_Velocity;
    public TMP_InputField Friction;
    public TMP_InputField Bounciness;
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
        Vector3 velocity = target.velocity;
        // Debug.Log("velocity is" + velocity);
        Mass.text = target.totalMass.ToString("F2");
        Gravity.text = target.gravity.y.ToString("F2");
        target.applyGravity = Apply_Gravity.isOn;
        X_Velocity.text = velocity.x.ToString("F2");
        Y_Velocity.text = velocity.y.ToString("F2");
        Z_Velocity.text = velocity.z.ToString("F2");
        Friction.text = target.friction.ToString("F2");
        Bounciness.text = target.bounciness.ToString("F2");
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {
        Vector3 grav = new Vector3(0, 0, 0);
        Vector3 velocity = new Vector3(0, 0, 0);
        if (Mass != null && !string.IsNullOrWhiteSpace(Mass.text))
            float.TryParse(Mass.text, out target.totalMass);

        if (Gravity != null && !string.IsNullOrWhiteSpace(Gravity.text))
        {
            float.TryParse(Gravity.text, out grav.y);
            target.gravity = grav;
        }

        target.applyGravity = Apply_Gravity.isOn;

        if (X_Velocity != null && !string.IsNullOrWhiteSpace(X_Velocity.text))
            float.TryParse(X_Velocity.text, out velocity.x);

        if (Y_Velocity != null && !string.IsNullOrWhiteSpace(Y_Velocity.text))
            float.TryParse(Y_Velocity.text, out velocity.y);

        if (Z_Velocity != null && !string.IsNullOrWhiteSpace(Z_Velocity.text))
            float.TryParse(Z_Velocity.text, out velocity.z);

        target.velocity = velocity;

        if (Friction != null && !string.IsNullOrWhiteSpace(Friction.text))
            float.TryParse(Friction.text, out target.friction);

        if (Bounciness != null && !string.IsNullOrWhiteSpace(Bounciness.text))
            float.TryParse(Bounciness.text, out target.bounciness);
    }
}
