using UnityEngine;
using UnityEngine.UI;

public class MenuManager : MonoBehaviour
{
    [System.Serializable]
    public class Menu
    {
        public Button menuButton;
        public GameObject menuPanel;
    }

    public Menu[] menus;
    public GameObject currentActiveMenu;

    [Header("Simulation")]
    public OctreeSpringFiller[] simulationScripts; // Now an array of scripts

    void Start()
    {
        // Initialize all menus as inactive except first
        foreach (Menu menu in menus)
        {
            menu.menuPanel.SetActive(false);
            menu.menuButton.onClick.AddListener(() => ToggleMenu(menu));
        }

        // Activate first menu by default
        if (menus.Length > 0)
        {
            ToggleMenu(menus[0]);
        }
    }

    public void ToggleMenu(Menu selectedMenu)
    {
        // Close current active menu
        if (currentActiveMenu != null)
        {
            currentActiveMenu.SetActive(false);
        }

        // Open selected menu
        selectedMenu.menuPanel.SetActive(true);
        currentActiveMenu = selectedMenu.menuPanel;
    }

    /// <summary>
    /// Applies UI settings to all simulation objects
    /// </summary>
    public void ApplyUIAndRun()
    {
        // Debug.Log("Applying settings to all simulation objects");

        // Find all MenuDataBinderBase scripts in child panels (even inactive)
        var binders = GetComponentsInChildren<MenuDataBinderBase>(true);

        foreach (var simulationScript in simulationScripts)
        {
            if (simulationScript == null) continue;

            foreach (var binder in binders)
            {
                binder.ApplyTo(simulationScript);
            }

            // Start each simulation
            //simulationScript.RunSimulation();
        }

        // Debug.Log($"Applied settings to {simulationScripts.Length} simulation objects");
    }
}

//using UnityEngine;
//using UnityEngine.UI;

//public class MenuManager : MonoBehaviour
//{
//    [System.Serializable]
//    public class Menu
//    {
//        public Button menuButton;
//        public GameObject menuPanel;
//    }

//    public Menu[] menus;
//    public GameObject currentActiveMenu;

//    [Header("Simulation")]
//    public OctreeSpringFiller simulationScript; // Assign your script in Inspector

//    void Start()
//    {
//        // Initialize all menus as inactive except first
//        foreach (Menu menu in menus)
//        {
//            menu.menuPanel.SetActive(false);
//            menu.menuButton.onClick.AddListener(() => ToggleMenu(menu));
//        }

//        // Activate first menu by default
//        if (menus.Length > 0)
//        {
//            ToggleMenu(menus[0]);
//        }
//    }

//    public void ToggleMenu(Menu selectedMenu)
//    {
//        // Close current active menu
//        if (currentActiveMenu != null)
//        {
//            currentActiveMenu.SetActive(false);
//        }

//        // Open selected menu
//        selectedMenu.menuPanel.SetActive(true);
//        currentActiveMenu = selectedMenu.menuPanel;
//    }

//    /// <summary>
//    /// Call this from your Run button to apply all UI settings and start the simulation.
//    /// </summary>
//    public void ApplyUIAndRun()
//    {
//        Debug.Log("clicked.");
//        // Find all MenuDataBinderBase scripts in child panels (even inactive)
//        var binders = GetComponentsInChildren<MenuDataBinderBase>(true);

//        foreach (var binder in binders)
//        {
//            binder.ApplyTo(simulationScript);
//        }

//        // Now start the simulation
//        //simulationScript.RunSimulation();

//        Debug.Log("Simulation started after applying UI values.");
//    }
//}
