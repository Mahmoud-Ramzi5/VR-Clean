using UnityEngine;
using UnityEngine.UI;

public class Menu7 : MenuDataBinderBase
{
    public GameObject panel;
    public OctreeSpringFiller target; // Now an array of scripts
    public Button cubeButton;  
    public Button sphereButton; 
    public Button cylinderButton;  
    public Button capsuleButton; 
    public MeshFilter targetMeshFilter;
    public Mesh cubeMesh;
    public Mesh sphereMesh;
    public Mesh cylinderMesh;
    public Mesh capsuleMesh;

    private Mesh selectedMesh; // Stores chosen mesh until Apply

    void Start()
    {
        // Set up button click listeners
        cubeButton.onClick.AddListener(() => SelectMesh(cubeMesh));
        sphereButton.onClick.AddListener(() => SelectMesh(sphereMesh));
        cylinderButton.onClick.AddListener(() => SelectMesh(cylinderMesh));
        capsuleButton.onClick.AddListener(() => SelectMesh(capsuleMesh));
    }

    void SelectMesh(Mesh newMesh)
    {
        selectedMesh = newMesh;
        Debug.Log("Mesh queued: " + newMesh.name);
        // Apply mesh change if one was selected
        if (selectedMesh != null && targetMeshFilter != null)
        {
            targetMeshFilter.mesh = selectedMesh;
            Debug.Log("Applied mesh: " + selectedMesh.name);

        }
        panel.SetActive(false);
    }

    public override void ApplyTo(OctreeSpringFiller target)
    {

    }
}