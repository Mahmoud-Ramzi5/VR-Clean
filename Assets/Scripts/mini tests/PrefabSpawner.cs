using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;

public class PrefabSpawner : MonoBehaviour
{
    [Header("Spawn Settings")]
    [SerializeField] private GameObject _templateInstance;
    [SerializeField] private Vector3 _spawnPosition = Vector3.one;
    [SerializeField] private Vector3 _spawnScale = Vector3.one;
    [SerializeField] private Quaternion _spawnRotation = Quaternion.identity;

    [Header("Physics Settings")]
    [SerializeField] private bool _resetPhysics = true;

    [Header("Spawn Controls")]
    [SerializeField] private KeyCode _spawnKey = KeyCode.Space;
    [SerializeField] private bool _spawnOnStart;
    private NativeArray<SpringPointData> _sharedPoints;
    private NativeArray<SpringConnectionData> _sharedConnections;
    private bool _springDataInitialized = false;
    private Vector3 _originalPrefabPosition;

    private void Start()
    {
     
    }

    private void InitializeSharedSpringData()
    {
        if (_springDataInitialized) return;

        var temp = Instantiate(_templateInstance, _spawnPosition, _spawnRotation);
        var filler = temp.GetComponent<OctreeSpringFiller>();
        filler.FillObjectWithSpringPoints(); // force generation
        _sharedPoints = new NativeArray<SpringPointData>(filler.allSpringPoints, Allocator.Persistent);
        _sharedConnections = new NativeArray<SpringConnectionData>(filler.allSpringConnections, Allocator.Persistent);
        Destroy(temp); // temp object used only for data gen
        _springDataInitialized = true;
    }

    void CopyPhysics(OctreeSpringFiller source, OctreeSpringFiller target)
    {
        target.springConstantL1 = source.springConstantL1;
        target.damperConstantL1 = source.damperConstantL1;
        target.connectionRadiusL1 = source.connectionRadiusL1;
        target.maxRestLengthL1 = source.maxRestLengthL1;

        target.springConstantL2 = source.springConstantL2;
        target.damperConstantL2 = source.damperConstantL2;
        target.connectionRadiusL2 = source.connectionRadiusL2;
        target.maxRestLengthL2 = source.maxRestLengthL2;

        target.springConstantL3 = source.springConstantL3;
        target.damperConstantL3 = source.damperConstantL3;
        target.connectionRadiusL3 = source.connectionRadiusL3;
        target.maxRestLengthL3 = source.maxRestLengthL3;

        target.totalMass = source.totalMass;
        target.bounciness = source.bounciness;
        target.friction = source.friction;
        target.isRigid = source.isRigid;
    }

    public void SetTemplateInstance(GameObject instance)
    {
        _templateInstance = instance;
    }

    private void Update()
    {
        if (Input.GetKeyDown(_spawnKey)) SpawnObject(_spawnPosition, _spawnRotation, _spawnScale);
    }

    public GameObject SpawnObject(Vector3 position, Quaternion rotation, Vector3 scale)
    {
        if (_templateInstance == null)
        {
            // Debug.LogError("No template instance assigned!");
            return null;
        }

        GameObject spawnedObject = Instantiate(_templateInstance, position, rotation);
        spawnedObject.transform.localScale = scale;

        // Destroy the spawned object after 30 seconds
        Destroy(spawnedObject, 30f);

        var sourceFiller = _templateInstance.GetComponent<OctreeSpringFiller>();
        var spawnedFiller = spawnedObject.GetComponent<OctreeSpringFiller>();

        if (sourceFiller != null && spawnedFiller != null)
        {
            CopyPhysics(sourceFiller, spawnedFiller);
        }

        var matMgr = spawnedObject.GetComponent<MaterialManager>();
        if (matMgr != null)
        {
            matMgr.skipApplyPhysicsOnStart = true;
        }

        var filler = spawnedObject.GetComponent<OctreeSpringFiller>();
        if (filler != null)
        {
            filler.skipPhysicsFromMaterial = true;
        }
        if (filler != null && _springDataInitialized)
        {
            filler.OverrideSpringData(_sharedPoints, _sharedConnections);
        }

        return spawnedObject;
    }

}