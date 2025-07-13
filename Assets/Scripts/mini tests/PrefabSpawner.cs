using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;

public class PrefabSpawner : MonoBehaviour
{
    [Header("Spawn Settings")]
    [SerializeField] private GameObject _prefab;
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

        var temp = Instantiate(_prefab, _spawnPosition, _spawnRotation);
        var filler = temp.GetComponent<OctreeSpringFiller>();
        filler.FillObjectWithSpringPoints(); // force generation
        _sharedPoints = new NativeArray<SpringPointData>(filler.allSpringPoints, Allocator.Persistent);
        _sharedConnections = new NativeArray<SpringConnectionData>(filler.allSpringConnections, Allocator.Persistent);
        Destroy(temp); // temp object used only for data gen
        _springDataInitialized = true;
    }

    private void Update()
    {
        if (Input.GetKeyDown(_spawnKey)) SpawnObject(_spawnPosition, _spawnRotation, _spawnScale);
    }

    public GameObject SpawnObject(Vector3 position, Quaternion rotation, Vector3 scale)
    {
        if (_prefab == null)
        {
            Debug.LogError("No prefab assigned to spawner!");
            return null;
        }

        InitializeSharedSpringData(); // ensure shared data exists

        GameObject spawnedObject = Instantiate(_prefab, position, rotation);
        spawnedObject.transform.localScale = scale;

        OctreeSpringFiller filler = spawnedObject.GetComponent<OctreeSpringFiller>();
        if (filler != null)
        {
            filler.OverrideSpringData(_sharedPoints, _sharedConnections);
        }
        return spawnedObject;
    }

}