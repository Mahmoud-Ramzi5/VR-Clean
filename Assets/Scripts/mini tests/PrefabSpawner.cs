using UnityEngine;

public class PrefabSpawner : MonoBehaviour
{
    [Header("Spawn Settings")]
    [SerializeField] private GameObject _prefab;
    [SerializeField] private Vector3 _spawnPosition = Vector3.zero;
    [SerializeField] private Vector3 _spawnScale = Vector3.one;
    [SerializeField] private Quaternion _spawnRotation = Quaternion.identity;

    [Header("Physics Settings")]

    [Header("Spawn Controls")]
    [SerializeField] private KeyCode _spawnKey = KeyCode.Space;
    [SerializeField] private bool _spawnOnStart;

    private void Start()
    {
        if (_spawnOnStart) SpawnObject();
    }

    private void Update()
    {
        if (Input.GetKeyDown(_spawnKey)) SpawnObject();
    }

    public GameObject SpawnObject()
    {
        return SpawnObject(_spawnPosition, _spawnRotation, _spawnScale);
    }

    public GameObject SpawnObject(Vector3 position, Quaternion rotation, Vector3 scale)
    {
        if (_prefab == null)
        {
            Debug.LogError("No prefab assigned to spawner!");
            return null;
        }

        GameObject spawnedObject = Instantiate(_prefab, position, rotation);
        spawnedObject.transform.localScale = scale;
        return spawnedObject;
    }
}