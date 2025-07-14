using UnityEngine;
using TMPro;
using System.Linq;
using Unity.Collections;

[DefaultExecutionOrder(100)] // Ensures this runs after most other scripts
public class PerformanceStatsMonitor : MonoBehaviour
{
    [Header("Text References")]
    public TMP_Text bodiesText;
    public TMP_Text verticesText;
    public TMP_Text indicesText;
    public TMP_Text trianglesText;
    public TMP_Text pointsText;
    public TMP_Text connectionsText;

    [Header("Settings")]
    [Tooltip("How often to update stats (seconds)")]
    public float updateInterval = 1.0f;

    [Tooltip("Format for numbers (N0=commas, 0=no commas)")]
    public string numberFormat = "N0";

    [Tooltip("Include inactive OctreeSpringFiller objects")]
    public bool includeInactiveObjects = true;

    [Tooltip("Keep searching for new objects at runtime")]
    public bool trackRuntimeObjects = true;

    [Tooltip("Show detailed debug messages")]
    public bool showDebugLogs = true;

    private float nextUpdateTime;
    private OctreeSpringFiller[] springFillers;

    void Start()
    {
        RefreshFillers();
        UpdateAllTexts("Initializing...");
        nextUpdateTime = Time.time + updateInterval;
    }

    void Update()
    {
        if (Time.time >= nextUpdateTime)
        {
            if (trackRuntimeObjects)
            {
                RefreshFillers();
            }
            UpdateStats();
            nextUpdateTime = Time.time + updateInterval;
        }
    }

    /// <summary>
    /// Re-find all OctreeSpringFiller objects in scene
    /// </summary>
    public void RefreshFillers()
    {
        var findMode = includeInactiveObjects ?
            FindObjectsInactive.Include :
            FindObjectsInactive.Exclude;

        springFillers = FindObjectsByType<OctreeSpringFiller>(
            findMode,
            FindObjectsSortMode.None);

        if (showDebugLogs)
        {
            // Debug.Log($"RefreshFillers() found {springFillers.Length} instances:");
            foreach (var filler in springFillers)
            {
                string status = filler == null ? "NULL" :
                    (!filler.gameObject.activeInHierarchy ? "INACTIVE" : "ACTIVE");
                // Debug.Log($"- {filler?.name ?? "NULL"} ({status})",
                //     filler != null ? filler.gameObject : null);
            }
        }
    }

    void UpdateStats()
    {
        if (springFillers == null || springFillers.Length == 0)
        {
            // if (showDebugLogs)
            //     Debug.LogWarning("No OctreeSpringFiller instances found");
            UpdateAllTexts("N/A");
            return;
        }

        int totalVertices = 0;
        int totalIndices = 0;
        int totalTriangles = 0;
        int totalPoints = 0;
        int totalConnections = 0;
        int activeBodies = 0;

        foreach (var filler in springFillers)
        {
            if (filler == null) continue;

            // Only count active objects for body count
            if (filler.gameObject.activeInHierarchy)
            {
                activeBodies++;
            }

            // Mesh data
            if (filler.targetMesh != null)
            {
                totalVertices += filler.targetMesh.vertexCount;
                totalIndices += filler.targetMesh.GetIndices(0).Length;
                totalTriangles += filler.targetMesh.triangles.Length / 3;
            }
            else if (showDebugLogs)
            {
                // Debug.LogWarning($"{filler.name} has no target mesh", filler.gameObject);
            }

            // Spring point data
            if (filler.allSpringPoints.IsCreated)
            {
                totalPoints += filler.allSpringPoints.Length;
            }
            else if (showDebugLogs)
            {
                // Debug.LogWarning($"{filler.name} spring points not created", filler.gameObject);
            }

            // Connection data
            if (filler.allSpringConnections.IsCreated)
            {
                totalConnections += filler.allSpringConnections.Length;
            }
            else if (showDebugLogs)
            {
                // Debug.LogWarning($"{filler.name} spring connections not created", filler.gameObject);
            }
        }

        // Update UI
        UpdateText(bodiesText, activeBodies, "Bodies:");
        UpdateText(verticesText, totalVertices, "Verticies:");
        UpdateText(indicesText, totalIndices, "Indices:");
        UpdateText(trianglesText, totalTriangles, "Triangles:");
        UpdateText(pointsText, totalPoints, "Points:");
        UpdateText(connectionsText, totalConnections, "Connections:");

        if (showDebugLogs)
        {
            // Debug.Log($"Stats Updated:\n" +
            //     $"Bodies: {activeBodies}\n" +
            //     $"Vertices: {totalVertices}\n" +
            //     $"Triangles: {totalTriangles}\n" +
            //     $"Points: {totalPoints}\n" +
            //     $"Connections: {totalConnections}");
        }
    }

    void UpdateText(TMP_Text textComponent, int value, string name)
    {
        if (textComponent != null)
        {
            textComponent.text = name + value.ToString(numberFormat);
        }
        else if (showDebugLogs)
        {
            // Debug.LogWarning("A text component is not assigned in inspector!");
        }
    }

    void UpdateAllTexts(string value)
    {
        if (bodiesText != null) bodiesText.text = "Bodies: " + value;
        if (verticesText != null) verticesText.text = "Verticies: " + value;
        if (indicesText != null) indicesText.text = "Indices: " + value;
        if (trianglesText != null) trianglesText.text = "Triangles: " + value;
        if (pointsText != null) pointsText.text = "Points: " + value;
        if (connectionsText != null) connectionsText.text = "Connections: " + value;
    }

    /// <summary>
    /// Force an immediate stats refresh
    /// </summary>
    public void ForceRefresh()
    {
        RefreshFillers();
        UpdateStats();
        nextUpdateTime = Time.time + updateInterval;
    }

    void OnValidate()
    {
        // Clamp update interval to prevent negative values
        updateInterval = Mathf.Max(0.1f, updateInterval);
    }
}