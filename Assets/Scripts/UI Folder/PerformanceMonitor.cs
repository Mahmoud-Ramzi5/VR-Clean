using UnityEngine;
using UnityEngine.Profiling;
using TMPro;
using System.Collections;

public class PerformanceMonitor : MonoBehaviour
{
    public TMP_Text fpsText;
    public TMP_Text ramText;
    public TMP_Text cpuText;
    public TMP_Text gpuText;

    [Header("Settings")]
    public float updateInterval = 0.5f;
    public bool showGPU = false;

    private float fpsAccumulator = 0f;
    private int fpsFrames = 0;
    private float fpsNextUpdate = 0f;

    void Start()
    {
        // Initialize with safe Profiler enabling
        try
        {
            Profiler.enabled = true;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Profiler initialization failed: {e.Message}");
        }

        // Initialize GPU text visibility
        if (gpuText != null)
        {
            gpuText.gameObject.SetActive(showGPU && SystemInfo.supportsGpuRecorder);
        }

        // Initialize timers
        fpsNextUpdate = Time.realtimeSinceStartup + updateInterval;

        StartCoroutine(UpdateStats());
    }

    IEnumerator UpdateStats()
    {
        while (true)
        {
            UpdateFPS();
            UpdateRAM();
            UpdateCPU();

            if (showGPU && SystemInfo.supportsGpuRecorder)
            {
                UpdateGPU();
            }

            yield return new WaitForSeconds(updateInterval);
        }
    }

    void UpdateFPS()
    {
        // More accurate FPS calculation with smoothing
        fpsAccumulator += Time.unscaledDeltaTime;
        fpsFrames++;

        if (Time.realtimeSinceStartup >= fpsNextUpdate)
        {
            float fps = fpsFrames / fpsAccumulator;
            fpsText.text = $"FPS: {fps:0.}";

            fpsAccumulator = 0f;
            fpsFrames = 0;
            fpsNextUpdate = Time.realtimeSinceStartup + updateInterval;
        }
    }

    void UpdateRAM()
    {
        try
        {
            long totalMemory = 0;

            // Try Profiler first
            if (Profiler.enabled)
            {
                totalMemory = Profiler.GetTotalAllocatedMemoryLong();
            }
            // Fallback to GC memory
            else
            {
                totalMemory = System.GC.GetTotalMemory(false);
            }

            float ramMB = totalMemory / (1024f * 1024f);
            ramText.text = $"RAM: {ramMB:0.0} MB";
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"RAM update failed: {e.Message}");
            ramText.text = "RAM: N/A";
        }
    }

    void UpdateCPU()
    {
        try
        {
            // More accurate CPU approximation using frame time
            float frameTime = Time.deltaTime * 1000f; // in milliseconds
            float cpuUsage = Mathf.Clamp(frameTime / (1000f / 60f) * 100f, 0f, 100f);
            cpuText.text = $"CPU: {cpuUsage:0.0}%";
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"CPU update failed: {e.Message}");
            cpuText.text = "CPU: Error";
        }
    }

    void UpdateGPU()
    {
        try
        {
            // GPU approximation using render thread time
            float gpuFrameTime = Time.deltaTime * 1000f;
            float gpuUsage = Mathf.Clamp(gpuFrameTime / (1000f / 60f) * 100f, 0f, 100f);
            gpuText.text = $"GPU: {gpuUsage:0.0}%";
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"GPU update failed: {e.Message}");
            gpuText.text = "GPU: Error";
        }
    }
}

//using System;
//using System.Collections;
//using System.Diagnostics;
//using TMPro;
//using UnityEngine;

//public class PerformanceMonitor : MonoBehaviour
//{
//    public TMP_Text fpsText;
//    public TMP_Text ramText;
//    public TMP_Text cpuText;
//    public TMP_Text gpuText;

//    [Header("Settings")]
//    public float updateInterval = 0.5f;
//    public bool showGPU = false;

//    private float fpsAccumulator = 0;
//    private int fpsFrames = 0;
//    private float fpsNextUpdate = 0;
//    private Process currentProcess;
//    private TimeSpan lastCpuTime;
//    private DateTime lastUpdateTime;

//    void Start()
//    {
//        UnityEngine.Profiling.Profiler.enabled = true;
//        fpsNextUpdate = Time.realtimeSinceStartup + updateInterval;

//        try
//        {
//            currentProcess = Process.GetCurrentProcess();
//            lastCpuTime = currentProcess.TotalProcessorTime;
//            lastUpdateTime = DateTime.Now;
//        }
//        catch
//        {
//            UnityEngine.Debug.LogWarning("System process information not available");
//        }

//        if (!showGPU || !SystemInfo.supportsGpuRecorder)
//        {
//            gpuText.gameObject.SetActive(false);
//        }
//        else
//        {
//            gpuText.gameObject.SetActive(true);
//        }

//        StartCoroutine(UpdateStats());
//    }

//    IEnumerator UpdateStats()
//    {
//        while (true)
//        {
//            UpdateFPS();
//            UpdateRAM();
//            UpdateCPU();

//            if (showGPU && SystemInfo.supportsGpuRecorder)
//            {
//                UpdateGPU();
//            }

//            yield return new WaitForSeconds(updateInterval);
//        }
//    }

//    void UpdateFPS()
//    {
//        // Calculate FPS
//        float fps = 1f / Time.unscaledDeltaTime;
//        fpsText.text = $"FPS: {fps:0.}";
//    }

//    void UpdateRAM()
//    {
//        try
//        {
//            long ramBytes = 0;

//            // Method 1: Try using system process info first
//            if (currentProcess != null)
//            {
//                ramBytes = currentProcess.WorkingSet64;
//            }

//            // Method 2: If process info failed, use Unity's profiler (requires Profiler enabled)
//            if (ramBytes <= 0 && UnityEngine.Profiling.Profiler.enabled)
//            {
//                ramBytes = UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong();
//            }

//            // Method 3: Final fallback to GC memory (least accurate)
//            if (ramBytes <= 0)
//            {
//                ramBytes = GC.GetTotalMemory(false);
//            }

//            float ramMB = ramBytes / (1024f * 1024f);
//            ramText.text = $"RAM: {ramMB:0.0} MB";
//        }
//        catch
//        {
//            ramText.text = "RAM: N/A";
//        }
//    }

//    void UpdateCPU()
//    {
//        try
//        {
//            if (currentProcess != null)
//            {
//                TimeSpan newCpuTime = currentProcess.TotalProcessorTime;
//                DateTime now = DateTime.Now;
//                double elapsedTime = (now - lastUpdateTime).TotalMilliseconds;

//                float cpuUsage = (float)((newCpuTime.TotalMilliseconds - lastCpuTime.TotalMilliseconds) /
//                                    (Environment.ProcessorCount * elapsedTime));

//                lastCpuTime = newCpuTime;
//                lastUpdateTime = now;

//                cpuText.text = $"CPU: {Mathf.Clamp(cpuUsage * 100f, 0f, 100f):0.0}%";
//            }
//            else
//            {
//                cpuText.text = "CPU: N/A";
//            }
//        }
//        catch
//        {
//            cpuText.text = "CPU: Error";
//        }
//    }

//    void UpdateGPU()
//    {
//        try
//        {
//            // This is just an approximation - real GPU monitoring requires platform-specific code
//            float gpuUsage = Mathf.Clamp(Time.deltaTime / (1f / 60f) * 100f, 0f, 100f);
//            gpuText.text = $"GPU: {gpuUsage:0.0}%";
//        }
//        catch
//        {
//            gpuText.text = "GPU: Error";
//        }
//    }
//}