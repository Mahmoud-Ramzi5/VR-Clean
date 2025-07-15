using System;
using System.Collections;
using System.Diagnostics;
using TMPro;
using UnityEngine;

public class PerformanceMonitor : MonoBehaviour
{
    public TMP_Text fpsText;
    public TMP_Text ramText;
    public TMP_Text cpuText;
    public TMP_Text gpuText;

    [Header("Settings")]
    public float updateInterval = 0.5f;
    public bool showGPU = false;

    private float fpsAccumulator = 0;
    private int fpsFrames = 0;
    private float fpsNextUpdate = 0;
    private Process currentProcess;
    private TimeSpan lastCpuTime;
    private DateTime lastUpdateTime;

    private float gpuFrameTimeMs = 0f;
    private bool systemInfoAvailable = false;

    void Start()
    {
        fpsNextUpdate = Time.realtimeSinceStartup + updateInterval;

        try
        {
            // Try to initialize system process info
            currentProcess = Process.GetCurrentProcess();
            lastCpuTime = currentProcess.TotalProcessorTime;
            lastUpdateTime = DateTime.UtcNow;
            systemInfoAvailable = true;
        }
        catch
        {
            systemInfoAvailable = false;
            UnityEngine.Debug.LogWarning("System process information not available in this platform/build");
        }

        // GPU monitoring
        if (!showGPU || !SystemInfo.supportsGpuRecorder)
        {
            gpuText.gameObject.SetActive(false);
        }
        else
        {
            gpuText.gameObject.SetActive(true);
        }

        StartCoroutine(UpdateStats());
    }

    IEnumerator UpdateStats()
    {
        while (true)
        {
            UpdateFPS();
            UpdateRAM();

            // Retry once per loop if Process failed in Start()
            if (!systemInfoAvailable)
            {
                try
                {
                    currentProcess = Process.GetCurrentProcess();
                    lastCpuTime = currentProcess.TotalProcessorTime;
                    lastUpdateTime = DateTime.UtcNow;
                    systemInfoAvailable = true;
                }
                catch (Exception e)
                {
                    UpdateCPU_Fallback();
                }
            }

            if (systemInfoAvailable)
            {
                UpdateCPU();
            }

            if (showGPU && SystemInfo.supportsGpuRecorder)
            {
                UpdateGPU();
            }

            yield return new WaitForSeconds(updateInterval);
        }
    }

    void UpdateFPS()
    {
        // Calculate FPS - this should always work
        float fps = 1f / Time.unscaledDeltaTime;
        fpsText.text = $"FPS: {fps:0.}";
    }

    void UpdateRAM()
    {
        try
        {
            long ramBytes = 0;

            // Try different methods to get RAM usage
            if (systemInfoAvailable)
            {
                try
                {
                    ramBytes = currentProcess.WorkingSet64;
                }
                catch
                {
                    // Fall through to next method
                }
            }

            // Try Unity's profiler (only works in Development Build)
            if (ramBytes <= 0 && UnityEngine.Profiling.Profiler.enabled)
            {
                ramBytes = UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong();
            }

            // Final fallback to GC memory (least accurate)
            if (ramBytes <= 0)
            {
                ramBytes = GC.GetTotalMemory(false);
            }

            float ramMB = ramBytes / (1024f * 1024f);
            ramText.text = $"RAM: {ramMB:0.0} MB";
        }
        catch
        {
            ramText.text = "RAM: N/A";
        }
    }

    void UpdateCPU()
    {
        try
        {
            var newCpuTime = currentProcess.TotalProcessorTime;
            var now = DateTime.UtcNow;
            var elapsed = (now - lastUpdateTime).TotalMilliseconds;
            var usedMs = (newCpuTime - lastCpuTime).TotalMilliseconds;

            if (elapsed <= 0) elapsed = 1;

            float cpuPercent = (float)(usedMs / (elapsed * Environment.ProcessorCount) * 100);
            cpuText.text = $"CPU: {cpuPercent:0.0}%";

            lastUpdateTime = now;
            lastCpuTime = newCpuTime;
        }
        catch (Exception e)
        {
            cpuText.text = "CPU: N/A";
        }
    }

    void UpdateCPU_Fallback()
    {
        float cpuMs = Time.deltaTime * 1000f;
        cpuText.text = $"CPU Time: {cpuMs:0.0} ms";
    }

    void UpdateGPU()
    {
        try
        {
            if (showGPU && SystemInfo.supportsGpuRecorder)
            {
                FrameTimingManager.CaptureFrameTimings();
                FrameTiming[] timings = new FrameTiming[1];
                uint frames = FrameTimingManager.GetLatestTimings(1, timings);

                if (frames > 0 && timings[0].gpuFrameTime > 0f)
                {
                    gpuFrameTimeMs = (float)(timings[0].gpuFrameTime);
                    gpuText.text = $"GPU: {gpuFrameTimeMs:0.0} ms";
                }
                else
                {
                    gpuText.text = "GPU: N/A";
                }
            }
            else
            {
                gpuText.text = "GPU: Unsupported";
            }
        }
        catch (Exception ex)
        {
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

//    private float gpuFrameTimeMs = 0f;

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
//            FrameTimingManager.CaptureFrameTimings();
//            FrameTiming[] timings = new FrameTiming[1];
//            uint frames = FrameTimingManager.GetLatestTimings(1, timings);

//            if (frames > 0 && timings[0].gpuFrameTime > 0f)
//                gpuFrameTimeMs = (float)(timings[0].gpuFrameTime);
//                // This is just an approximation - real GPU monitoring requires platform-specific code
//                // float gpuUsage = Mathf.Clamp(Time.deltaTime / (1f / 60f) * 100f, 0f, 100f);

//                gpuText.text = $"GPU: {gpuFrameTimeMs:0.0} ms";
//        }
//        catch
//        {
//            gpuText.text = "GPU: Error";
//        }
//    }
//}