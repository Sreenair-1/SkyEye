// SkyEye - Downward camera pipeline: optical flow, marker tracking + pose, survivor detection (Unit 5).
using SkyEye.Core;
using UnityEngine;

namespace SkyEye
{
    /// <summary>
    /// A small camera under the drone renders the ground into a 128×128 RenderTexture.
    /// Every frame the image is read back to the CPU and analysed:
    ///   - Lucas-Kanade optical flow (5×5 grid, 2-level pyramid) → ground velocity,
    ///   - fiducial marker detection → pose (height + offset) for precision landing,
    ///   - orange colour blob detection → survivor (life raft) position.
    /// The camera is on a stabilised gimbal: it always points straight down with north up,
    /// so image +x = east and +y = north.
    /// </summary>
    public class VisionSystem : MonoBehaviour
    {
        public DroneSimulation simulation;
        public Camera downCamera;
        public int resolution = 128;
        public float fieldOfView = 60f;

        public Texture DebugTexture { get { return debugTex; } }
        public bool ShowOverlay = true;

        // Results for the HUD
        public FlowVector[] LastFlow { get; private set; }
        public bool FlowValid { get; private set; }
        public int FlowInliers { get; private set; }
        public float FlowPixelsX { get; private set; }
        public float FlowPixelsY { get; private set; }
        public MarkerDetection LastMarker { get; private set; }
        public BlobDetection LastBlob { get; private set; }
        public float ProcessingMs { get; private set; }

        RenderTexture rt;
        Texture2D readTex, debugTex;
        byte[] rgba;
        Color32[] debugPixels;
        GrayImage prevGray;
        float prevCaptureTime = -1f;
        float pendingCaptureTime;
        float pendingRangefinder;
        bool hasPending;
        readonly System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();

        void Start()
        {
            rt = new RenderTexture(resolution, resolution, 16, RenderTextureFormat.ARGB32);
            rt.name = "DownCameraRT";
            rt.Create();
            if (downCamera != null)
            {
                downCamera.targetTexture = rt;
                downCamera.fieldOfView = fieldOfView;
                downCamera.aspect = 1f;
            }
            readTex = new Texture2D(resolution, resolution, TextureFormat.RGBA32, false);
            debugTex = new Texture2D(resolution, resolution, TextureFormat.RGBA32, false);
            debugTex.filterMode = FilterMode.Point;
            rgba = new byte[resolution * resolution * 4];
            debugPixels = new Color32[resolution * resolution];
        }

        void OnDestroy()
        {
            if (rt != null) { rt.Release(); Destroy(rt); }
            if (readTex != null) Destroy(readTex);
            if (debugTex != null) Destroy(debugTex);
        }

        public void ResetVision()
        {
            prevGray = null;
            prevCaptureTime = -1f;
            hasPending = false;
            FlowValid = false;
            LastFlow = null;
            LastMarker = new MarkerDetection();
            LastBlob = new BlobDetection();
        }

        /// <summary>Keeps the gimbal camera under the drone and remembers when this frame's image was taken.</summary>
        void LateUpdate()
        {
            if (simulation == null || downCamera == null) return;
            downCamera.transform.position = simulation.TruePosition + Vector3.down * 0.35f;
            downCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f); // straight down, north up
            pendingCaptureTime = simulation.SimTime;
            pendingRangefinder = simulation.RangefinderHeight;
            hasPending = true;
        }

        /// <summary>Called by DroneSimulation at the start of each frame (the image rendered last frame).</summary>
        public void ProcessFrame(bool running)
        {
            if (!hasPending || rt == null) return;
            hasPending = false;
            watch.Reset();
            watch.Start();

            var previous = RenderTexture.active;
            RenderTexture.active = rt;
            readTex.ReadPixels(new Rect(0, 0, resolution, resolution), 0, 0, false);
            RenderTexture.active = previous;
            Color32[] px = readTex.GetPixels32();
            for (int i = 0; i < px.Length; i++)
            {
                rgba[4 * i] = px[i].r; rgba[4 * i + 1] = px[i].g; rgba[4 * i + 2] = px[i].b; rgba[4 * i + 3] = 255;
            }
            var gray = GrayImage.FromRgba(rgba, resolution, resolution);
            float captureTime = pendingCaptureTime;
            float height = Mathf.Max(0.5f, pendingRangefinder + 0.35f); // camera height above the surface
            float mpp = PinholeCamera.MetresPerPixel(height, resolution, fieldOfView);
            float centre = 0.5f * (resolution - 1);

            // ---- 1. Optical flow → ground velocity
            FlowValid = false;
            if (prevGray != null && captureTime > prevCaptureTime + 1e-4f)
            {
                LastFlow = LucasKanade.TrackGrid(prevGray, gray, 5, 16);
                float mx, my;
                int inliers;
                FlowValid = LucasKanade.MedianFlow(LastFlow, out mx, out my, out inliers);
                FlowInliers = inliers;
                FlowPixelsX = mx; FlowPixelsY = my;
                float dt = captureTime - prevCaptureTime;
                if (FlowValid && running && height > 1.5f && height < 45f)
                {
                    // The ground moves opposite to the drone: v = −flow · (m per pixel) / Δt
                    simulation.OnFlowMeasurement(-mx * mpp / dt, -my * mpp / dt, captureTime);
                }
            }
            if (captureTime > prevCaptureTime + 1e-4f || prevGray == null)
            {
                prevGray = gray;
                prevCaptureTime = captureTime;
            }

            // ---- 2. Fiducial marker → pose
            LastMarker = MarkerDetector.Detect(gray, simulation.markerId);
            if (LastMarker.Found && running)
            {
                float h, e, n;
                MarkerDetector.EstimatePose(LastMarker, resolution, resolution, fieldOfView, simulation.markerSize, out h, out e, out n);
                simulation.OnMarker(h, e, n);
            }

            // ---- 3. Survivor (orange life raft) detection
            LastBlob = ColorBlobDetector.DetectOrange(rgba, resolution, resolution, 4);
            if (LastBlob.Found && running)
            {
                simulation.OnSurvivorSeen((LastBlob.Cx - centre) * mpp, (LastBlob.Cy - centre) * mpp);
            }

            watch.Stop();
            ProcessingMs = (float)watch.Elapsed.TotalMilliseconds;
            if (ShowOverlay) DrawDebug(px);
        }

        // ------------------------------------------------------------------ debug view

        void DrawDebug(Color32[] px)
        {
            System.Array.Copy(px, debugPixels, px.Length);
            var green = new Color32(60, 255, 90, 255);
            var red = new Color32(255, 60, 60, 255);
            if (LastFlow != null)
            {
                foreach (var f in LastFlow)
                {
                    if (!f.Valid) { Plot((int)f.X, (int)f.Y, red); continue; }
                    Line((int)f.X, (int)f.Y, (int)(f.X + f.Dx * 4f), (int)(f.Y + f.Dy * 4f), green);
                    Plot((int)f.X, (int)f.Y, new Color32(255, 255, 255, 255));
                }
            }
            if (LastMarker.Found && LastMarker.CornerX != null)
            {
                var mag = new Color32(255, 40, 255, 255);
                for (int k = 0; k < 4; k++)
                {
                    int j = (k + 1) % 4;
                    Line((int)LastMarker.CornerX[k], (int)LastMarker.CornerY[k], (int)LastMarker.CornerX[j], (int)LastMarker.CornerY[j], mag);
                }
                Cross((int)LastMarker.Cx, (int)LastMarker.Cy, mag);
            }
            if (LastBlob.Found)
            {
                var orange = new Color32(255, 160, 0, 255);
                Line(LastBlob.MinX - 2, LastBlob.MinY - 2, LastBlob.MaxX + 2, LastBlob.MinY - 2, orange);
                Line(LastBlob.MaxX + 2, LastBlob.MinY - 2, LastBlob.MaxX + 2, LastBlob.MaxY + 2, orange);
                Line(LastBlob.MaxX + 2, LastBlob.MaxY + 2, LastBlob.MinX - 2, LastBlob.MaxY + 2, orange);
                Line(LastBlob.MinX - 2, LastBlob.MaxY + 2, LastBlob.MinX - 2, LastBlob.MinY - 2, orange);
            }
            int c = resolution / 2;
            Cross(c, c, new Color32(255, 255, 255, 160));
            debugTex.SetPixels32(debugPixels);
            debugTex.Apply(false);
        }

        void Plot(int x, int y, Color32 col)
        {
            if (x < 0 || y < 0 || x >= resolution || y >= resolution) return;
            debugPixels[y * resolution + x] = col;
        }

        void Cross(int x, int y, Color32 col)
        {
            for (int k = -3; k <= 3; k++) { Plot(x + k, y, col); Plot(x, y + k, col); }
        }

        /// <summary>Bresenham line.</summary>
        void Line(int x0, int y0, int x1, int y1, Color32 col)
        {
            int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy, guard = 0;
            while (guard++ < 1000)
            {
                Plot(x0, y0, col);
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }
    }
}
