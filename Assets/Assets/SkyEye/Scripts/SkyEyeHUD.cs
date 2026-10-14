// SkyEye - On-screen control panel, live sensor graphs, computer-vision view and results (IMGUI).
using System.Globalization;
using SkyEye.Core;
using UnityEngine;

namespace SkyEye
{
    public class SkyEyeHUD : MonoBehaviour
    {
        public DroneSimulation sim;
        public VisionSystem vision;
        public CameraRig cameraRig;
        public ArOverlay arOverlay;
        public EnvironmentController environment;
        public RenderModeController renderModes;

        const float PanelWidth = 350f;
        const float RightWidth = 330f;

        bool panelVisible = true, showGraphs = true, showVision = true;
        Vector2 scroll;
        float scale = 1f;
        Rect panelScreen, rightScreen, resultsScreen;
        System.Action pending;
        GraphTexture errorGraph, pitchGraph;
        float graphTimer;

        GUIStyle panelStyle, titleStyle, headerStyle, labelStyle, smallStyle, valueStyle, bannerStyle, buttonStyle;
        Texture2D panelTex, bannerTex;

        static readonly string[] FusionNames = { "GPS only", "GPS + IMU", "+ Optical flow" };
        static readonly string[] ViewNames = { "Chase", "FPV (AR)", "Top map" };
        static readonly string[] WeatherNames = { "Day", "Dusk", "Fog" };
        static readonly string[] GeoNames = { "Polygon", "Wireframe", "Both" };
        static readonly string[] LightNames = { "Full", "Local only", "Ambient only" };

        void Start()
        {
            errorGraph = new GraphTexture(300, 100, new[] { new Color32(255, 80, 80, 255), new Color32(60, 220, 255, 255) }, 0f, 6f, false);
            pitchGraph = new GraphTexture(300, 100, new[]
            {
                new Color32(255, 255, 255, 255), new Color32(255, 170, 40, 255),
                new Color32(255, 80, 80, 140), new Color32(80, 255, 120, 255)
            }, -25f, 25f, false);
        }

        public bool IsOverUi(Vector2 p)
        {
            return panelScreen.Contains(p) || rightScreen.Contains(p) || resultsScreen.Contains(p);
        }

        void Defer(System.Action a) { pending = a; }

        void Update()
        {
            if (sim == null || errorGraph == null) return;
            graphTimer -= Time.deltaTime;
            if (graphTimer > 0f) return;
            graphTimer = 0.1f;
            if (sim.State == RunState.Running)
            {
                errorGraph.Push(sim.GpsError, sim.HorizontalError);
                pitchGraph.Push(sim.PitchTrue, Mathf.Clamp(sim.PitchGyroOnly, -25f, 25f), sim.PitchAccelOnly, sim.PitchFused);
            }
            if (showGraphs) { errorGraph.Redraw(); pitchGraph.Redraw(); }
        }

        // ================================================================ styles

        void InitStyles()
        {
            panelTex = MakeTex(new Color(0.06f, 0.08f, 0.12f, 0.9f));
            bannerTex = MakeTex(new Color(0.04f, 0.05f, 0.08f, 0.88f));
            panelStyle = new GUIStyle(GUI.skin.box) { padding = new RectOffset(12, 12, 10, 10) };
            panelStyle.normal.background = panelTex;
            titleStyle = new GUIStyle(GUI.skin.label) { fontSize = 21, fontStyle = FontStyle.Bold };
            titleStyle.normal.textColor = new Color(0.35f, 0.85f, 1f);
            headerStyle = new GUIStyle(GUI.skin.label) { fontSize = 13, fontStyle = FontStyle.Bold, margin = new RectOffset(4, 4, 10, 2) };
            headerStyle.normal.textColor = new Color(1f, 0.75f, 0.3f);
            labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 13 };
            labelStyle.normal.textColor = new Color(0.92f, 0.94f, 0.97f);
            smallStyle = new GUIStyle(labelStyle) { fontSize = 11, wordWrap = true, richText = true };
            smallStyle.normal.textColor = new Color(0.7f, 0.75f, 0.82f);
            valueStyle = new GUIStyle(labelStyle) { alignment = TextAnchor.MiddleRight, fontStyle = FontStyle.Bold };
            bannerStyle = new GUIStyle(GUI.skin.box) { fontSize = 14, alignment = TextAnchor.MiddleCenter, richText = true, wordWrap = true };
            bannerStyle.normal.background = bannerTex;
            bannerStyle.normal.textColor = Color.white;
            buttonStyle = new GUIStyle(GUI.skin.button) { fontSize = 12, fixedHeight = 26 };
        }

        static Texture2D MakeTex(Color c)
        {
            var t = new Texture2D(1, 1) { hideFlags = HideFlags.HideAndDontSave };
            t.SetPixel(0, 0, c);
            t.Apply();
            return t;
        }

        void OnDestroy()
        {
            if (panelTex) Destroy(panelTex);
            if (bannerTex) Destroy(bannerTex);
        }

        // ================================================================ GUI

        void OnGUI()
        {
            if (sim == null) return;
            if (panelStyle == null) InitStyles();
            HandleKeys(Event.current);

            scale = Mathf.Clamp(Screen.height / 900f, 0.75f, 2f);
            GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1f));
            float vw = Screen.width / scale, vh = Screen.height / scale;

            DrawBanner(vw);
            panelScreen = rightScreen = resultsScreen = Rect.zero;
            if (panelVisible)
            {
                var panel = new Rect(10f, 10f, PanelWidth, vh - 20f);
                panelScreen = ToScreen(panel);
                GUILayout.BeginArea(panel, panelStyle);
                scroll = GUILayout.BeginScrollView(scroll);
                DrawPanel();
                GUILayout.EndScrollView();
                GUILayout.EndArea();

                DrawRightColumn(vw, vh);
                if (sim.Experiment != ExperimentStage.None) DrawResults(vw, vh);
            }
            GUI.Label(new Rect(PanelWidth + 20f, vh - 24f, 700f, 22f),
                "Space Start/Pause   R Reset   1/2/3 Views   F Fusion mode   G Graphs   V Vision view   H Hide panel", smallStyle);
            GUI.matrix = Matrix4x4.identity;

            if (pending != null) { var a = pending; pending = null; a(); }
        }

        Rect ToScreen(Rect r) { return new Rect(r.x * scale, r.y * scale, r.width * scale, r.height * scale); }

        void DrawBanner(float vw)
        {
            string state = sim.Mission.ToString();
            string mode = FusionNames[(int)sim.fusionMode];
            string text = "<b>" + Pretty(state) + "</b>   |   Fusion: " + mode + "   |   t = " + (sim.SimTime - sim.MissionStartTime).ToString("0") + " s\n<size=12>" + sim.LastEvent + "</size>";
            float left = panelVisible ? PanelWidth + 20f : 10f;
            float right = panelVisible ? RightWidth + 20f : 10f;
            GUI.Box(new Rect(left, 10f, Mathf.Max(300f, vw - left - right), 46f), text, bannerStyle);
        }

        static string Pretty(string s)
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < s.Length; i++) { if (i > 0 && char.IsUpper(s[i])) sb.Append(' '); sb.Append(s[i]); }
            return sb.ToString().ToUpperInvariant();
        }

        void DrawPanel()
        {
            GUILayout.Label("SkyEye", titleStyle);
            GUILayout.Label("AR drone search-and-rescue simulator: sensor fusion, optical flow, marker landing", smallStyle);
            if (sim.ConfigError != null)
            {
                GUILayout.Label("<color=#ff6060>Setup error:</color> " + sim.ConfigError, smallStyle);
                return;
            }

            // ---- Mission controls
            Header("Mission");
            GUILayout.BeginHorizontal();
            string run = sim.State == RunState.Running ? "Pause" : (sim.State == RunState.Paused ? "Resume" : "Start mission");
            if (GUILayout.Button(run, buttonStyle)) Defer(sim.TogglePause);
            if (GUILayout.Button("Reset", buttonStyle)) Defer(sim.ResetAll);
            if (GUILayout.Button("Exit", buttonStyle)) Defer(Quit);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Speed x" + sim.timeScale.ToString("0"), labelStyle, GUILayout.Width(80));
            sim.timeScale = Mathf.Round(GUILayout.HorizontalSlider(sim.timeScale, 1f, 4f));
            GUILayout.EndHorizontal();
            GUI.enabled = !sim.ExperimentRunning;
            bool pl = GUILayout.Toggle(sim.precisionLanding, " Precision landing on the marker");
            if (pl != sim.precisionLanding) Defer(() => sim.precisionLanding = pl);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Wind", labelStyle, GUILayout.Width(80));
            sim.windStrength = GUILayout.HorizontalSlider(sim.windStrength, 0f, 3f);
            GUILayout.Label(sim.windStrength.ToString("0.0"), valueStyle, GUILayout.Width(30));
            GUILayout.EndHorizontal();

            // ---- Sensor fusion (Unit 6)
            Header("Sensor fusion (Unit 6)");
            int fm = GUILayout.Toolbar((int)sim.fusionMode, FusionNames, buttonStyle);
            if (fm != (int)sim.fusionMode) Defer(() => sim.SetFusionMode((FusionMode)fm));
            GUI.enabled = true;
            double lat, lon;
            sim.Geo.ToGeo(sim.LastGps.x, sim.LastGps.z, out lat, out lon);
            Row("GPS fix", lat.ToString("0.000000", CultureInfo.InvariantCulture) + ", " + lon.ToString("0.000000", CultureInfo.InvariantCulture));
            Row("GPS error (raw)", sim.GpsError.ToString("0.00") + " m");
            Row("Fused position error", sim.HorizontalError.ToString("0.00") + " m");
            Row("RMS error this flight", sim.EstimateStats.Rms.ToString("0.00") + " m");
            Row("Altitude (baro / laser)", sim.BaroAltitude.ToString("0.0") + " / " + sim.RangefinderHeight.ToString("0.0") + " m");
            Row("Speed (fused)", DroneSimulation.Horizontal(sim.EstimatedVelocity).magnitude.ToString("0.0") + " m/s");
            Row("Pitch true / gyro / fused", sim.PitchTrue.ToString("0.0") + " / " + sim.PitchGyroOnly.ToString("0") + " / " + sim.PitchFused.ToString("0.0") + "°");

            // ---- Vision (Unit 5)
            Header("Computer vision (Unit 5)");
            if (vision != null)
            {
                Row("Optical flow", vision.FlowValid ? vision.FlowInliers + " points tracked" : "no texture / lost");
                Row("Flow velocity", sim.LastFlowVelocity.x.ToString("0.0") + " E, " + sim.LastFlowVelocity.y.ToString("0.0") + " N m/s");
                Row("Landing marker", vision.LastMarker.Found ? "ID 0x" + vision.LastMarker.Id.ToString("X4") + " found" : "not visible");
                if (vision.LastMarker.Found)
                    Row("Marker pose", "h " + sim.MarkerOffset.y.ToString("0.0") + " m, off " + sim.MarkerOffset.x.ToString("0.0") + " E " + sim.MarkerOffset.z.ToString("0.0") + " N");
                Row("Survivor (orange)", sim.SurvivorConfirmed ? "CONFIRMED" : (vision.LastBlob.Found ? "seen " + sim.SurvivorSightings + "/5" : "searching"));
                Row("Vision time", vision.ProcessingMs.ToString("0.0") + " ms/frame");
            }
            if (sim.LandingError >= 0f) Row("Landing error", sim.LandingError.ToString("0.00") + " m");

            // ---- Views and AR
            Header("View & AR overlay");
            if (cameraRig != null)
            {
                int vm = GUILayout.Toolbar((int)cameraRig.Mode, ViewNames, buttonStyle);
                if (vm != (int)cameraRig.Mode) Defer(() => cameraRig.SetMode((ViewMode)vm));
            }
            if (arOverlay != null)
            {
                bool lab = GUILayout.Toggle(arOverlay.showLabels, " GPS-tagged AR labels");
                if (lab != arOverlay.showLabels) Defer(() => arOverlay.showLabels = lab);
                bool occ = GUILayout.Toggle(arOverlay.occlusionHandling, " Occlusion handling");
                if (occ != arOverlay.occlusionHandling) Defer(() => arOverlay.occlusionHandling = occ);
            }
            bool tr = GUILayout.Toggle(sim.ShowTrails, " Trails: true (white), fused (cyan), GPS (red)");
            if (tr != sim.ShowTrails) Defer(() => sim.SetShowTrails(tr));

            // ---- Rendering (Unit 5)
            Header("Lighting & rendering (Unit 5)");
            if (environment != null)
            {
                int wv = GUILayout.Toolbar((int)environment.Current, WeatherNames, buttonStyle);
                if (wv != (int)environment.Current) Defer(() =>
                {
                    environment.Apply((Weather)wv);
                    if (renderModes != null) renderModes.ApplyIllumination();
                });
            }
            if (renderModes != null)
            {
                var rm = renderModes;
                int g = GUILayout.Toolbar((int)rm.Geometry, GeoNames, buttonStyle);
                if (g != (int)rm.Geometry) Defer(() => rm.SetGeometry((GeometryMode)g));
                int il = GUILayout.Toolbar((int)rm.Illumination, LightNames, buttonStyle);
                if (il != (int)rm.Illumination) Defer(() => rm.SetIllumination((IlluminationMode)il));
                bool sh = GUILayout.Toggle(rm.ShadowsOn, " Shadows (light occlusion)");
                if (sh != rm.ShadowsOn) Defer(() => rm.SetShadows(sh));
                bool aa = GUILayout.Toggle(rm.AntiAliasingOn, " Anti-aliasing (MSAA)");
                if (aa != rm.AntiAliasingOn) Defer(() => rm.SetAntiAliasing(aa));
                Row("Triangles drawn", rm.TriangleCount.ToString("N0") + " / " + rm.RenderedObjects + " objects");
            }

            // ---- Experiment
            Header("Experiment & data");
            GUI.enabled = !sim.ExperimentRunning;
            if (GUILayout.Button("Run fusion comparison (3 missions)", buttonStyle))
                Defer(() => { sim.StartExperiment(); sim.timeScale = 4f; });
            GUI.enabled = true;
            GUILayout.Label("Flies the same mission (same seed, wind and sensor noise) with GPS only, GPS+IMU and GPS+IMU+optical flow.", smallStyle);
            if (GUILayout.Button("Export flight data to CSV", buttonStyle)) Defer(() => sim.ExportCsv());
            if (!string.IsNullOrEmpty(sim.LastExportPath)) GUILayout.Label("Saved to: " + sim.LastExportPath, smallStyle);
        }

        void DrawRightColumn(float vw, float vh)
        {
            float x = vw - RightWidth - 10f;
            float y = 64f;
            float h = 0f;
            if (showVision && vision != null && vision.DebugTexture != null) h += 300f;
            if (showGraphs) h += 290f;
            if (h <= 0f) return;
            var area = new Rect(x, y, RightWidth, h);
            rightScreen = ToScreen(area);
            GUI.Box(area, GUIContent.none, panelStyle);
            float cy = y + 8f;
            if (showVision && vision != null && vision.DebugTexture != null)
            {
                GUI.Label(new Rect(x + 10f, cy, RightWidth - 20f, 20f), "<b>Drone camera - computer vision</b>", smallStyle);
                GUI.DrawTexture(new Rect(x + 10f, cy + 20f, 240f, 240f), vision.DebugTexture, ScaleMode.StretchToFill, false);
                GUI.Label(new Rect(x + 256f, cy + 24f, 70f, 240f),
                    "<color=#3cff5a>flow</color>\n<color=#ff28ff>marker</color>\n<color=#ffa000>survivor</color>\n<color=#ff3c3c>lost</color>", smallStyle);
                cy += 290f;
            }
            if (showGraphs && errorGraph != null)
            {
                GUI.Label(new Rect(x + 10f, cy, RightWidth - 20f, 20f), "<b>Position error (m)</b>  <color=#ff5050>raw GPS</color>  <color=#3cdcff>fused</color>  (0-6 m)", smallStyle);
                GUI.DrawTexture(new Rect(x + 10f, cy + 20f, 300f, 100f), errorGraph.Texture);
                cy += 135f;
                GUI.Label(new Rect(x + 10f, cy, RightWidth - 20f, 20f), "<b>Pitch (°)</b>  true  <color=#ffaa28>gyro only</color>  <color=#ff5050>accel</color>  <color=#50ff78>complementary</color>", smallStyle);
                GUI.DrawTexture(new Rect(x + 10f, cy + 20f, 300f, 100f), pitchGraph.Texture);
            }
        }

        void DrawResults(float vw, float vh)
        {
            float w = 560f, h = 200f;
            var r = new Rect((vw - w) * 0.5f + PanelWidth * 0.3f, vh - h - 34f, w, h);
            resultsScreen = ToScreen(r);
            GUILayout.BeginArea(r, panelStyle);
            GUILayout.Label(sim.ExperimentRunning
                ? "Fusion comparison: flying mission " + (sim.Results.Count + 1) + " of 3 (" + FusionNames[(int)sim.ExperimentCurrentMode] + ")"
                : "Fusion comparison results (same seed, simulated)", headerStyle);
            GUILayout.BeginHorizontal();
            Cell("Mode", 120, true); Cell("RMS err (m)", 85, true); Cell("Max err (m)", 85, true);
            Cell("Landing (m)", 85, true); Cell("Survivor (m)", 85, true); Cell("Time (s)", 60, true);
            GUILayout.EndHorizontal();
            foreach (var res in sim.Results)
            {
                GUILayout.BeginHorizontal();
                Cell(FusionNames[(int)res.Mode], 120, false);
                Cell(res.RmsEstimateError.ToString("0.00"), 85, false);
                Cell(res.MaxEstimateError.ToString("0.00"), 85, false);
                Cell(res.LandingError >= 0f ? res.LandingError.ToString("0.00") : "-", 85, false);
                Cell(res.SurvivorError >= 0f ? res.SurvivorError.ToString("0.00") : "not found", 85, false);
                Cell(res.Completed ? res.MissionTime.ToString("0") : "timeout", 60, false);
                GUILayout.EndHorizontal();
            }
            if (sim.Results.Count > 0)
                GUILayout.Label("Raw GPS RMS error for reference: " + sim.Results[0].RmsGpsError.ToString("0.00") + " m", smallStyle);
            GUILayout.EndArea();
        }

        void Cell(string text, float width, bool header)
        {
            GUILayout.Label(header ? "<b>" + text + "</b>" : text, smallStyle, GUILayout.Width(width));
        }

        void Header(string text) { GUILayout.Label(text, headerStyle); }

        void Row(string name, string value)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(name, labelStyle);
            GUILayout.FlexibleSpace();
            GUILayout.Label(value, valueStyle);
            GUILayout.EndHorizontal();
        }

        void HandleKeys(Event e)
        {
            if (e.type != EventType.KeyDown) return;
            switch (e.keyCode)
            {
                case KeyCode.Space: Defer(sim.TogglePause); break;
                case KeyCode.R: Defer(sim.ResetAll); break;
                case KeyCode.Alpha1: if (cameraRig) Defer(() => cameraRig.SetMode(ViewMode.Chase)); break;
                case KeyCode.Alpha2: if (cameraRig) Defer(() => cameraRig.SetMode(ViewMode.FirstPersonAR)); break;
                case KeyCode.Alpha3: if (cameraRig) Defer(() => cameraRig.SetMode(ViewMode.TopMap)); break;
                case KeyCode.F:
                    if (!sim.ExperimentRunning) Defer(() => sim.SetFusionMode((FusionMode)(((int)sim.fusionMode + 1) % 3)));
                    break;
                case KeyCode.G: Defer(() => showGraphs = !showGraphs); break;
                case KeyCode.V: Defer(() => showVision = !showVision); break;
                case KeyCode.H: Defer(() => panelVisible = !panelVisible); break;
                default: return;
            }
            e.Use();
        }

        static void Quit()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }
    }
}
