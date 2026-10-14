// SkyEye - Generates the complete scene, textures, fiducial marker and materials.
// Menu: SkyEye > 1. Generate Scene
using System.Collections.Generic;
using SkyEye.Core;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

namespace SkyEye.EditorTools
{
    public static class SkyEyeSceneBuilder
    {
        public const string Root = "Assets/SkyEye";
        public const string SceneDir = Root + "/Scenes";
        public const string MatDir = Root + "/Materials";
        public const string TexDir = Root + "/Textures";
        public const string ScenePath = SceneDir + "/SkyEye_Main.unity";
        public const int MarkerId = 0x6A3D;

        static bool usingSrp;

        class Mats
        {
            public Material ground, water, concrete, marker, wallA, wallB, wallC, roofA, roofB, hospital, cross, tent, tree, trunk;
            public Material droneBody, droneArm, rotor, ledRedOn, ledGreenOn, ledOff, orange, skin, supply;
            public Material trailTrue, trailEst, trailGps, wire;
        }

        [MenuItem("SkyEye/1. Generate Scene", false, 1)]
        public static void GenerateFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Generate();
            EditorUtility.DisplayDialog("SkyEye", "Scene generated:\n" + ScenePath + "\n\nPress Play, then click 'Start mission'.", "OK");
        }

        public static void Generate()
        {
            usingSrp = GraphicsSettings.currentRenderPipeline != null;
            EnsureFolderPath(SceneDir);
            EnsureFolderPath(MatDir);
            EnsureFolderPath(TexDir);
            AssetDatabase.Refresh();

            var groundTex = CreateGroundTexture();
            var markerTex = CreateMarkerTexture(MarkerId);
            Mats m = CreateMaterials(groundTex, markerTex);

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            RenderSettings.skybox = null;
            RenderSettings.ambientMode = AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.5f, 0.54f, 0.6f);
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.Linear;
            RenderSettings.fogStartDistance = 200f;
            RenderSettings.fogEndDistance = 500f;

            var sunGo = new GameObject("Sun");
            var sun = sunGo.AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.intensity = 1.15f;
            sun.shadows = LightShadows.Soft;
            sunGo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

            // ---------------- world
            var world = new GameObject("FloodZone").transform;
            var ground = Prim(PrimitiveType.Plane, "Ground", world, new Vector3(0f, 0f, 10f), new Vector3(30f, 1f, 30f), m.ground, true);
            ground.GetComponent<Renderer>().sharedMaterial.mainTextureScale = new Vector2(6f, 6f);
            EditorUtility.SetDirty(m.ground);

            // Flood water (textureless: optical flow cannot track over calm water - a real limitation)
            Prim(PrimitiveType.Cube, "River", world, new Vector3(0f, 0.05f, -32f), new Vector3(300f, 0.1f, 14f), m.water, true);
            Prim(PrimitiveType.Cube, "FloodedField_W", world, new Vector3(-62f, 0.05f, 40f), new Vector3(30f, 0.1f, 60f), m.water, true);
            Prim(PrimitiveType.Cube, "FloodedField_E", world, new Vector3(62f, 0.05f, 30f), new Vector3(26f, 0.1f, 50f), m.water, true);

            var rng = new System.Random(11);
            var houses = new GameObject("Houses").transform;
            houses.SetParent(world, false);
            Vector3 survivorRoof = Vector3.zero;
            Material[] walls = { m.wallA, m.wallB, m.wallC };
            Material[] roofs = { m.roofA, m.roofB };
            float[] xs = { -40f, -25f, -10f, 5f, 20f, 35f };
            float[] zs = { 5f, 25f, 45f, 65f };
            foreach (float hx in xs)
                foreach (float hz in zs)
                {
                    bool isSurvivorHouse = Mathf.Approximately(hx, 20f) && Mathf.Approximately(hz, 45f);
                    if (!isSurvivorHouse && rng.NextDouble() < 0.2) continue;
                    float h = isSurvivorHouse ? 5f : 3f + (float)rng.NextDouble() * 5f;
                    float jx = isSurvivorHouse ? 0f : (float)(rng.NextDouble() - 0.5) * 4f;
                    float jz = isSurvivorHouse ? 0f : (float)(rng.NextDouble() - 0.5) * 4f;
                    var pos = new Vector3(hx + jx, 0f, hz + jz);
                    var house = new GameObject(isSurvivorHouse ? "House_Survivor" : "House").transform;
                    house.SetParent(houses, false);
                    house.localPosition = pos;
                    Prim(PrimitiveType.Cube, "Walls", house, new Vector3(0f, h * 0.5f, 0f), new Vector3(7f, h, 7f), walls[rng.Next(walls.Length)], true);
                    Prim(PrimitiveType.Cube, "Roof", house, new Vector3(0f, h + 0.15f, 0f), new Vector3(7.6f, 0.3f, 7.6f), roofs[rng.Next(roofs.Length)], true);
                    if (isSurvivorHouse) survivorRoof = pos + Vector3.up * (h + 0.3f);
                }

            // Hospital and relief camp (GPS-tagged points of interest)
            var hospitalPos = new Vector3(-55f, 0f, -55f);
            var hosp = new GameObject("Hospital").transform;
            hosp.SetParent(world, false);
            hosp.localPosition = hospitalPos;
            Prim(PrimitiveType.Cube, "Block", hosp, new Vector3(0f, 6f, 0f), new Vector3(18f, 12f, 12f), m.hospital, true);
            Prim(PrimitiveType.Cube, "CrossV", hosp, new Vector3(0f, 12.2f, 0f), new Vector3(1.2f, 0.2f, 5f), m.cross, false);
            Prim(PrimitiveType.Cube, "CrossH", hosp, new Vector3(0f, 12.2f, 0f), new Vector3(5f, 0.2f, 1.2f), m.cross, false);
            var campPos = new Vector3(48f, 0f, -55f);
            var camp = new GameObject("ReliefCamp").transform;
            camp.SetParent(world, false);
            camp.localPosition = campPos;
            for (int i = 0; i < 6; i++)
                Prim(PrimitiveType.Cube, "Tent", camp, new Vector3((i % 3) * 6f - 6f, 1.2f, (i / 3) * 6f - 3f), new Vector3(4f, 2.4f, 4f), m.tent, true);

            // Trees (no colliders)
            var trees = new GameObject("Trees").transform;
            trees.SetParent(world, false);
            for (int i = 0; i < 40; i++)
            {
                var p = new Vector3((float)(rng.NextDouble() * 160 - 80), 0f, (float)(rng.NextDouble() * 150 - 75));
                if (Mathf.Abs(p.z + 32f) < 9f || (Mathf.Abs(p.x) < 10f && Mathf.Abs(p.z + 60f) < 10f)) continue;
                var t = new GameObject("Tree").transform;
                t.SetParent(trees, false);
                t.localPosition = p;
                Prim(PrimitiveType.Cylinder, "Trunk", t, new Vector3(0f, 1.5f, 0f), new Vector3(0.4f, 1.5f, 0.4f), m.trunk, false);
                Prim(PrimitiveType.Sphere, "Crown", t, new Vector3(0f, 4f, 0f), new Vector3(3.2f, 3.2f, 3.2f), m.tree, false);
            }

            // ---------------- helipad with fiducial marker
            var padPos = new Vector3(0f, 0f, -60f);
            var pad = new GameObject("Helipad").transform;
            pad.SetParent(world, false);
            pad.localPosition = padPos;
            Prim(PrimitiveType.Cube, "Slab", pad, new Vector3(0f, 0.2f, 0f), new Vector3(12f, 0.4f, 12f), m.concrete, true);
            // Marker quad: 8 cells (6 black-bordered + white quiet zone); black square = 4 m
            float markerQuad = 4f * MarkerCodec.CellsWithQuiet / MarkerCodec.TotalCells;
            var markerGo = Prim(PrimitiveType.Quad, "LandingMarker", pad, new Vector3(0f, 0.41f, 0f), new Vector3(markerQuad, markerQuad, 1f), m.marker, false);
            markerGo.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);

            // ---------------- survivor on a roof with an orange life raft
            var survivor = new GameObject("Survivor").transform;
            survivor.position = survivorRoof;
            Prim(PrimitiveType.Cube, "LifeRaft", survivor, new Vector3(1.2f, 0.12f, 0.6f), new Vector3(2.6f, 0.25f, 2.0f), m.orange, false);
            Prim(PrimitiveType.Capsule, "Body", survivor, new Vector3(-1f, 0.85f, -0.8f), new Vector3(0.6f, 0.55f, 0.45f), m.orange, false);
            Prim(PrimitiveType.Sphere, "Head", survivor, new Vector3(-1f, 1.65f, -0.8f), Vector3.one * 0.35f, m.skin, false);
            var armL = new GameObject("ArmL").transform; armL.SetParent(survivor, false); armL.localPosition = new Vector3(-1.3f, 1.3f, -0.8f);
            var armR = new GameObject("ArmR").transform; armR.SetParent(survivor, false); armR.localPosition = new Vector3(-0.7f, 1.3f, -0.8f);
            Prim(PrimitiveType.Cube, "Arm", armL, new Vector3(0f, -0.3f, 0f), new Vector3(0.12f, 0.6f, 0.12f), m.skin, false);
            Prim(PrimitiveType.Cube, "Arm", armR, new Vector3(0f, -0.3f, 0f), new Vector3(0.12f, 0.6f, 0.12f), m.skin, false);
            var sa = survivor.gameObject.AddComponent<SurvivorAnimator>();
            sa.leftArm = armL; sa.rightArm = armR;

            // ---------------- drone
            var drone = new GameObject("Drone").transform;
            drone.position = padPos + Vector3.up * 0.7f;
            var model = new GameObject("Model").transform;
            model.SetParent(drone, false);
            model.localScale = Vector3.one * 1.5f;
            Prim(PrimitiveType.Cube, "Body", model, Vector3.zero, new Vector3(0.5f, 0.16f, 0.6f), m.droneBody, false);
            var arm1 = Prim(PrimitiveType.Cube, "Arm1", model, new Vector3(0f, 0.02f, 0f), new Vector3(1.5f, 0.05f, 0.08f), m.droneArm, false);
            arm1.transform.localRotation = Quaternion.Euler(0f, 45f, 0f);
            var arm2 = Prim(PrimitiveType.Cube, "Arm2", model, new Vector3(0f, 0.02f, 0f), new Vector3(1.5f, 0.05f, 0.08f), m.droneArm, false);
            arm2.transform.localRotation = Quaternion.Euler(0f, -45f, 0f);
            var rotors = new List<Transform>();
            for (int i = 0; i < 4; i++)
            {
                float ang = (45f + 90f * i) * Mathf.Deg2Rad;
                var hub = new GameObject("RotorHub" + i).transform;
                hub.SetParent(model, false);
                hub.localPosition = new Vector3(Mathf.Sin(ang) * 0.53f, 0.08f, Mathf.Cos(ang) * 0.53f);
                Prim(PrimitiveType.Cube, "Blade", hub, Vector3.zero, new Vector3(0.55f, 0.01f, 0.06f), m.rotor, false);
                Prim(PrimitiveType.Cylinder, "Guard", hub, Vector3.zero, new Vector3(0.6f, 0.005f, 0.6f), m.droneArm, false);
                rotors.Add(hub);
            }
            var ledL = Prim(PrimitiveType.Sphere, "LedRed", model, new Vector3(-0.2f, -0.06f, 0.3f), Vector3.one * 0.07f, m.ledOff, false);
            var ledR = Prim(PrimitiveType.Sphere, "LedGreen", model, new Vector3(0.2f, -0.06f, 0.3f), Vector3.one * 0.07f, m.ledOff, false);
            Prim(PrimitiveType.Sphere, "Gimbal", model, new Vector3(0f, -0.12f, 0.15f), Vector3.one * 0.12f, m.droneArm, false);
            Prim(PrimitiveType.Cube, "LegL", model, new Vector3(-0.22f, -0.2f, 0f), new Vector3(0.03f, 0.25f, 0.5f), m.droneArm, false);
            Prim(PrimitiveType.Cube, "LegR", model, new Vector3(0.22f, -0.2f, 0f), new Vector3(0.03f, 0.25f, 0.5f), m.droneArm, false);
            var visual = drone.gameObject.AddComponent<DroneVisual>();
            visual.rotors = rotors.ToArray();
            visual.blinkers = new[] { ledL.GetComponent<Renderer>(), ledR.GetComponent<Renderer>() };
            visual.blinkOn = m.ledRedOn;
            visual.blinkOff = m.ledOff;

            // Downward gimbal camera (renders into a texture at runtime)
            var downGo = new GameObject("DroneDownCamera");
            var downCam = downGo.AddComponent<Camera>();
            downCam.fieldOfView = 60f;
            downCam.nearClipPlane = 0.1f;
            downCam.farClipPlane = 300f;
            downCam.depth = -10f;
            downCam.clearFlags = CameraClearFlags.SolidColor;
            downCam.backgroundColor = new Color(0.55f, 0.72f, 0.92f);
            downGo.transform.SetPositionAndRotation(drone.position + Vector3.down * 0.35f, Quaternion.Euler(90f, 0f, 0f));

            // ---------------- main camera
            var camGo = new GameObject("Main Camera");
            camGo.tag = "MainCamera";
            var cam = camGo.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.55f, 0.72f, 0.92f);
            cam.fieldOfView = 55f;
            cam.nearClipPlane = 0.1f;
            cam.farClipPlane = 1000f;
            camGo.AddComponent<AudioListener>();
            camGo.transform.position = padPos + new Vector3(-8f, 8f, -14f);
            camGo.transform.LookAt(padPos);
            var rig = camGo.AddComponent<CameraRig>();

            // ---------------- managers
            var mgr = new GameObject("SkyEye Manager");
            var sim = mgr.AddComponent<DroneSimulation>();
            var vision = mgr.AddComponent<VisionSystem>();
            var hud = mgr.AddComponent<SkyEyeHUD>();
            var ar = mgr.AddComponent<ArOverlay>();
            var env = mgr.AddComponent<EnvironmentController>();
            var render = mgr.AddComponent<RenderModeController>();

            sim.droneRoot = drone;
            sim.droneVisual = visual;
            sim.vision = vision;
            sim.helipad = markerGo.transform;
            sim.survivor = survivor;
            sim.trueTrailMaterial = m.trailTrue;
            sim.estimateTrailMaterial = m.trailEst;
            sim.gpsTrailMaterial = m.trailGps;
            sim.supplyMaterial = m.supply;
            sim.markerId = MarkerId;
            sim.markerSize = 4f;

            vision.simulation = sim;
            vision.downCamera = downCam;

            rig.simulation = sim;
            rig.hud = hud;

            env.sun = sun;
            env.cameras = new[] { cam, downCam };

            render.sun = sun;
            render.targetCamera = cam;
            render.wireMaterial = m.wire;
            render.environment = env;

            ar.simulation = sim;
            ar.viewCamera = cam;
            var geo = new GeoReference(sim.referenceLatitude, sim.referenceLongitude);
            ar.points = new[]
            {
                Poi(geo, "District Hospital", hospitalPos, 16f, new Color(1f, 0.35f, 0.35f)),
                Poi(geo, "Relief Camp", campPos, 6f, new Color(0.4f, 0.7f, 1f)),
                Poi(geo, "Helipad (home)", padPos, 4f, new Color(0.4f, 1f, 0.5f)),
                Poi(geo, "Flooded school", new Vector3(-25f, 0f, 65f), 12f, new Color(1f, 0.9f, 0.4f))
            };

            hud.sim = sim;
            hud.vision = vision;
            hud.cameraRig = rig;
            hud.arOverlay = ar;
            hud.environment = env;
            hud.renderModes = render;

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.SaveAssets();
            Debug.Log("[SkyEye] Scene saved to " + ScenePath + " (render pipeline: " + (usingSrp ? "URP/SRP" : "Built-in") + ").");
            SkyEyeValidator.Validate(false);
        }

        static PointOfInterest Poi(GeoReference geo, string name, Vector3 local, float height, Color c)
        {
            double lat, lon;
            geo.ToGeo(local.x, local.z, out lat, out lon);
            return new PointOfInterest { name = name, latitude = lat, longitude = lon, height = height, colour = c };
        }

        // ================================================================ textures

        /// <summary>Procedural fields and mud with fine detail so optical flow has features to track.</summary>
        static Texture2D CreateGroundTexture()
        {
            const int size = 512;
            var tex = new Texture2D(size, size, TextureFormat.RGBA32, true);
            var rng = new System.Random(5);
            var lattice = new float[65, 65];
            for (int i = 0; i < 65; i++) for (int j = 0; j < 65; j++) lattice[i, j] = (float)rng.NextDouble();
            var px = new Color32[size * size];
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    float coarse = ValueNoise(lattice, x / 64f, y / 64f);
                    float mid = ValueNoise(lattice, x / 16f + 7.3f, y / 16f + 2.1f);
                    float fine = ValueNoise(lattice, x / 4f + 3.7f, y / 4f + 9.2f);
                    float speck = (float)rng.NextDouble();
                    float v = 0.45f * coarse + 0.3f * mid + 0.17f * fine + 0.08f * speck;
                    // green fields vs grey mud
                    Color grass = new Color(0.30f, 0.42f, 0.24f), mud = new Color(0.36f, 0.36f, 0.30f);
                    Color c = Color.Lerp(mud, grass, Mathf.SmoothStep(0.35f, 0.65f, coarse)) * (0.7f + 0.6f * v);
                    px[y * size + x] = c;
                }
            tex.SetPixels32(px);
            tex.Apply(true);
            tex.wrapMode = TextureWrapMode.Repeat;
            tex.filterMode = FilterMode.Bilinear;
            return SaveTexture(tex, TexDir + "/GroundTexture.asset");
        }

        static float ValueNoise(float[,] lat, float x, float y)
        {
            int x0 = Mathf.FloorToInt(x), y0 = Mathf.FloorToInt(y);
            float fx = x - x0, fy = y - y0;
            fx = fx * fx * (3f - 2f * fx); fy = fy * fy * (3f - 2f * fy);
            int n = lat.GetLength(0) - 1;
            int ax = ((x0 % n) + n) % n, ay = ((y0 % n) + n) % n;
            int bx = (ax + 1) % n, by = (ay + 1) % n;
            float a = lat[ax, ay], b = lat[bx, ay], c = lat[ax, by], d = lat[bx, by];
            return Mathf.Lerp(Mathf.Lerp(a, b, fx), Mathf.Lerp(c, d, fx), fy);
        }

        /// <summary>Marker creation: 8×8 cells (quiet zone + black border + 4×4 ID bits), 8 px per cell.</summary>
        static Texture2D CreateMarkerTexture(int id)
        {
            const int cell = 8;
            int n = MarkerCodec.CellsWithQuiet;
            var tex = new Texture2D(n * cell, n * cell, TextureFormat.RGBA32, false);
            var px = new Color32[n * cell * n * cell];
            var white = new Color32(240, 240, 240, 255);
            var black = new Color32(15, 15, 15, 255);
            for (int y = 0; y < n * cell; y++)
                for (int x = 0; x < n * cell; x++)
                {
                    int col = x / cell;
                    int row = n - 1 - y / cell;   // texture rows go up; marker rows are counted from the top
                    px[y * n * cell + x] = MarkerCodec.CellIsWhite(id, row, col) ? white : black;
                }
            tex.SetPixels32(px);
            tex.Apply(false);
            tex.filterMode = FilterMode.Point;
            tex.wrapMode = TextureWrapMode.Clamp;
            return SaveTexture(tex, TexDir + "/LandingMarker_" + id.ToString("X4") + ".asset");
        }

        static Texture2D SaveTexture(Texture2D tex, string path)
        {
            var existing = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
            if (existing != null) AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(tex, path);
            return tex;
        }

        // ================================================================ materials

        static Shader FindShader(bool unlit)
        {
            Shader s = null;
            if (usingSrp) s = Shader.Find(unlit ? "Universal Render Pipeline/Unlit" : "Universal Render Pipeline/Lit");
            if (s == null) s = Shader.Find(unlit ? "Unlit/Color" : "Standard");
            if (s == null) s = Shader.Find("Sprites/Default");
            return s;
        }

        static Material Mat(string name, Color c, bool unlit = false, float smoothness = 0.15f, Texture2D tex = null)
        {
            string path = MatDir + "/" + name + ".mat";
            Shader shader = (tex != null && unlit) ? FindShader(false) : FindShader(unlit);
            var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat == null) { mat = new Material(shader); AssetDatabase.CreateAsset(mat, path); }
            else if (mat.shader != shader) mat.shader = shader;
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", c);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", c);
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", smoothness);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);
            if (tex != null)
            {
                if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", tex);
                if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", tex);
            }
            mat.enableInstancing = true;
            EditorUtility.SetDirty(mat);
            return mat;
        }

        static Mats CreateMaterials(Texture2D groundTex, Texture2D markerTex)
        {
            var m = new Mats();
            m.ground = Mat("Ground", Color.white, false, 0.05f, groundTex);
            m.water = Mat("FloodWater", new Color(0.22f, 0.38f, 0.5f), false, 0.92f);
            m.concrete = Mat("Concrete", new Color(0.6f, 0.6f, 0.62f));
            m.marker = Mat("LandingMarker", Color.white, false, 0.1f, markerTex);
            m.wallA = Mat("WallA", new Color(0.75f, 0.77f, 0.8f));
            m.wallB = Mat("WallB", new Color(0.62f, 0.68f, 0.74f));
            m.wallC = Mat("WallC", new Color(0.7f, 0.74f, 0.7f));
            m.roofA = Mat("RoofGrey", new Color(0.32f, 0.34f, 0.38f));
            m.roofB = Mat("RoofTeal", new Color(0.2f, 0.42f, 0.45f));
            m.hospital = Mat("Hospital", new Color(0.93f, 0.93f, 0.95f));
            m.cross = Mat("RedCross", new Color(0.85f, 0.08f, 0.1f));
            m.tent = Mat("Tent", new Color(0.2f, 0.4f, 0.85f));
            m.tree = Mat("TreeCrown", new Color(0.16f, 0.36f, 0.18f));
            m.trunk = Mat("TreeTrunk", new Color(0.3f, 0.24f, 0.18f));
            m.droneBody = Mat("DroneBody", new Color(0.15f, 0.16f, 0.18f), false, 0.6f);
            m.droneArm = Mat("DroneArm", new Color(0.3f, 0.31f, 0.33f), false, 0.4f);
            m.rotor = Mat("Rotor", new Color(0.85f, 0.85f, 0.88f));
            m.ledRedOn = Mat("LedOn", new Color(1f, 0.1f, 0.1f), true);
            m.ledGreenOn = Mat("LedGreenOn", new Color(0.1f, 1f, 0.3f), true);
            m.ledOff = Mat("LedOff", new Color(0.2f, 0.2f, 0.2f), true);
            m.orange = Mat("LifeJacketOrange", new Color(1f, 0.45f, 0.05f), false, 0.3f);
            m.skin = Mat("Skin", new Color(0.62f, 0.45f, 0.36f));
            m.supply = Mat("ReliefKit", new Color(1f, 0.9f, 0.1f));
            m.trailTrue = Mat("TrailTrue", new Color(1f, 1f, 1f), true);
            m.trailEst = Mat("TrailFused", new Color(0.2f, 0.9f, 1f), true);
            m.trailGps = Mat("TrailGPS", new Color(1f, 0.25f, 0.25f), true);
            m.wire = Mat("Wireframe", new Color(0.3f, 1f, 0.6f), true);
            return m;
        }

        // ================================================================ helpers

        /// <summary>Creates a primitive. Colliders are kept only where raycasts need them (ground, buildings).</summary>
        static GameObject Prim(PrimitiveType type, string name, Transform parent, Vector3 localPos, Vector3 localScale,
                               Material mat, bool keepCollider)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localScale = localScale;
            if (!keepCollider)
            {
                var col = go.GetComponent<Collider>();
                if (col != null) Object.DestroyImmediate(col);
            }
            go.GetComponent<Renderer>().sharedMaterial = mat;
            return go;
        }

        static void EnsureFolderPath(string path)
        {
            string[] parts = path.Split('/');
            string current = parts[0];
            for (int i = 1; i < parts.Length; i++)
            {
                string next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                {
                    string guid = AssetDatabase.CreateFolder(current, parts[i]);
                    if (string.IsNullOrEmpty(guid))
                    {
                        System.IO.Directory.CreateDirectory(next);
                        AssetDatabase.ImportAsset(next);
                    }
                }
                current = next;
            }
        }
    }

    /// <summary>Offers to generate the scene the first time the project is opened.</summary>
    [InitializeOnLoad]
    public static class SkyEyeFirstRun
    {
        const string Key = "SkyEye.FirstRunAsked";

        static SkyEyeFirstRun()
        {
            if (Application.isBatchMode) return;
            EditorApplication.delayCall += () =>
            {
                if (SessionState.GetBool(Key, false)) return;
                SessionState.SetBool(Key, true);
                if (AssetDatabase.LoadAssetAtPath<SceneAsset>(SkyEyeSceneBuilder.ScenePath) != null) return;
                if (EditorUtility.DisplayDialog("SkyEye setup", "Generate the SkyEye scene now?\n(Also available from the menu SkyEye > 1. Generate Scene.)", "Generate now", "Later"))
                    SkyEyeSceneBuilder.GenerateFromMenu();
            };
        }
    }
}
