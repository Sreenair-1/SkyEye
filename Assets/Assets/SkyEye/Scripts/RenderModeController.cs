// SkyEye - Rendering technique switches (Unit 5: rendering techniques).
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace SkyEye
{
    public enum GeometryMode { Polygon, Wireframe, PolygonWithWireframe }
    public enum IlluminationMode { Full, LocalOnly, AmbientOnly }

    /// <summary>
    /// Live switches that show rendering concepts on the same scene:
    ///  - polygon (filled triangles) vs wireframe (triangle edges) vs both,
    ///  - illumination: full (direct + ambient + shadows), local only (direct light, no ambient
    ///    term standing in for secondary / global light), ambient only (flat, unlit look),
    ///  - shadows on/off (light occlusion) and MSAA anti-aliasing on/off (rasterization aliasing).
    /// Wireframes are line meshes built from each mesh's triangles, so they work in every pipeline.
    /// </summary>
    public class RenderModeController : MonoBehaviour
    {
        public Light sun;
        public Camera targetCamera;
        public Material wireMaterial;
        public EnvironmentController environment;

        public GeometryMode Geometry { get; private set; }
        public IlluminationMode Illumination { get; private set; }
        public bool ShadowsOn { get; private set; }
        public bool AntiAliasingOn { get; private set; }
        public int TriangleCount { get; private set; }
        public int RenderedObjects { get; private set; }

        readonly Dictionary<Mesh, Mesh> lineMeshCache = new Dictionary<Mesh, Mesh>();
        readonly Dictionary<Renderer, GameObject> wireOf = new Dictionary<Renderer, GameObject>();
        int originalMsaa = -1;
        LightShadows originalShadows = LightShadows.Soft;
        float timer;

        void Start()
        {
            if (sun != null) originalShadows = sun.shadows == LightShadows.None ? LightShadows.Soft : sun.shadows;
            ShadowsOn = sun == null || sun.shadows != LightShadows.None;
            originalMsaa = GetMsaa();
            AntiAliasingOn = originalMsaa > 1;
            CountTriangles();
        }

        void OnDestroy()
        {
            if (originalMsaa > 0) SetMsaa(originalMsaa); // MSAA lives in a project asset: restore it
            foreach (var m in lineMeshCache.Values) if (m) Destroy(m);
        }

        void Update()
        {
            timer -= Time.unscaledDeltaTime;
            if (timer > 0f) return;
            timer = 1f;
            if (Geometry != GeometryMode.Polygon) ApplyGeometryToAll(); // picks up new objects (e.g. relief kits)
            CountTriangles();
        }

        // ---------------------------------------------------------------- geometry

        public void SetGeometry(GeometryMode mode)
        {
            Geometry = mode;
            ApplyGeometryToAll();
        }

        void ApplyGeometryToAll()
        {
            foreach (var r in FindObjectsByType<MeshRenderer>(FindObjectsSortMode.None)) ApplyGeometry(r);
        }

        void ApplyGeometry(Renderer r)
        {
            if (r == null || r.gameObject.name == "Wire") return;
            bool wantWire = Geometry != GeometryMode.Polygon;
            GameObject wire;
            if (!wireOf.TryGetValue(r, out wire) || wire == null)
            {
                if (!wantWire) { r.enabled = true; return; }
                wire = CreateWire(r);
                if (wire == null) return;
                wireOf[r] = wire;
            }
            wire.SetActive(wantWire);
            r.enabled = Geometry != GeometryMode.Wireframe;
        }

        GameObject CreateWire(Renderer r)
        {
            var mf = r.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null || r.isPartOfStaticBatch || wireMaterial == null) return null;
            Mesh lines;
            if (!lineMeshCache.TryGetValue(mf.sharedMesh, out lines))
            {
                lines = BuildLineMesh(mf.sharedMesh);
                lineMeshCache[mf.sharedMesh] = lines;
            }
            var go = new GameObject("Wire");
            go.transform.SetParent(r.transform, false);
            go.AddComponent<MeshFilter>().sharedMesh = lines;
            var lr = go.AddComponent<MeshRenderer>();
            lr.sharedMaterial = wireMaterial;
            lr.shadowCastingMode = ShadowCastingMode.Off;
            lr.receiveShadows = false;
            return go;
        }

        /// <summary>Each triangle (a, b, c) contributes edges ab, bc, ca; shared edges are kept once.</summary>
        static Mesh BuildLineMesh(Mesh source)
        {
            var verts = source.vertices;
            var tris = source.triangles;
            var seen = new HashSet<long>();
            var idx = new List<int>(tris.Length * 2);
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                AddEdge(tris[i], tris[i + 1], seen, idx);
                AddEdge(tris[i + 1], tris[i + 2], seen, idx);
                AddEdge(tris[i + 2], tris[i], seen, idx);
            }
            var m = new Mesh { name = source.name + "_wire" };
            if (verts.Length > 65000) m.indexFormat = IndexFormat.UInt32;
            m.vertices = verts;
            m.SetIndices(idx.ToArray(), MeshTopology.Lines, 0);
            m.RecalculateBounds();
            return m;
        }

        static void AddEdge(int a, int b, HashSet<long> seen, List<int> idx)
        {
            int lo = Mathf.Min(a, b), hi = Mathf.Max(a, b);
            long key = ((long)lo << 32) | (uint)hi;
            if (!seen.Add(key)) return;
            idx.Add(lo);
            idx.Add(hi);
        }

        void CountTriangles()
        {
            int tris = 0, objs = 0;
            foreach (var r in FindObjectsByType<MeshRenderer>(FindObjectsSortMode.None))
            {
                if (!r.enabled || !r.gameObject.activeInHierarchy || r.gameObject.name == "Wire") continue;
                var mf = r.GetComponent<MeshFilter>();
                if (mf == null || mf.sharedMesh == null) continue;
                objs++;
                if (!r.isPartOfStaticBatch) tris += (int)(mf.sharedMesh.GetIndexCount(0) / 3);
            }
            TriangleCount = tris;
            RenderedObjects = objs;
        }

        // ---------------------------------------------------------------- illumination

        public void SetIllumination(IlluminationMode mode)
        {
            Illumination = mode;
            ApplyIllumination();
        }

        /// <summary>Re-applies the illumination mode on top of the current weather.</summary>
        public void ApplyIllumination()
        {
            if (environment != null) environment.Apply(environment.Current);
            switch (Illumination)
            {
                case IlluminationMode.LocalOnly:
                    RenderSettings.ambientLight = Color.black;
                    break;
                case IlluminationMode.AmbientOnly:
                    if (sun != null) sun.intensity = 0f;
                    RenderSettings.ambientLight = Color.white * 0.9f;
                    break;
            }
            if (sun != null) sun.shadows = ShadowsOn ? originalShadows : LightShadows.None;
        }

        public void SetShadows(bool on)
        {
            ShadowsOn = on;
            if (sun != null) sun.shadows = on ? originalShadows : LightShadows.None;
        }

        // ---------------------------------------------------------------- anti-aliasing

        public void SetAntiAliasing(bool on)
        {
            AntiAliasingOn = on;
            SetMsaa(on ? Mathf.Max(4, originalMsaa) : 1);
            if (targetCamera != null) targetCamera.allowMSAA = on;
        }

        static int GetMsaa()
        {
            var rp = GraphicsSettings.currentRenderPipeline;
            if (rp != null)
            {
                var prop = rp.GetType().GetProperty("msaaSampleCount");
                if (prop != null) return (int)prop.GetValue(rp, null);
            }
            return Mathf.Max(1, QualitySettings.antiAliasing);
        }

        static void SetMsaa(int samples)
        {
            var rp = GraphicsSettings.currentRenderPipeline;
            if (rp != null)
            {
                var prop = rp.GetType().GetProperty("msaaSampleCount");
                if (prop != null && prop.CanWrite) prop.SetValue(rp, samples, null);
            }
            QualitySettings.antiAliasing = samples <= 1 ? 0 : samples;
        }
    }
}
