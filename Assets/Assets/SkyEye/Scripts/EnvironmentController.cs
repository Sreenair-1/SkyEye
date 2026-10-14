// SkyEye - Lighting conditions: clear day, dusk and fog (Unit 5: lighting conditions).
using UnityEngine;

namespace SkyEye
{
    public enum Weather { ClearDay, Dusk, Fog }

    /// <summary>
    /// Changes the sun, ambient light and fog. These also change what the drone's camera sees:
    /// at dusk the image is darker and in fog the contrast drops, so marker detection range
    /// and optical-flow quality fall - a real effect of rendering, not a scripted one.
    /// </summary>
    public class EnvironmentController : MonoBehaviour
    {
        public Light sun;
        public Camera[] cameras;
        public Weather Current { get; private set; }

        void Start() { Apply(Weather.ClearDay); }

        public void Apply(Weather w)
        {
            Current = w;
            Color sky;
            switch (w)
            {
                case Weather.Dusk:
                    sky = new Color(0.85f, 0.5f, 0.35f);
                    if (sun) { sun.intensity = 0.45f; sun.color = new Color(1f, 0.6f, 0.4f); sun.transform.rotation = Quaternion.Euler(12f, -60f, 0f); }
                    RenderSettings.ambientLight = new Color(0.32f, 0.25f, 0.25f);
                    RenderSettings.fog = true;
                    RenderSettings.fogMode = FogMode.Linear;
                    RenderSettings.fogStartDistance = 120f;
                    RenderSettings.fogEndDistance = 400f;
                    break;
                case Weather.Fog:
                    sky = new Color(0.72f, 0.74f, 0.76f);
                    if (sun) { sun.intensity = 0.6f; sun.color = new Color(0.9f, 0.92f, 0.95f); sun.transform.rotation = Quaternion.Euler(50f, -30f, 0f); }
                    RenderSettings.ambientLight = new Color(0.55f, 0.56f, 0.58f);
                    RenderSettings.fog = true;
                    RenderSettings.fogMode = FogMode.Exponential;
                    RenderSettings.fogDensity = 0.035f;
                    break;
                default:
                    sky = new Color(0.55f, 0.72f, 0.92f);
                    if (sun) { sun.intensity = 1.15f; sun.color = new Color(1f, 0.96f, 0.88f); sun.transform.rotation = Quaternion.Euler(50f, -30f, 0f); }
                    RenderSettings.ambientLight = new Color(0.5f, 0.54f, 0.6f);
                    RenderSettings.fog = true;
                    RenderSettings.fogMode = FogMode.Linear;
                    RenderSettings.fogStartDistance = 200f;
                    RenderSettings.fogEndDistance = 500f;
                    break;
            }
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.fogColor = sky;
            if (cameras != null)
                foreach (var c in cameras) if (c) c.backgroundColor = sky;
        }
    }
}
