// SkyEye - Location-based AR overlay with occlusion handling (Units 5 and 6).
using SkyEye.Core;
using UnityEngine;

namespace SkyEye
{
    [System.Serializable]
    public class PointOfInterest
    {
        public string name;
        public double latitude;
        public double longitude;
        public float height;     // label height above sea level (m)
        public Color colour = Color.white;
    }

    /// <summary>
    /// Like Nokia City Lens or Pokémon GO, labels are GPS-tagged (latitude/longitude) and are
    /// converted to local metres, projected through the camera and drawn on screen with their
    /// distance and compass bearing from the drone's FUSED position.
    /// Occlusion handling: a ray from the camera to each label tests whether a building is in
    /// the way; hidden labels are dimmed instead of floating unrealistically over the building.
    /// </summary>
    public class ArOverlay : MonoBehaviour
    {
        public DroneSimulation simulation;
        public Camera viewCamera;
        public PointOfInterest[] points;
        public bool showLabels = true;
        public bool occlusionHandling = true;

        GUIStyle box, small;

        void OnGUI()
        {
            if (!showLabels || simulation == null || viewCamera == null || points == null) return;
            if (box == null)
            {
                box = new GUIStyle(GUI.skin.box) { fontSize = 13, alignment = TextAnchor.MiddleCenter, richText = true };
                small = new GUIStyle(GUI.skin.label) { fontSize = 11, alignment = TextAnchor.UpperCenter };
            }
            foreach (var p in points) DrawPoi(p.name, ToWorld(p), p.colour);
            if (simulation.SurvivorConfirmed)
            {
                var s = simulation.SurvivorEstimate;
                DrawPoi("SURVIVOR (detected)", new Vector3(s.x, simulation.SurvivorEstimate.y + 8f, s.z), new Color(1f, 0.55f, 0.1f));
            }
        }

        Vector3 ToWorld(PointOfInterest p)
        {
            float e, n;
            simulation.Geo.ToLocal(p.latitude, p.longitude, out e, out n);
            return new Vector3(e, p.height, n);
        }

        void DrawPoi(string name, Vector3 world, Color colour)
        {
            Vector3 sp = viewCamera.WorldToScreenPoint(world);
            if (sp.z <= 0.5f) return; // behind the camera
            Vector3 camPos = viewCamera.transform.position;
            float dist = Vector3.Distance(camPos, world);
            bool hidden = false;
            if (occlusionHandling)
            {
                RaycastHit hit;
                hidden = Physics.Linecast(camPos, world, out hit) && hit.distance < dist - 1f;
            }
            Vector3 rel = world - simulation.EstimatedPosition;
            float horiz = new Vector2(rel.x, rel.z).magnitude;
            float bearing = GeoReference.BearingDeg(rel.x, rel.z);
            string hex = ColorUtility.ToHtmlStringRGB(colour);
            string text = "<b><color=#" + hex + ">" + name + "</color></b>\n" + horiz.ToString("0") + " m  ·  " + bearing.ToString("000") + "°";

            float w = 170f, h = 40f;
            float x = sp.x - w * 0.5f, y = Screen.height - sp.y - h - 12f;
            Color old = GUI.color;
            GUI.color = hidden ? new Color(1f, 1f, 1f, 0.35f) : Color.white;
            GUI.Box(new Rect(x, y, w, h), text, box);
            GUI.Box(new Rect(sp.x - 3f, Screen.height - sp.y - 3f, 6f, 6f), GUIContent.none);
            if (hidden) GUI.Label(new Rect(x, y + h, w, 18f), "(behind a building)", small);
            GUI.color = old;
        }
    }
}
