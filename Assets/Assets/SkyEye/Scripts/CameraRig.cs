// SkyEye - Chase, first-person (AR) and top-map camera views.
using UnityEngine;

namespace SkyEye
{
    public enum ViewMode { Chase, FirstPersonAR, TopMap }

    /// <summary>
    /// Chase: perspective camera orbiting behind the drone (drag to rotate, scroll to zoom).
    /// First person (AR): camera on the drone's nose; the AR overlay labels the world.
    /// Top map: orthographic (parallel) projection of the whole flood zone with the flight trails.
    /// Mouse input is read from IMGUI events so no input-system setup is needed.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public class CameraRig : MonoBehaviour
    {
        public DroneSimulation simulation;
        public SkyEyeHUD hud;
        public ViewMode Mode { get; private set; }

        public float yaw = 20f, pitch = 25f, distance = 14f;
        public Vector3 mapCentre = new Vector3(0f, 0f, 5f);
        public float mapSize = 85f;

        Camera cam;
        bool dragging;

        void Awake() { cam = GetComponent<Camera>(); }

        public void SetMode(ViewMode m)
        {
            Mode = m;
            cam.orthographic = m == ViewMode.TopMap;
            if (m == ViewMode.TopMap) cam.orthographicSize = mapSize;
            cam.fieldOfView = m == ViewMode.FirstPersonAR ? 70f : 55f;
        }

        void LateUpdate()
        {
            if (simulation == null) return;
            Vector3 drone = simulation.TruePosition;
            float k = 1f - Mathf.Exp(-8f * Time.unscaledDeltaTime);
            Vector3 targetPos;
            Quaternion targetRot;
            switch (Mode)
            {
                case ViewMode.FirstPersonAR:
                {
                    Transform d = simulation.droneRoot;
                    float droneYaw = d != null ? d.eulerAngles.y : 0f;
                    Quaternion r = Quaternion.Euler(15f, droneYaw, 0f);
                    targetPos = drone + Quaternion.Euler(0f, droneYaw, 0f) * new Vector3(0f, -0.1f, 0.8f);
                    targetRot = r;
                    k = 1f; // rigidly attached
                    break;
                }
                case ViewMode.TopMap:
                    targetPos = mapCentre + Vector3.up * 200f;
                    targetRot = Quaternion.Euler(90f, 0f, 0f);
                    cam.orthographicSize = Mathf.Lerp(cam.orthographicSize, mapSize, k);
                    break;
                default:
                {
                    Quaternion orbit = Quaternion.Euler(pitch, yaw, 0f);
                    targetPos = drone + orbit * new Vector3(0f, 0f, -distance);
                    targetRot = Quaternion.LookRotation(drone - targetPos, Vector3.up);
                    break;
                }
            }
            transform.position = Vector3.Lerp(transform.position, targetPos, k);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, k);
        }

        void OnGUI()
        {
            Event e = Event.current;
            bool overUi = hud != null && hud.IsOverUi(e.mousePosition);
            switch (e.type)
            {
                case EventType.MouseDown:
                    dragging = !overUi && (e.button == 0 || e.button == 1);
                    break;
                case EventType.MouseUp:
                    dragging = false;
                    break;
                case EventType.MouseDrag:
                    if (dragging && Mode == ViewMode.Chase)
                    {
                        yaw += e.delta.x * 0.3f;
                        pitch = Mathf.Clamp(pitch + e.delta.y * 0.2f, -5f, 85f);
                        e.Use();
                    }
                    break;
                case EventType.ScrollWheel:
                    if (!overUi)
                    {
                        if (Mode == ViewMode.TopMap) mapSize = Mathf.Clamp(mapSize + e.delta.y * 3f, 20f, 160f);
                        else distance = Mathf.Clamp(distance + e.delta.y * 0.8f, 4f, 60f);
                        e.Use();
                    }
                    break;
            }
        }
    }
}
