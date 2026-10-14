// SkyEye - Setup checks. Menu: SkyEye > 3. Validate Setup
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SkyEye.EditorTools
{
    public static class SkyEyeValidator
    {
        [MenuItem("SkyEye/3. Validate Setup", false, 3)]
        public static void ValidateFromMenu() { Validate(true); }

        public static bool Validate(bool dialog)
        {
            var sb = new StringBuilder();
            int fails = 0;
            System.Action<string, bool> check = (name, ok) =>
            {
                sb.AppendLine((ok ? "[PASS] " : "[FAIL] ") + name);
                if (!ok) fails++;
            };

            check("Scene file exists", AssetDatabase.LoadAssetAtPath<SceneAsset>(SkyEyeSceneBuilder.ScenePath) != null);
            bool inBuild = false;
            foreach (var s in EditorBuildSettings.scenes) if (s.enabled && s.path == SkyEyeSceneBuilder.ScenePath) inBuild = true;
            check("Scene in build list", inBuild);

            var active = EditorSceneManager.GetActiveScene();
            if (active.path == SkyEyeSceneBuilder.ScenePath)
            {
                var sims = Object.FindObjectsByType<DroneSimulation>(FindObjectsSortMode.None);
                check("One DroneSimulation", sims.Length == 1);
                if (sims.Length == 1)
                {
                    var s = sims[0];
                    check("Drone, vision, helipad and survivor assigned", s.droneRoot != null && s.vision != null && s.helipad != null && s.survivor != null);
                    check("Vision camera assigned", s.vision != null && s.vision.downCamera != null);
                    check("Drone has no colliders (rangefinder rays must pass it)", s.droneRoot != null && s.droneRoot.GetComponentsInChildren<Collider>().Length == 0);
                }
                check("Main camera with CameraRig", Camera.main != null && Camera.main.GetComponent<CameraRig>() != null);
                int missing = 0;
                foreach (var root in active.GetRootGameObjects())
                    foreach (var t in root.GetComponentsInChildren<Transform>(true))
                        missing += GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject);
                check("No missing scripts", missing == 0);
            }
            else sb.AppendLine("[INFO] Open " + SkyEyeSceneBuilder.ScenePath + " for scene-level checks.");

            string summary = fails == 0 ? "All SkyEye checks passed." : fails + " check(s) failed.";
            if (fails == 0) Debug.Log("[SkyEye] " + summary + "\n" + sb); else Debug.LogError("[SkyEye] " + summary + "\n" + sb);
            if (dialog) EditorUtility.DisplayDialog("SkyEye validation", summary + "\n\n" + sb, "OK");
            return fails == 0;
        }
    }
}
