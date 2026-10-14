// SkyEye - One-click Windows build. Menu: SkyEye > 2. Build Windows Executable
// Batch: Unity.exe -batchmode -quit -projectPath <p> -executeMethod SkyEye.EditorTools.SkyEyeBuildTools.BuildWindowsBatch -logFile build.log
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SkyEye.EditorTools
{
    public static class SkyEyeBuildTools
    {
        public const string OutputFolder = "Builds/Windows";
        public const string ExeName = "SkyEye.exe";

        [MenuItem("SkyEye/2. Build Windows Executable", false, 2)]
        public static void BuildFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            bool ok = BuildWindows();
            string exe = Path.Combine(Path.GetFullPath(OutputFolder), ExeName);
            EditorUtility.DisplayDialog("SkyEye build", ok ? "Build succeeded:\n" + exe + "\n\nSubmit the WHOLE folder, not only the .exe."
                                                            : "Build failed - see the Console.", "OK");
            if (ok) EditorUtility.RevealInFinder(exe);
        }

        public static void BuildWindowsBatch() { EditorApplication.Exit(BuildWindows() ? 0 : 1); }

        public static bool BuildWindows()
        {
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(SkyEyeSceneBuilder.ScenePath) == null) SkyEyeSceneBuilder.Generate();
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(SkyEyeSceneBuilder.ScenePath, true) };
            PlayerSettings.companyName = "SIT Pune CA3";
            PlayerSettings.productName = "SkyEye";
            PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
            PlayerSettings.defaultScreenWidth = 1600;
            PlayerSettings.defaultScreenHeight = 900;
            PlayerSettings.resizableWindow = true;
            PlayerSettings.runInBackground = true;
            Directory.CreateDirectory(OutputFolder);
            var options = new BuildPlayerOptions
            {
                scenes = new[] { SkyEyeSceneBuilder.ScenePath },
                locationPathName = Path.Combine(OutputFolder, ExeName),
                target = BuildTarget.StandaloneWindows64,
                targetGroup = BuildTargetGroup.Standalone,
                options = BuildOptions.None
            };
            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result == BuildResult.Succeeded)
            {
                Debug.Log("[SkyEye] Build succeeded: " + report.summary.outputPath);
                return true;
            }
            Debug.LogError("[SkyEye] Build " + report.summary.result + " with " + report.summary.totalErrors + " error(s).");
            return false;
        }
    }
}
