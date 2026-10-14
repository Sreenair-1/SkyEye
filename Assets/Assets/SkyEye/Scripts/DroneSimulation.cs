// SkyEye - Main simulation: drone dynamics, simulated sensors, sensor fusion and the rescue mission.
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using SkyEye.Core;
using UnityEngine;

namespace SkyEye
{
    public enum RunState { Ready, Running, Paused }
    public enum FusionMode { GpsOnly, GpsImu, GpsImuFlow }
    public enum MissionState { OnPad, Takeoff, Search, ApproachSurvivor, DropSupplies, ReturnHome, PrecisionLanding, Landed, TimedOut }
    public enum ExperimentStage { None, Running, Done }

    /// <summary>Results of one mission, used by the fusion comparison experiment.</summary>
    public sealed class MissionResult
    {
        public FusionMode Mode;
        public float MissionTime;
        public float RmsEstimateError;      // horizontal |estimate - truth|, RMS over the flight
        public float MaxEstimateError;
        public float RmsGpsError;           // raw GPS horizontal error, for reference
        public float LandingError;          // distance from helipad centre at touchdown
        public float SurvivorError;         // estimated vs true survivor position
        public bool SurvivorFound;
        public bool Completed;
    }

    /// <summary>
    /// Runs the whole simulation in fixed 0.02 s steps:
    ///  1. wind and drone dynamics (ground truth),
    ///  2. simulated sensors: IMU (accelerometer + gyroscope), GPS, barometer, laser rangefinder,
    ///  3. sensor fusion (Kalman filter per axis; complementary filter for tilt),
    ///  4. the mission state machine, which flies using the ESTIMATED state only,
    ///  5. metrics, trails and CSV logging.
    /// Vision results (optical flow, marker pose, survivor detection) arrive from VisionSystem.
    /// </summary>
    public class DroneSimulation : MonoBehaviour
    {
        [Header("Scene references")]
        public Transform droneRoot;
        public DroneVisual droneVisual;
        public VisionSystem vision;
        public Transform helipad;            // centre of the landing marker
        public Transform survivor;           // ground truth (only used for scoring)
        public Material trueTrailMaterial, estimateTrailMaterial, gpsTrailMaterial, supplyMaterial;

        [Header("Mission")]
        public float cruiseAltitude = 25f;
        public float cruiseSpeed = 6f;
        public float survivorHoverAltitude = 15f;
        public float searchMinX = -45f, searchMaxX = 45f;
        public float[] searchLanesZ = { 0f, 20f, 40f, 60f };
        public int markerId = 0x6A3D;
        public float markerSize = 4f;        // side of the black square (m)
        public bool precisionLanding = true;
        public float missionTimeout = 420f;

        [Header("Sensors and fusion")]
        public FusionMode fusionMode = FusionMode.GpsImuFlow;
        [Range(0f, 3f)] public float windStrength = 0.8f;
        public float gpsSigma = 1.0f, gpsBiasSigma = 1.2f, gpsVerticalSigma = 2.0f;
        public float accelSigma = 0.15f, baroSigma = 0.15f, flowSigma = 0.2f;
        public int seed = 2026;

        [Header("Geo reference (approx. Sangli, Maharashtra - 2019 flood region)")]
        public double referenceLatitude = 16.8524;
        public double referenceLongitude = 74.5815;

        [Header("Playback")]
        [Range(1f, 4f)] public float timeScale = 1f;

        public const float FixedDt = 0.02f;
        const float Gravity = 9.81f;

        // ---------------------------------------------------------------- public state (read by UI)
        public RunState State { get; private set; }
        public MissionState Mission { get; private set; }
        public float SimTime { get; private set; }
        public float MissionStartTime { get; private set; }
        public Vector3 TruePosition { get; private set; }
        public Vector3 TrueVelocity { get; private set; }
        public Vector3 TrueAcceleration { get; private set; }
        public Vector3 EstimatedPosition { get; private set; }
        public Vector3 EstimatedVelocity { get; private set; }
        public Vector3 LastGps { get; private set; }
        public float RangefinderHeight { get; private set; }
        public float BaroAltitude { get; private set; }
        public GeoReference Geo { get; private set; }
        public string LastEvent { get; private set; }
        public float HorizontalError { get { return Horizontal(EstimatedPosition - TruePosition).magnitude; } }
        public float GpsError { get { return Horizontal(LastGps - TruePosition).magnitude; } }
        public ErrorStats EstimateStats { get; private set; }
        public ErrorStats GpsStats { get; private set; }

        // Attitude (pitch) estimation demo
        public float PitchTrue { get; private set; }
        public float PitchGyroOnly { get; private set; }
        public float PitchAccelOnly { get; private set; }
        public float PitchFused { get; private set; }

        // Vision-derived state
        public bool FlowActive { get; private set; }
        public Vector2 LastFlowVelocity { get; private set; }
        public float LastFlowTime { get; private set; }
        public bool MarkerVisible { get { return SimTime - lastMarkerTime < 0.5f; } }
        public Vector3 MarkerOffset { get; private set; } // (east, height, north) from vision
        public bool SurvivorConfirmed { get; private set; }
        public Vector3 SurvivorEstimate { get; private set; }
        public int SurvivorSightings { get; private set; }
        public float LandingError { get; private set; }

        // Experiment
        public ExperimentStage Experiment { get; private set; }
        public readonly List<MissionResult> Results = new List<MissionResult>();
        public FusionMode ExperimentCurrentMode { get; private set; }

        public string ConfigError { get; private set; }
        public string LastExportPath { get; private set; }

        // ---------------------------------------------------------------- private
        KalmanPV kfE, kfU, kfN;
        ComplementaryFilter pitchFilter;
        NoiseSource noise;
        RandomWalkBias gpsBiasE, gpsBiasN, gpsBiasU, baroBias;
        Vector3 accelBias;
        float gyroBias;
        Vector3 gpsVelLowPass;
        Vector3 prevGps;
        float gpsTimer, baroTimer, logTimer, trailTimer;
        float accumulator;
        float lastMarkerTime = -100f;
        Vector3 landingTarget;           // in the ESTIMATE frame
        bool landingTargetSet;
        float stateTimer;
        int waypointIndex;
        readonly List<Vector3> searchWaypoints = new List<Vector3>();
        Vector3 survivorSum;
        float prevPitchTrue;
        float windSeedX, windSeedZ;
        LineRenderer trueTrail, estTrail, gpsTrail;
        readonly List<Vector3> truePts = new List<Vector3>(), estPts = new List<Vector3>(), gpsPts = new List<Vector3>();
        readonly List<string> logRows = new List<string>();
        readonly List<GameObject> drops = new List<GameObject>();
        readonly Queue<FusionMode> experimentQueue = new Queue<FusionMode>();
        Vector3 homePosition;

        public bool ShowTrails { get; private set; }

        // ================================================================ lifecycle

        void Awake()
        {
            EstimateStats = new ErrorStats();
            GpsStats = new ErrorStats();
            Geo = new GeoReference(referenceLatitude, referenceLongitude);
            ShowTrails = true;
        }

        void Start()
        {
            Application.targetFrameRate = 60;
            if (droneRoot == null || vision == null || helipad == null)
                ConfigError = "Scene is not set up. Run menu SkyEye > 1. Generate Scene.";
            trueTrail = MakeTrail("Trail_True", trueTrailMaterial, 0.25f);
            estTrail = MakeTrail("Trail_Estimate", estimateTrailMaterial, 0.2f);
            gpsTrail = MakeTrail("Trail_GPS", gpsTrailMaterial, 0.12f);
            ResetMission();
        }

        LineRenderer MakeTrail(string name, Material m, float width)
        {
            var go = new GameObject(name);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.widthMultiplier = width;
            lr.sharedMaterial = m;
            lr.positionCount = 0;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            return lr;
        }

        public void SetShowTrails(bool show)
        {
            ShowTrails = show;
            foreach (var lr in new[] { trueTrail, estTrail, gpsTrail }) if (lr) lr.enabled = show;
        }

        // ================================================================ controls

        public void StartOrResume() { if (ConfigError == null) State = RunState.Running; }
        public void Pause() { if (State == RunState.Running) State = RunState.Paused; }
        public void TogglePause() { if (State == RunState.Running) Pause(); else StartOrResume(); }

        public void ResetAll()
        {
            Experiment = ExperimentStage.None;
            experimentQueue.Clear();
            ResetMission();
            State = RunState.Ready;
        }

        public void SetFusionMode(FusionMode m)
        {
            fusionMode = m;
        }

        /// <summary>Flies the same mission (same seed, wind and noise) with each fusion mode.</summary>
        public void StartExperiment()
        {
            if (ConfigError != null) return;
            Results.Clear();
            experimentQueue.Clear();
            experimentQueue.Enqueue(FusionMode.GpsOnly);
            experimentQueue.Enqueue(FusionMode.GpsImu);
            experimentQueue.Enqueue(FusionMode.GpsImuFlow);
            Experiment = ExperimentStage.Running;
            NextExperimentRun();
        }

        void NextExperimentRun()
        {
            ExperimentCurrentMode = experimentQueue.Dequeue();
            fusionMode = ExperimentCurrentMode;
            ResetMission();
            State = RunState.Running;
        }

        public bool ExperimentRunning { get { return Experiment == ExperimentStage.Running; } }

        void ResetMission()
        {
            noise = new NoiseSource(seed);
            gpsBiasE = new RandomWalkBias(gpsBiasSigma, 20f);
            gpsBiasN = new RandomWalkBias(gpsBiasSigma, 20f);
            gpsBiasU = new RandomWalkBias(gpsVerticalSigma, 20f);
            baroBias = new RandomWalkBias(0.3f, 60f);
            accelBias = new Vector3(noise.Gaussian(0.05f), noise.Gaussian(0.05f), noise.Gaussian(0.05f));
            gyroBias = 0.8f; // deg/s - a typical MEMS gyro bias, so gyro-only tilt drifts visibly
            windSeedX = noise.Uniform() * 100f;
            windSeedZ = noise.Uniform() * 100f + 50f;

            homePosition = helipad != null ? helipad.position : Vector3.zero;
            TruePosition = homePosition + Vector3.up * 0.3f;
            TrueVelocity = Vector3.zero;
            TrueAcceleration = Vector3.zero;

            kfE = new KalmanPV(0.5f); kfU = new KalmanPV(0.5f); kfN = new KalmanPV(0.5f);
            pitchFilter = new ComplementaryFilter(0.98f);
            PitchTrue = PitchGyroOnly = PitchAccelOnly = PitchFused = prevPitchTrue = 0f;

            LastGps = prevGps = TruePosition;
            gpsVelLowPass = Vector3.zero;
            EstimatedPosition = TruePosition;
            EstimatedVelocity = Vector3.zero;
            kfE.Init(TruePosition.x, 0f, 4f, 0.25f);
            kfU.Init(TruePosition.y, 0f, 1f, 0.25f);
            kfN.Init(TruePosition.z, 0f, 4f, 0.25f);

            SimTime = 0f; MissionStartTime = 0f;
            gpsTimer = baroTimer = logTimer = trailTimer = accumulator = stateTimer = 0f;
            lastMarkerTime = -100f;
            landingTargetSet = false;
            FlowActive = false;
            LastFlowVelocity = Vector2.zero;
            LastFlowTime = -100f;
            SurvivorConfirmed = false;
            SurvivorSightings = 0;
            survivorSum = Vector3.zero;
            SurvivorEstimate = Vector3.zero;
            LandingError = -1f;
            EstimateStats.Reset();
            GpsStats.Reset();
            Mission = MissionState.OnPad;
            LastEvent = "On the helipad. Press Start.";
            waypointIndex = 0;
            BuildSearchPattern();
            truePts.Clear(); estPts.Clear(); gpsPts.Clear();
            UpdateTrails();
            logRows.Clear();
            foreach (var d in drops) if (d) Destroy(d);
            drops.Clear();
            RangefinderHeight = 0.3f;
            BaroAltitude = TruePosition.y;
            if (vision != null) vision.ResetVision();
            ApplyDroneTransform(1f);
        }

        void BuildSearchPattern()
        {
            searchWaypoints.Clear();
            for (int i = 0; i < searchLanesZ.Length; i++)
            {
                bool leftToRight = i % 2 == 0;
                searchWaypoints.Add(new Vector3(leftToRight ? searchMinX : searchMaxX, cruiseAltitude, searchLanesZ[i]));
                searchWaypoints.Add(new Vector3(leftToRight ? searchMaxX : searchMinX, cruiseAltitude, searchLanesZ[i]));
            }
        }

        // ================================================================ main loop

        void Update()
        {
            if (vision != null) vision.ProcessFrame(State == RunState.Running);

            if (State == RunState.Running)
            {
                accumulator += Mathf.Min(Time.deltaTime, 0.1f) * timeScale;
                int steps = 0;
                while (accumulator >= FixedDt && steps < 400 && State == RunState.Running)
                {
                    accumulator -= FixedDt;
                    Step(FixedDt);
                    steps++;
                }
            }
            ApplyDroneTransform(Time.deltaTime);
        }

        void Step(float dt)
        {
            SimTime += dt;

            // ---- 1. Mission logic produces a velocity command from the ESTIMATED state
            Vector3 desiredVel = MissionLogic(dt);

            // ---- 2. Velocity controller -> acceleration command (limited like a real multirotor)
            Vector3 acc = (desiredVel - EstimatedVelocity) * 2.0f;
            Vector3 accH = Horizontal(acc);
            if (accH.magnitude > 4f) accH = accH.normalized * 4f;
            acc = new Vector3(accH.x, Mathf.Clamp(acc.y, -3f, 3f), accH.z);
            bool grounded = Mission == MissionState.OnPad || Mission == MissionState.Landed || Mission == MissionState.TimedOut;

            // ---- 3. Ground-truth dynamics with wind gusts and drag
            Vector3 wind = grounded ? Vector3.zero : new Vector3(
                (Mathf.PerlinNoise(windSeedX, SimTime * 0.15f) - 0.5f) * 2f * windStrength, 0f,
                (Mathf.PerlinNoise(windSeedZ, SimTime * 0.15f) - 0.5f) * 2f * windStrength);
            Vector3 trueAcc = grounded ? Vector3.zero : acc + wind - 0.15f * TrueVelocity;
            Vector3 v = grounded ? Vector3.zero : TrueVelocity + trueAcc * dt;
            Vector3 p = TruePosition + v * dt;
            float surface = SurfaceHeightBelow(p);
            if (p.y < surface + 0.3f) { p.y = surface + 0.3f; if (v.y < 0f) v.y = 0f; }
            TrueAcceleration = trueAcc;
            TrueVelocity = v;
            TruePosition = p;

            // ---- 4. Sensors
            RangefinderHeight = Mathf.Max(0f, TruePosition.y - SurfaceHeightBelow(TruePosition)) + noise.Gaussian(0.03f);
            Vector3 imuAcc = TrueAcceleration + accelBias + new Vector3(noise.Gaussian(accelSigma), noise.Gaussian(accelSigma), noise.Gaussian(accelSigma));

            // Attitude demo: pitch follows the northward acceleration (a multirotor tilts to accelerate)
            PitchTrue = Mathf.Atan2(TrueAcceleration.z, Gravity) * Mathf.Rad2Deg;
            float gyroRate = (PitchTrue - prevPitchTrue) / dt + gyroBias + noise.Gaussian(0.3f);
            prevPitchTrue = PitchTrue;
            PitchGyroOnly += gyroRate * dt;
            PitchAccelOnly = PitchTrue + noise.Gaussian(4f);  // vibration makes accelerometer tilt noisy
            PitchFused = pitchFilter.Update(gyroRate, PitchAccelOnly, dt);

            // ---- 5. Fusion: predict with the IMU every step
            kfE.Predict(dt, imuAcc.x);
            kfU.Predict(dt, imuAcc.y);
            kfN.Predict(dt, imuAcc.z);

            baroTimer += dt;
            if (baroTimer >= 0.1f)
            {
                baroTimer -= 0.1f;
                BaroAltitude = TruePosition.y + baroBias.Step(0.1f, noise) + noise.Gaussian(baroSigma);
                if (fusionMode != FusionMode.GpsOnly) kfU.UpdatePosition(BaroAltitude, baroSigma * baroSigma + 0.09f);
            }

            gpsTimer += dt;
            if (gpsTimer >= 0.2f)
            {
                gpsTimer -= 0.2f;
                Vector3 gps = TruePosition + new Vector3(
                    gpsBiasE.Step(0.2f, noise) + noise.Gaussian(gpsSigma),
                    gpsBiasU.Step(0.2f, noise) + noise.Gaussian(gpsVerticalSigma),
                    gpsBiasN.Step(0.2f, noise) + noise.Gaussian(gpsSigma));
                prevGps = LastGps;
                LastGps = gps;
                GpsStats.Add(GpsError);
                float rH = gpsSigma * gpsSigma + gpsBiasSigma * gpsBiasSigma;
                kfE.UpdatePosition(gps.x, rH);
                kfN.UpdatePosition(gps.z, rH);
                if (fusionMode == FusionMode.GpsOnly) kfU.UpdatePosition(gps.y, gpsVerticalSigma * gpsVerticalSigma * 2f);
                // GPS-only velocity: smoothed difference of consecutive fixes
                gpsVelLowPass = Vector3.Lerp(gpsVelLowPass, (LastGps - prevGps) / 0.2f, 0.35f);
            }

            FlowActive = fusionMode == FusionMode.GpsImuFlow && SimTime - LastFlowTime < 0.3f;

            if (fusionMode == FusionMode.GpsOnly)
            {
                // No filtering at all: the drone flies on raw GPS fixes.
                EstimatedPosition = LastGps;
                EstimatedVelocity = gpsVelLowPass;
            }
            else
            {
                EstimatedPosition = new Vector3(kfE.Position, kfU.Position, kfN.Position);
                EstimatedVelocity = new Vector3(kfE.Velocity, kfU.Velocity, kfN.Velocity);
            }

            if (!grounded && Mission != MissionState.Takeoff) EstimateStats.Add(HorizontalError);

            // ---- 6. Trails and logging
            trailTimer += dt;
            if (trailTimer >= 0.25f)
            {
                trailTimer -= 0.25f;
                AddPoint(truePts, TruePosition);
                AddPoint(estPts, EstimatedPosition);
                AddPoint(gpsPts, LastGps);
                UpdateTrails();
            }
            logTimer += dt;
            if (logTimer >= 0.5f) { logTimer -= 0.5f; LogRow(); }

            // ---- 7. Experiment sequencing
            if (Experiment == ExperimentStage.Running &&
                (Mission == MissionState.Landed || Mission == MissionState.TimedOut))
            {
                Results.Add(MakeResult());
                if (experimentQueue.Count > 0) NextExperimentRun();
                else { Experiment = ExperimentStage.Done; State = RunState.Paused; }
            }
        }

        // ================================================================ vision callbacks

        /// <summary>Ground velocity measured by optical flow (m/s east, m/s north).</summary>
        public void OnFlowMeasurement(float velEast, float velNorth, float captureTime)
        {
            LastFlowVelocity = new Vector2(velEast, velNorth);
            LastFlowTime = SimTime;
            if (fusionMode != FusionMode.GpsImuFlow) return;
            float r = flowSigma * flowSigma;
            kfE.UpdateVelocity(velEast, r);
            kfN.UpdateVelocity(velNorth, r);
        }

        /// <summary>Marker pose from the downward camera: height above it and horizontal offset to it.</summary>
        public void OnMarker(float heightAbove, float east, float north)
        {
            lastMarkerTime = SimTime;
            MarkerOffset = new Vector3(east, heightAbove, north);
            if (Mission == MissionState.ReturnHome || Mission == MissionState.PrecisionLanding)
            {
                // Target in the estimate frame: wherever we believe we are + what the camera sees.
                landingTarget = new Vector3(EstimatedPosition.x + east, 0f, EstimatedPosition.z + north);
                landingTargetSet = true;
            }
        }

        /// <summary>Orange life-raft seen at a horizontal offset (m) from the drone.</summary>
        public void OnSurvivorSeen(float east, float north)
        {
            Vector3 observed = new Vector3(EstimatedPosition.x + east, 0f, EstimatedPosition.z + north);
            if (SurvivorConfirmed)
            {
                // Keep refining while hovering above: the view is then straight down, so it is most accurate.
                if (Mission == MissionState.ApproachSurvivor || Mission == MissionState.DropSupplies)
                    SurvivorEstimate = Vector3.Lerp(SurvivorEstimate, observed, 0.05f);
                return;
            }
            if (Mission != MissionState.Search) return;
            survivorSum += observed;
            SurvivorSightings++;
            if (SurvivorSightings >= 5)
            {
                SurvivorEstimate = survivorSum / SurvivorSightings;
                SurvivorConfirmed = true;
                double lat, lon;
                Geo.ToGeo(SurvivorEstimate.x, SurvivorEstimate.z, out lat, out lon);
                LastEvent = "Survivor detected at " + lat.ToString("0.00000", CultureInfo.InvariantCulture) + ", " +
                            lon.ToString("0.00000", CultureInfo.InvariantCulture);
                SetMission(MissionState.ApproachSurvivor);
            }
        }

        // ================================================================ mission

        void SetMission(MissionState s)
        {
            Mission = s;
            stateTimer = 0f;
        }

        Vector3 MissionLogic(float dt)
        {
            stateTimer += dt;
            Vector3 est = EstimatedPosition;
            if (SimTime - MissionStartTime > missionTimeout && Mission != MissionState.Landed && Mission != MissionState.OnPad)
            {
                SetMission(MissionState.TimedOut);
                LastEvent = "Mission timed out.";
            }

            switch (Mission)
            {
                case MissionState.OnPad:
                    if (State == RunState.Running)
                    {
                        MissionStartTime = SimTime;
                        SetMission(MissionState.Takeoff);
                        LastEvent = "Take-off.";
                    }
                    return Vector3.zero;

                case MissionState.Takeoff:
                    if (est.y > cruiseAltitude - 1f)
                    {
                        SetMission(MissionState.Search);
                        LastEvent = "Searching the flood zone (lawn-mower pattern).";
                    }
                    return GoTo(new Vector3(homePosition.x, cruiseAltitude, homePosition.z), 3f);

                case MissionState.Search:
                    if (waypointIndex >= searchWaypoints.Count)
                    {
                        LastEvent = "Search finished without a confirmed survivor. Returning home.";
                        SetMission(MissionState.ReturnHome);
                        return Vector3.zero;
                    }
                    Vector3 wp = searchWaypoints[waypointIndex];
                    if (Horizontal(wp - est).magnitude < 2.5f) waypointIndex++;
                    return GoTo(wp, cruiseSpeed);

                case MissionState.ApproachSurvivor:
                {
                    Vector3 target = new Vector3(SurvivorEstimate.x, survivorHoverAltitude + 5f, SurvivorEstimate.z);
                    if (Horizontal(target - est).magnitude < 1.5f && EstimatedVelocity.magnitude < 1.5f)
                    {
                        SetMission(MissionState.DropSupplies);
                        LastEvent = "Hovering over the survivor - dropping the relief kit.";
                    }
                    return GoTo(target, 4f);
                }

                case MissionState.DropSupplies:
                {
                    if (stateTimer > 3f)
                    {
                        SpawnSupplyDrop();
                        SetMission(MissionState.ReturnHome);
                        LastEvent = "Relief kit dropped. Returning to the helipad.";
                    }
                    return GoTo(new Vector3(SurvivorEstimate.x, survivorHoverAltitude + 5f, SurvivorEstimate.z), 2f);
                }

                case MissionState.ReturnHome:
                {
                    Vector3 target = new Vector3(homePosition.x, 20f, homePosition.z);
                    if (Horizontal(target - est).magnitude < 2f && EstimatedVelocity.magnitude < 1.5f)
                    {
                        SetMission(MissionState.PrecisionLanding);
                        LastEvent = precisionLanding ? "Over the helipad - searching for the landing marker." : "Over the helipad - GPS landing.";
                    }
                    return GoTo(target, cruiseSpeed);
                }

                case MissionState.PrecisionLanding:
                    return LandingLogic(est);

                default: // Landed, TimedOut
                    return Vector3.zero;
            }
        }

        Vector3 LandingLogic(Vector3 est)
        {
            float height = RangefinderHeight;
            Vector3 target;
            bool aligned;
            if (precisionLanding && landingTargetSet)
            {
                target = landingTarget;
                aligned = Horizontal(target - est).magnitude < 0.8f;
            }
            else
            {
                target = new Vector3(homePosition.x, 0f, homePosition.z);
                aligned = Horizontal(target - est).magnitude < 1.5f;
            }

            Vector3 vel = Horizontal(target - est) * 0.8f;
            if (vel.magnitude > 2f) vel = vel.normalized * 2f;

            if (height < 3f) vel.y = -0.6f;                       // final blind descent (marker too close to see)
            else if (aligned) vel.y = -1.0f;
            else vel.y = 0f;                                       // hold altitude until centred
            if (precisionLanding && !MarkerVisible && height > 5f && stateTimer > 3f && !landingTargetSet)
                vel.y = 0f;                                        // keep looking for the marker

            if (height < 0.35f && TrueVelocity.y <= 0.05f)
            {
                SetMission(MissionState.Landed);
                Vector3 pad = helipad != null ? helipad.position : homePosition;
                LandingError = Horizontal(TruePosition - pad).magnitude;
                LastEvent = "Landed. Touchdown error " + LandingError.ToString("0.00") + " m from the marker centre.";
                State = Experiment == ExperimentStage.Running ? State : RunState.Paused;
            }
            return vel;
        }

        Vector3 GoTo(Vector3 target, float maxSpeed)
        {
            Vector3 d = target - EstimatedPosition;
            Vector3 h = Horizontal(d) * 0.8f;
            if (h.magnitude > maxSpeed) h = h.normalized * maxSpeed;
            float vy = Mathf.Clamp(d.y * 0.8f, -1.5f, 2.5f);
            return new Vector3(h.x, vy, h.z);
        }

        void SpawnSupplyDrop()
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = "ReliefKit";
            Object.Destroy(box.GetComponent<Collider>());
            box.transform.position = TruePosition + Vector3.down * 0.6f;
            box.transform.localScale = new Vector3(0.6f, 0.4f, 0.6f);
            if (supplyMaterial != null) box.GetComponent<Renderer>().sharedMaterial = supplyMaterial;
            var fall = box.AddComponent<SupplyDrop>();
            fall.groundY = SurfaceHeightBelow(box.transform.position) + 0.2f;
            drops.Add(box);
        }

        MissionResult MakeResult()
        {
            float survErr = -1f;
            if (SurvivorConfirmed && survivor != null)
                survErr = Horizontal(SurvivorEstimate - survivor.position).magnitude;
            return new MissionResult
            {
                Mode = fusionMode,
                MissionTime = SimTime - MissionStartTime,
                RmsEstimateError = EstimateStats.Rms,
                MaxEstimateError = EstimateStats.Max,
                RmsGpsError = GpsStats.Rms,
                LandingError = LandingError,
                SurvivorError = survErr,
                SurvivorFound = SurvivorConfirmed,
                Completed = Mission == MissionState.Landed
            };
        }

        // ================================================================ helpers

        public static Vector3 Horizontal(Vector3 v) { return new Vector3(v.x, 0f, v.z); }

        /// <summary>Height of whatever is directly below (roofs, helipad, ground) using a downward ray.</summary>
        public float SurfaceHeightBelow(Vector3 p)
        {
            RaycastHit hit;
            if (Physics.Raycast(new Vector3(p.x, 200f, p.z), Vector3.down, out hit, 400f)) return hit.point.y;
            return 0f;
        }

        void ApplyDroneTransform(float dt)
        {
            if (droneRoot == null) return;
            droneRoot.position = TruePosition;
            Vector3 hv = Horizontal(TrueVelocity);
            float yaw = droneRoot.eulerAngles.y;
            if (hv.magnitude > 0.5f) yaw = Mathf.LerpAngle(yaw, Mathf.Atan2(hv.x, hv.z) * Mathf.Rad2Deg, Mathf.Clamp01(dt * 2f));
            float tiltX = Mathf.Atan2(TrueAcceleration.z, Gravity) * Mathf.Rad2Deg;
            float tiltZ = -Mathf.Atan2(TrueAcceleration.x, Gravity) * Mathf.Rad2Deg;
            droneRoot.rotation = Quaternion.Euler(tiltX, 0f, tiltZ) * Quaternion.Euler(0f, yaw, 0f);
            if (droneVisual != null)
            {
                bool flying = Mission != MissionState.OnPad && Mission != MissionState.Landed && Mission != MissionState.TimedOut;
                droneVisual.SetSpin(flying ? 1f : 0f);
            }
        }

        void AddPoint(List<Vector3> pts, Vector3 p)
        {
            pts.Add(p);
            if (pts.Count > 800) pts.RemoveAt(0);
        }

        void UpdateTrails()
        {
            SetLine(trueTrail, truePts);
            SetLine(estTrail, estPts);
            SetLine(gpsTrail, gpsPts);
        }

        static void SetLine(LineRenderer lr, List<Vector3> pts)
        {
            if (lr == null) return;
            lr.positionCount = pts.Count;
            if (pts.Count > 0) lr.SetPositions(pts.ToArray());
        }

        // ================================================================ data export

        const string CsvHeader = "time_s,mission,fusion,true_x,true_y,true_z,est_x,est_y,est_z,gps_x,gps_y,gps_z,est_error_m,gps_error_m,rangefinder_m,flow_active,flow_ve,flow_vn,marker_visible,survivor_confirmed,pitch_true,pitch_gyro,pitch_accel,pitch_fused";

        void LogRow()
        {
            var c = CultureInfo.InvariantCulture;
            logRows.Add(string.Join(",", new[]
            {
                SimTime.ToString("0.0", c), Mission.ToString(), fusionMode.ToString(),
                TruePosition.x.ToString("0.00", c), TruePosition.y.ToString("0.00", c), TruePosition.z.ToString("0.00", c),
                EstimatedPosition.x.ToString("0.00", c), EstimatedPosition.y.ToString("0.00", c), EstimatedPosition.z.ToString("0.00", c),
                LastGps.x.ToString("0.00", c), LastGps.y.ToString("0.00", c), LastGps.z.ToString("0.00", c),
                HorizontalError.ToString("0.000", c), GpsError.ToString("0.000", c), RangefinderHeight.ToString("0.00", c),
                FlowActive ? "1" : "0", LastFlowVelocity.x.ToString("0.00", c), LastFlowVelocity.y.ToString("0.00", c),
                MarkerVisible ? "1" : "0", SurvivorConfirmed ? "1" : "0",
                PitchTrue.ToString("0.00", c), PitchGyroOnly.ToString("0.00", c), PitchAccelOnly.ToString("0.00", c), PitchFused.ToString("0.00", c)
            }));
            if (logRows.Count > 20000) logRows.RemoveAt(0);
        }

        /// <summary>Writes the flight log and experiment results as CSV. Returns the folder or null.</summary>
        public string ExportCsv()
        {
            string[] dirs =
            {
                Path.Combine(Path.GetDirectoryName(Application.dataPath) ?? ".", "SkyEye_Export"),
                Path.Combine(Application.persistentDataPath, "SkyEye_Export")
            };
            foreach (var dir in dirs)
            {
                try
                {
                    Directory.CreateDirectory(dir);
                    string stamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
                    var sb = new StringBuilder();
                    sb.AppendLine(CsvHeader);
                    foreach (var r in logRows) sb.AppendLine(r);
                    File.WriteAllText(Path.Combine(dir, "flightlog_" + stamp + ".csv"), sb.ToString());
                    if (Results.Count > 0)
                    {
                        var c = CultureInfo.InvariantCulture;
                        var rb = new StringBuilder();
                        rb.AppendLine("fusion_mode,completed,mission_time_s,rms_estimate_error_m,max_estimate_error_m,rms_gps_error_m,landing_error_m,survivor_found,survivor_error_m,seed");
                        foreach (var r in Results)
                            rb.AppendLine(string.Join(",", new[]
                            {
                                r.Mode.ToString(), r.Completed ? "1" : "0", r.MissionTime.ToString("0.0", c),
                                r.RmsEstimateError.ToString("0.000", c), r.MaxEstimateError.ToString("0.000", c), r.RmsGpsError.ToString("0.000", c),
                                r.LandingError.ToString("0.000", c), r.SurvivorFound ? "1" : "0", r.SurvivorError.ToString("0.000", c), seed.ToString(c)
                            }));
                        File.WriteAllText(Path.Combine(dir, "fusion_experiment_" + stamp + ".csv"), rb.ToString());
                    }
                    LastExportPath = dir;
                    Debug.Log("[SkyEye] CSV exported to " + dir);
                    return dir;
                }
                catch (System.Exception ex)
                {
                    Debug.LogWarning("[SkyEye] Could not write to " + dir + ": " + ex.Message);
                }
            }
            return null;
        }
    }
}
