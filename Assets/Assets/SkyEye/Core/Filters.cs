// SkyEye - Sensor fusion filters (Unit 6). Pure C#, unit-tested.
using System;

namespace SkyEye.Core
{
    /// <summary>
    /// Kalman filter for ONE axis with state x = [position, velocity].
    /// Prediction uses the IMU acceleration as a control input:
    ///     p = p + v·dt + ½·a·dt²,   v = v + a·dt,   P = F P Fᵀ + Q
    /// Corrections use scalar measurements:
    ///     GPS / barometer  -> position  (H = [1 0])
    ///     optical flow     -> velocity  (H = [0 1])
    ///     K = P Hᵀ / (H P Hᵀ + R),  x = x + K (z - H x),  P = (I - K H) P
    /// Three of these (east, up, north) give the drone's fused 3D position.
    /// </summary>
    public sealed class KalmanPV
    {
        public float Position { get; private set; }
        public float Velocity { get; private set; }
        public bool Initialised { get; private set; }

        /// <summary>Std. dev. of the unmodelled acceleration (m/s²) - process noise.</summary>
        public float AccelNoise;

        float pp, pv, vv; // covariance [[pp, pv], [pv, vv]]

        public KalmanPV(float accelNoise)
        {
            AccelNoise = accelNoise;
        }

        public float PositionStdDev { get { return (float)Math.Sqrt(Math.Max(0f, pp)); } }
        public float VelocityStdDev { get { return (float)Math.Sqrt(Math.Max(0f, vv)); } }

        public void Init(float position, float velocity, float positionVar, float velocityVar)
        {
            Position = position; Velocity = velocity;
            pp = positionVar; pv = 0f; vv = velocityVar;
            Initialised = true;
        }

        public void Predict(float dt, float accel)
        {
            if (!Initialised) return;
            Position += Velocity * dt + 0.5f * accel * dt * dt;
            Velocity += accel * dt;
            float q = AccelNoise * AccelNoise;
            float dt2 = dt * dt;
            float npp = pp + 2f * dt * pv + dt2 * vv + q * dt2 * dt2 * 0.25f;
            float npv = pv + dt * vv + q * dt2 * dt * 0.5f;
            float nvv = vv + q * dt2;
            pp = npp; pv = npv; vv = nvv;
        }

        /// <summary>Position measurement z with variance r. Returns the innovation z - p.</summary>
        public float UpdatePosition(float z, float r)
        {
            if (!Initialised) { Init(z, 0f, r, 4f); return 0f; }
            float s = pp + r;
            float k0 = pp / s, k1 = pv / s;
            float y = z - Position;
            Position += k0 * y;
            Velocity += k1 * y;
            float npp = (1f - k0) * pp;
            float npv = (1f - k0) * pv;
            float nvv = vv - k1 * pv;
            pp = npp; pv = npv; vv = nvv;
            return y;
        }

        /// <summary>Velocity measurement z with variance r. Returns the innovation z - v.</summary>
        public float UpdateVelocity(float z, float r)
        {
            if (!Initialised) return 0f;
            float s = vv + r;
            float k0 = pv / s, k1 = vv / s;
            float y = z - Velocity;
            Position += k0 * y;
            Velocity += k1 * y;
            float npp = pp - k0 * pv;
            float npv = (1f - k1) * pv;
            float nvv = (1f - k1) * vv;
            pp = npp; pv = npv; vv = nvv;
            return y;
        }
    }

    /// <summary>
    /// Complementary filter for one tilt angle: trusts the gyroscope for fast changes and the
    /// accelerometer for the long-term reference.
    ///     angle = α·(angle + gyroRate·dt) + (1 − α)·accelAngle
    /// Gyro-only integration drifts (bias accumulates); accelerometer-only is noisy;
    /// the blend is both smooth and drift-free.
    /// </summary>
    public sealed class ComplementaryFilter
    {
        public float Alpha;
        public float Angle { get; private set; }

        public ComplementaryFilter(float alpha) { Alpha = alpha; }

        public void Reset(float angle) { Angle = angle; }

        public float Update(float gyroRate, float accelAngle, float dt)
        {
            Angle = Alpha * (Angle + gyroRate * dt) + (1f - Alpha) * accelAngle;
            return Angle;
        }
    }

    /// <summary>Seeded noise generator for simulated sensors.</summary>
    public sealed class NoiseSource
    {
        readonly Random rng;
        public NoiseSource(int seed) { rng = new Random(seed); }

        /// <summary>Normal(0, sigma²) sample (Box-Muller).</summary>
        public float Gaussian(float sigma)
        {
            double u1 = 1.0 - rng.NextDouble();
            double u2 = rng.NextDouble();
            return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2) * sigma);
        }

        public float Uniform() { return (float)rng.NextDouble(); }
    }

    /// <summary>
    /// First-order Gauss-Markov process: a slowly wandering bias, like the error of a GPS fix
    /// or the drift of a barometer.  b = b·e^(−dt/τ) + σ·√(1 − e^(−2dt/τ))·N(0,1)
    /// </summary>
    public sealed class RandomWalkBias
    {
        public float Value { get; private set; }
        readonly float sigma, tau;

        public RandomWalkBias(float sigma, float tau) { this.sigma = sigma; this.tau = tau; }

        public void Reset() { Value = 0f; }

        public float Step(float dt, NoiseSource noise)
        {
            float a = (float)Math.Exp(-dt / tau);
            Value = Value * a + noise.Gaussian(sigma * (float)Math.Sqrt(Math.Max(0.0, 1.0 - a * a)));
            return Value;
        }
    }

    /// <summary>Running root-mean-square and maximum of an error signal.</summary>
    public sealed class ErrorStats
    {
        double sumSq;
        public int Count { get; private set; }
        public float Max { get; private set; }

        public void Reset() { sumSq = 0.0; Count = 0; Max = 0f; }

        public void Add(float error)
        {
            float e = Math.Abs(error);
            sumSq += (double)e * e;
            Count++;
            if (e > Max) Max = e;
        }

        public float Rms { get { return Count == 0 ? 0f : (float)Math.Sqrt(sumSq / Count); } }
    }
}
