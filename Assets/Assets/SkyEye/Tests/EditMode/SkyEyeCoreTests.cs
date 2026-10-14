// SkyEye - EditMode unit tests for the core algorithms.
// Unity: Window > General > Test Runner > EditMode > Run All
using System;
using NUnit.Framework;
using SkyEye.Core;

namespace SkyEye.Tests
{
    public class OpticalFlowTests
    {
        static float Pattern(float x, float y)
        {
            return 0.5f + 0.25f * (float)(Math.Sin(0.7 * x) * Math.Cos(0.5 * y)) + 0.2f * (float)Math.Sin(0.23 * x + 0.31 * y);
        }

        static GrayImage Shifted(int size, float sx, float sy)
        {
            var g = new GrayImage(size, size);
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                    g[x, y] = Pattern(x - sx, y - sy);
            return g;
        }

        [TestCase(1.0f, 0.5f)]
        [TestCase(-2.5f, 1.5f)]
        [TestCase(0.3f, 0.0f)]
        public void LucasKanadeRecoversKnownShift(float sx, float sy)
        {
            var flow = LucasKanade.TrackGrid(Shifted(64, 0f, 0f), Shifted(64, sx, sy), 5, 10);
            float mx, my;
            int inliers;
            Assert.IsTrue(LucasKanade.MedianFlow(flow, out mx, out my, out inliers));
            Assert.AreEqual(sx, mx, 0.05f);
            Assert.AreEqual(sy, my, 0.05f);
            Assert.GreaterOrEqual(inliers, 20);
        }

        [Test]
        public void FlatImageHasNoTrackableFeatures()
        {
            var flat = new GrayImage(64, 64);
            for (int i = 0; i < flat.Data.Length; i++) flat.Data[i] = 0.5f;
            float mx, my;
            int inliers;
            Assert.IsFalse(LucasKanade.MedianFlow(LucasKanade.TrackGrid(flat, flat, 5, 10), out mx, out my, out inliers));
        }

        [Test]
        public void MetresPerPixelFromPinholeModel()
        {
            // 128 px, 60° FOV: f = 64 / tan(30°) = 110.85 px; at 25 m one pixel covers 0.2255 m
            Assert.AreEqual(110.85f, PinholeCamera.FocalPixels(128, 60f), 0.01f);
            Assert.AreEqual(0.2255f, PinholeCamera.MetresPerPixel(25f, 128, 60f), 0.001f);
        }
    }

    public class MarkerTests
    {
        const int Id = 0x6A3D;

        /// <summary>Draws the marker (8×8 cells incl. quiet zone) centred at (cx, cy), 'cell' pixels per cell.</summary>
        static GrayImage Render(int size, int id, float cx, float cy, int cell)
        {
            var g = new GrayImage(size, size);
            int n = MarkerCodec.CellsWithQuiet;
            float x0 = cx - n * cell / 2f, y0 = cy - n * cell / 2f;
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    float u = (x + 0.5f - x0) / cell, v = (y + 0.5f - y0) / cell;
                    float bg = 0.45f + 0.1f * (float)Math.Sin((y * size + x) * 0.37);
                    if (u >= 0 && u < n && v >= 0 && v < n)
                    {
                        int col = (int)u, row = n - 1 - (int)v;
                        g[x, y] = MarkerCodec.CellIsWhite(id, row, col) ? 0.95f : 0.05f;
                    }
                    else g[x, y] = bg;
                }
            return g;
        }

        [Test]
        public void EncodeDecodeRoundTrip()
        {
            var bits = new bool[4, 4];
            for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) bits[r, c] = MarkerCodec.Bit(Id, r, c);
            Assert.AreEqual(Id, MarkerCodec.Encode(bits));
            Assert.AreEqual(0, MarkerCodec.HammingDistance(Id, Id));
            Assert.AreEqual(16, MarkerCodec.HammingDistance(0x0000, 0xFFFF));
        }

        [TestCase(64f, 64f, 4)]
        [TestCase(50f, 80f, 3)]
        [TestCase(70f, 40f, 5)]
        public void DetectsMarkerAndEstimatesPose(float cx, float cy, int cell)
        {
            var d = MarkerDetector.Detect(Render(128, Id, cx, cy, cell), Id);
            Assert.IsTrue(d.Found);
            Assert.AreEqual(6 * cell, d.SidePixels, 1.0f);
            float h, e, n;
            MarkerDetector.EstimatePose(d, 128, 128, 60f, 4f, out h, out e, out n);
            float f = PinholeCamera.FocalPixels(128, 60f);
            float hTrue = f * 4f / (6 * cell);
            Assert.AreEqual(hTrue, h, 0.5f);
            Assert.AreEqual((cx - 63.5f) * hTrue / f, e, 0.3f);
            Assert.AreEqual((cy - 63.5f) * hTrue / f, n, 0.3f);
        }

        [Test]
        public void RejectsMarkerWithWrongId()
        {
            Assert.IsFalse(MarkerDetector.Detect(Render(128, Id, 64f, 64f, 4), 0x1234).Found);
        }
    }

    public class FusionTests
    {
        [Test]
        public void KalmanFusionBeatsRawGps()
        {
            var noise = new NoiseSource(2);
            var withFlow = new KalmanPV(0.5f);
            var gpsImu = new KalmanPV(0.5f);
            withFlow.Init(0f, 0f, 4f, 1f);
            gpsImu.Init(0f, 0f, 4f, 1f);
            float p = 0f, v = 0f;
            var errFlow = new ErrorStats();
            var errImu = new ErrorStats();
            var errGps = new ErrorStats();
            for (int i = 0; i < 3000; i++)
            {
                const float dt = 0.02f;
                float a = (float)Math.Sin(i * 0.01);
                p += v * dt + 0.5f * a * dt * dt;
                v += a * dt;
                float imu = a + noise.Gaussian(0.2f) + 0.05f;
                withFlow.Predict(dt, imu);
                gpsImu.Predict(dt, imu);
                if (i % 10 == 0)
                {
                    float z = p + noise.Gaussian(2f);
                    withFlow.UpdatePosition(z, 4f);
                    gpsImu.UpdatePosition(z, 4f);
                    if (i > 500) errGps.Add(z - p);
                }
                if (i % 4 == 0) withFlow.UpdateVelocity(v + noise.Gaussian(0.25f), 0.0625f);
                if (i > 500) { errFlow.Add(withFlow.Position - p); errImu.Add(gpsImu.Position - p); }
            }
            Assert.Less(errImu.Rms, errGps.Rms);       // GPS + IMU beats raw GPS
            Assert.Less(errFlow.Rms, errImu.Rms);      // adding optical-flow velocity is better still
            Assert.Less(errFlow.Rms, 0.6f);
        }

        [Test]
        public void ComplementaryFilterRemovesGyroDrift()
        {
            var noise = new NoiseSource(4);
            var cf = new ComplementaryFilter(0.98f);
            float gyroOnly = 0f;
            const float dt = 0.02f, bias = 0.8f;
            for (int i = 0; i < 3000; i++)
            {
                float rate = bias + noise.Gaussian(0.3f);     // true angle stays 0
                gyroOnly += rate * dt;
                cf.Update(rate, noise.Gaussian(4f), dt);
            }
            Assert.Greater(Math.Abs(gyroOnly), 40f);           // 60 s × 0.8 °/s ≈ 48° of drift
            Assert.Less(Math.Abs(cf.Angle), 3f);               // fused angle stays near the truth
        }

        [Test]
        public void ErrorStatsRmsAndMax()
        {
            var s = new ErrorStats();
            s.Add(3f); s.Add(-4f);
            Assert.AreEqual(Math.Sqrt(12.5), s.Rms, 1e-4);
            Assert.AreEqual(4f, s.Max, 1e-6f);
        }
    }

    public class DetectionAndGeoTests
    {
        [Test]
        public void FindsCompactOrangeBlob()
        {
            int w = 32, h = 32;
            var rgba = new byte[w * h * 4];
            for (int i = 0; i < w * h; i++) { rgba[4 * i] = 80; rgba[4 * i + 1] = 100; rgba[4 * i + 2] = 70; rgba[4 * i + 3] = 255; }
            for (int y = 10; y < 14; y++)
                for (int x = 20; x < 25; x++)
                {
                    int i = 4 * (y * w + x);
                    rgba[i] = 250; rgba[i + 1] = 115; rgba[i + 2] = 15;
                }
            var d = ColorBlobDetector.DetectOrange(rgba, w, h);
            Assert.IsTrue(d.Found);
            Assert.AreEqual(20, d.Count);
            Assert.AreEqual(22f, d.Cx, 1e-3f);
            Assert.AreEqual(11.5f, d.Cy, 1e-3f);
        }

        [Test]
        public void GeoRoundTripAndBearing()
        {
            var g = new GeoReference(16.8524, 74.5815);
            double lat, lon;
            g.ToGeo(120f, -80f, out lat, out lon);
            float e, n;
            g.ToLocal(lat, lon, out e, out n);
            Assert.AreEqual(120f, e, 0.01f);
            Assert.AreEqual(-80f, n, 0.01f);
            Assert.AreEqual(90f, GeoReference.BearingDeg(10f, 0f), 1e-3f);
            Assert.AreEqual(180f, GeoReference.BearingDeg(0f, -10f), 1e-3f);
        }
    }
}
