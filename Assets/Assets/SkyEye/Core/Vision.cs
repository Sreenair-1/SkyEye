// SkyEye - Computer vision on the drone's downward camera (Unit 5). Pure C#, unit-tested.
// Images use a bottom-left origin (row 0 = bottom), matching Unity's Texture2D.ReadPixels,
// so +x = east and +y = north for a north-up, straight-down camera.
using System;
using System.Collections.Generic;

namespace SkyEye.Core
{
    /// <summary>Single-channel float image (0..1).</summary>
    public sealed class GrayImage
    {
        public readonly int Width, Height;
        public readonly float[] Data;

        public GrayImage(int width, int height)
        {
            Width = width; Height = height;
            Data = new float[width * height];
        }

        public float this[int x, int y]
        {
            get
            {
                if (x < 0) x = 0; else if (x >= Width) x = Width - 1;
                if (y < 0) y = 0; else if (y >= Height) y = Height - 1;
                return Data[y * Width + x];
            }
            set { Data[y * Width + x] = value; }
        }

        /// <summary>Bilinear interpolation (clamped at the border).</summary>
        public float Sample(float x, float y)
        {
            int x0 = (int)Math.Floor(x), y0 = (int)Math.Floor(y);
            float fx = x - x0, fy = y - y0;
            float a = this[x0, y0], b = this[x0 + 1, y0], c = this[x0, y0 + 1], d = this[x0 + 1, y0 + 1];
            return (a * (1f - fx) + b * fx) * (1f - fy) + (c * (1f - fx) + d * fx) * fy;
        }

        /// <summary>Half-resolution image (2×2 box average) - one pyramid level.</summary>
        public GrayImage Downsample()
        {
            var r = new GrayImage(Math.Max(1, Width / 2), Math.Max(1, Height / 2));
            for (int y = 0; y < r.Height; y++)
                for (int x = 0; x < r.Width; x++)
                    r.Data[y * r.Width + x] = 0.25f * (this[2 * x, 2 * y] + this[2 * x + 1, 2 * y] +
                                                       this[2 * x, 2 * y + 1] + this[2 * x + 1, 2 * y + 1]);
            return r;
        }

        /// <summary>Luminance image from RGBA32 bytes (as returned by Texture2D.GetPixels32).</summary>
        public static GrayImage FromRgba(byte[] rgba, int width, int height)
        {
            var g = new GrayImage(width, height);
            for (int i = 0; i < width * height; i++)
                g.Data[i] = (0.299f * rgba[4 * i] + 0.587f * rgba[4 * i + 1] + 0.114f * rgba[4 * i + 2]) / 255f;
            return g;
        }
    }

    public struct FlowVector
    {
        public float X, Y;    // feature position in the previous image (pixels)
        public float Dx, Dy;  // motion to the next image (pixels)
        public bool Valid;
    }

    /// <summary>
    /// Pyramidal Lucas-Kanade optical flow. For a small window around a point, find the shift d
    /// that minimises Σ (I₂(p + d) − I₁(p))². Linearising gives the 2×2 system
    ///     G·d = −b,   G = Σ [Ix² IxIy; IxIy Iy²],   b = Σ [Ix·It; Iy·It]
    /// solved iteratively, coarse-to-fine on an image pyramid so larger motions can be tracked.
    /// </summary>
    public static class LucasKanade
    {
        public const float MinEigenvalue = 1e-4f;

        static List<GrayImage> Pyramid(GrayImage img, int levels)
        {
            var p = new List<GrayImage> { img };
            for (int i = 1; i < levels; i++) p.Add(p[i - 1].Downsample());
            return p;
        }

        /// <summary>Refines (dx, dy) for one point on one pyramid level. Returns false if the window has no texture.</summary>
        public static bool TrackLevel(GrayImage prev, GrayImage next, float x, float y, int halfWin, int iterations,
                                      ref float dx, ref float dy)
        {
            int n = (2 * halfWin + 1) * (2 * halfWin + 1);
            var ix = new float[n];
            var iy = new float[n];
            var i1 = new float[n];
            float gxx = 0f, gxy = 0f, gyy = 0f;
            int k = 0;
            for (int wy = -halfWin; wy <= halfWin; wy++)
                for (int wx = -halfWin; wx <= halfWin; wx++, k++)
                {
                    float px = x + wx, py = y + wy;
                    ix[k] = 0.5f * (prev.Sample(px + 1f, py) - prev.Sample(px - 1f, py));
                    iy[k] = 0.5f * (prev.Sample(px, py + 1f) - prev.Sample(px, py - 1f));
                    i1[k] = prev.Sample(px, py);
                    gxx += ix[k] * ix[k]; gxy += ix[k] * iy[k]; gyy += iy[k] * iy[k];
                }
            float det = gxx * gyy - gxy * gxy;
            float half = 0.5f * (gxx + gyy);
            float minEig = (half - (float)Math.Sqrt(Math.Max(0f, 0.25f * (gxx - gyy) * (gxx - gyy) + gxy * gxy))) / n;
            if (det <= 1e-12f || minEig < MinEigenvalue) return false;

            for (int it = 0; it < iterations; it++)
            {
                float bx = 0f, by = 0f;
                k = 0;
                for (int wy = -halfWin; wy <= halfWin; wy++)
                    for (int wx = -halfWin; wx <= halfWin; wx++, k++)
                    {
                        float diff = next.Sample(x + wx + dx, y + wy + dy) - i1[k];
                        bx += ix[k] * diff;
                        by += iy[k] * diff;
                    }
                float ddx = -(gyy * bx - gxy * by) / det;
                float ddy = -(-gxy * bx + gxx * by) / det;
                dx += ddx; dy += ddy;
                if (ddx * ddx + ddy * ddy < 1e-4f) break;
            }
            return true;
        }

        /// <summary>Tracks a regular grid of points from prev to next (coarse-to-fine).</summary>
        public static FlowVector[] TrackGrid(GrayImage prev, GrayImage next, int gridN, int margin,
                                             int halfWin = 3, int levels = 2, int iterations = 8)
        {
            var p1 = Pyramid(prev, levels);
            var p2 = Pyramid(next, levels);
            var result = new FlowVector[gridN * gridN];
            float span = prev.Width - 2f * margin;
            int idx = 0;
            for (int gy = 0; gy < gridN; gy++)
                for (int gx = 0; gx < gridN; gx++, idx++)
                {
                    float x = margin + span * (gx + 0.5f) / gridN;
                    float y = margin + span * (gy + 0.5f) / gridN;
                    float dx = 0f, dy = 0f;
                    bool ok = true;
                    for (int l = levels - 1; l >= 0; l--)
                    {
                        float s = 1 << l;
                        if (l < levels - 1) { dx *= 2f; dy *= 2f; }
                        if (!TrackLevel(p1[l], p2[l], x / s, y / s, halfWin, iterations, ref dx, ref dy)) { ok = false; break; }
                    }
                    // Reject non-finite results and motions larger than the pyramid can follow.
                    if (float.IsNaN(dx) || float.IsNaN(dy) || Math.Abs(dx) > 12f || Math.Abs(dy) > 12f) ok = false;
                    result[idx] = new FlowVector { X = x, Y = y, Dx = dx, Dy = dy, Valid = ok };
                }
            return result;
        }

        /// <summary>
        /// Robust overall image motion: median of the valid vectors. Vectors far from the median
        /// (outliers such as moving objects or water) are excluded from the count.
        /// </summary>
        public static bool MedianFlow(FlowVector[] flow, out float mx, out float my, out int inliers, int minInliers = 6)
        {
            var xs = new List<float>();
            var ys = new List<float>();
            foreach (var f in flow) if (f.Valid) { xs.Add(f.Dx); ys.Add(f.Dy); }
            mx = 0f; my = 0f; inliers = 0;
            if (xs.Count < minInliers) return false;
            mx = Median(xs); my = Median(ys);
            foreach (var f in flow)
                if (f.Valid && Math.Abs(f.Dx - mx) < 1.0f && Math.Abs(f.Dy - my) < 1.0f) inliers++;
            return inliers >= minInliers;
        }

        static float Median(List<float> v)
        {
            v.Sort();
            int n = v.Count;
            return n % 2 == 1 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
        }
    }

    /// <summary>Pinhole model of a square camera looking straight down.</summary>
    public static class PinholeCamera
    {
        /// <summary>Focal length in pixels for an image of height h and vertical field of view.</summary>
        public static float FocalPixels(int imageHeight, float verticalFovDeg)
        {
            return 0.5f * imageHeight / (float)Math.Tan(0.5 * verticalFovDeg * Math.PI / 180.0);
        }

        /// <summary>Ground size of one pixel (m) at a given height above the ground.</summary>
        public static float MetresPerPixel(float height, int imageHeight, float verticalFovDeg)
        {
            return height / FocalPixels(imageHeight, verticalFovDeg);
        }
    }

    /// <summary>
    /// Fiducial marker (ArUco-style): a 6×6 grid of cells whose outer ring is black and whose
    /// inner 4×4 cells encode a 16-bit ID (white = 1). A white quiet zone surrounds it.
    /// Bit (row, col), with row 0 at the TOP, is bit number row·4 + col.
    /// </summary>
    public static class MarkerCodec
    {
        public const int InnerCells = 4;
        public const int TotalCells = 6;      // inner bits + black border
        public const int CellsWithQuiet = 8;  // + white quiet zone

        public static bool Bit(int id, int row, int col) { return ((id >> (row * InnerCells + col)) & 1) == 1; }

        public static int Encode(bool[,] bits)
        {
            int id = 0;
            for (int r = 0; r < InnerCells; r++)
                for (int c = 0; c < InnerCells; c++)
                    if (bits[r, c]) id |= 1 << (r * InnerCells + c);
            return id;
        }

        /// <summary>Cell colour (true = white) for cell (row, col) of the 8×8 marker image incl. quiet zone, row 0 at the top.</summary>
        public static bool CellIsWhite(int id, int row, int col)
        {
            if (row == 0 || col == 0 || row == CellsWithQuiet - 1 || col == CellsWithQuiet - 1) return true;   // quiet zone
            if (row == 1 || col == 1 || row == CellsWithQuiet - 2 || col == CellsWithQuiet - 2) return false;  // black border
            return Bit(id, row - 2, col - 2);
        }

        /// <summary>Rotates a bit grid 90° clockwise.</summary>
        public static bool[,] Rotate(bool[,] b)
        {
            var r = new bool[InnerCells, InnerCells];
            for (int i = 0; i < InnerCells; i++)
                for (int j = 0; j < InnerCells; j++)
                    r[j, InnerCells - 1 - i] = b[i, j];
            return r;
        }

        public static int HammingDistance(int a, int b)
        {
            int x = a ^ b, n = 0;
            while (x != 0) { n += x & 1; x >>= 1; }
            return n;
        }
    }

    public struct MarkerDetection
    {
        public bool Found;
        public int Id;
        public float Cx, Cy;          // centre (pixels)
        public float SidePixels;      // mean side length of the black square
        public float AngleDeg;        // in-plane rotation of the top edge
        public float[] CornerX, CornerY; // TL, TR, BR, BL
        public int CandidatePixels;
    }

    /// <summary>
    /// Marker tracking: threshold → connected components → corner extraction → bit sampling →
    /// ID verification. Pose (height and horizontal offset) then follows from the pinhole model,
    /// because the physical marker size is known.
    /// </summary>
    public static class MarkerDetector
    {
        public static MarkerDetection Detect(GrayImage img, int expectedId, int maxBitErrors = 1)
        {
            var none = new MarkerDetection { Found = false };
            int w = img.Width, h = img.Height;
            float mn = 1f, mx = 0f;
            foreach (var v in img.Data) { if (v < mn) mn = v; if (v > mx) mx = v; }
            if (mx - mn < 0.15f) return none; // no contrast (fog, darkness)
            float thr = mn + 0.4f * (mx - mn);

            var label = new int[w * h];
            var queue = new int[w * h];
            int next = 0;
            MarkerDetection best = none;
            int bestScore = 0;

            for (int start = 0; start < w * h; start++)
            {
                if (label[start] != 0 || img.Data[start] >= thr) continue;
                next++;
                int head = 0, tail = 0, count = 0;
                queue[tail++] = start; label[start] = next;
                int minX = w, minY = h, maxX = -1, maxY = -1;
                int sMinI = start, sMaxI = start, dMinI = start, dMaxI = start;
                int sMin = int.MaxValue, sMax = int.MinValue, dMin = int.MaxValue, dMax = int.MinValue;
                while (head < tail)
                {
                    int i = queue[head++];
                    int x = i % w, y = i / w;
                    count++;
                    if (x < minX) minX = x; if (x > maxX) maxX = x;
                    if (y < minY) minY = y; if (y > maxY) maxY = y;
                    int s = x + y, d = x - y;
                    if (s < sMin) { sMin = s; sMinI = i; }
                    if (s > sMax) { sMax = s; sMaxI = i; }
                    if (d < dMin) { dMin = d; dMinI = i; }
                    if (d > dMax) { dMax = d; dMaxI = i; }
                    // 4-connected neighbours
                    if (x > 0) Visit(i - 1, img, thr, label, queue, ref tail, next);
                    if (x < w - 1) Visit(i + 1, img, thr, label, queue, ref tail, next);
                    if (y > 0) Visit(i - w, img, thr, label, queue, ref tail, next);
                    if (y < h - 1) Visit(i + w, img, thr, label, queue, ref tail, next);
                }

                int bw = maxX - minX + 1, bh = maxY - minY + 1;
                if (count < 30 || bw < 8 || bh < 8) continue;
                float aspect = bw / (float)bh;
                if (aspect < 0.6f || aspect > 1.6f) continue;
                if (minX == 0 || minY == 0 || maxX == w - 1 || maxY == h - 1) continue; // cut by the image edge

                // Corners (bottom-left origin): min(x+y)=BL, max(x+y)=TR, min(x-y)=TL, max(x-y)=BR
                var cx = new float[4];
                var cy = new float[4];
                SetCorner(cx, cy, 0, dMinI, w); // TL
                SetCorner(cx, cy, 1, sMaxI, w); // TR
                SetCorner(cx, cy, 2, dMaxI, w); // BR
                SetCorner(cx, cy, 3, sMinI, w); // BL

                // Sample the 4×4 inner bits (cell centres) with a bilinear map of the corners.
                var bits = new bool[MarkerCodec.InnerCells, MarkerCodec.InnerCells];
                for (int r = 0; r < MarkerCodec.InnerCells; r++)
                    for (int c = 0; c < MarkerCodec.InnerCells; c++)
                    {
                        float u = (c + 1.5f) / MarkerCodec.TotalCells;   // across, from the left
                        float v = (r + 1.5f) / MarkerCodec.TotalCells;   // down, from the top
                        float px = Bilinear(cx, u, v), py = Bilinear(cy, u, v);
                        bits[r, c] = img.Sample(px, py) >= thr;
                    }

                int bestRot = -1, bestErr = 99, decoded = 0;
                var rot = bits;
                for (int k = 0; k < 4; k++)
                {
                    int id = MarkerCodec.Encode(rot);
                    int err = MarkerCodec.HammingDistance(id, expectedId);
                    if (err < bestErr) { bestErr = err; bestRot = k; decoded = id; }
                    rot = MarkerCodec.Rotate(rot);
                }
                if (bestErr > maxBitErrors) continue;

                if (count > bestScore)
                {
                    bestScore = count;
                    float side = 0.25f * (Dist(cx, cy, 0, 1) + Dist(cx, cy, 1, 2) + Dist(cx, cy, 2, 3) + Dist(cx, cy, 3, 0)) + 1f;
                    best = new MarkerDetection
                    {
                        Found = true,
                        Id = expectedId,
                        Cx = 0.25f * (cx[0] + cx[1] + cx[2] + cx[3]),
                        Cy = 0.25f * (cy[0] + cy[1] + cy[2] + cy[3]),
                        SidePixels = side,
                        AngleDeg = (float)(Math.Atan2(cy[1] - cy[0], cx[1] - cx[0]) * 180.0 / Math.PI) + 90f * bestRot,
                        CornerX = cx,
                        CornerY = cy,
                        CandidatePixels = count
                    };
                }
            }
            return best;
        }

        static void Visit(int j, GrayImage img, float thr, int[] label, int[] queue, ref int tail, int id)
        {
            if (label[j] != 0 || img.Data[j] >= thr) return;
            label[j] = id;
            queue[tail++] = j;
        }

        static void SetCorner(float[] cx, float[] cy, int k, int index, int w)
        {
            cx[k] = index % w;
            cy[k] = index / w;
        }

        /// <summary>Bilinear map of normalised (u across, v down from the top) onto the TL,TR,BR,BL quad.</summary>
        static float Bilinear(float[] q, float u, float v)
        {
            float top = q[0] + u * (q[1] - q[0]);
            float bottom = q[3] + u * (q[2] - q[3]);
            return top + v * (bottom - top);
        }

        static float Dist(float[] x, float[] y, int a, int b)
        {
            float dx = x[a] - x[b], dy = y[a] - y[b];
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Pose estimation from the detected marker with the pinhole model:
        ///   height = f · S / side_px,   east = (cx − W/2)·height/f,   north = (cy − H/2)·height/f
        /// (S = physical side of the black square, f = focal length in pixels).
        /// </summary>
        public static void EstimatePose(MarkerDetection d, int width, int height, float fovDeg, float markerSize,
                                        out float heightAbove, out float east, out float north)
        {
            float f = PinholeCamera.FocalPixels(height, fovDeg);
            heightAbove = f * markerSize / Math.Max(1f, d.SidePixels);
            east = (d.Cx - 0.5f * (width - 1)) * heightAbove / f;
            north = (d.Cy - 0.5f * (height - 1)) * heightAbove / f;
        }
    }

    public struct BlobDetection
    {
        public bool Found;
        public float Cx, Cy;
        public int Count, MinX, MinY, MaxX, MaxY;
    }

    /// <summary>
    /// Colour-based object detection: finds orange (life-jacket) pixels using chromaticity
    /// (colour divided by brightness), which keeps working when the lighting dims.
    /// </summary>
    public static class ColorBlobDetector
    {
        public static bool IsOrange(byte r, byte g, byte b)
        {
            int sum = r + g + b;
            if (sum < 90) return false;
            float rn = r / (float)sum, gn = g / (float)sum, bn = b / (float)sum;
            return rn > 0.52f && gn > 0.20f && gn < 0.38f && bn < 0.16f;
        }

        public static BlobDetection DetectOrange(byte[] rgba, int width, int height, int minPixels = 3)
        {
            var d = new BlobDetection { MinX = width, MinY = height, MaxX = -1, MaxY = -1 };
            double sx = 0, sy = 0;
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    int i = 4 * (y * width + x);
                    if (!IsOrange(rgba[i], rgba[i + 1], rgba[i + 2])) continue;
                    d.Count++;
                    sx += x; sy += y;
                    if (x < d.MinX) d.MinX = x; if (x > d.MaxX) d.MaxX = x;
                    if (y < d.MinY) d.MinY = y; if (y > d.MaxY) d.MaxY = y;
                }
            d.Found = d.Count >= minPixels;
            if (d.Found)
            {
                // Reject large or scattered orange areas (e.g. a whole field tinted orange at dusk):
                // a life raft is a small, compact blob.
                int area = (d.MaxX - d.MinX + 1) * (d.MaxY - d.MinY + 1);
                if (d.Count > width * height / 6 || area > 4 * d.Count + 16) d.Found = false;
            }
            if (d.Count > 0) { d.Cx = (float)(sx / d.Count); d.Cy = (float)(sy / d.Count); }
            return d;
        }
    }

    /// <summary>
    /// Local tangent plane conversion between GPS coordinates and metres east/north of a
    /// reference point (equirectangular approximation, accurate to centimetres over a few km).
    /// </summary>
    public sealed class GeoReference
    {
        public readonly double Lat0, Lon0;
        const double MetresPerDegLat = 110574.0;
        readonly double metresPerDegLon;

        public GeoReference(double lat0, double lon0)
        {
            Lat0 = lat0; Lon0 = lon0;
            metresPerDegLon = 111320.0 * Math.Cos(lat0 * Math.PI / 180.0);
        }

        public void ToLocal(double lat, double lon, out float east, out float north)
        {
            east = (float)((lon - Lon0) * metresPerDegLon);
            north = (float)((lat - Lat0) * MetresPerDegLat);
        }

        public void ToGeo(float east, float north, out double lat, out double lon)
        {
            lat = Lat0 + north / MetresPerDegLat;
            lon = Lon0 + east / metresPerDegLon;
        }

        /// <summary>Compass bearing (0 = north, 90 = east) from a displacement.</summary>
        public static float BearingDeg(float east, float north)
        {
            float b = (float)(Math.Atan2(east, north) * 180.0 / Math.PI);
            return b < 0f ? b + 360f : b;
        }
    }
}
