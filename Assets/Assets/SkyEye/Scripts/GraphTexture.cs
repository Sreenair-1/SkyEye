// SkyEye - Tiny line-chart renderer drawing into a Texture2D (used for live sensor graphs).
using System.Collections.Generic;
using UnityEngine;

namespace SkyEye
{
    public sealed class GraphTexture
    {
        public readonly Texture2D Texture;
        readonly int width, height, capacity;
        readonly List<float[]> samples = new List<float[]>();
        readonly Color32[] colours;
        readonly Color32[] pixels;
        readonly float fixedMin, fixedMax;
        readonly bool autoScale;
        public float ShownMin { get; private set; }
        public float ShownMax { get; private set; }

        public GraphTexture(int width, int height, Color32[] seriesColours, float min, float max, bool autoScale)
        {
            this.width = width; this.height = height;
            capacity = width;
            colours = seriesColours;
            fixedMin = min; fixedMax = max;
            this.autoScale = autoScale;
            Texture = new Texture2D(width, height, TextureFormat.RGBA32, false) { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
            pixels = new Color32[width * height];
        }

        public void Clear() { samples.Clear(); }

        public void Push(params float[] values)
        {
            samples.Add((float[])values.Clone());
            if (samples.Count > capacity) samples.RemoveAt(0);
        }

        public void Redraw()
        {
            var bg = new Color32(16, 20, 28, 230);
            var grid = new Color32(45, 52, 64, 255);
            for (int i = 0; i < pixels.Length; i++) pixels[i] = bg;
            for (int g = 1; g < 4; g++) for (int x = 0; x < width; x++) pixels[(g * height / 4) * width + x] = grid;

            float min = fixedMin, max = fixedMax;
            if (autoScale && samples.Count > 0)
            {
                min = float.MaxValue; max = float.MinValue;
                foreach (var s in samples) foreach (var v in s) { if (v < min) min = v; if (v > max) max = v; }
                if (max - min < 1e-3f) { max += 1f; min -= 1f; }
                float pad = 0.1f * (max - min); min -= pad; max += pad;
            }
            ShownMin = min; ShownMax = max;
            int zeroY = Mathf.RoundToInt((0f - min) / (max - min) * (height - 1));
            if (zeroY > 0 && zeroY < height) for (int x = 0; x < width; x++) pixels[zeroY * width + x] = new Color32(80, 90, 105, 255);

            int series = colours.Length;
            for (int si = 0; si < series; si++)
            {
                int prevX = -1, prevY = 0;
                for (int i = 0; i < samples.Count; i++)
                {
                    if (si >= samples[i].Length) continue;
                    int x = i * (width - 1) / Mathf.Max(1, capacity - 1);
                    int y = Mathf.Clamp(Mathf.RoundToInt((samples[i][si] - min) / (max - min) * (height - 1)), 0, height - 1);
                    if (prevX >= 0) Line(prevX, prevY, x, y, colours[si]);
                    prevX = x; prevY = y;
                }
            }
            Texture.SetPixels32(pixels);
            Texture.Apply(false);
        }

        void Line(int x0, int y0, int x1, int y1, Color32 c)
        {
            int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy, guard = 0;
            while (guard++ < 2000)
            {
                if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height)
                {
                    pixels[y0 * width + x0] = c;
                    if (y0 + 1 < height) pixels[(y0 + 1) * width + x0] = c; // 2 px thick
                }
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }
    }
}
