using System;
using System.IO;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Off-road drivability per cell, from the world files:
    ///  - dtm_processed.raw: 16-bit heights, value/256 = meters, file row 0 =
    ///    world SOUTH (unlike the PNGs). Checked against prefabs.xml y (mean
    ///    error 2.3 m this way vs 15 m mirrored).
    ///  - dtm.raw vs dtm_processed.raw: where prefab placement re-leveled the
    ///    ground = a building footprint (towns, wilderness POIs).
    ///  - splat4_processed.png B: water surface height (0 = none) - matches
    ///    Navezgane's water_info.xml levels.
    ///  - biomes.png: pine forest etc. - trees slow an off-road drive down.
    /// TerrainMul scales the off-road cost: 1 = flat open ground.
    /// Pure file IO + math (shared with tools/sim).
    /// </summary>
    internal sealed partial class RoadNetwork
    {
        public byte[] TerrainMul;      // off-road cost multiplier x4 (4 = 1.0); null = no terrain data
        public byte[] HeightMax;       // highest ground in each cell, m (ceil); null = no terrain data

        const float FlatSlope = 0.12f;      // rise/run: ~7 deg
        const float GentleSlope = 0.25f;    // ~14 deg
        const float SteepSlope = 0.45f;     // ~24 deg; above = cliff
        const float MulGentle = 2.5f;
        const float MulSteep = 7.5f;
        const float MulBlocked = 60f;       // cliff, water
        const float MulStructure = 20f;     // re-leveled by a prefab: someone's house
        const float FlatSpeed = 10f;        // off-road speed cap on flat open ground
        const float RoughSpeed = 7f;        // ... anywhere else off-road

        public bool HasTerrain => TerrainMul != null;

        public const float RunwayBefore = 45f;   // touchdown zone + roll before the landing point
        public const float RunwayAfter = 15f;    // overrun past it

        /// <summary>
        /// A place near dest to land a gyrocopter coming from 'from': the
        /// strip from RunwayBefore short of it to RunwayAfter past it (along
        /// the approach) must be all road, or all flat open ground, and level.
        /// Roads win over fields; then the nearest to dest. False = none found
        /// within radius (land at dest itself).
        /// </summary>
        public bool FindLandingSite(Vector3 from, Vector3 dest, float radius, out Vector3 site, out bool onRoad)
        {
            site = dest;
            onRoad = false;
            if (HeightMax == null || TerrainMul == null) return false;
            float best = float.MaxValue;
            float step = CellSize;
            for (float dz = -radius; dz <= radius; dz += step)
            {
                for (float dx = -radius; dx <= radius; dx += step)
                {
                    float r = Mathf.Sqrt(dx * dx + dz * dz);
                    if (r > radius) continue;
                    Vector3 p = new Vector3(dest.x + dx, 0f, dest.z + dz);
                    Vector3 dir = p - from;
                    dir.y = 0f;
                    if (dir.sqrMagnitude < 1f) continue;
                    dir.Normalize();
                    bool road = IsFineRoad(p.x, p.z);
                    float score = r + (road ? 0f : 40f);
                    if (score >= best) continue;
                    if (!RunwayClear(p, dir, road)) continue;
                    best = score;
                    site = p;
                    onRoad = road;
                }
            }
            if (best == float.MaxValue) return false;
            int cx, cy;
            if (WorldToCell(site.x, site.z, out cx, out cy)) site.y = HeightMax[cy * CellsX + cx];
            return true;
        }

        bool RunwayClear(Vector3 p, Vector3 dir, bool road)
        {
            float lo = float.MaxValue, hi = float.MinValue;
            Vector3 side = new Vector3(dir.z, 0f, -dir.x);
            for (float t = -RunwayBefore; t <= RunwayAfter; t += 2f)
            {
                Vector3 q = p + dir * t;
                int cx, cy;
                if (!WorldToCell(q.x, q.z, out cx, out cy)) return false;
                int i = cy * CellsX + cx;
                if (RoadblockAt(q.x, q.z) != null) return false;   // barriers on that asphalt
                if (road)
                {
                    // the body's width on the road, not just its centerline
                    if (!IsFineRoad(q.x, q.z) || !IsFineRoad(q.x + side.x * 1.5f, q.z + side.z * 1.5f) ||
                        !IsFineRoad(q.x - side.x * 1.5f, q.z - side.z * 1.5f)) return false;
                }
                else if (Cells[i] != CellOffRoad || TerrainMul[i] > 5)
                {
                    return false;   // flat & open (desert, plains, wasteland): no slope, forest, water, buildings, town lots
                }
                float h = HeightMax[i];
                if (h < lo) lo = h;
                if (h > hi) hi = h;
                if (hi - lo > 1.5f) return false;
            }
            return true;
        }

        /// <summary>
        /// Highest ground (m) within halfWidth of the straight line a-b, from
        /// the heightmap (no trees or buildings). NaN without terrain data.
        /// </summary>
        public float MaxHeightAlong(Vector3 a, Vector3 b, float halfWidth)
        {
            if (HeightMax == null) return float.NaN;
            Vector3 d = b - a;
            d.y = 0f;
            float len = d.magnitude;
            Vector3 dir = len > 1e-3f ? d / len : Vector3.forward;
            Vector3 side = new Vector3(dir.z, 0f, -dir.x);
            float top = float.NaN;
            for (float t = 0f; t <= len + CellSize; t += CellSize * 0.5f)
            {
                for (float w = -halfWidth; w <= halfWidth; w += CellSize)
                {
                    Vector3 p = a + dir * Mathf.Min(t, len) + side * w;
                    int cx, cy;
                    if (!WorldToCell(p.x, p.z, out cx, out cy)) continue;
                    float h = HeightMax[cy * CellsX + cx];
                    if (float.IsNaN(top) || h > top) top = h;
                }
            }
            return top;
        }

        public float OffRoadMul(int cell)
        {
            return TerrainMul[cell] * 0.25f;
        }

        /// <summary>Speed cap for a route point (MaxValue on roads).</summary>
        public float SpeedCapAt(float wx, float wz)
        {
            if (IsFineRoad(wx, wz)) return float.MaxValue;
            if (TerrainMul != null && WorldToCell(wx, wz, out int cx, out int cy) &&
                OffRoadMul(cy * CellsX + cx) <= 1.5f)
            {
                return FlatSpeed;
            }
            return RoughSpeed;
        }

        /// <summary>Reads the terrain files in dir; returns a status line. Leaves TerrainMul null on failure.</summary>
        public string AttachTerrain(string dir)
        {
            string dtmFile = Path.Combine(dir, "dtm_processed.raw");
            string rawFile = Path.Combine(dir, "dtm.raw");
            if (!File.Exists(dtmFile)) dtmFile = rawFile;
            if (!File.Exists(dtmFile)) return "no dtm - off-road costs flat";
            bool diffPrefabs = File.Exists(rawFile) && dtmFile != rawFile;

            long len = new FileInfo(dtmFile).Length;
            int n = (int)Math.Round(Math.Sqrt(len / 2.0));
            if ((long)n * n * 2 != len) return "dtm size " + len + " is not square - off-road costs flat";
            float mpp = WorldW / n;                 // heightmap meters per pixel
            if (CellSize < mpp) return "cells smaller than a heightmap pixel - off-road costs flat";
            int W = CellsX, H = CellsY;
            // each pixel belongs to the cell whose world-space bounds hold it, so
            // any cell size (fractional, or not dividing the map) lines up with
            // the road cells
            var colCell = new int[n];
            for (int x = 0; x < n; x++) colCell[x] = Mathf.Min(W - 1, (int)(x * mpp / CellSize));

            // --- cell mean height, highest ground, max prefab re-leveling per cell ---
            var height = new float[W * H];
            var heightMax = new byte[W * H];
            var structure = new bool[W * H];
            using (var fp = new FileStream(dtmFile, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20))
            using (var fr = diffPrefabs ? new FileStream(rawFile, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20) : null)
            {
                var bp = new byte[n * 2];
                var br = diffPrefabs ? new byte[n * 2] : null;
                var sum = new float[W];
                var cnt = new int[W];
                var maxDiff = new float[W];
                var maxH = new float[W];
                int curRow = -1;
                for (int y = 0; y <= n; y++)
                {
                    int cy = y < n ? Mathf.Min(H - 1, (int)(y * mpp / CellSize)) : -1;
                    if (cy != curRow)
                    {
                        if (curRow >= 0)
                        {
                            for (int cx = 0; cx < W; cx++)
                            {
                                int i = curRow * W + cx;
                                height[i] = cnt[cx] > 0 ? sum[cx] / cnt[cx] : 0f;
                                heightMax[i] = (byte)Math.Min(255, (int)Math.Ceiling(maxH[cx]));
                                structure[i] = maxDiff[cx] > 0.5f;
                            }
                        }
                        if (y == n) break;
                        curRow = cy;
                        Array.Clear(sum, 0, W);
                        Array.Clear(cnt, 0, W);
                        Array.Clear(maxDiff, 0, W);
                        Array.Clear(maxH, 0, W);
                    }
                    ReadFully(fp, bp, n * 2);
                    if (fr != null) ReadFully(fr, br, n * 2);
                    for (int x = 0; x < n; x++)
                    {
                        int i = x * 2;
                        float hp = (bp[i] | (bp[i + 1] << 8)) / 256f;
                        int cx = colCell[x];
                        sum[cx] += hp;
                        cnt[cx]++;
                        if (hp > maxH[cx]) maxH[cx] = hp;
                        if (br != null)
                        {
                            float d = Math.Abs(hp - (br[i] | (br[i + 1] << 8)) / 256f);
                            if (d > maxDiff[cx]) maxDiff[cx] = d;
                        }
                    }
                }
            }

            // --- water surface per cell (splat4 B), biome per cell ---
            byte[] water = null;
            string wf = Path.Combine(dir, "splat4_processed.png");
            if (File.Exists(wf))
            {
                try { water = LoadPerCellMax(wf, (r, g, b, a) => b); }
                catch (Exception) { water = null; }
            }
            byte[] biome = null;
            int bw = 0, bh = 0;
            string bf = Path.Combine(dir, "biomes.png");
            if (File.Exists(bf))
            {
                try { biome = PngRoadMask.Load(bf, 1, BiomeCode, out bw, out bh); }
                catch (Exception) { biome = null; }
            }

            // --- combine ---
            HeightMax = heightMax;
            TerrainMul = new byte[W * H];
            int flat = 0, cliff = 0, wet = 0, built = 0;
            float diag = CellSize * 1.4142136f;
            for (int cy = 0; cy < H; cy++)
            {
                for (int cx = 0; cx < W; cx++)
                {
                    int i = cy * W + cx;
                    float h = height[i], slope = 0f;
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            if (dx == 0 && dy == 0) continue;
                            int x = cx + dx, y = cy + dy;
                            if (x < 0 || y < 0 || x >= W || y >= H) continue;
                            float sl = Math.Abs(height[y * W + x] - h) / (dx != 0 && dy != 0 ? diag : CellSize);
                            if (sl > slope) slope = sl;
                        }
                    }
                    float mul = slope <= FlatSlope ? 1f : slope <= GentleSlope ? MulGentle
                        : slope <= SteepSlope ? MulSteep : MulBlocked;
                    bool isCliff = mul >= MulBlocked;
                    if (mul <= MulGentle && biome != null)
                    {
                        int bx = Mathf.Min(bw - 1, cx * bw / W), by = Mathf.Min(bh - 1, cy * bh / H);
                        mul *= BiomeMul(biome[by * bw + bx]);
                    }
                    bool isWet = water != null && water[i] > 0 && h < water[i] - 0.3f;
                    if (isWet) mul = MulBlocked;
                    bool isBuilt = structure[i] || Near(structure, cx, cy, W, H);
                    if (isBuilt) mul = Math.Max(mul, MulStructure);
                    TerrainMul[i] = (byte)Mathf.Clamp(Mathf.RoundToInt(mul * 4f), 4, 255);
                    if (Cells[i] == CellOffRoad)
                    {
                        if (mul <= 1.5f) flat++;
                        if (isCliff) cliff++;
                        if (isWet) wet++;
                        if (isBuilt) built++;
                    }
                }
            }
            // town lots are flat and rarely re-leveled enough to read as
            // built, but they are yards, fences and houses: never a shortcut
            int town = 0;
            if (TownTiles != null)
            {
                byte townMul = (byte)Mathf.RoundToInt(MulStructure * 4f);
                int span = Mathf.CeilToInt(TownTileSize / CellSize);
                foreach (var t in TownTiles)
                {
                    WorldToCell(t.x + CellSize * 0.5f, t.y + CellSize * 0.5f, out int x0, out int y0);
                    for (int y = Mathf.Max(0, y0); y < Mathf.Min(H, y0 + span); y++)
                    {
                        for (int x = Mathf.Max(0, x0); x < Mathf.Min(W, x0 + span); x++)
                        {
                            int i = y * W + x;
                            if (Cells[i] != CellOffRoad || TerrainMul[i] >= townMul) continue;
                            TerrainMul[i] = townMul;
                            town++;
                        }
                    }
                }
            }
            int off = CellsX * CellsY - RoadCellCount;
            int den = Math.Max(1, off);
            return "terrain (off-road cells): flat&open " + (100 * flat / den) + "%, cliff " + (100 * cliff / den) +
                "%, water " + (100 * wet / den) + "%, building " + (100 * built / den) + "%, town lots " +
                (100 * town / den) + "%" +
                (water == null ? " (no water map)" : "") + (biome == null ? " (no biome map)" : "") +
                (diffPrefabs ? "" : " (no building map)");
        }

        static bool Near(bool[] m, int cx, int cy, int W, int H)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int x = cx + dx, y = cy + dy;
                    if (x >= 0 && y >= 0 && x < W && y < H && m[y * W + x]) return true;
                }
            }
            return false;
        }

        // biomes.png colors (RWG): pine forest, burnt forest, snow, wasteland, desert
        static byte BiomeCode(byte r, byte g, byte b, byte a)
        {
            if (r == 0 && g == 64 && b == 0) return 1;
            if (r == 186 && g == 0 && b == 255) return 2;
            if (r == 255 && g == 255 && b == 255) return 3;
            if (r == 255 && g == 168 && b == 0) return 4;
            if (r == 255 && g == 228 && b == 119) return 5;
            return 0;
        }

        /// <summary>Tree/rubble density: how much slower an off-road leg is there.</summary>
        static float BiomeMul(byte code)
        {
            switch (code)
            {
                case 1: return 2.0f;   // pine forest: dense trees
                case 2: return 1.4f;   // burnt forest
                case 3: return 1.5f;   // snow: trees, rocks
                case 4: return 1.2f;   // wasteland: rubble
                default: return 1f;    // desert / unknown
            }
        }

        /// <summary>
        /// A whole-map PNG reduced to the cell grid (per-cell max), for any
        /// cell size: an integral pixel stride streams it at that factor,
        /// otherwise it's read at full resolution and binned by world bounds.
        /// Null when the image doesn't cover the map's cells.
        /// </summary>
        byte[] LoadPerCellMax(string file, PngRoadMask.Classify classify)
        {
            int W = CellsX, H = CellsY;
            int sw0, sh0;
            PngRoadMask.ReadSize(file, out sw0, out sh0);
            float stride = CellSize / (WorldW / sw0);
            int si = Mathf.RoundToInt(stride);
            if (si >= 1 && Mathf.Abs(stride - si) < 1e-3f)
            {
                byte[] m = PngRoadMask.Load(file, si, classify, out int ww, out int wh);
                return ww == W && wh == H ? m : null;
            }
            byte[] px = PngRoadMask.Load(file, 1, classify, out int sw, out int sh);
            float mppX = WorldW / sw, mppY = WorldH / sh;
            var outp = new byte[W * H];
            for (int y = 0; y < sh; y++)
            {
                int cy = Mathf.Min(H - 1, (int)(y * mppY / CellSize));
                for (int x = 0; x < sw; x++)
                {
                    int cx = Mathf.Min(W - 1, (int)(x * mppX / CellSize));
                    byte v = px[y * sw + x];
                    int i = cy * W + cx;
                    if (v > outp[i]) outp[i] = v;
                }
            }
            return outp;
        }

        static void ReadFully(Stream s, byte[] buf, int count)
        {
            int off = 0;
            while (off < count)
            {
                int r = s.Read(buf, off, count - off);
                if (r <= 0) throw new EndOfStreamException();
                off += r;
            }
        }
    }
}
