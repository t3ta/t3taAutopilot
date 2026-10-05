using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Prefabs that physically block a vehicle, from prefabs.xml and the
    /// game's prefab definitions (Data/Prefabs/**/NAME.xml: PrefabSize,
    /// ThemeTags). Every recorded stuck but one was at one of these:
    ///  - road checkpoints (roadside_checkpoint_*, the town gateway
    ///    part_driveway_gateway_checkpoint_*, ThemeTags "checkpoint"): military
    ///    and concrete barriers, cones and barricades standing on asphalt the
    ///    splat map shows as a plain road;
    ///  - trader compounds (trader_*): fenced, barriers at the gate.
    /// Road drainage culverts (part_road_drainage) are a ditch running under
    /// the road: the asphalt over them is fine, but the ditch either side is a
    /// trench a detour along the shoulder drove into. Their off-road cells get
    /// RoadblockMul too.
    /// Every other POI (not a part_ piece, not a checkpoint) gets LotMul over
    /// its footprint, asphalt included: gas station forecourts and store car
    /// parks are asphalt in the splat map, so plans and detours cut across
    /// them - into bollards, fences and shopping carts - instead of taking
    /// the street round the block. The streets themselves belong to the
    /// rwg_tile, not to a POI.
    /// (Many houses, schools and camps also carry sandbags or cones, but beside
    /// the road - matching on the block palette flagged 400 prefabs.)
    /// The footprint (rotated, plus RoadblockMargin: barriers stand at the
    /// edges and on the road just outside) gets RoadblockMul on every cell,
    /// road or not, so plans and detours go around when there is a way.
    /// Pure file IO + math (shared with tools/sim).
    /// </summary>
    internal sealed partial class RoadNetwork
    {
        public static float RoadblockMul = 12f;
        public static float LotMul = 4f;
        const float LotInset = 2f;
        public const float RoadblockMargin = 8f;

        public byte[] HazardMul;                    // cost multiplier per cell (1 = none); null = no prefab data
        public readonly List<Rect> Roadblocks = new List<Rect>();   // world x/z footprints incl. margin
        public readonly List<Rect> Lots = new List<Rect>();         // POI footprints, inset

        static readonly Regex PrefabRx = new Regex(
            @"name=""([^""]+)""[^>]*position=""(-?\d+),-?\d+,(-?\d+)""[^>]*rotation=""(\d)""", RegexOptions.Compiled);
        static readonly Regex SizeRx = new Regex(@"PrefabSize""\s+value=""(\d+),\s*(\d+),\s*(\d+)""", RegexOptions.Compiled);
        static readonly Regex ThemeRx = new Regex(@"ThemeTags""\s+value=""([^""]*)""", RegexOptions.Compiled);

        public int DitchCount, LotCount;

        public float HazardAt(int cell) => HazardMul == null ? 1f : HazardMul[cell];

        static bool IsDitch(string name) => name.StartsWith("part_road_drainage");

        /// <summary>A POI with a lot of its own (not a road-side part, not a town tile).</summary>
        static bool IsLot(string name) => !name.StartsWith("part_") && !name.StartsWith("rwg_tile_");

        static bool IsRoadblock(string name, string themeTags)
        {
            return name.StartsWith("trader_") || name.Contains("checkpoint") ||
                themeTags.IndexOf("checkpoint", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        /// <param name="prefabDirs">Data/Prefabs folders (their POIs / Parts / RWGTiles subfolders are searched)</param>
        /// <returns>status line</returns>
        public string AttachPrefabs(string prefabsXml, IList<string> prefabDirs)
        {
            if (!File.Exists(prefabsXml)) return "no prefabs.xml";
            var info = new Dictionary<string, PrefabInfo>();
            int unknown = 0;
            HazardMul = new byte[CellsX * CellsY];
            for (int i = 0; i < HazardMul.Length; i++) HazardMul[i] = 1;
            byte mul = (byte)Mathf.Clamp(Mathf.RoundToInt(RoadblockMul), 1, 255);
            byte lotMul = (byte)Mathf.Clamp(Mathf.RoundToInt(LotMul), 1, 255);
            foreach (Match m in PrefabRx.Matches(File.ReadAllText(prefabsXml)))
            {
                string name = m.Groups[1].Value;
                if (name.StartsWith("rwg_tile_")) continue;
                PrefabInfo pi;
                if (!info.TryGetValue(name, out pi))
                {
                    pi = Describe(name, prefabDirs);
                    info[name] = pi;
                }
                if (!pi.Roadblock && !pi.Ditch && !pi.Lot) continue;
                if (pi.SizeX == 0) { if (!pi.Lot) unknown++; continue; }
                int x = int.Parse(m.Groups[2].Value), z = int.Parse(m.Groups[3].Value);
                int rot = int.Parse(m.Groups[4].Value);
                float sx = rot % 2 == 0 ? pi.SizeX : pi.SizeZ, sz = rot % 2 == 0 ? pi.SizeZ : pi.SizeX;
                if (pi.Ditch)
                {
                    MarkRect(new Rect(x, z, sx, sz), mul, true);
                    DitchCount++;
                    continue;
                }
                if (!pi.Roadblock)
                {
                    // inset by at least half a cell: any cell touching the inset
                    // rectangle then has its centre inside the footprint, so the
                    // edge cells don't reach onto the street (at any cellSize)
                    float inset = Mathf.Max(LotInset, CellSize * 0.5f);
                    if (sx > 2f * inset && sz > 2f * inset)
                    {
                        var lot = new Rect(x + inset, z + inset, sx - 2f * inset, sz - 2f * inset);
                        Lots.Add(lot);
                        if (lotMul > 1) MarkRect(lot, lotMul, false);
                    }
                    LotCount++;
                    continue;
                }
                var r = new Rect(x - RoadblockMargin, z - RoadblockMargin, sx + 2f * RoadblockMargin, sz + 2f * RoadblockMargin);
                Roadblocks.Add(r);
                MarkRect(r, mul, false);
            }
            return Roadblocks.Count + " roadblock prefabs (checkpoints, trader compounds), " + DitchCount + " drainage ditches, " + LotCount + " POI lots" +
                (unknown > 0 ? ", " + unknown + " without a size" : "");
        }

        void MarkRect(Rect r, byte mul, bool offRoadOnly)
        {
            int cx0, cy0, cx1, cy1;
            ClampToCell(r.xMin, r.yMin, out cx0, out cy0);
            ClampToCell(r.xMax, r.yMax, out cx1, out cy1);
            for (int cy = cy0; cy <= cy1; cy++)
            {
                for (int cx = cx0; cx <= cx1; cx++)
                {
                    int i = cy * CellsX + cx;
                    if (offRoadOnly && Cells[i] != CellOffRoad) continue;
                    if (HazardMul[i] < mul) HazardMul[i] = mul;
                }
            }
        }

        /// <summary>The roadblock footprint holding (x, z), or null.</summary>
        public Rect? RoadblockAt(float x, float z)
        {
            foreach (var r in Roadblocks)
            {
                if (r.Contains(new Vector2(x, z))) return r;
            }
            return null;
        }

        /// <summary>
        /// A destination inside a roadblock footprint (a trader compound, a
        /// checkpoint) is swapped for the nearest road point outside it within
        /// maxDist - driving at it would only ram the fence. Unchanged otherwise.
        /// </summary>
        public Vector3 OutsideRoadblocks(Vector3 dest, float maxDist)
        {
            if (RoadblockAt(dest.x, dest.z) == null) return dest;
            float step = FineMpp;
            for (float r = step; r <= maxDist; r += step)
            {
                int n = Mathf.Max(8, Mathf.CeilToInt(2f * Mathf.PI * r / step));
                float best = float.MaxValue;
                Vector3 bestP = dest;
                for (int k = 0; k < n; k++)
                {
                    float a = k * 2f * Mathf.PI / n;
                    float x = dest.x + r * Mathf.Cos(a), z = dest.z + r * Mathf.Sin(a);
                    if (!IsFineRoad(x, z) || RoadblockAt(x, z) != null) continue;
                    float d = (x - dest.x) * (x - dest.x) + (z - dest.z) * (z - dest.z);
                    if (d < best) { best = d; bestP = new Vector3(x, dest.y, z); }
                }
                if (best < float.MaxValue) return bestP;
            }
            return dest;
        }

        /// <summary>
        /// Where to stop for a destination that is a whole area (a quest POI):
        /// a road point outside the area, on the side 'from' approaches.
        /// Rings around the center are searched outward (up to extra meters
        /// past the area's edge). The first ring with a road point on the
        /// approach half (facing 'from' as seen from the center) wins, nearest
        /// to 'from' within it; the far side is only used when the approach
        /// half has no road within ApproachBand meters of the first road found,
        /// so a far-side road a little closer to the center doesn't send the
        /// car around the POI. Without a road: just outside the edge (EdgeOutside).
        /// </summary>
        public Vector3 OutsideArea(Vector3 dest, Rect area, Vector3 from, float extra)
        {
            const float ApproachBand = 20f;
            float step = FineMpp;
            float maxDist = 0.5f * area.size.magnitude + extra;
            float stopAt = maxDist;
            float fx = from.x - dest.x, fz = from.z - dest.z;   // toward the approach side
            float bestAny = float.MaxValue, anyRing = 0f;
            Vector3 anyP = dest;
            for (float r = step; r <= stopAt; r += step)
            {
                int n = Mathf.Max(8, Mathf.CeilToInt(2f * Mathf.PI * r / step));
                float bestFront = float.MaxValue;
                Vector3 frontP = dest;
                for (int k = 0; k < n; k++)
                {
                    float a = k * 2f * Mathf.PI / n;
                    float ox = r * Mathf.Cos(a), oz = r * Mathf.Sin(a);
                    float x = dest.x + ox, z = dest.z + oz;
                    if (area.Contains(new Vector2(x, z)) || !IsFineRoad(x, z) || RoadblockAt(x, z) != null) continue;
                    float d = (x - from.x) * (x - from.x) + (z - from.z) * (z - from.z);
                    if (bestAny == float.MaxValue || (r == anyRing && d < bestAny))
                    {
                        // the first ring with any road: fallback if the approach half has none
                        anyRing = r;
                        stopAt = Mathf.Min(maxDist, r + ApproachBand);
                        bestAny = d;
                        anyP = new Vector3(x, dest.y, z);
                    }
                    if (ox * fx + oz * fz > 0f && d < bestFront) { bestFront = d; frontP = new Vector3(x, dest.y, z); }
                }
                if (bestFront < float.MaxValue) return frontP;
            }
            return bestAny < float.MaxValue ? anyP : EdgeOutside(dest, area, from);
        }

        /// <summary>
        /// Where to stop for a precise spot inside an area (a quest's rally
        /// marker in its POI): the road point outside the area nearest to the
        /// spot, if it is at most 'extra' meters farther than the area's edge;
        /// else just outside the edge nearest to the spot (EdgeOutside).
        /// </summary>
        public Vector3 OutsideAreaNear(Vector3 spot, Rect area, float extra)
        {
            Vector3 edge = EdgeOutside(spot, area, spot);
            float edgeDist = Mathf.Sqrt((edge.x - spot.x) * (edge.x - spot.x) + (edge.z - spot.z) * (edge.z - spot.z));
            float step = FineMpp;
            for (float r = step; r <= edgeDist + extra; r += step)
            {
                int n = Mathf.Max(8, Mathf.CeilToInt(2f * Mathf.PI * r / step));
                for (int k = 0; k < n; k++)
                {
                    float a = k * 2f * Mathf.PI / n;
                    float x = spot.x + r * Mathf.Cos(a), z = spot.z + r * Mathf.Sin(a);
                    if (area.Contains(new Vector2(x, z)) || !IsFineRoad(x, z) || RoadblockAt(x, z) != null) continue;
                    return new Vector3(x, spot.y, z);   // rings grow outward: the first hit is the nearest
                }
            }
            return edge;
        }

        /// <summary>
        /// The point just outside the area's edge nearest to 'from' (the
        /// approach side). Needs no road data, so it also works on network
        /// clients, which have no world files.
        /// </summary>
        public static Vector3 EdgeOutside(Vector3 dest, Rect area, Vector3 from)
        {
            const float margin = 4f;
            float ex = Mathf.Clamp(from.x, area.xMin - margin, area.xMax + margin);
            float ez = Mathf.Clamp(from.z, area.yMin - margin, area.yMax + margin);
            if (area.Contains(new Vector2(ex, ez)))
            {
                // 'from' is inside the area: leave by the nearest edge
                float dl = ex - area.xMin, dr = area.xMax - ex, db = ez - area.yMin, dt = area.yMax - ez;
                float m = Mathf.Min(Mathf.Min(dl, dr), Mathf.Min(db, dt));
                if (m == dl) ex = area.xMin - margin;
                else if (m == dr) ex = area.xMax + margin;
                else if (m == db) ez = area.yMin - margin;
                else ez = area.yMax + margin;
            }
            return new Vector3(ex, dest.y, ez);
        }

        struct PrefabInfo
        {
            public int SizeX, SizeZ;
            public bool Roadblock, Ditch, Lot;
        }

        static PrefabInfo Describe(string name, IList<string> prefabDirs)
        {
            var pi = new PrefabInfo();
            // cheap name test first: only roadblocks need their definition read
            pi.Roadblock = IsRoadblock(name, "");
            pi.Ditch = IsDitch(name);
            pi.Lot = !pi.Roadblock && IsLot(name);
            foreach (string root in prefabDirs ?? new string[0])
            {
                foreach (string sub in new[] { "POIs", "Parts", "RWGTiles", "" })
                {
                    string xml = Path.Combine(Path.Combine(root, sub), name + ".xml");
                    if (!File.Exists(xml)) continue;
                    try
                    {
                        string text = File.ReadAllText(xml);
                        var t = ThemeRx.Match(text);
                        pi.Roadblock = IsRoadblock(name, t.Success ? t.Groups[1].Value : "");
                        pi.Lot = !pi.Roadblock && IsLot(name);
                        if (!pi.Roadblock && !pi.Ditch && !pi.Lot) return pi;
                        var m = SizeRx.Match(text);
                        if (m.Success)
                        {
                            pi.SizeX = int.Parse(m.Groups[1].Value);
                            pi.SizeZ = int.Parse(m.Groups[3].Value);
                        }
                    }
                    catch (Exception) { }
                    return pi;
                }
            }
            return pi;
        }
    }
}
