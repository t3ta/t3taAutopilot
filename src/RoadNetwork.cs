using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Navigability grid built from the world's road splatmap.
    /// Source: splat3_processed.png R (asphalt) | G (gravel) - the final
    /// splat including roads stamped by town/tile prefabs. The raw splat3.png
    /// only has the RWG connector roads (none at all on Navezgane), with gaps
    /// wherever a town tile sits.
    /// Orientation: image TOP row = world NORTH (z = +worldSize/2). Verified
    /// on three worlds: 90-100% of road-side prefabs in prefabs.xml sit within
    /// 12 m of road with this mapping vs 26-31% z-mirrored.
    ///
    /// Two layers:
    ///  - Fine: the road mask at ~2 m, used to pull route points onto the
    ///    road centerline.
    ///  - Cells: a coarse (CellSize) grid for A*, plus EdgeDist (distance in
    ///    cells to the nearest off-road cell) so A* prefers the middle of wide
    ///    roads over their shoulders.
    ///
    /// This half has no game dependencies so the offline simulator
    /// (tools/sim) can build it too. Loading from the running game
    /// lives in RoadNetwork.Loader.cs.
    /// </summary>
    internal sealed partial class RoadNetwork
    {
        public const byte CellRoad = 0;       // asphalt
        public const byte CellOffRoad = 1;
        public const byte CellGravel = 2;

        // Fine mask values
        public const byte FineNone = 0;
        public const byte FineGravel = 1;
        public const byte FineAsphalt = 2;

        public int CellsX, CellsY;
        public float CellSize;
        public float WorldW, WorldH;   // meters
        public byte[] Cells;           // CellRoad / CellGravel / CellOffRoad
        public byte[] EdgeDist;        // cells to nearest off-road cell (0 = off-road)
        public int RoadCellCount;

        public int FineW, FineH;
        public float FineMpp;          // meters per fine pixel
        public byte[] Fine;            // FineNone/Gravel/Asphalt, row 0 = world south

        /// <param name="fine">Fine* class per pixel, row 0 = world south</param>
        public static RoadNetwork Build(byte[] fine, int w, int h, float metersPerPx, float cellSize)
        {
            var net = new RoadNetwork
            {
                WorldW = w * metersPerPx,
                WorldH = h * metersPerPx,
                CellSize = cellSize,
                FineW = w,
                FineH = h,
                FineMpp = metersPerPx,
                Fine = fine
            };

            net.CellsX = Mathf.CeilToInt(net.WorldW / cellSize);
            net.CellsY = Mathf.CeilToInt(net.WorldH / cellSize);
            net.Cells = new byte[net.CellsX * net.CellsY];
            // each cell covers the fine pixels inside its own world-space bounds
            // (a single rounded stride drifts when cellSize isn't a multiple of
            // the fine resolution, e.g. 3 m cells over a 2 m mask)
            float pxPerCell = cellSize / metersPerPx;
            for (int cy = 0; cy < net.CellsY; cy++)
            {
                int fy0 = Mathf.Min(h - 1, (int)(cy * pxPerCell));
                int fy1 = Mathf.Min(h, Mathf.Max(fy0 + 1, (int)((cy + 1) * pxPerCell)));
                for (int cx = 0; cx < net.CellsX; cx++)
                {
                    int fx0 = Mathf.Min(w - 1, (int)(cx * pxPerCell));
                    int fx1 = Mathf.Min(w, Mathf.Max(fx0 + 1, (int)((cx + 1) * pxPerCell)));
                    byte cls = FineNone;
                    for (int fy = fy0; fy < fy1; fy++)
                    {
                        int row = fy * w;
                        for (int fx = fx0; fx < fx1; fx++)
                        {
                            if (net.Fine[row + fx] > cls) cls = net.Fine[row + fx];
                        }
                    }
                    net.Cells[cy * net.CellsX + cx] =
                        cls == FineAsphalt ? CellRoad : cls == FineGravel ? CellGravel : CellOffRoad;
                    if (cls != FineNone) net.RoadCellCount++;
                }
            }
            net.BuildEdgeDist();
            return net;
        }

        public const float TownTileSize = 150f;   // RWG town tile edge; prefabs.xml gives its min corner

        static readonly Regex TileRx = new Regex(
            @"name=""(rwg_tile_[^""]*)""[^>]*position=""(-?\d+),-?\d+,(-?\d+)""", RegexOptions.Compiled);

        /// <summary>Min corners (world x, z) of the RWG town tiles; null = none read.</summary>
        public System.Collections.Generic.List<Vector2> TownTiles;

        /// <summary>Min corners (world x, z) of the town tiles in prefabs.xml.</summary>
        public static System.Collections.Generic.List<Vector2> ReadTownTiles(string prefabsXml)
        {
            var list = new System.Collections.Generic.List<Vector2>();
            if (!File.Exists(prefabsXml)) return list;
            foreach (Match m in TileRx.Matches(File.ReadAllText(prefabsXml)))
            {
                // old-west streets are gravel: leave those tiles alone
                if (m.Groups[1].Value.StartsWith("rwg_tile_oldwest")) continue;
                list.Add(new Vector2(int.Parse(m.Groups[2].Value), int.Parse(m.Groups[3].Value)));
            }
            return list;
        }

        /// <summary>
        /// Drops gravel from the fine mask inside RWG town tiles. Town streets
        /// are asphalt; the gravel prefabs paint there is sidewalks, lot
        /// outlines, yard paths and house floors, which A* otherwise takes for
        /// roads and routes through front yards.
        /// </summary>
        public static void StripTownGravel(byte[] fine, int w, int h, float metersPerPx,
            System.Collections.Generic.List<Vector2> tiles)
        {
            float halfW = w * metersPerPx * 0.5f, halfH = h * metersPerPx * 0.5f;
            int span = Mathf.RoundToInt(TownTileSize / metersPerPx);
            foreach (var t in tiles)
            {
                int x0 = Mathf.FloorToInt((t.x + halfW) / metersPerPx);
                int y0 = Mathf.FloorToInt((t.y + halfH) / metersPerPx);
                for (int y = Mathf.Max(0, y0); y < Mathf.Min(h, y0 + span); y++)
                {
                    int row = y * w;
                    for (int x = Mathf.Max(0, x0); x < Mathf.Min(w, x0 + span); x++)
                    {
                        if (fine[row + x] == FineGravel) fine[row + x] = FineNone;
                    }
                }
            }
        }

        /// <summary>Two-pass chessboard distance transform over road cells.</summary>
        void BuildEdgeDist()
        {
            int W = CellsX, H = CellsY;
            EdgeDist = new byte[W * H];
            for (int y = 0; y < H; y++)
            {
                for (int x = 0; x < W; x++)
                {
                    int i = y * W + x;
                    if (Cells[i] == CellOffRoad) { EdgeDist[i] = 0; continue; }
                    int d = 255;
                    if (x > 0) d = Mathf.Min(d, EdgeDist[i - 1] + 1); else d = 1;
                    if (y > 0)
                    {
                        d = Mathf.Min(d, EdgeDist[i - W] + 1);
                        if (x > 0) d = Mathf.Min(d, EdgeDist[i - W - 1] + 1);
                        if (x < W - 1) d = Mathf.Min(d, EdgeDist[i - W + 1] + 1);
                    }
                    else d = 1;
                    EdgeDist[i] = (byte)Mathf.Min(d, 255);
                }
            }
            for (int y = H - 1; y >= 0; y--)
            {
                for (int x = W - 1; x >= 0; x--)
                {
                    int i = y * W + x;
                    if (EdgeDist[i] == 0) continue;
                    int d = EdgeDist[i];
                    if (x < W - 1) d = Mathf.Min(d, EdgeDist[i + 1] + 1); else d = 1;
                    if (y < H - 1)
                    {
                        d = Mathf.Min(d, EdgeDist[i + W] + 1);
                        if (x < W - 1) d = Mathf.Min(d, EdgeDist[i + W + 1] + 1);
                        if (x > 0) d = Mathf.Min(d, EdgeDist[i + W - 1] + 1);
                    }
                    else d = 1;
                    EdgeDist[i] = (byte)d;
                }
            }
        }

        public bool WorldToCell(float wx, float wz, out int cx, out int cy)
        {
            cx = Mathf.FloorToInt((wx + WorldW * 0.5f) / CellSize);
            cy = Mathf.FloorToInt((wz + WorldH * 0.5f) / CellSize);
            return cx >= 0 && cy >= 0 && cx < CellsX && cy < CellsY;
        }

        public void ClampToCell(float wx, float wz, out int cx, out int cy)
        {
            WorldToCell(wx, wz, out cx, out cy);
            cx = Mathf.Clamp(cx, 0, CellsX - 1);
            cy = Mathf.Clamp(cy, 0, CellsY - 1);
        }

        public Vector3 CellToWorld(int cx, int cy)
        {
            return new Vector3(
                cx * CellSize - WorldW * 0.5f + CellSize * 0.5f,
                0f,
                cy * CellSize - WorldH * 0.5f + CellSize * 0.5f);
        }

        public byte CellAt(int cx, int cy)
        {
            return Cells[cy * CellsX + cx];
        }

        public bool IsRoadAt(float wx, float wz)
        {
            int cx, cy;
            return WorldToCell(wx, wz, out cx, out cy) && CellAt(cx, cy) != CellOffRoad;
        }

        /// <summary>Fine-resolution road test (splat pixel under the point).</summary>
        public bool IsFineRoad(float wx, float wz)
        {
            int fx = Mathf.FloorToInt((wx + WorldW * 0.5f) / FineMpp);
            int fy = Mathf.FloorToInt((wz + WorldH * 0.5f) / FineMpp);
            if (fx < 0 || fy < 0 || fx >= FineW || fy >= FineH) return false;
            return Fine[fy * FineW + fx] != 0;
        }
    }
}
