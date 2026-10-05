using System;
using System.Collections.Generic;
using System.IO;

namespace T3taAutopilot
{
    /// <summary>Game-side loading of the road splatmap for RoadNetwork.</summary>
    internal sealed partial class RoadNetwork
    {
        /// <summary>
        /// World folder of a locally generated/loaded world, or null (network
        /// client, or files missing). Main thread only.
        /// </summary>
        public static string FindWorldDir(World world)
        {
            var cp = world.ChunkCache?.ChunkProvider as ChunkProviderGenerateWorld;
            if (cp == null)
            {
                Log.Out("[t3taAutopilot] No local world files (network client) - road routing disabled");
                return null;
            }
            // worldLocation points at the world folder itself; FullPath is
            // ".../GeneratedWorlds/<WorldName>" (Folder alone is the parent).
            string dir = cp.worldLocation.FullPath;
            if (string.IsNullOrEmpty(dir) || !Directory.Exists(dir))
            {
                dir = Path.Combine(cp.worldLocation.Folder, cp.worldLocation.Name);
            }
            if (string.IsNullOrEmpty(dir) || !Directory.Exists(dir))
            {
                Log.Out("[t3taAutopilot] World folder not found - road routing disabled");
                return null;
            }
            return dir;
        }

        /// <summary>
        /// Reads the road splatmap from dir. Thread-safe (no engine calls
        /// besides Log): takes a few seconds for an 8k world, so run it off
        /// the main thread.
        /// </summary>
        /// <param name="prefabDirs">the game's Data/Prefabs (read on the main thread by the caller)</param>
        public static RoadNetwork LoadFromDir(string dir, AutopilotConfig cfg, IList<string> prefabDirs)
        {
            try
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                string file = Path.Combine(dir, "splat3_processed.png");
                PngRoadMask.Classify test = (r, g, b, a) =>
                    r > 127 ? FineAsphalt : g > 127 ? FineGravel : FineNone;
                string source = "splat3_processed R|G";
                if (!File.Exists(file))
                {
                    file = Path.Combine(dir, "splat3.png");
                    test = (r, g, b, a) => a > 127 ? FineAsphalt : FineNone;
                    source = "splat3 alpha (no town streets)";
                }
                if (!File.Exists(file))
                {
                    Log.Out("[t3taAutopilot] No road splatmap in " + dir + " - road routing disabled");
                    return null;
                }

                // splat3 maps are 1 m/px; downsample 2x -> 2 m fine mask
                const int factor = 2;
                byte[] mask = PngRoadMask.Load(file, factor, test, out int w, out int h);
                var towns = ReadTownTiles(Path.Combine(dir, "prefabs.xml"));
                StripTownGravel(mask, w, h, factor, towns);
                var net = Build(mask, w, h, factor, cfg.CellSize);
                net.TownTiles = towns;
                string terrain;
                try { terrain = net.AttachTerrain(dir); }
                catch (Exception te) { net.TerrainMul = null; terrain = "terrain load failed: " + te.Message; }
                try { terrain += "; " + net.AttachPrefabs(Path.Combine(dir, "prefabs.xml"), prefabDirs); }
                catch (Exception pe) { net.HazardMul = null; terrain += "; prefab load failed: " + pe.Message; }
                Log.Out("[t3taAutopilot] RoadNetwork built from " + source + ": " + net.CellsX + "x" + net.CellsY +
                    " cells (" + net.CellSize + "m), " + net.RoadCellCount + " road cells, " + towns.Count + " town tiles; " + terrain + "; " +
                    sw.ElapsedMilliseconds + " ms");
                return net;
            }
            catch (Exception e)
            {
                Log.Error("[t3taAutopilot] RoadNetwork load failed: " + e);
                return null;
            }
        }
    }
}
