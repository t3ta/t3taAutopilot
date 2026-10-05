using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using T3taAutopilot;
using T3taAutopilot.Old;
using UnityEngine;
using Color = System.Drawing.Color;
using Graphics = System.Drawing.Graphics;

namespace AutopilotSim
{
    /// <summary>
    /// Kinematic stand-in for EntityVehicle (defaults: vehicleTruck4x4).
    /// Steering follows the game's lastInputController law: the wheel angle
    /// moves toward input*SteerAngleMax at SteerRate*1.5*clamp(1-(v/10)^2,.15,1)
    /// deg/s - i.e. above ~9 m/s the wheel barely moves.
    /// </summary>
    sealed class SimVehicle
    {
        public float SteerAngleMax = 32f, SteerRate = 130f, VMax = 14f, VMaxNoTurbo = 10f, WheelBase = 2.7f;
        public float VelocityMax = 10f;   // game's drifting cap: turbo only while moveForward != 0
        public float HalfWidth = 1.2f, FrontZ = 2.4f;
        public float Accel = 3.5f, BrakeDecel = 7f, LatGrip = 6f;
        public Vector3 Pos, Fwd = Vector3.forward;
        public float V, Wheel;
        // rigid-body lag the kinematic model lacks: yaw rate eases toward the
        // steering-geometry rate with this time constant; steer input delay
        public static float YawTau = 0f, SteerDelay = 0f;
        float yawRate;
        readonly Queue<float> steerQ = new Queue<float>();

        public void Step(float throttle, float steer, bool brake, float dt)
        {
            float num17 = (V * 0.1f) * (V * 0.1f);
            float rate = SteerRate * Mathf.Clamp(1f - num17, 0.15f, 1f) * dt * 1.5f;
            if (SteerDelay > 0f)
            {
                steerQ.Enqueue(steer);
                steer = steerQ.Count > (int)(SteerDelay / dt) ? steerQ.Dequeue() : 0f;
            }
            Wheel = Mathf.MoveTowards(Wheel, steer * SteerAngleMax, rate);

            float capTarget = throttle != 0f ? VMax : VMaxNoTurbo;
            VelocityMax = Mathf.MoveTowards(VelocityMax, capTarget, (capTarget > VelocityMax ? 2.5f : 1.5f) * dt);

            if (brake) V = Mathf.MoveTowards(V, 0f, BrakeDecel * dt);
            else if (throttle > 0f) V += throttle * Accel * (1f - V / VMax) * dt - (throttle < 0.1f ? 0.4f * dt : 0f);
            else if (throttle < 0f) V = V > 0.5f ? Mathf.MoveTowards(V, 0f, 4f * dt) : Mathf.Max(-3f, V + throttle * 2f * dt);
            else V = Mathf.MoveTowards(V, 0f, 0.4f * dt);
            if (V > VelocityMax) V = VelocityMax;

            float curv = Mathf.Tan(Wheel * Mathf.Deg2Rad) / WheelBase;
            float maxCurv = LatGrip / Mathf.Max(V * V, 0.01f);          // tyres slide past this
            curv = Mathf.Clamp(curv, -maxCurv, maxCurv);
            float target = V * curv;
            yawRate = YawTau > 0f ? Mathf.Lerp(yawRate, target, Mathf.Clamp01(dt / YawTau)) : target;
            float dyaw = yawRate * dt;
            Fwd = Rot(Fwd, dyaw * Mathf.Rad2Deg);
            Pos += Fwd * V * dt;
        }

        public static Vector3 Rot(Vector3 v, float deg)
        {
            float r = deg * Mathf.Deg2Rad, c = Mathf.Cos(r), s = Mathf.Sin(r);
            // + = clockwise seen from above (Unity: +yaw turns toward +x)
            return new Vector3(v.x * c + v.z * s, 0f, -v.x * s + v.z * c);
        }
    }

    sealed class TrialResult
    {
        public bool Arrived, Stuck, TimedOut;
        public float Time, PathLen, OffRoadMeters, DrivenMeters, MaxXte, SumXte;
        public int Collisions, Samples, Detours, TurboToggles, Weaves, FenceHits, OffsetSwitches, QuickSwitches;
        public float LastWeaveSign, LastOffset, LastSwitchAt = -1f;
        public float BrakeTime, DriveTime;
        public List<Vector3> Track = new List<Vector3>();
        public List<Vector3> Fine = new List<Vector3>();
        public List<string> HitInfo = new List<string>();
        public List<Vector3> HitAt = new List<Vector3>();
        public string Why = "";
        public List<Vector3> LastPath;
        public HashSet<int> Blocked;
    }

    static class Program
    {
        static RoadNetwork net;
        static float OffRoadCost = 12f;
        static float FlatCost = 1.6f;
        static float GateHalf = 12f;
        static float WreckGap = 120f;
        static float ProbeMargin = 0.25f;
        const float Dt = 0.02f;
        const float Cruise = 14f;

        static int Main(string[] args)
        {
            if (Environment.GetEnvironmentVariable("AD_GYRO") != null)
            {
                GyroSim.Run();
                return 0;
            }
            string worldDir = args.Length > 0 ? args[0] :
                Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
                    "7DaysToDie", "GeneratedWorlds", "West Xuyofu Territory");
            int trials = args.Length > 1 ? int.Parse(args[1]) : 30;
            int seed = args.Length > 2 ? int.Parse(args[2]) : 1;
            string outDir = args.Length > 3 ? args[3] : Path.Combine(Path.GetTempPath(), "t3taAutopilot-sim");
            float offRoadCost = args.Length > 4 ? float.Parse(args[4]) : 12f;
            OffRoadCost = offRoadCost;
            if (args.Length > 5) MaskSource = args[5];
            if (args.Length > 6) GateHalf = float.Parse(args[6]);
            if (args.Length > 7) WreckGap = float.Parse(args[7]);
            Directory.CreateDirectory(outDir);
            float Env(string k, float d) { var v = Environment.GetEnvironmentVariable(k); return v != null ? float.Parse(v) : d; }
            Driver.AvoidSpeed = Env("AD_AVOID_SPEED", Driver.AvoidSpeed);
            FlatCost = Env("AD_FLAT_COST", FlatCost);
            Driver.AvoidHeadway = Env("AD_HEADWAY", Driver.AvoidHeadway);
            Driver.ObstacleDecel = Env("AD_OBS_DECEL", Driver.ObstacleDecel);
            ProbeMargin = Env("AD_MARGIN", ProbeMargin);
            SimVehicle.YawTau = Env("AD_YAW_TAU", 0f);
            SimVehicle.SteerDelay = Env("AD_STEER_DELAY", 0f);
            Driver.OffRoadOffsetCost = Env("AD_OFFROAD_OFFSET_COST", Driver.OffRoadOffsetCost);
            Driver.OffsetHoldSec = Env("AD_HOLD_SEC", Driver.OffsetHoldSec);
            PathFollower.MaxCornerCut = Env("AD_CORNER_CUT", PathFollower.MaxCornerCut);
            RoadNetwork.LotMul = Env("AD_LOT_MUL", RoadNetwork.LotMul);
            Driver.OffsetHoldDecel = Env("AD_HOLD_DECEL", Driver.OffsetHoldDecel);
            Driver.StraightNeedMax = Env("AD_STRAIGHT_NEED", Driver.StraightNeedMax);
            PathFollower.LookGain = Env("AD_LOOK_GAIN", PathFollower.LookGain);
            PathFollower.LookMax = Env("AD_LOOK_MAX", PathFollower.LookMax);

            var sw = System.Diagnostics.Stopwatch.StartNew();
            net = LoadNet(worldDir);
            Console.WriteLine($"net {net.CellsX}x{net.CellsY} road cells {net.RoadCellCount} ({sw.ElapsedMilliseconds} ms)");

            // AD_LANDAT="x,z,fromX,fromZ" : the landing site for one destination, and where a
            // no-strip landing goes instead (outside a trader compound / checkpoint)
            var landAt = Environment.GetEnvironmentVariable("AD_LANDAT");
            if (landAt != null)
            {
                var v = landAt.Split(',').Select(float.Parse).ToArray();
                var d = new Vector3(v[0], 0f, v[1]);
                bool ok = net.FindLandingSite(new Vector3(v[2], 0f, v[3]), d, 120f, out Vector3 site, out bool road);
                Console.WriteLine(ok ? $"strip: {(road ? "road" : "field")} ({site.x:F0},{site.z:F0})" : "strip: none within 120 m");
                Console.WriteLine($"roadblock at the destination: {net.RoadblockAt(d.x, d.z)}");
                var outside = net.OutsideRoadblocks(d, 80f);   // OutsideRoadblockSearch
                Console.WriteLine($"outside it: ({outside.x:F0},{outside.z:F0}), {Vector3.Distance(d, new Vector3(outside.x, 0, outside.z)):F0} m away, road={net.IsFineRoad(outside.x, outside.z)}");
                return 0;
            }

            // AD_LANDSITE=N : pick N random destinations, report the landing site chosen for each
            var siteSpec = Environment.GetEnvironmentVariable("AD_LANDSITE");
            if (siteSpec != null)
            {
                var srng = new System.Random(3);
                int roadN = 0, fieldN = 0, noneN = 0;
                for (int k = 0; k < int.Parse(siteSpec); k++)
                {
                    var d = new Vector3((float)(srng.NextDouble() - 0.5) * net.WorldW * 0.8f, 0f, (float)(srng.NextDouble() - 0.5) * net.WorldH * 0.8f);
                    var from = d + new Vector3(600f, 0f, 300f);
                    var sw2 = System.Diagnostics.Stopwatch.StartNew();
                    bool ok = net.FindLandingSite(from, d, 120f, out Vector3 site, out bool road);
                    if (!ok) noneN++; else if (road) roadN++; else fieldN++;
                    if (k < 12) Console.WriteLine($"dest ({d.x:F0},{d.z:F0}) -> " + (ok ? $"{(road ? "road" : "field")} ({site.x:F0},{site.z:F0}) {Vector3.Distance(new Vector3(d.x, 0, d.z), new Vector3(site.x, 0, site.z)):F0} m away" : "none") + $"  {sw2.ElapsedMilliseconds} ms");
                }
                Console.WriteLine($"road {roadN}  field {fieldN}  none {noneN}");
                return 0;
            }

            // AD_ROUTE="x1,z1,x2,z2;..." : dump planned paths (one per line, "x,z x,z ...") and exit
            var routeSpec = Environment.GetEnvironmentVariable("AD_ROUTE");
            if (routeSpec != null)
            {
                using var w = new StreamWriter(Path.Combine(outDir, "routes.txt"));
                foreach (var spec in routeSpec.Split(';'))
                {
                    var v = spec.Split(',').Select(float.Parse).ToArray();
                    var target = net.OutsideRoadblocks(new Vector3(v[2], 0, v[3]), 80f);
                    if (target.x != v[2] || target.z != v[3]) Console.WriteLine($"  destination {v[2]},{v[3]} is inside a roadblock prefab -> {target.x:F0},{target.z:F0}");
                    var rp = RoutePlanner.FindPath(net, new Vector3(v[0], 0, v[1]), target, 1f, offRoadCost, FlatCost);
                    w.WriteLine(rp == null ? "" : string.Join(" ", rp.Select(q => $"{q.x:F1},{q.z:F1}")));
                    if (rp != null) Console.WriteLine($"route {spec}: {rp.Zip(rp.Skip(1), (q1, q2) => Vector3.Distance(q1, q2)).Sum():F0} m, {rp.Count(q => net.RoadblockAt(q.x, q.z) != null) * RoutePlanner.Spacing:F0} m inside roadblock footprints, {rp.Count(q => !net.IsFineRoad(q.x, q.z)) * RoutePlanner.Spacing:F0} m off road, {rp.Count(q => InLot(q)) * RoutePlanner.Spacing:F0} m through POI lots");
                }
                return 0;
            }

            // AD_TRIP="x1,z1,x2,z2;..." : drive these trips (no obstacles) and report
            // how far the car ran off the asphalt where the route itself was on it
            // (corner cutting), then exit
            var tripSpec = Environment.GetEnvironmentVariable("AD_TRIP");
            if (tripSpec != null)
            {
                foreach (var spec in tripSpec.Split(';'))
                {
                    var v = spec.Split(',').Select(float.Parse).ToArray();
                    Vector3 a = new Vector3(v[0], 0, v[1]), b = new Vector3(v[2], 0, v[3]);
                    var path = RoutePlanner.FindPath(net, a, b, 1f, offRoadCost, FlatCost);
                    if (path == null) { Console.WriteLine($"trip {spec}: no route"); continue; }
                    var r = RunNew(path, path[0], b, 0f, null);
                    float cut = 0f;
                    for (int i = 1; i < r.Fine.Count; i++)
                    {
                        Vector3 q = r.Fine[i];
                        if (net.IsFineRoad(q.x, q.z)) continue;
                        float best = float.MaxValue; Vector3 near = q;
                        foreach (var pp in path) { float d = (pp.x - q.x) * (pp.x - q.x) + (pp.z - q.z) * (pp.z - q.z); if (d < best) { best = d; near = pp; } }
                        if (net.IsFineRoad(near.x, near.z)) cut += Vector3.Distance(r.Fine[i - 1], q);
                    }
                    Console.WriteLine($"trip {spec}: {Fmt(r)}  off the asphalt beside an on-road route {cut:F0} m");
                }
                return 0;
            }

            CheckOrientation(worldDir);

            var rng = new System.Random(seed);
            var roadPts = SampleRoadPoints(rng, 20000);
            var newAgg = new List<TrialResult>();
            var newObsAgg = new List<TrialResult>();
            var gateAgg = new List<TrialResult>();
            var oldAgg = new List<TrialResult>();
            int done = 0, attempts = 0, zooms = 0, gz = 0;
            while (done < trials && attempts < trials * 20)
            {
                attempts++;
                Vector3 a = roadPts[rng.Next(roadPts.Count)], b = roadPts[rng.Next(roadPts.Count)];
                float d = Vector3.Distance(a, b);
                if (d < 700f || d > 2200f) continue;

                sw.Restart();
                var path = RoutePlanner.FindPath(net, a, b, 1f, offRoadCost, FlatCost);
                long planMs = sw.ElapsedMilliseconds;
                var oldWps = OldRoutePlanner.FindPath(net, a, b, 1f, 4f);
                if (path == null || oldWps == null) continue;

                float startYaw = (float)(rng.NextDouble() * 60 - 30);
                var obstacles = PlaceObstacles(path, new System.Random(seed * 7919 + attempts));   // per-trip rng: configs compare on identical trips
                if (Fences) obstacles.AddRange(PlaceFences(path, new System.Random(seed * 104729 + attempts)));

                var rNew = RunNew(path, a, b, startYaw, null);
                var rNewObs = RunNew(path, a, b, startYaw, obstacles);
                var gate = PlaceGate(path);
                var rGate = RunNew(path, a, b, startYaw, gate);
                gateAgg.Add(rGate);
                var rOld = RunOld(oldWps, a, b, startYaw);
                newAgg.Add(rNew); newObsAgg.Add(rNewObs); oldAgg.Add(rOld);
                Console.WriteLine($"#{done,2} {d,5:F0}m plan {planMs,4}ms | OLD {Fmt(rOld)} | NEW {Fmt(rNew)} | NEW+wrecks({obstacles.Count}) {Fmt(rNewObs)} | GATE {Fmt(rGate)}");
                if (!rGate.Arrived && gz++ < 4) { Render(Path.Combine(outDir, $"gate{gz:D2}.png"), path, oldWps, rOld, rNew, rGate, gate); RenderZoom(Path.Combine(outDir, $"gatezoom{gz:D2}.png"), new Vector3(gate[gate.Count / 2].x, 0, gate[gate.Count / 2].y), rGate.LastPath ?? path, rGate, gate); }
                if (rNewObs.HitAt.Count > 0 && zooms < 8)
                {
                    for (int h = 0; h < rNewObs.HitAt.Count && zooms < 8; h++, zooms++)
                    {
                        Console.WriteLine($"   zoom{zooms}: {rNewObs.HitInfo[h]}");
                        RenderZoom(Path.Combine(outDir, $"zoom{zooms:D2}.png"), rNewObs.HitAt[h], path, rNewObs, obstacles);
                    }
                }
                if (done < 6)
                {
                    Render(Path.Combine(outDir, $"trial{done:D2}.png"), path, oldWps, rOld, rNew, rNewObs, obstacles);
                }
                done++;
            }
            Console.WriteLine();
            Summary("OLD         ", oldAgg);
            Summary("NEW         ", newAgg);
            Summary("NEW+wrecks  ", newObsAgg);
            Summary("NEW+gate    ", gateAgg);
            Console.WriteLine("images: " + outDir);
            return 0;
        }

        /// <summary>Inside a POI footprint.</summary>
        static bool InLot(Vector3 q)
        {
            foreach (var r in net.Lots) if (r.Contains(new Vector2(q.x, q.z))) return true;
            return false;
        }

        static string Fmt(TrialResult r)
        {
            string st = r.Arrived ? "ok " : r.Stuck ? "STK" : "TMO";
            return $"{st} {r.Time,5:F0}s det {r.Detours} offroad {100f * r.OffRoadMeters / Mathf.Max(1f, r.DrivenMeters),5:F1}% xteMax {r.MaxXte,5:F1} hit {r.Collisions}";
        }

        static void Summary(string label, List<TrialResult> rs)
        {
            int ok = rs.Count(r => r.Arrived);
            float off = rs.Sum(r => r.OffRoadMeters) / Mathf.Max(1f, rs.Sum(r => r.DrivenMeters));
            float avgT = rs.Where(r => r.Arrived).Select(r => r.Time).DefaultIfEmpty(0).Average();
            float avgV = rs.Where(r => r.Arrived).Select(r => r.DrivenMeters / r.Time).DefaultIfEmpty(0).Average();
            int hits = rs.Sum(r => r.Collisions);
            float xte95 = Percentile(rs.Select(r => r.MaxXte).ToList(), 0.95f);
            float totT = rs.Where(r => r.Arrived).Sum(r => r.Time), totD = rs.Where(r => r.Arrived).Sum(r => r.DrivenMeters);
            float brakePct = 100f * rs.Sum(r => r.BrakeTime) / Mathf.Max(0.001f, rs.Sum(r => r.DriveTime));
            float togglesPerKm = rs.Sum(r => r.TurboToggles) / Mathf.Max(0.001f, rs.Sum(r => r.DrivenMeters) / 1000f);
            float weavesPerKm = rs.Sum(r => r.Weaves) / Mathf.Max(0.001f, rs.Sum(r => r.DrivenMeters) / 1000f);
            Console.WriteLine($"{label} steer reversals/km {weavesPerKm,5:F1}  fence hits {rs.Sum(r => r.FenceHits)}  detours {rs.Sum(r => r.Detours)}  stuck {rs.Count(r => r.Stuck)}  offset switches {rs.Sum(r => r.OffsetSwitches)} (<=0.2s {rs.Sum(r => r.QuickSwitches)})");
            Console.WriteLine($"{label} brake {brakePct,4:F1}% sprint toggles/km {togglesPerKm,5:F1}  arrived {ok}/{rs.Count}  off-road {100 * off:F1}% of distance  collisions {hits}  avg {avgV:F1} m/s  maxXTE p95 {xte95:F1} m  total {totT / 60f:F1} min {totD / 1000f:F1} km");
        }

        static float Percentile(List<float> v, float p)
        {
            if (v.Count == 0) return 0;
            v.Sort();
            return v[Math.Min(v.Count - 1, (int)(p * v.Count))];
        }

        // ------------------------------------------------------------------
        static string MaskSource = "processed";

        /// <summary>Same source/predicate/downsampling as RoadNetwork.LoadFromDir.</summary>
        static RoadNetwork LoadNet(string dir)
        {
            bool processed = MaskSource == "processed";
            string file = Path.Combine(dir, processed ? "splat3_processed.png" : "splat3.png");
            PngRoadMask.Classify test = processed
                ? (r, g, b, a) => r > 127 ? RoadNetwork.FineAsphalt : g > 127 ? RoadNetwork.FineGravel : RoadNetwork.FineNone
                : (r, g, b, a) => a > 127 ? RoadNetwork.FineAsphalt : RoadNetwork.FineNone;
            var m = PngRoadMask.Load(file, 2, test, out int mw, out int mh);
            bool towns = Environment.GetEnvironmentVariable("AD_NO_TOWNS") == null;
            var tiles = RoadNetwork.ReadTownTiles(Path.Combine(dir, "prefabs.xml"));
            if (towns) RoadNetwork.StripTownGravel(m, mw, mh, 2f, tiles);
            var cellEnv = Environment.GetEnvironmentVariable("AD_CELL_SIZE");
            var rn = RoadNetwork.Build(m, mw, mh, 2f, cellEnv != null ? float.Parse(cellEnv) : 4f);
            if (towns) rn.TownTiles = tiles;
            var rbMul = Environment.GetEnvironmentVariable("AD_ROADBLOCK_MUL");
            if (rbMul != null) RoadNetwork.RoadblockMul = float.Parse(rbMul);
            if (Environment.GetEnvironmentVariable("AD_NO_PREFABS") == null)
            {
                string game = Environment.GetEnvironmentVariable("AD_GAME_DIR") ??
                    @"C:\Program Files (x86)\Steam\steamapps\common\7 Days To Die";
                Console.WriteLine(rn.AttachPrefabs(Path.Combine(dir, "prefabs.xml"), new[] { Path.Combine(game, "Data", "Prefabs") }));
            }
            if (Environment.GetEnvironmentVariable("AD_NO_TERRAIN") == null) Console.WriteLine(rn.AttachTerrain(dir));
            var dump = Environment.GetEnvironmentVariable("AD_DUMP_TERRAIN");
            if (dump != null && rn.TerrainMul != null)
            {
                int k = 2, W = rn.CellsX / k;
                using var img = new Bitmap(W, W);
                for (int y = 0; y < W; y++) for (int x = 0; x < W; x++)
                {
                    int i = (rn.CellsY - 1 - y * k) * rn.CellsX + x * k;   // north up
                    float tm = rn.TerrainMul[i] * 0.25f;
                    Color c = rn.Cells[i] != RoadNetwork.CellOffRoad ? Color.FromArgb(60, 60, 60)
                        : tm >= 60f ? Color.FromArgb(40, 90, 220)       // cliff or water (see below)
                        : tm >= 20f ? Color.FromArgb(220, 40, 40)       // building
                        : tm >= 7f ? Color.FromArgb(150, 110, 70)       // steep
                        : tm >= 2.4f ? Color.FromArgb(200, 190, 120)    // gentle / forest
                        : Color.FromArgb(140, 210, 120);               // flat open
                    img.SetPixel(x, y, c);
                }
                img.Save(dump);
            }
            return rn;
        }

        /// <summary>
        /// Road-side prefab parts must sit next to roads. Compare the
        /// mapping in RoadNetwork against its z-mirror.
        /// </summary>
        static void CheckOrientation(string dir)
        {
            string xml = File.ReadAllText(Path.Combine(dir, "prefabs.xml"));
            var re = new Regex("name=\"(part_driveway[^\"]*|part_highway_transition[^\"]*)\" position=\"(-?\\d+),(-?\\d+),(-?\\d+)\"");
            int n = 0, hitA = 0, hitB = 0;
            foreach (Match m in re.Matches(xml))
            {
                float x = float.Parse(m.Groups[2].Value), z = float.Parse(m.Groups[4].Value);
                n++;
                if (NearRoad(x, z, 12f)) hitA++;
                if (NearRoad(x, -z, 12f)) hitB++;
            }
            Console.WriteLine($"orientation check: {n} roadside prefabs within 12m of road: current mapping {hitA} ({100f * hitA / Math.Max(1, n):F0}%), z-mirrored {hitB} ({100f * hitB / Math.Max(1, n):F0}%)");
        }

        static bool NearRoad(float x, float z, float r)
        {
            for (float dx = -r; dx <= r; dx += 2f)
                for (float dz = -r; dz <= r; dz += 2f)
                    if (net.IsFineRoad(x + dx, z + dz)) return true;
            return false;
        }

        static List<Vector3> SampleRoadPoints(System.Random rng, int count)
        {
            var list = new List<Vector3>();
            int guard = 0;
            while (list.Count < count && guard++ < count * 400)
            {
                float x = (float)(rng.NextDouble() - 0.5) * net.WorldW * 0.9f;
                float z = (float)(rng.NextDouble() - 0.5) * net.WorldH * 0.9f;
                if (net.IsFineRoad(x, z)) list.Add(new Vector3(x, 0, z));
            }
            return list;
        }

        /// <summary>A closed gate: a wall across the route at mid-way, 2*GateHalf wide.</summary>
        static List<Vector4> PlaceGate(List<Vector3> path)
        {
            int i = path.Count / 2;
            Vector3 t = (path[Math.Min(i + 2, path.Count - 1)] - path[Math.Max(i - 2, 0)]).normalized;
            Vector3 n = new Vector3(t.z, 0, -t.x);
            var obs = new List<Vector4>();
            for (float o = -GateHalf; o <= GateHalf; o += 1.2f)
            {
                Vector3 c = path[i] + n * o;
                obs.Add(new Vector4(c.x, c.z, 0.8f, 0));
            }
            return obs;
        }

        static bool Fences = Environment.GetEnvironmentVariable("AD_FENCES") != null;

        /// <summary>
        /// Roadside fences: posts (r 0.3 m, every 1 m) just outside the road
        /// edge along random stretches - what an avoidance swerve runs into
        /// in-game (yards, farms, guard rails).
        /// </summary>
        static List<Vector4> PlaceFences(List<Vector3> path, System.Random rng)
        {
            var obs = new List<Vector4>();
            var seen = new HashSet<long>();
            float s = 0f, runEnd = -1f, next = 20f;
            int sides = 0;
            for (int i = 1; i < path.Count - 1; i++)
            {
                s += Vector3.Distance(path[i - 1], path[i]);
                if (s > runEnd && s >= next)
                {
                    runEnd = s + 30f + (float)rng.NextDouble() * 90f;
                    next = runEnd + 20f + (float)rng.NextDouble() * 80f;
                    sides = 1 + rng.Next(3);   // 1 = left, 2 = right, 3 = both
                }
                if (s > runEnd || !net.IsFineRoad(path[i].x, path[i].z)) continue;
                Vector3 t = (path[i + 1] - path[i - 1]); t.y = 0; t.Normalize();
                Vector3 n = new Vector3(t.z, 0, -t.x);
                for (int side = -1; side <= 1; side += 2)
                {
                    if ((sides & (side < 0 ? 1 : 2)) == 0) continue;
                    float e = 0f;
                    while (e < 14f && net.IsFineRoad(path[i].x + n.x * side * e, path[i].z + n.z * side * e)) e += 0.5f;
                    if (e >= 14f) continue;   // plaza/junction: no edge
                    for (float k = 0f; k < 2f; k += 1f)   // path points are 2 m apart
                    {
                        Vector3 c = path[i] + t * k + n * side * (e + 0.5f);
                        long key = ((long)Mathf.FloorToInt(c.x) << 32) ^ (uint)Mathf.FloorToInt(c.z);
                        if (!seen.Add(key)) continue;
                        if (net.IsFineRoad(c.x, c.z)) continue;   // never on the road itself
                        obs.Add(new Vector4(c.x, c.z, 0.3f, 1));
                    }
                }
            }
            return obs;
        }

        static List<Vector4> PlaceObstacles(List<Vector3> path, System.Random rng)
        {
            // wrecks on the road: (x, z, radius) at random along the route
            var obs = new List<Vector4>();
            float s = 0f, next = 60f + (float)rng.NextDouble() * 100f;
            for (int i = 1; i < path.Count - 15; i++)
            {
                s += Vector3.Distance(path[i - 1], path[i]);
                if (s < next) continue;
                next = s + WreckGap + (float)rng.NextDouble() * WreckGap * 1.25f;
                if (!net.IsFineRoad(path[i].x, path[i].z)) continue;
                Vector3 t = (path[Math.Min(i + 2, path.Count - 1)] - path[i - 1]).normalized;
                Vector3 n = new Vector3(t.z, 0, -t.x);
                Vector3 c = path[i] + n * (float)(rng.NextDouble() * 3.0 - 1.5);
                obs.Add(new Vector4(c.x, c.z, 1.3f, 0));
            }
            return obs;
        }

        // ------------------------------------------------------------------
        // NEW controller: the mod's own Driver class (same code as in-game)
        // ------------------------------------------------------------------
        static TrialResult RunNew(List<Vector3> path, Vector3 start, Vector3 dest, float startYaw, List<Vector4> obstacles)
        {
            var blocked = new HashSet<int>();
            var main = new PathFollower();
            main.SetPath(path, start, Cruise, null);
            int detours = 0;
            bool lastTurbo = false;
            var veh = new SimVehicle { Pos = start };
            Vector3 dir0 = (path[Math.Min(3, path.Count - 1)] - path[0]); dir0.y = 0;
            veh.Fwd = SimVehicle.Rot(dir0.sqrMagnitude > 0.01f ? dir0.normalized : Vector3.forward, startYaw);
            var drv = new Driver { Cruise = Cruise, WheelBase = veh.WheelBase, SteerMax = veh.SteerAngleMax,
                HalfWidth = veh.HalfWidth, IsRoad = net.IsFineRoad };
            var hitPts = new List<Vector3>();
            drv.SetPath(path, start, p => net.SpeedCapAt(p.x, p.z));
            var r = new TrialResult { PathLen = drv.Follower.Length };
            r.Blocked = blocked;
            var free = new float[Driver.ProbeAngles.Length];
            float limit = drv.Follower.Length / 2f + 120f;
            var touching = new HashSet<Vector4>();
            var near = new List<Vector4>();
            float nearAt = 0f;

            for (float t = 0; ; t += Dt)
            {
                if (t > limit) { r.TimedOut = true; r.Why = "timeout"; r.HitAt.Add(veh.Pos); r.HitInfo.Add($"TIMEOUT v={veh.V:F1} want={drv.Desired:F1} off={drv.Offset} boxed={drv.Boxed} xte={drv.Follower.CrossTrack:F1} prog={drv.Follower.Progress:F0}/{drv.Follower.Length:F0}"); break; }
                float speed = Mathf.Abs(veh.V);
                float range = Driver.ProbeRange(speed);
                if (obstacles != null && t >= nearAt)
                {
                    nearAt = t + 0.25f;
                    near.Clear();
                    foreach (var ob in obstacles)
                    {
                        float dx = ob.x - veh.Pos.x, dz = ob.y - veh.Pos.z;
                        if (dx * dx + dz * dz < 50f * 50f) near.Add(ob);
                    }
                }
                Probe(veh, obstacles != null ? near : null, range, free, drv, t);
                main.Update(veh.Pos);
                var o = drv.Step(veh.Pos, veh.Fwd, veh.V, speed, dest, free, range, t, Dt);
                if (o.Arrived) { r.Arrived = true; r.Time = t; break; }
                if (o.StuckEvent && drv.StuckCount >= 2 && detours < 8)
                {
                    RoutePlanner.MarkBlockedAhead(net, drv.Follower, veh.Pos, veh.Fwd, 0f, 14f, 7f, blocked);
                    hitPts.Clear();
                    drv.RecentHits(hitPts);
                    RoutePlanner.MarkBlockedPoints(net, hitPts, veh.HalfWidth + 1.5f, veh.Pos, veh.Fwd, blocked);
                    detours++;
                    drv.ResetStuck();
                    var np = RoutePlanner.Bypass(net, main, veh.Pos, veh.Fwd, blocked);
                    if (np != null) { drv.SetPath(np, veh.Pos, p => net.SpeedCapAt(p.x, p.z)); r.LastPath = np; }
                    r.Detours++;
                }
                if (o.StuckEvent && drv.StuckCount > 4) { r.Stuck = true; r.Why = "stuck"; r.Time = t; break; }
                Vector3 before = veh.Pos;
                r.DriveTime += Dt;
                if (o.Brake && veh.V > 1f) r.BrakeTime += Dt;
                bool turbo = o.Throttle != 0f;
                if (veh.V > 1f && turbo != lastTurbo) r.TurboToggles++;
                lastTurbo = turbo;
                if (veh.V > 8f && Mathf.Abs(veh.Wheel) > 1.5f && Mathf.Sign(veh.Wheel) != r.LastWeaveSign)
                {
                    r.Weaves++;
                    r.LastWeaveSign = Mathf.Sign(veh.Wheel);
                }
                if (drv.Offset != r.LastOffset)
                {
                    r.OffsetSwitches++;
                    if (r.LastSwitchAt >= 0f && t - r.LastSwitchAt <= 0.2f) { r.QuickSwitches++; if (Environment.GetEnvironmentVariable("AD_DBG_SW") != null) Console.WriteLine($"SW {r.LastOffset}->{drv.Offset} v={veh.V:F1} rev={drv.Reversing} boxed={drv.Boxed} obs={(obstacles==null?-1:obstacles.Count)} stuckN={drv.StuckCount}"); }
                    r.LastSwitchAt = t;
                    r.LastOffset = drv.Offset;
                }
                veh.Step(o.Throttle, o.Steer, o.Brake, Dt);
                drv.Follower.Update(veh.Pos);
                Account(r, before, veh.Pos, drv.Follower.CrossTrack, obstacles != null ? near : null, veh, touching);
            }
            return r;
        }

        static void Probe(SimVehicle veh, List<Vector4> obs, float range, float[] free, Driver drv, float now)
        {
            Vector3 origin = veh.Pos + veh.Fwd * (veh.FrontZ - 0.5f);
            for (int i = 0; i < Driver.ProbeAngles.Length; i++)
            {
                float reach = Driver.ProbeReach(i, range);
                Vector3 dir = SimVehicle.Rot(veh.Fwd, Driver.ProbeAngles[i]);
                float best = reach;
                Vector3 contact = Vector3.zero;
                if (obs != null)
                {
                    foreach (var o in obs)
                    {
                        float wx = o.x - origin.x, wz = o.y - origin.z;
                        float along = wx * dir.x + wz * dir.z;
                        if (along <= 0f) continue;
                        float lat = Mathf.Abs(wx * dir.z - wz * dir.x);
                        float rr = o.z + veh.HalfWidth + ProbeMargin;   // AutopilotController.ProbeSideMargin
                        if (lat >= rr) continue;
                        float d = along - Mathf.Sqrt(rr * rr - lat * lat);
                        if (d > 0f && d < best)
                        {
                            best = d;
                            // touch point on the wreck's outline nearest the sweep axis
                            Vector3 foot = origin + dir * along;
                            Vector3 toFoot = new Vector3(foot.x - o.x, 0f, foot.z - o.y);
                            if (toFoot.sqrMagnitude < 1e-4f) toFoot = -dir;
                            contact = new Vector3(o.x, 0f, o.y) + toFoot.normalized * o.z;
                        }
                    }
                }
                free[i] = best >= reach ? range : best;
                if (best < reach && drv != null) drv.AddObstaclePoint(contact, now);
            }
        }

        // ------------------------------------------------------------------
        // OLD controller (pre-rewrite): sparse waypoints, steer = err/max,
        // speed from distance-to-destination only. No obstacles.
        // ------------------------------------------------------------------
        static TrialResult RunOld(List<Vector3> wps, Vector3 start, Vector3 dest, float startYaw)
        {
            var veh = new SimVehicle { Pos = start };
            Vector3 dir0 = wps[0] - start; dir0.y = 0;
            veh.Fwd = SimVehicle.Rot(dir0.sqrMagnitude > 0.01f ? dir0.normalized : Vector3.forward, startYaw);
            var pathDense = new List<Vector3> { start };
            pathDense.AddRange(wps);
            var ref_ = new PathFollower();   // only to measure cross-track vs the old route
            ref_.SetPath(RoutePlanner.Resample(pathDense, 2f), start, Cruise, null);
            var r = new TrialResult { PathLen = ref_.Length };
            bool lastTurbo = false;
            int idx = 0; float stuckT = 0f; int stuckCount = 0; float reverseUntil = -1f, lastSteer = 0f;
            float limit = ref_.Length / 2f + 120f;
            var touching = new HashSet<Vector4>();
            for (float t = 0; ; t += Dt)
            {
                if (t > limit) { r.TimedOut = true; break; }
                ref_.Update(veh.Pos);
                float dist = HDist(veh.Pos, dest);
                if (dist < 7f && Mathf.Abs(veh.V) < 0.6f) { r.Arrived = true; r.Time = t; break; }
                while (idx < wps.Count - 1)
                {
                    float d = HDist(veh.Pos, wps[idx]);
                    if (d < 9f) { idx++; continue; }
                    Vector3 to = wps[idx] - veh.Pos; to.y = 0;
                    if (Vector3.Dot(veh.Fwd, to.normalized) < -0.5f && d < 30f) { idx++; continue; }
                    break;
                }
                float throttle = 0f, steer; bool brake = false;
                if (t < reverseUntil) { throttle = -0.7f; steer = -lastSteer; }
                else if (dist < 7f) { brake = true; steer = 0; }
                else
                {
                    Vector3 to = wps[idx] - veh.Pos; to.y = 0;
                    float err = Vector3.SignedAngle(veh.Fwd, to, Vector3.up);
                    steer = Mathf.Clamp(err / veh.SteerAngleMax, -1f, 1f);
                    float desired = Mathf.Clamp(dist * 0.3f, 2.5f, Cruise);
                    if (Mathf.Abs(err) > 55f) desired = Mathf.Min(desired, 2.2f);
                    else if (Mathf.Abs(err) > 25f) desired = Mathf.Min(desired, 6f);
                    if (veh.V > desired + 2.5f) brake = true;
                    else
                    {
                        throttle = Mathf.Clamp((desired - veh.V) * 0.45f, -0.5f, 1f);
                        if (throttle < 0f && veh.V < 0.5f) { throttle = 0; brake = true; }
                    }
                    stuckT = (throttle > 0.05f && veh.V < 0.4f) ? stuckT + Dt : 0f;
                    if (stuckT > 3f)
                    {
                        if (++stuckCount > 4) { r.Stuck = true; r.Time = t; break; }
                        reverseUntil = t + 1.7f; stuckT = 0; lastSteer = Mathf.Abs(steer) > 0.1f ? steer : 0.8f;
                    }
                    else lastSteer = steer;
                }
                Vector3 before = veh.Pos;
                if (veh.V > 1f && (throttle != 0f) != lastTurbo) r.TurboToggles++;
                lastTurbo = throttle != 0f;
                veh.Step(throttle, steer, brake, Dt);
                Account(r, before, veh.Pos, ref_.CrossTrack, null, veh, touching);
            }
            return r;
        }

        static void Account(TrialResult r, Vector3 a, Vector3 b, float xte, List<Vector4> obs, SimVehicle veh, HashSet<Vector4> touching)
        {
            float step = HDist(a, b);
            r.DrivenMeters += step;
            // off-road = any corner of the footprint off the road mask
            Vector3 side = new Vector3(veh.Fwd.z, 0, -veh.Fwd.x) * (veh.HalfWidth * 0.8f);
            bool on = net.IsFineRoad(b.x + side.x, b.z + side.z) && net.IsFineRoad(b.x - side.x, b.z - side.z);
            if (!on) r.OffRoadMeters += step;
            r.MaxXte = Mathf.Max(r.MaxXte, xte);
            r.SumXte += xte; r.Samples++;
            if (r.Samples % 25 == 0) r.Track.Add(b);
            if (r.Samples % 3 == 0) r.Fine.Add(b);
            if (obs != null)
            {
                for (int i = 0; i < obs.Count; i++)
                {
                    var o = obs[i];
                    bool hit = false;
                    for (float z = -1.5f; z <= 1.5f; z += 1.5f)
                    {
                        Vector3 c = b + veh.Fwd * z;
                        float dx = c.x - o.x, dz = c.z - o.y;
                        if (dx * dx + dz * dz < (o.z + veh.HalfWidth * 0.9f) * (o.z + veh.HalfWidth * 0.9f)) hit = true;
                    }
                    if (hit && touching.Add(o))
                    {
                        r.Collisions++;
                        if (o.w == 1f) r.FenceHits++;
                        r.HitAt.Add(new Vector3(o.x, 0, o.y));
                        r.HitInfo.Add($"v={veh.V:F1} wheel={veh.Wheel:F0} xte={xte:F1}");
                    }
                    if (!hit) touching.Remove(o);
                }
            }
        }

        static Color FineColor(float wx, float wz)
        {
            int fx = (int)Math.Floor((wx + net.WorldW * 0.5f) / net.FineMpp);
            int fy = (int)Math.Floor((wz + net.WorldH * 0.5f) / net.FineMpp);
            byte c = (fx < 0 || fy < 0 || fx >= net.FineW || fy >= net.FineH) ? (byte)0 : net.Fine[fy * net.FineW + fx];
            if (c == RoadNetwork.FineAsphalt) return Color.FromArgb(140, 140, 140);
            if (c == RoadNetwork.FineGravel) return Color.FromArgb(205, 185, 150);
            if (net.TerrainMul != null && net.WorldToCell(wx, wz, out int tcx, out int tcy))
            {
                float tm = net.TerrainMul[tcy * net.CellsX + tcx] * 0.25f;
                return tm >= 60f ? Color.FromArgb(120, 160, 230) : tm >= 20f ? Color.FromArgb(240, 150, 150)
                    : tm >= 7f ? Color.FromArgb(190, 165, 135) : tm >= 2.4f ? Color.FromArgb(215, 225, 175)
                    : Color.FromArgb(235, 245, 225);
            }
            return Color.FromArgb(235, 240, 225);
        }

        static float HDist(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x, dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }

        // ------------------------------------------------------------------
        static void RenderZoom(string file, Vector3 c, List<Vector3> path, TrialResult r, List<Vector4> obs)
        {
            const float R = 45f, scale = 8f;
            int W = (int)(2 * R * scale);
            using var bmp = new Bitmap(W, W);
            float minX = c.x - R, maxZ = c.z + R;
            for (int y = 0; y < W; y++)
                for (int x = 0; x < W; x++)
                    bmp.SetPixel(x, y, net.IsFineRoad(minX + x / scale, maxZ - y / scale) ? Color.FromArgb(150, 150, 150) : Color.FromArgb(235, 240, 225));
            using var g = Graphics.FromImage(bmp);
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            PointF P(Vector3 v) => new PointF((v.x - minX) * scale, (maxZ - v.z) * scale);
            var near = path.Where(p => HDist(p, c) < R * 1.5f).ToList();
            if (near.Count > 1) g.DrawLines(new Pen(Color.Blue, 2), near.Select(P).ToArray());
            if (r.Blocked != null)
                foreach (int ci in r.Blocked)
                {
                    var cc = P(net.CellToWorld(ci % net.CellsX, ci / net.CellsX));
                    float hs = net.CellSize * scale * 0.5f;
                    g.FillRectangle(new SolidBrush(Color.FromArgb(90, 255, 0, 0)), cc.X - hs, cc.Y - hs, 2 * hs, 2 * hs);
                }
            var tr = r.Fine.Where(p => HDist(p, c) < R * 1.5f).ToList();
            foreach (var p in tr) { var q = P(p); g.FillEllipse(Brushes.Green, q.X - 2, q.Y - 2, 4, 4); }
            foreach (var o in obs)
            {
                var q = P(new Vector3(o.x, 0, o.y));
                float rr = o.z * scale;
                g.FillEllipse(Brushes.Black, q.X - rr, q.Y - rr, 2 * rr, 2 * rr);
            }
            bmp.Save(file, ImageFormat.Png);
        }

        static void Render(string file, List<Vector3> path, List<Vector3> oldWps, TrialResult rOld,
            TrialResult rNew, TrialResult rObs, List<Vector4> obs)
        {
            var all = path.Concat(oldWps).Concat(rOld.Track).ToList();
            float minX = all.Min(p => p.x) - 40, maxX = all.Max(p => p.x) + 40;
            float minZ = all.Min(p => p.z) - 40, maxZ = all.Max(p => p.z) + 40;
            float scale = Mathf.Min(1400f / (maxX - minX), 1400f / (maxZ - minZ));
            scale = Mathf.Min(scale, 3f);
            int W = (int)((maxX - minX) * scale), H = (int)((maxZ - minZ) * scale);
            using var bmp = new Bitmap(W, H);
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                {
                    float wx = minX + x / scale, wz = maxZ - y / scale;
                    bmp.SetPixel(x, y, FineColor(wx, wz));
                }
            using var g = Graphics.FromImage(bmp);
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            PointF P(Vector3 v) => new PointF((v.x - minX) * scale, (maxZ - v.z) * scale);
            void Line(List<Vector3> pts, Color c, float w)
            {
                if (pts.Count < 2) return;
                using var pen = new Pen(c, w);
                g.DrawLines(pen, pts.Select(P).ToArray());
            }
            var oldLine = new List<Vector3> { path[0] }; oldLine.AddRange(oldWps);
            Line(oldLine, Color.FromArgb(160, 255, 150, 0), 2f);
            Line(rOld.Track, Color.FromArgb(220, 220, 0, 0), 2f);
            Line(path, Color.FromArgb(160, 0, 90, 255), 2f);
            Line(rObs.Track, Color.FromArgb(230, 0, 150, 0), 2f);
            foreach (var o in obs)
            {
                var c = P(new Vector3(o.x, 0, o.y));
                float rr = Mathf.Max(3f, o.z * scale);
                g.FillEllipse(Brushes.Black, c.X - rr, c.Y - rr, 2 * rr, 2 * rr);
            }
            using var font = new Font("Consolas", 12);
            g.DrawString("orange=old waypoints  red=old drive  blue=new path  green=new drive (black=wrecks)", font, Brushes.Black, 5, 5);
            bmp.Save(file, ImageFormat.Png);
        }
    }
}
