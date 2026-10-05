using System;
using System.Collections.Generic;
using T3taAutopilot;
using UnityEngine;

namespace AutopilotSim
{
    /// <summary>Deterministic regressions for the safety issues, using the mod's own sources.</summary>
    static class RegressionTests
    {
        public static int Run()
        {
            var cases = new Action[]
            {
                TaxiStopsAtWall, TaxiBrakesBeforeWall, TaxiMovesOnClearGround,
                TaxiStopsWithPusherCoasting,
                TakeoffStillStopsInWalledYard, RetargetLeavesLongLandingSafely,
                RetargetLeavesGoAround, MarkerUpdateKeepsLongLanding,
                BypassAvoidsClosedRectangle, ClosedTargetIsRejected,
                DiagonalGapIsRejected, BypassReopensCellsBehindVehicle
            };
            int failed = 0;
            foreach (var test in cases)
            {
                try { test(); Console.WriteLine("PASS " + test.Method.Name); }
                catch (Exception e) { failed++; Console.WriteLine("FAIL " + test.Method.Name + ": " + e.Message); }
            }
            Console.WriteLine((cases.Length - failed) + "/" + cases.Length + " regressions passed");
            return failed == 0 ? 0 : 1;
        }

        static void Require(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }

        static Pilot WalledPilot(float free)
        {
            var p = new Pilot { ClimbScanned = true, CourseNeed = float.PositiveInfinity, NoseFree = free };
            for (int i = 0; i < p.ClimbNeed.Length; i++) p.ClimbNeed[i] = float.PositiveInfinity;
            return p;
        }

        static Pilot.Output Taxi(Pilot p, float speed)
        {
            return p.Step(Vector3.zero, Vector3.forward, new Vector3(0, 0, speed), 0,
                new Vector3(0, 0, 80), 0, 0, 0, true, 0.02f);
        }

        static void TaxiStopsAtWall()
        {
            var p = WalledPilot(1);
            var o = Taxi(p, 0);
            Require(p.Phase == "taxi" && o.Blocked && o.Throttle == 0 && o.Up && o.Down,
                "80 m trip must brake and hand back control with an obstacle 1 m away");
        }

        static void TaxiBrakesBeforeWall()
        {
            var o = Taxi(WalledPilot(4), 5);
            Require(o.Up && o.Down && !o.Blocked && o.Throttle == 0,
                "moving taxi must brake before reaching the stop margin");
        }

        static void TaxiMovesOnClearGround()
        {
            var o = Taxi(new Pilot { NoseFree = Pilot.TaxiProbeReach }, 0);
            Require(o.Throttle > 0 && !o.Blocked && !o.Up, "clear short trip must still move");
        }

        static void TaxiStopsWithPusherCoasting()
        {
            // Start with a spun-up pusher: throttle-off alone does not remove thrust.
            var g = new GyroSim();
            g.Airborne(Vector3.zero, new Vector3(0, 0, 5));
            g.Grounded = true;
            var p = new Pilot();
            const float wall = 18f, bodyPad = 1.6f;
            bool stopped = false;
            for (int i = 0; i < 1000; i++)
            {
                p.NoseFree = Mathf.Min(Pilot.TaxiProbeReach, Mathf.Max(0, wall - bodyPad - g.Pos.z));
                var o = p.Step(g.Pos, g.Fwd, g.Vel, g.YawRate * Mathf.Rad2Deg,
                    new Vector3(0, 0, 80), 0, 0, 0, g.Grounded, 0.02f);
                g.Step(o.Throttle, o.Yaw, o.Up, o.Down);
                Require(g.Pos.z < wall - bodyPad, "taxi touched the wall with residual pusher thrust");
                Require(g.Grounded, "taxi braking must stay on the wheels");
                if (o.Blocked) { stopped = true; break; }
            }
            Require(stopped && g.Vel.magnitude < 0.6f, "blocked taxi must stop and hand back control");
        }

        static void TakeoffStillStopsInWalledYard()
        {
            var p = WalledPilot(1);
            var o = p.Step(Vector3.zero, Vector3.forward, Vector3.zero, 0,
                new Vector3(0, 0, 120), 0, 0, 0, true, 0.02f);
            Require(p.NoTakeoffRun && o.Throttle == 0 && o.Up && o.Down, "blocked takeoff must still stop");
        }

        static Pilot LongLanding()
        {
            var p = new Pilot();
            p.Step(new Vector3(0, 6, 10), Vector3.forward, new Vector3(0, 0, 9), 0,
                Vector3.zero, 0, 0, 0, false, 0.02f);
            Require(p.LandingLong, "fixture must establish the old long landing");
            return p;
        }

        static void RetargetLeavesLongLandingSafely()
        {
            var p = LongLanding();
            p.Retarget(false, 6);
            var dest = new Vector3(1000, 0, 1000);
            var o = p.Step(new Vector3(0, 6, 10), Vector3.forward, new Vector3(0, 0, 9), 0,
                dest, 0, 0, 0, false, 0.02f);
            Require(!p.LandingLong && p.GoingAround && p.TargetAlt >= 45 && o.Throttle == 1 && o.Yaw == 0,
                "explicit low retarget must climb straight out, not land or turn low");
            p.Step(new Vector3(0, 45, 310), Vector3.forward, new Vector3(0, 0, 15), 0,
                dest, 0, 0, 0, false, 0.02f);
            Require(!p.GoingAround && p.Phase == "cruise", "after climbing it must fly toward the new target");
        }

        static void RetargetLeavesGoAround()
        {
            var p = new Pilot();
            p.Step(new Vector3(0, 18, 10), Vector3.forward, new Vector3(0, 0, 9), 0,
                Vector3.zero, 0, 0, 0, false, 0.02f);
            Require(p.GoingAround, "fixture must establish a go-around");
            p.Retarget(false, 60);
            p.Step(new Vector3(0, 60, 10), Vector3.forward, new Vector3(0, 0, 9), 0,
                new Vector3(1000, 0, 1000), 0, 0, 0, false, 0.02f);
            Require(!p.GoingAround && !p.LandingLong && p.Phase == "cruise", "high retarget must leave the old go-around");
        }

        static void MarkerUpdateKeepsLongLanding()
        {
            var p = LongLanding();
            p.Step(new Vector3(0, 6, 10), Vector3.forward, new Vector3(0, 0, 9), 0,
                new Vector3(0, 0, 1), 0, 0, 0, false, 0.02f);
            Require(p.LandingLong, "automatic small marker update must not interrupt a landing");
        }

        static RoadNetwork Net()
        {
            var fine = new byte[128 * 128];
            for (int i = 0; i < fine.Length; i++) fine[i] = RoadNetwork.FineAsphalt;
            return RoadNetwork.Build(fine, 128, 128, 1, 4);
        }

        static HashSet<int> Wall(RoadNetwork net)
        {
            var blocked = new HashSet<int>();
            for (int y = 0; y < net.CellsY; y++)
                for (int x = 0; x < net.CellsX; x++)
                {
                    Vector3 c = net.CellToWorld(x, y);
                    if (Math.Abs(c.x) <= 24 && c.z >= 12 && c.z <= 24) blocked.Add(y * net.CellsX + x);
                }
            return blocked;
        }

        static PathFollower MainRoute(Vector3 start)
        {
            var main = new PathFollower();
            main.SetPath(RoutePlanner.Resample(new List<Vector3> { start, new Vector3(0, 0, 60) }, 2),
                start, 14, null);
            return main;
        }

        // Independent dense sampling checks the full output segments, not just its vertices.
        static void AssertClear(RoadNetwork net, List<Vector3> path, HashSet<int> blocked)
        {
            Require(path != null && path.Count >= 2, "a safe bypass must exist for this fixture");
            for (int i = 1; i < path.Count; i++)
            {
                int steps = Math.Max(1, (int)Math.Ceiling(Vector3.Distance(path[i - 1], path[i]) / 0.02f));
                for (int j = 0; j <= steps; j++)
                {
                    Vector3 p = Vector3.Lerp(path[i - 1], path[i], (float)j / steps);
                    Require(net.WorldToCell(p.x, p.z, out int x, out int y) && !blocked.Contains(y * net.CellsX + x),
                        "output segment crosses a closed cell near " + p.x + "," + p.z);
                }
            }
        }

        static void BypassAvoidsClosedRectangle()
        {
            var net = Net();
            var wall = Wall(net);
            var path = RoutePlanner.Bypass(net, MainRoute(Vector3.zero), Vector3.zero, Vector3.forward, wall);
            AssertClear(net, path, wall);
            Require(Vector3.Distance(path[path.Count - 1], new Vector3(0, 0, 60)) < 0.01f,
                "safe bypass must retain the route's destination");
        }

        static void ClosedTargetIsRejected()
        {
            var net = Net();
            net.WorldToCell(0, 60, out int x, out int y);
            var path = RoutePlanner.FindPath(net, Vector3.zero, new Vector3(0, 0, 60), 1, 12, 1.6f,
                new HashSet<int> { y * net.CellsX + x });
            Require(path == null, "closed destination must not be appended to a partial path");
        }

        static void DiagonalGapIsRejected()
        {
            var net = Net();
            var blocked = new HashSet<int>();
            int start = 16 * net.CellsX + 16, target = 17 * net.CellsX + 17;
            for (int i = 0; i < net.Cells.Length; i++) if (i != start && i != target) blocked.Add(i);
            var path = RoutePlanner.FindPath(net, net.CellToWorld(16, 16), net.CellToWorld(17, 17), 1, 12, 1.6f, blocked);
            Require(path == null, "diagonal corner between closed cells must not be traversed");
        }

        static void BypassReopensCellsBehindVehicle()
        {
            var net = Net();
            var wall = Wall(net);
            var start = new Vector3(1.5f, 0, -1.5f);
            net.WorldToCell(start.x, start.z, out int x, out int y);
            int reopened = y * net.CellsX + x;
            wall.Add(reopened);
            var path = RoutePlanner.Bypass(net, MainRoute(start), start, Vector3.forward, wall);
            Require(wall.Contains(reopened), "bypass must not mutate the caller's blocked set");
            wall.Remove(reopened);
            AssertClear(net, path, wall);
        }
    }
}
