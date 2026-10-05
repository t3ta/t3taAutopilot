using System;
using T3taAutopilot;
using UnityEngine;

namespace AutopilotSim
{
    /// <summary>
    /// Stand-in for the game's vehicleGyrocopter (vehicles.xml + the motor /
    /// force code in EntityVehicle.PhysicsFixedUpdate), per 50 Hz step:
    ///  motor0 (top rotor) rpm += lerp(.002,.05, fwdVel/9), max 3, drag .993
    ///  motor1 (pusher)    rpm += lerp(0,.1, moveForward*1.35), max 10.8, drag .993
    ///  force0 up .19*rpm0/3 (0 above y 260), force1 fwd .15*rpm1/8,
    ///  yaw torque .03*strafe, pitch torque -.02 jump / +.02 down,
    ///  velocity *= .997, angular velocity *= .97, gravity 9.81,
    ///  horizontal speed capped at velocityMax (15 while moveForward != 0,
    ///  drifting to 9 at 1.5 m/s^2 otherwise).
    /// Roll: the game banks the craft toward -lean, lean = clamp(wheelDir/25
    ///  *2*(v/10)^2 * 8, +-8) deg, wheelDir -> moveStrafe*25 at
    ///  130*clamp(1-(v/10)^2,.15,1)*1.5 deg/s (vehicles.xml sets no
    ///  steerAngleMax for the gyro, so the default 25 applies); roll rate +=
    ///  (|roll+lean|-2)*.01 toward it when |roll+lean| > 2 (tiltUpForce 1,
    ///  no tilt dampening). A roll tilts the lift sideways.
    /// Orientation is kept as a nose / up frame plus a roll angle
    /// (UnityEngine's Quaternion is native code, unusable outside the engine).
    /// </summary>
    sealed class GyroSim
    {
        const float Dt = 0.02f;
        public Vector3 Pos, Vel;
        public Vector3 Fwd = Vector3.forward, Up = Vector3.up;
        public float YawRate, PitchRate;   // rad/s (+ = right / nose up)
        public float Roll, RollRate;       // deg, deg/s (+ = right wing down)
        float wheelDir;
        float rpm0, rpm1, velMax = 9f;
        public bool Grounded;
        public static float LiftTilt = 0.5f; // share of the rotor lift that tilts with the body (AD_LIFT_TILT): 0.5 matches the recorded speed loss with the throttle off, 1 kept it at the cap
        public static float VelPer = 1f;       // VehicleVelocityMaxPer (AD_VEL_PER): 1.18 = supercharger; scales the speed caps
        public static bool ClimbScan = true;   // AD_CLIMB_SCAN=0: take off without the level sweeps (as before)
        public static float VDrag = 0.15f;  // extra vertical drag, 1/s (AD_VDRAG): 0.15 matches the recorded sink with the throttle off (-2.4 m/s at 9 m/s, -17 deg) and on (-5.5..-8 m/s)
        public Func<float, float, float> Ground = (x, z) => 0f;

        /// <summary>Start flying: at pos, moving at vel (level, nose along it), rotor and prop spun up.</summary>
        public void Airborne(Vector3 pos, Vector3 vel)
        {
            Pos = pos;
            Vel = vel;
            Fwd = new Vector3(vel.x, 0f, vel.z).normalized;
            rpm0 = 3f;
            rpm1 = 8f;
            velMax = 15f * VelPer;
            Grounded = false;
        }

        public void Step(float throttle, float strafe, bool up, bool down)
        {
            float fwdVel = Vector3.Dot(Vel, Fwd);
            rpm0 *= 0.993f;
            float n0 = fwdVel / 9f;
            if (n0 >= 0.01f) rpm0 = Mathf.Min(rpm0 + Mathf.Lerp(0.002f, 0.05f, n0), 3f);
            rpm1 *= 0.993f;
            if (throttle != 0f) rpm1 = Mathf.Min(rpm1 + Mathf.Lerp(0f, 0.1f, throttle * 1.35f), 10.8f);

            float target = (throttle > 0f ? 15f : 9f) * VelPer;   // reverse: velocityMax_turbo backward = 9
            velMax = Mathf.MoveTowards(velMax, target, (target > velMax ? 2.5f : 1.5f) * Dt);

            // bank-into-the-turn spring
            float v = Vel.magnitude;
            float num17 = (v * 0.1f) * (v * 0.1f);
            wheelDir = Mathf.MoveTowards(wheelDir, strafe * 25f, 130f * Mathf.Clamp(1f - num17, 0.15f, 1f) * Dt * 1.5f);
            float lean = Mathf.Clamp(wheelDir / 25f * 2f * num17 * 8f, -8f, 8f);
            float f2 = lean - Roll;                  // wants Roll = lean (right turn: right wing down)
            if (Mathf.Abs(f2) > 2f) RollRate += Mathf.Clamp((Mathf.Abs(f2) - 2f) * Mathf.Sign(f2) * 0.01f, -4f, 4f) * Mathf.Rad2Deg;
            RollRate *= 0.97f;
            Roll += RollRate * Dt;

            Vector3 rightAx = Vector3.Cross(Up, Fwd).normalized;
            float rr = Roll * Mathf.Deg2Rad;
            Vector3 liftDir = Up * Mathf.Cos(rr) + rightAx * Mathf.Sin(rr);
            if (Pos.y < 260f) Vel += Vector3.Lerp(Vector3.up * Vector3.Dot(liftDir, Vector3.up), liftDir, LiftTilt) * (0.19f * rpm0 / 3f);
            Vel += Fwd * (0.15f * rpm1 / 8f);
            Vel.y -= 9.81f * Dt;
            Vel.y *= 1f - VDrag * Dt;

            YawRate += 0.03f * strafe;
            if (up) PitchRate += 0.02f;
            if (down) PitchRate -= 0.02f;

            Vel *= 0.997f;
            YawRate *= 0.97f;
            PitchRate *= 0.97f;
            float h = Mathf.Sqrt(Vel.x * Vel.x + Vel.z * Vel.z);
            if (h > velMax) { Vel.x *= velMax / h; Vel.z *= velMax / h; }

            // rotate the frame: yaw about world up, pitch about the right axis
            Fwd = Rotate(Fwd, Vector3.up, YawRate * Dt);
            Up = Rotate(Up, Vector3.up, YawRate * Dt);
            Vector3 right = Vector3.Cross(Up, Fwd).normalized;
            Fwd = Rotate(Fwd, right, -PitchRate * Dt).normalized;
            Up = Vector3.Cross(Fwd, right).normalized;

            Pos += Vel * Dt;
            float g = Ground(Pos.x, Pos.z);
            Grounded = Pos.y <= g + 0.01f;
            if (Grounded)
            {
                Roll = 0f;
                RollRate = 0f;
                Pos.y = g;
                if (Vel.y < 0f) Vel.y = 0f;
                // wheels: rolling drag, brakes on jump, nose can't dig in
                float hv = Mathf.Sqrt(Vel.x * Vel.x + Vel.z * Vel.z);
                float decel = (up ? 5f : 0.5f) * Dt;
                if (hv > 1e-3f) { float k = Mathf.Max(0f, hv - decel) / hv; Vel.x *= k; Vel.z *= k; }
                float pitch = Mathf.Asin(Mathf.Clamp(Fwd.y, -1f, 1f));
                if (pitch < 0f || (!up && pitch > 0f && hv < 5f))
                {
                    Fwd = new Vector3(Fwd.x, 0f, Fwd.z).normalized;
                    Up = Vector3.up;
                    PitchRate = 0f;
                }
            }
        }

        /// <summary>Rotation of v about unit axis k by a rad (a > 0 about +y turns +z toward +x).</summary>
        static Vector3 Rotate(Vector3 v, Vector3 k, float a)
        {
            float c = Mathf.Cos(a), s = Mathf.Sin(a);
            return v * c + Vector3.Cross(k, v) * s + k * (Vector3.Dot(k, v) * (1f - c));
        }

        /// <summary>
        /// AD_GYRO=1: fly scenarios (flat, hill wall, tower at the destination)
        /// and print a trace + outcome.
        /// </summary>
        static float Noise = 0f;
        static readonly System.Random Rng = new System.Random(7);
        static float N(float a) => (float)(Rng.NextDouble() * 2 - 1) * a * Noise;

        /// <summary>
        /// AD_GYRO_REPLAY=&lt;telemetry csv&gt;,t0,t1[,groundY]: start from the
        /// recorded state at t0 (rotor at full rpm, pusher at the rpm the
        /// recorded throttle holds), feed the recorded inputs and print the
        /// simulated vs recorded height / speed / sink / pitch every second.
        /// Checks the model against real flying (manual landings especially).
        /// </summary>
        static void Replay(string spec)
        {
            var parts = spec.Split(',');
            string path = parts[0];
            float t0 = float.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
            float t1 = float.Parse(parts[2], System.Globalization.CultureInfo.InvariantCulture);
            var lines = System.IO.File.ReadAllLines(path);
            var head = lines[0].Split(',');
            int C(string n) => Array.IndexOf(head, n);
            var rows = new System.Collections.Generic.List<float[]>();
            string[] cols = { "t", "x", "y", "z", "yaw", "pitch", "vx", "vy", "vz", "throttle", "strafe", "jump", "down" };
            for (int i = 1; i < lines.Length; i++)
            {
                var f = lines[i].Split(',');
                var r = new float[cols.Length];
                for (int k = 0; k < cols.Length; k++)
                {
                    string s = f[C(cols[k])];
                    r[k] = s.Length > 0 ? float.Parse(s, System.Globalization.CultureInfo.InvariantCulture) : 0f;
                }
                if (r[0] >= t0 && r[0] <= t1) rows.Add(r);
            }
            if (rows.Count == 0) { Console.WriteLine("no samples in range"); return; }
            float groundY = parts.Length > 3 ? float.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture) : -1000f;
            var s0 = rows[0];
            float yaw = s0[4] * Mathf.Deg2Rad, pitch = s0[5] * Mathf.Deg2Rad;
            var g = new GyroSim { Ground = (x, z) => groundY };
            g.Pos = new Vector3(s0[1], s0[2], s0[3]);
            g.Vel = new Vector3(s0[6], s0[7], s0[8]);
            g.Fwd = new Vector3(Mathf.Sin(yaw) * Mathf.Cos(pitch), Mathf.Sin(pitch), Mathf.Cos(yaw) * Mathf.Cos(pitch));
            Vector3 right = Vector3.Cross(Vector3.up, g.Fwd).normalized;
            g.Up = Vector3.Cross(g.Fwd, right).normalized;
            g.rpm0 = 3f;
            g.rpm1 = s0[9] > 0f ? Mathf.Min(10.8f, Mathf.Lerp(0f, 0.1f, s0[9] * 1.35f) / 0.007f) : 0f;
            g.velMax = (s0[9] != 0f ? 15f : Mathf.Max(9f, new Vector3(g.Vel.x, 0, g.Vel.z).magnitude / VelPer)) * VelPer;   // AD_VEL_PER: same caps as Step
            Console.WriteLine("    t |    y sim/rec  |  hs sim/rec | vy sim/rec  | pitch sim/rec | in");
            int idx = 0;
            float nextPrint = t0;
            for (float t = t0; t <= rows[rows.Count - 1][0]; t += Dt)
            {
                while (idx + 1 < rows.Count && rows[idx + 1][0] <= t) idx++;
                var r = rows[idx];
                if (t >= nextPrint)
                {
                    nextPrint += 1f;
                    float hs = new Vector3(g.Vel.x, 0, g.Vel.z).magnitude, rhs = new Vector3(r[6], 0, r[8]).magnitude;
                    Console.WriteLine($"{r[0],5:F1} | {g.Pos.y,6:F1} {r[2],6:F1} | {hs,5:F1} {rhs,5:F1} | {g.Vel.y,5:F1} {r[7],5:F1} | " +
                                      $"{Mathf.Asin(g.Fwd.y) * Mathf.Rad2Deg,6:F1} {r[5],6:F1} | thr {r[9],5:F2} str {r[10],5:F2} {(r[11] > 0 ? "U" : " ")}{(r[12] > 0 ? "D" : " ")}");
                }
                g.Step(r[9], r[10], r[11] > 0f, r[12] > 0f);
            }
        }

        public static void Run()
        {
            var replay = Environment.GetEnvironmentVariable("AD_GYRO_REPLAY");
            LiftTilt = Environment.GetEnvironmentVariable("AD_LIFT_TILT") is string lt ? float.Parse(lt) : LiftTilt;
            ClimbScan = Environment.GetEnvironmentVariable("AD_CLIMB_SCAN") != "0";
            VDrag = Environment.GetEnvironmentVariable("AD_VDRAG") is string vd ? float.Parse(vd) : VDrag;
            VelPer = Environment.GetEnvironmentVariable("AD_VEL_PER") is string vp ? float.Parse(vp) : VelPer;
            if (replay != null) { Replay(replay); return; }
            float E(string k, float d) { var v = Environment.GetEnvironmentVariable(k); return v != null ? float.Parse(v) : d; }
            Noise = E("AD_GYRO_NOISE", 0f);
            Pilot.YawSmoothing = E("AD_YAW_SMOOTH", Pilot.YawSmoothing);
            Pilot.YawSlew = E("AD_YAW_SLEW", Pilot.YawSlew);
            Pilot.FfGain = E("AD_FF_GAIN", Pilot.FfGain);
            Pilot.FlareHeight = E("AD_FLARE", Pilot.FlareHeight);
            Pilot.GlideSlope = E("AD_GLIDE", Pilot.GlideSlope);
            Pilot.ApproachSpeed = E("AD_APPROACH_SPEED", Pilot.ApproachSpeed);
            if (Environment.GetEnvironmentVariable("AD_LOITER") != null)
            {
                // circling while a destination is picked in the air (autopilot off)
                LoiterScenario("cruise 15 m/s, 60 m up", 60f, new Vector3(0f, 0f, 15f), (x, z) => 0f);
                LoiterScenario("slow 5 m/s, 60 m up", 60f, new Vector3(0f, 0f, 5f), (x, z) => 0f);
                LoiterScenario("sinking 4 m/s at 9 m/s, 50 m up", 50f, new Vector3(0f, -4f, 9f), (x, z) => 0f);
                LoiterScenario("low: 12 m up at 10 m/s", 12f, new Vector3(0f, 0f, 10f), (x, z) => 0f);
                LoiterScenario("hillside rising 0.5 under the circle", 70f, new Vector3(0f, 0f, 12f), (x, z) => Mathf.Clamp(x * 0.5f + 20f, 0f, 60f));
                LoiterScenario("near the ceiling: 245 m up", 245f, new Vector3(0f, 0f, 15f), (x, z) => 0f);
                return;
            }
            if (Environment.GetEnvironmentVariable("AD_TREES") != null)
            {
                // a wide stand across the course (+z) whose edge is off to the side (+x):
                // the way out is diagonal, past the edge
                foreach (float cd in new[] { 45f, 70f, 100f })
                    foreach (float edge in new[] { 20f, 45f, 70f })
                        foreach (float ct in new[] { 15f, 22f })
                            Scenario($"trees {ct} m tall from {cd} m out, edge {edge} m aside", 600f, (x, z) => 0f, -1f, Trees(-200f, edge, cd, cd + 60f, ct, 7));
                return;
            }
            Scenario("flat 600 m", 600f, (x, z) => 0f);
            Scenario("ridge 60 m high at 300 m", 700f, (x, z) => z > 280f && z < 340f ? 60f : 0f);
            Scenario("rising terrain to 120 m", 900f, (x, z) => Mathf.Clamp((z - 200f) * 0.25f, 0f, 120f));
            Scenario("dest behind (turn back)", -400f, (x, z) => 0f);
            // starts inside the approach distance while still on the ground
            Scenario("downhill departure, dest behind", -400f, (x, z) => Mathf.Abs(z) < 60f ? 100f : 0f);
            Scenario("short hop 80 m", 80f, (x, z) => 0f);
            // the recorded flight: cruise at 150 m (route top 100 + 50) over flat ground, land
            Scenario("high cruise 150 m then land", 1500f, (x, z) => 0f, 100f);
            // a longer trip: where a higher speed cap (AD_VEL_PER) pays off
            Scenario("flat 3 km", 3000f, (x, z) => 0f);
            // on short final far too high (18 m up 10 m out at 9 m/s): it overshoots.
            // Recorded: turning back low circled at 2-9 m, 22-32 deg/s, drifted 47 m
            Scenario("overshoot: 18 m up 10 m short", 5f, (x, z) => 0f, -1f, null,
                g => g.Airborne(new Vector3(30f, 18f, -5f), new Vector3(0f, -1f, 9f)));
            // the same over high ground (230 m): the go-around height is capped at the
            // ceiling (250), it must still come back round
            Scenario("overshoot on high ground (230 m)", 5f, (x, z) => 230f, -1f, null,
                g => g.Airborne(new Vector3(30f, 248f, -5f), new Vector3(0f, -1f, 9f)));
            // a yard walled in all round 22 m out (8 m sheds every 20 deg) except behind
            // (150-210 deg): pick the gap and keep to it, don't swing left and right
            Scenario("cluttered yard, gap behind", 600f, (x, z) => 0f, -1f, YardRing());
            // started on the final glide 24 m above the path: goes around (and must turn back)
            Scenario("high on final (started high)", 200f, (x, z) => 0f, -1f, null,
                g => g.Airborne(new Vector3(30f, 45f, 110f), new Vector3(0f, 0f, 12f)));
            // the landing site moves closer on the dive (a quest's rally marker
            // showing up; recorded: 20 m too high 100 m out): power down on the final.
            // 90 m closer is too much for any sink rate: that one goes around
            Scenario("landing site moves 50 m closer on the approach", 600f, (x, z) => 0f, -1f, null, null,
                (g, d) => d.z > 575f && new Vector3(d.x - g.Pos.x, 0f, d.z - g.Pos.z).magnitude < 170f ? new Vector3(d.x, d.y, d.z - 50f) : d);
            Scenario("landing site moves 90 m closer on the approach", 600f, (x, z) => 0f, -1f, null, null,
                (g, d) => d.z > 550f && new Vector3(d.x - g.Pos.x, 0f, d.z - g.Pos.z).magnitude < 170f ? new Vector3(d.x, d.y, d.z - 90f) : d);
            // floating past low (6 m up 5 m short): too low to go around - land long, no turn
            Scenario("long landing: 6 m up 5 m short", 5f, (x, z) => 0f, -1f, null,
                g => g.Airborne(new Vector3(30f, 6f, 0f), new Vector3(0f, -0.5f, 9f)));
            // take-off with something in front (the downward rays miss trees:
            // only the level sweeps see these)
            Scenario("tree line 15 m tall 45 m ahead", 600f, (x, z) => 0f, -1f, new[] { new Obstacle(-35f, 35f, 45f, 55f, 15f) });
            Scenario("shed 5 m tall 18 m ahead", 600f, (x, z) => 0f, -1f, new[] { new Obstacle(-6f, 6f, 18f, 26f, 5f) });
            Scenario("trees on three sides", 600f, (x, z) => 0f, -1f, new[] {
                new Obstacle(-40f, 40f, 40f, 50f, 18f), new Obstacle(-50f, -40f, -20f, 50f, 18f), new Obstacle(40f, 50f, -20f, 50f, 18f) });
            Scenario("tree line 25 m tall 90 m ahead", 600f, (x, z) => 0f, -1f, new[] { new Obstacle(-60f, 60f, 90f, 100f, 25f) });
            // the course 7.5 deg off the 15 deg grid (dest at x 30, z 228) with a
            // narrow pole on that exact line 50 m out: only the exact-course sweep sees it
            Scenario("pole on the exact course, 7.5 deg off the grid", 228f, (x, z) => 0f, -1f, new[] { new Obstacle(4.5f, 8.5f, 47.5f, 51.5f, 20f) });
            // a quest's strip on the street: houses 10 m tall from 25 m past it (the strip is checked to 15 m)
            // (recorded: the floor over them held the approach 3-12 m up over the strip)
            Scenario("houses 10 m tall past the strip", 600f, (x, z) => 0f, -1f, new[] {
                new Obstacle(-10f, 70f, 625f, 675f, 10f) });
            // a stand of trees across the course, the way out diagonal past its edge;
            // with AD_SCAN_MISS the sweeps slip through gaps (AD_TREES=1: a sweep of these)
            Scenario("trees 15 m tall from 70 m out, edge 70 m aside", 600f, (x, z) => 0f, -1f, Trees(-200f, 70f, 70f, 130f, 15f, 7));
            // on a road between guardrails 1 m tall, 6.5 m apart, off its centre line,
            // the course 3 deg off the road (recorded: at the rotor's width every
            // way along the road read blocked, it swung left / right and stuck)
            Scenario("road between guardrails", 600f, (x, z) => 0f, -1f, new[] {
                new Obstacle(-3.4f, -3f, -30f, 400f, 1f), new Obstacle(3f, 3.4f, -30f, 400f, 1f) },
                g => g.Pos = new Vector3(-0.8f, 0f, 0f));
            // walls 4 m tall all round, 6 m out: no take-off run anywhere - stop, don't run at a wall
            Scenario("walled yard", 600f, (x, z) => 0f, -1f, new[] {
                new Obstacle(-8f, 8f, 6f, 8f, 4f), new Obstacle(-8f, 8f, -8f, -6f, 4f), new Obstacle(-8f, -6f, -8f, 8f, 4f), new Obstacle(6f, 8f, -8f, 8f, 4f) });
        }

        /// <summary>
        /// Pilot.Loiter for 90 s from an airborne start: the height it holds
        /// (after 20 s to settle), how low it gets, how slow, where it drifts.
        /// </summary>
        static void LoiterScenario(string name, float agl, Vector3 vel, Func<float, float, float> ground)
        {
            var g = new GyroSim { Ground = ground };
            g.Airborne(new Vector3(0f, ground(0f, 0f) + agl, 0f), vel);
            var p = new Pilot();
            p.Reset();
            float alt = g.Pos.y;
            float minAgl = float.MaxValue, minSpeed = float.MaxValue, maxDev = 0f, maxR = 0f, maxRoll = 0f, rateSum = 0f;
            int rateN = 0;
            bool touched = false;
            Vector3 c = Vector3.zero;
            int cn = 0;
            for (int step = 0; step * Dt < 90f; step++)
            {
                float t = step * Dt;
                float yawDeg = g.YawRate * Mathf.Rad2Deg + N(4f);
                Vector3 velM = g.Vel + new Vector3(N(0.8f), N(0.3f), N(0.8f));
                var o = p.Loiter(g.Pos, g.Fwd, velM, yawDeg, alt, ground(g.Pos.x, g.Pos.z), 1f, Dt);
                g.Step(o.Throttle, o.Yaw, o.Up, o.Down);
                float a = g.Pos.y - ground(g.Pos.x, g.Pos.z);
                minAgl = Mathf.Min(minAgl, a);
                touched |= g.Grounded;
                if (t > 20f)
                {
                    minSpeed = Mathf.Min(minSpeed, new Vector3(g.Vel.x, 0f, g.Vel.z).magnitude);
                    maxDev = Mathf.Max(maxDev, Mathf.Abs(g.Pos.y - p.TargetAlt));
                    maxRoll = Mathf.Max(maxRoll, Mathf.Abs(g.Roll));
                    rateSum += g.YawRate * Mathf.Rad2Deg;
                    rateN++;
                    c += new Vector3(g.Pos.x, 0f, g.Pos.z);
                    cn++;
                }
            }
            c /= Mathf.Max(cn, 1);
            // radius: how far from the circle's centre it was at the end (one more lap)
            for (int step = 0; step * Dt < 20f; step++)
            {
                var o = p.Loiter(g.Pos, g.Fwd, g.Vel, g.YawRate * Mathf.Rad2Deg, alt, ground(g.Pos.x, g.Pos.z), 1f, Dt);
                g.Step(o.Throttle, o.Yaw, o.Up, o.Down);
                maxR = Mathf.Max(maxR, new Vector3(g.Pos.x - c.x, 0f, g.Pos.z - c.z).magnitude);
            }
            Console.WriteLine($"{name,-40} minAGL={minAgl,6:F1} dev={maxDev,5:F1} (target {p.TargetAlt:F0}) minV={minSpeed,5:F1} " +
                $"rate={rateSum / Mathf.Max(rateN, 1),5:F1}deg/s radius={maxR,5:F0} roll={maxRoll,4:F0}{(touched ? "  TOUCHED GROUND" : "")}");
        }

        static Obstacle[] YardRing()
        {
            var list = new System.Collections.Generic.List<Obstacle>();
            for (int a = 0; a < 360; a += 20)
            {
                if (a > 140 && a < 220) continue;   // the way out
                float r = a * Mathf.Deg2Rad, cx = 22f * Mathf.Sin(r), cz = 22f * Mathf.Cos(r);
                list.Add(new Obstacle(cx - 4f, cx + 4f, cz - 4f, cz + 4f, 8f));
            }
            return list.ToArray();
        }
        /// <summary>
        /// A stand of single trees (trunk + crown 3 m across, 6-9 m apart,
        /// 70-100% of top tall): the sweeps catch one or slip through a gap
        /// depending on where they start, as in game.
        /// </summary>
        static Obstacle[] Trees(float x0, float x1, float z0, float z1, float top, int seed)
        {
            var r = new System.Random(seed);
            var list = new System.Collections.Generic.List<Obstacle>();
            for (float z = z0; z < z1; z += 6f + (float)r.NextDouble() * 3f)
                for (float x = x0 + (float)r.NextDouble() * 6f; x < x1; x += 6f + (float)r.NextDouble() * 3f)
                    list.Add(new Obstacle(x - 1.5f, x + 1.5f, z - 1.5f, z + 1.5f, top * (0.7f + 0.3f * (float)r.NextDouble())));
            return list.ToArray();
        }

        // AD_SCAN_MISS=p: each sweep level slips through a gap between the trees
        // with probability p (in game one sweep saw the course clear, the next
        // one blocked 3 m ahead - and it had turned into the trees)
        static readonly float ScanMiss = Environment.GetEnvironmentVariable("AD_SCAN_MISS") is string sm ? float.Parse(sm, System.Globalization.CultureInfo.InvariantCulture) : 0f;
        // gyro body half width + ProbeSideMargin (AD_BODY_PAD=2.4: the rotor's, as before)
        static readonly float BodyPad = Environment.GetEnvironmentVariable("AD_BODY_PAD") is string bp ? float.Parse(bp, System.Globalization.CultureInfo.InvariantCulture) : 1.6f;
        static readonly bool LandClip = Environment.GetEnvironmentVariable("AD_LAND_CLIP") != "0";

        readonly struct Obstacle
        {
            public readonly float X0, X1, Z0, Z1, Top;
            public Obstacle(float x0, float x1, float z0, float z1, float top) { X0 = x0; X1 = x1; Z0 = z0; Z1 = z1; Top = top; }
            public bool Covers(float x, float z, float pad) => x > X0 - pad && x < X1 + pad && z > Z0 - pad && z < Z1 + pad;
        }

        /// <summary>What ScanClimbOut would measure: level box sweeps (2.4 m half width, 0.6 m half height) in each direction.</summary>
        static void ScanClimbOut(Pilot p, GyroSim g, Obstacle[] obs, Vector3 dest)
        {
            Vector3 to = dest - g.Pos;
            to.y = 0f;
            Vector3 vflat = new Vector3(g.Vel.x, 0f, g.Vel.z);
            if (p.GoingAround && vflat.sqrMagnitude > 1f) to = vflat;   // as ScanClimbOut: straight ahead
            int course = Pilot.DirIndex(to);
            int heading = Pilot.ScanHeading(g.Vel, g.Fwd, g.Grounded);
            for (int i = 0; i < Pilot.ClimbDirs; i++)
            {
                if (!Pilot.NeedsScan(i, course, heading, g.Grounded)) { p.ClimbNeed[i] = float.PositiveInfinity; continue; }
                p.ClimbNeed[i] = SweepNeed(g, obs, Pilot.DirVector(i));
            }
            if (to.sqrMagnitude > 1f) p.CourseNeed = SweepNeed(g, obs, to.normalized);   // the exact course
            Vector3 nose = new Vector3(g.Fwd.x, 0f, g.Fwd.z).normalized;
            p.NoseFree = float.PositiveInfinity;
            for (float d = 0.5f; d <= Pilot.NoseRoom + 1f; d += 0.5f)
            {
                Vector3 q = g.Pos + nose * d;
                bool hit = false;
                foreach (var o in obs) hit |= o.Top > g.Pos.y + Pilot.ClimbLevels[0] - 0.6f && o.Covers(q.x, q.z, BodyPad);
                if (hit) { p.NoseFree = d; break; }
            }
            p.ClimbScanned = true;
        }

        static float SweepNeed(GyroSim g, Obstacle[] obs, Vector3 dir)
        {
            float roll = Pilot.TakeoffRoll(g.Grounded, g.Vel, dir);
            var free = new float[Pilot.ClimbLevels.Length];
            for (int k = 0; k < Pilot.ClimbLevels.Length; k++)
            {
                float y = g.Pos.y + Pilot.ClimbLevels[k];
                free[k] = Pilot.ClimbReach;
                for (float d = 0.5f; d < Pilot.ClimbReach; d += 0.5f)
                {
                    Vector3 q = g.Pos + dir * d;
                    bool hit = false;
                    // body width below the rotor, the rotor's above (as ScanClimbOut)
                    float pad = Pilot.ClimbLevels[k] < Pilot.RotorLevel ? BodyPad : 2.4f;
                    foreach (var o in obs) hit |= o.Top > y - 0.6f && o.Covers(q.x, q.z, pad);
                    if (hit) { free[k] = ScanMiss > 0f && Rng.NextDouble() < ScanMiss ? Pilot.ClimbReach : d; break; }
                }
            }
            return Pilot.NeedGradient(free, roll);
        }

        static void Scenario(string name, float destZ, Func<float, float, float> ground, float routeTopOverride = -1f, Obstacle[] obs = null,
            Action<GyroSim> start = null, Func<GyroSim, Vector3, Vector3> retarget = null)
        {
            obs = obs ?? new Obstacle[0];
            float lowTurn = 0f, drift = 0f;   // max |yaw rate| below 10 m AGL airborne; max distance from dest once within 15 m
            int groundReversals = 0, goArounds = 0;   // yaw reversals on the wheels before lift-off; go-arounds
            float lastGroundYawSign = 0f;
            bool everAirborne = false, wasGoingAround = false;
            bool near = false;
            int obstacleHits = 0;
            bool noRun = false;
            bool inObstacle = false;
            float maxDetour = 0f;
            int detourSwitches = 0;   // take-off / climb-out heading changes (recorded: 12 in 4 s after lift-off, then a hit)
            float lastDetour = 0f;
            var g = new GyroSim { Ground = ground };
            start?.Invoke(g);
            var p = new Pilot { Cruise = 15f * VelPer };   // what FlightCruise reads off the vehicle
            Vector3 dest = new Vector3(30f, ground(30f, destZ), destZ);
            float routeTop = 0f;
            for (float f = 0f; f <= 1f; f += 0.002f)
                routeTop = Mathf.Max(routeTop, ground(dest.x * f, dest.z * f));
            p.RouteTop = routeTopOverride >= 0f ? routeTopOverride : routeTop;
            float minClear = float.MaxValue, maxAlt = 0f, maxRoll = 0f, rollSq = 0f;
            int rollN = 0, rollFlips = 0;
            float lastRollSign = 0f;
            string trace = "";
            bool arrived = false, climbed = false;
            float overDest = float.NaN;   // height above ground when first within 40 m of the destination
            float touchdown = float.NaN;  // distance to the destination at the first touchdown after flying
            float tdSpeed = float.NaN, tdSink = float.NaN;   // at that touchdown
            float prevVy = 0f;
            bool wasAir = false;
            float t = 0f;
            for (int step = 0; t < 300f; t = ++step * Dt)
            {
                // what the controller would measure: highest ground along the
                // heading for the next 20 + 10*v m, ground under the craft
                Vector3 flat = new Vector3(g.Fwd.x, 0f, g.Fwd.z).normalized;
                float hv = new Vector3(g.Vel.x, 0f, g.Vel.z).magnitude;
                if (retarget != null) dest = retarget(g, dest);
                // (the downward rays hit roofs: an obstacle's top counts;
                // on the approach only up to 10 m short of the strip, as ScanTerrain)
                float top = ground(g.Pos.x, g.Pos.z);
                float reach = 20f + 10f * hv;
                if (LandClip && p.Phase == "approach" && !p.LandingLong)
                    reach = Mathf.Min(reach, new Vector3(dest.x - g.Pos.x, 0f, dest.z - g.Pos.z).magnitude - 10f);
                for (float d = 4f; d <= reach; d += 8f)
                {
                    Vector3 q = g.Pos + flat * d;
                    top = Mathf.Max(top, ground(q.x, q.z));
                    foreach (var ob in obs) if (ob.Covers(q.x, q.z, 0f)) top = Mathf.Max(top, ob.Top);
                }
                float yawDeg = g.YawRate * Mathf.Rad2Deg + N(4f);
                Vector3 velM = g.Vel + new Vector3(N(0.8f), N(0.3f), N(0.8f));
                if (ClimbScan && (g.Grounded || g.Pos.y - ground(g.Pos.x, g.Pos.z) < Pilot.ClimbOutHeight) && p.Phase != "approach" && p.Phase != "taxi")
                {
                    if (step % 12 == 0) ScanClimbOut(p, g, obs, dest);   // every 0.24 s
                }
                else p.ClimbScanned = false;
                var o = p.Step(g.Pos, g.Fwd, velM, yawDeg, dest, ground(g.Pos.x, g.Pos.z), top,
                    ground(dest.x, dest.z), g.Grounded, Dt);
                if (o.Arrived) { arrived = true; break; }
                if (p.NoTakeoffRun) { noRun = true; break; }   // the controller disengages here
                g.Step(o.Throttle, o.Yaw, o.Up, o.Down);
                float clear = g.Pos.y - ground(g.Pos.x, g.Pos.z);
                float dd = new Vector3(dest.x - g.Pos.x, 0f, dest.z - g.Pos.z).magnitude;
                if (!g.Grounded) everAirborne = true;
                if (g.Grounded && !everAirborne)
                {
                    float yr = g.YawRate * Mathf.Rad2Deg;
                    if (Mathf.Abs(yr) > 8f)
                    {
                        float sgn = Mathf.Sign(yr);
                        if (lastGroundYawSign != 0f && sgn != lastGroundYawSign) groundReversals++;
                        lastGroundYawSign = sgn;
                    }
                }
                if (p.GoingAround && !wasGoingAround) goArounds++;
                wasGoingAround = p.GoingAround;
                if (dd < 15f) near = true;
                if (near) drift = Mathf.Max(drift, dd);
                if (!g.Grounded && clear < 10f && near) lowTurn = Mathf.Max(lowTurn, Mathf.Abs(g.YawRate * Mathf.Rad2Deg));
                bool inside = false;
                foreach (var ob in obs) inside |= g.Pos.y < ob.Top && ob.Covers(g.Pos.x, g.Pos.z, 1.5f);
                if (inside && !inObstacle) { obstacleHits++; trace += $" HIT@{t:F1}s y={g.Pos.y:F1}"; }
                inObstacle = inside;
                maxDetour = Mathf.Max(maxDetour, Mathf.Abs(p.Detour));
                if ((p.Detour != 0f) != (lastDetour != 0f) || Mathf.Abs(p.Detour - lastDetour) > 20f) { detourSwitches++; lastDetour = p.Detour; }
                if (clear > 10f) climbed = true;
                if (climbed && p.Phase == "cruise" && ground(g.Pos.x, g.Pos.z) > 1f) minClear = Mathf.Min(minClear, clear);
                maxAlt = Mathf.Max(maxAlt, g.Pos.y);
                if (!g.Grounded && g.Pos.y - ground(g.Pos.x, g.Pos.z) > 3f) wasAir = true;
                if (wasAir && g.Grounded && float.IsNaN(touchdown))
                {
                        touchdown = new Vector3(dest.x - g.Pos.x, 0, dest.z - g.Pos.z).magnitude;
                    tdSpeed = hv;
                    tdSink = prevVy;
                }
                prevVy = g.Vel.y;
                if (float.IsNaN(overDest) && new Vector3(dest.x - g.Pos.x, 0, dest.z - g.Pos.z).magnitude < 40f)
                    overDest = g.Pos.y - ground(g.Pos.x, g.Pos.z);
                if (!g.Grounded)
                {
                    maxRoll = Mathf.Max(maxRoll, Mathf.Abs(g.Roll));
                    rollSq += g.RollRate * g.RollRate; rollN++;
                    if (Mathf.Abs(g.RollRate) > 3f && Mathf.Sign(g.RollRate) != lastRollSign) { rollFlips++; lastRollSign = Mathf.Sign(g.RollRate); }
                }
                if (!g.Grounded && clear < 0.05f && g.Vel.y < -3f) trace += $" CRASH@{t:F0}s vy={g.Vel.y:F1}";
                if (step % 250 == 0)   // every 5 s (t += Dt drifted off the 5 s marks)
                {
                    trace += $"\n   t={t,3:F0} {p.Phase,-8} pos=({g.Pos.x,5:F0},{g.Pos.y,4:F0},{g.Pos.z,5:F0}) v={hv,4:F1} vy={g.Vel.y,5:F1} " +
                             $"pitch={Mathf.Asin(g.Fwd.y) * Mathf.Rad2Deg,5:F1} want={p.DesiredPitch,5:F1} alt*={p.TargetAlt,4:F0} dist={new Vector3(dest.x - g.Pos.x, 0, dest.z - g.Pos.z).magnitude,4:F0}";
                }
            }
            float miss = new Vector3(dest.x - g.Pos.x, 0, dest.z - g.Pos.z).magnitude;
            Console.WriteLine($"== {name}: {(arrived ? "ARRIVED" : "no arrival")} in {t:F0}s, stopped {miss:F1} m from dest, " +
                              $"max alt {maxAlt:F0}, {overDest:F0} m up at 40 m out, touchdown {touchdown:F0} m out at {tdSpeed:F1} m/s sinking {-tdSink:F1}, min clearance over raised ground {minClear:F1} m, " +
                              $"roll max {maxRoll:F1} deg, roll rate rms {Mathf.Sqrt(rollSq / Mathf.Max(1, rollN)):F1} deg/s, roll swings {rollFlips}" +
                              (obs.Length > 0 ? $", obstacle hits {obstacleHits}, max detour {maxDetour:F0} deg, detour switches {detourSwitches}" : "") +
                              $", near dest: low turn max {lowTurn:F0} deg/s, drift max {drift:F0} m" +
                              $", ground yaw reversals {groundReversals}, go-arounds {goArounds}" +
                              (noRun ? $", STOPPED: no take-off run at {t:F1}s, moved {new Vector3(g.Pos.x, 0f, g.Pos.z).magnitude:F1} m" : "") + trace);
        }
    }
}
