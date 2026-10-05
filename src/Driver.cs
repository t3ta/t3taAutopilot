using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// One-frame driving decision: path tracking, speed planning, obstacle
    /// avoidance and stuck recovery, given the vehicle state and the free
    /// distance of each obstacle probe. Pure logic - AutopilotController does
    /// the physics casts and writes the result into MovementInput, and
    /// tools/sim runs this exact class against a vehicle model.
    ///
    /// Avoidance shifts the pursuit target sideways off the route (a lateral
    /// offset) instead of steering at a bare gap: the car pulls out, drives
    /// parallel past the obstacle, and only steps back toward the route after
    /// the way back has been clear for ReturnHoldMeters of travel - so the
    /// body doesn't cut back into what it just passed.
    /// </summary>
    internal sealed class Driver
    {
        public static readonly float[] ProbeAngles = { -60f, -50f, -40f, -30f, -20f, -10f, 0f, 10f, 20f, 30f, 40f, 50f, 60f };
        static readonly float[] Offsets = { 0f, -1.5f, 1.5f, -3f, 3f, -4.5f, 4.5f, -6f, 6f };

        public const float ArriveDistance = 7f;
        const float StuckSpeedEps = 0.4f;     // m/s below which we count as not moving
        const float StuckSeconds = 3f;        // pushing but not moving this long => reverse
        const float BoxedSeconds = 1.2f;      // boxed in this long => reverse
        const float ReverseSeconds = 1.7f;
        const float StopMargin = 3f;          // keep this far from a hit
        const float BoxedDist = 4f;           // best way forward shorter than this = boxed in
        const float ReturnHoldMeters = 12f;   // way back must stay clear this far before stepping back
        // A new offset is kept at least this long unless the way it aims is
        // about to be hit: the clear test flickers frame to frame (probe rays
        // at the edge of an obstacle, the arc check from a lateral position
        // that moves as the car turns), and re-picking every frame flipped the
        // offset 0 <-> 6 m ten times a second, steering in neither direction.
        public static float OffsetHoldSec = 0.6f;
        // held while a hard stop (this decel, m/s^2) still ends short of what it aims at;
        // tuned in tools/sim: 0.6 s / 20 cut the <=0.2 s flips ~60%, 1 s hit more
        public static float OffsetHoldDecel = 20f;
        // Tuned in tools/sim with a wreck every 35-80 m: the old
        // 6 m/s avoid cap and stop-distance clearance made the car crawl past
        // every abandoned car.
        // Re-tuned with roadside fences in the sim (AD_FENCES): at 12 m/s a
        // swerve ran on into whatever lines the road.
        public static float AvoidSpeed = 8f;        // speed cap while offset from the route
        public static float AvoidHeadway = 1.8f;    // s: clearance needed = speed * this + 6 m
        public static float ObstacleDecel = 5f;     // m/s^2 assumed when capping speed for a hit ahead
        // The game only runs turbo (sprint) while moveForward != 0, and its
        // speed cap drifts 14 -> 10 m/s at 1.5 m/s^2 whenever turbo drops out.
        // Coasting at exactly 0 throttle toggled sprint every few frames and
        // kept the car slow, so coast/brake with a token throttle instead.
        const float HoldThrottle = 0.02f;
        const float CruiseThrottle = 0.25f;   // feed-forward at zero speed error
        const int CenterProbe = 6;

        public readonly PathFollower Follower = new PathFollower();
        public float Cruise = 14f;
        public float WheelBase = 2.6f;
        public float SteerMax = 30f;
        public float HalfWidth = 1f;          // vehicle half width, m
        /// <summary>Road test at a world point (x, z); null = no road map.</summary>
        public System.Func<float, float, bool> IsRoad;
        public static float OffRoadOffsetCost = 4f;   // avoidance: cost of an offset that leaves the road
        public static float StraightNeedMax = 10f;    // straight-probe clearance required, capped (m)

        public float Offset { get; private set; }   // lateral offset from the route, m (+ = right)
        public int StuckCount { get; private set; }
        public bool Boxed { get; private set; }
        public float Desired { get; private set; }
        public float Alpha { get; private set; }
        public bool Reversing => time < reverseUntil;

        float holdLeft;
        float offsetAt = -100f;   // when Offset last changed
        float preferSide;
        float reverseUntil = -1f, reverseSteer;
        float stuckTimer, boxedTimer;
        float time;

        public struct Output
        {
            public float Throttle;     // -1..1 (negative = reverse)
            public float Steer;        // -1..1 absolute wheel target (+ = right)
            public bool Brake;
            public bool Arrived;       // stopped at the destination
            public bool StuckEvent;    // started an unstick reverse this frame
        }

        public void Reset()
        {
            Follower.Clear();
            Offset = 0f;
            holdLeft = 0f;
            preferSide = 0f;
            reverseUntil = -1f;
            stuckTimer = boxedTimer = 0f;
            StuckCount = 0;
            Boxed = false;
            hitMap.Clear();
        }

        public void ResetStuck()
        {
            StuckCount = 0;
        }

        /// <summary>Switch to a new route; drops any avoidance offset.</summary>
        public void SetPath(System.Collections.Generic.List<Vector3> path, Vector3 pos,
            System.Func<Vector3, float> speedCap)
        {
            Follower.SetPath(path, pos, Cruise, speedCap);
            Offset = 0f;
            holdLeft = 0f;
        }

        const float HitMemorySec = 6f;        // obstacle contact points kept this long
        const float PathClearMargin = 0.5f;   // body-to-obstacle gap wanted along the planned arc

        struct Hit { public Vector3 P; public float T; }
        // contact points on a 1 m grid (one per cell, latest wins)
        readonly System.Collections.Generic.Dictionary<long, Hit> hitMap =
            new System.Collections.Generic.Dictionary<long, Hit>();
        readonly System.Collections.Generic.List<Vector3> nearHits = new System.Collections.Generic.List<Vector3>();
        readonly System.Collections.Generic.List<long> staleKeys = new System.Collections.Generic.List<long>();
        float lastPrune;

        /// <summary>
        /// Where a probe actually touched an obstacle (world space). Used to
        /// check the curved path each avoidance offset would really drive -
        /// straight probe rays miss a wreck sitting on the inside of a bend -
        /// and to close off the obstacle's extent when detouring.
        /// </summary>
        public void AddObstaclePoint(Vector3 p, float now)
        {
            long key = ((long)Mathf.FloorToInt(p.x) << 32) ^ (uint)Mathf.FloorToInt(p.z);
            hitMap[key] = new Hit { P = new Vector3(p.x, 0f, p.z), T = now };
            if (now - lastPrune > 1f)
            {
                lastPrune = now;
                staleKeys.Clear();
                foreach (var kv in hitMap)
                {
                    if (now - kv.Value.T > HitMemorySec) staleKeys.Add(kv.Key);
                }
                foreach (var k in staleKeys) hitMap.Remove(k);
            }
        }

        /// <summary>Obstacle contact points from the last few seconds.</summary>
        public void RecentHits(System.Collections.Generic.List<Vector3> outPts)
        {
            foreach (var h in hitMap.Values)
            {
                if (time - h.T <= HitMemorySec) outPts.Add(h.P);
            }
        }

        public static float ProbeRange(float speed)
        {
            return Mathf.Clamp(10f + 1.6f * speed, 12f, 32f);
        }

        /// <summary>Side probes reach less far: they only matter once we turn.</summary>
        public static float ProbeReach(int i, float range)
        {
            return range * Mathf.Lerp(1f, 0.6f, Mathf.Abs(ProbeAngles[i]) / 60f);
        }

        /// <param name="free">free distance per ProbeAngles entry (range when clear)</param>
        public Output Step(Vector3 pos, Vector3 fwd, float fwdSpeed, float speed, Vector3 dest,
            float[] free, float range, float now, float dt)
        {
            time = now;
            var o = new Output();
            Follower.Update(pos);

            float dist = HDist(pos, dest);
            if (dist < ArriveDistance)
            {
                o.Brake = true;
                o.Arrived = speed <= 0.6f;
                return o;
            }

            if (Reversing)
            {
                o.Throttle = -0.7f;
                o.Steer = reverseSteer;
                return o;
            }

            // --- obstacle avoidance: choose the lateral offset ---
            // clearance to swerve around something, not to stop before it
            float need = Mathf.Clamp(speed * AvoidHeadway + 6f, 10f, range);
            PrepareArcCheck(pos, speed, need);
            routeOnRoad = IsRoad != null && OffsetOnRoad(0f, speed);
            bool clearNow = Clear(Offset, pos, fwd, speed, free, need);
            if (!clearNow && time - offsetAt < OffsetHoldSec &&
                FreeToward(Offset, pos, fwd, speed, free) >= StopMargin + speed * speed / (2f * OffsetHoldDecel))
            {
                holdLeft = ReturnHoldMeters;   // just picked: give it time to take effect
            }
            else if (!clearNow)
            {
                bool found = false;
                float bestCost = float.MaxValue, best = Offset;
                for (int i = 0; i < Offsets.Length; i++)
                {
                    float c = Offsets[i];
                    if (!Clear(c, pos, fwd, speed, free, need)) continue;
                    float cost = Mathf.Abs(c - Offset);
                    float side = Offset != 0f ? Mathf.Sign(Offset) : preferSide;
                    if (side != 0f && c != 0f && Mathf.Sign(c) != side) cost += 2f;
                    if (routeOnRoad && !OffsetOnRoad(c, speed)) cost += OffRoadOffsetCost;
                    if (cost < bestCost) { bestCost = cost; best = c; found = true; }
                }
                if (!found)
                {
                    // nothing clears the stopping distance: aim at the roomiest gap
                    float bestFree = -1f;
                    for (int i = 0; i < Offsets.Length; i++)
                    {
                        float f = FreeToward(Offsets[i], pos, fwd, speed, free);
                        if (f > bestFree + 0.5f) { bestFree = f; best = Offsets[i]; }
                    }
                }
                SetOffset(best);
                holdLeft = ReturnHoldMeters;
            }
            else if (Offset != 0f)
            {
                float back = Mathf.Sign(Offset) * Mathf.Max(0f, Mathf.Abs(Offset) - 1.5f);
                if (Clear(back, pos, fwd, speed, free, need)) holdLeft -= speed * dt;
                else holdLeft = ReturnHoldMeters;
                if (holdLeft <= 0f)
                {
                    SetOffset(back);
                    holdLeft = ReturnHoldMeters * 0.3f;
                }
            }
            if (Offset == 0f && Follower.CrossTrack < 1.5f) preferSide = 0f;

            float offFree = FreeToward(Offset, pos, fwd, speed, free);
            float bestAny = offFree;
            for (int i = 0; i < Offsets.Length; i++) bestAny = Mathf.Max(bestAny, FreeToward(Offsets[i], pos, fwd, speed, free));
            Boxed = bestAny < BoxedDist;

            // --- path tracking ---
            float alpha;
            Vector3 target;
            float wheelAngle = Follower.SteerAngle(pos, fwd, speed, WheelBase, Offset, out alpha, out target);
            Alpha = alpha;

            float desired = Mathf.Min(Cruise, Follower.SpeedLimit(Cruise));
            desired = Mathf.Min(desired, Mathf.Max(2.5f, dist * 0.35f));
            float absAlpha = Mathf.Abs(alpha);
            if (absAlpha > 60f) desired = Mathf.Min(desired, 3f);      // e.g. just after a U-turn plan
            else if (absAlpha > 30f) desired = Mathf.Min(desired, 6f);
            if (Offset != 0f) desired = Mathf.Min(desired, AvoidSpeed);
            // speed is bounded by the way we're actually heading; the straight-
            // ahead probe only counts when something is right on the bumper
            // (floored so the car can still creep round it while turning away -
            // a hard 0 here parked it for good, since it wasn't "boxed")
            float capFree = free[CenterProbe] < BoxedDist + 3f
                ? Mathf.Min(offFree, Mathf.Max(free[CenterProbe], StopMargin + 0.5f)) : offFree;
            desired = Mathf.Min(desired, Mathf.Sqrt(2f * ObstacleDecel * Mathf.Max(0f, capFree - StopMargin)));
            Desired = desired;

            o.Steer = Mathf.Clamp(wheelAngle / Mathf.Max(5f, SteerMax), -1f, 1f);

            // --- speed control ---
            float err = desired - fwdSpeed;
            if (desired < 0.3f || Boxed)
            {
                o.Brake = speed > 0.3f;
                o.Throttle = o.Brake && fwdSpeed > 0f ? HoldThrottle : 0f;
            }
            else if (err < -3f)
            {
                o.Brake = true;
                o.Throttle = HoldThrottle;
            }
            else
            {
                o.Throttle = Mathf.Clamp(CruiseThrottle + err * 0.4f, HoldThrottle, 1f);
            }

            // --- stuck detection: pushing but not moving, or boxed in ---
            // not moving for StuckSeconds counts whatever the reason (pushing
            // against something, or holding for a speed cap that never lifts)
            stuckTimer = fwdSpeed < StuckSpeedEps ? stuckTimer + dt : 0f;
            boxedTimer = Boxed ? boxedTimer + dt : 0f;
            if (stuckTimer > StuckSeconds || boxedTimer > BoxedSeconds)
            {
                StuckCount++;
                StartReverse(o.Steer, ReverseSeconds);
                o.StuckEvent = true;
                o.Throttle = -0.7f;
                o.Steer = reverseSteer;
                o.Brake = false;
            }
            return o;
        }

        /// <summary>
        /// Back up steering the rear away from where we were heading; next
        /// time forward, prefer passing on the other side.
        /// </summary>
        public void StartReverse(float steer, float seconds)
        {
            float side = Mathf.Abs(steer) > 0.1f ? Mathf.Sign(steer)
                : Offset != 0f ? Mathf.Sign(Offset) : (preferSide != 0f ? preferSide : 1f);
            reverseSteer = -side * 0.8f;
            reverseUntil = time + seconds;
            preferSide = -side;
            Offset = 0f;
            holdLeft = 0f;
            stuckTimer = boxedTimer = 0f;
        }

        bool routeOnRoad;

        void SetOffset(float o)
        {
            if (o != Offset) offsetAt = time;
            Offset = o;
        }

        /// <summary>Would the body's outer edge stay on the road at this offset?</summary>
        bool OffsetOnRoad(float offset, float speed)
        {
            if (IsRoad == null || !Follower.HasPath) return true;
            float edge = offset + (offset >= 0f ? HalfWidth : -HalfWidth);
            for (int k = 1; k <= 2; k++)
            {
                float s = Follower.Progress + PathFollower.LookAhead(speed) * k * 0.5f;
                Vector3 p = Follower.PointAt(s) + Follower.RightAt(s) * edge;
                if (!IsRoad(p.x, p.z)) return false;
            }
            return true;
        }

        bool Clear(float offset, Vector3 pos, Vector3 fwd, float speed, float[] free, float need)
        {
            // the straight probe toward the pursuit point only matches the
            // route near the car; on a bend its far end sweeps the roadside,
            // so beyond that the recorded hit points are checked along the arc
            float straight = Mathf.Min(need, StraightNeedMax);
            return FreeToward(offset, pos, fwd, speed, free) >= straight && ArcClear(offset, speed);
        }

        // route samples ahead, reused for every candidate offset this frame
        const float ArcStep = 1.5f;
        readonly Vector3[] arcP = new Vector3[40];
        readonly Vector3[] arcR = new Vector3[40];
        readonly float[] arcD = new float[40];
        int arcN;
        float arcLat0;

        void PrepareArcCheck(Vector3 pos, float speed, float need)
        {
            nearHits.Clear();
            float r2 = (need + 6f) * (need + 6f);
            foreach (var h in hitMap.Values)
            {
                if (time - h.T > HitMemorySec) continue;
                float dx = h.P.x - pos.x, dz = h.P.z - pos.z;
                if (dx * dx + dz * dz < r2) nearHits.Add(h.P);
            }
            arcN = 0;
            if (nearHits.Count == 0 || !Follower.HasPath) return;
            float s0 = Follower.Progress;
            Vector3 right0 = Follower.RightAt(s0);
            Vector3 rel = pos - Follower.PointAt(s0);
            arcLat0 = rel.x * right0.x + rel.z * right0.z;   // signed: + = right of the route
            for (float d = 2f; d <= need && arcN < arcP.Length; d += ArcStep)
            {
                arcP[arcN] = Follower.PointAt(s0 + d);
                arcR[arcN] = Follower.RightAt(s0 + d);
                arcD[arcN] = d;
                arcN++;
            }
        }

        /// <summary>
        /// Would the body, easing from where it is now to this offset over the
        /// look-ahead distance and then following the (curved) route at that
        /// offset, pass within PathClearMargin of a known obstacle point?
        /// </summary>
        bool ArcClear(float offset, float speed)
        {
            if (arcN == 0) return true;
            float la = Follower.LookDistance(speed);   // where the pursuit point really is
            float rr = HalfWidth + PathClearMargin;
            rr *= rr;
            for (int i = 0; i < arcN; i++)
            {
                float o = Mathf.Lerp(arcLat0, offset, Mathf.Clamp01(arcD[i] / la));
                float px = arcP[i].x + arcR[i].x * o, pz = arcP[i].z + arcR[i].z * o;
                for (int j = 0; j < nearHits.Count; j++)
                {
                    float dx = nearHits[j].x - px, dz = nearHits[j].z - pz;
                    if (dx * dx + dz * dz < rr) return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Free distance toward the pursuit target for a given offset: the
        /// smaller of the two probes bracketing its bearing (conservative).
        /// </summary>
        float FreeToward(float offset, Vector3 pos, Vector3 fwd, float speed, float[] free)
        {
            Vector3 to = Follower.OffsetTarget(speed, offset) - pos;
            to.y = 0f;
            float ang = to.sqrMagnitude > 1e-4f ? Vector3.SignedAngle(fwd, to, Vector3.up) : 0f;
            float a0 = ProbeAngles[0], step = ProbeAngles[1] - ProbeAngles[0];
            float fi = Mathf.Clamp((ang - a0) / step, 0f, ProbeAngles.Length - 1);
            int lo = Mathf.FloorToInt(fi), hi = Mathf.Min(lo + 1, ProbeAngles.Length - 1);
            if (fi - lo < 0.2f) return free[lo];
            if (hi - fi < 0.2f) return free[hi];
            return Mathf.Min(free[lo], free[hi]);
        }

        static float HDist(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x, dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }
    }
}
