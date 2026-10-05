using System;
using System.Collections.Generic;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Tracks a dense route polyline: keeps a monotonic progress along it,
    /// gives a pure-pursuit steering angle, and a speed limit from the
    /// curvature ahead (so the vehicle brakes BEFORE a bend - the game cuts
    /// steering rate to 15% above ~10 m/s, so a car can't turn at speed).
    /// Pure logic (no engine calls) so tools/sim can run it.
    /// </summary>
    internal sealed class PathFollower
    {
        public static float LatAccel = 3f;        // m/s^2 allowed in bends (config: cornerAccel)
        public const float BrakeDecel = 3.5f;     // m/s^2 planning deceleration
        const float MinBendSpeed = 3.5f;
        const float CurvatureSpan = 3;            // points each side for curvature (x Spacing m)
        const float SearchBehind = 6f;            // meters of path searched behind progress
        const float SearchAhead = 40f;            // ... and ahead

        List<Vector3> pts;
        float[] cum;          // cumulative length at each point
        float[] vAllow;       // speed allowed at each point (curvature / off-road)
        int seg;              // current segment pts[seg] -> pts[seg+1]
        float progress;       // meters along the path

        public bool HasPath => pts != null && pts.Count >= 2;
        public float Progress => progress;
        public float Length => HasPath ? cum[cum.Length - 1] : 0f;
        public float Remaining => Length - progress;
        public float CrossTrack { get; private set; }   // distance to the path
        public List<Vector3> Points => pts;

        /// <param name="speedCap">per-point speed cap (off-road legs); may be null</param>
        public void SetPath(List<Vector3> path, Vector3 pos, float cruise, Func<Vector3, float> speedCap)
        {
            if (path == null || path.Count == 0)
            {
                pts = null;
                return;
            }
            if (path.Count == 1)
            {
                path = new List<Vector3> { new Vector3(pos.x, 0f, pos.z), path[0] };
            }
            pts = path;
            int n = pts.Count;
            cum = new float[n];
            for (int i = 1; i < n; i++)
            {
                cum[i] = cum[i - 1] + HDist(pts[i - 1], pts[i]);
            }

            var curv = new float[n];
            int k = (int)CurvatureSpan;
            for (int i = 0; i < n; i++)
            {
                int a = Math.Max(0, i - k), b = Math.Min(n - 1, i + k);
                if (a == i || b == i) continue;
                Vector3 d0 = pts[i] - pts[a], d1 = pts[b] - pts[i];
                d0.y = 0f; d1.y = 0f;
                float arc = 0.5f * (cum[b] - cum[a]);
                if (arc < 0.5f || d0.sqrMagnitude < 1e-4f || d1.sqrMagnitude < 1e-4f) continue;
                float dth = Vector3.Angle(d0, d1) * Mathf.Deg2Rad;
                curv[i] = dth / arc;
            }
            vAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                // widen each bend by +-2 points so the limit holds through it
                float c = 0f;
                for (int j = Math.Max(0, i - 2); j <= Math.Min(n - 1, i + 2); j++) c = Mathf.Max(c, curv[j]);
                float v = c > 1e-4f ? Mathf.Sqrt(LatAccel / c) : cruise;
                v = Mathf.Clamp(v, MinBendSpeed, cruise);
                if (speedCap != null) v = Mathf.Min(v, speedCap(pts[i]));
                vAllow[i] = v;
            }
            vAllow[n - 1] = Mathf.Min(vAllow[n - 1], 2f);

            // initial progress: closest point within the first stretch of the
            // new path (plans start at the vehicle, loops may pass near it later)
            seg = 0;
            progress = 0f;
            Locate(pos, 0, n - 2, 120f);
        }

        public void Clear()
        {
            pts = null;
        }

        /// <summary>Advance progress to the projection of pos (windowed search).</summary>
        public void Update(Vector3 pos)
        {
            if (!HasPath) return;
            int lo = seg, hi = seg;
            while (lo > 0 && progress - cum[lo] < SearchBehind) lo--;
            while (hi < pts.Count - 2 && cum[hi] - progress < SearchAhead) hi++;
            Locate(pos, lo, hi, float.MaxValue);
        }

        void Locate(Vector3 pos, int lo, int hi, float maxAlong)
        {
            float best = float.MaxValue;
            int bestSeg = seg;
            float bestProg = progress;
            for (int i = lo; i <= hi && i < pts.Count - 1; i++)
            {
                if (cum[i] > maxAlong) break;
                Vector3 a = pts[i], b = pts[i + 1];
                float ax = b.x - a.x, az = b.z - a.z;
                float len2 = ax * ax + az * az;
                float t = len2 > 1e-6f ? ((pos.x - a.x) * ax + (pos.z - a.z) * az) / len2 : 0f;
                t = Mathf.Clamp01(t);
                float px = a.x + ax * t - pos.x, pz = a.z + az * t - pos.z;
                float d2 = px * px + pz * pz;
                if (d2 < best)
                {
                    best = d2;
                    bestSeg = i;
                    bestProg = cum[i] + (cum[i + 1] - cum[i]) * t;
                }
            }
            seg = bestSeg;
            progress = bestProg;
            CrossTrack = Mathf.Sqrt(best);
        }

        public Vector3 PointAt(float s)
        {
            int n = pts.Count;
            if (s <= 0f) return pts[0];
            if (s >= cum[n - 1]) return pts[n - 1];
            int i = seg;
            while (i > 0 && cum[i] > s) i--;
            while (i < n - 2 && cum[i + 1] < s) i++;
            float l = cum[i + 1] - cum[i];
            float t = l > 1e-5f ? (s - cum[i]) / l : 0f;
            return Vector3.Lerp(pts[i], pts[i + 1], t);
        }

        /// <summary>The path from s to the end (starting exactly at PointAt(s)).</summary>
        public List<Vector3> Tail(float s)
        {
            var outp = new List<Vector3> { PointAt(s) };
            for (int i = 0; i < pts.Count; i++)
            {
                if (cum[i] > s + 0.5f) outp.Add(pts[i]);
            }
            return outp;
        }

        /// <summary>Unit vector pointing to the RIGHT of the path direction at s.</summary>
        public Vector3 RightAt(float s)
        {
            Vector3 t = PointAt(s + 1.5f) - PointAt(s - 1.5f);
            t.y = 0f;
            if (t.sqrMagnitude < 1e-6f) return Vector3.zero;
            t.Normalize();
            return new Vector3(t.z, 0f, -t.x);
        }

        // 0.55 / 12 m wove at speed: the real rigid body lags the wheel, so a
        // short look-ahead overshoots (tools/sim AD_YAW_TAU)
        public static float LookGain = 1.0f, LookMax = 18f;

        /// <summary>Pure-pursuit look-ahead distance for the current speed.</summary>
        public static float LookAhead(float speed)
        {
            return Mathf.Clamp(3.5f + LookGain * speed, 5f, LookMax);
        }

        /// <summary>
        /// Pure pursuit: front-wheel angle (degrees, + = right) that arcs the
        /// vehicle through the look-ahead point. alphaDeg = heading error to it.
        /// </summary>
        public float SteerAngle(Vector3 pos, Vector3 fwd, float speed, float wheelBase, float lateralOffset,
            out float alphaDeg, out Vector3 target)
        {
            target = OffsetTarget(speed, lateralOffset);
            return PursuitAngle(pos, fwd, target, wheelBase, out alphaDeg);
        }

        // In a tight corner the straight line to a speed-based look-ahead
        // point runs well inside the bend: a 100 deg town corner at 6 m/s put
        // the 4x4 3.7 m inside the route, into the stop sign and the fence on
        // the corner. The look-ahead is shortened until that line stays
        // within MaxCornerCut of the path (0 = off).
        public static float MaxCornerCut = 1.0f;   // sim: 1.0 cut the least on the recorded town trips
        const float LookMinCorner = 4f;
        float cutCacheProgress = float.NaN, cutCacheSpeed, cutCacheLimit, cutCacheLook;

        /// <summary>LookAhead(speed), shortened so the chord doesn't cut the path's bends.</summary>
        public float LookDistance(float speed)
        {
            float look = LookAhead(speed);
            if (MaxCornerCut <= 0f || !HasPath) return look;
            if (progress == cutCacheProgress && speed == cutCacheSpeed && MaxCornerCut == cutCacheLimit) return cutCacheLook;
            Vector3 a = PointAt(progress);
            while (look > LookMinCorner && ChordCut(a, look) > MaxCornerCut)
            {
                look = Mathf.Max(LookMinCorner, look * 0.8f);
            }
            cutCacheProgress = progress;
            cutCacheSpeed = speed;
            cutCacheLimit = MaxCornerCut;
            cutCacheLook = look;
            return look;
        }

        /// <summary>Largest distance of the path between progress and progress + look from the straight chord.</summary>
        float ChordCut(Vector3 a, float look)
        {
            Vector3 b = PointAt(progress + look);
            float cx = b.x - a.x, cz = b.z - a.z;
            float len = Mathf.Sqrt(cx * cx + cz * cz);
            if (len < 0.5f) return 0f;
            cx /= len;
            cz /= len;
            float worst = 0f;
            for (float d = 1f; d < look; d += 1f)
            {
                Vector3 p = PointAt(progress + d);
                float off = Mathf.Abs((p.x - a.x) * cz - (p.z - a.z) * cx);
                if (off > worst) worst = off;
            }
            return worst;
        }

        /// <summary>Look-ahead point shifted sideways (+ = right) off the path.</summary>
        public Vector3 OffsetTarget(float speed, float lateralOffset)
        {
            float s = progress + LookDistance(speed);
            Vector3 t = PointAt(s);
            if (lateralOffset != 0f) t += RightAt(s) * lateralOffset;
            return t;
        }

        public static float PursuitAngle(Vector3 pos, Vector3 fwd, Vector3 target, float wheelBase,
            out float alphaDeg)
        {
            Vector3 to = target - pos;
            to.y = 0f;
            float ld = Mathf.Max(to.magnitude, 1f);
            alphaDeg = Vector3.SignedAngle(fwd, to, Vector3.up);
            if (Mathf.Abs(alphaDeg) >= 90f)
            {
                return Mathf.Sign(alphaDeg) * 90f;   // target behind: full lock
            }
            float a = alphaDeg * Mathf.Deg2Rad;
            return Mathf.Atan(2f * wheelBase * Mathf.Sin(a) / ld) * Mathf.Rad2Deg;
        }

        /// <summary>
        /// Highest speed from which every upcoming point's allowed speed can
        /// still be reached braking at BrakeDecel.
        /// </summary>
        public float SpeedLimit(float cruise)
        {
            if (!HasPath) return cruise;
            float horizon = cruise * cruise / (2f * BrakeDecel) + 10f;
            float lim = cruise;
            for (int i = seg; i < pts.Count; i++)
            {
                float ds = cum[i] - progress;
                if (ds > horizon) break;
                float v = Mathf.Sqrt(vAllow[i] * vAllow[i] + 2f * BrakeDecel * Mathf.Max(0f, ds));
                if (v < lim) lim = v;
            }
            return lim;
        }

        static float HDist(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x, dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }
    }
}
