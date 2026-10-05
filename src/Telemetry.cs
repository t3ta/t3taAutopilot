using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Drive recorder. While the local player sits in a driver's seat -
    /// autopilot or manual - it writes one session per vehicle ride to
    /// &lt;mod&gt;/telemetry:
    ///  - &lt;id&gt;.csv          samples at SampleHz: pose, velocity, yaw/pitch/roll
    ///                       rates, actual wheel angle, the inputs that went to
    ///                       the vehicle, obstacle probes and autopilot state
    ///  - &lt;id&gt;.events.jsonl one JSON object per event (engage, takeover, stuck,
    ///                       route planned, landing site, ...)
    /// Manual driving is recorded too: takeovers are the interesting cases,
    /// and human driving calibrates the simulator's vehicle model. Oldest
    /// sessions are deleted beyond MaxMB. Main thread only.
    /// </summary>
    internal static class Telemetry
    {
        const float SampleHz = 10f;
        const float FlushSec = 2f;

        static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
        static string dir;
        static bool enabled;
        static long maxBytes;

        static StreamWriter samples, events;
        static EntityVehicle vehicle;
        static EntityVehicle cappedVehicle;   // ride whose own record reached the cap: not recorded again until left
        static string sessionId;
        static float sessionStart, nextSample, lastFlush, lastPrune;
        const float PruneEverySec = 300f;   // a long ride must not outgrow the cap until it ends
        static readonly StringBuilder sb = new StringBuilder(512);

        public static bool Active => samples != null;
        public static EntityVehicle Vehicle => vehicle;

        public static void Configure(string modPath, AutopilotConfig cfg)
        {
            enabled = cfg.Telemetry;
            maxBytes = (long)Math.Max(10, cfg.TelemetryMaxMB) * 1024 * 1024;
            dir = Path.Combine(modPath, "telemetry");
        }

        /// <summary>Per frame from the vehicle input postfix, after the autopilot wrote its input.</summary>
        public static void OnVehicleFrame(EntityVehicle veh, EntityPlayerLocal drv)
        {
            if (!enabled || veh == null || drv == null || veh.AttachedMainEntity != drv) return;
            if (veh == cappedVehicle) return;
            if (vehicle != veh)
            {
                End("vehicle changed");
                Begin(veh);
                if (!Active) return;
            }
            float now = Time.time;
            if (now < nextSample) return;
            nextSample = now + 1f / SampleHz;
            try
            {
                WriteSample(veh, now);
                if (now - lastFlush > FlushSec)
                {
                    lastFlush = now;
                    samples.Flush();
                    events.Flush();
                }
                if (now - lastPrune > PruneEverySec)
                {
                    lastPrune = now;
                    if (Prune(sessionId) > maxBytes)
                    {
                        // this ride alone is over the cap: stop here and keep what we have
                        Log.Warning("[t3taAutopilot] telemetry: " + sessionId + " reached telemetryMaxMB, recording of this ride stopped");
                        End("size cap reached");
                        cappedVehicle = veh;
                    }
                }
            }
            catch (Exception e)
            {
                Log.Warning("[t3taAutopilot] telemetry write failed, recording stopped: " + e.Message);
                Close();
                enabled = false;
            }
        }

        /// <summary>From GameUpdate: close the session once the player is out of that seat.</summary>
        public static void Watchdog(EntityPlayerLocal p)
        {
            if (cappedVehicle != null && (p == null || p.AttachedToEntity != cappedVehicle)) cappedVehicle = null;
            if (!Active) return;
            if (p == null || vehicle == null || p.AttachedToEntity != vehicle || vehicle.AttachedMainEntity != p)
            {
                End("left vehicle");
            }
        }

        static void Begin(EntityVehicle veh)
        {
            try
            {
                Directory.CreateDirectory(dir);
                Prune(null);
                lastPrune = Time.time;
                string cls = ClassName(veh);
                string baseId = DateTime.Now.ToString("yyyyMMdd-HHmmss", Inv) + "_" + Sanitize(cls);
                sessionId = baseId;
                // re-entering within the same second must not overwrite the last ride
                for (int n = 2; File.Exists(Path.Combine(dir, sessionId + ".csv")); n++) sessionId = baseId + "-" + n;
                samples = new StreamWriter(Path.Combine(dir, sessionId + ".csv"), false, new UTF8Encoding(false), 1 << 16);
                events = new StreamWriter(Path.Combine(dir, sessionId + ".events.jsonl"), false, new UTF8Encoding(false), 1 << 14);
                vehicle = veh;
                sessionStart = Time.time;
                nextSample = 0f;
                lastFlush = Time.time;
                WriteHeader();
                Event("session_start", "vehicle", cls, "world", GamePrefs.GetString(EnumGamePrefs.GameWorld),
                    "wall_clock", DateTime.Now.ToString("o", Inv), "steer_max", veh.vehicle != null ? veh.vehicle.SteerAngleMax : 0f,
                    "gyro", veh is EntityVGyroCopter);
                Log.Out("[t3taAutopilot] telemetry: recording " + sessionId);
            }
            catch (Exception e)
            {
                // like a failed write: stop for this game session instead of
                // retrying (and warning, and creating files) every frame
                Log.Warning("[t3taAutopilot] telemetry could not start, recording stopped: " + e.Message);
                Close();
                enabled = false;
            }
        }

        public static void End(string reason)
        {
            if (!Active) return;
            Event("session_end", "reason", reason, "duration_s", Time.time - sessionStart);
            Close();
            try { Prune(null); } catch (Exception) { }
        }

        static void Close()
        {
            try { samples?.Dispose(); } catch (Exception) { }
            try { events?.Dispose(); } catch (Exception) { }
            samples = null;
            events = null;
            vehicle = null;
        }

        /// <summary>
        /// One event line. fields: alternating name, value (string, bool,
        /// number, Vector3, float[]). Position and mode are added.
        /// </summary>
        public static void Event(string kind, params object[] fields)
        {
            if (!Active) return;
            try
            {
                sb.Length = 0;
                sb.Append("{\"t\":").Append(F(Time.time - sessionStart)).Append(",\"kind\":");
                Str(kind);
                sb.Append(",\"mode\":");
                Str(AutopilotController.TelemetryMode);
                if (vehicle != null)
                {
                    sb.Append(",\"pos\":");
                    Vec(vehicle.position);
                }
                for (int i = 0; i + 1 < fields.Length; i += 2)
                {
                    sb.Append(',');
                    Str(fields[i] as string ?? "field" + i);
                    sb.Append(':');
                    Val(fields[i + 1]);
                }
                sb.Append('}');
                events.WriteLine(sb.ToString());
            }
            catch (Exception e)
            {
                Log.Warning("[t3taAutopilot] telemetry event failed: " + e.Message);
            }
        }

        static void WriteHeader()
        {
            sb.Length = 0;
            sb.Append("t,mode,x,y,z,yaw,pitch,roll,vx,vy,vz,fwd_speed,yaw_rate,pitch_rate,roll_rate," +
                      "wheel_dir,wheels_down,throttle,strafe,jump,down,running,on_road,engine,health");
            sb.Append(",desired,offset,xte,alpha,boxed,progress,route_len,dest_dist,phase,target_alt,want_pitch,agl");
            for (int i = 0; i < Driver.ProbeAngles.Length; i++) sb.Append(",free").Append(i);
            samples.WriteLine(sb.ToString());
        }

        static readonly float[] probeScratch = new float[Driver.ProbeAngles.Length];

        static void WriteSample(EntityVehicle veh, float now)
        {
            Transform pt = veh.PhysicsTransform;
            if (pt == null) return;
            var rb = veh.vehicleRB;
            var mi = veh.movementInput;
            Vector3 f = pt.forward, r = pt.right;
            float yaw = Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg;
            float pitch = Mathf.Asin(Mathf.Clamp(f.y, -1f, 1f)) * Mathf.Rad2Deg;   // + = nose up
            float roll = Mathf.Asin(Mathf.Clamp(r.y, -1f, 1f)) * Mathf.Rad2Deg;    // + = right side up
            Vector3 v = rb != null ? rb.velocity : Vector3.zero;
            // angular velocity in the vehicle frame, deg/s: x = pitch (+ nose down), y = yaw (+ right), z = roll
            Vector3 w = rb != null ? pt.InverseTransformDirection(rb.angularVelocity) * Mathf.Rad2Deg : Vector3.zero;
            var auto = AutopilotController.TelemetrySnapshot(veh, probeScratch);

            sb.Length = 0;
            sb.Append(F(now - sessionStart)).Append(',').Append(AutopilotController.TelemetryMode);
            Append(veh.position.x); Append(veh.position.y); Append(veh.position.z);
            Append(yaw); Append(pitch); Append(roll);
            Append(v.x); Append(v.y); Append(v.z);
            Append(veh.vehicle != null ? veh.vehicle.CurrentForwardVelocity : 0f);
            Append(w.y); Append(-w.x); Append(w.z);
            Append(veh.wheelDir);
            sb.Append(',').Append(veh.GetWheelsOnGround());
            Append(mi != null ? mi.moveForward : 0f);
            Append(mi != null ? mi.moveStrafe : 0f);
            sb.Append(',').Append(mi != null && mi.jump ? 1 : 0);
            sb.Append(',').Append(mi != null && mi.down ? 1 : 0);
            sb.Append(',').Append(mi != null && mi.running ? 1 : 0);
            sb.Append(',').Append(auto.OnRoad ? 1 : 0);
            sb.Append(',').Append(veh.IsEngineRunning ? 1 : 0);
            sb.Append(',').Append(veh.vehicle != null ? veh.vehicle.GetHealth() : 0);
            if (auto.Driving)
            {
                Append(auto.Desired); Append(auto.Offset); Append(auto.Xte); Append(auto.Alpha);
                sb.Append(',').Append(auto.Boxed ? 1 : 0);
                Append(auto.Progress); Append(auto.RouteLen);
            }
            else sb.Append(",,,,,,,");
            if (auto.Engaged) Append(auto.DestDist); else sb.Append(',');
            sb.Append(',').Append(auto.Phase ?? "");
            if (auto.Flying) { Append(auto.TargetAlt); Append(auto.WantPitch); Append(auto.Agl); }
            else sb.Append(",,,");
            for (int i = 0; i < probeScratch.Length; i++)
            {
                if (auto.HaveProbes) Append(probeScratch[i]); else sb.Append(',');
            }
            samples.WriteLine(sb.ToString());
        }

        /// <summary>
        /// Delete the oldest sessions while the folder exceeds maxBytes. The
        /// session being recorded (keep) and the newest other one (the ride
        /// that just ended) are never deleted.
        /// </summary>
        /// <returns>bytes of the keep session on disk</returns>
        static long Prune(string keep)
        {
            // a session is ID.csv + ID.events.jsonl: delete both together, never
            // leave half of one behind
            var sessions = new Dictionary<string, List<FileInfo>>();
            long total = 0;
            foreach (var fi in new DirectoryInfo(dir).GetFiles("*"))
            {
                total += fi.Length;
                int dot = fi.Name.IndexOf('.');
                string id = dot > 0 ? fi.Name.Substring(0, dot) : fi.Name;
                List<FileInfo> group;
                if (!sessions.TryGetValue(id, out group)) sessions[id] = group = new List<FileInfo>();
                group.Add(fi);
            }
            long keepBytes = 0;
            List<FileInfo> kept;
            if (keep != null && sessions.TryGetValue(keep, out kept))
            {
                foreach (var fi in kept) keepBytes += fi.Length;
                sessions.Remove(keep);
            }
            if (total <= maxBytes) return keepBytes;
            var order = new List<List<FileInfo>>(sessions.Values);
            order.Sort((a, b) => Newest(a).CompareTo(Newest(b)));
            if (order.Count > 0) order.RemoveAt(order.Count - 1);
            foreach (var group in order)
            {
                if (total <= maxBytes) break;
                foreach (var fi in group)
                {
                    total -= fi.Length;
                    try { fi.Delete(); } catch (Exception) { }
                }
            }
            return keepBytes;
        }

        static DateTime Newest(List<FileInfo> group)
        {
            var t = DateTime.MinValue;
            foreach (var fi in group) if (fi.LastWriteTimeUtc > t) t = fi.LastWriteTimeUtc;
            return t;
        }

        static string ClassName(EntityVehicle veh)
        {
            try { return EntityClass.list[veh.entityClass].entityClassName; }
            catch (Exception) { return veh.GetType().Name; }
        }

        static string Sanitize(string s)
        {
            var b = new StringBuilder(s.Length);
            foreach (char c in s) b.Append(char.IsLetterOrDigit(c) || c == '-' || c == '_' ? c : '_');
            return b.ToString();
        }

        static string F(float x) => float.IsNaN(x) || float.IsInfinity(x) ? "" : x.ToString("0.###", Inv);
        static void Append(float x) { sb.Append(',').Append(F(x)); }

        static void Str(string s)
        {
            sb.Append('"');
            foreach (char c in s ?? "")
            {
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4", Inv));
                        else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }

        static void Vec(Vector3 v)
        {
            sb.Append('[').Append(F(v.x)).Append(',').Append(F(v.y)).Append(',').Append(F(v.z)).Append(']');
        }

        static void Val(object o)
        {
            switch (o)
            {
                case null: sb.Append("null"); break;
                case string s: Str(s); break;
                case bool b: sb.Append(b ? "true" : "false"); break;
                case float x: { string t = F(x); sb.Append(t.Length > 0 ? t : "null"); break; }
                case double d: { string t = F((float)d); sb.Append(t.Length > 0 ? t : "null"); break; }
                case int i: sb.Append(i.ToString(Inv)); break;
                case Vector3 v: Vec(v); break;
                case float[] arr:
                    sb.Append('[');
                    for (int i = 0; i < arr.Length; i++)
                    {
                        if (i > 0) sb.Append(',');
                        string t = F(arr[i]);
                        sb.Append(t.Length > 0 ? t : "null");
                    }
                    sb.Append(']');
                    break;
                default: Str(Convert.ToString(o, Inv)); break;
            }
        }
    }
}
