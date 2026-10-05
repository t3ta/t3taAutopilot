using UnityEngine;

namespace T3taAutopilot
{
    /// <summary>
    /// Gyrocopter autopilot: flies a straight line to the destination, high
    /// enough to clear whatever the controller measured ahead, then slows
    /// down, sinks and lands. Pure logic - AutopilotController measures the
    /// ground with downward raycasts, tools/sim flies it against a
    /// model of the game's gyrocopter.
    ///
    /// The game's gyrocopter (vehicles.xml, all data driven):
    ///  - lift comes from the top rotor, whose rpm follows FORWARD SPEED; at
    ///    ~4 m/s and up it is at max and lift is just under gravity;
    ///  - moveForward spins the pusher prop (thrust along the nose);
    ///  - moveStrafe is a yaw torque, jump pitches the nose up (and brakes
    ///    the wheels), down pitches it down.
    /// So altitude is held with pitch (thrust tilted up/down), and the only
    /// way to lose lift is to slow down. There are no brakes in the air.
    /// </summary>
    internal sealed class Pilot
    {
        public const float Ceiling = 250f;        // lift fades out at y 260
        // The downward rays miss most trees, so the ground margin is what
        // keeps us out of the canopy (tall pines reach ~30 m).
        const float CruiseClearance = 25f;        // above the highest thing measured ahead
        const float MinAboveGround = 45f;
        const float ApproachDist = 110f;          // grounded this close: taxi instead of flying
        // With the throttle off the gyro sinks at only ~2.4 m/s whatever the
        // pitch (the rotor lifts just under gravity as long as it's moving).
        // Recorded manual landings slowed down first, then sank at ~2 m/s at
        // 4-7 m/s: a ~0.35 slope, touching down at ~2-4 m/s. But that slope
        // from cruise height starts 500-600 m out: recorded approaches crept
        // down at 9 m/s and were taken over 250-450 m out. Nose down WITH the
        // throttle on the pusher drives it down: recorded 10.7 m/s sinking
        // 6-8 m/s at -17 deg. So: dive on power, then slow down for a short
        // unpowered glide.
        public static float GlideSlope = 0.28f;   // m of height per m of distance on the final glide
        public static float ApproachSpeed = 7f;   // on the glide: ~2 m/s sink at 0.28
        public static float DescentSlope = 0.5f;  // powered dive down to the final glide
        public static float FinalHeight = 30f;    // the final glide starts this high over the destination
        const float DescentSpeed = 11f;           // on the dive (recorded 10.7 at full throttle, -17 deg)
        const float DescentSink = 7f;             // m/s asked for at most on the dive
        const float DescentLead = 40f;            // pitching over and building up the sink takes this far
        const float FinalSpeed = 4.5f;            // at touchdown; below ~3.6 m/s the rotor loses lift
        const float TouchdownBefore = 15f;        // aim to touch down this far short, then roll and brake
        const float TaxiSpeed = 5f;               // short trips stay on the wheels
        const float TakeoffSpeed = 7f;            // no pitch-up (= wheel brake) below this on the ground
        const float TakeoffAlign = 20f;           // turn on the wheels until the nose is this close to the course
        const float CommitRunSpeed = 5f;          // on the ground faster than this: keep the take-off run's direction
        const float MaxClimbPitch = 30f;
        const float SlowClimbPitch = 15f;         // below ~6 m/s: nose-up only bleeds speed (recorded 57 deg stalls)
        const float MaxDivePitch = -18f;
        const float CourseLead = 1f;              // nose lead per degree of course error

        // Take-off and climb-out: the controller sweeps boxes level out in
        // ClimbDirs directions at ClimbLevels over the wheels; NeedGradient
        // turns what they hit into the climb gradient that direction needs.
        // Recorded take-offs lifted off 12-32 m into the run and climbed
        // ~0.35 (16 m in the next 45 m), then ~0.5; the ones that went
        // straight at what was in front hit it on the ground roll or 3-7 m
        // up, 40-75 m out.
        public const int ClimbDirs = 24;          // 15 deg apart, clockwise from +z
        public static readonly float[] ClimbLevels = { 0.8f, 3f, 6f, 10f, 15f, 21f, 28f, 36f };
        public const float ClimbReach = 100f;
        public const float ClimbOutHeight = 40f;  // AGL: below this the climb-out directions are used
        public const float RotorLevel = 2f;       // sweep levels below this are the body's width, above the rotor's
        public const float NoseRoom = 3f;         // lining up on the wheels: no creeping forward with less room than this
        /// <summary>Free distance right in front of the nose at the body's height (set with ClimbNeed).</summary>
        public float NoseFree = float.PositiveInfinity;
        public static float MaxClimbGrad = 0.3f;
        const float ClimbBuildTime = 1.5f;        // s from lift-off to climbing at MaxClimbGrad
        // Picking a climb-out direction: each 15 deg off the course costs 1,
        // needing the full MaxClimbGrad costs GradWeight more - so a direction
        // with room to spare beats one just clear of the trees next to it.
        public static float GradWeight = 2f;
        // On the ground stick to the direction picked: keep it while it needs
        // up to KeepOnGround x MaxClimbGrad, and when nothing is climbable
        // while it's within LeastBadMargin of the least bad one.
        const float KeepOnGround = 1.5f;
        const float LeastBadMargin = 0.2f;
        const float ClimbMargin = 2f;             // over the top of what's in the way
        /// <summary>Climb gradient each direction needs (+inf = blocked); set with ClimbScanned before Step.</summary>
        public readonly float[] ClimbNeed = new float[ClimbDirs];
        /// <summary>
        /// Climb gradient the exact course (toward the destination, not rounded
        /// to a ClimbDirs direction) needs: 15 deg steps put the swept box 13 m
        /// off the real line 100 m out, and it's the real line that's flown
        /// when the course is taken. Set with ClimbNeed.
        /// </summary>
        public float CourseNeed;
        public bool ClimbScanned;
        /// <summary>On the ground with every direction blocked within the take-off run: brakes; the controller stops.</summary>
        public bool NoTakeoffRun { get; private set; }
        bool noRun;
        /// <summary>Degrees the take-off / climb-out heading is turned off the course (0 = on course).</summary>
        public float Detour { get; private set; }
        int detourDir = -1;
        float settledFor;                         // s at rest just above the ground on the approach
        const float SettleTime = 1.5f;
        float courseClearFor;                     // s the course has read climbable in a row
        const float CourseClearTime = 1f;         // detoured: back onto the course only after this long clear
        float detourBadFor;                       // s the detour picked has read too steep in a row
        const float DetourBadTime = 0.5f;         // on the wheels: pick another only after this long

        public float Cruise = 15f;                // the gyro's speed cap with the throttle on (set from the vehicle)
        const float TunedCruise = 15f;            // the stock cap the approach constants were tuned at
        public static float FfGain = 0.5f;        // share of the feed-forward pitch (1 overshot into hard touchdowns)
        public static float FlareHeight = 20f;    // below this AGL the sink rate is eased
        public static float YawSmoothing = 0.6f;  // s: time constant on the yaw command (0 = raw)
        public static float YawSlew = 1.5f;       // max change of the yaw command per second
        float yawCmd;
        /// <summary>Highest ground on the whole straight route (heightmap); NaN = unknown.</summary>
        public float RouteTop = float.NaN;
        const float RouteMargin = 50f;            // trees and buildings aren't in the heightmap

        // Overshot low down: turning back airborne near the ground circled at
        // 2-9 m AGL, 22-32 deg/s, bounced off the ground and drifted 47 m
        // (recorded). Landing straight ahead instead would put it down on
        // ground nobody checked (water, a cliff, a building). So go around:
        // below FlareHeight with the destination this far behind the course,
        // climb straight ahead to GoAroundHeight, fly out until the approach
        // has room again, then come back round at that height to the landing
        // site that was picked (and checked) for it.
        const float OvershootAngle = 90f;
        float turnBackSign;                       // turning back from > 135 deg off: this way (+1 right)
        // Only just past the destination: out at the end of a go-around over
        // high ground (capped at Ceiling, maybe only ~20 m AGL) the destination
        // is behind and low too, but that's the turn back, not an overshoot
        const float OvershootNear = 60f;
        const float GoAroundHeight = 45f;         // AGL to climb to before turning back
        const float GoAroundOut = 180f;           // fly out this far from the destination first
        // Lower than this over the destination it's a long landing, not an
        // overshoot: let it touch down (a supercharged glide floats past at
        // ~5 m; the recorded overshoots passed at 9-19 m)
        const float GoAroundMinAgl = 8f;
        const int MaxGoArounds = 2;               // then land long rather than circle for ever
        int goArounds;                            // for the current landing site
        Vector3 goAroundsFor;                     // ... this one
        bool haveGoAroundsFor;
        /// <summary>Overshot low down: climbing straight out, no turn until high and far enough (latched).</summary>
        public bool GoingAround { get; private set; }
        /// <summary>
        /// Overshot too low to go around (or out of go-arounds): landing long,
        /// straight ahead, no turn until the wheels touch; the taxi drives back (latched).
        /// </summary>
        public bool LandingLong { get; private set; }

        public float TargetAlt { get; private set; }
        public float DesiredPitch { get; private set; }
        public string Phase { get; private set; } = "";

        float lastPitch;
        bool havePitch;
        bool approaching;   // latched: descending must not flip back to cruise

        public struct Output
        {
            public float Throttle;   // 0..1 pusher prop
            public float Yaw;        // -1..1 yaw torque (+ = right)
            public bool Up;          // nose up (jump; also wheel brake)
            public bool Down;        // nose down (both together: brake without pitching)
            public bool Arrived;
            /// <summary>Came to rest on something that isn't the ground (a fence, a roof edge) on the approach: stop.</summary>
            public bool Landed;
        }

        public void Reset()
        {
            havePitch = false;
            approaching = false;
            GoingAround = false;
            LandingLong = false;
            turnBackSign = 0f;
            goArounds = 0;
            haveGoAroundsFor = false;
            detourDir = -1;
            courseClearFor = 0f;
            settledFor = 0f;
            detourBadFor = 0f;
            NoseFree = float.PositiveInfinity;
            Detour = 0f;
            NoTakeoffRun = false;
            yawCmd = 0f;
            Phase = "";
        }

        /// <param name="fwd">nose direction (3D, unit)</param>
        /// <param name="vel">world velocity</param>
        /// <param name="yawRateDeg">yaw rate, deg/s (+ = turning right)</param>
        /// <param name="groundBelow">ground / roof height under the craft</param>
        /// <param name="topAhead">highest surface measured along the way ahead</param>
        /// <param name="destGround">ground height at the destination</param>
        /// <param name="grounded">wheels touching</param>
        public Output Step(Vector3 pos, Vector3 fwd, Vector3 vel, float yawRateDeg, Vector3 dest,
            float groundBelow, float topAhead, float destGround, bool grounded, float dt)
        {
            var o = new Output();
            // a new landing site (destination picked again mid-flight): its own go-arounds
            if (!haveGoAroundsFor || (dest - goAroundsFor).sqrMagnitude > 20f * 20f)
            {
                goArounds = 0;
                turnBackSign = 0f;   // a new target: pick the turn-back way afresh
                goAroundsFor = dest;
                haveGoAroundsFor = true;
            }
            Vector3 to = dest - pos;
            to.y = 0f;
            float dist = to.magnitude;
            Vector3 fwdFlat = new Vector3(fwd.x, 0f, fwd.z);
            float speed = new Vector3(vel.x, 0f, vel.z).magnitude;
            float fwdSpeed = fwdFlat.sqrMagnitude > 1e-4f ? Vector3.Dot(vel, fwdFlat.normalized) : 0f;

            // --- where to be: height and speed ---
            // fly level above the highest point of the whole route; the live
            // measurements only matter where they reach higher (or without a map)
            float cruiseAlt = Mathf.Max(topAhead + CruiseClearance, groundBelow + MinAboveGround);
            if (!float.IsNaN(RouteTop)) cruiseAlt = Mathf.Max(cruiseAlt, RouteTop + RouteMargin);
            cruiseAlt = Mathf.Min(Ceiling, cruiseAlt);
            float alt, wantSpeed;
            float agl = pos.y - groundBelow;
            bool diving = false;
            // start down early enough to dive, then glide, from the height we're at
            float finalDist = FinalHeight / GlideSlope;
            // The dive runs at the speed cap (the throttle is on for the sink
            // rate), and the sink is limited: DescentSlope is tuned for the
            // stock cap of 15 m/s. Faster (a supercharger's 17.7) the same sink
            // covers more ground, so plan a proportionally flatter path starting
            // further out, or it arrives high and overflies the strip.
            float diveSlope = DescentSlope * Mathf.Min(1f, TunedCruise / Mathf.Max(Cruise, 1f));
            float approachDist = Mathf.Max(ApproachDist, TouchdownBefore + finalDist
                + Mathf.Max(0f, pos.y - destGround - FinalHeight) / diveSlope + DescentLead);
            if (dist < approachDist) approaching = true;
            else if (dist > approachDist + 60f) approaching = false;
            // released only on the wheels, or high and far enough to come back round
            // released on the wheels, or far enough out and near the go-around height
            // it can actually reach (capped at Ceiling over high ground)
            float goAroundAlt = Mathf.Min(Ceiling, Mathf.Max(groundBelow + GoAroundHeight, topAhead + CruiseClearance));
            if (GoingAround && (grounded || (dist > GoAroundOut && pos.y > goAroundAlt - 10f)))
                GoingAround = false;
            if (LandingLong && grounded) LandingLong = false;   // on the wheels: taxi back
            if (LandingLong)
            {
                // straight ahead down to the ground, over whatever stands up ahead
                Phase = "approach";
                alt = LandLongAlt(groundBelow, topAhead);
                wantSpeed = FinalSpeed;
            }
            else if (GoingAround)
            {
                Phase = "goaround";
                // straight out and up, over whatever stands up ahead
                alt = goAroundAlt;
                wantSpeed = Cruise;
            }
            else if (grounded && dist < ApproachDist)
            {
                // close enough to drive there (or already landed and rolling)
                Phase = "taxi";
                alt = pos.y;
                wantSpeed = Mathf.Clamp(dist * 0.3f, 0f, TaxiSpeed);
            }
            else if (!approaching || grounded)
            {
                // on the wheels further out: take off first, even inside the
                // approach distance (leaving high ground for a destination far
                // below) - "approach" cuts the throttle near the ground
                Phase = grounded ? "takeoff" : "cruise";
                alt = cruiseAlt;
                wantSpeed = Cruise;
            }
            else
            {
                Phase = "approach";
                // dive, then glide down to touch down short of the destination,
                // never below what's ahead; keep flying speed until the wheels touch
                float d = Mathf.Max(0f, dist - TouchdownBefore);
                diving = d > finalDist;
                float glide = destGround - 1f + (diving ? FinalHeight + (d - finalDist) * diveSlope : d * GlideSlope);
                // something standing up ahead (not just the landing ground): stay over it
                float floor = topAhead > destGround + 3f ? topAhead + 4f : float.MinValue;
                alt = Mathf.Min(cruiseAlt, Mathf.Max(glide, floor));
                // the dive is on power; at the final glide the throttle goes
                // off (the speed cap drops to 9, then drag): glide at
                // ApproachSpeed, ease to FinalSpeed near the end
                wantSpeed = diving ? DescentSpeed
                    : Mathf.Clamp(FinalSpeed + d * 0.05f, FinalSpeed, ApproachSpeed);
                // above the glide path: power is what brings it down, not
                // slowing down (throttle off = ~2.4 m/s sink whatever the pitch)
                float high = pos.y - Mathf.Max(glide, floor);
                if (!diving && high > 3f && agl > FlareHeight)
                    wantSpeed = Mathf.Min(DescentSpeed, wantSpeed + (high - 3f) * 0.5f);
                // overshot low down: go around (from the next step)
                Vector3 velNow = new Vector3(vel.x, 0f, vel.z);
                if (agl < FlareHeight && speed > 2f && dist > 1f && dist < OvershootNear && Vector3.Angle(velNow, to) > OvershootAngle)
                {
                    if (agl > GoAroundMinAgl && goArounds < MaxGoArounds)
                    {
                        GoingAround = true;
                        goArounds++;
                        Phase = "goaround";
                        alt = goAroundAlt;
                        wantSpeed = Cruise;
                    }
                    else
                    {
                        // too low to go around (or out of go-arounds): land long, no turn
                        LandingLong = true;
                        alt = LandLongAlt(groundBelow, topAhead);
                        wantSpeed = FinalSpeed;
                    }
                }
            }
            // (no slowing down to climb: cutting the throttle right after
            // lift-off, just above the ground, stalled it in the game)
            TargetAlt = alt;

            // --- take-off / climb-out: around what's in the way ---
            Vector3 steer = to;
            if (GoingAround || LandingLong)
            {
                // hold the course we're on: no turn back this low
                Vector3 v = new Vector3(vel.x, 0f, vel.z);
                steer = v.sqrMagnitude > 1f ? v : fwdFlat;
            }
            Detour = 0f;
            noRun = false;
            if (ClimbScanned && dist > 1f && (Phase == "takeoff" || Phase == "cruise" || Phase == "goaround") && (grounded || agl < ClimbOutHeight))
            {
                // going around, "the course" is straight ahead, not back to the destination
                int pick = PickClimbDir(GoingAround ? DirIndex(steer) : DirIndex(to), ScanHeading(vel, fwd, grounded), grounded,
                    grounded ? fwdSpeed : 0f, dt);
                if (pick >= 0)
                {
                    steer = DirVector(pick);
                    Detour = Vector3.SignedAngle(to, steer, Vector3.up);
                }
            }
            else detourDir = -1;

            // --- heading: yaw rate toward the destination ---
            // Airborne the craft slides: steer the COURSE (velocity), not the
            // nose, or it orbits the destination with the nose pointing at it.
            Vector3 velFlat = new Vector3(vel.x, 0f, vel.z);
            float yawErr = 0f;
            if (dist > 1f && fwdFlat.sqrMagnitude > 1e-4f)
            {
                float noseErr = Vector3.SignedAngle(fwdFlat, steer, Vector3.up);
                float courseErr = speed > 3f ? Vector3.SignedAngle(velFlat, steer, Vector3.up) : noseErr;
                // point the nose past the target by the course error, so the
                // thrust pulls the velocity round onto it
                yawErr = grounded ? noseErr : Mathf.Clamp(noseErr + CourseLead * courseErr, -150f, 150f);
                // Flying straight away from the target (a go-around flies out
                // exactly that way) the nose and course errors are +-180 with
                // opposite signs and cancel: no turn at all, it flew on for km
                // (simulator). Past 135 deg off, turn one way, held until round.
                if (grounded) turnBackSign = 0f;   // a fresh turn-back picks its own way
                if (!grounded && Mathf.Abs(courseErr) > 135f)
                {
                    if (turnBackSign == 0f) turnBackSign = noseErr >= 0f ? 1f : -1f;
                    yawErr = 150f * turnBackSign;
                }
                else if (Mathf.Abs(courseErr) < 90f) turnBackSign = 0f;
            }
            float wantRate = Mathf.Clamp(yawErr * 1.2f, -35f, 35f);
            float rawYaw = Mathf.Clamp((wantRate - yawRateDeg) / 25f, -1f, 1f);
            // The game banks the craft toward moveStrafe (a weak, undamped
            // spring): a chattering yaw command rocks it side to side. Airborne,
            // feed it a smooth, slew-limited command.
            if (grounded || YawSmoothing <= 0f) yawCmd = rawYaw;
            else
            {
                float target = Mathf.Lerp(yawCmd, rawYaw, Mathf.Clamp01(dt / YawSmoothing));
                yawCmd = Mathf.MoveTowards(yawCmd, target, YawSlew * dt);
            }
            o.Yaw = yawCmd;

            // --- speed: the pusher prop only pushes; drag slows us ---
            float spd = grounded ? fwdSpeed : speed;
            o.Throttle = wantSpeed <= 0f ? 0f : Mathf.Clamp(0.3f + (wantSpeed - spd) * 0.35f, 0f, 1f);
            // below where we want to be: power. The thrust is what climbs (the
            // rotor only just holds us up); at the speed cap the extra thrust
            // goes into climbing. Recorded take-offs dropped to 0.15 throttle a
            // metre up once at 15 m/s, then hit trees.
            if (Phase != "taxi" && !(grounded && Phase == "takeoff"))
                o.Throttle = Mathf.Max(o.Throttle, Mathf.Clamp01((alt - pos.y - 2f) * 0.1f));

            // --- height: pitch the thrust up or down ---
            float pitch = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f)) * Mathf.Rad2Deg;   // + = nose up
            float pitchRate = havePitch && dt > 0f ? (pitch - lastPitch) / dt : 0f;
            lastPitch = pitch;
            havePitch = true;

            // on the approach, above the flare, power dives it down faster
            float maxSink = Phase == "approach" && agl > FlareHeight ? DescentSink : 3.5f;
            float wantVy = Mathf.Clamp((alt - pos.y) * 0.4f, -maxSink, 8f);
            // flare: the last 10 m, sink ever more gently
            if (Phase == "approach" && agl < FlareHeight) wantVy = Mathf.Max(wantVy, -0.6f - 0.12f * agl);
            // feed-forward: the pitch whose thrust gives wantVy at this speed;
            // the proportional term alone settled ~1 m/s short (recorded
            // approaches sank at -2.7 m/s asked for -3.5 and overflew the strip)
            float ff = Mathf.Asin(Mathf.Clamp(wantVy / Mathf.Max(speed, 4f), -0.6f, 0.6f)) * Mathf.Rad2Deg;
            float maxPitch = Mathf.Lerp(SlowClimbPitch, MaxClimbPitch, Mathf.InverseLerp(4f, 8f, speed));
            DesiredPitch = Mathf.Clamp(FfGain * ff + (wantVy - vel.y) * 5f, MaxDivePitch, maxPitch);
            // on the dive the thrust (nose down) is what sinks us: throttle
            // for the sink rate, not the speed
            if (diving && agl > FlareHeight)
                o.Throttle = Mathf.Max(o.Throttle, Mathf.Clamp01(0.5f + (vel.y - wantVy) * 0.25f));
            float cmd = (DesiredPitch - pitch) - 0.3f * pitchRate;
            o.Up = cmd > 2f;
            o.Down = cmd < -2f;

            // Resting on something just above the ground (the wheels on a
            // fence, a block): "grounded" never comes, and just over the
            // ground the throttle is off - it sat there (recorded: 0.73 m up
            // inside a trader's fence). Stopped and not sinking: that's down.
            settledFor = Phase == "approach" && !grounded && agl < 1.5f && speed < 0.5f && Mathf.Abs(vel.y) < 0.3f
                ? settledFor + dt : 0f;
            if (settledFor > SettleTime) o.Landed = true;

            // wheels about to touch: no thrust, no nose-up - a recorded
            // landing bounced back into the air on full throttle here
            if (Phase == "approach" && agl < 1.2f)
            {
                o.Throttle = 0f;
                o.Up = false;
                o.Down = pitch > 4f;
            }

            if (Phase == "taxi")
            {
                // stay on the wheels. jump is the wheel brake but also pitches
                // the nose up (and at landing-roll speed that lifts off), so
                // brake with jump + down: their pitch torques cancel.
                bool brake = speed > wantSpeed + 0.5f;
                // turn toward it before driving at it (wheels don't steer: the
                // yaw torque turns us, tightest when slow)
                if (Mathf.Abs(yawErr) > 40f)
                {
                    o.Throttle = Mathf.Min(o.Throttle, 0.15f);
                    brake |= speed > 2.5f;
                }
                if (dist < 12f)
                {
                    o.Throttle = 0f;
                    brake = speed > 0.3f;
                    o.Arrived = speed < 1f;
                }
                o.Up = brake;
                o.Down = brake || pitch > 2f;   // keep the nose down: nose-up at speed lifts off
            }
            else if (grounded && Phase == "takeoff" && Mathf.Abs(yawErr) > TakeoffAlign)
            {
                // Line up on the wheels first. Recorded take-offs turned during
                // the run and lifted off still turning (rolling +-15 deg, 60-90
                // deg off course, a metre over whatever was beside the strip).
                // creep forward to turn only with room in front (recorded: pushed
                // into a guardrail at 0.25 for 5 s, nose up, turning 2 deg/s)
                o.Throttle = speed < 2f && NoseFree > NoseRoom ? 0.25f : 0f;
                bool brake = speed > 4f;
                o.Up = brake;
                o.Down = brake || pitch > 2f;
            }
            else if (grounded && speed < TakeoffSpeed)
            {
                o.Up = false;   // rolling: pitch-up is also the wheel brake
            }
            NoTakeoffRun = noRun && grounded;
            if (NoTakeoffRun)
            {
                // nowhere to take off to: don't run at the wall
                o.Throttle = 0f;
                o.Yaw = 0f;
                o.Up = true;
                o.Down = true;
            }
            return o;
        }

        // Loiter: the gyro can't hover (the rotor only lifts at speed), so
        // while the player picks a destination in the air it circles, holding
        // its height and a speed well above where the rotor loses lift.
        public const float LoiterSpeed = 10f;
        public const float LoiterMinAgl = 30f;    // over the tall pines
        const float LoiterRate = 20f;             // deg/s asked for (flies ~14: a ~45 m radius, simulator)

        /// <summary>
        /// Circle at LoiterRate (turn: +1 right, -1 left) holding alt (world y,
        /// never below LoiterMinAgl over groundBelow). Call Reset before the
        /// first step.
        /// </summary>
        public Output Loiter(Vector3 pos, Vector3 fwd, Vector3 vel, float yawRateDeg, float alt, float groundBelow,
            float turn, float dt)
        {
            var o = new Output();
            Phase = "loiter";
            alt = Mathf.Min(Ceiling, Mathf.Max(alt, groundBelow + LoiterMinAgl));
            TargetAlt = alt;
            float speed = new Vector3(vel.x, 0f, vel.z).magnitude;

            // a steady yaw rate, fed smooth and slew-limited as in Step
            float rawYaw = Mathf.Clamp((turn * LoiterRate - yawRateDeg) / 25f, -1f, 1f);
            if (YawSmoothing <= 0f) yawCmd = rawYaw;
            else
            {
                float target = Mathf.Lerp(yawCmd, rawYaw, Mathf.Clamp01(dt / YawSmoothing));
                yawCmd = Mathf.MoveTowards(yawCmd, target, YawSlew * dt);
            }
            o.Yaw = yawCmd;

            o.Throttle = Mathf.Clamp(0.3f + (LoiterSpeed - speed) * 0.35f, 0f, 1f);
            o.Throttle = Mathf.Max(o.Throttle, Mathf.Clamp01((alt - pos.y - 2f) * 0.1f));

            float pitch = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f)) * Mathf.Rad2Deg;   // + = nose up
            float pitchRate = havePitch && dt > 0f ? (pitch - lastPitch) / dt : 0f;
            lastPitch = pitch;
            havePitch = true;
            float wantVy = Mathf.Clamp((alt - pos.y) * 0.4f, -3.5f, 8f);
            float ff = Mathf.Asin(Mathf.Clamp(wantVy / Mathf.Max(speed, 4f), -0.6f, 0.6f)) * Mathf.Rad2Deg;
            float maxPitch = Mathf.Lerp(SlowClimbPitch, MaxClimbPitch, Mathf.InverseLerp(4f, 8f, speed));
            DesiredPitch = Mathf.Clamp(FfGain * ff + (wantVy - vel.y) * 5f, MaxDivePitch, maxPitch);
            float cmd = (DesiredPitch - pitch) - 0.3f * pitchRate;
            o.Up = cmd > 2f;
            o.Down = cmd < -2f;
            return o;
        }

        /// <summary>
        /// Direction to take off / climb out along instead of the course
        /// (index into ClimbNeed), or -1 = the course is clear. Airborne only
        /// within 90 deg of the heading: it can't turn round down low.
        /// </summary>
        int PickClimbDir(int course, int heading, bool grounded, float rolling, float dt)
        {
            // Rolling fast on the ground: take off along the run if it's
            // climbable, and turn onto the course once airborne. Switching
            // mid-run braked it at 9 m/s to turn 48 deg back onto the course,
            // then picked another way; it lifted off swerving into a wall
            // (recorded). Only a run that's blocked is given up.
            if (rolling > CommitRunSpeed && DirDiff(course, heading) > 1 && ClimbNeed[heading] <= MaxClimbGrad)
            {
                detourDir = heading;
                return heading;
            }
            // back onto the course only once it's clearly climbable, and has
            // stayed so for CourseClearTime: one sweep slips through a gap
            // between trees (recorded: clear at 0.22, blocked 3 m ahead 0.35 s
            // later - it had turned back into them and hit one 17 m up)
            float courseOk = detourDir >= 0 ? MaxClimbGrad * 0.8f : MaxClimbGrad;
            courseClearFor = CourseNeed <= courseOk ? courseClearFor + dt : 0f;
            if (CourseNeed <= courseOk && (detourDir < 0 || courseClearFor >= CourseClearTime)) { detourDir = -1; return -1; }
            int maxOff = grounded ? ClimbDirs / 2 : ClimbDirs / 4;
            // On the ground, stick to the direction picked with some margin: in a
            // cluttered yard every direction sits near the limit, and re-picking
            // as the craft turned swung it +120 -> -75 -> -150 -> +120 deg,
            // spinning 8-12 s on the wheels (recorded). Past that margin, one bad
            // sweep still doesn't swing it to the other side (recorded beside
            // guardrails: +90 / -90 / +105 / -120 deg in turn, never running): it
            // has to stay bad for DetourBadTime, and be climbable at all. Not for
            // ever though: kept at any need it ran into trees (simulator).
            // Airborne it must stay climbable.
            float keep = grounded ? MaxClimbGrad * KeepOnGround : MaxClimbGrad;
            if (detourDir >= 0 && DirDiff(detourDir, heading) <= maxOff)
            {
                detourBadFor = ClimbNeed[detourDir] <= keep ? 0f : detourBadFor + dt;
                if (ClimbNeed[detourDir] <= keep ||
                    (grounded && detourBadFor < DetourBadTime && !float.IsPositiveInfinity(ClimbNeed[detourDir])))
                    return detourDir;
            }
            detourBadFor = 0f;
            // climbable ones: least turned off the course, with the most room to spare
            int best = -1;
            float bestScore = float.MaxValue;
            for (int i = 0; i < ClimbDirs; i++)
            {
                // the grid direction nearest the course counts too: the exact course
                // is what's blocked (CourseNeed), the rounded one may be clear
                if (DirDiff(i, heading) > maxOff || ClimbNeed[i] > MaxClimbGrad) continue;
                float score = DirDiff(i, course) + GradWeight * ClimbNeed[i] / MaxClimbGrad;
                if (score < bestScore - 0.01f ||
                    (score < bestScore + 0.01f && DirDiff(i, heading) < DirDiff(best, heading)))
                {
                    best = i;
                    bestScore = score;
                }
            }
            if (best < 0)
            {
                // nothing climbable: the least bad one (course first on a tie)
                for (int i = 0; i < ClimbDirs; i++)
                {
                    if (DirDiff(i, heading) > maxOff) continue;
                    if (best < 0 || ClimbNeed[i] < ClimbNeed[best] - 0.01f) best = i;
                }
                // on the ground keep the one already taken unless another is clearly better
                if (grounded && best >= 0 && detourDir >= 0 && DirDiff(detourDir, heading) <= maxOff &&
                    ClimbNeed[detourDir] <= ClimbNeed[best] + LeastBadMargin)
                    best = detourDir;
                // blocked within the run everywhere: on the ground that's no take-off
                noRun = float.IsPositiveInfinity(CourseNeed) && (best < 0 || float.IsPositiveInfinity(ClimbNeed[best]));
                if (best < 0 || ClimbNeed[best] >= CourseNeed - 0.01f) { detourDir = -1; return -1; }
            }
            detourDir = best;
            return best;
        }

        /// <summary>
        /// Landing long: down to the ground, but never below what stands up
        /// ahead (the approach's floor: 4 m over the highest thing measured
        /// along the way, if it's more than the ground here).
        /// </summary>
        static float LandLongAlt(float groundBelow, float topAhead)
        {
            return Mathf.Max(groundBelow - 1f, topAhead > groundBelow + 3f ? topAhead + 4f : float.MinValue);
        }

        /// <summary>
        /// Heading the climb-out works from: the course over the ground when
        /// airborne and moving (it slides), else the nose.
        /// </summary>
        public static int ScanHeading(Vector3 vel, Vector3 fwd, bool grounded)
        {
            Vector3 v = new Vector3(vel.x, 0f, vel.z);
            return DirIndex(!grounded && v.magnitude > 3f ? v : new Vector3(fwd.x, 0f, fwd.z));
        }

        /// <summary>
        /// Does direction i need sweeping? On the ground every direction can be
        /// taken off along; airborne only those within 90 deg of the heading
        /// (PickClimbDir never turns further), plus the course itself.
        /// </summary>
        public static bool NeedsScan(int i, int course, int heading, bool grounded)
        {
            return grounded || i == course || DirDiff(i, heading) <= ClimbDirs / 4;
        }

        public static int DirIndex(Vector3 flat)
        {
            float deg = Mathf.Atan2(flat.x, flat.z) * Mathf.Rad2Deg;
            return Mathf.RoundToInt(Mathf.Repeat(deg, 360f) / (360f / ClimbDirs)) % ClimbDirs;
        }

        public static Vector3 DirVector(int i)
        {
            float a = i * (360f / ClimbDirs) * Mathf.Deg2Rad;
            return new Vector3(Mathf.Sin(a), 0f, Mathf.Cos(a));
        }

        static int DirDiff(int a, int b)
        {
            int d = Mathf.Abs(a - b) % ClimbDirs;
            return Mathf.Min(d, ClimbDirs - d);
        }

        /// <summary>
        /// Ground still to roll before lift-off along dir (recorded: 12-32 m from
        /// standing), or airborne the distance until the climb is fully built
        /// up. Only the speed along dir counts: rolling backwards or sideways,
        /// the run along dir starts from standing.
        /// </summary>
        public static float TakeoffRoll(bool grounded, Vector3 vel, Vector3 dir)
        {
            float along = Mathf.Max(0f, vel.x * dir.x + vel.z * dir.z);
            if (grounded) return Mathf.Max(8f, 28f - 2f * along);
            // Just off the ground the climb is still building up (recorded:
            // 0.1 at lift-off, 0.24 over the first 57 m, pitching up for ~1.5 s):
            // the distance until it climbs at MaxClimbGrad doesn't count yet.
            float grad = along > 1f ? vel.y / along : 0f;
            return along * ClimbBuildTime * Mathf.Clamp01(1f - grad / MaxClimbGrad);
        }

        /// <summary>
        /// Climb gradient needed to clear one direction: free[k] = how far the
        /// sweep at ClimbLevels[k] got (>= ClimbReach = nothing). What stops
        /// level k is taken to reach up to the next level (the last: +8 m).
        /// </summary>
        public static float NeedGradient(float[] free, float roll)
        {
            float need = 0f;
            for (int k = 0; k < ClimbLevels.Length; k++)
            {
                if (free[k] >= ClimbReach) continue;
                float top = k + 1 < ClimbLevels.Length ? ClimbLevels[k + 1] : ClimbLevels[k] + 8f;
                float run = free[k] - roll;
                need = Mathf.Max(need, run < 1f ? float.PositiveInfinity : (top + ClimbMargin) / run);
            }
            return need;
        }
    }
}
