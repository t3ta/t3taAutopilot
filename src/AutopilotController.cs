using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace T3taAutopilot
{
    internal enum DriveState
    {
        Off,
        Engaged
    }

    /// <summary>
    /// The quest POI a destination marker stands for. The car never drives into
    /// its footprint (Rect). NearMarker: the marker is a precise spot inside it
    /// (the rally marker that starts the quest): stop outside the footprint as
    /// close to that spot as possible, instead of on the approach side.
    /// </summary>
    internal struct PoiArea
    {
        public Rect Rect;
        public bool NearMarker;
    }

    /// <summary>What the autopilot drives to (Shift + toggle key cycles).</summary>
    internal enum DestSource
    {
        Map,      // tracked waypoint, else the quick marker
        Quest,    // the tracked quest's current marker (POI, trader, treasure)
        Home,     // the bedroll
        Waypoint  // one saved waypoint, picked in the destination list
    }

    /// <summary>
    /// Autopilot state machine. Follows a dense route polyline (road-centered
    /// A* path when a road splatmap is available, else a straight line) with
    /// pure pursuit + a curvature speed profile, swerves around obstacles
    /// found by a fan of swept boxes, and writes the result into
    /// EntityVehicle.movementInput every frame from the MoveByAttachedEntity
    /// postfix.
    /// </summary>
    internal static class AutopilotController
    {
        const float OffRouteDist = 25f;          // replan when this far off the route
        const float ReplanCooldownSec = 10f;
        const float PlanWaitSec = 8f;            // wait this long for the first plan, then go direct
        const float LoadWaitSec = 45f;           // ... or this long while the road map is still loading
        const int MaxStuckRetries = 4;
        const int StuckBeforeDetour = 2;         // stuck this often in a row => drive around it
        const int MaxDetours = 8;                // per trip, then fall back to give-up logic
        const float ProbeHeight = 1.0f;          // probe box center above the wheels' ground line
        const float ProbeHalfHeight = 0.4f;      // box spans 0.6 .. 1.4 m above ground
        const float ProbeSideMargin = 0.4f;      // extra width each side of the body

        internal static AutopilotConfig Cfg = new AutopilotConfig();

        sealed class RouteResult
        {
            public List<Vector3> Waypoints;
            public string Error;
            public int Epoch;
        }

        static DriveState state = DriveState.Off;
        static EntityVehicle vehicle;
        static EntityPlayerLocal player;
        static Vector3 destination;      // where we drive to: the marker, moved out of a fenced roadblock prefab
        static Vector3 markerPos;        // the marker itself (to notice when the player moves it)
        static PoiArea? markerArea;         // the quest POI the marker stands for: stop on the road outside it
        const float OutsideRoadblockSearch = 80f;
        const float OutsideAreaSearch = 60f;     // past the POI edge
        const float RallyRoadExtra = 15f;        // a road this much farther than the POI edge still beats the edge
        const float ArrivalHoldSec = 5f;         // quest arrival: wait this long for the marker to move on
        static float arrivedAt = -1f;            // Time.time the car stopped at a quest destination, -1 = not arrived
        static DestSource destSource = DestSource.Map;
        // Quest codes are string hash codes: half of them are negative, so "none"
        // is 0 (the game's own unset value, Quest.SetupQuestCode), not -1 - a
        // negative pick fell back to the nearest quest (a turn-in at the trader).
        internal const int NoQuest = 0;
        static int selectedQuestCode = NoQuest;  // picked in the radial / list; NoQuest = tracked, else nearest
        static Waypoint selectedWaypoint;        // Waypoint source: the saved waypoint picked in the list
        static bool retargetWanted;              // destination picked again: re-plan even if it barely moved
        static float unresolvedSince = -1f;      // quest / home destination gone since (Time.time), -1 = resolved
        const float UnresolvedStopSec = 2f;
        const float QuestUnresolvedStopSec = 10f;   // recorded: ~105 m from some POIs the quest marker is gone for over 2 s
        static bool savedTurnTowardsLook;
        static float engagedAt;
        static bool wasAvoiding;
        static float lastStatusLog;

        // Off, flying the gyro by hand, with the destination radial / list
        // open: the hands are off the controls and it would slow down and
        // sink. Circle in place instead until something is picked (engage
        // takes over from there) or the menu closes (the player's again).
        static EntityVehicle loiterVeh;
        static bool loiterDone;          // taken over by hand: not again until the menu closes
        static float loiterAlt, loiterTurn;
        static bool loiterSavedTurnTowardsLook;
        static EntityVehicle loiterMeasured;

        // vehicle geometry in PhysicsTransform local space (from the wheels)
        static float wheelBase = 2.6f;
        static float halfWidth = 1f;
        static float frontZ = 2f;
        static float groundY = -0.5f;
        static int probeMask = ~0;
        static bool leans;   // two-wheeler: the body banks into turns

        // road routing
        static volatile RoadNetwork roadNet;
        static World roadNetWorld;
        static volatile bool roadNetLoading;
        static int roadNetGen;           // bumped per world; stale loads discarded
        static readonly Driver driver = new Driver();
        static PathFollower follower => driver.Follower;
        static volatile RouteResult pendingRoute;
        static int planEpoch;            // bumped on each engagement; stale plans discarded
        // planEpoch bumps and result publishing share this lock: a worker
        // publishes only while its epoch is still current, so a superseded
        // plan can't overwrite the newer result in the single pendingRoute slot
        static readonly object planLock = new object();
        static float lastReplanAt = -999f;
        static bool directFallback;      // driving a straight line only because no road plan was available
        static volatile bool planningInFlight;   // a plan for the current epoch is wanted and not yet consumed
        // At most one A* worker runs at a time (each allocates several
        // CellsX*CellsY arrays): a request made while one is busy is parked
        // here and launched with the latest destination when it finishes.
        static volatile bool workerBusy;
        static bool replanWanted;
        // cells we got stuck on this trip (gates, walls across the road).
        // Only local bypasses use them; main thread only.
        static readonly HashSet<int> blockedCells = new HashSet<int>();
        static readonly PathFollower mainRoute = new PathFollower();   // route before any bypass
        static readonly List<Vector3> recentHits = new List<Vector3>();
        static int detours;

        static readonly float[] probeFree = new float[Driver.ProbeAngles.Length];

        // gyrocopter: straight-line flight instead of road driving
        static bool flying;
        static readonly Pilot pilot = new Pilot();
        static float groundBelow, topAhead, destGround, lastTerrainScan = -999f;
        static float lastClimbScan = -999f, lastDetour;
        static bool lastGoingAround;
        static int climbBatch;           // which third of the directions the next climb-out sweep does
        const int ClimbBatches = 3;      // a full sweep every 0.25 s, spread over 3 calls
        static bool grounded;
        static bool flown;               // left the ground since engaging: on the wheels again = landed, not yet taken off
        const float FlownHeight = 3f;    // wheels this far up = really flying
        static Vector3 landing;          // where to put down: a road / flat spot near the destination
        static bool landingRefined;      // re-picked from close by, with the real approach direction
        const float LandingRadius = 120f;
        const float LandingRefineDist = 700f;
        const float LandingScanShort = 10f;   // approach: look ahead only to this short of the strip
        // A quest's rally marker shows up close by and moves the destination:
        // re-picking the strip from there turned a recorded approach round
        // 150 deg at 25 m AGL. Keep the strip on the approach if it's this near.
        const float KeepLandingDist = 250f;

        internal static bool Engaged => state != DriveState.Off;

        // ------------------------------------------------------------------
        // Telemetry: what the recorder samples besides the vehicle itself.
        // ------------------------------------------------------------------
        internal struct TelemetryState
        {
            public bool Engaged, Driving, Flying, OnRoad, Boxed, HaveProbes;
            public float Desired, Offset, Xte, Alpha, Progress, RouteLen, DestDist, TargetAlt, WantPitch, Agl;
            public string Phase;
        }

        internal static string TelemetryMode => state == DriveState.Off ? "manual" : flying ? "auto-fly" : "auto";

        static EntityVehicle measuredForProbes;

        /// <summary>Autopilot state for the recorder; also runs the probes while driving by hand.</summary>
        internal static TelemetryState TelemetrySnapshot(EntityVehicle veh, float[] probesOut)
        {
            var t = new TelemetryState();
            Vector3 pos = veh.position;
            var net = roadNet;
            t.OnRoad = net != null && net.IsFineRoad(pos.x, pos.z);
            t.Engaged = state != DriveState.Off && veh == vehicle;
            if (t.Engaged && flying)
            {
                t.Flying = true;
                t.Phase = pilot.Phase;
                t.TargetAlt = pilot.TargetAlt;
                t.WantPitch = pilot.DesiredPitch;
                t.Agl = pos.y - groundBelow;
                t.DestDist = HorizontalDist(pos, landing);
                return t;
            }
            if (t.Engaged)
            {
                t.DestDist = HorizontalDist(pos, destination);
                if (follower.HasPath)
                {
                    t.Driving = true;
                    t.Desired = driver.Desired;
                    t.Offset = driver.Offset;
                    t.Xte = follower.CrossTrack;
                    t.Alpha = driver.Alpha;
                    t.Boxed = driver.Boxed;
                    t.Progress = follower.Progress;
                    t.RouteLen = follower.Length;
                }
            }
            else if (!(veh is EntityVGyroCopter) && veh.PhysicsTransform != null)
            {
                // driving by hand: record what the autopilot probes would see
                if (measuredForProbes != veh)
                {
                    MeasureVehicle(veh);
                    measuredForProbes = veh;
                }
                float speed = veh.vehicleRB != null ? veh.vehicleRB.velocity.magnitude : 0f;
                ScanObstacles(veh, Driver.ProbeRange(speed), false);
            }
            if (!(veh is EntityVGyroCopter))
            {
                t.HaveProbes = true;
                Array.Copy(probeFree, probesOut, probeFree.Length);
            }
            return t;
        }

        // ------------------------------------------------------------------
        // Per-frame entry from ModEvents.GameUpdate: hotkey + detach watchdog.
        // ------------------------------------------------------------------
        internal static void OnGameUpdate(ref ModEvents.SGameUpdateData _data)
        {
            var gm = GameManager.Instance;
            var world = gm != null ? gm.World : null;
            if (world == null)
            {
                if (state != DriveState.Off)
                {
                    Disengage(null, null);
                }
                EndLoiter(null);
                Telemetry.End("world unloaded");
                return;
            }
            var p = world.GetPrimaryPlayer();
            Telemetry.Watchdog(p);
            if (p == null)
            {
                return;
            }
            EnsureRoadNetwork(world);   // preload in the background once per world

            if (Input.GetKeyDown(Cfg.ToggleKeyCode) && !IsModalWindowOpen(p))
            {
                // hold: destination radial; a tap comes back as Toggle / CycleDestSource
                if (!DestinationMenu.TryOpen(p))
                {
                    if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    {
                        CycleDestSource(p);
                    }
                    else
                    {
                        Toggle(p);
                    }
                }
            }

            if (state != DriveState.Off && (vehicle == null || player == null ||
                p.AttachedToEntity != vehicle || vehicle.AttachedMainEntity != p))
            {
                Disengage(p, "Autopilot off (left vehicle)");
            }
            if (loiterVeh != null && (p.AttachedToEntity != loiterVeh || loiterVeh.AttachedMainEntity != p))
            {
                EndLoiter("left vehicle");   // no more input frames come to end it
            }
            if (loiterDone && !DestinationUiOpen(p))
            {
                loiterDone = false;
            }
        }

        static bool IsModalWindowOpen(EntityPlayerLocal p)
        {
            var ui = LocalPlayerUI.GetUIForPlayer(p);
            return ui != null && ui.windowManager != null && ui.windowManager.IsModalWindowOpen();
        }

        /// <summary>Our destination radial or the destination list window is open.</summary>
        static bool DestinationUiOpen(EntityPlayerLocal p)
        {
            var ui = LocalPlayerUI.GetUIForPlayer(p);
            if (ui == null) return false;
            var radial = ui.xui != null ? ui.xui.RadialWindow : null;
            if (radial != null && radial.IsOpen && radial.context is DestinationMenu.Context) return true;
            return ui.windowManager != null && ui.windowManager.IsWindowOpen(XUiC_AutopilotDestinations.GroupName);
        }

        // ------------------------------------------------------------------
        // Loiter: circle while a destination is picked in the air (autopilot off)
        // ------------------------------------------------------------------
        static void Loiter(EntityVehicle veh, EntityPlayerLocal drv)
        {
            MovementInput mi = veh.movementInput;
            bool want = !loiterDone && veh is EntityVGyroCopter && veh.AttachedMainEntity == drv && mi != null &&
                veh.PhysicsTransform != null && veh.vehicle != null && veh.vehicleRB != null &&
                (veh.IsEngineRunning || !veh.vehicle.HasEnginePart()) && DestinationUiOpen(drv);
            if (want)
            {
                var ui = LocalPlayerUI.GetUIForPlayer(drv);
                var actions = ui != null ? ui.playerInput?.VehicleActions : null;
                if (actions != null &&
                    (Mathf.Abs(actions.Move.Y) > 0.05f || Mathf.Abs(actions.Move.X) > 0.05f ||
                     actions.Brake.IsPressed || actions.Hop.IsPressed))
                {
                    loiterDone = loiterVeh != null;   // flying it by hand with the menu open: leave it be
                    want = false;
                }
            }
            if (!want)
            {
                if (loiterVeh != null) EndLoiter(loiterDone ? "manual input" : "menu closed");
                return;
            }

            Vector3 pos = veh.position;
            Vector3 vel = veh.vehicleRB.velocity;
            float yawRate = veh.vehicleRB.angularVelocity.y * Mathf.Rad2Deg;
            if (loiterVeh != veh)
            {
                // only in the air: on the wheels it stays put by itself
                float below;
                bool hitBelow = SurfaceAt(pos, out below);
                if (loiterMeasured != veh)
                {
                    MeasureVehicle(veh);   // groundY: the wheels' height
                    loiterMeasured = veh;
                }
                Vector3 feet = veh.PhysicsTransform.TransformPoint(new Vector3(0f, groundY, 0f)) + Origin.position;
                if (!hitBelow || feet.y - below < 1.5f)
                {
                    return;
                }
                loiterVeh = veh;
                loiterAlt = pos.y;
                loiterTurn = yawRate < -3f ? -1f : 1f;   // keep turning the way it already is
                groundBelow = below;
                lastTerrainScan = Time.time;
                pilot.Reset();
                loiterSavedTurnTowardsLook = EntityVehicle.isTurnTowardsLook;
                Telemetry.Event("loiter_start", "agl", pos.y - below, "speed", new Vector3(vel.x, 0f, vel.z).magnitude);
                Log.Out("[t3taAutopilot] circling while a destination is picked (y=" + pos.y.ToString("F0") + ")");
            }
            EntityVehicle.isTurnTowardsLook = false;   // the camera / cursor mustn't steer it
            if (Time.time - lastTerrainScan > 0.1f)
            {
                lastTerrainScan = Time.time;
                float below;
                groundBelow = SurfaceAt(pos, out below) ? below : pos.y - 50f;
            }
            var o = pilot.Loiter(pos, veh.PhysicsTransform.forward, vel, yawRate, loiterAlt, groundBelow,
                loiterTurn, Time.deltaTime);
            WriteFlightInput(mi, o.Throttle, o.Yaw, o.Up, o.Down);
        }

        static void EndLoiter(string reason)
        {
            if (loiterVeh == null) return;
            loiterVeh = null;
            EntityVehicle.isTurnTowardsLook = loiterSavedTurnTowardsLook;
            if (reason != null)
            {
                Telemetry.Event("loiter_end", "reason", reason);
                Log.Out("[t3taAutopilot] circling ended (" + reason + ")");
            }
        }

        // ------------------------------------------------------------------
        // Engage / disengage
        // ------------------------------------------------------------------
        internal static void Toggle(EntityPlayerLocal p)
        {
            if (state == DriveState.Off)
            {
                SelectTrackedQuest(p);
                TryEngage(p);
            }
            else
            {
                Disengage(p, "Autopilot off");
            }
        }

        /// <summary>
        /// A plain tap drives to the tracked quest when there is one, whatever
        /// was picked before: as if it were picked in the radial.
        /// </summary>
        static void SelectTrackedQuest(EntityPlayerLocal p)
        {
            var journal = p.QuestJournal;
            Quest q = journal != null ? journal.TrackedQuest : null;
            if (!HasQuestMarker(q)) return;
            if (destSource == DestSource.Quest && selectedQuestCode == q.QuestCode) return;
            destSource = DestSource.Quest;
            selectedQuestCode = q.QuestCode;
            selectedWaypoint = null;
            unresolvedSince = -1f;
            Log.Out("[t3taAutopilot] destination source: quest #" + q.QuestCode + " (tracked)");
            Telemetry.Event("dest_source", "source", destSource.ToString(), "quest", q.QuestCode);
        }

        internal static void Stop(EntityPlayerLocal p)
        {
            if (state != DriveState.Off)
            {
                Disengage(p, "Autopilot off");
            }
        }

        /// <summary>
        /// Picked from the radial: drive there. Engages when sitting in the
        /// driver's seat; while engaged the drive loop re-plans to it.
        /// questCode: the quest to follow (NoQuest = tracked, else nearest).
        /// </summary>
        internal static void SelectDestination(EntityPlayerLocal p, DestSource source, int questCode,
            Waypoint waypoint = null)
        {
            DestSource prevSource = destSource;
            int prevQuest = selectedQuestCode;
            Waypoint prevWaypoint = selectedWaypoint;
            destSource = source;
            selectedQuestCode = questCode;
            selectedWaypoint = waypoint;
            Vector3 dest;
            PoiArea? area;
            if (!TryGetDestination(p, out dest, out area))
            {
                // nothing to drive to there: keep the current destination (and the drive to it)
                string hint = NoDestinationHint();
                destSource = prevSource;
                selectedQuestCode = prevQuest;
                selectedWaypoint = prevWaypoint;
                GameManager.ShowTooltip(p, "Autopilot: " + hint + " - keeping the current destination");
                return;
            }
            unresolvedSince = -1f;   // another destination: not the lost one coming back
            Log.Out("[t3taAutopilot] destination source: " + DestLabel() + (questCode != NoQuest ? " #" + questCode : ""));
            Telemetry.Event("dest_source", "source", destSource.ToString(), "quest", questCode);
            var veh = p.AttachedToEntity as EntityVehicle;
            if (state == DriveState.Off && veh != null && veh.AttachedMainEntity == p)
            {
                TryEngage(p);
                return;
            }
            retargetWanted = true;   // re-plan even if the new target is next to the old one
            GameManager.ShowTooltip(p, "Autopilot destination: " + DestLabel() + " " + (int)dest.x + ", " + (int)dest.z);
        }

        static void TryEngage(EntityPlayerLocal p)
        {
            var veh = p.AttachedToEntity as EntityVehicle;
            if (veh == null || veh.AttachedMainEntity != p)
            {
                GameManager.ShowTooltip(p, "Autopilot: get in the driver's seat first");
                return;
            }
            if (!(veh is EntityVGyroCopter) && !Cfg.GroundVehicles)
            {
                GameManager.ShowTooltip(p, "Autopilot: gyrocopter only  (\"groundVehicles\": true in t3taAutopilot.json for the others)");
                return;
            }
            Vector3 dest;
            PoiArea? area;
            if (!TryGetDestination(p, out dest, out area))
            {
                GameManager.ShowTooltip(p, "Autopilot: " + NoDestinationHint() +
                    "  (Shift+" + Cfg.ToggleKey + ": change destination)");
                return;
            }
            EndLoiter("engaged");   // before saving the turn mode loitering switched off
            vehicle = veh;
            player = p;
            SetDestination(dest, area, veh is EntityVGyroCopter);   // flight keeps the marker; the strip search avoids roadblocks
            savedTurnTowardsLook = EntityVehicle.isTurnTowardsLook;
            wasAvoiding = false;
            driver.Reset();
            pendingRoute = null;
            planningInFlight = false;
            lock (planLock) planEpoch++;
            lastReplanAt = -999f;
            engagedAt = Time.time;
            detours = 0;
            directFallback = false;
            replanWanted = false;
            retargetWanted = false;
            unresolvedSince = -1f;
            arrivedAt = -1f;
            flown = false;
            blockedCells.Clear();
            mainRoute.Clear();
            state = DriveState.Engaged;

            MeasureVehicle(veh);
            flying = veh is EntityVGyroCopter;
            if (flying)
            {
                pilot.Reset();
                pilot.ClimbScanned = false;
                lastTerrainScan = -999f;
                lastClimbScan = -999f;
                climbBatch = 0;
                lastDetour = 0f;
                lastGoingAround = false;
                landingRefined = false;
                PickLanding(veh.position);
                SetRouteTop(veh.position);
                pilot.Cruise = FlightCruise(veh);
                Telemetry.Event("engage", "dest", dest, "flight", true, "landing", landing, "route_top", pilot.RouteTop,
                    "cruise", pilot.Cruise);
                Log.Out("[t3taAutopilot] Engaged (flight). Destination " + dest + " groundY=" + groundY.ToString("F2") +
                    " routeTop=" + pilot.RouteTop.ToString("F0") + " cruise=" + pilot.Cruise.ToString("F1"));
                GameManager.ShowTooltip(p, "Autopilot engaged (flight) -> " + DestLabel() + " " + (int)dest.x + ", " + (int)dest.z +
                    "  (WASD/Space or " + Cfg.ToggleKey + " to take over)");
                return;
            }
            driver.Cruise = Cfg.MaxCruiseSpeed;
            PathFollower.LatAccel = Mathf.Clamp(Cfg.CornerAccel, 1f, 8f);
            driver.WheelBase = wheelBase;
            driver.SteerMax = veh.vehicle.SteerAngleMax;
            driver.HalfWidth = halfWidth;
            driver.IsRoad = (x, z) => roadNet != null && roadNet.IsFineRoad(x, z);
            if (roadNet != null)
            {
                StartPlanning(veh.position, destination);   // the marker, moved out of a fenced prefab
            }
            else if (!roadNetLoading)
            {
                SetDirectPath(veh.position);
            }
            // else: the route is planned as soon as the road map finishes loading

            Telemetry.Event("engage", "dest", dest, "flight", false,
                "routing", roadNet != null ? "road" : roadNetLoading ? "loading" : "direct");
            Log.Out("[t3taAutopilot] Engaged. Destination " + dest +
                (roadNet != null ? " (road routing)" : roadNetLoading ? " (road map loading...)" : " (direct)") +
                " wheelBase=" + wheelBase.ToString("F2") + " halfWidth=" + halfWidth.ToString("F2") +
                " frontZ=" + frontZ.ToString("F2") + " groundY=" + groundY.ToString("F2") +
                " steerMax=" + veh.vehicle.SteerAngleMax + " mask=0x" + probeMask.ToString("X8"));
            GameManager.ShowTooltip(p, "Autopilot engaged -> " + DestLabel() + " " + (int)dest.x + ", " + (int)dest.z +
                "  (WASD/Space or " + Cfg.ToggleKey + " to take over)");
        }

        static void Disengage(EntityPlayerLocal p, string message)
        {
            if (state != DriveState.Off)
            {
                Telemetry.Event("disengage", "reason", message ?? "world unloaded");
            }
            EntityVehicle.isTurnTowardsLook = savedTurnTowardsLook;
            state = DriveState.Off;
            vehicle = null;
            player = null;
            driver.Reset();
            pendingRoute = null;
            planningInFlight = false;
            lock (planLock) planEpoch++;
            if (message != null && p != null)
            {
                GameManager.ShowTooltip(p, message);
            }
            if (message != null)
            {
                Log.Out("[t3taAutopilot] " + message);
            }
        }

        /// <summary>
        /// Vehicle footprint from its wheel colliders (PhysicsTransform local
        /// space) and the collision layers its body actually hits.
        /// </summary>
        static void MeasureVehicle(EntityVehicle veh)
        {
            wheelBase = 2.6f;
            halfWidth = 1f;
            frontZ = 2f;
            groundY = -0.5f;
            leans = veh.vehicle != null && veh.vehicle.TiltUpForce > 0f;   // banks with the steering
            Transform pt = veh.PhysicsTransform;
            var wheels = veh.wheels;
            if (pt != null && wheels != null && wheels.Length > 0)
            {
                float minZ = float.MaxValue, maxZ = float.MinValue, maxX = 0f, sumGround = 0f;
                int n = 0;
                for (int i = 0; i < wheels.Length; i++)
                {
                    var wc = wheels[i] != null ? wheels[i].wheelC : null;
                    if (wc == null) continue;
                    // wc.transform is the suspension mount; GetWorldPose gives the
                    // actual (compressed) wheel center
                    Vector3 wpos;
                    Quaternion wrot;
                    wc.GetWorldPose(out wpos, out wrot);
                    Vector3 l = pt.InverseTransformPoint(wpos);
                    minZ = Mathf.Min(minZ, l.z);
                    maxZ = Mathf.Max(maxZ, l.z);
                    maxX = Mathf.Max(maxX, Mathf.Abs(l.x));
                    sumGround += l.y - wc.radius;
                    n++;
                }
                if (n > 0)
                {
                    wheelBase = Mathf.Clamp(maxZ - minZ, 1f, 6f);
                    halfWidth = Mathf.Clamp(maxX + 0.35f, 0.45f, 1.6f);
                    frontZ = maxZ + 0.6f;
                    groundY = sumGround / n;
                }
            }

            probeMask = ~0;
            try
            {
                int layer = veh.vehicleRB != null ? veh.vehicleRB.gameObject.layer : pt.gameObject.layer;
                int mask = 0;
                for (int i = 0; i < 32; i++)
                {
                    if (!Physics.GetIgnoreLayerCollision(layer, i)) mask |= 1 << i;
                }
                if (mask != 0) probeMask = mask;
            }
            catch (Exception e)
            {
                Log.Warning("[t3taAutopilot] layer mask lookup failed: " + e.Message);
            }
        }

        // ------------------------------------------------------------------
        // Destination resolution, by the selected source. area: the quest POI
        // footprint when the target is one (null otherwise).
        // ------------------------------------------------------------------
        static bool TryGetDestination(EntityPlayerLocal p, out Vector3 dest, out PoiArea? area)
        {
            area = null;
            switch (destSource)
            {
                case DestSource.Quest:
                    return TryGetQuestDestination(p, out dest, out area);
                case DestSource.Home:
                    return TryGetHomeDestination(p, out dest);
                case DestSource.Waypoint:
                    return TryGetWaypointDestination(p, out dest);
                default:
                    return TryGetMapDestination(p, out dest);
            }
        }

        /// <summary>The picked saved waypoint, while it still exists.</summary>
        static bool TryGetWaypointDestination(EntityPlayerLocal p, out Vector3 dest)
        {
            var list = p.Waypoints != null ? p.Waypoints.Collection.list : null;
            if (list != null)
            {
                for (int i = 0; i < list.Count; i++)
                {
                    // the very object picked: another waypoint at the same spot doesn't count
                    if (ReferenceEquals(list[i], selectedWaypoint))
                    {
                        dest = list[i].pos.ToVector3();
                        return true;
                    }
                }
            }
            dest = Vector3.zero;
            return false;
        }

        /// <summary>Is this the destination currently selected (for the radial highlight)?</summary>
        internal static bool IsCurrent(EntityPlayerLocal p, DestSource source, int questCode,
            Waypoint waypoint = null)
        {
            if (destSource != source) return false;
            if (source == DestSource.Waypoint) return waypoint != null && ReferenceEquals(selectedWaypoint, waypoint);
            if (source != DestSource.Quest) return true;
            if (selectedQuestCode != NoQuest) return selectedQuestCode == questCode;
            var q = ResolveQuest(p);   // tracked / nearest (Shift+tap): the one it actually drives to
            return q != null && q.QuestCode == questCode;
        }

        internal static void CycleDestSource(EntityPlayerLocal p)
        {
            Vector3 dest;
            PoiArea? area;
            bool found = false;
            // while driving, skip sources with nothing to drive to (the old target would
            // silently stay); when parked, step one at a time so each shows its hint
            for (int i = 0; i < 3 && !found; i++)
            {
                destSource = destSource == DestSource.Map ? DestSource.Quest
                    : destSource == DestSource.Quest ? DestSource.Home : DestSource.Map;
                selectedQuestCode = NoQuest;
                found = TryGetDestination(p, out dest, out area);
                if (state == DriveState.Off) break;
            }
            if (found && state != DriveState.Off) retargetWanted = true;
            if (found) unresolvedSince = -1f;   // another source: not the lost destination coming back
            string msg = TryGetDestination(p, out dest, out area)
                ? "Autopilot destination: " + DestLabel() + " " + (int)dest.x + ", " + (int)dest.z +
                  " (" + (int)HorizontalDist(p.position, dest) + " m)"
                : "Autopilot destination: " + DestLabel() + " - " + NoDestinationHint();
            GameManager.ShowTooltip(p, msg);
            Log.Out("[t3taAutopilot] " + msg);
            Telemetry.Event("dest_source", "source", destSource.ToString());
        }

        static string DestLabel()
        {
            switch (destSource)
            {
                case DestSource.Quest: return "quest";
                case DestSource.Home: return "home";
                case DestSource.Waypoint: return "waypoint";
                default: return "map marker";
            }
        }

        static string NoDestinationHint()
        {
            switch (destSource)
            {
                case DestSource.Quest: return "no active quest with a map marker";
                case DestSource.Home: return "no bedroll placed";
                case DestSource.Waypoint: return "the waypoint was removed";
                default: return "no destination - track a waypoint on the map or set the quick marker";
            }
        }

        /// <summary>
        /// The tracked quest, else the nearest active one, that shows a marker.
        /// Quest.Position follows the current objective: the POI center while
        /// going there (area = its footprint), the quest giver once it is ready
        /// to turn in, the dig circle center for buried supplies.
        /// </summary>
        static bool TryGetQuestDestination(EntityPlayerLocal p, out Vector3 dest, out PoiArea? area)
        {
            dest = Vector3.zero;
            area = null;
            Quest q = ResolveQuest(p);
            if (q == null) return false;
            dest = q.Position;
            if (q.CurrentState == Quest.QuestState.InProgress)
            {
                Vector3 poi, size;
                if (q.GetPositionData(out poi, Quest.PositionDataTypes.POIPosition) &&
                    q.GetPositionData(out size, Quest.PositionDataTypes.POISize) && size.x > 0f && size.z > 0f)
                {
                    var r = new Rect(poi.x, poi.z, size.x, size.z);
                    if (r.Contains(new Vector2(dest.x, dest.z)))   // the marker is the POI, not the trader
                    {
                        // Near the POI the quest moves its marker from the POI to the
                        // rally marker (ObjectiveRallyPoint -> Activate position data)
                        Vector3 rally;
                        bool atRally = q.GetPositionData(out rally, Quest.PositionDataTypes.Activate) &&
                            HorizontalDist(rally, dest) < 1.5f;
                        area = new PoiArea { Rect = r, NearMarker = atRally };
                    }
                }
            }
            return true;
        }

        /// <summary>
        /// A quest, home or picked waypoint destination went away while driving
        /// (the quest was abandoned, failed or turned in and no other quest has a
        /// marker, the bedroll was picked up, the waypoint was deleted): stop
        /// instead of driving on to the old spot.
        /// It has to stay gone for UnresolvedStopSec (a quest: QuestUnresolvedStopSec), so a marker that blinks
        /// out while a quest switches objectives doesn't stop the car (a
        /// cleared POI isn't "gone": the quest moves on to turn-in). The map
        /// marker keeps the old behavior: drive to the last one seen.
        /// In the air it never stops (recorded: switched off 15 m up, 25 m
        /// short of the strip, as the marker moved to the rally point): the
        /// gyro lands where it was heading, or follows the marker once it is back.
        /// </summary>
        static bool DestinationGone(EntityPlayerLocal p, bool airborne = false)
        {
            Vector3 dest = Vector3.zero;
            PoiArea? area;
            bool map = destSource == DestSource.Map;
            if (map || TryGetDestination(p, out dest, out area))
            {
                if (unresolvedSince >= 0f && !map)   // switched to the map marker: not the lost one coming back
                {
                    Telemetry.Event("dest_back", "source", destSource.ToString(), "gap_s", Time.time - unresolvedSince,
                        "dest", dest, "from_old", HorizontalDist(dest, markerPos));
                    Log.Out("[t3taAutopilot] destination is back after " + (Time.time - unresolvedSince).ToString("F1") + " s");
                }
                unresolvedSince = -1f;
                return false;
            }
            if (unresolvedSince < 0f)
            {
                unresolvedSince = Time.time;
                Telemetry.Event("dest_lost", "source", destSource.ToString(), "from_marker", HorizontalDist(p.position, markerPos),
                    "airborne", airborne);
                Log.Out("[t3taAutopilot] destination is gone - " + (airborne ? "flying on to the landing spot" : "driving on for now"));
            }
            float wait = destSource == DestSource.Quest ? QuestUnresolvedStopSec : UnresolvedStopSec;
            if (airborne || Time.time - unresolvedSince < wait)
            {
                return false;
            }
            string why = destSource == DestSource.Home ? "the bedroll is gone"
                : destSource == DestSource.Waypoint ? "the waypoint was removed"
                : "the quest is no longer active";
            selectedQuestCode = NoQuest;   // next engage: tracked / nearest quest
            Disengage(p, "Autopilot off: " + why);
            return true;
        }

        /// <summary>The quest the Quest source drives to, or null.</summary>
        static Quest ResolveQuest(EntityPlayerLocal p)
        {
            var journal = p.QuestJournal;
            if (journal == null) return null;
            Quest q;
            if (selectedQuestCode != NoQuest)
            {
                // the one picked in the radial, and only that one: once it is
                // done, don't wander off to another quest
                q = journal.FindActiveQuest(selectedQuestCode);
                return HasQuestMarker(q) ? q : null;
            }
            if (HasQuestMarker(q = journal.TrackedQuest)) return q;
            q = null;
            float best = float.MaxValue;
            foreach (var c in journal.quests)
            {
                if (!HasQuestMarker(c)) continue;
                float d = HorizontalDist(p.position, c.Position);
                if (d < best) { best = d; q = c; }
            }
            return q;
        }

        internal static bool HasQuestMarker(Quest q)
        {
            return q != null && q.Active && q.HasPosition && q.Position != Vector3.zero;
        }

        internal static bool TryGetHomeDestination(EntityPlayerLocal p, out Vector3 dest)
        {
            var sp = p.GetSpawnPoint();
            dest = sp.IsUndef() ? Vector3.zero : sp.position;
            return !sp.IsUndef();
        }

        internal static bool TryGetMapDestination(EntityPlayerLocal p, out Vector3 dest)
        {
            var list = p.Waypoints != null ? p.Waypoints.Collection.list : null;
            if (list != null)
            {
                for (int i = 0; i < list.Count; i++)
                {
                    var wp = list[i];
                    if (wp.bTracked && !wp.bIsAutoWaypoint)
                    {
                        dest = wp.pos.ToVector3();
                        return true;
                    }
                }
            }
            Vector3i marker = p.markerPosition;
            if (!marker.Equals(Vector3i.zero))
            {
                dest = marker.ToVector3();
                return true;
            }
            dest = Vector3.zero;
            return false;
        }

        // ------------------------------------------------------------------
        // Road network: lazy-load once per world on the main thread.
        // ------------------------------------------------------------------
        static void EnsureRoadNetwork(World w)
        {
            if (ReferenceEquals(roadNetWorld, w))
            {
                return;
            }
            roadNetWorld = w;
            roadNet = null;
            // a new world / save: its quests (and quest codes) are different
            destSource = DestSource.Map;
            selectedQuestCode = NoQuest;
            selectedWaypoint = null;
            retargetWanted = false;
            int gen = ++roadNetGen;
            string dir = RoadNetwork.FindWorldDir(w);
            if (dir == null)
            {
                roadNetLoading = false;
                return;
            }
            roadNetLoading = true;
            var cfg = Cfg;
            // Application.dataPath is main-thread only: <game>/7DaysToDie_Data -> <game>/Data/Prefabs
            var prefabDirs = new List<string>();
            try { prefabDirs.Add(System.IO.Path.Combine(System.IO.Path.GetDirectoryName(Application.dataPath), "Data", "Prefabs")); }
            catch (Exception) { }
            ThreadPool.QueueUserWorkItem(_ =>
            {
                var net = RoadNetwork.LoadFromDir(dir, cfg, prefabDirs);
                if (gen == roadNetGen)
                {
                    roadNet = net;
                    roadNetLoading = false;
                }
            });
        }

        static void StartPlanning(Vector3 from, Vector3 to)
        {
            if (roadNet == null || planningInFlight) return;
            planningInFlight = true;
            lastReplanAt = Time.time;
            if (workerBusy)
            {
                replanWanted = true;   // launched from the frame loop once the busy worker ends
                return;
            }
            LaunchWorker(from, to);
        }

        static void LaunchWorker(Vector3 from, Vector3 to)
        {
            var net = roadNet;
            var cfg = Cfg;
            int epoch = planEpoch;
            workerBusy = true;
            ThreadPool.QueueUserWorkItem(_ =>
            {
                RouteResult res;
                try
                {
                    res = new RouteResult
                    {
                        Waypoints = RoutePlanner.FindPath(net, from, to, cfg.RoadCost, cfg.OffRoadCost, cfg.FlatOffRoadCost),
                        Epoch = epoch
                    };
                }
                catch (Exception e)
                {
                    res = new RouteResult { Error = e.ToString(), Epoch = epoch };
                }
                lock (planLock)
                {
                    if (epoch == planEpoch) pendingRoute = res;   // superseded: drop it
                }
                workerBusy = false;
            });
        }

        static void SetDirectPath(Vector3 pos)
        {
            directFallback = true;
            var line = new List<Vector3> { new Vector3(pos.x, 0f, pos.z), new Vector3(destination.x, 0f, destination.z) };
            driver.SetPath(RoutePlanner.Resample(line, RoutePlanner.Spacing), pos, null);
        }

        static Func<Vector3, float> SpeedCaps()
        {
            var net = roadNet;
            if (net == null) return null;
            return p => net.SpeedCapAt(p.x, p.z);
        }

        // ------------------------------------------------------------------
        // Called from the MoveByAttachedEntity postfix every frame while the
        // local player is attached to a vehicle. Overwrites movementInput.
        // ------------------------------------------------------------------
        internal static void InjectVehicleInput(EntityVehicle veh, EntityPlayerLocal drv)
        {
            if (state == DriveState.Off)
            {
                Loiter(veh, drv);
                return;
            }
            if (veh != vehicle || drv != player)
            {
                return;
            }
            MovementInput mi = veh.movementInput;
            if (mi == null || veh.PhysicsTransform == null || veh.vehicle == null)
            {
                return;
            }

            // Steer-by-input instead of steer-towards-camera while engaged.
            EntityVehicle.isTurnTowardsLook = false;

            // Any real driving input hands control back to the player.
            var ui = LocalPlayerUI.GetUIForPlayer(drv);
            var actions = ui != null ? ui.playerInput?.VehicleActions : null;
            if (actions != null &&
                (Mathf.Abs(actions.Move.Y) > 0.05f || Mathf.Abs(actions.Move.X) > 0.05f ||
                 actions.Brake.IsPressed || actions.Hop.IsPressed))
            {
                // the case worth studying: what was going on when the player grabbed the wheel
                float[] free = flying ? null : (float[])probeFree.Clone();
                Telemetry.Event("takeover", "move_x", actions.Move.X, "move_y", actions.Move.Y,
                    "brake", actions.Brake.IsPressed, "hop", actions.Hop.IsPressed,
                    "speed", veh.vehicle.CurrentForwardVelocity,
                    "offset", flying ? 0f : driver.Offset, "xte", flying ? 0f : follower.CrossTrack,
                    "desired", flying ? 0f : driver.Desired, "boxed", !flying && driver.Boxed,
                    "stuck_count", flying ? 0 : driver.StuckCount, "detours", detours,
                    "phase", flying ? pilot.Phase : null, "free", free);
                Disengage(drv, "Autopilot off (manual input)");
                return;
            }

            if (flying)
            {
                Fly(veh, drv, mi);
                return;
            }

            if (!veh.IsEngineRunning && veh.vehicle.HasEnginePart())
            {
                WriteInput(mi, 0f, 0f, true);
                return;
            }

            float dt = Time.deltaTime;
            Vector3 pos = veh.position;

            if (DestinationGone(drv))
            {
                return;
            }

            // Keep following the waypoint if the player moves it while driving
            // (or the quest moves on to its next objective, or the source changes).
            // The POI stop rule changing counts too: a rally marker within 5 m of
            // the POI center still moves the stop to the side nearest to it.
            Vector3 liveDest;
            PoiArea? liveArea;
            if (TryGetDestination(drv, out liveDest, out liveArea) &&
                (retargetWanted || HorizontalDist(liveDest, markerPos) > 5f || !SameArea(liveArea, markerArea)))
            {
                retargetWanted = false;
                SetDestination(liveDest, liveArea, false);
                if (roadNet != null)
                {
                    planningInFlight = false;   // supersede any plan in flight
                    lock (planLock) planEpoch++;
                    StartPlanning(pos, destination);
                }
                else
                {
                    SetDirectPath(pos);
                }
            }

            ConsumePlan(pos);
            if (replanWanted && !workerBusy && roadNet != null)
            {
                replanWanted = false;
                LaunchWorker(pos, destination);   // the request parked while a worker was busy
            }

            // went straight while the road map was loading (or a plan was late):
            // switch to a road plan as soon as one can be made
            if (directFallback && follower.HasPath && roadNet != null && !planningInFlight &&
                Time.time - lastReplanAt > ReplanCooldownSec)
            {
                SetDestination(markerPos, markerArea, false);   // the direct path aimed at the raw marker
                StartPlanning(pos, destination);
            }

            if (!follower.HasPath)
            {
                if (roadNet != null && !planningInFlight && Time.time - lastReplanAt > ReplanCooldownSec)
                {
                    SetDestination(markerPos, markerArea, false);   // road map finished loading after engage
                    StartPlanning(pos, destination);
                }
                // the map load and the plan get their own deadlines: a plan
                // started once the map arrived has PlanWaitSec from then
                float waited = Time.time - engagedAt;
                if (roadNetLoading ? waited < LoadWaitSec : (planningInFlight && Time.time - lastReplanAt < PlanWaitSec))
                {
                    WriteInput(mi, 0f, 0f, true);   // hold until the route arrives
                    return;
                }
                Log.Warning("[t3taAutopilot] no route after " + waited.ToString("F0") + "s - driving direct");
                Telemetry.Event("direct_fallback", "waited_s", waited, "map_loading", roadNetLoading);
                SetDirectPath(pos);
            }

            Vector3 fwd = veh.PhysicsTransform.forward;
            fwd.y = 0f;
            if (fwd.sqrMagnitude < 0.001f)
            {
                return;
            }
            fwd.Normalize();

            // Pushed far off the route (avoidance detour gone wrong, fell off
            // a ledge, shoved by a horde): replan from here.
            if (roadNet != null && !planningInFlight && follower.CrossTrack > OffRouteDist &&
                Time.time - lastReplanAt > ReplanCooldownSec)
            {
                Log.Out("[t3taAutopilot] off route by " + follower.CrossTrack.ToString("F0") + "m - replanning");
                Telemetry.Event("replan_off_route", "xte", follower.CrossTrack);
                StartPlanning(pos, destination);
            }

            float fwdSpeed = veh.vehicle.CurrentForwardVelocity;
            float speed = veh.vehicleRB != null ? veh.vehicleRB.velocity.magnitude : Mathf.Abs(fwdSpeed);

            float range = Driver.ProbeRange(speed);
            if (!driver.Reversing)
            {
                ScanObstacles(veh, range, true);
            }
            mainRoute.Update(pos);   // keep the pre-bypass route's progress current for rejoining
            var o = driver.Step(pos, fwd, fwdSpeed, speed, destination, probeFree, range, Time.time, Time.deltaTime);

            if (o.Arrived)
            {
                // Near a quest POI the game only then moves the marker to the rally
                // marker: keep braking a moment, so a moved marker (picked up by the
                // re-plan above) sends the car on instead of ending the drive.
                if (destSource == DestSource.Quest)
                {
                    if (arrivedAt < 0f)
                    {
                        arrivedAt = Time.time;
                        Telemetry.Event("arrival_hold", "dest", destination);
                    }
                    // ... and while the marker is gone (switching to the rally marker):
                    // DestinationGone ends that wait if it doesn't come back
                    if (Time.time - arrivedAt < ArrivalHoldSec || unresolvedSince >= 0f)
                    {
                        WriteInput(mi, 0f, 0f, true);
                        return;
                    }
                }
                Disengage(drv, "Autopilot: arrived");
                return;
            }
            arrivedAt = -1f;

            bool avoiding = driver.Offset != 0f;
            if (avoiding != wasAvoiding)
            {
                Telemetry.Event("avoid", "on", avoiding, "offset", driver.Offset, "free", probeFree);
            }
            if (Cfg.DebugProbes && avoiding != wasAvoiding)
            {
                Log.Out("[t3taAutopilot] avoid " + (avoiding ? "ON offset=" + driver.Offset : "off") +
                    " free=[" + FreeSummary() + "]");
            }
            wasAvoiding = avoiding;
            if (Cfg.DebugProbes && Time.time - lastStatusLog > 2f)
            {
                lastStatusLog = Time.time;
                Log.Out("[t3taAutopilot] v=" + fwdSpeed.ToString("F1") + " want=" + driver.Desired.ToString("F1") +
                    " xte=" + follower.CrossTrack.ToString("F1") + " alpha=" + driver.Alpha.ToString("F0") +
                    " steer=" + o.Steer.ToString("F2") + " offset=" + driver.Offset +
                    " prog=" + follower.Progress.ToString("F0") + "/" + follower.Length.ToString("F0") +
                    " road=" + (roadNet != null && roadNet.IsFineRoad(pos.x, pos.z)));
            }

            if (o.StuckEvent)
            {
                Log.Out("[t3taAutopilot] stuck" + (driver.Boxed ? " (boxed in)" : "") + ", reversing - retry " +
                    driver.StuckCount + " free=[" + FreeSummary() + "]");
                if (Cfg.DebugProbes || Telemetry.Active)
                {
                    Transform pt = veh.PhysicsTransform;
                    Quaternion frame = ProbeFrame(pt);
                    string on = DescribeHits(veh, ProbeOrigin(pt, frame),
                        new Vector3(halfWidth + ProbeSideMargin, ProbeHalfHeight, 0.05f), pt.forward, frame, 6f);
                    if (Cfg.DebugProbes) Log.Out("[t3taAutopilot] stuck on: " + on);
                    Telemetry.Event("stuck", "retry", driver.StuckCount, "boxed", driver.Boxed, "free", probeFree,
                        "stuck_on", on);
                }
                if (driver.StuckCount >= StuckBeforeDetour && roadNet != null && mainRoute.HasPath &&
                    detours < MaxDetours)
                {
                    // Same spot keeps stopping us (closed gate, wall across the
                    // road): close this stretch and leave the road around it,
                    // rejoining the same route past it. A few ms of A* over a
                    // small area, so done inline.
                    int n = RoutePlanner.MarkBlockedAhead(roadNet, follower, pos, fwd, 0f, 14f, 7f, blockedCells);
                    recentHits.Clear();
                    driver.RecentHits(recentHits);
                    n += RoutePlanner.MarkBlockedPoints(roadNet, recentHits, halfWidth + 1.5f, pos, fwd, blockedCells);
                    var bypass = RoutePlanner.Bypass(roadNet, mainRoute, pos, fwd, blockedCells);
                    detours++;
                    driver.ResetStuck();
                    Telemetry.Event("detour", "n", detours, "cells_closed", n, "found", bypass != null);
                    if (bypass != null)
                    {
                        driver.SetPath(bypass, pos, SpeedCaps());
                        Log.Out("[t3taAutopilot] way blocked - off-road bypass " + detours + " (" + n + " cells closed, " +
                            follower.Length.ToString("F0") + "m to go)");
                        GameManager.ShowTooltip(drv, "Autopilot: way blocked, going around");
                    }
                    else
                    {
                        Log.Warning("[t3taAutopilot] way blocked - no bypass found");
                    }
                }
                else if (driver.StuckCount > MaxStuckRetries)
                {
                    Disengage(drv, "Autopilot: vehicle is stuck");
                    return;
                }
            }

            WriteInput(mi, o.Throttle, o.Steer, o.Brake);
        }

        static void ConsumePlan(Vector3 pos)
        {
            RouteResult rr;
            lock (planLock)   // take and clear together: a worker publishes under the same lock
            {
                rr = pendingRoute;
                pendingRoute = null;
            }
            if (rr == null) return;
            if (rr.Epoch != planEpoch)
            {
                return;   // stale plan (previous engagement / superseded destination)
            }
            planningInFlight = false;
            if (rr.Error != null)
            {
                Log.Error("[t3taAutopilot] route planning failed: " + rr.Error);
            }
            else if (rr.Waypoints != null && rr.Waypoints.Count > 0)
            {
                directFallback = false;
                driver.SetPath(rr.Waypoints, pos, SpeedCaps());
                mainRoute.SetPath(rr.Waypoints, pos, Cfg.MaxCruiseSpeed, null);
                blockedCells.Clear();
                Telemetry.Event("route_planned", "length", follower.Length, "points", rr.Waypoints.Count);
                Log.Out("[t3taAutopilot] route planned: " + follower.Length.ToString("F0") + "m, " +
                    rr.Waypoints.Count + " points");
            }
        }

        // ------------------------------------------------------------------
        // Gyrocopter: straight line, high enough to clear what's ahead.
        // ------------------------------------------------------------------
        static void Fly(EntityVehicle veh, EntityPlayerLocal drv, MovementInput mi)
        {
            if (!veh.IsEngineRunning && veh.vehicle.HasEnginePart())
            {
                // no brake input here: jump would pitch the nose up
                WriteFlightInput(mi, 0f, 0f, false, false);
                return;
            }
            // the landing counts as in the air: switched off at touchdown, nothing would brake the
            // roll-out (not the taxi of a gyro that never took off: that one stops like a car)
            if (DestinationGone(drv, !grounded || (flown && (pilot.Phase == "approach" || pilot.Phase == "taxi"))))
            {
                return;
            }
            Vector3 liveDest;
            PoiArea? liveArea;
            bool explicitRetarget = false;
            if (TryGetDestination(drv, out liveDest, out liveArea) && (retargetWanted || HorizontalDist(liveDest, markerPos) > 5f))
            {
                explicitRetarget = retargetWanted;
                retargetWanted = false;
                Vector3 oldLanding = landing;
                bool onStrip = HorizontalDist(landing, destination) > 1f;   // not just the old destination
                SetDestination(liveDest, liveArea, true);
                if (!explicitRetarget && onStrip && HorizontalDist(veh.position, oldLanding) < KeepLandingDist &&
                    HorizontalDist(oldLanding, destination) <= LandingRadius)
                    landing = oldLanding;   // the destination moved a little: land where we were heading
                else
                    PickLanding(veh.position);
                SetRouteTop(veh.position);
            }
            if (float.IsNaN(pilot.RouteTop) && roadNet != null && roadNet.HasTerrain)
            {
                PickLanding(veh.position);   // map finished loading after engage
                SetRouteTop(veh.position);
            }
            if (!landingRefined && HorizontalDist(veh.position, destination) < LandingRefineDist)
            {
                landingRefined = true;
                PickLanding(veh.position);   // the approach direction is known well now
            }

            Transform pt = veh.PhysicsTransform;
            Vector3 pos = veh.position;
            Vector3 vel = veh.vehicleRB != null ? veh.vehicleRB.velocity : Vector3.zero;
            float yawRate = veh.vehicleRB != null ? veh.vehicleRB.angularVelocity.y * Mathf.Rad2Deg : 0f;
            if (Time.time - lastTerrainScan > 0.1f)
            {
                lastTerrainScan = Time.time;
                ScanTerrain(veh, pos, vel);
            }
            // low down (and not landing): what's in front, level out in every direction
            if ((grounded || pos.y - groundBelow < Pilot.ClimbOutHeight) && pilot.Phase != "approach" && pilot.Phase != "taxi")
            {
                if (Time.time - lastClimbScan > 0.25f / ClimbBatches)
                {
                    lastClimbScan = Time.time;
                    ScanClimbOut(veh, pos, vel);
                }
            }
            else
            {
                pilot.ClimbScanned = false;
                climbBatch = 0;
            }
            if (grounded) ScanTaxiObstacles(veh, vel);
            if (explicitRetarget) pilot.Retarget(grounded, pos.y - groundBelow);
            pilot.Cruise = FlightCruise(veh);   // mods / buffs can change it mid-flight
            var o = pilot.Step(pos, pt.forward, vel, yawRate, landing, groundBelow, topAhead, destGround,
                grounded, Time.deltaTime);
            if (pilot.GoingAround != lastGoingAround)
            {
                lastGoingAround = pilot.GoingAround;
                Telemetry.Event(pilot.GoingAround ? "go_around" : "go_around_end", "agl", pos.y - groundBelow,
                    "dist", HorizontalDist(pos, landing));
                Log.Out("[t3taAutopilot] " + (pilot.GoingAround ? "overshot low - going around" : "go-around done, coming back round"));
            }
            if ((pilot.Detour != 0f) != (lastDetour != 0f) || Mathf.Abs(pilot.Detour - lastDetour) > 20f)
            {
                Telemetry.Event("climb_detour", "deg", pilot.Detour, "course_need", pilot.CourseNeed,
                    "agl", pos.y - groundBelow, "grounded", grounded);
                if (Cfg.DebugProbes)
                    Log.Out("[t3taAutopilot] climb-out " + (pilot.Detour != 0f ? "turned " + pilot.Detour.ToString("F0") + " deg off course" : "on course") +
                        " (course needs " + pilot.CourseNeed.ToString("F2") + ")");
                lastDetour = pilot.Detour;
            }
            if (pilot.NoTakeoffRun)
            {
                WriteFlightInput(mi, 0f, 0f, true, true);
                Telemetry.Event("no_takeoff_run", "course_need", pilot.CourseNeed);
                Disengage(drv, "Autopilot: no clear take-off run - move to open ground first");
                return;
            }
            if (o.Arrived)
            {
                WriteFlightInput(mi, 0f, 0f, true, true);
                Disengage(drv, "Autopilot: arrived");
                return;
            }
            if (o.Blocked)
            {
                WriteFlightInput(mi, 0f, 0f, true, true);
                Disengage(drv, "Autopilot: ground path blocked - take it from here");
                return;
            }
            if (o.Landed)
            {
                WriteFlightInput(mi, 0f, 0f, true, true);
                Telemetry.Event("landed_on_something", "agl", pos.y - groundBelow, "dist", HorizontalDist(pos, landing));
                Disengage(drv, "Autopilot: came down on something short of the landing site - take it from here");
                return;
            }
            if (Cfg.DebugProbes && Time.time - lastStatusLog > 2f)
            {
                lastStatusLog = Time.time;
                Log.Out("[t3taAutopilot] fly " + pilot.Phase + " y=" + pos.y.ToString("F0") + " alt*=" + pilot.TargetAlt.ToString("F0") +
                    " agl=" + (pos.y - groundBelow).ToString("F0") + " top=" + topAhead.ToString("F0") +
                    " v=" + new Vector3(vel.x, 0f, vel.z).magnitude.ToString("F1") + " vy=" + vel.y.ToString("F1") +
                    " pitch=" + (Mathf.Asin(Mathf.Clamp(pt.forward.y, -1f, 1f)) * Mathf.Rad2Deg).ToString("F0") +
                    " want=" + pilot.DesiredPitch.ToString("F0") + " routeTop=" + pilot.RouteTop.ToString("F0") +
                    " dist=" + HorizontalDist(pos, landing).ToString("F0") +
                    (pilot.Detour != 0f ? " detour=" + pilot.Detour.ToString("F0") : "") +
                    " gnd=" + grounded + " in=" + o.Throttle.ToString("F2") + "/" + o.Yaw.ToString("F2") +
                    (o.Up ? " U" : "") + (o.Down ? " D" : ""));
            }
            WriteFlightInput(mi, o.Throttle, o.Yaw, o.Up, o.Down);
        }

        /// <summary>
        /// Driving: a marker inside a trader compound / checkpoint is replaced by
        /// the nearest road point outside it (the car would only ram the fence).
        /// A quest POI (area) is not driven into either: the car stops on the
        /// road outside it. Flying keeps the marker: the landing strip search
        /// avoids roadblocks.
        /// </summary>
        static void SetDestination(Vector3 marker, PoiArea? area, bool flight)
        {
            markerPos = marker;
            markerArea = area;
            arrivedAt = -1f;   // a new target: a quest arrival hold starts over there
            destination = marker;
            if (flight) return;
            destination = OutsideTarget(marker, area, true);
        }

        /// <summary>
        /// Where a vehicle can stop for this marker: outside the quest POI's
        /// footprint, or on the road outside a trader compound / checkpoint;
        /// else the marker itself.
        /// </summary>
        static Vector3 OutsideTarget(Vector3 marker, PoiArea? area, bool log)
        {
            var net = roadNet;
            Vector3 destination = marker;
            if (area.HasValue)
            {
                // without road data (network client, missing splatmap) the POI
                // footprint alone still keeps the car out of the building
                Vector3 from = vehicle != null ? vehicle.position : marker;
                Rect rect = area.Value.Rect;
                if (area.Value.NearMarker)
                {
                    // the rally marker: the road outside the POI nearest to it, else
                    // just outside the footprint edge nearest to it
                    destination = net != null ? net.OutsideAreaNear(marker, rect, RallyRoadExtra)
                        : RoadNetwork.EdgeOutside(marker, rect, marker);
                }
                else
                {
                    destination = net != null ? net.OutsideArea(marker, rect, from, OutsideAreaSearch)
                        : RoadNetwork.EdgeOutside(marker, rect, from);
                }
                if (log)
                {
                    Log.Out("[t3taAutopilot] destination is a quest POI - stopping outside it, " +
                        HorizontalDist(destination, marker).ToString("F0") + " m from its center");
                    Telemetry.Event("dest_outside_poi", "marker", marker, "target", destination);
                }
                return destination;
            }
            if (net == null) return destination;
            Vector3 outside = net.OutsideRoadblocks(marker, OutsideRoadblockSearch);
            if (HorizontalDist(outside, marker) > 1f)
            {
                destination = outside;
                if (log)
                {
                    Log.Out("[t3taAutopilot] destination is inside a fenced / barricaded prefab - stopping on the road outside, " +
                        HorizontalDist(outside, marker).ToString("F0") + " m away");
                    Telemetry.Event("dest_outside_roadblock", "marker", marker, "target", outside);
                }
            }
            return destination;
        }

        static bool SameArea(PoiArea? a, PoiArea? b)
        {
            if (a.HasValue != b.HasValue) return false;
            return !a.HasValue || (a.Value.NearMarker == b.Value.NearMarker && a.Value.Rect == b.Value.Rect);
        }

        /// <summary>
        /// Cruise speed for the gyro: the horizontal speed cap the game applies
        /// with the throttle on (EntityVehicle clamps it to the turbo
        /// velocityMax, 15 for the gyro, times VehicleVelocityMaxPer: a
        /// supercharger adds 18%, some buffs cut it). Flying at the cap; above
        /// it the game would only clip the speed.
        /// </summary>
        static float FlightCruise(EntityVehicle veh)
        {
            var v = veh.vehicle;
            float cap = v != null ? v.VelocityMaxTurboForward * v.EffectVelocityMaxPer : 0f;
            return cap > 1f ? cap : 15f;
        }

        static void SetRouteTop(Vector3 from)
        {
            var net = roadNet;
            pilot.RouteTop = net != null ? net.MaxHeightAlong(from, landing, 30f) : float.NaN;
        }

        /// <summary>Road or flat open strip near the destination to land on (else the destination itself).</summary>
        static void PickLanding(Vector3 from)
        {
            landing = destination;
            var net = roadNet;
            if (from == Vector3.zero || net == null || !net.HasTerrain)
            {
                return;
            }
            if (!landingRefined && HorizontalDist(from, destination) < LandingRefineDist) landingRefined = true;
            Vector3 site;
            bool onRoad;
            if (net.FindLandingSite(from, destination, LandingRadius, out site, out onRoad))
            {
                landing = site;   // y: the strip's heightmap elevation (used until its chunk loads)
                Telemetry.Event("landing_site", "site", site, "road", onRoad, "from_dest", HorizontalDist(site, destination));
                Log.Out("[t3taAutopilot] landing on " + (onRoad ? "road" : "open ground") + " at (" + site.x.ToString("F0") +
                    ", " + site.z.ToString("F0") + "), " + HorizontalDist(site, destination).ToString("F0") + " m from the destination");
            }
            else
            {
                // No strip: not onto the marker itself when that's inside a trader
                // compound / checkpoint or a quest building - where a car would stop
                // (recorded: flew into the trader's building, dropped inside the fence)
                landing = OutsideTarget(destination, markerArea, false);
                bool moved = HorizontalDist(landing, destination) > 1f;
                Telemetry.Event("landing_site", "site", landing, "road", false, "none_within", LandingRadius, "outside", moved);
                Log.Out("[t3taAutopilot] no road / flat strip within " + LandingRadius + " m - landing at the " +
                    (moved ? "road outside the destination's compound / building, " + HorizontalDist(landing, destination).ToString("F0") + " m away" : "destination"));
            }
        }

        static readonly RaycastHit[] downHits = new RaycastHit[16];
        const float ScanTop = 320f;   // world y the downward rays start from (build limit ~255)

        /// <summary>
        /// Ground / roof / tree heights (world y) under the craft, along its
        /// course ahead and at the destination, from downward rays. Unloaded
        /// chunks give no hit and are skipped.
        /// </summary>
        static void ScanTerrain(EntityVehicle veh, Vector3 pos, Vector3 vel)
        {
            float below;
            bool hitBelow = SurfaceAt(pos, out below);
            groundBelow = hitBelow ? below : pos.y - 50f;
            Vector3 feet = veh.PhysicsTransform.TransformPoint(new Vector3(0f, groundY, 0f)) + Origin.position;
            grounded = hitBelow && feet.y - below < 0.4f && Mathf.Abs(vel.y) < 1.5f;
            if (!hitBelow || feet.y - below > FlownHeight) flown = true;   // not a hop over a bump while taxiing

            Vector3 course = new Vector3(vel.x, 0f, vel.z);
            float speed = course.magnitude;
            if (speed < 3f)
            {
                Vector3 f = veh.PhysicsTransform.forward;
                course = new Vector3(f.x, 0f, f.z);
            }
            course.Normalize();
            Vector3 side = new Vector3(course.z, 0f, -course.x);
            float reach = Mathf.Min(20f + 10f * speed, 170f);
            // On the approach only what's flown over before touching down
            // counts: the floor over the houses / trees past a quest's strip
            // held recorded approaches 3-12 m over it until they overflew it.
            if (pilot.Phase == "approach" && !pilot.LandingLong)
                reach = Mathf.Min(reach, HorizontalDist(pos, landing) - LandingScanShort);
            float top = hitBelow ? below : float.MinValue;
            for (float d = 4f; d <= reach; d += 8f)
            {
                for (int k = -1; k <= 1; k++)
                {
                    float h;
                    if (SurfaceAt(pos + course * d + side * (5f * k), out h) && h > top) top = h;
                }
            }
            topAhead = top > float.MinValue ? top : pos.y - 30f;

            float dg;
            destGround = SurfaceAt(landing, out dg) ? dg : landing.y;
        }

        /// <summary>Taxi and landing roll-out: scan both nose and travel direction far enough to brake.</summary>
        static void ScanTaxiObstacles(EntityVehicle veh, Vector3 vel)
        {
            Transform pt = veh.PhysicsTransform;
            Vector3 feet = pt.TransformPoint(new Vector3(0f, groundY, 0f));
            Vector3 half = new Vector3(halfWidth + ProbeSideMargin, ProbeHalfHeight, 0.05f);
            Vector3 origin = feet + Vector3.up * ProbeHeight;
            Vector3 nose = new Vector3(pt.forward.x, 0f, pt.forward.z);
            Vector3 unused;
            pilot.NoseFree = nose.sqrMagnitude > 1e-4f
                ? Cast(veh, origin, half, nose.normalized, Quaternion.LookRotation(nose, Vector3.up),
                    Pilot.TaxiProbeReach, out unused) : 0f;
            Vector3 travel = new Vector3(vel.x, 0f, vel.z);
            if (travel.sqrMagnitude > 1f)
            {
                pilot.NoseFree = Mathf.Min(pilot.NoseFree, Cast(veh, origin, half, travel.normalized,
                    Quaternion.LookRotation(travel, Vector3.up), Pilot.TaxiProbeReach, out unused));
            }
        }

        static readonly float[] climbFree = new float[Pilot.ClimbLevels.Length];

        /// <summary>
        /// Take-off / climb-out: a box as wide as the rotor swept level out
        /// from the wheel line at each of Pilot.ClimbLevels, in each of
        /// Pilot.ClimbDirs directions; Pilot.ClimbNeed gets the climb gradient
        /// each direction needs. The downward rays miss trees; these hit the
        /// trunks and walls in front.
        /// Each call sweeps one ClimbBatches-th of the directions (a full sweep
        /// per 0.25 s without the whole load landing on one frame), and airborne
        /// only those within 90 deg of the heading plus the course: at most
        /// 64 box casts a call, ~13 of the 24 directions once airborne.
        /// </summary>
        static void ScanClimbOut(EntityVehicle veh, Vector3 pos, Vector3 vel)
        {
            Transform pt = veh.PhysicsTransform;
            Vector3 feet = pt.TransformPoint(new Vector3(0f, groundY, 0f));
            Vector3 half = new Vector3(Mathf.Max(halfWidth, 2f) + ProbeSideMargin, 0.6f, 0.05f);
            // below the rotor only the body's width: at the rotor's width the
            // guardrails beside a road blocked every way along it (recorded:
            // left and right in turn, then pushed into the rail, stuck)
            Vector3 bodyHalf = new Vector3(halfWidth + ProbeSideMargin, 0.6f, 0.05f);
            // the course: to the landing site, or straight ahead while going around
            Vector3 toDest = landing - pos;
            toDest.y = 0f;
            Vector3 velFlat = new Vector3(vel.x, 0f, vel.z);
            if (pilot.GoingAround && velFlat.sqrMagnitude > 1f) toDest = velFlat;
            int course = Pilot.DirIndex(toDest);
            int heading = Pilot.ScanHeading(vel, pt.forward, grounded);
            int per = Pilot.ClimbDirs / ClimbBatches;
            Vector3 unused;
            // the exact course, every call: what's flown when the course is clear
            if (toDest.sqrMagnitude > 1f)
            {
                Vector3 cdir = toDest.normalized;
                Quaternion crot = Quaternion.LookRotation(cdir, Vector3.up);
                for (int k = 0; k < Pilot.ClimbLevels.Length; k++)
                    climbFree[k] = Cast(veh, feet + Vector3.up * Pilot.ClimbLevels[k],
                        Pilot.ClimbLevels[k] < Pilot.RotorLevel ? bodyHalf : half, cdir, crot, Pilot.ClimbReach, out unused);
                pilot.CourseNeed = Pilot.NeedGradient(climbFree, Pilot.TakeoffRoll(grounded, vel, cdir));
            }
            for (int i = climbBatch * per; i < (climbBatch + 1) * per; i++)
            {
                if (!Pilot.NeedsScan(i, course, heading, grounded))
                {
                    pilot.ClimbNeed[i] = float.PositiveInfinity;   // out of reach: never picked
                    continue;
                }
                Vector3 dir = Pilot.DirVector(i);
                Quaternion rot = Quaternion.LookRotation(dir, Vector3.up);
                for (int k = 0; k < Pilot.ClimbLevels.Length; k++)
                    climbFree[k] = Cast(veh, feet + Vector3.up * Pilot.ClimbLevels[k],
                        Pilot.ClimbLevels[k] < Pilot.RotorLevel ? bodyHalf : half, dir, rot, Pilot.ClimbReach, out unused);
                pilot.ClimbNeed[i] = Pilot.NeedGradient(climbFree, Pilot.TakeoffRoll(grounded, vel, dir));
            }
            // right in front of the nose, the body's width: turning on the wheels
            // doesn't creep forward into what's there
            Vector3 nose = new Vector3(pt.forward.x, 0f, pt.forward.z);
            if (nose.sqrMagnitude > 1e-4f)
            {
                nose.Normalize();
                pilot.NoseFree = Cast(veh, feet + Vector3.up * Pilot.ClimbLevels[0], bodyHalf, nose,
                    Quaternion.LookRotation(nose, Vector3.up), Pilot.NoseRoom + 1f, out unused);
            }
            climbBatch = (climbBatch + 1) % ClimbBatches;
            if (climbBatch == 0) pilot.ClimbScanned = true;   // every direction swept at least once
        }

        /// <summary>Highest surface (world y) at x/z that isn't an entity (our craft, zombies, vehicles).</summary>
        static bool SurfaceAt(Vector3 world, out float y)
        {
            y = 0f;
            Vector3 origin = new Vector3(world.x, ScanTop, world.z) - Origin.position;
            int n = Physics.RaycastNonAlloc(origin, Vector3.down, downHits, ScanTop + 64f, probeMask,
                QueryTriggerInteraction.Ignore);
            bool found = false;
            for (int i = 0; i < n; i++)
            {
                var h = downHits[i];
                if (h.transform.name.IndexOf("grass", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                var e = GameUtils.GetHitRootEntity(h.transform.tag, h.transform) ??
                        h.transform.GetComponentInParent<Entity>();
                if (e != null) continue;
                float wy = h.point.y + Origin.position.y;
                if (!found || wy > y) { y = wy; found = true; }
            }
            return found;
        }

        static void WriteFlightInput(MovementInput mi, float throttle, float yaw, bool up, bool down)
        {
            mi.moveForward = throttle;
            mi.moveStrafe = yaw;
            mi.jump = up;                    // gyro: nose up (and wheel brake)
            mi.down = down;                  // gyro: nose down; both = brake without pitching
            mi.running = true;
            mi.lastInputController = true;
        }

        static void WriteInput(MovementInput mi, float throttle, float steer, bool brake)
        {
            mi.moveForward = throttle;
            mi.moveStrafe = steer;
            mi.jump = brake;                 // vehicle brake is bound to the "jump" flag
            mi.down = false;
            mi.running = true;               // turbo torque/speed table; also avoids the 50% motor cut
            mi.lastInputController = true;   // analog steering: moveStrafe is an absolute wheel target
        }

        internal static float HorizontalDist(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x, dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }

        static string FreeSummary()
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < probeFree.Length; i++)
            {
                if (i > 0) sb.Append(' ');
                sb.Append(probeFree[i].ToString("F0"));
            }
            return sb.ToString();
        }

        // ------------------------------------------------------------------
        // Obstacle scan: a fan of boxes as wide as the vehicle swept from the
        // front bumper, at 0.6-1.4 m above the wheels' ground line, tilted
        // with the vehicle so slopes it sits on stay parallel to the sweep.
        // All in PhysicsTransform (origin-shifted physics) space.
        // ------------------------------------------------------------------
        // BoxCastNonAlloc returns hits in no particular order and silently
        // drops the rest once the buffer is full. A 30 m sweep through roadside
        // grass (which has colliders) filled the old 24 slots, so fences and
        // wrecks were dropped until a few meters away.
        static readonly RaycastHit[] hitBuffer = new RaycastHit[512];
        static int saturatedCasts;

        /// <summary>
        /// Orientation of the probe fan. A bike banks 5-12 deg into every
        /// bend, and a fan banked with it swept the ground on the inside:
        /// recorded rides lost 2-4 m of free distance on the inside of turns
        /// (a car loses it on the outside, where the roadside is) and swerved
        /// wide around nothing. So a two-wheeler keeps its pitch but not its
        /// roll; a car still tilts with the body to match a side slope.
        /// </summary>
        static Quaternion ProbeFrame(Transform pt)
        {
            if (!leans) return pt.rotation;
            Vector3 f = pt.forward;
            return Mathf.Abs(f.y) < 0.95f ? Quaternion.LookRotation(f, Vector3.up) : pt.rotation;
        }

        /// <summary>Center of the probe box at the front bumper, in the probe frame.</summary>
        static Vector3 ProbeOrigin(Transform pt, Quaternion frame)
        {
            Vector3 local = new Vector3(0f, groundY + ProbeHeight, frontZ - 0.5f);
            return leans ? pt.position + frame * local : pt.TransformPoint(local);
        }

        /// <param name="autopilot">false while the player drives: fill probeFree for the recorder only</param>
        static void ScanObstacles(EntityVehicle veh, float range, bool autopilot)
        {
            Transform pt = veh.PhysicsTransform;
            Quaternion frame = ProbeFrame(pt);
            Vector3 up = frame * Vector3.up;
            Vector3 origin = ProbeOrigin(pt, frame);
            Vector3 half = new Vector3(halfWidth + ProbeSideMargin, ProbeHalfHeight, 0.05f);
            for (int i = 0; i < Driver.ProbeAngles.Length; i++)
            {
                Vector3 dir = frame * (Quaternion.Euler(0f, Driver.ProbeAngles[i], 0f) * Vector3.forward);
                float reach = Driver.ProbeReach(i, range);
                Vector3 hitPoint;
                float free = Cast(veh, origin, half, dir, Quaternion.LookRotation(dir, up), reach, out hitPoint);
                float was = probeFree[i];
                probeFree[i] = free >= reach ? range : free;
                if ((Cfg.DebugProbes || Telemetry.Active) && i == Driver.ProbeAngles.Length / 2 && was > 14f && free < 8f)
                {
                    // something appeared close without being seen from afar
                    string hits = DescribeHits(veh, origin, half, dir, Quaternion.LookRotation(dir, up), reach);
                    if (Cfg.DebugProbes && autopilot)
                    {
                        Log.Out("[t3taAutopilot] late hit center " + was.ToString("F0") + "->" + free.ToString("F1") + "m: " + hits);
                    }
                    Telemetry.Event("late_hit", "was", was, "free", free, "hits", hits);
                }
                if (free < reach && autopilot)
                {
                    driver.AddObstaclePoint(hitPoint + Origin.position, Time.time);   // physics -> world
                }
            }
        }

        /// <summary>
        /// Distance to the first real obstacle along the sweep, or dist if
        /// clear; point = where it was touched (physics space). Ignored: our
        /// own vehicle, every entity that isn't a vehicle (zombies, animals,
        /// players, corpses, dropped items - just drive through them), grass
        /// and cactus colliders (DriveThrough), and drivable ground (surface
        /// normal within 45 deg of up).
        /// </summary>
        static float Cast(EntityVehicle veh, Vector3 origin, Vector3 half, Vector3 dir, Quaternion rot, float dist,
            out Vector3 point)
        {
            float nearest = dist;
            point = Vector3.zero;
            int n = Physics.BoxCastNonAlloc(origin, half, dir, hitBuffer, rot, dist, probeMask,
                QueryTriggerInteraction.Ignore);
            if (n == hitBuffer.Length && Cfg.DebugProbes && (++saturatedCasts & 255) == 1)
            {
                Log.Warning("[t3taAutopilot] probe hit buffer full (" + n + ") - obstacles may be missed");
            }
            for (int i = 0; i < n; i++)
            {
                var h = hitBuffer[i];
                if (h.distance <= 0f) continue;          // started overlapping (own body, curb)
                if (h.distance >= nearest) continue;
                if (DriveThrough(h.transform.name)) continue;
                if (Vector3.Angle(h.normal, Vector3.up) < 45f) continue;
                if (IsSlopeGraze(h, dir)) continue;
                // body-part tags resolve via GetHitRootEntity; untagged colliders
                // (character capsules, ragdolls) via the hierarchy
                var e = GameUtils.GetHitRootEntity(h.transform.tag, h.transform) ??
                        h.transform.GetComponentInParent<Entity>();
                if (e == veh) continue;
                if (e != null && !(e is EntityVehicle)) continue;
                nearest = h.distance;
                point = h.point;
            }
            return nearest;
        }

        /// <summary>
        /// A terrain hit with a steep contact normal is usually the sweep's
        /// bottom edge grazing the start of a rise (the box sits 0.6 m above
        /// the wheels), not a wall. Look at the ground 1 m further on: if it
        /// rises less than MaxRisePerMeter there, the slope is drivable.
        /// </summary>
        static bool IsSlopeGraze(RaycastHit h, Vector3 dir)
        {
            if (h.collider == null || h.collider.name != "terrainCollider") return false;
            Vector3 flat = new Vector3(dir.x, 0f, dir.z);
            if (flat.sqrMagnitude < 1e-4f) return false;
            Vector3 top = h.point + flat.normalized * 1f + Vector3.up * 3f;
            RaycastHit g;
            if (!h.collider.Raycast(new Ray(top, Vector3.down), out g, 6f)) return false;
            return g.point.y - h.point.y < MaxRisePerMeter;
        }

        const float MaxRisePerMeter = 0.8f;   // ~39 deg

        /// <summary>
        /// Plants the car goes straight through instead of steering round.
        /// Cactus: hitting one costs the vehicle about its block health x 2.5
        /// (~187 for a 4x4, whatever the speed above a crawl - EntityVehicle
        /// caps the self damage at the damage the block took), but weaving
        /// round every cactus in a desert held the car at the 8 m/s avoid cap
        /// and swung the offset between +-6 m for a minute on a recorded ride.
        /// </summary>
        static bool DriveThrough(string name)
        {
            return name.IndexOf("grass", StringComparison.OrdinalIgnoreCase) >= 0 ||
                name.IndexOf("cactus", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        /// <summary>Debug: the nearest hits of one sweep and why each was kept or skipped.</summary>
        static string DescribeHits(EntityVehicle veh, Vector3 origin, Vector3 half, Vector3 dir, Quaternion rot, float dist)
        {
            int n = Physics.BoxCastNonAlloc(origin, half, dir, hitBuffer, rot, dist, probeMask,
                QueryTriggerInteraction.Ignore);
            System.Array.Sort(hitBuffer, 0, n, HitDistance.Instance);
            var sb = new System.Text.StringBuilder("n=" + n);
            for (int i = 0, shown = 0; i < n && shown < 8; i++)
            {
                var h = hitBuffer[i];
                float ang = Vector3.Angle(h.normal, Vector3.up);
                var e = GameUtils.GetHitRootEntity(h.transform.tag, h.transform) ??
                        h.transform.GetComponentInParent<Entity>();
                if (e == veh) continue;   // own body parts
                shown++;
                string why = h.distance <= 0f ? "overlap"
                    : DriveThrough(h.transform.name) ? "drive-through"
                    : ang < 45f ? "ground"
                    : IsSlopeGraze(h, dir) ? "slope"
                    : e != null && !(e is EntityVehicle) ? "entity" : "HIT";
                sb.Append(" | ").Append(h.collider.name).Append('/').Append(h.transform.name)
                    .Append(" L").Append(h.collider.gameObject.layer)
                    .Append(" d=").Append(h.distance.ToString("F1"))
                    .Append(" a=").Append(ang.ToString("F0"))
                    .Append(' ').Append(why);
            }
            return sb.ToString();
        }

        sealed class HitDistance : System.Collections.Generic.IComparer<RaycastHit>
        {
            public static readonly HitDistance Instance = new HitDistance();
            public int Compare(RaycastHit a, RaycastHit b) => a.distance.CompareTo(b.distance);
        }
    }
}
