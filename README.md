# t3ta Autopilot

Autopilot for 7 Days to Die (v3.2). Out of the box it flies the
**gyrocopter**: take-off, cruise above the terrain, landing on a road or flat
spot near the destination. Driving the other vehicles along the roads is
experimental and off by default (`"groundVehicles": true` in `t3taAutopilot.json`,
see Configuration).

## Install

1. Download `t3taAutopilot-<version>.zip` from the
   [releases](https://github.com/t3ta/t3taAutopilot/releases) and extract it
   into the game's `Mods` folder, so that `Mods/t3taAutopilot/ModInfo.xml`
   exists (the `Mods` folder of the game install, or
   `%APPDATA%\7DaysToDie\Mods`).
2. Start the game with **EAC off** (any .dll mod needs that).

See Notes / limitations for multiplayer.

## Usage

1. Put a waypoint on the map and **track** it (or set the quick marker).
2. Get in the driver's seat of a gyrocopter.
3. Press **G** — it takes off, flies to the waypoint and lands near it (see
   Gyrocopter below). With `groundVehicles` on, any other vehicle drives itself
   toward the waypoint, following roads when a road splatmap is available.
4. Press **G** again, or touch **WASD / Space / C**, to take over manually.

The destination is re-read every frame, so you can move the marker while
driving and the vehicle will re-plan and follow.

### Destinations

**Hold G** for a destination menu on the game's radial (like hold-E): move the
cursor onto an entry and release G to drive there. Release over nothing to
cancel. In the driver's seat, picking an entry engages the autopilot; while
driving, the vehicle re-plans to the new target.

| Entry      | Target |
|------------|--------|
| Stop       | disengage (only while driving) |
| Map marker | the tracked waypoint, else the quick marker (default) |
| Home       | your bedroll |
| List...    | opens the destination list (below) |
| one per quest | each active quest that shows a marker, nearest first, with its distance (as many as fit: the radial has 13 slots) |

**Destination list**: the radial's `List...` opens a window listing everything
the radial offers plus **every saved waypoint** and every quest, nearest first,
with the distance. Tabs filter by kind (All / Quests / Waypoints, the latter
with map marker and home), 8 rows per page; the current destination is
highlighted. While driving, `Stop Autopilot` stays at the top of every tab, so
you can stop from any of them. **Go** drives there, exactly like picking it in
the radial; Esc or Close closes it. A picked waypoint is followed while that
very waypoint exists (renaming it is fine; another waypoint at the same spot
doesn't count): delete it while driving and the autopilot stops after 2 s. The window is defined in
`Config/XUi_InGame` (deployed along with the DLL).

**Bookmarks**: the star next to `Go` bookmarks a waypoint, quest or home
(the map marker moves, so it has no star). Bookmarked rows come first on every
tab (after `Stop Autopilot`) and have their own **Bookmarks** tab. The list
keeps its order while it is open, so a row doesn't jump away when you click its
star; it is re-sorted on the next tab switch or reopen. Bookmarks are kept per
world and save in `Mods/t3taAutopilot/bookmarks.json`: a waypoint by its position
(moving it drops the bookmark), a quest by its quest code.

The current destination is highlighted. A quick **tap** of G still toggles the
autopilot for the current destination - or, when a quest is tracked and shows a
marker, for that quest (as if picked from the menu). **Shift+tap** cycles map marker → quest
(tracked, else nearest) → home without the menu. While driving, it skips the
ones with nothing to drive to. A quest picked from the menu is followed until it
is done. The autopilot doesn't switch to another quest by itself: if the quest
is abandoned, failed or turned in while driving, the autopilot stops. The same
goes for the quest / home sources when nothing is left to drive to (no quest
with a marker, bedroll picked up) for 2 s - 10 s for a quest, whose marker can
be gone for a few seconds while it moves to the rally point near the POI. A
gyro in the air doesn't stop for that: it lands where it was heading. The map
marker source keeps driving to the last marker seen. Entering another world or save resets the destination
to the map marker.

Picking an entry with nothing to drive to (e.g. `Map marker (none)`) keeps the
current destination. Picking a destination again always re-plans, even when the
new target is next to the old one (a quest POI at the map marker still stops
outside the POI).
For the gyrocopter, picking a destination clears the old landing sequence.
If it is still airborne below the flare height, it climbs straight out before
turning toward the new target. Small automatic quest-marker updates keep the
current landing strip when it remains suitable.

- A quest marker follows the quest: the POI while it is in progress, the quest
  giver once it is ready to turn in, the dig circle for buried supplies. When
  the objective changes mid-drive, the route changes with it.
- The vehicle doesn't drive into a quest POI. It stops on the road just outside
  its footprint, on the side you arrive from; a road on the far side is used
  only if the near side has none. If there is no road within 60 m, or no road
  data at all (a multiplayer client has no world files), it stops just outside
  the edge. On a generated world 1099 of 1105 quest POIs got a road stop,
  1.9 m outside the footprint on average (90% within 4.3 m).
- **Rally marker**: near the POI the game moves the quest marker from the POI
  to its rally marker (the spot where you start the quest). The vehicle picks
  that up and heads for the road outside the POI nearest to the rally marker
  (a road up to 15 m farther than the footprint edge; else just outside the
  edge nearest to it). If it already stopped, it waits 5 s braked for the
  marker to move and then drives on. On two generated worlds (rotation-0 POIs
  with a rally marker, 547) the stop ended 10-12 m from the rally marker
  (median; 90% within 24 m), vs 21-23 m for the approach-side stop.
- Home drives to the bedroll itself. If the bedroll is behind walls, the
  vehicle stops where it gets stuck.

## How it works

- `VehicleInputPatch` — Harmony postfix on `EntityVehicle.MoveByAttachedEntity`.
  The game calls it each frame for the local driver to copy
  `PlayerActionsVehicle` input into `EntityVehicle.movementInput`; the postfix
  overwrites those fields while autopilot is engaged, so the stock physics path
  (`PhysicsFixedUpdate`) consumes them unchanged.
- `movementInput.moveStrafe` is written with `lastInputController = true`, which
  makes the game treat it as an absolute wheel-angle target (`-1..1` →
  `±SteerAngleMax`) instead of keyboard-style rate accumulation.
- `movementInput.jump` is the brake flag; `running = true` keeps full motor
  torque authority available.
- `EntityVehicle.isTurnTowardsLook` (static) is forced off while engaged and
  restored on disengage.

### Road routing

- `RoadNetwork` reads the world's **`splat3_processed.png`** (R = asphalt,
  G = gravel) - the final splat that includes streets stamped by town/tile
  prefabs. The raw `splat3.png` only holds RWG connector roads with gaps at
  every town (and is empty on Navezgane); it is used only as a fallback.
  The 8192² PNG is streamed row by row by `PngRoadMask` into a 2 m mask on a
  worker thread, preloaded as soon as the world is entered (~16 MB).
- Orientation: image top row = world **north**. Checked against
  `prefabs.xml` on three worlds: 90-100% of road-side prefab parts sit within
  12 m of road this way vs 24-31% z-mirrored. (v0.1's "top = south" flip
  mirrored every route onto roads that don't exist.)
- `RoutePlanner` runs A* on a 4 m grid on a worker thread. Road cells near
  the road edge cost more (drive in the middle, not on the shoulder), gravel
  costs a bit more than asphalt, cells deep inside big painted areas (lots,
  plazas) cost more.
- **Off-road where it's easy**: `RoadNetwork.Terrain` rates every cell from
  the world files, and off-road cost = `flatOffRoadCost` (1.6) × that rating:
  - slope from `dtm_processed.raw` (u16/256 = m, file row 0 = **south** -
    checked against prefab y in `prefabs.xml`): flat ≤7° ×1, ≤14° ×2.5,
    ≤24° ×7.5, steeper ×60
  - water from `splat4_processed.png` B (= water surface height; matches
    Navezgane's `water_info.xml`): ground below it ×60
  - buildings: where `dtm_processed.raw` differs from `dtm.raw` by >0.5 m
    (prefab placement re-leveled the ground), plus a 1-cell margin: ×20
  - trees: pine forest ×2, snow ×1.5, burnt ×1.4, wasteland ×1.2
  Off-road speed cap: 10 m/s on flat open ground, 7 m/s elsewhere. Worlds
  without a `dtm` fall back to a flat `offRoadCost` (12).
  (sim, 30 identical trips each, wreck every 35-80 m: total time roads-only
  → terrain-aware: West Xuyofu 100 → 84 min, Lulica 141 → 107 min.)
- The cell path becomes a dense 2 m polyline: each point is pulled sideways
  onto the centerline of the road it's on (fine mask scan), then smoothed.

### Controller

- `PathFollower` tracks progress along the polyline and steers with **pure
  pursuit** (look-ahead 3.5 m + 1.0 s x speed, 5-18 m, wheelbase measured
  from the wheel colliders). In a tight bend the look-ahead is shortened
  (down to 4 m) until the straight line to it stays within 1 m of the path:
  the full distance put the 4x4 3.7 m inside a town corner, into the stop
  sign on it.
- **Speed is planned from curvature ahead**: every point gets an allowed
  speed `sqrt(cornerAccel / curvature)` (default 3 m/s²), and the car brakes early enough (3.5 m/s²)
  to meet each one. This matters because the game scales the steering rate
  by `clamp(1 - (v/10)², 0.15, 1)` - above ~9 m/s the wheels barely turn,
  so a car that enters a bend fast simply cannot make it.
- `Driver` (pure logic, shared with the simulator) makes the per-frame
  decision. Obstacles: 13 **box sweeps** (vehicle width + 0.4 m margin each
  side, 0.6-1.4 m above the wheels' ground line, -60°..+60°) from the front
  bumper, using the collision layers the vehicle body actually hits. Own
  vehicle, **every non-vehicle entity** (zombies, animals, players, corpses,
  items - resolved by body-part tag or, for untagged colliders, the parent
  hierarchy; the car just drives through them), grass, **cacti** and
  drivable ground (normal within 45° of up) are ignored. A cactus costs the
  vehicle about its block health x 2.5 (~187) when hit at any speed above a
  crawl, but weaving round every one in a desert held the car at the avoid
  speed cap and swung it side to side.
- **Avoidance offsets the route sideways** (±1.5..6 m): the car pulls out,
  runs parallel past the obstacle at ≤12 m/s, and only steps back once the
  way back has stayed clear for 12 m of travel. An offset counts as clear
  when (a) the probe toward its pursuit point has `1.8 s × speed + 6 m` of
  room and (b) the curved route at that offset keeps the body 0.5 m from
  every obstacle contact point seen in the last 6 s - straight probes alone
  missed wrecks on the inside of bends. Speed is capped only by the free
  distance in the chosen direction (5 m/s² assumed), not by the car it is
  already swerving around. With a wreck every 35-80 m the sim averages
  10.7 m/s (vs 7.9 m/s with the earlier 6 m/s cap; 11.0 m/s on a clear road).
  A newly picked offset is held for 0.6 s unless a hard stop (20 m/s²) would
  no longer end short of what it aims at: the clear test flickers frame to
  frame, and re-picking every frame flipped the offset 0 <-> 6 m ten times a
  second (72 of 129 switches within 0.2 s in one recorded 4x4 run). In the sim
  that cut flips within 0.2 s by about 55%.
- Anything that holds the car still for 3 s (not only pushing against
  something) counts as stuck and triggers the reverse routine.
- **Throttle keeps sprint (turbo) engaged**: the game runs turbo only while
  `moveForward != 0`, and its speed cap sinks 14 → 10 m/s at 1.5 m/s² the
  moment turbo drops. Coasting/braking therefore hold a token 0.02 throttle
  and cruise uses a feed-forward throttle instead of on/off control
  (sim: 169 → 0.3 sprint toggles per km).
- Unstick: if pushing but not moving for 3 s, or boxed in for 1.2 s, it
  reverses with opposite steering for 1.7 s and then prefers the other side.
- **Off-road bypass**: stuck twice in a row (closed gate, wall across the
  road), it closes the next 14 m of route ±7 m plus everything its probes hit
  in the last 6 s, then plans a *local* detour (off-road cost 2) that leaves
  the road, goes round the closed area and rejoins the **same route** past
  it. A map-wide reroute tends to find another gated road in this game, so
  it deliberately doesn't. Cells right behind the vehicle stay open so the
  detour can start by backing off. Up to 8 per trip; closed cells are
  forgotten on the next trip. The bypass search reopens closed cells behind
  and beside the car (it's standing there, so it's drivable) and is rejected
  if it can't actually reach the rejoin point. After that it gives up after 4 retries.
  Replans when >25 m off route.
  (sim: wall 24 m / 50 m across the road -> 59/60 and 60/60 trips arrive)

### Checkpoints and trader compounds

Road checkpoints (`roadside_checkpoint_*`, the town-gateway
`part_driveway_gateway_checkpoint_*`) put military / concrete barriers, cones
and barricades on asphalt the splat map shows as a plain road, and trader
compounds are fenced. Every recorded stuck but one was at one of these. Their
footprints (from `prefabs.xml` + the game's `Data/Prefabs/*/NAME.xml`
`PrefabSize`, rotated, plus 8 m) cost 12x in A*, so plans and detours go
around when there is a way, and a destination inside one is moved to the
nearest road outside it. Gyro landing strips avoid them too.
Road drainage culverts (`part_road_drainage`) cost 12x on their off-road
cells only: the asphalt over the culvert is fine, the ditch beside it is not.
Every other POI footprint (inset 2 m; `part_*` pieces and town tiles
excluded) costs 4x, asphalt included: gas station forecourts and store car
parks are asphalt in the splat map, and plans and detours cut across them -
into bollards, fences and shopping carts - instead of taking the street
round the block.

### Gyrocopter

In a gyrocopter the autopilot flies instead of driving (no road routing):

- **Route**: a straight line to the destination, level at the highest ground
  along it (heightmap, ±30 m) + 50 m — trees and buildings aren't in the
  heightmap — capped at 250 m (the rotor's lift fades out at y 260). Where
  downward rays along the course measure something higher, it climbs over it.
- **Speed**: the game clips the gyro's horizontal speed to its turbo
  velocityMax (15 m/s) times VehicleVelocityMaxPer (a supercharger adds 18%,
  some buffs cut it); pitching down doesn't get past it. It cruises at that
  cap, read off the vehicle every frame. Faster than the stock 15 the powered
  dive starts further out on a proportionally flatter path (the sink rate is
  limited), so short hops gain nothing (600 m: 50 -> 53 s in the simulator at
  1.18) and long ones do (3 km: 210 -> 188 s). `maxCruiseSpeed` is for
  driving only.
- **Take-off**: turns on the wheels until the nose is within 20° of the
  course, then runs straight and lifts off; below the height it wants it
  climbs on full throttle (the thrust is what climbs). On the ground and
  below 40 m it sweeps boxes level out (0.8-36 m over the wheels, 100 m, 24
  directions) — the downward rays miss trees — and works out the climb
  gradient each direction needs after the ground roll. If the course needs
  more than 0.3 it takes off / climbs out along a direction that doesn't
  (airborne: within 90° of its heading), picked by how far it turns off the
  course and how much room it leaves (each 15° costs as much as needing a
  sixth of 0.3 more), and turns back onto the course once it needs 0.24 or
  less. On the ground it sticks to the direction it picked (while that needs
  up to 1.5 x 0.3, or is within 0.2 of the least bad one when nothing is
  climbable) instead of re-picking as it turns: recorded take-offs from a
  cluttered yard swung +120 / -75 / -150 deg and spun 8-12 s on the wheels.
  The course itself is swept along the exact line to the destination
  (15° steps would put the box 13 m off it 100 m out). The sweeps are spread
  over three calls per 0.25 s, and airborne cover only the directions within
  90° of the heading plus the course. On the ground with every direction
  blocked within the take-off run (a walled yard) it doesn't run at a wall: it
  brakes and disengages ("no clear take-off run").
  Once rolling faster than 5 m/s it keeps the run's direction while that
  direction is climbable, and turns onto the course once airborne (switching
  mid-run braked it to turn back, then lifted off swerving). Having turned
  off the course, it turns back onto it only after the course has read
  climbable for a second in a row: one sweep slips through a gap between
  trees (recorded: clear, then blocked 3 m ahead 0.35 s later, and it hit a
  tree 17 m up). Just off the ground the climb is still building up (0.1 at
  lift-off, 0.24 over the first 57 m), so the distance until it climbs at
  0.3 doesn't count toward clearing what's ahead.
  The sweeps below the rotor (0.8 m) are only the body's width: at the
  rotor's width the guardrails beside a road blocked every way along it
  (recorded: it swung left and right in turn, then pushed into the rail).
  Past the 1.5 x 0.3 it keeps a picked direction on the wheels, it gives it
  up only after reading too steep for 0.5 s. Lining up, it doesn't creep
  forward with less than 3 m in front of the nose (it turns on the spot,
  slower).
- **Turning back**: with the destination more than 135° off its course (it
  flew away from it, e.g. out of a go-around) it turns one way and holds that
  until round: straight away the nose and course errors (+-180 with opposite
  signs) cancelled and it flew on for kilometres (simulator).
- **Picking a destination in the air**: flying by hand with the autopilot
  off, opening the destination radial or list takes the hands off the
  controls, and the gyro would slow down and sink. It circles instead -
  holding its height (at least 30 m over the ground) at ~10 m/s on a ~45 m
  radius - until an entry is picked (the autopilot takes over from there) or
  the menu closes. Touch WASD / Space / C and it's yours again until the menu
  closes. Only in the air: on the wheels it stays put.
- **Control**: the game's gyro gets lift from forward speed and climbs by
  pitching its thrust, so height is held with nose up (jump) / down; it
  steers the course, not the nose (it slides), and the yaw command is
  smoothed — the game banks the craft toward it on an undamped spring.
- **Landing**: picks a level 60 m strip of road (preferred) or flat open
  ground along the approach within 120 m of the destination. The rotor lifts
  just under gravity whatever the pitch, so with the throttle off the gyro
  sinks at only ~2.4 m/s; nose down on power it sinks at 6-8 m/s. So it dives
  on power (0.5 slope, throttle for the sink rate) down to 30 m over the
  destination, then cuts the throttle and glides at ~7 m/s (0.28 slope),
  touches down at ~4.5 m/s about 15 m short, rolls and brakes (jump +
  down: their pitch torques cancel), and stops there. No strip found: lands
  where a car would stop - on the road outside a trader compound /
  checkpoint, outside a quest POI's footprint - else at the destination
  (recorded: it flew into a trader's building and dropped inside the fence).
  Come to rest just above the ground on the approach (on a fence, a block)
  for 1.5 s, it stops and hands over. Destinations within 110 m are reached on the wheels.
  On the approach it only stays over what stands up before the strip (the
  look-ahead stops 10 m short of it): the houses and trees past a quest's
  strip held it 3-12 m over the strip until it overflew it. When the
  destination moves close by (a quest's rally marker appears) it keeps the
  strip it's heading for if that's within 120 m of the new destination.
  If it overshoots anyway, passing within 60 m of the destination 8-20 m up with it more
  than 90° behind its course, it doesn't turn back that low (recorded:
  circling 2-9 m up at 22-32 deg/s, bouncing off the ground, drifting 47 m)
  and doesn't land on ground nobody checked either: it goes around - climbs
  straight ahead to 45 m (capped at the 250 m ceiling over high ground; over
  anything measured ahead, with the take-off
  sweeps against what's in front), flies out to 180 m from the destination,
  then comes back round to the landing site it picked. Lower than 8 m it's
  a long landing; after two go-arounds it lands long too. Landing long it
  also holds its course (no turn until the wheels touch, over anything
  measured ahead) and taxis back.

### Offline simulator

Run the deterministic safety regressions (no world files required):

```powershell
dotnet run --project tools/sim -c Release -- --regression
```

They cover gyrocopter taxi braking with residual thrust, explicit retargeting
during landing, and bypass segments staying outside closed navigation cells.

`tools/sim` compiles the mod's engine-independent sources
(`RoadNetwork`, `PngRoadMask`, `RoutePlanner`, `PathFollower`, `Driver`)
against the game's `UnityEngine.CoreModule` and drives them with a kinematic
vehicle that reproduces `EntityVehicle`'s speed-dependent steering rate, on a
real world's splatmap, with random wrecks on the road:

```powershell
cd tools/sim
dotnet run -c Release -- "<world dir>" 50 7 "<out dir>" 12
```

`AD_FENCES=1` adds fence posts along the road edge (what a swerve runs into),
`AD_GYRO=1` flies the gyrocopter autopilot (with tree lines / sheds in front on take-off; `AD_CLIMB_SCAN=0` without the level sweeps) against a model of the game's gyro
(lift, thrust, pitch/yaw torques, bank spring; `AD_VEL_PER` scales its speed caps like VehicleVelocityMaxPer, 1.18 = supercharger; `AD_VDRAG` sets its extra vertical drag, default 0.15 = the recorded sink; 0 makes it sink
faster), `AD_GYRO=1 AD_GYRO_REPLAY=<telemetry
csv>,t0,t1[,groundY]` replays recorded inputs through that model and prints
simulated vs recorded height / speed / sink / pitch, `AD_BODY_PAD=2.4` sweeps the low level at the rotor's width as before, `AD_TREES=1` sweeps take-offs through stands of single trees and
`AD_SCAN_MISS=p` lets each sweep level slip through a gap with probability p
(the output counts the detour switches), `AD_LANDAT=x,z,fromX,fromZ` shows the landing site for one destination (and the no-strip fallback), `AD_LANDSITE=N` reports
the landing strips picked for N random destinations, `AD_GYRO=1 AD_LOITER=1` circles for 90 s from six airborne starts
(height held, lowest point, slowest speed, radius). The gyro scenarios'
obstacles are boxes with a height: they check the pilot's take-off / climb-out
decisions, not the game's box casts (slopes, drive-through blocks and entities
the controller ignores). Check those in game with the `climb_detour` telemetry
events. Short taxi trips also scan ahead and along the travel direction,
reduce speed to allow the pusher to spin down, and stop with a
`ground path blocked` message when there is no safe way forward.

It prints per-trial and summary stats (arrival, % of distance with a wheel
off the road mask, max cross-track error, wreck contacts) and renders route
images. v0.1 controller vs now (50 random 0.7-2.2 km trips per world,
v0.1 given the corrected map orientation):

| world | off-road v0.1 → now | cross-track p95 | wreck contacts |
|---|---|---|---|
| West Xuyofu Territory | 21% → 3.3% | 4.4 → 0.9 m | 0 |
| Lulica Mountains | 20% → 3.0% | 4.5 → 1.1 m | 1 |
| Navezgane | 33% → 3.3% | 4.7 → 1.0 m | 0 |

### Telemetry

Off by default. With `"telemetry": true` in `t3taAutopilot.json`, every ride in a
driver's seat is recorded - autopilot and manual alike - to
`Mods/t3taAutopilot/telemetry/<date>_<vehicle>`:

- `.csv`: 10 Hz samples - pose, velocity, yaw/pitch/roll rates, the actual
  wheel angle, the inputs the vehicle received, the 13 obstacle probes (also
  while driving by hand: what the autopilot would have seen) and the autopilot
  state (target speed, avoidance offset, cross-track error, gyro phase).
- `.events.jsonl`: engage / disengage, route planned, avoidance on/off, late
  hits and what we got stuck on (collider names), detours, landing sites and
  manual **takeovers** with the inputs and autopilot state at that moment.

`python tools/telemetry/stuck_report.py` joins every stuck with the world files
(road mask, town tile, heightmap slope, prefabs, what the probes touched) and
draws a map per place. `python tools/telemetry/summarize.py [--takeovers]` lists the sessions and,
for each takeover, the 5 s of driving and 10 s of events that led up to it.
Manual driving also gives real input -> yaw-rate response to calibrate the
simulator's vehicle model (`AD_YAW_TAU`, `AD_STEER_DELAY`).

### Configuration

`t3taAutopilot.json` (all fields optional):

| key             | default | meaning                                   |
|-----------------|---------|-------------------------------------------|
| maxCruiseSpeed  | 14      | cruise speed cap when driving, m/s (the gyro flies at its game speed cap) |
| cellSize        | 4       | nav grid cell edge, meters                |
| roadCost        | 1       | A* cost multiplier on road cells          |
| flatOffRoadCost | 1.6     | A* cost on flat open ground (1.3 = cut across more, 12 = stay on roads) |
| offRoadCost     | 12      | A* cost off road when the world has no terrain data |
| cornerAccel     | 3       | lateral accel allowed in bends, m/s² (higher = faster, riskier) |
| toggleKey       | "G"     | engage/disengage key (Unity KeyCode name) |
| groundVehicles  | false   | also drive cars / bikes along the roads (experimental); off: gyrocopter only |
| debugProbes     | false   | log avoidance/status/stuck details        |
| telemetry       | false   | record every drive to `<mod>/telemetry`   |
| telemetryMaxMB  | 500     | oldest rides deleted beyond this (checked every 5 min; the latest ride is kept, and a single ride stops recording at it) |

## Build & deploy

```powershell
dotnet build -c Release src/t3taAutopilot.csproj
powershell tools/deploy.ps1    # copies to %APPDATA%\7DaysToDie\Mods\t3taAutopilot
powershell tools/package.ps1   # dist/t3taAutopilot-<version>.zip for a release
```

The repository root is the mod folder: `ModInfo.xml` + `t3taAutopilot.dll` +
`Config/` are what the game needs.

Set `-p:GameDir="..."` to override the Steam install path used for references.

## Notes / limitations

- Requires **EAC off** (any .dll mod does). Client-side only — works on servers
  because the local client simulates the driven vehicle.
- Road routing needs the generated-world splatmap, which is only available on
  the host. On a remote server it falls back to straight-line pursuit.
- If the destination marker is inside a town block or a field, the last
  stretch leaves the road; obstacle avoidance still applies there.
- The grid doesn't know about terrain slope or water — a cheap road path may
  still cross an unbridged river if the splatmap doesn't show it.
- Gyrocopter: flight has no obstacle avoidance beyond the cruise height and
  the downward rays, and a landing strip is chosen from the heightmap and
  splatmap only — a tree or wreck on it isn't known. Without the world files
  (remote server) it cruises 45 m above the measured ground and lands at the
  destination.
- Gyrocopter: the circle flown while a destination is picked in the air isn't
  checked for obstacles. It holds its height (at least 30 m over the ground
  right below) on a ~45 m radius, so open the menu with room around, clear of
  tall buildings and cliffs.
- If the engine stalls (no fuel / broken), the autopilot holds the brake — start
  the engine manually.

## License

MIT - see `LICENSE`.
