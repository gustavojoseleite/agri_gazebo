# agri_gazebo

The agricultural simulation platform: citrus-farm worlds, every vehicle's Gazebo half
(plugins, sensors, bridge) and the launch files that put any of them into any world. Shared by
every project in the workspace; nothing here knows about a particular project.

```
agri_gz_worlds/               worlds shared by every vehicle
├── worlds/
│   ├── empty.sdf             ground, sun, the systems: the smoke test
│   └── citrus_farm{,_90,_60,_30}.sdf      one property, one road network each
├── models/                   the props those fields <include>
├── scripts/                  the generators: the worlds, and the citrus meshes
└── docs/citrus/              the photographs the farms were built from
agri_gz_bringup/              the launches every vehicle shares (one vehicle, a convoy),
│                             the vehicles Python module behind them, the gz sensor macros
autonomous_tractor_gz/        the tractor in Gazebo: gz tags, plugins, bridge, spawn height, launch
pickup_gz/                    the same, for the pickup
```

Driving a vehicle is not here: keyboard teleoperation is in the `vehicle_control` repository,
and the camera road follower is in the `ftl` repository, which uses it to drive its leader.


`agri_gz_bringup` is what keeps a second vehicle from being a copy of the first. Its
`sim.launch.py` brings up any of them — it finds `urdf/<vehicle>.gz.xacro` and
`config/bridge.yaml` inside `<vehicle>_gz` by convention — and its
`urdf/sensors.gz.xacro` holds the Gazebo half of every sensor: rate, noise, field of view.
The description half (where the sensor sits) lives in the `robot_description` repository,
and the two are joined by link name.

Targets **Gazebo Harmonic**, the version paired with ROS 2 Jazzy. The commands are `gz sim`
(the older `ign gazebo` belongs to Fortress, and `gzserver`/`gzclient` to Gazebo Classic, which
reached end of life in January 2025).

## Running

```bash
ros2 launch autonomous_tractor_gz sim.launch.py                # world + tractor + bridge, GUI
ros2 launch autonomous_tractor_gz sim.launch.py gui:=false     # headless, for CI
ros2 launch autonomous_tractor_gz sim.launch.py world:=empty.sdf
ros2 launch autonomous_tractor_gz sim.launch.py world:=citrus_farm.sdf

# The same, through the generic launch, plus where to put the machine:
ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor \
    world:=citrus_farm_60.sdf x:=-40 y:=6 yaw:=1.5708
```

`world:` is a file under `agri_gz_worlds/worlds/`. Bare `gz sim citrus_farm_90.sdf` also works,
from any directory: the environment hook puts that directory on `GZ_SIM_RESOURCE_PATH`.

Drive it from a second shell. Join the *running* container rather than starting another one, so
both shells share the same ROS graph:

```bash
docker exec -it ros2-ws-dev bash
```

Then drive it, with the keyboard node from the `vehicle_control` repository (see its README
for why it steers by curvature and not by yaw rate):

```bash
ros2 run vehicle_control teleop_key      # keyboard: throttle latches, wheel self-centres
```

Or command it directly, which is what a script would do:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.3}}" -r 20
ros2 topic echo /odom
ros2 control list_controllers           # both should be active a few seconds after the spawn
```

`angular.z` is a **yaw rate**, not a steering angle: the steering controller works out what
the front wheels have to do to produce it, and the two of them end up at different angles — that is
what Ackermann means. Ask for more than the geometry can give and it saturates at the
steering limit instead of refusing.

### How the vehicles are driven

By **ros2_control**, the default: each vehicle's `<ros2_control>` block (from
`vehicle_common_description`) gets the Gazebo hardware plugin, `gz_ros2_control` runs a
controller manager for it inside the simulator, in the vehicle's namespace, and
`vehicles.py` spawns `joint_state_broadcaster` and `ackermann_steering_controller` and starts
`cmd_vel_limiter` in front of it. The controllers, their parameters and the limits are
`vehicle_control`'s, the same as on the mock and on the machine; nothing about driving is
configured here. Topics are unchanged: `cmd_vel` in, `odom`, `tf` and `joint_states` out.

The controllers are active a few seconds after a vehicle spawns; until then it does not move,
and a `cmd_vel` sent earlier is simply dropped by the limiter after 0.5 s. In a convoy each
vehicle has its own manager, `/<namespace>/controller_manager`, and every one blocks the
simulator for a moment while it loads, so a five-vehicle start-up takes the better part of a
minute. Run Gazebo with a GPU: without one `citrus_farm` runs at about a tenth of real time.

`control:=plugin` (on `sim.launch.py`, `convoy.launch.py` and FTL's launches) brings back the
old path, the Gazebo Ackermann plugin with the drive section of `bridge.yaml`. It stays while
ros2_control is being proven and then goes; the comparison is in `vehicle_control`'s README.

### A convoy

```bash
ros2 launch agri_gz_bringup convoy.launch.py                      # pickup + 4 tractors
ros2 launch agri_gz_bringup convoy.launch.py spacing:=15.0 followers:=2
ros2 launch agri_gz_bringup convoy.launch.py leader:=autonomous_tractor follower:=pickup
```

By default a **pickup** in front and **four autonomous tractors** behind it, nose to tail
down the main carreador at ten metres, the leader on the world origin facing
east. `leader:=` and `follower:=` take any vehicle type that has a `<vehicle>_gz` package;
the followers are named `<follower_prefix>_<n>` (`tractor_1`… by default). The convoy only
lines machines up: what they do next belongs to the project that launched it. Ten metres between `base_link`s is 6.4 m of daylight between one 3.6 m
tractor and the next.

Two things make five vehicles in one world work, and both are worth knowing
before spawning a sixth:

- **Each machine is its own ROS namespace.** Its controller manager, its
  controllers and its sensors' topics live in it; without it all five would
  arrive on `/cmd_vel` and `/odom` and fight. Driving the third one is
  `ros2 run vehicle_control teleop_key --ros-args -r __ns:=/tractor_3`.
  Each one also keeps its **TF tree in its namespace** (`/tractor_3/tf`,
  `/tractor_3/tf_static`): every vehicle uses the same frame names, and on one
  global `/tf` they would overwrite each other. A node that reads a vehicle's TF
  remaps `/tf` and `/tf_static` to the relative names, as `ftl_bringup` does.
- **Only the leader carries a camera.** A camera is the most expensive thing in
  any of these worlds — 16.8 of the 62.1 wall-seconds it took to simulate 20 s
  of `citrus_farm_60`, for one — so the followers are spawned with
  `camera:=false`. They keep GNSS and IMU, which are cheap and are what a
  follower localises with; the camera half of their bridge config is dropped
  with the camera, so nothing subscribes to a topic that will never carry a
  message. `followers_camera:=true` gives them cameras too, at the price you
  would expect; `followers_sensors:=false` leaves them with no sensors at all.

What one vehicle's bringup consists of lives in the `agri_gz_bringup.vehicles` Python module
(`agri_gz_bringup/agri_gz_bringup/vehicles.py`, installed with `ament_python_install_package`)
and is used by `sim.launch.py`, `convoy.launch.py` and any project that composes its own
scenario, because a vehicle is three nodes and a rule about topic names and having that written
twice is having two definitions that will disagree:

```python
from agri_gz_bringup.vehicles import vehicle_nodes
nodes = vehicle_nodes("pickup", "pickup", "/pickup", (0.0, 0.0, None, 0.0))   # z None: spawn.yaml
```

Each vehicle is dropped from its own height, declared in `<vehicle>_gz/config/spawn.yaml`
(`z: 0.4` for the tractor on 0.75 m wheels, `0.3` for the pickup), so no launch has to know
how big a machine is.

### Driving itself, by the edges of the road

`road_follow`, which drives a vehicle down the carreadores by its camera, lives in the `ftl`
repository now: FTL uses it to drive the leader. Its README carries the whole story of how it
finds the road. It runs against any vehicle launched from here:

```bash
ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor world:=citrus_farm.sdf
ros2 run road_follow road_follow                     # second shell
```

What a vehicle publishes: `/odom`, `/joint_states` and `/tf` from its controllers, and across
the bridge `/clock`, `/imu/data`, `/gnss/fix`, `/camera_front/image_raw` and
`/camera_front/camera_info`. No lidar — see below.

The vehicle starts **at the centre of the world**, on the carreador that runs through the origin
and pointing down it, in any of these worlds and with no per-world pose: see the conventions
below. `/gnss/fix` is reported at `gnss_link`, which sits over the **centre of the machine** at
2.685 m, and `/camera_front/image_raw` comes from the **front face** of it at 1.460 m — both
placed and explained in `robot_description`, which owns where a sensor is. Averaging 285 fixes
with the tractor parked on the origin puts the antenna at 1.122 ± 0.058 m east and
0.018 ± 0.058 m north of it, against the 1.075 m and 0 the description asks for.

## Agricultural worlds

`agri_gz_worlds/worlds/` holds one property in five cuts, and the empty world to fall back on. The
field is laid out the way a real one is: talhões of planted rows, carreadores wide enough to turn
a machine in, a headland at the ends of the rows, and a boundary that has to be respected.

The four farms share their ground, their planting, their mata and their yard. The **only**
difference between them is the bends their roads turn, which is what makes them worth comparing:
ask what a 30 costs a robot that a 90 does not, and the world is the only variable. Three of them
take one angle throughout; the fourth mixes all three, for when the field should have the variety
a real property has rather than the one variable an experiment wants.

| World | Field | What it is for |
|-------|-------|----------------|
| `citrus_farm_90.sdf` | 410 × 410 m, 5421 trees, 10 bends | **Roads that cross the property**, not rings inside it: four carreadores run edge to edge and cross each other, and the talhões are whatever the crossings leave over. Every bend is 90°, boundary included. The tractor and its bin trailer are parked in the yard beside the shed, 20 m from the origin. All four tree models are planted in it, as in its two siblings. |
| `citrus_farm_60.sdf` | the same property, 4835 trees, three carreadores | Each road takes one turn out and one back. A boundary that closes on itself has turned 360°, so at 60° it is a hexagon. This is also the world carrying the one talhão that **bears fruit** — six rows, 131 trees, the first 11 m south of the origin. |
| `citrus_farm_30.sdf` | the same property, 4538 trees, two carreadores | A 30 barely changes where a road is pointing, so a corner takes three bends in a row and two roads is all the property will hold: any more and they run alongside each other instead of crossing. The angle showing through the layout rather than being decoration on it. |
| `citrus_farm.sdf` | a **560 m square**, one crossroads, **four talhões** of 266 m, 13 600 trees, **plus four oblique roads**: 60° in each upper talhão, 30° in each lower one | The property this workspace is really about. Four boundary corners of 90°, all bladed round at 24 m and all painted; two carreadores running edge to edge and crossing at the origin at a right angle, cut square, with no number on either — a crossroads is not a turn. The planting is the four quadrants those roads leave and nothing else: **every talhão is bounded by carreadores on all four sides**, rows east-west in two of them and north-south in the other two, and **one crown to a block** — two smooth and two lumpy, one of each pair carrying fruit. The mata is a **belt right round the outside**. Each upper oblique road is **60° at all three of its points**: it leaves the boundary road at 60° rather than square to it, turns 60° in the **middle of its block**, and meets the main carreador at 60°. Each lower one is **30° at both its junctions**, off the main and onto the side boundary, with the bend 72 m from the middle of its block: at 30° no rectangle can have both, and the section below shows why. Each pair mirrors itself, east and west, and all four are painted with their angle. |
| `empty.sdf` | ground, sun, the systems | The smoke test. If a vehicle misbehaves here, the world is not the reason. |

**What is loose in the headland: standing timber and rocks.** There were cupinzeiros here — forty
1 m mounds of brown ellipsoids, scattered beside the carreadores — and they came out: at that size
nobody looking at the world could tell what they were meant to be, and a brown lump beside a road
reads as something dropped rather than something grown. They are `forest_tree` now, standing where
the mounds stood, 7.5 m from the carreador's centreline, which puts the trunk 1.5 m outside the
bare earth and leaves the canopy leaning a metre over the shoulder, the way a tree beside a farm
road does. What it must not lean over is the 7 m of running surface, and at 2.6 m of canopy from
7.5 m out it clears that by 1.4 m.

Trees and rocks are placed **alternately** rather than one kind and then the other, which matters
in `citrus_farm`: a property planted right up to its carreadores has room for twenty-two loose
things where the worlds whose blocks leave slack take all fifty-eight, and whichever kind ran
first took the lot. Taking turns, `citrus_farm` ends with 14 trees and 8 rocks and the other three
with the full 40 and 18.

### Four crowns, and the rows are baked

A property is planted with **four tree models**, and which talhão gets which is one table,
`PLANTING`, near the top of the generator. The same table in all three worlds, so the crowns are
never what is different between them:

| Talhão | Crown | Why |
|--------|-------|-----|
| eleven of fourteen | `citrus_tree` | The polygon crown, baked eight to a run. The default, and the only one that is nearly free. |
| **e** (north west), **l** (south of the yard) | `citrus_round` | One ellipsoid, same envelope. The control block. |
| **n** (at the crossroads) | `citrus_tree_fruit` | 131 trees carrying oranges, 11 m south of the origin. |
| the four matas | `forest_tree` | Two nested ellipsoids on a trunk, scattered, not rowed. |

They are spread rather than adjacent on purpose: no route across the property misses them, and
none of them can be confused with a change in the ground. On the plan in each world's header they
are `:`, `o`, `*` and `#`.

These worlds used to draw a citrus as **one ellipsoid floating at 1.75 m with nothing under it**.
The defence was cost: a trunk would double the visual count at 2275 trees. That defence was about
a trunk as a *second shape*. The mesh crown carries **its trunk inside the same mesh**, so the
trunk is free — Gazebo charges per **entity**, not per triangle — and `citrus_round` is the old
ellipsoid brought back deliberately, skirted to the ground rather than hovering over nothing. The
shape was never the problem; the floating was.

What is *not* free is the entity. The server walks every `<visual>` on every step whether or not
it moves and whether or not anything is rendering, and `citrus_farm_60` carried **7255** of them,
4966 being trees. Stripping the visuals out entirely took it from 312 physics steps a second to
1428 — that is where the real-time factor was going.

So the trees are **baked into runs**: eight to a mesh, then four, then two, from a library in
`models/citrus_tree/meshes/`, leaving at most one tree of any row planted on its own. Measured
back to back on the same machine, `citrus_farm_60`:

| | visuals | steps/s | RTF at the world's 3 ms |
|---|---|---|---|
| one visual per tree | 7255 | 142, 147 | 0.43, 0.44 |
| baked runs | 3545 | 277, 285 | 0.83, 0.86 |

**1.95× on both repeats**, with the geometry unchanged — the same crowns, in the same places, with the same
per-tree spin, scale and jitter, baked in instead of applied per visual. (Both runs shared the
machine with another simulation, which is why the absolute numbers are low; the ratio is the
point. Alone, the old world measured 312 steps/s.)

Whole rows would be better still — 339 visuals instead of 620 — and are impossible: an OBJ of one
crown is 87 KB, so unique rows would put **430 MB** of geometry in one world and 1.3 GB across the
three. Six runs of each length is 10 MB for all of them, and 48 distinct trees is enough that a
row of forty does not visibly repeat. Eight crowns is also 24 m of hedge, the length a stock
section of row was always cut to.

Three talhões are planted tree by tree anyway, and that is the point of them. The block
immediately south of the crossroads — six rows, 131 trees, the first 11 m from the origin —
carries **fruit**, and the fruit mesh belongs to one crown, so there is no baked run wearing any.
The two round blocks give up the runs for the opposite reason: a primitive cannot be baked into a
mesh at all. `PLANTING` is what puts each of them where it is.

**What the sensors cost, measured.** With the tractor spawned in `citrus_farm_60`, timing a fixed
20 s of *simulated* time (repeatable to 2%; sampling `/stats` instead is not — it ranged 0.19 to
0.78 for one configuration):

| | wall clock for 20 s of sim | RTF |
|---|---|---|
| lidar 360° at 10 Hz + camera, plants colliding | 62.1 s | 0.32 |
| without the lidar | 32.8 s | 0.61 |
| without the lidar, plants not colliding | 27.0 s | **0.74** |
| also without the roads (not done — see below) | 25.0 s | 0.80 |

The parts are additive: physics and everything that is not a sensor, 14.9 s; the lidar, 30.4 s;
the camera, 16.8 s. The lidar was half the budget for one sensor, which is why the tractor no
longer carries one. Physics alone would run at RTF 1.34.

Two things that sound like fixes and are not. **Raising the physics step**: 3 ms → 6 ms buys 5%
and 10 ms buys the same 5%, because physics is a quarter of the cost and the sensors are clocked
in simulated time. **Lowering sensor resolution**: 1280×720 → 640×480 and 900 → 450 lidar samples
changed nothing measurable. The cost is one scene traversal per frame, and it scales with the
number of *visuals*, not with pixels: 3563 visuals → 62.1 s, 2925 → 49.8 s, 2128 → 35.4 s.

**What was on the table, and has been taken.** The road surface used to be cut into tiles by the
folds in the ground, with no collisions at all: 1435 visuals in `citrus_farm_60`, 40% of that
world, against 1097 in the 30 and 740 in the 90. Deleting them outright took the configuration
above from 27.0 s to 25.0 s of wall clock, and before the lidar came off, 62.1 s → 35.4 s. They
are now one mesh per world instead, which is that saving without the deletion. (An earlier note
here said the measurement sat inside the noise. It did — because it was taken with no sensor
rendering, which is exactly when visuals look free.) Raising the physics step from
3 ms would buy the rest almost linearly: measured on the small rover these worlds were first
tested with, 1 ms to 8 ms on flat ground made no difference at all (no vertical movement), so
the room is there if it is ever needed — a 4 t machine on soft tyres is worth re-measuring
before relying on it.

Four conventions hold in all four, and are worth keeping in a fifth:

- **+X along the rows, +Y across them.** Everything reads more easily once that is fixed.
- **The origin is on a carreador, on flat ground, at z = 0.** The profile is level between
  x = ±25 m in every one of these worlds, so the middle of the property is a levelled yard
  whatever the relief does further out. That is what lets `sim.launch.py` spawn a vehicle in any
  of them **at the centre of the simulation, with no per-world pose**: the defaults are
  `x:=0 y:=0 yaw:=0`, and the drop height is the vehicle's own (`config/spawn.yaml`), not a property of the world.
  Measured, with the tractor spawned in each of the four: it settles at
  (0.0000, 0.0000, 0.0000) with roll, pitch and yaw all 0.00°.
- **Ground markings have no collision.** The worked soil of a talhão and the surface of a
  carreador are painted on the ground, not built up on it: a 2 cm lip is nothing to a tractor and
  a wall to a small robot on 4 cm casters.
- **Every plant is static.** A field of dynamic bodies is a field the physics engine solves
  contacts for at every step, and a barrier that gets nudged aside on the first bump.

**No plant collides, anywhere.** Not in the worlds, where the generator has stripped them since
the `--collisions ground` default went in, and not in the four tree models either, which now
carry no `<collision>` at all — so a world that `<include>`s one gets the same deal. A camera and
a GPU lidar both work off *visuals*, so foliage is still exactly what a sensor sees; the solver
was being charged for trunk cylinders nothing in a perception run ever touches. Twelve collisions
survive in each world and they are all ground or building: the eleven terrain slabs and the shed.
Driving through a tree is the trade; falling through the ground is not.
`generate_agriculture_worlds.py --collisions full` is still there and puts the world's plant
collisions back — the model files have none to put back.

### The model library

`agri_gz_worlds/models/` is on `GZ_SIM_RESOURCE_PATH`, so any world in the workspace can
`<include>` these by `model://<name>`. Eight of them: four trees, three machines and a tank.

None of them collides. Sizes are measured on the geometry, not on the radii it was built from.

| Tree | Size | Visuals | Notes |
|------|------|---------|-------|
| `citrus_tree` | 3.95 × 2.83 m | 1 | Polygon crown, trunk inside the same mesh. Six variants in `meshes/`, plus the runs of 2, 4 and 8 the rows are actually laid from — which is what makes one visual do for eight trees. **What most of every talhão is planted with.** |
| `citrus_tree_fruit` | 3.95 × 2.83 m | 2 | The same crown with a second mesh of oranges in it. No baked run wears fruit, so a fruiting row is planted one tree at a time and then pays for the fruit on top: sixteen times the entities of the same row in runs. One talhão carries it. |
| `citrus_round` | 3.12 × 2.76 m | 2 | Trunk and one ellipsoid — the sphere the mesh is pushed out of, averaged round in plan, on 0.44 m of visible trunk. The **control**: what a perception stack does differently between a round talhão and a mesh one came from the foliage and not from the field. |
| `forest_tree` | 5.16 × 6.71 m | 3 | Two ellipsoids, one inside the other, on a trunk — canopy from 2.59 m up, so a boom passes under it. What the **mata** is made of, scattered rather than rowed. |
| `carreador` | — | 0 | Not a model anyone places: the baked roads. One OBJ per world holding every carreador of it as a named strip — bare earth and running surface — which each world picks out by name with `<submesh>`. Written with the worlds, never by hand. |

| Machine | Notes |
|---------|-------|
| `bin_trailer` | Four wheels, three bins. Behind a tractor it makes 9 m of vehicle, which is what actually decides how wide a road and how open a bend has to be. Parked in the yard, hitch to the tractor's drawbar. |
| `tractor` | 3.6 × 2.15 × 2.6 m. The ruler every carreador was laid out against — and the same object the workspace drives: `autonomous_tractor_description` copies its bodywork panel for panel in `params.xacro`, down to the blue. The two of them in one world are one machine in two states, the parked one by the shed and the driven one on the carreador. |
| `harvest_bin` | Field bin left in an alley: on no map, and gone next week. |
| `water_tank` | The one tall, isolated landmark in a field where every row looks alike. |

Only four of the nine are `<include>`d by a world — the machines and the tank. A model is also a
**definition**, and that is the other half of what this directory is for: the worlds reference
`citrus_tree`'s meshes by URI, and reproduce `citrus_round`'s ellipsoid and `forest_tree`'s pair
of them as inline geometry. The numbers live in one place and the model file is where they are
explained.

Why inline at all: measured on 300 identical static objects, **300 models cost 0.8 ms a step, the
same 300 as links inside one model 0.5 ms, as plain shapes inside a single link 0.3 ms**. Gazebo
charges per entity and a model is several of them, so where the count runs to hundreds — every
tree, every loose tree, every rock — the geometry goes inline, into one model per quadrant. Where
the count is five, `model://` stays: it is the better way to write a world and the difference does
not show up in a step.

### Roads that curve, and rows that are cut to them

All three worlds are built the same way, and it is worth knowing before editing one. A carreador
is written the way it is driven:

```python
Path(0, 0, 0).straight(44).turn(-30, 26).straight(34).turn(-60, 20)
```

Positive angles turn left, and the second number is the centreline radius — the one that decides
whether a rig gets round the bend or has to shunt. The centreline becomes a spine — the two ends
of each straight, a station every 2.5 m of arc, and one station per sharp vertex carrying both of
its headings — and each layer of the road is a ribbon of triangles laid along it. A whole road is
one model and each of its layers one visual, however long it runs.

The rows are then **not** laid out and drawn. Each row line is walked at 0.5 m and kept only
where the ground is plantable: inside its own talhão, off every road corridor, out of the mata,
clear of the yard, and 4.5 m from the neighbouring block. The contiguous stretches that survive
become rows; a stretch too short for three trees is left bare. That is what makes the field end
square against a curve, and it means editing a road automatically re-cuts every block beside it.

Each world's header carries a plan of itself, rasterised by the generator at 5 m per character.
It is the fastest way to see what a change did without opening the simulator.

Every bend gets its angle painted automatically, from `Path.marks()`: **in the middle of the arc
itself**, dark figures on a pale board, digits side by side, and **every number laid out north up**
whatever direction its road runs — consistent orientation is what makes a set of them scannable
from above. Seven boxes to a digit, because SDF has no text geometry and these worlds carry no
textures.

**Every board is the same size, always.** It used to shrink to 68% of itself when the search could
not find a spot clear of a fold in the ground, and that is a worse fault than the one it fixed: on
a property whose numbers come in two sizes, the small ones look as though they meant something
different — a lesser bend, a different kind of marking — when all they meant was that the ground
under them was creased. Size is not information here; the digits are. So the search moves the
board and never resizes it, and it has more places to move to: nine points along the arc instead
of three, then a few metres past either end. Across the four worlds it now finds a clean spot for
all 48 numbers without once falling back.

It used to sit 28 m back along the straight that led into the bend, the way a road sign does, and
that is the wrong model: a sign is a warning and needs the setback, a number on the ground is a
label. Set back, it has to be matched to its bend by eye, and on a property of mixed angles the
number before a bend and the number after it are both in view from one spot with nothing to say
which is which. On the arc there is no question. Of the 48 numbers across the four worlds, 34 sit
dead centre on their arc; the other 14 are shifted along it, or a few metres past its end, to
clear a junction or a fold in the ground that the board could not lie flat across.
`Path.close_to_y()` computes the last straight of each block loop, so changing a radius still
lands the road back on the carreador it left.

No road ends anywhere except on another road. The dividing carreadores run edge to edge and
finish on a perimeter that closes on itself, and `check_network()` enforces that at generation
time — it refuses to write a world whose roads break any of three rules:

| Rule | What it catches |
|------|-----------------|
| **inside the boundary** | A road whose centreline strays more than 6 m outside the perimeter. |
| **both ends on it** | A road that stops in the middle of a field, which is a road a machine has to reverse out of. |
| **not alongside** | Two roads within 32 m of each other and pointing the same way for more than 25 m. |

Every one of those was a real fault before it was a check. `citrus_farm_60`'s third carreador
used to start at the bottom point of the hexagon and set off due east, where the property is
**35 m wide**: 180 of its 270 m ran outside the boundary and came back in, which on the plan is a
road and a boundary crossing twice with a lens between them — a figure of eight — and in the world
is a road through nobody's field. It survived because `close_to_boundary()` only ever looks at
where a road *ends*, and nothing looked at where it had been. That road now runs from the
south-east edge to the north-west one and crosses both the others on the way.

Crossing another carreador is reported rather than enforced. `citrus_farm_90`'s `south` runs edge
to edge and meets nothing but the perimeter, which is thinner than the design wants but is not
wrong the way the three above are wrong, so generating it prints a note and carries on.

### Where a 90 belongs, and why a bend needs room

Two findings, both from looking at `citrus_farm` after it was first built, and both now
rules rather than opinions.

**A bend needs straight either side of it.** A turn of angle θ at radius *r* eats `r·tan(θ/2)`
off the straight at each end: 6.4 m for a 30 at r=24, 13.9 m for a 60, **24 m for a 90**. The
first cut of that world put a 60 and a 90 sixty metres apart on paper, which is
60 − 13.9 − 24 = **22 m** of real road between the two arcs — two road widths. They ran into each
other and the pair read as one long sweep, so the 90 looked like a road that had wandered rather
than a 90. `check_network(bend_spacing=45)` is that rule now, and `citrus_farm` leaves 50 m
between every pair of bends. The single-angle worlds deliberately do not take the rule: three 30s
back to back is how a road of 30s turns a corner there.

**A 90 belongs where a road changes what it is doing.** A 30 or a 60 nudges a road and it carries
on across the property; a 90 turns it onto the other axis. So a 90 reads right where a carreador
stops running one way and starts running another, and reads wrong dropped into the middle of a
crossing run. Radius is not the lever it looks like: the floor is **22 m** of centreline, set by a
tractor and a loaded bin_trailer needing 18.5 m on the inside of the bend, so a 90 cannot be made
much sharper than it is. Spacing and placement are the levers.

**Two roads crossing are two surfaces at the same height, and the stacking order is by layer,
not by road.** Every carreador used to be drawn at z = 0.013, so a crossing put two flat faces
over the same ground at the same height, and coplanar faces are what a renderer cannot choose
between: the junction flickers as the camera moves, which looks like a fault in the junction and
is a fault in the z. Staggering the roads fixed the flicker and introduced a worse fault, which
took a camera to find. Stacking each road as a unit — earth, surface, markings, next road a
millimetre higher — puts one carreador's **running surface over another carreador's markings**.
Seven metres of surface crossing at 30° covers **fourteen metres** of what it crosses, so the
marking down the middle of the main road simply stopped at every junction. The camera-driven
tractor followed what was left, which belonged to the joining road, and drove off down the 30°
road into the talhão — twice, before the world was suspected rather than the follower.

So the stacking runs by **layer and not by road**. `ROAD_LAYERS` is two bands — earth 10 mm and
running surface 14, with the painted numbers at 18 — and a road's place is a half-millimetre step
**inside** its band. Every surface is then above every earth strip whoever owns either, so a
crossing shows both roads graded rather than one road's shoulder laid across the other's track.
The generator asserts the bands still clear the stagger, which with seven roads is 3 mm of stagger
against 4 mm of band and would not survive nine.

**A sharp corner is a mitre, not a patch.** Two road strips meeting at a vertex leave the outer
quadrant uncovered — 6 m of bite out of a 12 m road at 90° — and that used to be covered by laying
a square of the road's own width over it, square to the road rather than to the bisector (on the
bisector it fills the notch too, and sticks out past both edges by `width/√2`: **8.5 m of road
pointing off into the grass like an arrowhead**). The patch is gone. A vertex is now a station
whose two headings differ, and the edge of the ribbon steps out along the **mitre** — the point
where the two offset lines cross, which is the normal stretched by `1/cos(half the turn)`, 1.41 at
a right angle. That is the same square corner the patch was drawing, from the same two lines of
arithmetic as every other station, with nothing to lay over anything and no separate piece to cut
around the folds.

**The yard is chosen before the planting.** The clearing that keeps rows out of it is derived from
where it lands, so the two cannot come apart. They did once: the yard searched for clear ground
after the planting was already laid, moved, and left its clearing behind — a shed with citrus **4 m
from its wall**. Now it is 31 m, and `check_props()` keeps every piece of the yard, the machinery
and the bins **15 m** clear of any carreador, because the roads are the one part of these worlds
the vehicle is entitled to.

**Right angles outside, the other angles inside.** A real property is squared off at its edge,
because that is where a surveyor drew it, and the roads that cut its blocks up run at whatever
angle gets them across. `citrus_farm` is laid out that way: a **360 × 420 m rectangle** whose
four corners turn 90 — two cut square, two bladed round at 24 m — two frame carreadores crossing it
edge to edge and meeting each other square in the middle, and two **diagonals** cutting the blocks
those leave, carrying the 30s and the 60s. A crossroads is not something you turn at, so the frame
roads carry no painted number at all; every bend that does turn a road carries one.

**A farm turns both ways.** A graded road is bladed round where there is room and simply cut where
two blocks meet square, and the aerial photographs are full of both — mostly the second. So
`Path.turn(degrees, radius)` takes **radius 0**: no arc, a vertex. It costs no straight either,
where an arc eats `r·tan(θ/2)` at each end, which is how a sharp 90 fits where a rounded one would
not. Two strips of road meeting at a vertex leave a triangular notch on the outside of the turn,
which the mitre at that station fills exactly, and which is the piece of ground a grader squares
off anyway. `check_network(pairs=True)` then insists that **every angle appears both ways**: the
world carries a sharp 30, 60 and 90 and a rounded 30, 60 and 90, and will not build without
all six.

**The carreadores belong to the vehicle.** Nothing a machine would have to drive round may stand
within **15 m** of a road centreline: the 6 m of bare earth the road wears, plus room to pass
whatever is parked without putting a wheel in the crop. `check_props()` enforces it, and the yard
— shed, tank, bins, the parked tractor and its trailer — is placed as one block whose anchor is
searched for until every piece of it clears. It caught the world it was written for: the water
tank had been standing **1 m** off a centreline and the field bins 4.8 m, which is a tank and two
bins parked in a road a driverless tractor is meant to have to itself.

**And the vehicle starts on it, pointing along it.** `sim.launch.py` spawns at 0 0 with yaw 0 and
no per-world pose, so the carreador that carries the origin has to run along **+x**: a machine
that starts 20° across its own road spends its first move correcting. `check_network` measures the
heading of the road under the origin and refuses anything more than 3° off.

**The number goes on the arc, and off the junction.** Twenty-two metres of clearance to any other
road: the board's own 5 m, plus the 11 m of bare earth the other road is wearing, plus enough to
see which road the number belongs to. The search takes the middle of the arc, then either side of
it along the arc, and only then up to 12 m past its ends — never far enough to look like the
number of the straight. Across the four worlds the tightest number now clears **20.2 m**, against
**5.1 m** before, which was a 90 lying across a crossroads.

### One property, four road networks

The four come out of one builder, `_build_farm()`, which takes the block roads as a spec and
nothing else. The terrain, the talhões, the mata, the yard and the way rows are cut are shared
code, so the only difference between the four worlds is the geometry of their carreadores —
which is what makes them worth comparing.

Three of them take their angle from `through_network()`, which implies it on every bend; the
crossroads world takes `mixed_network()`, which is handed each turn in degrees,
because a mixture is the whole point of them.

All four are laid out as **through routes**: each road runs from one edge of the
property to another and they cross each other on the way, so the blocks are what the crossings
leave over. An earlier cut ringed each quarter of the farm with a loop, which produced a rectangle
inside a rectangle inside a rectangle where every route went round something and none of them went
anywhere.

How many roads a property will hold turns out to be a function of the angle. At 90° four fit; at
60°, three; at 30°, two, because a 30 barely changes where a road is pointing, so any third road
ends up running alongside one of the others rather than crossing it.

Road layouts are declarative — `dogleg([(x, y), ...], radius)` takes the corners and works out the
straights so each arc is tangent to both — so changing a network is moving a corner, not redoing
trigonometry.

Two rules keep the networks readable, the first of them enforced by `check_network()` above:

- **No two carreadores run alongside each other.** Crossing is a junction and junctions are the
  point; a loop passing 5 m outside the perimeter is one wide mess with a strip of hedge stranded
  down the middle. Every loop is sized so nothing parallel comes within **32 m**.
- **A bend's angle is painted in the bend.** The arc is the widest, flattest piece of road there
  is — 11 m of bare earth through the curve, against a 7.8 by 6.0 m board — and it is the one
  place a label cannot be read as belonging to the next bend along.

Two constraints do the design work in the single-angle worlds. A loop that leaves the east-west
carreador heading north and rejoins it heading south has turned **180°**, so the angle fixes how
many bends it has: two 90s, three 60s, six 30s, each loop being half a regular polygon. A boundary
that closes on itself has turned **360°**, so the perimeter is a polygon of `360/angle` sides.
Nothing is planted outside it.

### Sixty degrees can sit in the middle of a block; thirty cannot

`citrus_farm` adds four oblique roads to its crossroads, and they are the same idea
run at two angles: leave a carreador at the angle, turn the angle in open ground, arrive on
another carreador at the angle. Three junctions of three different kinds — a T off the boundary, a
bend in the open, a T onto the main — all taking the same turn of the wheel.

**At 60° the arithmetic closes.** Sixty degrees off a horizontal road puts the first leg at −60;
turning 60 puts the second at −120; and both −60 and −120 are at 60° to the horizontal, so the
first leg can finish on a horizontal road and the second can too. Nothing is over-determined, and
the bend is free to sit wherever it is wanted — here, **3 m from the centre of its 266 m block**,
which is the other half of the point: a machine entering from the north meets the curve with the
whole block around it rather than against a fence, and the rows on both sides are cut to it.

**At 30° it does not, and no width of block will make it.** A leg at 30° to the horizontal meets a
*vertical* road at 60°; a leg at 60° meets a *horizontal* road at 60°. The two legs of a 30° bend
are −30 and −60, so one of them wants to finish on a horizontal carreador and the other on a
vertical one — one asks for a tall block, the other for a wide one, and no rectangle is both. That
is why widening the lower talhões does not fix it: the constraint is on the *ratio* the two legs
demand at once, and a rectangle has one ratio.

So the lower roads keep the angle and give up the position: **30° off the main, 30° onto the side
boundary, bend 72 m from the middle** of the block — still inside its central third. The other
branch of the trade is a bend dead centre with one junction at 60°, which gives away the angle the
world exists to test. Either is one line of spec; the choice is which of the two properties
matters more.

### Ground that is not flat

These worlds carry relief, and how is worth knowing, because the obvious ways do not work.
**Gazebo Harmonic's DART collides with neither a heightmap nor a trimesh in this build** — both
were tried and measured, and a sphere dropped on either falls straight through (to z = -1686 and
z = -1093 respectively, the mesh loading without an error first). Boxes do collide, so the ground
is built from tilted slabs, and since a box top is a plane the surface is piecewise planar. Each
slab may tilt about **one axis only**: tilt about two and the tops project to parallelograms that
no longer meet at the corners, which at these sizes is 10–20 cm of gap. Hence relief along X and
level along Y. The eleven slabs meet within **1.3 mm**.

**The ground reaches past the fence, and `citrus_farm` reaches furthest.** `extent` is how far the
terrain goes from the origin, and the profile's end grades are carried on rather than a new
landform invented. The default 260 m suits the 410 m properties; `citrus_farm` asks for **460**
against a property that reaches 332, because an aerial view showed why 340 was not enough — ground
that stops 8 m past the last tree is a cliff at the edge of the world, and the whole farm appeared
to float on nothing. It costs no entities: the slabs are the same eleven boxes, longer, and the
world still runs 20 s of simulation in 22.7 s.

Everything is placed on that surface: rows are broken at the folds so each hedge lies in one plane,
and anything resting on the ground takes the local slope with it.

Roads need more care than that, and getting it wrong is easy. The folds run north to south, so a
carreador that runs north to south has a fold **down the middle of it** rather than across it, and
one drawn as a flat plank ends up with one edge buried and the other in the air — 55 cm at the
edges of 11 m of bare earth where the grade goes from −7% to +3%, which reads as a road cut in
half lengthwise.

Boxes were the first answer and they are the wrong shape for it. A box is one plane, so the road
had to be cut around every crease: along its length where a fold crossed its centreline, across
its width where one ran beneath it, and then chopped every 3 m near the fold where the road met it
at an angle and the crease ran diagonally across the rectangle, which neither cut alone fixes.
That left **6 to 19 cm** of worst corner, against 55 to 79 before it — an approximation, with a
granularity to tune, and expensive: see the cost section below, where it turned out to be most of
what `citrus_farm` was paying.

**A road is now one mesh, and the fold is an edge of it.** A mesh is under no obligation to be
planar, so the crease does not have to be avoided — it can be part of the geometry, which is what
it is on the ground. `ribbon()` walks the road as a strip of quads, clips each one by every fold
that passes under it (`_one_plane_each()`, a convex polygon cut by a vertical line), and
triangulates the pieces. Every triangle that comes out therefore lies in exactly one plane of the
terrain, and every vertex sits on the ground at its own x. Measured over all four worlds by
sampling the interior of all 3046 triangles against the terrain: **worst deviation 0.000 mm**,
where the boxes left 19 cm. Exact, with nothing to tune, and one visual instead of several
hundred.

**A carreador is two strips and nothing else.** Twelve metres of bare earth, and 7 m of graded
running surface down the middle of it, a few counts lighter. No markings: two were tried and both
came out — a painted white line, because nobody paints a farm track, and then a pair of wheel
ruts, because two hard-edged stripes 2.1 m apart read as paint as well and matched the bitola of
no machine on the property. The lesson from the line survives them both and is in the z-order note
above: a mark is only a reference if the road that joins yours does not bury it.

The road the camera steers by is now the surface itself, whose two edges are symmetric about the
centreline by construction. That is not a consolation prize — it is a feature the road really has,
and it does not have to be maintained in two places.

### Four things sized for a big world

410 m of property, four to five thousand trees and sixteen metres of relief are paid for in five
places, all worth understanding before copying them:

- they step at **3 ms**, where `empty.sdf` steps at 1;
- their citrus is **one crown per tree with no separate trunk** — a grown tree is skirted to the
  ground anyway — and the rows are baked eight crowns to a mesh;
- their **hedgerows are collision boxes and crowns inside one link per block**, not one model a row;
- their mata and loose ground furniture are **inlined into one model per quadrant**, not 300
  includes;
- their **carreadores are baked meshes**, a strip per layer per road, not a tiled floor of boxes.

The last two are measured, on 300 identical static objects: **300 models cost 0.8 ms a step, the
same 300 as links in one model 0.5 ms, as plain shapes in a single link 0.3 ms.** Gazebo charges
per entity and a model is several of them. Together they are why five thousand trees and full
terrain relief arrive as thirty-odd entities and keep up with the clock:

| World | visuals | collisions | models + includes | 20 s of sim, headless | RTF |
|---|---|---|---|---|---|
| `citrus_farm.sdf` | 3778 | 14 | 32 + 8 | 22.7 s | 0.88 |
| `citrus_farm_90.sdf` | 2138 | 12 | 32 + 8 | 22.5 s | 0.89 |
| `citrus_farm_60.sdf` | 1915 | 12 | 32 + 8 | 22.4 s | 0.89 |
| `citrus_farm_30.sdf` | 1971 | 12 | 36 + 8 | 22.4 s | 0.89 |
| `empty.sdf` | 1 | 1 | 1 | 22.1 s | 0.90 |

Every world is now within 0.7 s of the empty one, which is the point of the two paragraphs below:
the roads used to be most of the difference.

**A static visual is not free, even with nothing rendering it.** This was assumed the other way
round for a while — the server has no camera, so why should it care — and `citrus_farm`
disproved it. The controlled version: take that world, keep all 39 models exactly where they are,
and collapse the eight oblique-road models from 1139 box tiles to one box each. Nothing else
changes, and in the same session it goes from **33.0 s to 24.6 s** of wall clock for 20 s of
simulation. Against the same world without those four roads in the clean
session, 3904 visuals cost 23.6 s and 5243 cost 28.8 s.

Solve that against the empty world, which runs 20 s of simulation in 22.1 s with one visual, and
the shape of it is simple: about **1.1 ms a step of fixed cost and 0.6 µs a step per static
visual**. At a 3 ms step, that put 3.2 ms of visual budget into the crossroads world against a
3 ms step — the whole of the difference between RTF 0.85 and RTF 0.69, from roads a robot cannot
even collide with.

**So the roads were baked into meshes, and the cost went away.** One OBJ per world in
`models/carreador/meshes/`, every strip of every carreador a named object inside it, each SDF
visual picking one out with `<submesh>`: 1486 road visuals in `citrus_farm` became **23**,
and the whole property's roads across all four worlds are 3046 triangles and 153 kB. Headless,
that world went from 28.8 s to **22.7 s**; interactively, from an RTF of 0.59–0.66 to **0.91**.
The same trade as the trees, and the same lesson twice: at 0.6 µs a visual, what a scene costs is
decided by how many entities it is cut into, not by how much geometry it contains. Thirteen
thousand trees one visual each would never have run either.

**The belt gives ground to the blocks, and the world stays the size it was.** The mata started as
four patches inside the property, became one patch outside it, then a belt right round the
outside, and has been trimmed twice since: 122 000 m² of canopy, then 60 000, now **24 000** — a
strip **18 m** across where it started at 40. None of that went into making the world bigger.
Nothing in it reaches past **332 m** from the origin, which is exactly where the belt reached when
the property was a 400 m square; what changed is the fence, which has moved out from 200 m to
**280** through ground the wood used to stand on. A belt is scenery nobody drives through and a
talhão is what the machine is there for, so when the two want the same metres the talhão takes
them — and what the wood is for, closing the horizon, 18 m of it does as well as 40.

**Bake the variety, do not ration it.** Crown variety used to be something a world could barely
afford. Measured on the crossroads world with its north-west quadrant planted plain and its
south-east one fruiting, both 9 hectares:

| | visuals, before | visuals, now |
|---|---|---|
| the fruiting talhão | 3194 | 244 |
| the plain talhão | 2986 | 213 |
| the two polygon-crown talhões | 922 | 918 |
| everything else — roads, mata, terrain | 896 | 669 |
| **whole world** | **7998**, RTF 0.45 | **2044**, RTF 1.00 |

The fruit and the plain crown were never expensive trees. They were trees that could not be
**baked eight to a mesh**, so each one cost a visual of its own and the fruit cost two, and the
bill scaled with the block. `generate_citrus_meshes.py` now writes **four libraries** instead of
one — two crowns, lumpy and smooth, each with and without fruit — every one in runs of eight,
four and two, and a talhão is planted from whichever it wants. A block of fruit costs what a
block of anything costs, so crown variety is now a free choice rather than a budget. The mesh
library went from 10 MB to 47 MB, which is the whole price: baked fruit is cut to 18 oranges a
tree against the single tree's 60, because a run is what a nine hectare block is planted from and
nobody counts oranges at that distance.

Server only — `gz sim -s -r --iterations`, no vehicle spawned and nothing rendering — so the
difference between the three is inside the noise, and the empty world's number is not the bargain
it looks: it buys its 0.90 over **20000** steps of 1 ms against the farms' 6667 of 3 ms, a third
of the cost a step. The interactive figure is the one that matters day to day, and it holds up:
`citrus_farm_60` with the GUI open, the tractor spawned and its camera publishing, on **Mesa
software rendering with no GPU**, sits at **RTF 0.69 to 0.80** over six samples. Measured the
same way at each step: 0.92 to 1.10 with every talhão in baked runs, 0.74 to 1.07 once two of
them were round and one fruiting, 0.69 to 0.80 once the round trees were given trunks. That
sequence, and not the headless one, is what a visual actually costs — it is charged when
something renders it, and the camera is what renders it.

If more is wanted back, the round trunks are the place to look: they are 580 of the 4658 visuals
in that world and they exist for looks, not for driving. Baking the round tree into runs of eight
the way the mesh tree is baked would take a round talhão from one visual a tree to one per eight,
at the cost of the round crown no longer being a primitive.

Parked machinery goes in the yard, and moving machinery on the carreadores; neither belongs in
an alley. An alley is 6.5 m of carreira less **3.95 m** of canopy, so a 2.15 m tractor clears it
by 0.2 m a side — it fits, just, and it also blocks the alley outright, which is the whole
argument. That figure used to be quoted as 3.5 m of canopy and 0.4 m a side, and both were wrong:
nobody had measured the mesh, which came out of its generator at **4.6 m**, leaving 1.9 m of
alley that the tractor did not fit down at all. Cutting the crowns back by a seventh is what
made the arithmetic true.

### Changing a field

The three worlds are generated, and the generated SDF is committed — the build never runs the
generator, and Gazebo only ever reads the SDF. Edit the layout constants at the top of the
matching builder in `agri_gz_worlds/scripts/generate_agriculture_worlds.py` (row spacing, lane width,
slope, number of rows) and regenerate. Two tables near the top of that file are the ones worth
knowing: `THROUGH_BLOCKS`, which is where the talhões are, and `PLANTING`, which is what each of
them is planted with — moving a block to `"round"` or `"fruit"` there is the whole of changing a
crown:

```bash
python3 agri_gz_worlds/scripts/generate_agriculture_worlds.py          # rewrite in place
python3 agri_gz_worlds/scripts/generate_agriculture_worlds.py --out /tmp   # look first
```

The trees themselves come out of a second generator, which writes the six crowns — trunk and all
— and their fruit into `models/citrus_tree/meshes/` as OBJ. Change the shape of a citrus — how many
lobes, how deep the skirt, how much fruit — there, and rerun it; the worlds do not have to be
regenerated afterwards, since they only ever name the mesh files.

```bash
python3 agri_gz_worlds/scripts/generate_citrus_meshes.py               # rewrite in place
python3 agri_gz_worlds/scripts/generate_citrus_meshes.py --out /tmp    # look first
```

Both generators need nothing but Python 3, and the OBJ carries its colours in a `.mtl` beside it
rather than in the world: one mesh holds a green crown and a brown trunk, and an SDF `<material>`
can only override a mesh with a single colour.

## Adding a vehicle

Copy `autonomous_tractor_gz/` to `<vehicle>_gz/` and point it at the matching
`<vehicle>_description`. Three things do not get copied:

- the worlds, which stay in `agri_gz_worlds/` — that is why it is a package of its own;
- the launch, which stays in `agri_gz_bringup/` — the copy keeps only the wrapper that names the
  vehicle, and the wrapper is a dozen lines;
- the sensor behaviour, which stays in `agri_gz_bringup/urdf/sensors.gz.xacro` — a second machine
  carrying the same lidar should not carry a second opinion about its noise.

Keep the three file names the generic launches look for: `urdf/<vehicle>.gz.xacro`,
`config/bridge.yaml` and `config/spawn.yaml` (its `z:` is the height the vehicle is dropped
from). Once it builds, the vehicle can be spawned alone (`sim.launch.py robot:=<vehicle>`) or
take either role in a convoy (`leader:=` / `follower:=`), in every project.

## Why the Gazebo tags are here and not in the description

`autonomous_tractor_gz/urdf/autonomous_tractor.gz.xacro` includes the pure description from
`autonomous_tractor_description` and adds the `<gazebo>` blocks — plugins, friction, sensor
noise. The dependency points one way: `autonomous_tractor_gz` → `autonomous_tractor_description`,
never back. A description package that depends on Gazebo can no longer be used with rviz alone,
with MoveIt, or on the real machine.

## Two traps this workspace has already fallen into

**A plugin's relative topic is not scoped by the model.** `<topic>cmd_vel</topic>` inside a
plugin publishes on `/cmd_vel`, at the root, not on `/model/<name>/cmd_vel` — so two machines in
one world both drive off the same topic and both publish the same odometry. Every topic in
`autonomous_tractor.gz.xacro` is therefore written out in full, starting with `/model/<name>/`,
and `bridge.yaml` is a template with `{name}` in it so it can name the same ones.

**A GNSS's horizontal noise is in degrees.** Gazebo adds it to the latitude and longitude as they
are, so `<stddev>1.0</stddev>` is not one metre of scatter but one degree of it — 111 km. The
`gz_navsat` macro takes metres and divides; the symptom, if it is ever bypassed, is a fix that
looks plausible and wanders across state lines. A world also needs `<spherical_coordinates>`, or
there is no datum to convert against: all of these worlds carry the same one, Bebedouro/SP.
