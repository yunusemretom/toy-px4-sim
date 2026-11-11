# toy-px4-sim

[Türkçe](README.md) | **English**

Ready-to-fly **Gazebo simulation** of fixed-wing UAVs commonly used in competitions. The same
aircraft and the same mission course work with both **PX4** and **ArduPilot**.

| Aircraft | Autopilot | World | Tools |
|---|---|---|---|
| TOY (the team's aircraft), X-UAV Talon, Mini Talon, Skywalker X8, Skywalker 1900 | PX4 SITL · ArduPilot (ArduPlane) SITL | Offline mission course | Up to 10 vehicles at once, pymavlink test scripts |

**Highlights**

- 🛩️ **5 aircraft** built from real dimensions: moving control surfaces, aerodynamics computed with AVL.
- 🔁 **Two autopilots**, one model source: PX4 and ArduPilot variants are generated from the same definition.
- 🗺️ **Mission course**: runway, race pylons, gates, ground targets for camera detection.
- 👥 **Multi-vehicle**: mixed fleets of different aircraft, in real time.
- ⚡ **NVIDIA support**: on hybrid laptops Gazebo is automatically routed to the NVIDIA GPU.

> PX4 and ArduPilot are not part of this repository. Each is installed from its own official
> repository; this repo links the aircraft, world and settings into them. A `git pull` here updates
> everything.

---

## Contents

1. [Quick start: PX4](#quick-start-px4)
2. [Quick start: ArduPilot](#quick-start-ardupilot)
3. [Usage](#usage)
4. [Aircraft](#aircraft)
5. [Mission course](#mission-course)
6. [Connections and test scripts](#connections-and-test-scripts)
7. [Performance](#performance)
8. [Development](#development)
9. [Troubleshooting](#troubleshooting)

---

## Quick start: PX4

Requires Ubuntu 22.04 / 24.04. Gazebo Harmonic comes with the PX4 setup.

```bash
# 1. PX4 (once)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh          # then log out and back in

# 2. This repo
git clone https://github.com/yunusemretom/toy-px4-sim.git
cd toy-px4-sim
./install.sh ~/PX4-Autopilot
pip install -r requirements.txt

# 3. Run
./run.sh talon
```

Gazebo opens with the aircraft waiting on the course runway. For a takeoff and maneuver test, in a
second terminal:

```bash
python3 scripts/maneuver_test.py
```

## Quick start: ArduPilot

No PX4 installation needed.

```bash
# 1. Dependencies (once)
sudo apt install rapidjson-dev libgz-sim8-dev libopencv-dev libgstreamer1.0-dev \
     libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
     gstreamer1.0-libav gstreamer1.0-gl
pip install --user empy==3.3.4 pexpect future pymavlink

# 2. ArduPilot + Gazebo plugin (once; downloads and builds ~1.5 GB)
./ardupilot/setup.sh ~/

# 3. Run
./ardupilot/run.sh talon
```

For a takeoff and maneuver test, in a second terminal:

```bash
python3 scripts/ap_test.py maneuver
```

To connect with MAVProxy: `mavproxy.py --master tcp:127.0.0.1:5760 --console --map`

> **Note:** Run commands from inside the `toy-px4-sim` folder. From anywhere else, use the full
> path: `~/toy-px4-sim/ardupilot/run.sh talon`

---

## Usage

### Single aircraft

| | PX4 | ArduPilot |
|---|---|---|
| Start | `./run.sh <model>` | `./ardupilot/run.sh <model>` |
| Without a window | `--headless` | `--headless` |
| Free camera | `--no-follow` | `--no-follow` |
| Don't force NVIDIA | `--no-nvidia` | `--no-nvidia` |
| Empty world | `--world default` | — |
| Stop | Ctrl+C | Ctrl+C |

`<model>`: `toy`, `talon`, `mini_talon`, `x8`, `skywalker1900` (default `toy`).

### Multiple aircraft

Up to 10 aircraft; each runs its own autopilot process.

```bash
./multi.sh --headless --vehicles "talon:2,x8:2,mini_talon:2,skywalker1900:2,toy:2"     # PX4
./ardupilot/run.sh --headless --vehicles "talon:2,x8:2,mini_talon:2,skywalker1900:2,toy:2"   # ArduPilot
```

To take them all off together to staggered altitudes (20, 28, 36 … m):

```bash
python3 scripts/swarm_test.py 10        # PX4
python3 scripts/ap_test.py swarm 10     # ArduPilot
```

Aircraft are lined up north of the runway, 6 m apart, facing east. QGroundControl sees all of them
on UDP 14550. Logs are in `logs/`.

### Takeoff location

The course's default location is an example value. Once your flying field is known (PX4 only):

```bash
PX4_HOME_LAT=39.80 PX4_HOME_LON=32.60 PX4_HOME_ALT=900 ./run.sh talon
```

---

## Aircraft

| Model | Name | Span | Mass | Layout | Trim speed |
|---|---|---|---|---|---|
| TOY (the team's aircraft) | `toy` | 1.40 m | 2.75 kg | conventional tail, pusher | 18 m/s |
| X-UAV Talon | `talon` | 1.72 m | 2.8 kg | V-tail, pusher | 15 m/s |
| X-UAV Mini Talon | `mini_talon` | 1.30 m | 1.6 kg | V-tail, pusher | 15 m/s |
| Skywalker X8 | `x8` | 2.12 m | 3.2 kg | flying wing, elevons, pusher | 18 m/s |
| Skywalker 1900 | `skywalker1900` | 1.90 m | 2.0 kg | T-tail, pusher | 13 m/s |

Every aircraft has moving control surfaces, a pusher motor and a nose camera (640×480 @ 20 Hz,
pointing 15° down). Aerodynamic coefficients are computed with
[AVL](https://web.mit.edu/drela/Public/web/avl/).

### Test results

Each aircraft passed the same test with both autopilots: takeoff, then hard 40°-bank turns with
altitude changes to 40, 20 and 35 m.

| | PX4 | ArduPilot |
|---|---|---|
| Individual maneuver test | 5/5 aircraft, lowest altitude 18–21 m | 5/5 aircraft, lowest altitude 19 m |
| 10-aircraft mixed fleet (`--headless`) | 10/10 at target altitude, real time | 10/10 at target altitude, real time |

### Where the dimensions come from

- **TOY:** the team's CAD assembly (`cad/1-Montaj.STEP`). Wing mounted at +3.2° incidence, Clark Y airfoil.
- **Others:** span, length, wing area, weight and tail layout from values published by vendors.
  Chord distribution, tail sizes, fuselage sections and motor position were estimated from photo
  proportions. The X8's inertia is a measured value (Løw-Hansen et al., *CEAS Aeronautical
  Journal*, 2025).
- **Validation:** the computed X8 center of gravity (443 mm from the nose) matches the
  manufacturer's recommended 430–440 mm.

### Notes for the real aircraft

> ⚠️ **TOY's center of gravity:** at 80 mm behind the leading edge, AVL gives a static margin of only
> **~1% MAC** (fuselage not modeled; it may be even lower in reality). In simulation the autopilot
> compensates. On the real aircraft, move the CG forward before the first flight (~6–6.5 cm behind
> the leading edge).

- Speed and gain settings were computed for simulation; on a real aircraft they are starting
  values and should be tuned in flight (autotune).
- The real **X8** is hand- or catapult-launched. In simulation it has a small skid under the nose so
  it can take off from the runway.

---

## Mission course

A realistic-looking test field that needs no internet access. All textures and models are in the repo.

```
                 north (+y)
                    ^
   P4 (0,300) ------ G2 (150,300) ------ P3 (300,300)
      |                                       |
      |   T1 (80,225)   T2 (150,225)   T3 (220,225)
      |                                       |
   P1 (0,150) ------ G1 (150,150) ------ P2 (300,150)

   ===== RUNWAY (x: -40..160) =====>  east (+x)     P  race pylon
   * aircraft at y = 6..60, facing east              G  gate
       hangar (25,-32)  tower (60,-38)   H (230,-70) T  ground target
                                                     H  landing zone
```

| Item | Details |
|---|---|
| Runway | 200 × 18 m, asphalt with markings |
| Race pylons (P) | 4, red-and-white, 10 m |
| Gates (G) | 2, orange, 16 m wide × 12 m high |
| Ground targets (T) | Red, blue and yellow rings |
| Landing zone (H) | White circled "H" |
| Surroundings | Hangar, tower, windsock, 160 trees, cloudy sky |

Coordinates are in meters. Pylons, gates and buildings have collisions; trees don't, for
performance. To change the course, edit the `YERLESIM` (layout) section of `tools/gen_world.py`
and run `python3 tools/gen_world.py`.

---

## Connections and test scripts

| Connection | PX4 | ArduPilot |
|---|---|---|
| QGroundControl | UDP 14550 (automatic) | UDP 14550 (automatic) |
| pymavlink / MAVSDK / MAVProxy | `udp:127.0.0.1:14540` (aircraft i: `14540+i`) | `tcp:127.0.0.1:5760` (aircraft i: `5760+10i`) |
| ROS 2 | uXRCE-DDS agent, UDP 8888 | — |
| Camera video stream | UDP 5600 (RTP/H.264) | — |

**Test scripts** (`scripts/`)

| Script | Autopilot | What it does |
|---|---|---|
| `maneuver_test.py` | PX4 | Takeoff, hard turns, altitude changes; prints control surface commands |
| `swarm_test.py [n]` | PX4 | Takes n aircraft off to staggered altitudes |
| `fw_test.py` | PX4 | Simple takeoff, loiter, RTL |
| `surf_test.py` | PX4 | Moves each control surface ±25° in turn (aircraft on the ground) |
| `ap_test.py maneuver` | ArduPilot | TAKEOFF, then hard turns and altitude changes in GUIDED mode |
| `ap_test.py swarm [n]` | ArduPilot | Takes n aircraft off to staggered altitudes |
| `grab_cam.py <file.png>` | — | Saves a single frame from the nose camera |

If `ap_test.py` is stopped with Ctrl+C, it switches the aircraft to RTL.

> PX4 won't arm without seeing a ground station. If QGroundControl isn't open, your own script must
> send a `MAV_TYPE_GCS` heartbeat (see `scripts/fw_test.py`).

---

## Performance

On hybrid (Intel + NVIDIA) laptops Gazebo renders on the **integrated GPU** by default and stutters.
`run.sh` and `ardupilot/run.sh` automatically route Gazebo to the NVIDIA GPU when one is present.

Measured on an RTX 4050 laptop, in flight:

| Setup | Real-time factor |
|---|---|
| Single aircraft, Intel GPU | 0.29 – 1.00 (stutters) |
| Single aircraft, NVIDIA | **1.00** |
| 10 aircraft, PX4, `--headless` | **1.00** |
| 10 aircraft, ArduPilot, `--headless` | **1.00** |

**For multi-aircraft runs:** use `--headless`; the window also costs CPU.

<details>
<summary>Manual NVIDIA setup and optimizations</summary>

If you don't use `run.sh`:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

World optimizations: the 160 trees are one mesh per color (a few draw calls) with no collisions;
the aircraft camera runs at 640×480 @ 20 Hz instead of 1280×960 @ 30 Hz. Together these cut the
Gazebo server's CPU load for a single aircraft from 140% to 52%.

</details>

---

## Development

### Repository layout

```
run.sh, multi.sh             PX4: single / multiple aircraft
install.sh, uninstall.sh     link into / unlink from PX4
ardupilot/                   ArduPilot: setup.sh, run.sh, params/
airframes/                   PX4 airframes (22000–22004)
models/<name>/               PX4 models
models/<name>_ap/            ArduPilot models
models/toy_ground/           course ground, runway, trees
worlds/                      toy_parkur (PX4), toy_parkur_ap (ArduPilot)
tools/                       generators (see below)
scripts/                     test scripts
cad/1-Montaj.STEP            TOY's CAD assembly
```

### Adding a new aircraft

Add a definition to `tools/plane_specs.py`. Conventional tail, T-tail, V-tail and flying wing are
supported. Then:

```bash
python3 tools/gen_plane.py <name>        # PX4 model + airframe (requires AVL)
python3 tools/gen_ardupilot.py <name>    # ArduPilot model + parameters
./install.sh ~/PX4-Autopilot             # link into PX4
```

For the airframe number, use **22000–22999**, the range PX4 reserves for custom models.

<details>
<summary>How the generators work</summary>

**`tools/gen_plane.py`** (PX4), from a single definition:

1. Builds the meshes (control surfaces as separate, moving parts) and the AVL geometry.
2. Finds the neutral point with AVL and places the CG for a given static margin (10% MAC for
   conventional aircraft, 6% for the flying wing).
3. Sets the tail incidence (wing-tip washout on the flying wing) so the aircraft trims hands-off
   at cruise speed.
4. Computes separate AdvancedLiftDrag coefficients for every control surface.
5. Writes the PX4 airframe: control allocation from AVL derivatives, speeds from the model's own
   stall speed, rate gains scaled from TOY by control-power ratio.

**`tools/gen_ardupilot.py`** generates the ArduPilot variant from the PX4 model. Aerodynamics,
meshes and joints stay the same; what changes:

| | PX4 model | ArduPilot model |
|---|---|---|
| Autopilot bridge | PX4 `gz_bridge` | `ArduPilotPlugin` (JSON, lock-step) |
| Sensors | IMU, barometer, magnetometer, GPS | IMU only (ArduPilot SITL simulates the rest) |
| IMU frame | same as body | rotated 180° about x (the plugin expects forward-right-down) |
| Motor | `MulticopterMotorModel` | Gazebo `Thruster` (throttle → thrust, N) |
| Control surfaces | PX4 servo topics | `COMMAND` channels write to the same joint controllers |
| World | `toy_parkur` (4 ms) | `toy_parkur_ap` (4 ms, real time, system plugins in the world) |

ArduPilot servo directions are measured, not guessed: `tools/ap_calibrate.py` runs ArduPlane with
its built-in model and records the PWM direction of every servo function
(`tools/ardupilot_signs.json`); the generator combines this with the AVL derivatives to pick the
right function (aileron 4, elevator 19, rudder 21, elevon 77/78, V-tail 79/80) and direction for
each surface.

</details>

<details>
<summary>Recomputing TOY's aerodynamics (AVL)</summary>

TOY was built by hand (CAD meshes). When control surface sizes or the CG change:

```bash
nano models/toy/avl/toy.avl                   # edit the geometry
models/toy/avl/run_avl.sh ~/Avl/bin/avl       # run AVL
python3 models/toy/avl/avl_to_sdf.py          # update model.sdf, print static margin
python3 tools/gen_ardupilot.py toy            # update the ArduPilot variant
```

AVL 3.36: <https://web.mit.edu/drela/Public/web/avl/> (builds with gfortran).

| TOY | Value | Source |
|---|---|---|
| Span / area / MAC | 1.40 m / 0.308 m² / 0.222 m | CAD |
| Airfoil / wing incidence | Clark Y (~NACA 3412) / +3.2° | CAD |
| Mass / inertia | 2.75 kg / Ixx 0.15, Iyy 0.14, Izz 0.27 kg·m² | estimate |
| Center of gravity | 80 mm behind the wing root leading edge | team |
| Motor | pusher, 12" prop, 4S, ~29 N static thrust | estimate |
| Control surfaces | ailerons (outer half, 25% chord), elevator/rudder (30% chord), ±25° | assumption |

</details>

<details>
<summary>What <code>install.sh</code> does</summary>

| In this repo | In PX4 (symlink) |
|---|---|
| `models/*/` | `Tools/simulation/gz/models/` |
| `worlds/*.sdf` | `Tools/simulation/gz/worlds/` |
| `airframes/*` | `ROMFS/px4fmu_common/init.d-posix/airframes/` |

It also adds the airframe names to PX4's `airframes/CMakeLists.txt`; that file will show as
modified in the PX4 repo. To undo: `./uninstall.sh ~/PX4-Autopilot`

</details>

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `no such file or directory: ./run.sh` | Run the command inside `toy-px4-sim`, or give the full path. |
| Gazebo stutters | Start with `run.sh` (it selects NVIDIA); `gz sim` should appear in `nvidia-smi`. Use `--headless` for many aircraft. |
| Can't zoom with the mouse | Zoom doesn't work in follow mode: `--no-follow` |
| PX4: `PX4 server already running` | A previous simulation is still running: `pkill -x px4; pkill -f "gz sim"` |
| PX4: arming denied (`No connection to the GCS`) | Open QGroundControl or send a GCS heartbeat from your script. |
| ArduPilot: Gazebo frozen, aircraft doesn't connect | `ardupilot/run.sh` cleans up leftovers from earlier runs itself; if it still happens: `pkill -x arduplane; pkill -f "gz sim"` |
| Some aircraft don't take off in a fleet | Something may be in the takeoff path (aircraft run east); check the logs in `logs/`. |
| Control surfaces seem not to move | In level flight deflections are 1–2°; they become obvious in the maneuver test. |
