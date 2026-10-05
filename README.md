# carla_sim_class

A minimal CARLA + ROS 2 Humble sandbox for four things only:

1. Generating custom simulation scenarios — two independent options: OpenSCENARIO + `scenario_runner` (standards-based, external dependency), or a plain YAML actor/trigger list with no external dependency at all. Pick whichever fits.
2. Spawning a simulated ego vehicle equipped with **only a lidar** (plus the pseudo-sensors needed for odometry/TF/control).
3. Generating seeded, reproducible background traffic.
4. Driving the ego vehicle by hand with a Logitech G29 wheel and pedals.

There is no platooning/ACC control stack, no custom vehicle-state messages, and no V2V logic here — those are a separate concern this repo deliberately leaves out.

## What's included

```
src/
├── carla_ros/                   # submodule: gezp/carla_ros (branch humble-carla-0.9.14),
│                                 # fork of carla-simulator/ros-bridge for CARLA 0.9.14 / ROS 2 Humble.
│                                 # carries its own nested submodule, carla_msgs.
├── autoware_perception_msgs/    # vendored copy of a subset of Autoware's perception messages.
│                                 # Do not edit directly -- see "Notes" below.
├── sim_bringup/                 # top-level launch files tying everything together
│   ├── config/objects_lidar_only.json
│   └── launch/ego_vehicle.launch.py, sim_demo.launch.py
├── sim_traffic/                 # seeded background traffic (vehicles + pedestrians)
├── g29_control/                 # G29 wheel -> CarlaEgoVehicleControl, + optional route CSV logger
├── sim_scenarios/                # OpenSCENARIO (.xosc) playback via scenario_runner
└── sim_scenarios_yaml/           # native alternative: plain YAML actor/trigger list, no scenario_runner
```

- **`carla_ros`** provides the bridge, the object-spawning node, and the scenario-runner wrapper. It is the only large piece here; everything else is small and specific to one of the four capabilities above.
- **`autoware_perception_msgs`** is included so that a perception pipeline built against this sandbox's lidar output publishes messages in a format compatible with real Autoware-style AV stacks, rather than inventing a one-off schema. It carries no logic of its own.
- **`sim_bringup`** is the only package you need to touch to change what sensors the ego vehicle carries or how the pieces are wired together.
- **`sim_scenarios` vs. `sim_scenarios_yaml`**: two independent ways to do requirement 1, not two steps of the same pipeline. `sim_scenarios` orchestrates the external, standards-based `scenario_runner` project against `.xosc` (OpenSCENARIO) files. `sim_scenarios_yaml` is a small self-contained ROS node that spawns actors straight from a YAML file — no external project, no standard to learn, at the cost of being far less expressive (no maneuver/trigger logic beyond "spawn this blueprint at this pose when X happens"). Use either, both, or neither.

## What's needed to use it

- Ubuntu 22.04, ROS 2 Humble.
- A running CARLA 0.9.14 server, and the matching Python API wheel installed in the environment that runs the ROS nodes (`pip3 install <carla-0.9.14-...-wheel>.whl`). Nothing in this repo vendors the CARLA server or its Python API.
- The ROS 2 `joy` package (`sudo apt install ros-humble-joy`) and a Logitech G29 plugged in, for requirement 4. No force-feedback is configured anywhere in this repo — FFB behavior comes from whatever the kernel HID driver does by default.
- For requirement 1 via `sim_scenarios` (OpenSCENARIO) only: a checkout of [`scenario_runner`](https://github.com/carla-simulator/scenario_runner) **at tag `v0.9.13`** (see "Notes" below for why), with `SCENARIO_RUNNER_PATH` pointing at it, plus its `requirements.txt` installed and two extra packages it needs but doesn't declare: `pip install pexpect "antlr4-python3-runtime==4.10"`. This repo only orchestrates `scenario_runner`; it does not implement scenario-generation logic itself.
- For requirement 1 via `sim_scenarios_yaml` (native) only: nothing extra beyond `python3-yaml`, which `rosdep install` pulls in automatically.

## How to use it

Build:

```bash
git clone --recurse-submodules <this-repo-url>
cd carla_sim_class
rosdep install --from-paths src --ignore-src -r
colcon build --symlink-install
source install/setup.bash
```

Start the CARLA server first, then:

```bash
# Spawn the bridge + lidar-only ego vehicle
ros2 launch sim_bringup ego_vehicle.launch.py

# In another terminal: seeded background traffic
ros2 launch sim_traffic seeded_traffic.launch.py seed:=1234 num_vehicles:=30

# In another terminal: drive it with the G29
ros2 launch g29_control manual_recording.launch.py

# Or run a custom OpenSCENARIO scenario instead of/alongside manual driving
ros2 launch sim_scenarios run_scenario.launch.py scenario_file:=/abs/path/my_scenario.xosc

# Or run a custom native YAML scenario instead -- no scenario_runner needed
ros2 launch sim_scenarios_yaml run_yaml_scenario.launch.py scenario_yaml:=/abs/path/my_scenario.yaml

# Or bring up everything (ego + traffic + G29 + a scenario) in one shot
ros2 launch sim_bringup sim_demo.launch.py seed:=42 num_vehicles:=20 with_scenario:=true
ros2 launch sim_bringup sim_demo.launch.py seed:=42 num_vehicles:=20 with_yaml_scenario:=true
```

To write your own OpenSCENARIO scenario, copy `src/sim_scenarios/scenarios/FollowLeadingVehicle.xosc` (CARLA's stock example, vendored here for reference) and edit it, or author one from scratch per the OpenSCENARIO spec that `scenario_runner` consumes.

To write your own native YAML scenario instead, copy `src/sim_scenarios_yaml/scenarios/example_cone_scenario.yaml` and edit it. The schema is a flat list of actors, each with a `blueprint`, a `spawn_point` (CARLA-frame x/y/z/yaw, same convention as `objects_lidar_only.json`), and a `trigger` — `immediate` (default), `time` (after N seconds), or `ego_distance` (after the ego has travelled N metres, tracked from its own odometry, no recorded route needed). See `yaml_scenario_node.py`'s module docstring for the full schema.

To change what sensors the ego vehicle carries, edit `src/sim_bringup/config/objects_lidar_only.json` — see `src/carla_ros/carla_spawn_objects/config/objects.json` in the submodule for the full schema and other sensor types available.

## Notes on things that may need clarification

- **The bridge blocks until something drives the vehicle.** `ego_vehicle.launch.py` sets `synchronous_mode_wait_for_vehicle_control_command:=True`, so the simulation will not advance past the first tick until something publishes to `/carla/<role_name>/vehicle_control_cmd` — normally the G29 node, but any controller works. This is deliberate (it's what makes seeded traffic reproducible), but it can look like a hang if you spawn the ego and nothing else.
- **`sim_demo.launch.py`'s `with_scenario:=true` and `with_yaml_scenario:=true` only run each package's bundled example scenario**, not an arbitrary file — passing a custom `scenario_file`/`scenario_yaml` through either shortcut isn't wired up. For a custom scenario, launch `sim_scenarios run_scenario.launch.py` or `sim_scenarios_yaml run_yaml_scenario.launch.py` directly with your own file instead.
- **`g29_control` publishes raw `CarlaEgoVehicleControl` directly**, not through `carla_ackermann_control`. That's simpler and is what the original node already did, but it means there's no Ackermann-style smoothing layer; the Ackermann packages aren't vendored here at all. If you want that smoothing, it would need to be added back in.
- **`autoware_perception_msgs` is a static, hand-maintained copy**, not something this repo can refresh automatically. Its own description notes it was transcribed from the Autoware message schema rather than generated from a real Autoware checkout — diff against `github.com/autowarefoundation/autoware_msgs` before assuming full wire-compatibility with external Autoware tooling, and treat this directory as something to replace wholesale (not hand-edit) if a newer/different subset is ever needed.
- **The G29 axis/button mapping is hardcoded** in `g29_to_carla_node.py` (steer=axis 0, throttle=axis 2, brake=axis 3, reverse=button 4), based on one specific wheel verified with `jstest`. A different G29 unit or a different `joy` driver version could map axes differently — verify with `jstest` or `ros2 topic echo /joy` before assuming it's correct on a new machine.
- **Pin `scenario_runner` to tag `v0.9.13`, not `master` or newer tags, when pairing with CARLA 0.9.14.** `scenario_runner` enforces its own minimum-CARLA-client check at startup: `master` and `v0.9.15` reject anything below CARLA 0.9.15 outright (`ImportError: CARLA version 0.9.15 or newer required`), while `v0.9.13` only requires `>= 0.9.12` and works fine against a 0.9.14 client. There is no `v0.9.14` tag upstream.
- **Two `scenario_runner`/`carla_ros_scenario_runner` dependencies are never declared anywhere and will surface as import errors the first time you run a scenario**: `pexpect` (imported directly by `carla_ros_scenario_runner`'s own node code) and, on `scenario_runner` versions that include OSC2 support, `antlr4-python3-runtime==4.10` (imported unconditionally at the top of `scenario_runner.py` even when you're only running a plain XML `.xosc` file, not an OSC2 one). Install both alongside `scenario_runner`'s own `requirements.txt` before trying to run a scenario. If you pip-install these into a shared Python environment rather than a venv, watch out for `scenario_runner`'s pinned `numpy`/`opencv-python` versions — they can be much older than what other ROS packages in the same environment expect; installing into an isolated location (e.g. `pip install --target=<dir>` and only prepending that directory to `PYTHONPATH` for the scenario launch itself) avoids downgrading anything shared.
- **The bundled `FollowLeadingVehicle.xosc` references a vehicle blueprint (`vehicle.lincoln.mkz2017`) that doesn't exist in CARLA 0.9.14's blueprint library.** `scenario_runner` logs a warning and falls back to `vehicle.tesla.model3`, but the fallback did not reliably succeed in testing (`Error: Unable to add actors`). If you see this, check `world.get_blueprint_library().filter('vehicle.*')` against whatever blueprint your `.xosc` names before assuming the scenario itself is broken.
- **End-to-end scenario execution via `sim_scenarios` (ego actually driving the maneuver to completion) has not been cleanly observed.** What has been confirmed live, against a running CARLA 0.9.14 server: the ego vehicle and its lidar-only sensor suite spawn correctly (`sim_bringup`), seeded traffic spawns the exact requested vehicle count under a given seed (`sim_traffic`), and `sim_scenarios run_scenario.launch.py` correctly invokes `scenario_runner`, which correctly recognizes the spawned `ego_vehicle` by role name and begins loading the scenario — up to the point of the blueprint issue above. G29 hardware input has not been tested at all (no physical wheel was available during validation).
- **`sim_scenarios_yaml` has only been build- and launch-argument-checked, not run live against a CARLA server.** Its `ego_distance` trigger in particular is worth a second look before trusting it for anything precise: it accumulates straight-line distance between consecutive odometry samples starting from whenever the node happens to receive its first one, which depends on when the node starts relative to the ego — if the ego is already moving when the node connects, that initial movement is not counted.
