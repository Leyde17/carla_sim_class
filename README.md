# carla_sim_class

A minimal CARLA + ROS 2 Humble sandbox, trimmed down from `PathPlanningWS`, for four things only:

1. Generating custom simulation scenarios (OpenSCENARIO + `scenario_runner`).
2. Spawning a simulated ego vehicle equipped with **only a lidar** (plus the pseudo-sensors needed for odometry/TF/control).
3. Generating seeded, reproducible background traffic.
4. Driving the ego vehicle by hand with a Logitech G29 wheel and pedals.

It does not include any of the platooning/ACC control stack, custom FSM messages, or V2V logic from `PathPlanningWS` — those are a separate, unrelated concern. See "Why `autoware_perception_msgs` is here" below for the one exception.

## Layout

```
src/
├── carla_ros/                   # submodule: gezp/carla_ros (branch humble-carla-0.9.14),
│                                 # fork of carla-simulator/ros-bridge for CARLA 0.9.14 / ROS 2 Humble.
│                                 # carries its own nested submodule, carla_msgs.
├── autoware_perception_msgs/    # vendored copy, see note below. Do not edit directly.
├── sim_bringup/                 # top-level launch files tying everything together
│   ├── config/objects_lidar_only.json
│   └── launch/ego_vehicle.launch.py, sim_demo.launch.py
├── sim_traffic/                 # seeded background traffic (vehicles + pedestrians)
├── g29_control/                 # G29 wheel -> CarlaEgoVehicleControl, + optional route CSV logger
└── sim_scenarios/                # OpenSCENARIO (.xosc) playback via scenario_runner
```

## Prerequisites

- Ubuntu 22.04, ROS 2 Humble.
- A running CARLA 0.9.14 server, and the matching Python API wheel installed in the environment that runs the ROS nodes (`pip3 install <carla-0.9.14-...-wheel>.whl` — see `src/carla_ros/README.md` for where to get it). Nothing in this repo vendors the CARLA server or its Python API.
- The ROS 2 `joy` package (`sudo apt install ros-humble-joy`) and a Logitech G29 plugged in, for requirement 4. No force-feedback is configured anywhere in this repo — FFB behavior comes from whatever the kernel HID driver does by default.
- For requirement 1 only: a separate checkout of [`scenario_runner`](https://github.com/carla-simulator/scenario_runner), with `SCENARIO_RUNNER_PATH` pointing at it. This repo only orchestrates `scenario_runner`; it does not implement scenario-generation logic itself.

## Build

```bash
git clone --recurse-submodules <this-repo-url>
cd carla_sim_class
rosdep install --from-paths src --ignore-src -r
colcon build --symlink-install
source install/setup.bash
```

## Usage

Start the CARLA server first, then:

```bash
# Spawn the bridge + lidar-only ego vehicle
ros2 launch sim_bringup ego_vehicle.launch.py

# In another terminal: seeded background traffic
ros2 launch sim_traffic seeded_traffic.launch.py seed:=1234 num_vehicles:=30

# In another terminal: drive it with the G29
ros2 launch g29_control manual_recording.launch.py

# Or run a custom scenario instead of/alongside manual driving
ros2 launch sim_scenarios run_scenario.launch.py scenario_file:=/abs/path/my_scenario.xosc

# Or bring up everything (ego + traffic + G29) in one shot
ros2 launch sim_bringup sim_demo.launch.py seed:=42 num_vehicles:=20
```

Note: the bridge is launched with `synchronous_mode_wait_for_vehicle_control_command:=True`, so the world will not advance until something is publishing to `/carla/<role_name>/vehicle_control_cmd` — that's the G29 node (or your own controller) once it starts.

To write your own scenario, copy `src/sim_scenarios/scenarios/FollowLeadingVehicle.xosc` (CARLA's stock example, vendored here for reference) and edit it, or author one from scratch per the OpenSCENARIO spec that `scenario_runner` consumes.

To change what sensors the ego vehicle carries, edit `src/sim_bringup/config/objects_lidar_only.json` — see `src/carla_ros/carla_spawn_objects/config/objects.json` in the submodule for the full schema and other sensor types available.

## Why `autoware_perception_msgs` is here

`src/autoware_perception_msgs` mirrors `PathPlanningWS/src/autoware_perception_msgs` — a vendored subset of real Autoware perception messages (`DetectedObjects`, `TrackedObjects`, etc.) — so that anyone building a perception pipeline against this sandbox's lidar output produces messages in the same wire format the real AV stack expects, making it easier to plug their work into that stack later. It is **not edited directly here**: it's a one-way copy (perception's own repo → `PathPlanningWS` → here), so if you need an updated subset, pull the current copy from `PathPlanningWS/src/autoware_perception_msgs` rather than hand-editing this one. It is also, per its own description, transcribed from the Autoware message schema rather than copied from a real Autoware checkout — diff against `github.com/autowarefoundation/autoware_msgs` before assuming full wire-compatibility with external Autoware tooling.
