# carla_sim_class

A minimal CARLA + ROS 2 Humble sandbox for four things only:

1. Generating custom simulation scenarios (OpenSCENARIO + `scenario_runner`).
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
└── sim_scenarios/                # OpenSCENARIO (.xosc) playback via scenario_runner
```

- **`carla_ros`** provides the bridge, the object-spawning node, and the scenario-runner wrapper. It is the only large piece here; everything else is small and specific to one of the four capabilities above.
- **`autoware_perception_msgs`** is included so that a perception pipeline built against this sandbox's lidar output publishes messages in a format compatible with real Autoware-style AV stacks, rather than inventing a one-off schema. It carries no logic of its own.
- **`sim_bringup`** is the only package you need to touch to change what sensors the ego vehicle carries or how the pieces are wired together.

## What's needed to use it

- Ubuntu 22.04, ROS 2 Humble.
- A running CARLA 0.9.14 server, and the matching Python API wheel installed in the environment that runs the ROS nodes (`pip3 install <carla-0.9.14-...-wheel>.whl`). Nothing in this repo vendors the CARLA server or its Python API.
- The ROS 2 `joy` package (`sudo apt install ros-humble-joy`) and a Logitech G29 plugged in, for requirement 4. No force-feedback is configured anywhere in this repo — FFB behavior comes from whatever the kernel HID driver does by default.
- For requirement 1 only: a separate checkout of [`scenario_runner`](https://github.com/carla-simulator/scenario_runner), with `SCENARIO_RUNNER_PATH` pointing at it. This repo only orchestrates `scenario_runner`; it does not implement scenario-generation logic itself.

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

# Or run a custom scenario instead of/alongside manual driving
ros2 launch sim_scenarios run_scenario.launch.py scenario_file:=/abs/path/my_scenario.xosc

# Or bring up everything (ego + traffic + G29) in one shot
ros2 launch sim_bringup sim_demo.launch.py seed:=42 num_vehicles:=20
```

To write your own scenario, copy `src/sim_scenarios/scenarios/FollowLeadingVehicle.xosc` (CARLA's stock example, vendored here for reference) and edit it, or author one from scratch per the OpenSCENARIO spec that `scenario_runner` consumes.

To change what sensors the ego vehicle carries, edit `src/sim_bringup/config/objects_lidar_only.json` — see `src/carla_ros/carla_spawn_objects/config/objects.json` in the submodule for the full schema and other sensor types available.

## Notes on things that may need clarification

- **The bridge blocks until something drives the vehicle.** `ego_vehicle.launch.py` sets `synchronous_mode_wait_for_vehicle_control_command:=True`, so the simulation will not advance past the first tick until something publishes to `/carla/<role_name>/vehicle_control_cmd` — normally the G29 node, but any controller works. This is deliberate (it's what makes seeded traffic reproducible), but it can look like a hang if you spawn the ego and nothing else.
- **`sim_demo.launch.py`'s `with_scenario:=true` only runs the bundled example scenario**, not an arbitrary file — passing a custom `scenario_file` through that shortcut isn't wired up. For a custom scenario, launch `sim_scenarios run_scenario.launch.py` directly with your own `scenario_file`/`scenario_name` instead.
- **`g29_control` publishes raw `CarlaEgoVehicleControl` directly**, not through `carla_ackermann_control`. That's simpler and is what the original node already did, but it means there's no Ackermann-style smoothing layer; the Ackermann packages aren't vendored here at all. If you want that smoothing, it would need to be added back in.
- **`autoware_perception_msgs` is a static, hand-maintained copy**, not something this repo can refresh automatically. Its own description notes it was transcribed from the Autoware message schema rather than generated from a real Autoware checkout — diff against `github.com/autowarefoundation/autoware_msgs` before assuming full wire-compatibility with external Autoware tooling, and treat this directory as something to replace wholesale (not hand-edit) if a newer/different subset is ever needed.
- **The G29 axis/button mapping is hardcoded** in `g29_to_carla_node.py` (steer=axis 0, throttle=axis 2, brake=axis 3, reverse=button 4), based on one specific wheel verified with `jstest`. A different G29 unit or a different `joy` driver version could map axes differently — verify with `jstest` or `ros2 topic echo /joy` before assuming it's correct on a new machine.
- **Only the `FollowLeadingVehicle.xosc` example scenario has been smoke-tested** as part of putting this repo together; actual scenario execution (via `scenario_runner`) has not been run end-to-end here, since the distrobox/host used for validation didn't have a `scenario_runner` checkout available. The ego-vehicle spawn and seeded-traffic paths have been verified live against a running CARLA 0.9.14 server; G29 hardware input has not.
