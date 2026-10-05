#!/usr/bin/env python3
"""
yaml_scenario_node.py -- spawn a fixed set of actors from a plain YAML file,
each on its own trigger, using only direct CARLA Python API calls.

This is the lightweight alternative to sim_scenarios' OpenSCENARIO +
scenario_runner path: no external project to install, no standard to learn,
just a YAML list of "what to spawn, where, and when". It trades the
standard-compliance and tooling of OpenSCENARIO for simplicity -- pick
whichever fits the people you're handing the scenario-writing job to.

YAML schema:

    actors:
      - blueprint: static.prop.trafficcone01
        spawn_point: {x: 10.0, y: 5.0, z: 0.2, yaw: 0.0}
        # trigger defaults to "immediate" if omitted
      - blueprint: static.prop.trafficcone01
        spawn_point: {x: 40.0, y: 5.0, z: 0.2}
        trigger: {type: time, after: 8.0}
      - blueprint: vehicle.tesla.model3
        spawn_point: {x: -20.0, y: 8.0, z: 0.3, yaw: 90.0}
        trigger: {type: ego_distance, after_m: 25.0}
        autopilot: true
        physics: true

Every spawn_point field (roll/pitch/yaw, x/y/z) is optional except x/y/z;
angles default to 0.0. Coordinates are in CARLA's own frame (the same
convention used by sim_bringup's objects_lidar_only.json), so there's no
ROS/CARLA axis conversion to worry about.

Trigger types:
  immediate     spawn as soon as this node connects (the default).
  time          spawn `after` seconds of this node's own uptime.
  ego_distance  spawn once the ego (role_name `ego_role_name`, default
                "ego_vehicle") has travelled `after_m` metres of straight-line
                path length since this node started listening to its
                odometry. No recorded route is needed -- just accumulated
                distance between consecutive odometry samples.

Each actor is spawned at most once and left in place for the rest of the
run; this node does not despawn or move anything afterward. Spawned actors
default to physics disabled (frozen exactly at the given pose) unless
`physics: true` is set -- appropriate for static props, overridden for
vehicles you want to actually drive or hand to autopilot.

Run standalone:
  ros2 run sim_scenarios_yaml yaml_scenario --ros-args \
      -p scenario_yaml:=/abs/path/my_scenario.yaml

Or via the accompanying launch file.
"""

import time

import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    import carla
except ImportError:                      # pragma: no cover
    carla = None


class YamlScenario(Node):

    def __init__(self):
        super().__init__('yaml_scenario')

        self.declare_parameter('scenario_yaml', '')
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)

        g = lambda n: self.get_parameter(n).value
        scenario_yaml = str(g('scenario_yaml'))
        self._host = str(g('carla_host'))
        self._port = int(g('carla_port'))

        self._start_time = time.monotonic()
        self._ego_last_xy = {}
        self._ego_distance = {}

        if not scenario_yaml:
            self.get_logger().error(
                'scenario_yaml is required; node will not spawn anything.')
            self._actors = []
        else:
            self._actors = _load_scenario(scenario_yaml)
            self.get_logger().info(
                f'Loaded {len(self._actors)} actor(s) from {scenario_yaml}.')

        self._client = None
        self._world = None
        if carla is None:
            self.get_logger().error(
                'carla PythonAPI is not importable; this node cannot spawn '
                'anything. Run it where the CARLA .egg / PythonAPI is on the path.')
        else:
            try:
                self._client = carla.Client(self._host, self._port)
                self._client.set_timeout(10.0)
                self._world = self._client.get_world()
                self.get_logger().info('Connected to CARLA for spawning.')
            except Exception as e:                      # pragma: no cover
                self.get_logger().error(f'Could not connect to CARLA: {e}')

        self._ego_subs = {}
        for actor in self._actors:
            trig = actor['trigger']
            if trig['type'] == 'ego_distance':
                role = trig['ego_role_name']
                if role not in self._ego_subs:
                    self._ego_subs[role] = self.create_subscription(
                        Odometry, f'/carla/{role}/odometry',
                        lambda msg, role=role: self._ego_odom_cb(role, msg), 10)

        # Spawn immediate/time actors as soon as we're connected, then poll
        # periodically for time-based triggers. ego_distance triggers are
        # checked from the odometry callback instead.
        self._spawn_ready(reason='startup')
        self.create_timer(0.5, lambda: self._spawn_ready(reason='timer'))

    # -------------------------------------------------------------------
    def _ego_odom_cb(self, role, msg: Odometry):
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        last = self._ego_last_xy.get(role)
        if last is not None:
            dx, dy = xy[0] - last[0], xy[1] - last[1]
            self._ego_distance[role] = self._ego_distance.get(role, 0.0) \
                + (dx * dx + dy * dy) ** 0.5
        self._ego_last_xy[role] = xy
        self._spawn_ready(reason='odom')

    # -------------------------------------------------------------------
    def _spawn_ready(self, reason):
        if self._world is None:
            return
        now = time.monotonic() - self._start_time
        for actor in self._actors:
            if actor['spawned']:
                continue
            trig = actor['trigger']
            ready = False
            if trig['type'] == 'immediate':
                ready = True
            elif trig['type'] == 'time':
                ready = now >= trig['after']
            elif trig['type'] == 'ego_distance':
                ready = self._ego_distance.get(trig['ego_role_name'], 0.0) >= trig['after_m']
            if ready:
                self._spawn(actor)

    # -------------------------------------------------------------------
    def _spawn(self, actor):
        bp_lib = self._world.get_blueprint_library()
        try:
            bp = bp_lib.find(actor['blueprint'])
        except IndexError:
            self.get_logger().error(
                f"Unknown blueprint '{actor['blueprint']}'; skipping this actor "
                f"for good (check world.get_blueprint_library() for valid ids).")
            actor['spawned'] = True   # don't retry an unknown blueprint forever
            return

        sp = actor['spawn_point']
        tf = carla.Transform(
            carla.Location(x=sp['x'], y=sp['y'], z=sp['z']),
            carla.Rotation(roll=sp['roll'], pitch=sp['pitch'], yaw=sp['yaw']))

        new_actor = self._world.try_spawn_actor(bp, tf)
        if new_actor is None:
            self.get_logger().warn(
                f"Spawn failed for '{actor['blueprint']}' at "
                f"({sp['x']:.1f}, {sp['y']:.1f}, {sp['z']:.1f}) -- possible "
                f"collision at the spawn point; will retry next cycle.")
            return

        if not actor['physics']:
            new_actor.set_simulate_physics(False)
        if actor['autopilot'] and hasattr(new_actor, 'set_autopilot'):
            new_actor.set_autopilot(True)

        actor['spawned'] = True
        actor['actor_id'] = new_actor.id
        self.get_logger().info(
            f"Spawned '{actor['blueprint']}' (id={new_actor.id}) at "
            f"({sp['x']:.1f}, {sp['y']:.1f}, {sp['z']:.1f}), "
            f"trigger={actor['trigger']['type']}")


# ============================ scenario loading ============================ #

def _load_scenario(yaml_path):
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    actors = []
    for a in doc.get('actors', []):
        sp_in = a.get('spawn_point', {})
        spawn_point = {
            'x': float(sp_in['x']),
            'y': float(sp_in['y']),
            'z': float(sp_in.get('z', 0.0)),
            'roll': float(sp_in.get('roll', 0.0)),
            'pitch': float(sp_in.get('pitch', 0.0)),
            'yaw': float(sp_in.get('yaw', 0.0)),
        }

        trig_in = a.get('trigger', {'type': 'immediate'})
        trig_type = trig_in.get('type', 'immediate')
        if trig_type == 'time':
            trigger = {'type': 'time', 'after': float(trig_in.get('after', 0.0))}
        elif trig_type == 'ego_distance':
            trigger = {
                'type': 'ego_distance',
                'ego_role_name': str(trig_in.get('ego_role_name', 'ego_vehicle')),
                'after_m': float(trig_in.get('after_m', 0.0)),
            }
        else:
            trigger = {'type': 'immediate'}

        actors.append({
            'blueprint': str(a['blueprint']),
            'spawn_point': spawn_point,
            'trigger': trigger,
            'autopilot': bool(a.get('autopilot', False)),
            'physics': bool(a.get('physics', False)),
            'spawned': False,
            'actor_id': None,
        })
    return actors


def main(args=None):
    rclpy.init(args=args)
    node = YamlScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
