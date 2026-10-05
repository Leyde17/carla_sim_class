#!/usr/bin/env python3
"""
Seeded CARLA traffic generator, written as a *passive* client for use with
carla_ros_bridge running as the ticking master (passive:=False on the bridge).

Hard rules this node obeys, because breaking any of them destroys determinism
or fights the bridge for control of the clock:

  * never calls world.tick()
  * never calls world.apply_settings()
  * never calls client.load_world() / reload_world()
  * apply_batch_sync(..., due_tick_cue=False) so batches never cue a tick
  * waits on world.wait_for_tick() when it needs the server to advance

It also reports the observed server tick rate once per wall-clock second, which
is the cheapest way to spot the ~1 Hz crawl that means the bridge is timing out
waiting for a vehicle control command.

Run standalone:
  python3 seeded_traffic_node.py --ros-args -p seed:=1234 -p num_vehicles:=40

Or via the accompanying launch file.
"""

import json
import os
import time

import carla
import rclpy
from rclpy.node import Node

SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
DestroyActor = carla.command.DestroyActor
FutureActor = carla.command.FutureActor


# Every one of these must be supplied by the YAML (or a --ros-args override).
# There are deliberately no in-code defaults: a hidden default is a scenario
# variable that does not appear in the config you archive with your results.
REQUIRED_PARAMS = [
    'host', 'port', 'tm_port', 'timeout',
    'seed', 'num_vehicles', 'num_walkers',
    'tm_synchronous_mode',
    'vehicle_filter', 'walker_filter', 'exclude_bikes',
    'wait_for_ego', 'ego_role_names', 'ego_wait_timeout', 'ego_clearance_m',
    'global_distance_to_leading_vehicle', 'global_percentage_speed_difference',
    'auto_lane_change',
    'percentage_walkers_running', 'percentage_walkers_crossing',
    'manifest_dir', 'report_tick_rate',
]


class SeededTraffic(Node):

    def __init__(self):
        # Parameters are declared from whatever the YAML / CLI overrides
        # provide, rather than hardcoded here.
        super().__init__('seeded_traffic',
                         automatically_declare_parameters_from_overrides=True)

        self.p = self._load_params()

        if self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is True. This node only uses wall-clock timers '
                'for diagnostics; sim time here risks deadlocking against the '
                "bridge's wait-for-control. Recommend use_sim_time:=False.")

        # --- state ------------------------------------------------------
        self.vehicles = []
        self.walkers = []
        self.controllers = []
        self.manifest = {}
        self._last_frame = None
        self._last_wall = None

        self.rng = __import__('random').Random(self.p['seed'])

        self._connect()
        self._verify_world()
        if self.p['wait_for_ego']:
            self._wait_for_ego()
        self._configure_tm()
        self._spawn_vehicles()
        if self.p['num_walkers'] > 0:
            self._spawn_walkers()
        self._write_manifest()

        if self.p['report_tick_rate']:
            self.create_timer(1.0, self._report_tick_rate)

        self.get_logger().info(
            'Traffic ready: %d vehicles, %d walkers, seed %d. '
            'Node is now idle and will NOT tick the world.'
            % (len(self.vehicles), len(self.walkers), self.p['seed']))

    # -------------------------------------------------------------------
    def _load_params(self):
        """Read every required parameter, failing loudly if the YAML did not
        reach this node. A mis-scoped params file (wrong node name at the top
        of the YAML) is silently ignored by ROS 2 -- this turns that into an
        immediate, readable error."""
        values, missing = {}, []
        for name in REQUIRED_PARAMS:
            try:
                v = self.get_parameter(name).value
            except Exception:  # noqa: BLE001 - undeclared in this rclpy version
                v = None
            if v is None:
                missing.append(name)
            else:
                values[name] = v

        if missing:
            raise RuntimeError(
                'Missing parameters: %s\n'
                'Pass a params file (ros2 launch, or --ros-args --params-file '
                '<path>) and check that the top-level key in the YAML matches '
                "this node's name ('seeded_traffic') or is the /** wildcard. "
                'A mismatched key is dropped without warning by ROS 2.'
                % ', '.join(missing))

        self.get_logger().info('Resolved configuration:')
        for k in REQUIRED_PARAMS:
            self.get_logger().info('  %-38s = %s' % (k, values[k]))
        return values

    def _connect(self):
        self.client = carla.Client(self.p['host'], self.p['port'])
        self.client.set_timeout(self.p['timeout'])
        self.world = self.client.get_world()          # NOT load_world
        self.map_name = self.world.get_map().name
        self.get_logger().info('Connected to %s (CARLA %s / client %s)' % (
            self.map_name,
            self.client.get_server_version(),
            self.client.get_client_version()))

    def _verify_world(self):
        """Read-only sanity check. We refuse to run if the world is not in the
        state the bridge should have put it in, rather than 'fixing' it and
        stomping on the ticking master."""
        s = self.world.get_settings()
        self.delta = s.fixed_delta_seconds
        problems = []
        if not s.synchronous_mode:
            problems.append('world is in asynchronous mode')
        if not s.fixed_delta_seconds:
            problems.append('fixed_delta_seconds is not set')
        if getattr(s, 'substepping', False) and s.max_substep_delta_time and \
                s.fixed_delta_seconds and \
                s.fixed_delta_seconds > s.max_substep_delta_time * s.max_substeps:
            problems.append('physics substepping cannot cover fixed_delta_seconds')

        if problems:
            raise RuntimeError(
                'World is not configured for deterministic replay (%s). Launch '
                'the bridge with synchronous_mode:=True and fixed_delta_seconds '
                'set; this node deliberately does not call apply_settings().'
                % '; '.join(problems))

        self.get_logger().info(
            'World OK: synchronous, fixed_delta_seconds=%.4f (%.1f Hz)'
            % (self.delta, 1.0 / self.delta))

    def _wait_for_ego(self):
        names = list(self.p['ego_role_names'])
        deadline = time.time() + self.p['ego_wait_timeout']
        self.get_logger().info('Waiting for ego (role_name in %s)...' % names)
        while time.time() < deadline:
            for a in self.world.get_actors().filter('vehicle.*'):
                if a.attributes.get('role_name') in names:
                    self.ego = a
                    self.get_logger().info(
                        'Found ego id=%d role_name=%s'
                        % (a.id, a.attributes.get('role_name')))
                    return
            try:
                self.world.wait_for_tick(seconds=5.0)
            except RuntimeError:
                self.get_logger().warn(
                    'No tick received in 5s while waiting for ego. Is the '
                    'bridge running as ticking master?')
        self.ego = None
        self.get_logger().warn(
            'Ego not found before timeout. Continuing, but note that traffic '
            'may spawn on top of a later-spawned ego, and run-to-run actor '
            'sets may differ.')

    def _configure_tm(self):
        self.tm = self.client.get_trafficmanager(self.p['tm_port'])

        # The TM is a CLIENT-SIDE module living in THIS process. Its
        # synchronous step is driven by world.tick() called from the process
        # that owns the TM instance. This node never ticks (the bridge does,
        # in its own process), so tm_synchronous_mode=True freezes every
        # autopilot vehicle: the TM control loop waits for a step that never
        # comes. Set it True only if this process also owns the tick.
        tm_sync = bool(self.p['tm_synchronous_mode'])
        if tm_sync:
            self.get_logger().error(
                'tm_synchronous_mode is True but this node does not tick the '
                'world. The Traffic Manager will never be stepped and NO '
                'vehicle will move. Set it False (TM free-runs, weaker '
                'determinism), or move traffic spawning into the process that '
                'owns world.tick() (passive bridge).')
        else:
            self.get_logger().warn(
                'TM is in asynchronous mode. Vehicles will move, but TM '
                'stepping is not locked to the world tick, so runs of the '
                'same seed will NOT be bit-identical. Fine for pipeline '
                'testing; not sufficient for a reproducible corpus.')
        self.tm.set_synchronous_mode(tm_sync)

        self.tm.set_random_device_seed(self.p['seed'])
        self.tm.set_hybrid_physics_mode(False)    # explicit: hybrid breaks determinism
        self.tm.set_global_distance_to_leading_vehicle(
            self.p['global_distance_to_leading_vehicle'])
        self.tm.global_percentage_speed_difference(
            self.p['global_percentage_speed_difference'])
        self.world.set_pedestrians_seed(self.p['seed'])
        self.world.set_pedestrians_cross_factor(
            self.p['percentage_walkers_crossing'] / 100.0)
        self.get_logger().info(
            'TrafficManager on port %d seeded with %d'
            % (self.p['tm_port'], self.p['seed']))

    # -------------------------------------------------------------------
    def _vehicle_blueprints(self):
        bps = list(self.world.get_blueprint_library().filter(
            self.p['vehicle_filter']))
        if self.p['exclude_bikes']:
            bps = [b for b in bps
                   if int(b.get_attribute('number_of_wheels')) == 4]
        # Library iteration order is not contractually stable across versions
        # or machines. Sorting makes rng.choice() reproducible.
        return sorted(bps, key=lambda b: b.id)

    def _spawn_vehicles(self):
        bps = self._vehicle_blueprints()
        points = list(enumerate(self.world.get_map().get_spawn_points()))

        # Exclude points occupied by anything already in the world (the ego).
        existing = [a.get_location()
                    for a in self.world.get_actors().filter('vehicle.*')]
        clear = self.p['ego_clearance_m']
        points = [(i, tf) for (i, tf) in points
                  if all(tf.location.distance(loc) > clear for loc in existing)]

        self.rng.shuffle(points)

        n = self.p['num_vehicles']
        if n > len(points):
            self.get_logger().warn(
                'Requested %d vehicles but only %d free spawn points; '
                'clamping. Record this: actor count is part of the scenario.'
                % (n, len(points)))
            n = len(points)
        chosen = points[:n]

        batch, planned = [], []
        for sp_index, tf in chosen:
            bp = self.rng.choice(bps)
            if bp.has_attribute('color'):
                colors = sorted(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', self.rng.choice(colors))
            if bp.has_attribute('driver_id'):
                ids = sorted(bp.get_attribute('driver_id').recommended_values)
                bp.set_attribute('driver_id', self.rng.choice(ids))
            bp.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(bp, tf).then(
                SetAutopilot(FutureActor, True, self.p['tm_port'])))
            planned.append({'spawn_point_index': sp_index,
                            'blueprint': bp.id,
                            'x': round(tf.location.x, 3),
                            'y': round(tf.location.y, 3)})

        frame_before = self.world.get_snapshot().frame
        # due_tick_cue=False is critical: True would ask the server to tick and
        # steal a step from the bridge.
        responses = self.client.apply_batch_sync(batch, False)

        for rec, resp in zip(planned, responses):
            if resp.error:
                rec['actor_id'] = None
                rec['error'] = resp.error
                self.get_logger().warn('Spawn failed: %s' % resp.error)
            else:
                rec['actor_id'] = resp.actor_id
                self.vehicles.append(resp.actor_id)

        for actor in self.world.get_actors(self.vehicles):
            self.tm.auto_lane_change(actor, bool(self.p['auto_lane_change']))

        self.manifest['vehicles'] = planned
        self.manifest['frame_before_vehicle_spawn'] = frame_before
        self.manifest['frame_after_vehicle_spawn'] = \
            self.world.get_snapshot().frame

    def _spawn_walkers(self):
        bps = sorted(self.world.get_blueprint_library().filter(
            self.p['walker_filter']), key=lambda b: b.id)

        # get_random_location_from_navigation() consumes the server-side
        # pedestrian RNG seeded above, so call count and order matter.
        spawn_tfs, speeds = [], []
        attempts, want = 0, self.p['num_walkers']
        while len(spawn_tfs) < want and attempts < want * 10:
            attempts += 1
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                tf = carla.Transform()
                tf.location = loc
                spawn_tfs.append(tf)

        batch, planned = [], []
        for tf in spawn_tfs:
            bp = self.rng.choice(bps)
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')
            if bp.has_attribute('speed'):
                vals = bp.get_attribute('speed').recommended_values
                run = self.rng.random() * 100.0 < self.p['percentage_walkers_running']
                speed = vals[2] if (run and len(vals) > 2) else vals[1]
            else:
                speed = '0.0'
            speeds.append(float(speed))
            batch.append(SpawnActor(bp, tf))
            planned.append({'blueprint': bp.id, 'speed': float(speed),
                            'x': round(tf.location.x, 3),
                            'y': round(tf.location.y, 3)})

        results = self.client.apply_batch_sync(batch, False)
        kept_speeds = []
        for rec, resp, spd in zip(planned, results, speeds):
            if resp.error:
                rec['actor_id'] = None
                rec['error'] = resp.error
            else:
                rec['actor_id'] = resp.actor_id
                self.walkers.append(resp.actor_id)
                kept_speeds.append(spd)

        # The walker must exist on the server before its controller attaches.
        # We cannot tick, so we wait for the bridge's next tick.
        self._wait_tick('after walker spawn')

        ctrl_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        ctrl_batch = [SpawnActor(ctrl_bp, carla.Transform(), wid)
                      for wid in self.walkers]
        ctrl_results = self.client.apply_batch_sync(ctrl_batch, False)
        for resp in ctrl_results:
            if not resp.error:
                self.controllers.append(resp.actor_id)

        self._wait_tick('after controller spawn')

        for ctrl, spd in zip(self.world.get_actors(self.controllers), kept_speeds):
            ctrl.start()
            ctrl.go_to_location(self.world.get_random_location_from_navigation())
            ctrl.set_max_speed(spd)

        self.manifest['walkers'] = planned

    def _wait_tick(self, why):
        try:
            self.world.wait_for_tick(seconds=10.0)
        except RuntimeError:
            self.get_logger().error(
                'No tick from the server within 10s (%s). The bridge is not '
                'advancing the world -- if wait-for-vehicle-control is on, '
                'nothing is publishing to the ego control topic.' % why)

    # -------------------------------------------------------------------
    def _write_manifest(self):
        self.manifest.update({
            'seed': self.p['seed'],
            'map': self.map_name,
            'fixed_delta_seconds': self.delta,
            'num_vehicles_requested': self.p['num_vehicles'],
            'num_vehicles_spawned': len(self.vehicles),
            'num_walkers_spawned': len(self.walkers),
            'server_version': self.client.get_server_version(),
            'tm_port': self.p['tm_port'],
            'resolved_params': dict(self.p),
        })
        os.makedirs(self.p['manifest_dir'], exist_ok=True)
        path = os.path.join(
            self.p['manifest_dir'],
            'manifest_seed%d_%d.json' % (self.p['seed'], int(time.time())))
        with open(path, 'w') as f:
            json.dump(self.manifest, f, indent=2, sort_keys=True)
        self.get_logger().info('Manifest: %s' % path)
        self.get_logger().info(
            'Diff two runs of the same seed with: diff <(jq -S . a.json) '
            '<(jq -S . b.json)')

    def _report_tick_rate(self):
        frame = self.world.get_snapshot().frame
        now = time.time()
        if self._last_frame is not None:
            dframes = frame - self._last_frame
            rate = dframes / max(now - self._last_wall, 1e-6)
            if dframes == 0:
                self.get_logger().warn(
                    'Server has not ticked in ~1s. The bridge is blocked.')
            elif rate < 2.0:
                self.get_logger().warn(
                    'Server tick rate %.2f Hz -- this is the wait-for-control '
                    'timeout signature, not real-time slowdown. Check that '
                    'something is publishing vehicle_control_cmd every cycle.'
                    % rate)
            else:
                self.get_logger().info(
                    'frame=%d  tick rate=%.1f Hz  (sim rate %.2fx real time)'
                    % (frame, rate, rate * self.delta))
        self._last_frame, self._last_wall = frame, now

    # -------------------------------------------------------------------
    def destroy_traffic(self):
        self.get_logger().info('Destroying spawned actors...')
        for ctrl in self.world.get_actors(self.controllers):
            try:
                ctrl.stop()
            except RuntimeError:
                pass
        ids = self.controllers + self.walkers + self.vehicles
        self.client.apply_batch([DestroyActor(x) for x in ids])
        self.get_logger().info('Destroyed %d actors.' % len(ids))


def main():
    rclpy.init()
    node = None
    try:
        node = SeededTraffic()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print('[seeded_traffic] %s' % exc)
    finally:
        if node is not None:
            try:
                node.destroy_traffic()
            except Exception as exc:  # noqa: BLE001
                print('[seeded_traffic] cleanup failed: %s' % exc)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()