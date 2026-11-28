"""Learn a closed corridor, then drive laps from its saved route."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
from threading import Thread
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.exceptions import InvalidTopicNameException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.validate_full_topic_name import validate_full_topic_name
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from corridor import BEAM_WIDTH, plan
from lap_map import (
    LapGate, build_closed_route, find_loop_edge, keyframes_for_loop, parse_slam_graph,
    repair_route_in_grid, validate_route_in_grid, write_learning_artifacts,
)
from robot_guard import MAX_ROBOT_SPEED_MPS, RobotGuard


MIN_TURN_RADIUS_M = 0.65
SENSOR_TIMEOUT_S = 0.50
ROBOT_SENSOR_TIMEOUT_S = 0.25
STARTUP_TIMEOUT_S = 70.0
PLAN_PERIOD_S = 0.32
BLOCKED_TIMEOUT_S = 4.0
PROGRESS_TIMEOUT_S = 14.0
SETTLE_TIMEOUT_S = 7.0
STOPPED_SPEED_MPS = 0.025
LOOP_OVERLAP_M = 0.65
LOOP_OVERLAP_SPEED_MPS = 0.12
LOOP_OVERLAP_TIMEOUT_S = 8.0


def yaw(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def pose(transform):
    return transform.translation.x, transform.translation.y, yaw(transform.rotation)


def local_to_frame(px, py, frame_pose):
    x, y, heading = frame_pose
    c, s = math.cos(heading), math.sin(heading)
    return x + c*px - s*py, y + s*px + c*py


def frame_to_local(px, py, frame_pose):
    x, y, heading = frame_pose
    c, s = math.cos(heading), math.sin(heading)
    return c*(px - x) + s*(py - y), -s*(px - x) + c*(py - y)


def ros_path(points, frame_id, stamp):
    path = RosPath()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for index, (x, y) in enumerate(points):
        before = points[max(0, index - 1)]
        after = points[min(len(points) - 1, index + 1)]
        angle = math.atan2(after[1] - before[1], after[0] - before[0])
        sample = PoseStamped(header=path.header)
        sample.pose.position.x, sample.pose.position.y = x, y
        sample.pose.orientation.z = math.sin(angle / 2)
        sample.pose.orientation.w = math.cos(angle / 2)
        path.poses.append(sample)
    return path


class LearnedRoute:
    def __init__(self, route):
        self.points = [(x, y) for x, y, _ in route]
        self.index = 0
        self.distance = sum(math.dist(point, self.points[index - 1])
                            for index, point in enumerate(self.points))

    def local_reference(self, car_pose):
        count = len(self.points)
        search = range(self.index - 12, self.index + 30)
        self.index = min((index % count for index in search),
                         key=lambda index: math.dist(self.points[index], car_pose[:2]))
        if math.dist(self.points[self.index], car_pose[:2]) > 0.70:
            raise ValueError('Localization moved too far from the learned lap')
        result = []
        distance = 0.0
        previous = self.points[self.index]
        for offset in range(count):
            point = self.points[(self.index + offset) % count]
            distance += math.dist(point, previous)
            if distance > 3.4:
                break
            result.append(frame_to_local(*point, car_pose))
            previous = point
        return result


class LearningDriver(Node):
    def __init__(self, args):
        super().__init__('track_learning_driver')
        self.set_parameters([Parameter('use_sim_time', value=not args.real_time)])
        self.args = args
        self.guard = RobotGuard(self) if args.real_time else None
        self.sensor_timeout = ROBOT_SENSOR_TIMEOUT_S if self.guard else SENSOR_TIMEOUT_S
        self.buffer = Buffer()
        # Keep timestamped frames current while the Python planner evaluates
        # candidate paths. Each lookup still uses the selected scan timestamp.
        live_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.tf_node = Node('track_learning_transforms', enable_rosout=False,
                            start_parameter_services=False)
        self.listener = TransformListener(self.buffer, self.tf_node, qos=live_qos)
        self.tf_executor = SingleThreadedExecutor()
        self.tf_executor.add_node(self.tf_node)
        self.tf_thread = Thread(target=self.tf_executor.spin)
        self.tf_thread.start()
        self.scans = deque(maxlen=12)
        self.scan = self.grid = self.graph = None
        self.odom_stamp = 0.0
        self.frame_issue = 'waiting for the first scans'
        self.map_received = self.graph_received = 0.0
        self.map_updates = 0
        self.map_correction = None
        self.last_data_wall = time.monotonic()
        self.speed = 0.0
        self.command = self.create_publisher(TwistStamped, '/cmd_vel_nav', 1)
        self.reference_pub = self.create_publisher(
            RosPath, '/track_learning/learned_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.plan_pub = self.create_publisher(RosPath, '/track_learning/local_path', 1)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/oval/tracking_speed_limit', 1)
        self.follow = ActionClient(self, FollowPath, '/follow_path')
        self.pending = self.goal = self.result = None
        # Receive physical sensors independently of path planning so replay
        # cannot starve the next scan or odometry sample.
        inputs = self.tf_node if self.guard else self
        inputs.create_subscription(LaserScan, '/scan', self.receive_scan, live_qos)
        inputs.create_subscription(Odometry, args.odom_topic, self.receive_odometry, live_qos)
        inputs.create_subscription(OccupancyGrid, '/map', self.receive_map,
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        inputs.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization',
                                   self.receive_graph, 1)

    def receive_scan(self, scan):
        self.scans.append(scan)

    def receive_odometry(self, odometry):
        velocity = odometry.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        if self.guard and (not math.isfinite(speed) or odometry.header.frame_id != 'odom'
                           or odometry.child_frame_id not in ('base_link', 'base_footprint')):
            return
        self.speed = speed
        self.odom_stamp = Time.from_msg(odometry.header.stamp).nanoseconds * 1e-9

    def receive_map(self, grid):
        self.grid = grid
        self.map_received = time.monotonic()
        self.map_updates += 1

    def receive_graph(self, graph):
        self.graph = parse_slam_graph(graph.markers)
        self.graph_received = time.monotonic()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def frame(self, last_stamp):
        now = self.now()
        if time.monotonic() - min(self.map_received, self.graph_received) > 5.0:
            self.frame_issue = 'the occupancy grid or pose graph has not updated'
            return None
        odom_age = now - self.odom_stamp
        if not -0.05 <= odom_age <= self.sensor_timeout:
            self.frame_issue = f'odometry timestamp age is {odom_age:.3f} s'
            return None
        try:
            map_tf = self.buffer.lookup_transform('map', 'odom', Time())
        except TransformException as error:
            self.frame_issue = f'map pose transform: {error}'
            return None
        if self.guard:
            map_age = now - Time.from_msg(map_tf.header.stamp).nanoseconds * 1e-9
            if not -0.25 <= map_age <= 1.2:
                self.frame_issue = f'SLAM correction timestamp age is {map_age:.3f} s'
                return None
        map_odom = pose(map_tf.transform)
        self.frame_issue = 'no new lidar scan in the synchronized data window'
        for scan in reversed(tuple(self.scans)):
            stamp = Time.from_msg(scan.header.stamp)
            scan_time = stamp.nanoseconds * 1e-9
            if scan_time <= last_stamp or now - scan_time > self.sensor_timeout:
                break
            if scan_time > now:
                continue
            try:
                laser = pose(self.buffer.lookup_transform(
                    'base_link', scan.header.frame_id, stamp).transform)
                odometry = pose(self.buffer.lookup_transform('odom', 'base_link', stamp).transform)
            except TransformException as error:
                self.frame_issue = f'lidar/odometry transform: {error}'
                continue
            x, y = local_to_frame(*odometry[:2], map_odom)
            localized = x, y, math.remainder(odometry[2] + map_odom[2], 2*math.pi)
            self.scan = scan
            self.map_correction = map_odom
            return scan_time, laser, odometry, localized
        return None

    def scan_points(self, laser):
        angle = self.scan.angle_min
        upper = min(self.scan.range_max, 6.0)
        points = []
        for distance in self.scan.ranges:
            if self.scan.range_min <= distance <= upper:
                points.append(local_to_frame(distance*math.cos(angle), distance*math.sin(angle), laser))
            angle += self.scan.angle_increment
        return points

    def velocity(self, speed, turn=0.0):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist.linear.x = speed
        command.twist.angular.z = turn
        self.command.publish(command)

    def set_limit(self, speed):
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.speed_limit = speed
        self.speed_limit_pub.publish(limit)

    def local_path(self, laser, localized, learned):
        reference = learned.local_reference(localized) if learned else None
        if self.guard and reference:
            reference = reference[::2]
        return plan(self.scan_points(laser), reference=reference, learning=learned is None,
                    min_turn_radius=self.args.min_turn_radius,
                    beam_width=8 if self.guard else BEAM_WIDTH)

    def publish_reference(self, learned):
        self.reference_pub.publish(ros_path(
            learned.points + learned.points[:1], 'map', self.get_clock().now().to_msg()))

    def wait_for_arm(self, stop_requested, learned=None):
        """Offer checked zeros while the operator authorizes a physical lap."""
        last_stamp = checked_at = 0.0
        ready = announced = False
        deadline = time.monotonic() + 180.0
        while rclpy.ok() and not stop_requested():
            rclpy.spin_once(self, timeout_sec=0.04)
            now = self.now()
            status = self.guard.current()
            frame = self.frame(last_stamp)
            if self.guard.armed() and (not ready or now - checked_at > PLAN_PERIOD_S):
                raise ValueError('Arm only after the track-learning driver reports ready')
            if time.monotonic() >= deadline:
                raise ValueError(f'Timed out waiting for a safe arm: {self.frame_issue}')
            if frame is None or status is None or not self.follow.server_is_ready():
                continue
            last_stamp, laser, _, localized = frame
            if now - checked_at >= 0.16:
                checked_at = now
                try:
                    self.local_path(laser, localized, learned)
                except ValueError as error:
                    ready = False
                    self.frame_issue = str(error)
                    if self.guard.armed():
                        raise
                else:
                    ready = True
                if learned is not None:
                    self.publish_reference(learned)
            self.velocity(0.0)
            if ready:
                self.guard.heartbeat()
                if not announced:
                    phase = 'Learned route' if learned else 'Mapping lap'
                    print(f'{phase} ready. Enable the guarded drive to start.', flush=True)
                    announced = True
                if self.guard.armed():
                    self.last_data_wall = time.monotonic()
                    return frame
        return None

    def follow_path(self, path):
        self.check_action()
        if self.pending is None:
            request = FollowPath.Goal(path=path, controller_id='FollowPath',
                                      goal_checker_id='general_goal_checker')
            self.pending = self.follow.send_goal_async(request)

    def check_action(self):
        if self.pending is not None and self.pending.done():
            self.goal = self.pending.result()
            self.pending = None
            if not self.goal.accepted:
                raise ValueError('MPPI rejected the checked route')
            self.result = self.goal.get_result_async()
        if self.result is not None and self.result.done():
            status = self.result.result().status
            self.goal = self.result = None
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise ValueError('MPPI ended the checked route before reaching its goal')

    def cancel(self):
        if self.goal is not None:
            self.goal.cancel_goal_async()
        self.goal = self.result = None
        if self.pending is not None:
            def cancel_accepted(future):
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()

            self.pending.add_done_callback(cancel_accepted)
            self.pending = None

    def record(self, gate, first_lap, *, allow_geometric_return):
        graph, grid = self.graph, self.grid
        edge = find_loop_edge(graph, gate.start, min_distance=self.args.minimum_lap_distance)
        if edge is None and not allow_geometric_return:
            raise ValueError('Waiting for SLAM to associate the start and end of the lap')
        keyframes = keyframes_for_loop(
            graph, gate.start, edge, min_distance=self.args.minimum_lap_distance)
        route = build_closed_route(keyframes, min_turn_radius=self.args.min_turn_radius)
        info = grid.info
        origin = (info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation))
        grid_parameters = info.width, info.height, info.resolution, origin, grid.data
        if allow_geometric_return:
            checked = repair_route_in_grid(route, *grid_parameters,
                                           min_turn_radius=self.args.min_turn_radius)
        else:
            validate_route_in_grid(route, *grid_parameters)
            checked = route
        write_learning_artifacts(
            self.args.output_dir, checked, *grid_parameters, metadata={
                'closure': 'slam_loop_edge' if edge else 'geometric_map_return',
                'loop_edge': list(edge) if edge else None,
                'map_adjusted_route': checked is not route,
                'mapping_lap': first_lap,
            },
        )
        return LearnedRoute(checked), edge

    def run(self, stop_requested):
        gate = LapGate(min_distance=self.args.minimum_lap_distance)
        laps = []
        learned = None
        completed = None
        lap_started = plan_at = report_at = 0.0
        last_progress_at = last_progress = 0.0
        failed_at = snapshot_at = None
        overlap_start = None
        previous_correction = None
        gate_map_updates = 0
        failed_reason = ''
        active_command = (0.0, 0.0)
        previous_stamp = 0.0
        peak_speed = 0.0
        startup_deadline = time.monotonic() + STARTUP_TIMEOUT_S
        if self.guard:
            if self.wait_for_arm(stop_requested) is None:
                return
        while rclpy.ok() and not stop_requested():
            rclpy.spin_once(self, timeout_sec=0.04)
            now = self.now()
            if self.guard and (completed is None or overlap_start is not None):
                self.guard.require_armed()
            frame = self.frame(previous_stamp)
            if lap_started and time.monotonic() - self.last_data_wall > 2.5:
                raise ValueError(f'Sensor synchronization timed out: {self.frame_issue}')
            if frame is None or not self.follow.server_is_ready():
                if lap_started and now - previous_stamp > self.sensor_timeout:
                    self.cancel()
                    self.velocity(0.0)
                    if self.guard:
                        raise ValueError(f'Physical run lost synchronized inputs: {self.frame_issue}')
                if not lap_started and time.monotonic() > startup_deadline:
                    raise ValueError('No live SLAM map, synchronized sensors or active controller')
                continue
            self.last_data_wall = time.monotonic()
            previous_stamp, laser, odometry, localized = frame
            if self.guard and learned is not None and previous_correction is not None:
                a, b = previous_correction, self.map_correction
                if math.dist(a[:2], b[:2]) > 0.25 or abs(math.remainder(b[2] - a[2], 2*math.pi)) > 0.20:
                    raise ValueError('SLAM localization jumped during replay')
            previous_correction = self.map_correction
            if not lap_started:
                lap_started = last_progress_at = now
                print('Mapping lap started without a preloaded map.', flush=True)
            peak_speed = max(peak_speed, self.speed)
            crossed = gate.update(odometry, localized)
            if gate.distance - last_progress >= 0.15:
                last_progress_at, last_progress = now, gate.distance
            if not crossed and now - last_progress_at > PROGRESS_TIMEOUT_S:
                raise ValueError('The rover stopped making progress on the track')
            if crossed and completed is None:
                completed = {
                    'lap': len(laps) + 1, 'mode': 'mapping' if learned is None else 'learned',
                    'sim_seconds': round(now - lap_started, 3),
                    'distance_m': round(gate.distance, 3),
                    'mean_speed_mps': round(gate.distance / (now - lap_started), 3),
                    'peak_speed_mps': round(peak_speed, 3),
                }
                snapshot_at = now
                gate_map_updates = self.map_updates
                if learned is None and find_loop_edge(
                        self.graph, gate.start, min_distance=self.args.minimum_lap_distance) is None:
                    # Scan matching needs observations on both sides of the
                    # seam. Collect a short overlap at reduced speed.
                    overlap_start = odometry
                    plan_at = 0.0
                elif learned is None or len(laps) + 1 == self.args.laps:
                    self.cancel()
                    self.velocity(0.0)
                print(f"{completed['mode']} lap {completed['lap']} crossed the start gate "
                      f"after {completed['distance_m']:.2f} m.", flush=True)

            if overlap_start is not None:
                edge = find_loop_edge(
                    self.graph, gate.start, min_distance=self.args.minimum_lap_distance)
                if (edge is not None or math.dist(overlap_start[:2], odometry[:2]) >= LOOP_OVERLAP_M
                        or now - snapshot_at >= LOOP_OVERLAP_TIMEOUT_S):
                    overlap_start = None
                    snapshot_at = now
                    gate_map_updates = self.map_updates
                    active_command = (0.0, 0.0)

            if completed is not None and overlap_start is None:
                if learned is None or len(laps) + 1 == self.args.laps:
                    self.velocity(0.0)
                    if self.guard:
                        self.guard.latch()
                    if now - snapshot_at > SETTLE_TIMEOUT_S:
                        raise ValueError(f'Could not finish the lap safely: {failed_reason or "rover did not settle"}')
                    if (self.speed > STOPPED_SPEED_MPS or now - snapshot_at < 0.8
                            or (self.guard and not self.guard.stopped())):
                        continue
                if learned is None:
                    if self.map_updates == gate_map_updates:
                        continue
                    try:
                        learned, edge = self.record(
                            gate, completed, allow_geometric_return=now - snapshot_at >= 3.0)
                    except ValueError as error:
                        failed_reason = str(error)
                        self.last_data_wall = time.monotonic()
                        continue
                    self.last_data_wall = time.monotonic()
                    closure = f'SLAM loop edge {edge}' if edge else 'validated geometric return'
                    self.publish_reference(learned)
                    print(f'Saved SLAM map and a {learned.distance:.2f} m learned path '
                          f'using {closure}.', flush=True)
                laps.append(completed)
                if len(laps) == self.args.laps:
                    summary = self.args.output_dir / 'run.json'
                    summary.write_text(json.dumps({'laps': laps}, indent=2) + '\n', encoding='utf-8')
                    print('Requested mapping and learned laps completed.', flush=True)
                    return
                if self.guard and len(laps) == 1:
                    print('Mapping complete. Review the saved map and route, then reset '
                          'the stop and re-arm for replay.', flush=True)
                    armed_frame = self.wait_for_arm(stop_requested, learned)
                    if armed_frame is None:
                        return
                    previous_stamp, laser, odometry, localized = armed_frame
                    previous_correction = self.map_correction
                    now = self.now()
                gate = LapGate(min_distance=max(self.args.minimum_lap_distance, 0.72 * learned.distance))
                gate.update(odometry, localized)
                lap_started, peak_speed = now, self.speed
                last_progress_at, last_progress = now, 0.0
                completed = failed_at = None
                plan_at = 0.0
                print(f'Starting learned lap {len(laps) + 1} at up to {self.args.replay_speed:.2f} m/s.',
                      flush=True)

            if now >= plan_at:
                plan_at = now + PLAN_PERIOD_S
                try:
                    local_path = self.local_path(laser, localized, learned)
                    if self.guard and self.now() - previous_stamp > self.sensor_timeout:
                        raise ValueError('Physical path check exceeded the fresh sensor window')
                    stamp = self.get_clock().now().to_msg()
                    path = ros_path([local_to_frame(x, y, odometry) for x, y in local_path],
                                    'odom', stamp)
                    self.plan_pub.publish(path)
                    if learned is None:
                        target = local_path[1]
                        turn_limit = 1 / self.args.min_turn_radius
                        curvature = max(-turn_limit, min(turn_limit,
                                        2 * target[1] / (target[0]**2 + target[1]**2)))
                        speed = self.args.learning_speed
                        if overlap_start is not None:
                            speed = min(speed, LOOP_OVERLAP_SPEED_MPS)
                        active_command = speed, speed * curvature
                    else:
                        self.publish_reference(learned)
                        self.set_limit(self.args.replay_speed)
                        self.follow_path(path)
                    failed_at = None
                except ValueError as error:
                    if self.guard:
                        raise
                    if failed_at is None:
                        failed_at = now
                        print(f'Holding for a checked path: {error}', flush=True)
                    if now - failed_at >= BLOCKED_TIMEOUT_S:
                        raise
                    self.cancel()
                    active_command = (0.0, 0.0)
                    self.velocity(0.0)
            if learned is None:
                self.velocity(*active_command)
            if self.guard:
                self.guard.heartbeat()
            if now >= report_at:
                report_at = now + 5.0
                mode = 'Mapping' if learned is None else 'Learned'
                start_gap = math.dist(gate.start[:2], localized[:2])
                print(f'{mode} lap {len(laps) + 1}: {gate.distance:.1f} m traveled, '
                      f'{self.speed:.2f} m/s, {start_gap:.2f} m from start, '
                      f'heading change {gate.turn:.2f} rad.', flush=True)

    def stop(self):
        if self.guard:
            self.guard.latch()
        self.cancel()
        deadline = time.monotonic() + 0.65
        publish_at = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if time.monotonic() >= publish_at:
                if self.guard:
                    self.guard.latch()
                self.velocity(0.0)
                publish_at = time.monotonic() + 0.05
            rclpy.spin_once(self, timeout_sec=0.04)

    def close(self):
        self.tf_executor.shutdown()
        self.tf_thread.join()
        self.listener.unregister()
        self.tf_node.destroy_node()
        self.destroy_node()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path,
                        help='New directory for the SLAM map, learned route and timing')
    parser.add_argument('--laps', type=int, default=2, help='Total laps including the mapping lap')
    parser.add_argument('--real-time', action='store_true',
                        help='Use the existing guarded Jetson drive and wall-clock sensors')
    parser.add_argument('--min-turn-radius', type=float,
                        help='Measured larger of left and right turning radii, in metres')
    parser.add_argument('--odom-topic', help='Odometry topic used by the launch and drive guard')
    parser.add_argument('--learning-speed', type=float)
    parser.add_argument('--replay-speed', type=float)
    parser.add_argument('--minimum-lap-distance', type=float, default=8.0)
    args = parser.parse_args(argv)
    if args.output_dir.exists():
        parser.error('--output-dir must be a new directory')
    if args.real_time and args.min_turn_radius is None:
        parser.error('--real-time requires --min-turn-radius measured on the robot')
    if args.min_turn_radius is None:
        args.min_turn_radius = MIN_TURN_RADIUS_M
    if not math.isfinite(args.min_turn_radius) or args.min_turn_radius < MIN_TURN_RADIUS_M:
        parser.error('--min-turn-radius must be finite and at least 0.65 m')
    learning_cap, replay_cap = (0.10, MAX_ROBOT_SPEED_MPS) if args.real_time else (0.42, 0.42)
    if args.learning_speed is None:
        args.learning_speed = 0.10 if args.real_time else 0.20
    if args.replay_speed is None:
        args.replay_speed = replay_cap
    if (args.laps < 1 or not 0.0 < args.learning_speed <= learning_cap
            or not args.learning_speed < args.replay_speed <= replay_cap):
        parser.error(f'Choose at least one lap and a mapping speed up to {learning_cap:g} m/s '
                     f'below a replay speed up to {replay_cap:g} m/s')
    if args.odom_topic is None:
        args.odom_topic = '/odom' if args.real_time else '/demo/odom'
    try:
        validate_full_topic_name(args.odom_topic)
    except InvalidTopicNameException as error:
        parser.error(str(error))
    if not math.isfinite(args.minimum_lap_distance) or args.minimum_lap_distance <= 0:
        parser.error('--minimum-lap-distance must be positive and finite')

    return args


def main():
    args = parse_args()
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    node = LearningDriver(args)
    try:
        node.run(lambda: stopping)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    finally:
        try:
            node.stop()
        finally:
            node.close()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
