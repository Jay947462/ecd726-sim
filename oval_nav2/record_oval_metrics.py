# Copyright 2026 ECD 726 project contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Record repeatable sim-time measurements for the ECD726 oval track."""

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from statistics import correlation, mean, StatisticsError
from xml.etree import ElementTree

from oval_lane import LAP, project, world, wrap


ROOT = Path(__file__).resolve().parents[1]
STEERING_JOINTS = ('steer_l_steer_joint', 'steer_r_steer_joint')
SPEED_START_MPS = 0.01
FINISH_TOLERANCE_M = 0.25  # general_goal_checker.xy_goal_tolerance
SENSOR_MAX_AGE_SEC = 0.25
MAX_STEERING_LAG_SEC = 1.0


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    yaw: float
    half_length: float
    half_width: float

    def local(self, x, y):
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return tuple((self.x + c * x - s * y, self.y + s * x + c * y)
                     for x in (-self.half_length, self.half_length)
                     for y in (-self.half_width, self.half_width))

    def local_distance(self, x, y):
        return math.hypot(max(0.0, abs(x) - self.half_length),
                          max(0.0, abs(y) - self.half_width))

    def separates(self, corners):
        xs, ys = zip(*corners)
        return (min(xs) > self.half_length or max(xs) < -self.half_length
                or min(ys) > self.half_width or max(ys) < -self.half_width)

    def distance_to(self, other, corners, other_corners):
        own = tuple(self.local(*point) for point in other_corners)
        theirs = tuple(other.local(*point) for point in corners)
        if not (self.separates(own) or other.separates(theirs)):
            return 0.0
        return min(min(self.local_distance(*point) for point in own),
                   min(other.local_distance(*point) for point in theirs))


def planar_pose(element):
    x, y, _, _, _, yaw = map(float, element.findtext('pose', '0 0 0 0 0 0').split())
    return x, y, yaw


def compose(parent, child):
    x, y, yaw = parent
    dx, dy, dyaw = child
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, yaw + dyaw


def obstacles(world_file):
    result = []
    for model in ElementTree.parse(world_file).findall('./world/model'):
        name = model.get('name')
        is_cone = name.startswith('traffic_cone_')
        if not is_cone and name not in ('inner_boundary', 'outer_boundary'):
            continue
        for link in model.findall('link'):
            position = compose(planar_pose(model), planar_pose(link))
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size is None:
                    continue  # Each cone's base box encloses its cylindrical tiers in XY.
                length, width, _ = map(float, size.split())
                pose = compose(position, planar_pose(collision))
                box = Box(*pose, length / 2.0, width / 2.0)
                result.append((box, box.corners(), math.hypot(length, width) / 2.0, is_cone))
    if not result:
        raise ValueError(f'No oval walls or cone bases found in {world_file}')
    return result


def robot_dimensions(model_file):
    from yaml import safe_load

    namespace = '{http://www.ros.org/wiki/xacro}'
    names = ('base_length', 'base_width', 'wheel_ygap',
             'wheel_radius', 'wheel_width', 'max_steer')
    sizes = {prop.get('name'): float(prop.get('value'))
             for prop in ElementTree.parse(model_file).findall(f'{namespace}property')
             if prop.get('name') in names}
    with (ROOT / 'config/rover_sim.yaml').open(encoding='utf-8') as stream:
        sizes['wheel_xoff'] = float(safe_load(stream)['wheelbase_m']) / 2.0
    return sizes


def robot_envelope(pose, sizes, angles):
    # The chassis box encloses the horizontal projections of both front sensors.
    # Enclose the measured front-wheel projections, plus the straight rear wheels.
    radius = sizes['wheel_radius']
    half_tread = sizes['wheel_width'] / 2.0
    half_length = sizes['base_length'] / 2.0
    wheel_center = sizes['base_width'] / 2.0 + sizes['wheel_ygap']
    left_extent = right_extent = half_tread
    if angles is None:
        angles = (sizes['max_steer'],) * 2
    for index, angle in enumerate(angles):
        c, s = abs(math.cos(angle)), abs(math.sin(angle))
        half_length = max(half_length, sizes['wheel_xoff'] + radius * c + half_tread * s)
        wheel_extent = radius * s + half_tread * c
        if index == 0:
            left_extent = max(left_extent, wheel_extent)
        else:
            right_extent = max(right_extent, wheel_extent)
    center = compose(pose, (0.0, (left_extent - right_extent) / 2.0, 0.0))
    return Box(*center, half_length, wheel_center + (left_extent + right_extent) / 2.0)


def clearances(robot, obstacles_to_check):
    distances = {True: math.inf, False: math.inf}
    corners = robot.corners()
    radius = math.hypot(robot.half_length, robot.half_width)
    for obstacle, obstacle_corners, obstacle_radius, is_cone in obstacles_to_check:
        center_distance = math.hypot(robot.x - obstacle.x, robot.y - obstacle.y)
        lower_bound = center_distance - radius - obstacle_radius
        if lower_bound < distances[is_cone]:
            distances[is_cone] = min(distances[is_cone],
                                     robot.distance_to(obstacle, corners, obstacle_corners))
    return tuple(distance if math.isfinite(distance) else None
                 for distance in (distances[True], distances[False]))


def distance_to_path(x, y, points):
    distance = math.inf
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        if length_squared > 0.0:
            fraction = min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / length_squared))
            distance = min(distance, math.hypot(x - ax - fraction * dx, y - ay - fraction * dy))
    return distance if math.isfinite(distance) else None


def yaw(rotation):
    return math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def gazebo_target_steering(vx, wz, sizes):
    # Match Harmonic's geometric conversion from Twist to the two steering angles.
    if abs(vx) < SPEED_START_MPS:
        return None
    if abs(wz) < 0.001:  # AckermannSteering::UpdateVelocity straight-ahead threshold.
        return 0.0
    wheelbase = 2.0 * sizes['wheel_xoff']
    half_kingpin_width = sizes['base_width'] / 2.0 + sizes['wheel_ygap']
    joint_limit = sizes['max_steer']
    radius = vx / wz
    radius = math.copysign(max(abs(radius), wheelbase / math.sin(joint_limit)), radius)
    angles = [math.atan(wheelbase / (radius + side * half_kingpin_width)) for side in (-1, 1)]
    return mean(max(-joint_limit, min(joint_limit, angle)) for angle in angles)


def statistics(rows, key):
    values = [row[key] for row in rows if row[key] is not None and math.isfinite(row[key])]
    if not values:
        return None
    return {
        'samples': len(values),
        'mean': mean(values),
        'rms': math.sqrt(mean(x * x for x in values)),
        'min': min(values),
        'max': max(values),
    }


def steering_lag(rows, sample_hz):
    """Estimate response lag via correlation, including plant and sampling latency."""
    best = None
    for samples in range(round(MAX_STEERING_LAG_SEC * sample_hz) + 1):
        lag = samples / sample_hz
        pairs = [(a['target_steering_rad'], b['mean_steering_rad'])
                 for a, b in zip(rows, rows[samples:])
                 if a['target_steering_rad'] is not None and b['mean_steering_rad'] is not None
                 and abs(b['elapsed_sec'] - a['elapsed_sec'] - lag) <= 0.5 / sample_hz]
        if len(pairs) < sample_hz:
            continue
        try:
            score = correlation(*zip(*pairs))
        except StatisticsError:
            continue
        if best is None or score > best['correlation']:
            best = {'lag_sec': lag, 'correlation': score, 'samples': len(pairs)}
    return best


def summarize(rows, args, completed_laps, overlaps, contact_events, reason):
    return {
        'stop_reason': reason,
        'requested_laps': args.laps,
        'laps_completed': completed_laps,
        'samples': len(rows),
        'elapsed_sim_seconds': rows[-1]['elapsed_sec'] if rows else None,
        'progress_m': rows[-1]['progress_m'] if rows else None,
        'track_pose_source': args.world_pose_topic or 'map -> base_link TF',
        'reference_path_topic': args.reference_topic,
        'wheel_odometry_speed_mps': statistics(rows, 'speed_mps'),
        'ground_truth_speed_mps': statistics(rows, 'ground_truth_speed_mps'),
        'track_cross_track_error_m': statistics(rows, 'track_error_m'),
        'reference_cross_track_error_m': statistics(rows, 'reference_error_m'),
        'ground_truth_reference_cross_track_error_m': statistics(
            rows, 'ground_truth_reference_error_m'),
        'localization_position_error_m': statistics(rows, 'localization_error_m'),
        'steering_angle_left_rad': statistics(rows, 'steering_left_rad'),
        'steering_angle_right_rad': statistics(rows, 'steering_right_rad'),
        'steering_rate_left_rad_s': statistics(rows, 'steering_rate_left_rad_s'),
        'steering_rate_right_rad_s': statistics(rows, 'steering_rate_right_rad_s'),
        'steering_rate_rms_rad_s': (statistics(rows, 'steering_rate_rms_rad_s') or {}).get('rms'),
        'steering_tracking_error_rad': statistics(rows, 'steering_tracking_error_rad'),
        'estimated_command_to_steering_response': steering_lag(rows, args.sample_hz),
        'command_header_to_recorder_receive_sec': statistics(rows, 'command_header_age_sec'),
        'minimum_obstacle_clearance_m': (statistics(rows, 'clearance_m') or {}).get('min'),
        'minimum_cone_clearance_m': (statistics(rows, 'cone_clearance_m') or {}).get('min'),
        'minimum_wall_clearance_m': (statistics(rows, 'wall_clearance_m') or {}).get('min'),
        'clearance_method': ('rotated simulated chassis and wheel envelope '
                             'versus SDF box projections'),
        'estimated_footprint_overlap_events': overlaps if rows else None,
        'collision_count': contact_events,
        'collision_contact_topic': args.contacts_topic,
    }


def sample_bag(node, args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def clock(message):
        node.bag_time = stamp_seconds(message.clock)

    def transforms(message, static):
        for transform in message.transforms:
            if static:
                node.buffer.set_transform_static(transform, 'bag')
            else:
                node.buffer.set_transform(transform, 'bag')

    callbacks = {
        '/clock': clock,
        '/tf': lambda msg: transforms(msg, False),
        '/tf_static': lambda msg: transforms(msg, True),
        '/demo/odom': node.on_odometry,
        '/joint_states': node.on_joints,
        '/demo/cmd_vel': node.on_command,
        args.reference_topic: node.on_reference,
    }
    if args.world_pose_topic:
        callbacks[args.world_pose_topic] = (
            node.on_world_pose if args.world_pose_type == 'pose-array'
            else node.on_world_transforms
        )
    if args.contacts_topic:
        callbacks[args.contacts_topic] = node.on_contacts

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {entry.name: get_message(entry.type) for entry in reader.get_all_topics_and_types()
             if entry.name in callbacks}
    if '/clock' not in types:
        raise ValueError('MCAP recording must contain the simulation /clock topic')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    next_sample = 0.0
    period = 1.0 / args.sample_hz
    while reader.has_next() and node.stop_reason == 'interrupted':
        topic, data, _ = reader.read_next()
        callbacks[topic](deserialize_message(data, types[topic]))
        if topic == '/clock' and node.bag_time >= next_sample:
            node.sample()
            next_sample = node.bag_time + period
    if node.stop_reason == 'interrupted':
        node.stop_reason = 'end of available bag data'


def record(args):
    import rclpy
    from geometry_msgs.msg import PoseArray, TwistStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException, TransformListener

    sizes = robot_dimensions(ROOT / 'src/components/shared.sdf.xacro')
    boxes = obstacles(args.world_file)

    class Recorder(Node):
        def __init__(self):
            super().__init__(
                'oval_metrics', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.buffer = Buffer()
            self.bag_time = 0.0 if args.bag else None
            self.rows = []
            self.start = None
            self.odometry = self.joints = self.command = self.pose = None
            self.ground_truth_speed = None
            self.reference = ()
            self.completed_laps = self.overlaps = self.contacts = 0
            self.overlapping = False
            self.last_contact = None
            self.contacts_seen = False
            self.stop_reason = 'interrupted'
            self.progress = self.station = None
            if args.bag:
                return
            self.listener = TransformListener(self.buffer, self)
            self.subscribe(Odometry, '/demo/odom', self.on_odometry)
            self.subscribe(JointState, '/joint_states', self.on_joints)
            self.subscribe(TwistStamped, '/demo/cmd_vel', self.on_command)
            self.subscribe(RosPath, args.reference_topic, self.on_reference)
            if args.world_pose_topic:
                if args.world_pose_type == 'pose-array':
                    self.subscribe(PoseArray, args.world_pose_topic, self.on_world_pose)
                else:
                    self.subscribe(TFMessage, args.world_pose_topic, self.on_world_transforms)
            if args.contacts_topic:
                from ros_gz_interfaces.msg import Contacts
                self.subscribe(Contacts, args.contacts_topic, self.on_contacts)
            self.create_timer(1.0 / args.sample_hz, self.sample)

        def now(self):
            return (self.bag_time if self.bag_time is not None
                    else self.get_clock().now().nanoseconds * 1e-9)

        def subscribe(self, kind, topic, callback):
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)

        def on_odometry(self, msg):
            speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            if math.isfinite(speed):
                self.odometry = stamp_seconds(msg.header.stamp), speed

        def on_joints(self, msg):
            if any(name not in msg.name for name in STEERING_JOINTS):
                return
            indices = tuple(msg.name.index(name) for name in STEERING_JOINTS)
            if max(indices) >= len(msg.position):
                return
            angles = tuple(msg.position[index] for index in indices)
            if not all(map(math.isfinite, angles)):
                return
            timestamp = stamp_seconds(msg.header.stamp)
            rates = None
            if max(indices) < len(msg.velocity):
                rates = tuple(msg.velocity[index] for index in indices)
                if not all(map(math.isfinite, rates)):
                    rates = None
            elif self.joints and timestamp > self.joints[0]:
                dt = timestamp - self.joints[0]
                rates = tuple(
                    (angle - previous) / dt for angle, previous in zip(angles, self.joints[1]))
            self.joints = timestamp, angles, rates

        def on_command(self, msg):
            vx, wz = msg.twist.linear.x, msg.twist.angular.z
            if math.isfinite(vx) and math.isfinite(wz):
                self.command = self.now(), stamp_seconds(msg.header.stamp), vx, wz

        def on_reference(self, msg):
            if msg.header.frame_id == 'map':
                self.reference = tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses)

        def on_world_pose(self, msg):
            if msg.poses:
                # On this world the rover is the only dynamic model. SceneBroadcaster
                # publishes model poses before links and the PoseArray bridge drops names.
                pose = msg.poses[0]
                timestamp = stamp_seconds(msg.header.stamp) or self.now()
                self.set_world_pose(
                    timestamp, (pose.position.x, pose.position.y, yaw(pose.orientation)))

        def on_world_transforms(self, msg):
            if not msg.transforms:
                return
            pose = next((tf for tf in msg.transforms if tf.child_frame_id == 'ecd726_bot'),
                        msg.transforms[0])
            t = pose.transform.translation
            timestamp = stamp_seconds(pose.header.stamp) or self.now()
            self.set_world_pose(timestamp, (t.x, t.y, yaw(pose.transform.rotation)))

        def set_world_pose(self, timestamp, pose):
            if not all(map(math.isfinite, pose)):
                return
            if self.pose and timestamp > self.pose[0]:
                dt = timestamp - self.pose[0]
                self.ground_truth_speed = math.dist(pose[:2], self.pose[1][:2]) / dt
            self.pose = timestamp, pose

        def on_contacts(self, msg):
            self.contacts_seen = True
            if self.start is None:
                return
            for contact in msg.contacts:
                names = (contact.collision1.name, contact.collision2.name)
                if any('ecd726_bot' in name for name in names) and any(
                        model in name for name in names
                        for model in ('traffic_cone_', 'inner_boundary', 'outer_boundary')):
                    now = self.now()
                    if self.last_contact is None or now - self.last_contact > SENSOR_MAX_AGE_SEC:
                        self.contacts += 1
                    self.last_contact = now
                    break

        def map_pose(self):
            try:
                stamped = self.buffer.lookup_transform('map', 'base_link', Time())
            except TransformException:
                return None
            if self.now() - stamp_seconds(stamped.header.stamp) > SENSOR_MAX_AGE_SEC:
                return None
            tf = stamped.transform
            return tf.translation.x, tf.translation.y, yaw(tf.rotation)

        def sample(self):
            now = self.now()
            map_pose = self.map_pose()
            track_pose = (self.pose[1] if self.pose and now - self.pose[0] <= SENSOR_MAX_AGE_SEC
                          else None)
            if not args.world_pose_topic:
                track_pose = map_pose
            if track_pose is None:
                return
            command = (self.command
                       if self.command and now - self.command[0] <= SENSOR_MAX_AGE_SEC else None)
            if self.start is None:
                if not command or abs(command[2]) < SPEED_START_MPS:
                    return
                self.start = now
                self.get_logger().info('Recording from the first fresh motion command')

            x, y, heading = track_pose
            station, offset = project(x, y)
            self.progress = (wrap(station) if self.station is None else
                             self.progress + wrap(station - self.station))
            self.station = station
            next_lap = (self.completed_laps + 1) * LAP
            if (self.progress >= next_lap - FINISH_TOLERANCE_M
                    and math.dist(world(0.0, 0.0)[:2], (x, y)) <= FINISH_TOLERANCE_M):
                self.completed_laps += 1
                self.get_logger().info(
                    f'Completed lap {self.completed_laps} at {now - self.start:.2f} sim seconds')

            angles = rates = None
            if self.joints and now - self.joints[0] <= SENSOR_MAX_AGE_SEC:
                _, angles, rates = self.joints
            envelope = robot_envelope(track_pose, sizes, angles)
            cone_distance, wall_distance = clearances(envelope, boxes)
            clearance = min(
                distance for distance in (cone_distance, wall_distance) if distance is not None)
            overlap = clearance == 0.0
            self.overlaps += int(overlap and not self.overlapping)
            self.overlapping = overlap
            speed = None
            if self.odometry and now - self.odometry[0] <= SENSOR_MAX_AGE_SEC:
                speed = self.odometry[1]
            command_age = command_vx = command_wz = target_angle = steering_error = None
            measured_angle = mean(angles) if angles else None
            if command:
                rx, stamp, command_vx, command_wz = command
                if 0.0 < stamp <= rx:
                    command_age = rx - stamp
                target_angle = gazebo_target_steering(command_vx, command_wz, sizes)
                if target_angle is not None and measured_angle is not None:
                    steering_error = abs(target_angle - measured_angle)
            reference_error = distance_to_path(*map_pose[:2], self.reference) if map_pose else None
            ground_truth_reference_error = (distance_to_path(x, y, self.reference)
                                            if args.world_pose_topic else None)
            localization_error = (math.dist(map_pose[:2], (x, y))
                                  if map_pose and args.world_pose_topic else None)
            self.rows.append({
                'sim_time_sec': now,
                'elapsed_sec': now - self.start,
                'x_m': x,
                'y_m': y,
                'heading_rad': heading,
                'progress_m': self.progress,
                'laps': self.completed_laps,
                'track_error_m': abs(offset),
                'reference_error_m': reference_error,
                'ground_truth_reference_error_m': ground_truth_reference_error,
                'localization_error_m': localization_error,
                'speed_mps': speed,
                'ground_truth_speed_mps': self.ground_truth_speed,
                'command_vx_mps': command_vx,
                'command_wz_rad_s': command_wz,
                'command_header_age_sec': command_age,
                'target_steering_rad': target_angle,
                'mean_steering_rad': measured_angle,
                'steering_tracking_error_rad': steering_error,
                'steering_left_rad': angles[0] if angles else None,
                'steering_right_rad': angles[1] if angles else None,
                'steering_rate_left_rad_s': rates[0] if rates else None,
                'steering_rate_right_rad_s': rates[1] if rates else None,
                'steering_rate_rms_rad_s': math.hypot(*rates) / math.sqrt(2.0) if rates else None,
                'clearance_m': clearance,
                'cone_clearance_m': cone_distance,
                'wall_clearance_m': wall_distance,
                'footprint_overlap': overlap,
            })
            if self.completed_laps >= args.laps:
                self.stop_reason = 'laps completed'
            elif args.distance_m and self.progress >= args.distance_m:
                self.stop_reason = 'distance limit'
            elif args.duration_sec and now - self.start >= args.duration_sec:
                self.stop_reason = 'sim time limit'

    rclpy.init(args=[])
    node = Recorder()
    print(f'Recorder ready. Requested laps: {args.laps}.', flush=True)
    try:
        if args.bag:
            sample_bag(node, args)
        else:
            while rclpy.ok() and node.stop_reason == 'interrupted':
                rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
        csv_path = Path(f'{args.output_prefix}.csv')
        json_path = Path(f'{args.output_prefix}.json')
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            if node.rows:
                writer = csv.DictWriter(stream, fieldnames=node.rows[0].keys())
                writer.writeheader()
                writer.writerows(node.rows)
        contacts = node.contacts if node.contacts_seen else None
        summary = summarize(
            node.rows, args, node.completed_laps, node.overlaps, contacts, node.stop_reason)
        json_path.write_text(
            json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        print(
            f'Wrote {len(node.rows)} samples to {csv_path} and summary to {json_path}.',
            flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output-prefix', required=True, type=Path, help='Prefix for .csv and .json files')
    parser.add_argument('--laps', type=int, default=3)
    parser.add_argument(
        '--duration-sec', type=float, help='Optional sim-time limit after motion starts')
    parser.add_argument('--distance-m', type=float,
                        help='Optional distance along the track for matched comparisons')
    parser.add_argument(
        '--bag', type=Path, help='Read an MCAP rosbag with /clock instead of subscribing live')
    parser.add_argument('--sample-hz', type=float, default=20.0)
    parser.add_argument('--reference-topic', default='/oval/reference_path')
    parser.add_argument('--world-file', type=Path, default=ROOT / 'world/ecd726_oval_track.sdf')
    parser.add_argument(
        '--world-pose-topic', help='Pose bridge for the rover, the sole dynamic model in the oval')
    parser.add_argument(
        '--world-pose-type', choices=('pose-array', 'tf-message'), default='pose-array')
    parser.add_argument(
        '--contacts-topic', help='Optional ros_gz_interfaces/msg/Contacts collision evidence')
    args = parser.parse_args()
    if args.laps <= 0 or not math.isfinite(args.sample_hz) or args.sample_hz <= 0:
        parser.error('lap count and sample frequency must be positive')
    if args.duration_sec is not None and (
            not math.isfinite(args.duration_sec) or args.duration_sec <= 0):
        parser.error('simulation duration must be positive')
    if args.distance_m is not None and (
            not math.isfinite(args.distance_m) or args.distance_m <= 0):
        parser.error('comparison distance must be positive')
    if args.world_pose_type == 'tf-message' and not args.world_pose_topic:
        parser.error('--world-pose-type tf-message requires --world-pose-topic')
    record(args)


if __name__ == '__main__':
    main()
