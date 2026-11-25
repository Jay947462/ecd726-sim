"""Gate wall-clock Nav2 commands before the existing VESC drive guards."""

import json
import math
import time
import yaml

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Header, String
from std_srvs.srv import SetBool, Trigger


DEFAULTS = {
    'wheelbase_m': 0.324,
    'erpm_per_mps': 0.0,
    'left_angle_rad': 0.0,
    'right_angle_rad': 0.0,
    'max_speed_mps': 0.15,
    'overspeed_margin_mps': 0.05,
    'command_timeout_s': 0.35,
    'scan_timeout_s': 0.30,
    'odom_timeout_s': 0.30,
    'driver_timeout_s': 0.80,
    'guard_status_timeout_s': 1.25,
    'odom_topic': '/odom',
}


def validate_parameters(params):
    for name, default in DEFAULTS.items():
        if not isinstance(params[name], type(default)):
            raise ValueError(f'{name} requires a {type(default).__name__} value')
        if isinstance(default, float) and (
                not math.isfinite(params[name]) or params[name] <= 0.0):
            raise ValueError(f'Set a measured or positive value for {name}')
        if name.endswith('_timeout_s') and params[name] > 1.5:
            raise ValueError(f'{name} must be at most 1.5 seconds')
    if max(params['left_angle_rad'], params['right_angle_rad']) >= math.pi / 2:
        raise ValueError('Steering limits must be effective wheel angles in radians')
    if params['max_speed_mps'] > 0.20 or params['overspeed_margin_mps'] > 0.10:
        raise ValueError('First-test speed limit must be at most 0.20 m/s, with a margin at most 0.10 m/s')
    if not math.isfinite(params['max_speed_mps'] * params['erpm_per_mps']):
        raise ValueError('Speed calibration produces an invalid ERPM limit')
    if not params['odom_topic'].startswith('/'):
        raise ValueError('odom_topic must be an absolute ROS topic')


def load_autonomy_config(path):
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    try:
        overrides = document['autonomy_node']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise ValueError('Expected autonomy_node.ros__parameters in the calibration file') from error
    if not isinstance(overrides, dict) or overrides.keys() - DEFAULTS.keys():
        raise ValueError('Unknown autonomy parameters in the calibration file')
    params = {**DEFAULTS, **overrides}
    validate_parameters(params)
    return params


class AutonomyNode(Node):
    def __init__(self):
        super().__init__('autonomy_node')

        descriptor = ParameterDescriptor(read_only=True)
        params = {name: self.declare_parameter(name, value, descriptor).value
                  for name, value in DEFAULTS.items()}
        validate_parameters(params)
        self.wheelbase = params['wheelbase_m']
        self.erpm_per_mps = params['erpm_per_mps']
        self.left_angle = params['left_angle_rad']
        self.right_angle = params['right_angle_rad']
        self.speed_limit = params['max_speed_mps']
        self.overspeed_margin = params['overspeed_margin_mps']
        self.timeouts = {
            'command': params['command_timeout_s'],
            'scan': params['scan_timeout_s'],
            'odometry': params['odom_timeout_s'],
            'driver': params['driver_timeout_s'],
            'drive guard': params['guard_status_timeout_s'],
            'steering guard': params['guard_status_timeout_s'],
        }
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.samples = {}
        self.target = (0.0, 0.0)
        self.armed_after = None
        self.stop_input = False
        self.stop_latched = False
        self.reason = 'disabled'

        self.speed_pub = self.create_publisher(Float64, '/autonomy/guarded_erpm', 1)
        self.steer_pub = self.create_publisher(Float64, '/autonomy/guarded_steering', 1)
        self.stop_pub = self.create_publisher(Bool, '/drive/stop', 1)
        self.status_pub = self.create_publisher(String, '/autonomy/status', 1)
        self.create_subscription(Bool, '/drive/stop', self.stop, 1)
        self.create_subscription(TwistStamped, '/oval/cmd_vel_checked', self.command, 1)
        self.create_subscription(Header, '/oval/driver_alive', self.heartbeat, 1)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, params['odom_topic'], self.odom, qos_profile_sensor_data)
        self.create_subscription(String, '/drive/status', self.drive_status, 1)
        self.create_subscription(String, '/steering/status', self.steering_status, 1)
        self.create_service(SetBool, '/autonomy/enable', self.enable)
        self.create_service(Trigger, '/autonomy/reset_stop', self.reset_stop)
        self.create_timer(0.02, self.tick, clock=self.steady_clock)
        self.create_timer(0.5, self.status, clock=self.steady_clock)

    def stale(self, source, now, wall):
        received, stamp = self.samples.get(source, (None, None))
        timeout = self.timeouts[source]
        return (received is None or now - received >= timeout
                or (stamp is not None and wall - stamp >= timeout))

    def missing_input(self, now, wall):
        return next((source for source in self.timeouts if self.stale(source, now, wall)), None)

    def note(self, source, header=None):
        now = time.monotonic()
        wall = self.wall_clock.now().nanoseconds * 1e-9
        if self.armed_after is not None and self.stale(source, now, wall):
            self.trip(f'{source} timed out')
        stamp = None
        if header is not None:
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            previous = self.samples.get(source, (None, None))[1]
            if not -0.05 <= wall - stamp < self.timeouts[source]:
                return False
            if previous is not None and stamp <= previous:
                return False
        self.samples[source] = now, stamp
        return True

    def command(self, msg):
        twist = msg.twist
        v, yaw_rate = twist.linear.x, twist.angular.z
        if not (math.isfinite(v) and math.isfinite(yaw_rate) and twist.linear.y == 0.0):
            if self.armed_after is not None:
                self.trip('invalid command')
            return
        if not self.note('command', msg.header):
            # A duplicate timestamp must not keep the watchdog alive. A stop
            # at that timestamp must still win over the previous request.
            if v <= 0.0:
                self.target = (0.0, 0.0)
                self.tick()
            return
        if v <= 0.0:
            self.target = (0.0, 0.0)
        else:
            angle = math.atan2(self.wheelbase * yaw_rate, v)
            limit = self.left_angle if angle >= 0.0 else self.right_angle
            self.target = (min(v, self.speed_limit) * self.erpm_per_mps,
                           max(-1.0, min(1.0, angle / limit)))
        self.tick()

    def heartbeat(self, msg):
        if msg.frame_id == 'map':
            self.note('driver', msg)

    def scan(self, msg):
        if (msg.header.frame_id and math.isfinite(msg.angle_increment)
                and msg.angle_increment != 0.0 and 0.0 <= msg.range_min < msg.range_max):
            # The live oval planner also requires eight usable returns.
            returns = (r for r in msg.ranges if msg.range_min <= r <= min(msg.range_max, 6.0))
            if sum(1 for _ in zip(range(8), returns)) == 8:
                self.note('scan', msg.header)

    def odom(self, msg):
        velocity = msg.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        if (msg.header.frame_id == 'odom' and msg.child_frame_id in ('base_link', 'base_footprint')
                and math.isfinite(speed) and self.note('odometry', msg.header)
                and speed > self.speed_limit + self.overspeed_margin):
            self.samples.pop('odometry', None)
            if self.armed_after is not None:
                self.trip('measured overspeed')

    def drive_status(self, msg):
        self.guard_status('drive guard', msg)

    def steering_status(self, msg):
        self.guard_status('steering guard', msg)

    def guard_status(self, source, msg):
        try:
            status = json.loads(msg.data)
            ready = status['enabled'] is True and status['stop_latched'] is False
        except (ValueError, TypeError, KeyError):
            ready = False
        if ready:
            self.note(source)
        else:
            self.samples.pop(source, None)
            if self.armed_after is not None:
                self.trip(f'{source} disabled')

    def disarm(self, reason):
        self.armed_after = None
        self.target = (0.0, 0.0)
        self.reason = reason
        self.publish(0.0, 0.0)

    def trip(self, reason):
        self.stop_latched = self.stop_input = True
        self.disarm(reason)
        self.stop_pub.publish(Bool(data=True))
        self.status()
        self.get_logger().warn(f'Autonomy stop latched: {reason}')

    def stop(self, msg):
        self.stop_input = msg.data
        if msg.data:
            if not self.stop_latched:
                self.disarm('software stop')
            self.stop_latched = True
            self.status()

    def reset_stop(self, _request, response):
        response.success = not self.stop_input
        if response.success:
            self.stop_latched = False
            self.disarm('disabled')
            self.status()
        response.message = ('Stop reset; autonomy remains disabled' if response.success
                            else 'Stop input is still active')
        return response

    def enable(self, request, response):
        if not request.data:
            self.disarm('disabled')
            response.success, response.message = True, 'Autonomy disabled'
        elif self.stop_latched:
            response.success, response.message = False, 'Reset the latched stop first'
        else:
            wall = self.wall_clock.now().nanoseconds * 1e-9
            missing = self.missing_input(time.monotonic(), wall)
            response.success = missing is None
            if missing:
                response.message = f'Fresh {missing} required'
            else:
                if self.armed_after is None:
                    self.armed_after = wall
                    self.reason = 'enabled'
                response.message = 'Autonomy enabled; waiting for a new checked command'
        self.status()
        return response

    def tick(self):
        if self.armed_after is None:
            self.publish(0.0, 0.0)
            return
        wall = self.wall_clock.now().nanoseconds * 1e-9
        missing = self.missing_input(time.monotonic(), wall)
        if missing:
            self.trip(f'{missing} timed out')
            return
        if self.samples['command'][1] > self.armed_after:
            self.publish(*self.target)

    def publish(self, erpm, steering):
        self.speed_pub.publish(Float64(data=erpm))
        self.steer_pub.publish(Float64(data=steering))

    def status(self):
        self.status_pub.publish(String(data=json.dumps({
            'enabled': self.armed_after is not None,
            'stop_latched': self.stop_latched,
            'state': self.reason,
            'max_speed_mps': self.speed_limit,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutonomyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.disarm('shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
