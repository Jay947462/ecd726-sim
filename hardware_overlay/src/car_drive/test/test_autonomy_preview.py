"""Check installed autonomy and VESC guards over ROS without a USB driver."""

import json
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool, Float64, Header, String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402


def test_autonomy_preview_stops_and_stays_disarmed(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '121')
    params = tmp_path / 'preview.yaml'
    params.write_text(
        'autonomy_node:\n  ros__parameters:\n    erpm_per_mps: 4000.0\n'
        '    left_angle_rad: 0.5\n    right_angle_rad: 0.5\n', encoding='utf-8')
    rclpy.init(args=[])
    node = Node('autonomy_preview_test')
    command = node.create_publisher(TwistStamped, '/oval/cmd_vel_checked', 1)
    scan = node.create_publisher(LaserScan, '/scan', 1)
    odom = node.create_publisher(Odometry, '/odom', 1)
    heartbeat = node.create_publisher(Header, '/oval/driver_alive', 1)
    publisher = node.create_publisher(Bool, '/drive/stop', 1)
    sources = {'command', 'scan', 'odom', 'driver'}
    values = {'autonomy': {}, 'drive': {}, 'steering': {}}
    speed, current, steering = [], [], []
    parts = ('drive', 'steering', 'autonomy')
    for part in parts:
        node.create_subscription(
            String, f'/{part}/status',
            lambda msg, part=part: values.update({part: json.loads(msg.data)}), 1)
    node.create_subscription(Float64, '/drive/preview/speed', lambda m: speed.append(m.data), 1)
    node.create_subscription(Float64, '/drive/preview/current', lambda m: current.append(m.data), 1)
    node.create_subscription(Float64, '/steering/preview/servo_position',
                             lambda m: steering.append(m.data), 1)
    enables = {part: node.create_client(SetBool, f'/{part}/enable') for part in parts}
    resets = {part: node.create_client(Trigger, f'/{part}/reset_stop') for part in parts}

    def send():
        stamp = node.get_clock().now().to_msg()
        if 'command' in sources:
            msg = TwistStamped(header=Header(stamp=stamp))
            msg.twist.linear.x = 0.25
            msg.twist.angular.z = 0.1
            command.publish(msg)
        if 'scan' in sources:
            scan.publish(LaserScan(header=Header(stamp=stamp, frame_id='lidar_link'),
                                   range_min=0.06, range_max=10.0, angle_increment=0.01,
                                   ranges=[1.0] * 20))
        if 'odom' in sources:
            odom.publish(Odometry(header=Header(stamp=stamp, frame_id='odom'),
                                  child_frame_id='base_link'))
        if 'driver' in sources:
            heartbeat.publish(Header(stamp=stamp, frame_id='map'))

    node.create_timer(0.04, send)

    def until(condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert condition(), values

    def service(client, request):
        future = client.call_async(request)
        until(future.done)
        return future.result()

    def enabled(part):
        return values[part].get('enabled')

    def all_latched():
        return all(values[part].get('stop_latched') and enabled(part) is False for part in parts)

    def arm():
        for part in parts[:-1]:
            assert service(enables[part], SetBool.Request(data=True)).success
        until(lambda: enabled('drive') and enabled('steering'))
        assert service(enables['autonomy'], SetBool.Request(data=True)).success
        until(lambda: speed and speed[-1] == 600.0 and steering and steering[-1] < 0.5)

    def clear():
        publisher.publish(Bool(data=False))
        until(lambda: service(resets['autonomy'], Trigger.Request()).success)
        for part in parts[:-1]:
            assert service(resets[part], Trigger.Request()).success
        until(lambda: all(values[part].get('stop_latched') is False for part in parts))
        assert all(enabled(part) is False for part in parts)

    launch = subprocess.Popen(
        ['ros2', 'launch', 'car_drive', 'drive.launch.py', 'steering:=true',
         'steering_calibrated:=true', 'max_erpm:=1000.0', f'autonomy_config:={params}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        until(lambda: all(values.values()), timeout=8.0)
        assert all(enabled(part) is False for part in parts)
        assert not service(enables['autonomy'], SetBool.Request(data=True)).success
        assert not node.get_subscriptions_info_by_topic('/drive/command_erpm')
        assert not node.get_subscriptions_info_by_topic('/steering/command')
        assert not node.get_publishers_info_by_topic('/drive/vesc/commands/motor/speed')

        current.clear()
        arm()
        current.clear()
        until(lambda: publisher.get_subscription_count() == 3)
        publisher.publish(Bool(data=True))
        until(all_latched)
        until(lambda: current and current[-1] == 0.0 and steering[-1] == 0.5)
        assert not service(resets['autonomy'], Trigger.Request()).success
        assert not service(enables['autonomy'], SetBool.Request(data=True)).success
        clear()

        for source, reason in (('scan', 'scan'), ('odom', 'odometry'),
                               ('driver', 'driver'), ('command', 'command')):
            arm()
            sources.remove(source)
            until(all_latched)
            assert values['autonomy']['state'] == f'{reason} timed out'
            sources.add(source)
            clear()
    finally:
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)
        try:
            output, _ = launch.communicate(timeout=6.0)
        except subprocess.TimeoutExpired:
            launch.kill()
            output, _ = launch.communicate(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        print(output)
    assert launch.returncode == 0
