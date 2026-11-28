"""Use the existing Jetson autonomy gate for physical track learning."""

import json
import math
import time

from std_msgs.msg import Bool, Header, String


MAX_ROBOT_SPEED_MPS = 0.15
GUARD_STATUS_TIMEOUT_S = 1.25


class RobotGuard:
    def __init__(self, node):
        self.node = node
        self.status = None
        self.received_at = 0.0
        self.stop_input = False
        self.alive = node.create_publisher(Header, '/oval/driver_alive', 1)
        self.stop = node.create_publisher(Bool, '/drive/stop', 1)
        node.create_subscription(String, '/autonomy/status', self.receive_status, 1)
        node.create_subscription(Bool, '/drive/stop', self.receive_stop, 1)

    def receive_status(self, message):
        try:
            status = json.loads(message.data)
            speed = status['max_speed_mps']
            if (type(status['enabled']) is not bool or type(status['stop_latched']) is not bool
                    or not math.isfinite(speed) or not 0.0 < speed <= MAX_ROBOT_SPEED_MPS):
                raise ValueError('Use a validated autonomy guard cap of at most 0.15 m/s')
        except (ValueError, TypeError, KeyError):
            self.status = None
            return
        self.status = status
        self.received_at = time.monotonic()

    def receive_stop(self, message):
        self.stop_input = message.data

    def current(self):
        if time.monotonic() - self.received_at <= GUARD_STATUS_TIMEOUT_S:
            return self.status
        return None

    def armed(self):
        return (self.current() is not None and self.status['enabled']
                and not self.status['stop_latched'] and not self.stop_input)

    def require_armed(self):
        if not self.armed():
            raise ValueError('Autonomy guard stopped or its status is stale')

    def stopped(self):
        return (self.current() is not None and not self.status['enabled']
                and self.status['stop_latched'])

    def heartbeat(self):
        self.alive.publish(Header(stamp=self.node.get_clock().now().to_msg(), frame_id='map'))

    def latch(self):
        self.stop.publish(Bool(data=True))
