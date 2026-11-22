import argparse
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser(description="Drive clockwise laps of the ECD726 oval")
    parser.add_argument("--laps", type=int, default=1)
    args = parser.parse_args()
    if args.laps < 1:
        parser.error("--laps must be positive")
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter("use_sim_time", value=True)])
    # Map-frame gates with tangent headings along the oval.
    halves = (
        [
            (1.285, 1.871, -12.2),
            (2.571, 1.883, 12.5),
            (3.844, 1.877, -16.2),
            (5.09, 1.412, -34.6),
            (5.766, 0.264, -75.3),
            (5.999, -1.0, -90.0),
            (5.558, -2.168, -122.8),
            (4.93, -3.229, -128.2),
            (3.844, -3.877, -163.6),
            (2.571, -4.155, -169.0),
            (1.285, -4.089, 167.6),
            (-0.0, -3.893, 167.4),
        ],
        [
            (-1.285, -3.861, -168.1),
            (-2.571, -4.016, -173.6),
            (-3.902, -4.075, 161.6),
            (-4.966, -3.269, 136.5),
            (-5.725, -2.245, 112.6),
            (-5.794, -1.0, 86.7),
            (-5.709, 0.238, 72.7),
            (-4.963, 1.266, 40.9),
            (-3.889, 2.032, 26.4),
            (-2.571, 2.082, -12.3),
            (-1.285, 2.0, 0.0),
            (0.0, 2.0, 0.0),
        ],
    )
    tree = str(Path(__file__).with_name("oval_lap.xml").resolve())
    buffer = Buffer()
    listener = TransformListener(buffer, nav)
    try:
        for lap in range(1, args.laps + 1):
            for half, gates in enumerate(halves, start=1):
                poses = []
                for x, y, degrees in gates:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = float(x)
                    pose.pose.position.y = float(y)
                    pose.pose.orientation.z = math.sin(math.radians(degrees) / 2)
                    pose.pose.orientation.w = math.cos(math.radians(degrees) / 2)
                    poses.append(pose)
                x, y, _ = gates[-1]
                print(f"Lap {lap}, half {half}: {len(gates)} goals to ({x}, {y})", flush=True)
                accepted = nav.goThroughPoses(poses, behavior_tree=tree)
                if accepted is False:
                    raise SystemExit("Nav2 rejected the route. Check Terminal 2.")
                while not nav.isTaskComplete():
                    pass
                if nav.getResult() != TaskResult.SUCCEEDED:
                    raise SystemExit(f"Half {half} did not succeed. Check Terminal 2.")
                deadline = time.monotonic() + 5.0
                last = "map to base_link unavailable"
                while time.monotonic() < deadline:
                    rclpy.spin_once(nav, timeout_sec=0.1)
                    try:
                        tf = buffer.lookup_transform("map", "base_link", Time())
                    except TransformException:
                        continue
                    pos = tf.transform.translation
                    last = f"localized position ({pos.x:.2f}, {pos.y:.2f})"
                    if math.hypot(pos.x - x, pos.y - y) <= 0.45:
                        print(f"Half {half} verified: {last}", flush=True)
                        break
                else:
                    raise SystemExit(f"Nav2 returned success but checkpoint ({x}, {y}) was unconfirmed: {last}.")
            print(f"Lap {lap} completed.", flush=True)
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
