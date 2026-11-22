"""Drive clockwise with live wall-centering and footprint-aware passing goals."""
import argparse
from collections import deque
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from oval_lane import LAP, SIDE, FRONT, HALF_WIDTH, project, world, wrap, mapped_obstacles, route


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def scan_in_map(msg, transform):
    a = yaw(transform.rotation) + msg.angle_min
    points = []
    for i,r in enumerate(msg.ranges):
        if msg.range_min <= r <= min(msg.range_max,6.0):
            angle = a+i*msg.angle_increment
            points.append(project(transform.translation.x+r*math.cos(angle),
                                  transform.translation.y+r*math.sin(angle)))
    return points


def goals(path):
    xy = [world(s,d)[:2] for s,d in path]
    indices = list(range(4,len(path)-2,4)) + [len(path)-1]
    poses = []
    for i in indices:
        x,y = xy[i]
        a,b = xy[max(0,i-2)],xy[min(i+2,len(xy)-1)]
        heading = math.atan2(b[1]-a[1], b[0]-a[0])
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = x,y
        p.pose.orientation.z = math.sin(heading/2)
        p.pose.orientation.w = math.cos(heading/2)
        poses.append(p)
    return poses


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--laps",type=int,default=1)
    args=parser.parse_args()
    if args.laps<1: parser.error("--laps must be positive")
    folder=Path(__file__).resolve().parent
    mapped=mapped_obstacles(folder/"track.pgm")
    rclpy.init()
    nav=BasicNavigator()
    nav.set_parameters([Parameter("use_sim_time",value=True)])
    buffer=Buffer()
    listener=TransformListener(buffer,nav)
    scans=deque(maxlen=24)
    def on_scan(message):
        scans.append(message)

    nav.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    last_good=time.monotonic()
    last_sample=-1
    progress=None
    active=False
    path=[]
    finish=LAP*args.laps
    next_send=0
    try:
        while rclpy.ok():
            rclpy.spin_once(nav,timeout_sec=.05)
            if time.monotonic()-last_good>12:
                raise SystemExit("No fresh synchronized scan and map TF. Check Gazebo Play, /clock, /scan and AMCL.")
            now=nav.get_clock().now().nanoseconds
            if now-last_sample<250000000 and last_sample>=0: continue
            try:
                base=buffer.lookup_transform("map","base_link",Time())
            except TransformException:
                continue
            for msg in reversed(scans):
                stamp=Time.from_msg(msg.header.stamp)
                if stamp.nanoseconds<=last_sample or now-stamp.nanoseconds>750000000: break
                try:
                    laser=buffer.lookup_transform("map",msg.header.frame_id,stamp).transform
                    break
                except TransformException:
                    continue
            else:
                continue
            if stamp.nanoseconds<=last_sample or now-stamp.nanoseconds>750000000: continue
            last_sample=stamp.nanoseconds
            pos=base.transform.translation
            s,d=project(pos.x,pos.y)
            if progress is None:
                if math.hypot(pos.x,pos.y-2)>.45 or abs(yaw(base.transform.rotation))>.55:
                    raise SystemExit("Start at (0, 2), pointing along +X, with AMCL localized.")
                progress=wrap(s)
            else:
                delta=wrap(s-progress)
                if abs(delta)>1: raise SystemExit("Localization jumped. Inspect map to base_link before retrying.")
                progress+=delta
            points=scan_in_map(msg,laser)
            if len(points)<8: continue
            last_good=time.monotonic()
            if active and nav.isTaskComplete():
                active=False
                if nav.getResult()!=TaskResult.SUCCEEDED:
                    raise SystemExit("Nav2 ended the current segment. Inspect planner and controller logs.")
                if path[-1][0]>=finish-.001 and progress>=finish-.4 and math.hypot(pos.x,pos.y-2)<.45:
                    print("Requested lap count completed.",flush=True)
                    return
            # Renew the forward horizon early; also react to a new interior return
            # that blocks a previously usable section before the next normal update.
            hazard=any(abs(v)<HALF_WIDTH-.12 and any(
                u>progress+.15 and abs(wrap(k-u))<FRONT and abs(v-e)<SIDE+.055
                for u,e in path) for k,v in points)
            if active and progress<next_send and not hazard: continue
            if active and path[-1][0]>=finish-.001 and not hazard: continue
            length=min(3.2,finish-progress)
            if length<.35:
                if active: continue
                raise SystemExit("Finish pose was not verified. Inspect localization.")
            try:
                new,measured=route(progress,d,length,points,mapped,path)
            except ValueError as error:
                raise SystemExit(str(error))
            poses=goals(new)
            if nav.goThroughPoses(poses,behavior_tree=str(folder/"oval_lap.xml")) is False:
                raise SystemExit("Nav2 rejected the local route.")
            active=True
            path=new
            next_send=progress+1.25
            print(f"Lap progress {progress:.1f}/{finish:.1f} m | both walls measured at "
                  f"{measured}/{len(path)-1} sections | target offsets "
                  f"{min(e for _,e in new):+.2f} to {max(e for _,e in new):+.2f} m",flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if active and rclpy.ok(): nav.cancelTask()
        nav.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=="__main__":
    main()
