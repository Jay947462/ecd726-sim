"""Drive clockwise with live wall-centering and footprint-aware passing goals."""
from oval_pass_stable import plan_pass
from nav2_msgs.msg import SpeedLimit

import argparse
import json
from collections import deque
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from oval_lane import RADIUS, LAP, SIDE, FRONT, HALF_WIDTH, project, world, wrap, mapped_obstacles, plan_horizon


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
    poses = []
    last_s = path[0][0]
    for i, (s, d) in enumerate(path[1:], start=1):
        x, y, tangent = world(s, d)
        # Sample the circular arcs more closely and give Smac the tangent
        # at each gate. A one-sided chord biases the last heading on a bend.
        on_bend = abs(x) > 3.0
        spacing = 0.45 if on_bend else 0.65
        if i < len(path)-1 and (s-last_s < spacing or path[-1][0]-s < 0.30):
            continue
        last_s = s
        before = path[max(0, i-2)]
        after = path[min(len(path)-1, i+2)]
        lateral_slope = (after[1]-before[1])/(after[0]-before[0])
        arc_scale = 1+d/RADIUS if on_bend else 1.0
        heading = tangent + math.atan2(lateral_slope, arc_scale)
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = x,y
        p.pose.orientation.z = math.sin(heading/2)
        p.pose.orientation.w = math.cos(heading/2)
        poses.append(p)
    return poses


def tracking_samples(path):
    """Interpolate the lane offsets at costmap resolution and align poses to the line."""
    xy = []
    for (s0, d0), (s1, d1) in zip(path, path[1:]):
        count = max(1, math.ceil(math.hypot(s1-s0, d1-d0) / .025))
        for i in range(count):
            fraction = i / count
            xy.append(world(s0 + fraction*(s1-s0), d0 + fraction*(d1-d0))[:2])
    xy.append(world(*path[-1])[:2])
    for i, (x, y) in enumerate(xy):
        ax, ay = xy[max(0, i-1)]
        bx, by = xy[min(i+1, len(xy)-1)]
        yield x, y, math.atan2(by-ay, bx-ax)


def reference_offset(path, progress, fallback):
    """Anchor overlapping updates to the previous line while MPPI corrects tracking error."""
    for (s0, d0), (s1, d1) in zip(path, path[1:]):
        if s0 <= progress <= s1:
            fraction = (progress-s0)/(s1-s0)
            return d0 + fraction*(d1-d0)
    return fallback


def tracking_limit(path, progress, offset, heading):
    def sample(station):
        return world(station, reference_offset(path, station, path[-1][1]))[:2]

    end = path[-1][0]
    here, ahead = sample(progress), sample(min(end, progress + .10))
    tangent = math.atan2(ahead[1]-here[1], ahead[0]-here[0])
    yaw_error = abs(math.remainder(heading-tangent, 2*math.pi))
    error = abs(offset-reference_offset(path, progress, offset))
    cap = .42
    station = progress
    while station + .30 <= min(end, progress + .85):
        a, b, c = (sample(station + ds) for ds in (0., .15, .30))
        scale = math.dist(a,b)*math.dist(b,c)*math.dist(a,c)
        cross = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        curvature = 2*abs(cross)/max(scale, 1e-9)
        cap = min(cap, math.sqrt(.085/max(curvature, .001)))
        station += .10
    # Limit speed independently for curvature and tracking recovery.
    recovery = .42 - min(.17, max(0., error-.02)*.85 + max(0., yaw_error-.04)*.28)
    return max(.25, min(cap, recovery))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--laps",type=int,default=1)
    parser.add_argument("--direct-reference", action="store_true",
                        help="Simulation comparison: send the lane reference straight to MPPI")
    parser.add_argument('--replan-log', type=Path,
                        help='Write timestamped replanning outcomes as JSON Lines')
    args=parser.parse_args()
    if args.laps<1: parser.error("--laps must be positive")
    folder=Path(__file__).resolve().parent
    mode = "DIRECT reference -> MPPI" if args.direct_reference else "Smac -> MPPI"
    print(f"Lane planner: {mode}; 6.0 m free-space lookahead", flush=True)
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
    reference_pub = nav.create_publisher(RosPath, '/oval/reference_path', 1)
    speed_pub = nav.create_publisher(SpeedLimit, "/oval/tracking_speed_limit", 10)
    speed_cap = .25
    limit_time = None
    def publish_limit(target, now):
        nonlocal speed_cap, limit_time
        dt = 0. if limit_time is None else max(0., min(.5, (now-limit_time)*1e-9))
        speed_cap = min(target, speed_cap + .15*dt)
        limit_time = now
        msg = SpeedLimit()
        msg.header.stamp = nav.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = speed_cap
        speed_pub.publish(msg)

    last_good=time.monotonic()
    last_sample=-1
    progress=None
    active=False
    path=[]
    finish=LAP*args.laps
    next_send=0
    blocked_since_ns = None
    next_reference_ns = 0
    replan_id = 0
    active_replan = None
    replan_log = (args.replan_log.open('w', encoding='utf-8', buffering=1)
                  if args.replan_log else None)

    def log_replan(stage, outcome, attempt, **details):
        if replan_log is not None:
            print(json.dumps(dict(replan_id=attempt, stage=stage, outcome=outcome,
                                  sim_time_ns=nav.get_clock().now().nanoseconds,
                                  progress_m=progress, mode=mode, **details)), file=replan_log)

    if replan_log is not None and not args.direct_reference:
        smac_goals = set()
        terminal_states = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                           GoalStatus.STATUS_ABORTED: 'failed',
                           GoalStatus.STATUS_CANCELED: 'canceled'}

        def on_smac_status(message):
            for status in message.status_list:
                goal = bytes(status.goal_info.goal_id.uuid).hex()
                outcome = terminal_states.get(status.status)
                if outcome is not None and goal not in smac_goals:
                    smac_goals.add(goal)
                    log_replan('smac', outcome, None, goal_id=goal,
                               goal_stamp_ns=Time.from_msg(status.goal_info.stamp).nanoseconds)

        nav.create_subscription(GoalStatusArray, '/compute_path_through_poses/_action/status',
                                on_smac_status, qos_profile_action_status_default)

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
                result = nav.getResult()
                log_replan('navigation', result.name.lower(), active_replan)
                if result != TaskResult.SUCCEEDED:
                    raise SystemExit("Nav2 ended the current segment. Inspect planner and controller logs.")
                if path[-1][0]>=finish-.001 and progress>=finish-.4 and math.hypot(pos.x,pos.y-2)<.45:
                    print("Requested lap count completed.",flush=True)
                    return
            # Renew the forward horizon early; also react to a new interior return
            # that blocks a previously usable section before the next normal update.
            hazard=any(abs(v)<HALF_WIDTH-.12 and any(
                u>progress+.15 and abs(wrap(k-u))<FRONT and abs(v-e)<SIDE+.055
                for u,e in path) for k,v in points)
            prior_limit = .42
            if active and args.direct_reference and progress < path[-1][0]:
                prior_limit = tracking_limit(path, progress, d, yaw(base.transform.rotation))
            if active and not hazard:
                if path[-1][0] >= finish-.001:
                    if args.direct_reference:
                        publish_limit(prior_limit, now)
                    continue
                if args.direct_reference:
                    if now < next_reference_ns:
                        publish_limit(prior_limit, now)
                        continue
                elif progress < next_send:
                    continue
            length=min(6.0,finish-progress)
            if length<.35:
                if active: continue
                raise SystemExit("Finish pose was not verified. Inspect localization.")
            replan_id += 1
            started = time.monotonic()
            try:
                anchor = reference_offset(path, progress, d) if args.direct_reference else d
                if args.direct_reference:
                    new, _ = plan_pass(progress, d, yaw(base.transform.rotation),
                                       length, points, mapped, path)
                    detail = "rotated footprint checked"
                else:
                    new, measured = plan_horizon(progress,anchor,d,yaw(base.transform.rotation),
                                                 length,points,mapped,path)
                    detail = f"both walls measured at {measured}/{len(new)-1} sections"
            except ValueError as error:
                log_replan('lane_planner', 'failed', replan_id, reason=str(error),
                           requested_horizon_m=length, wall_duration_s=time.monotonic() - started,
                           scan_stamp_ns=stamp.nanoseconds)
                if active:
                    nav.cancelTask()
                    log_replan('navigation', 'cancel_requested', active_replan)
                    active = False
                if blocked_since_ns is None:
                    blocked_since_ns = now
                    print(f"Holding for a fresh scan at {progress:.2f} m: {error}", flush=True)
                elif now-blocked_since_ns >= 3_000_000_000:
                    capture = folder / "blocked_scan.json"
                    report = dict(progress=progress, offset=d, anchor=anchor,
                                  heading=yaw(base.transform.rotation), length=length,
                                  scan_stamp_ns=stamp.nanoseconds, scan=points,
                                  previous=path, reason=str(error))
                    try:
                        capture.write_text(json.dumps(report), encoding="utf-8")
                        print(f"Saved blocked geometry to {capture}", flush=True)
                    except OSError as exc:
                        print(f"Could not save blocked geometry: {exc}", flush=True)
                    raise SystemExit(f"Still blocked after 3 simulated seconds: {error}. "
                                     f"Progress {progress:.2f} m; offset {d:+.2f} m")
                continue
            log_replan('lane_planner', 'succeeded', replan_id, requested_horizon_m=length,
                       checked_horizon_m=new[-1][0] - progress,
                       wall_duration_s=time.monotonic() - started,
                       scan_stamp_ns=stamp.nanoseconds)
            blocked_since_ns = None
            reference = RosPath()
            reference.header.frame_id = "map"
            reference.header.stamp = nav.get_clock().now().to_msg()
            for px, py, tangent in tracking_samples(new):
                sample = PoseStamped()
                sample.header = reference.header
                sample.pose.position.x, sample.pose.position.y = px, py
                sample.pose.orientation.z = math.sin(tangent/2)
                sample.pose.orientation.w = math.cos(tangent/2)
                reference.poses.append(sample)
            reference_pub.publish(reference)
            if args.direct_reference:
                target = min(prior_limit, tracking_limit(new, progress, d, yaw(base.transform.rotation)))
                publish_limit(target, now)
                detail += f", speed cap {speed_cap:.2f} m/s (target {target:.2f})"
                accepted = nav.followPath(reference, controller_id="FollowPath",
                                          goal_checker_id="general_goal_checker")
            else:
                accepted = nav.goThroughPoses(goals(new), behavior_tree=str(folder/"oval_centerline.xml"))
            if accepted is False:
                log_replan('dispatch', 'rejected', replan_id)
                raise SystemExit("Nav2 rejected the local route.")
            log_replan('dispatch', 'accepted', replan_id)
            if active:
                log_replan('navigation', 'replacement_requested', active_replan,
                           replacement_id=replan_id)
            active=True
            active_replan = replan_id
            path=new
            next_reference_ns = now + 1_000_000_000
            next_send=progress+0.90
            print(f"Lap progress {progress:.1f}/{finish:.1f} m | {detail} | target offsets "
                  f"{min(e for _,e in new):+.2f} to {max(e for _,e in new):+.2f} m",flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if active and rclpy.ok():
            nav.cancelTask()
            log_replan('navigation', 'cancel_requested', active_replan)
        if args.direct_reference and rclpy.ok():
            speed_pub.publish(SpeedLimit())
        nav.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if replan_log is not None:
            replan_log.close()


if __name__=="__main__":
    main()
