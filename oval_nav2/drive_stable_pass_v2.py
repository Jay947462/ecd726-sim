"""Drive clockwise with live wall-centering and footprint-aware passing goals."""

import argparse
import json
from collections import deque
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from nav2_msgs.action import FollowPath, NavigateThroughPoses
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from oval_lane import RADIUS, LAP, SIDE, FRONT, HALF_WIDTH, project, world, wrap, mapped_obstacles, plan_horizon
from oval_pass_stable import plan_pass


REAL_SCAN_MAX_AGE_NS = 250_000_000
REAL_POSE_MAX_AGE_NS = 300_000_000
REAL_SENSORS_TIMEOUT_S = 0.75
ACTION_RESPONSE_TIMEOUT_S = 1.0
CANCEL_RESPONSE_TIMEOUT_S = 0.5


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
                        help="Send the lane reference straight to MPPI")
    parser.add_argument('--real-time', action='store_true',
                        help='Use the physical clock and detect cones from live lidar')
    parser.add_argument('--confirm-measured-oval', action='store_true',
                        help='Confirm matching 6 m straights, 3 m centerline radius, '
                             '1.293 m lane width and map start at (0, 2) facing +X')
    parser.add_argument('--replan-log', type=Path,
                        help='Write timestamped replanning outcomes as JSON Lines')
    args=parser.parse_args()
    if args.laps<1: parser.error("--laps must be positive")
    if args.real_time and not args.confirm_measured_oval:
        parser.error('--real-time requires --confirm-measured-oval after measuring and aligning the course')
    if args.confirm_measured_oval and not args.real_time:
        parser.error('--confirm-measured-oval is for --real-time runs')
    folder=Path(__file__).resolve().parent
    mode = "DIRECT reference -> MPPI" if args.direct_reference else "Smac -> MPPI"
    print(f"Lane planner: {mode}; 6.0 m free-space lookahead", flush=True)
    mapped = [] if args.real_time else mapped_obstacles(folder / 'track.pgm')
    scan_max_age_ns = REAL_SCAN_MAX_AGE_NS if args.real_time else 750_000_000
    # Nav2's absolute SpeedLimit can raise MPPI's configured physical speed cap.
    use_dynamic_speed_limit = args.direct_reference and not args.real_time
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    # Keep the ROS context alive until we can ask Nav2 to cancel its goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    old_handlers = {
        sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    nav=BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', value=not args.real_time)])
    buffer=Buffer()
    listener=TransformListener(buffer,nav)
    scans=deque(maxlen=24)
    def on_scan(message):
        scans.append(message)

    nav.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    reference_pub = nav.create_publisher(RosPath, '/oval/reference_path', 1)
    speed_pub = nav.create_publisher(SpeedLimit, "/oval/tracking_speed_limit", 10)
    heartbeat_pub = nav.create_publisher(Header, '/oval/driver_alive', 1) if args.real_time else None
    speed_cap = .25
    limit_time = None

    def publish_heartbeat(scan_stamp, pose_stamp):
        if heartbeat_pub is None or stop_requested:
            return
        timestamp = nav.get_clock().now()
        scan_age_ns = timestamp.nanoseconds - scan_stamp.nanoseconds
        pose_age_ns = timestamp.nanoseconds - Time.from_msg(pose_stamp).nanoseconds
        if 0 <= scan_age_ns <= REAL_SCAN_MAX_AGE_NS and 0 <= pose_age_ns <= REAL_POSE_MAX_AGE_NS:
            heartbeat_pub.publish(Header(stamp=timestamp.to_msg(), frame_id='map'))

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
    pending_goal = None
    replan_log = (args.replan_log.open('w', encoding='utf-8', buffering=1)
                  if args.replan_log else None)

    def log_replan(stage, outcome, attempt, **details):
        if replan_log is not None:
            print(json.dumps(dict(replan_id=attempt, stage=stage, outcome=outcome,
                                  sim_time_ns=nav.get_clock().now().nanoseconds,
                                  progress_m=progress, mode=mode, **details)), file=replan_log)

    def submit_goal(client, goal):
        nonlocal pending_goal
        deadline = time.monotonic() + ACTION_RESPONSE_TIMEOUT_S
        while not stop_requested and time.monotonic() < deadline:
            if client.server_is_ready():
                break
            rclpy.spin_once(nav, timeout_sec=.05)
        else:
            if stop_requested:
                return None
            raise SystemExit('Nav2 action server is unavailable.')
        if stop_requested:
            return None
        pending_goal = client.send_goal_async(goal)
        deadline = time.monotonic() + ACTION_RESPONSE_TIMEOUT_S
        while not pending_goal.done() and not stop_requested and time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=.05)
        if stop_requested:
            return None
        if not pending_goal.done():
            raise SystemExit('Nav2 did not acknowledge the goal.')
        return pending_goal.result()

    def cancel_goals():
        deadline = time.monotonic() + CANCEL_RESPONSE_TIMEOUT_S
        cancellations = []
        if active:
            cancellations.append(nav.goal_handle.cancel_goal_async())
            log_replan('navigation', 'cancel_requested', active_replan)
        if pending_goal is not None:
            rclpy.spin_until_future_complete(nav, pending_goal, timeout_sec=max(0., deadline - time.monotonic()))
            if pending_goal.done():
                handle = pending_goal.result()
                if handle.accepted and handle is not nav.goal_handle:
                    cancellations.append(handle.cancel_goal_async())
                    log_replan('navigation', 'cancel_requested', replan_id)
            else:
                nav.get_logger().warning('Pending Nav2 goal did not respond before driver exit.')
        for cancellation in cancellations:
            rclpy.spin_until_future_complete(nav, cancellation, timeout_sec=max(0., deadline - time.monotonic()))
            if not cancellation.done() or not cancellation.result().goals_canceling:
                nav.get_logger().warning('Nav2 did not confirm cancellation of a pending goal.')

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
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(nav,timeout_sec=.05)
            if stop_requested:
                break
            sensor_timeout = REAL_SENSORS_TIMEOUT_S if args.real_time and last_sample >= 0 else 12
            if time.monotonic() - last_good > sensor_timeout:
                if args.real_time:
                    raise SystemExit('No fresh synchronized scan and map TF. Check lidar, odometry and AMCL.')
                raise SystemExit('No fresh synchronized scan and map TF. Check Gazebo Play, /clock, /scan and AMCL.')
            now=nav.get_clock().now().nanoseconds
            if now-last_sample<250000000 and last_sample>=0: continue
            try:
                base=buffer.lookup_transform("map","base_link",Time())
            except TransformException:
                continue
            if args.real_time:
                pose_age_ns = now - Time.from_msg(base.header.stamp).nanoseconds
                if not 0 <= pose_age_ns <= REAL_POSE_MAX_AGE_NS:
                    continue
            for msg in reversed(scans):
                stamp=Time.from_msg(msg.header.stamp)
                scan_age_ns = now - stamp.nanoseconds
                if stamp.nanoseconds <= last_sample or scan_age_ns > scan_max_age_ns:
                    break
                if args.real_time:
                    scan_geometry = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
                    if (scan_age_ns < 0 or stamp.nanoseconds == 0 or not msg.header.frame_id
                            or not all(math.isfinite(value) for value in scan_geometry)
                            or msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max):
                        continue
                try:
                    laser=buffer.lookup_transform("map",msg.header.frame_id,stamp).transform
                    break
                except TransformException:
                    continue
            else:
                continue
            if stamp.nanoseconds <= last_sample or scan_age_ns > scan_max_age_ns:
                continue
            last_sample=stamp.nanoseconds
            pos=base.transform.translation
            if args.real_time and not all(math.isfinite(value) for value in (pos.x, pos.y, yaw(base.transform.rotation))):
                raise SystemExit('Invalid localization pose.')
            s,d=project(pos.x,pos.y)
            if args.real_time and abs(d) > HALF_WIDTH - SIDE:
                raise SystemExit('Track offset exceeded the measured lane envelope. Check the track and localization.')
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
            if active and use_dynamic_speed_limit and progress < path[-1][0]:
                prior_limit = tracking_limit(path, progress, d, yaw(base.transform.rotation))
            if active and not hazard:
                if path[-1][0] >= finish-.001:
                    if use_dynamic_speed_limit:
                        publish_limit(prior_limit, now)
                    publish_heartbeat(stamp, base.header.stamp)
                    continue
                if args.direct_reference:
                    if now < next_reference_ns:
                        if use_dynamic_speed_limit:
                            publish_limit(prior_limit, now)
                        publish_heartbeat(stamp, base.header.stamp)
                        continue
                elif progress < next_send:
                    publish_heartbeat(stamp, base.header.stamp)
                    continue
            length=min(6.0,finish-progress)
            if length<.35:
                if active:
                    if not hazard:
                        publish_heartbeat(stamp, base.header.stamp)
                    continue
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
                if args.real_time:
                    raise SystemExit(f'No checked route is available: {error}') from error
                if active:
                    cancel_goals()
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
            if stop_requested:
                break
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
                if use_dynamic_speed_limit:
                    target = min(prior_limit, tracking_limit(new, progress, d, yaw(base.transform.rotation)))
                    publish_limit(target, now)
                    detail += f", speed cap {speed_cap:.2f} m/s (target {target:.2f})"
                goal = FollowPath.Goal(path=reference, controller_id='FollowPath',
                                       goal_checker_id='general_goal_checker')
                nav.get_logger().info('Executing path...')
                handle = submit_goal(nav.follow_path_client, goal)
            else:
                goal = NavigateThroughPoses.Goal(poses=goals(new),
                                                 behavior_tree=str(folder / 'oval_centerline.xml'))
                nav.get_logger().info(f'Navigating with {len(goal.poses)} goals...')
                handle = submit_goal(nav.nav_through_poses_client, goal)
            if stop_requested:
                break
            if not handle.accepted:
                log_replan('dispatch', 'rejected', replan_id)
                raise SystemExit("Nav2 rejected the local route.")
            replaced_replan = active_replan if active else None
            nav.goal_handle = handle
            active = True
            active_replan = replan_id
            pending_goal = None
            nav.result_future = handle.get_result_async()
            log_replan('dispatch', 'accepted', replan_id)
            if replaced_replan is not None:
                log_replan('navigation', 'replacement_requested', replaced_replan,
                           replacement_id=replan_id)
            path=new
            next_reference_ns = now + 1_000_000_000
            next_send=progress+0.90
            publish_heartbeat(stamp, base.header.stamp)
            print(f"Lap progress {progress:.1f}/{finish:.1f} m | {detail} | target offsets "
                  f"{min(e for _,e in new):+.2f} to {max(e for _,e in new):+.2f} m",flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                cancel_goals()
                if use_dynamic_speed_limit:
                    speed_pub.publish(SpeedLimit())
        finally:
            nav.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if replan_log is not None:
                replan_log.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)


if __name__=="__main__":
    main()
