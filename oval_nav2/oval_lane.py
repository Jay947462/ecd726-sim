"""Lane coordinates and clearance planning for the ECD726 stadium-shaped oval."""
import math
from statistics import median

RADIUS = 3.0
HALF_WIDTH = 1.29301875 / 2
LAP = 12 + 2 * math.pi * RADIUS
# Half footprint plus local costmap padding and a small tracking allowance.
SIDE = 0.145 + 0.04
FRONT = 0.27 + 0.04 + 0.05


def wrap(delta):
    return (delta + LAP / 2) % LAP - LAP / 2


def project(x, y):
    """Return clockwise distance from spawn and signed outward distance."""
    if x > 3:
        angle = math.atan2(x - 3, y + 1)
        return 3 + RADIUS * angle, math.hypot(x - 3, y + 1) - RADIUS
    if x < -3:
        angle = math.atan2(-x - 3, -y - 1)
        return (9 + math.pi * RADIUS + RADIUS * angle) % LAP, math.hypot(x + 3, y + 1) - RADIUS
    if y >= -1:
        return x % LAP, y - 2
    return 6 + math.pi * RADIUS - x, -4 - y


def world(s, d):
    u = (s + 3) % LAP
    if u < 6:
        return u - 3, 2 + d, 0.0
    u -= 6
    if u < math.pi * RADIUS:
        a = u / RADIUS
        return 3 + (RADIUS+d)*math.sin(a), -1 + (RADIUS+d)*math.cos(a), -a
    u -= math.pi * RADIUS
    if u < 6:
        return 3-u, -4-d, -math.pi
    a = (u-6) / RADIUS
    return -3-(RADIUS+d)*math.sin(a), -1-(RADIUS+d)*math.cos(a), -math.pi-a


def mapped_obstacles(image):
    # The companion map is a 2.5 cm binary PGM with lower-left origin (-8,-6).
    magic, dims, peak, pixels = image.read_bytes().split(b'\n', 3)
    if (magic, dims, peak, len(pixels)) != (b'P5', b'640 400', b'255', 256000):
        raise ValueError("Use track.pgm bundled with the ECD726 oval")
    points = []
    for i, value in enumerate(pixels):
        if value == 0:
            s, d = project(-8 + (i%640+.5)*.025, -6 + (399-i//640+.5)*.025)
            if abs(d) < HALF_WIDTH-.10:
                points.append((s, d, .018))
    return points


def free_intervals(lo, hi, obstacles):
    """All allowed car-center intervals after accounting for obstacle and body width."""
    intervals = []
    cursor = lo + SIDE
    limit = hi - SIDE
    for left, right in sorted((d - pad - SIDE, d + pad + SIDE) for d, pad in obstacles):
        if right <= cursor or left >= limit:
            continue
        if cursor < left:
            intervals.append((cursor, left))
        cursor = max(cursor, right)
        if cursor >= limit:
            break
    if cursor < limit:
        intervals.append((cursor, limit))
    return intervals


def route(start, offset, length, scan, mapped, previous=()):
    # Bin scan returns by position along the oval, so curved walls can be
    # compared across the lane rather than comparing two unrelated range rays.
    wall = [(wrap(s-start), d) for s,d in scan if abs(abs(d)-HALF_WIDTH) < .12]
    obstacles = [(wrap(s-start), d, .075) for s,d in scan if abs(d) <= HALF_WIDTH-.12]
    obstacles += [(wrap(s-start), d, pad) for s,d,pad in mapped]
    count = max(2, math.ceil(length/.18))
    step = length/count
    costs = {offset: 0.0}
    parents = []
    measured = 0
    for i in range(1,count+1):
        t = i*step
        inside = [d for s,d in wall if d < 0 and abs(s-t)<.40]
        outside = [d for s,d in wall if d > 0 and abs(s-t)<.40]
        lo = median(inside) if len(inside)>=3 else -HALF_WIDTH
        hi = median(outside) if len(outside)>=3 else HALF_WIDTH
        measured += int(len(inside)>=3 and len(outside)>=3)
        # The mapped oval supplies the nominal center even around a blind bend.
        # Only a matched pair of walls can correct that center. An occluded wall
        # still constrains clearance through the existing one-sided bounds.
        center = 0.0
        if len(inside) >= 3 and len(outside) >= 3:
            center = max(-.04, min(.04, (lo + hi) / 2))
        nearby = [(d,pad) for s,d,pad in obstacles if abs(s-t) < (FRONT+pad)*1.16]
        intervals = free_intervals(lo, hi, nearby)
        prior = min(previous, key=lambda p: abs(p[0]-(start+t)), default=None)
        row, back = {}, {}
        for j in range(-20,21):
            d = j*.025
            interval = next(((left, right) for left, right in intervals if left <= d <= right), None)
            if interval is None:
                continue
            left, right = interval
            # A cone changes the usable lane. Favor the middle of the available
            # opening, with a preference for wider gaps when both are reachable.
            target = (left + right) / 2 if nearby else center
            clearance = min(d-left, right-d)
            choices = [(c + 5*((d-last)/step)**2,last) for last,c in costs.items()
                       if abs(d-last) <= .57*step+.001]
            if not choices: continue
            value,last = min(choices)
            sticky = 1.5*(d-prior[1])**2 if prior and abs(prior[0]-start-t)<.14 else 0
            row[d] = value + 20*(d-target)**2 + 45*max(0,.15-clearance)**2 + sticky
            back[d] = last
        if not row:
            widest = max((b-a+2*SIDE for a,b in intervals), default=0)
            raise ValueError(f"At {t:.2f} m ahead: wall bounds [{lo:+.2f}, {hi:+.2f}], "
                             f"{len(nearby)} obstacle samples, widest admissible opening "
                             f"{widest:.2f} m, no reachable offset")
        costs = row
        parents.append(back)
    d=min(costs,key=costs.get)
    path=[]
    for i in range(count,0,-1):
        path.append((start+i*step,d))
        d=parents[i-1][d]
    return [(start,offset)] + path[::-1], measured


def approach(start, offset, heading, length, scan, mapped, previous=(), *, check_to_end=False):
    """Try a curvature-limited transition into a clear lane before joining the normal route."""
    reach = max(2.8, length + .75) if check_to_end else 2.8
    points = [(s,d,p) for s,d,p in mapped if -.6 < wrap(s-start) < reach]
    points.extend((s,d,.075) for s,d in scan if abs(d)<HALF_WIDTH-.12 and -.6<wrap(s-start)<reach)
    obstacles = [(*world(s,d)[:2],p) for s,d,p in points]
    wall_hits = [world(s,d)[:2] for s,d in scan if abs(abs(d)-HALF_WIDTH)<.12 and -.6<wrap(s-start)<reach]
    slope = math.tan(wrap_angle(heading-world(start,offset)[2]))
    if abs(wrap_angle(heading-world(start,offset)[2]))>.65:
        raise ValueError("Heading too far across the lane for a forward recovery")
    if abs(world(start,0)[0])>3:
        slope *= 1+offset/RADIUS
    for span in (1.25, 1.55, 1.0, 1.85, .8):
        if span > length or (not check_to_end and span+.36>length): continue
        for goal in sorted((i*.05 for i in range(-7,8)),key=lambda v:abs(v-offset)+.3*abs(v)):
            checked_length = length if check_to_end else span
            count=math.ceil(checked_length/.025)
            prefix=[]
            xy=[]
            for i in range(count+1):
                distance = checked_length*i/count
                t=min(1.0, distance/span)
                d=offset+(goal-offset)*t*t*(3-2*t)+span*slope*t*(1-t)**2
                s=start+distance
                prefix.append((s,d))
                xy.append(world(s,d)[:2])
            valid=True
            last_heading=heading
            for i,(x,y) in enumerate(xy):
                a=xy[max(0,i-1)]
                b=xy[min(count,i+1)]
                angle=math.atan2(b[1]-a[1],b[0]-a[0])
                if i and abs(wrap_angle(angle-last_heading))>math.dist(xy[i-1],xy[i])/.65:
                    valid=False;break
                last_heading=angle
                ca,sa=math.cos(angle),math.sin(angle)
                # Test rotated padded body, including a small sampling allowance.
                front,side=FRONT+.015,SIDE+.015
                for u,v in ((front,side),(front,-side),(-front,side),(-front,-side),(0,side),(0,-side)):
                    if abs(project(x+u*ca-v*sa,y+u*sa+v*ca)[1])>=HALF_WIDTH-.005:
                        valid=False;break
                if not valid:break
                for ox,oy,pad in obstacles:
                    u,v=ox-x,oy-y
                    if abs(u*ca+v*sa)<=front+pad and abs(-u*sa+v*ca)<=side+pad:
                        valid=False;break
                if not valid:break
                for ox,oy in wall_hits:
                    u,v=ox-x,oy-y
                    if abs(u*ca+v*sa)<=front and abs(-u*sa+v*ca)<=side:
                        valid=False;break
                if not valid:break
            if valid:
                if check_to_end:
                    return prefix, 0
                try:
                    tail,n=route(start+span,goal,length-span,scan,mapped,previous)
                except ValueError:
                    continue
                return prefix[:-1]+tail,n
    raise ValueError("No verified forward approach fits the padded car at its present pose")


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def plan_horizon(start, anchor, offset, heading, length, scan, mapped, previous):
    """Prefer the full horizon. Allow a checked shorter segment with adequate lookahead."""
    attempts = [length] + [n for n in (3.6, 2.4) if n < length-.18]
    first_error = None
    for span in attempts:
        try:
            return route(start, anchor, span, scan, mapped, previous)
        except ValueError as error:
            if first_error is None:
                first_error = str(error)
        try:
            return approach(start, offset, heading, span, scan, mapped, previous)
        except ValueError:
            continue
    # The station bins can merge independent returns along a bend. Try a
    # continuous pass with the rotated padded footprint at 2.5 cm spacing.
    try:
        return approach(start, offset, heading, min(length, 2.4), scan, mapped,
                        previous, check_to_end=True)
    except ValueError:
        raise ValueError(first_error) from None
