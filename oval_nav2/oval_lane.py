"""Lane coordinates and clearance planning for the ECD726 stadium-shaped oval."""
import math
from statistics import median

RADIUS = 3.0
HALF_WIDTH = 1.29301875 / 2
LAP = 12 + 2 * math.pi * RADIUS
# Half footprint plus local costmap padding and a small tracking allowance.
SIDE = 0.145 + 0.04 + 0.04
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
        center = (lo+hi)/2
        nearby = [(d,pad) for s,d,pad in obstacles if abs(s-t) < (FRONT+pad)*1.16]
        prior = min(previous, key=lambda p: abs(p[0]-(start+t)), default=None)
        row, back = {}, {}
        for j in range(-20,21):
            d = j*.025
            if d-lo < SIDE or hi-d < SIDE: continue
            gap = min((abs(d-v)-pad-SIDE for v,pad in nearby), default=1.0)
            if gap < 0: continue
            choices = [(c + 5*((d-last)/step)**2,last) for last,c in costs.items()
                       if abs(d-last) <= .57*step+.001]
            if not choices: continue
            value,last = min(choices)
            sticky = 1.5*(d-prior[1])**2 if prior and abs(prior[0]-start-t)<.14 else 0
            row[d] = value + 8*(d-center)**2 + 25*max(0,.10-gap)**2 + sticky
            back[d] = last
        if not row: raise ValueError(f"No connected car-width corridor {t:.2f} m ahead")
        costs = row
        parents.append(back)
    d=min(costs,key=costs.get)
    path=[]
    for i in range(count,0,-1):
        path.append((start+i*step,d))
        d=parents[i-1][d]
    return [(start,offset)] + path[::-1], measured
