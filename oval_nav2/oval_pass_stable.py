"""Footprint-aware forward passes for the oval direct-reference driver."""
import math
from bisect import bisect_left
from collections import namedtuple

from oval_lane import FRONT, SIDE, HALF_WIDTH, RADIUS, project, world, wrap, wrap_angle

Section = namedtuple("Section", "s x y yaw arc nearby")


def plan_pass(start, offset, heading, length, scan, mapped, previous):
    """Search smooth forward passes and score clearance of the oriented body."""
    # Aggregate dense returns conservatively: each cell contains a disk large
    # enough to cover every original hit and its uncertainty.
    cells = {}
    for s, d, pad in [(s,d,.075 if abs(d)<HALF_WIDTH-.12 else 0.) for s,d in scan] + list(mapped):
        if not -.85 < wrap(s-start) < length+.85:
            continue
        x,y = world(s,d)[:2]
        key = math.floor(x/.04), math.floor(y/.04)
        radius = pad + math.hypot(x-(key[0]+.5)*.04,y-(key[1]+.5)*.04)
        cells[key] = max(cells.get(key,0.),radius)
    hits = [((i+.5)*.04,(j+.5)*.04,p) for (i,j),p in cells.items()]
    prior_s = [s for s,d in previous]
    error = wrap_angle(heading-world(start,offset)[2])
    if abs(error) > .70:
        raise ValueError("Heading too far across the lane for a forward pass")
    initial_slope = math.tan(error)*(1+offset/RADIUS if abs(world(start,0)[0])>3 else 1)
    sections = max(1,math.ceil(length/1.05))
    span = length/sections
    count = max(2,math.ceil(span/.04))
    # Each checked sample covers the motion to the next sample and heading change.
    front, side = FRONT+.032, SIDE+.032
    states = {offset:(0., [(start,offset)])}
    for row in range(sections):
        samples = []
        for i in range(count+1):
            s = start + (row+i/count)*span
            x,y,yaw = world(s,0.)
            samples.append(Section(s,x,y,yaw,abs(x)>3.,[(ox,oy,p) for ox,oy,p in hits
                if abs(x-ox)<1.25 and abs(y-oy)<1.25]))
        next_states = {}
        for last,(cost,path) in states.items():
            slope = initial_slope if row==0 else 0.
            for goal in sorted({last,0.,*(i*.045 for i in range(-10,11))}):
                if abs(goal-last)>.56*span: continue
                total=cost
                edge=[]
                last_angle=None
                for i,q in enumerate(samples):
                    t=i/count
                    d=last+(goal-last)*t*t*(3-2*t)+span*slope*t*(1-t)**2
                    deriv=(goal-last)*6*t*(1-t)/span+slope*(1-4*t+3*t*t)
                    arc=1+d/RADIUS if q.arc else 1.
                    angle=q.yaw+math.atan2(deriv,arc)
                    x=q.x-math.sin(q.yaw)*d
                    y=q.y+math.cos(q.yaw)*d
                    if last_angle is not None and abs(wrap_angle(angle-last_angle))>math.hypot(x-px,y-py)/.65:
                        break
                    last_angle,px,py=angle,x,y
                    ca,sa=math.cos(angle),math.sin(angle)
                    margin=min(HALF_WIDTH-abs(project(x+u*ca-v*sa,y+u*sa+v*ca)[1])
                               for u in (-front,0.,front) for v in (-side,side))
                    if margin<=0.: break
                    for ox,oy,pad in q.nearby:
                        dx,dy=ox-x,oy-y
                        gap=max(abs(dx*ca+dy*sa)-front, abs(-dx*sa+dy*ca)-side)-pad
                        margin=min(margin,gap)
                        if margin<=0.: break
                    if margin<=0.: break
                    idx=bisect_left(prior_s,q.s)
                    prior=previous[idx][1] if idx<len(previous) else d
                    total += (4*d*d+3*deriv*deriv+90*max(0.,.14-margin)**2+
                              (6.0 if q.s-start<1.2 else 1.5)*(d-prior)**2)*span/count
                    if i:edge.append((q.s,d))
                else:
                    if goal not in next_states or total<next_states[goal][0]:
                        next_states[goal]=(total,path+edge)
        if not next_states:
            # Only send a partial path if it has a full maneuver ahead.
            if row*span>=1.5: break
            raise ValueError(f"No reachable swept-footprint pass in section {row+1} at {row*span:.2f} m ahead")
        states=next_states
    return min(states.values(),key=lambda v:v[0])[1], 0
