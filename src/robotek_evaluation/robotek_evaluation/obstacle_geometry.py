"""Planar geometry of the Obstacle judge: the car footprint, sign boxes (possibly tilted), distances."""
import math

def rectangle(x, y, yaw, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b)
            for a, b in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]]

def edges(polygon):
    return list(zip(polygon, polygon[1:]+polygon[:1]))

def point_segment_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

def polygon_distance(a, b):
    # Separating axis theorem includes containment and intersecting edges.
    separated = False
    for p, q in edges(a)+edges(b):
        nx, ny = -(q[1]-p[1]), q[0]-p[0]
        pa = [nx*x+ny*y for x, y in a]
        pb = [nx*x+ny*y for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            separated = True
            break
    if not separated:
        return 0.0
    return min([point_segment_distance(p, x, y) for p in a for x, y in edges(b)]
               + [point_segment_distance(p, x, y) for p in b for x, y in edges(a)])

def collision_polygons(row):
    x, y, yaw = (float(row[k]) for k in ('x_gt', 'y_gt', 'yaw_gt'))
    shapes = {'chassis': rectangle(x, y, yaw, .0885, .045)}
    for axle, wx in [('front', .065), ('rear', -.065)]:
        for side, wy, column in [('left', .05, 'steer_actual'),
                                 ('right', -.05, 'steer_right_actual')]:
            angle = float(row[column]) if axle == 'front' else 0.0
            assert math.isfinite(angle), 'Missing measured steering'
            cx = x+math.cos(yaw)*wx-math.sin(yaw)*wy
            cy = y+math.sin(yaw)*wx+math.cos(yaw)*wy
            # Horizontal cylinder projects to a rectangle in the ground plane.
            shapes[axle+'_'+side+'_wheel'] = rectangle(cx, cy, yaw+angle, .03, .006)
    return shapes

def quat_multiply(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)

def quat_rotate(q, v):
    x, y, z, w = q
    return ((1-2*(y*y+z*z))*v[0]+2*(x*y-z*w)*v[1]+2*(x*z+y*w)*v[2],
            2*(x*y+z*w)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-x*w)*v[2],
            2*(x*z-y*w)*v[0]+2*(y*z+x*w)*v[1]+(1-2*(x*x+y*y))*v[2])

def quat_from_yaw(yaw):
    return (0., 0., math.sin(yaw/2), math.cos(yaw/2))

def tilt_degrees(q):
    # Angle between the pillar's vertical axis and world up.
    return math.degrees(math.acos(max(-1., min(1., quat_rotate(q, (0, 0, 1))[2]))))

def box_projection(x, y, q, hx=.025, hy=.025, hz=.05):
    # Ground projection of a (possibly knocked-down) box: its 8 corners in xy.
    return [(x+dx, y+dy) for dx, dy, _ in
            (quat_rotate(q, (a, b, c)) for a in (-hx, hx) for b in (-hy, hy) for c in (-hz, hz))]
