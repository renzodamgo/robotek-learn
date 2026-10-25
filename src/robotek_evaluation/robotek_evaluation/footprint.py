"""Where the body and the wheels of the car are on the floor, from its recorded pose and steering."""
import math


def rectangle(x, y, yaw, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]]


def collision_polygons(row):
    """The chassis and the four wheels as rectangles on the ground, for one pose row (x_gt, y_gt, yaw_gt, steering)."""
    x, y, yaw = (float(row[k]) for k in ('x_gt', 'y_gt', 'yaw_gt'))
    shapes = {'chassis': rectangle(x, y, yaw, .0885, .045)}
    for axle, wx in [('front', .065), ('rear', -.065)]:
        for side, wy, column in [('left', .05, 'steer_actual'),
                                 ('right', -.05, 'steer_right_actual')]:
            angle = float(row[column]) if axle == 'front' else 0.0
            assert math.isfinite(angle), 'Missing measured steering'
            cx = x + math.cos(yaw) * wx - math.sin(yaw) * wy
            cy = y + math.sin(yaw) * wx + math.cos(yaw) * wy
            # A horizontal cylinder projects to a rectangle on the ground.
            shapes[axle + '_' + side + '_wheel'] = rectangle(cx, cy, yaw + angle, .03, .006)
    return shapes
