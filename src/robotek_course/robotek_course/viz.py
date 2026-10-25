"""RViz markers for a Scene: what the LiDAR tells the strategy, drawn where the car sees it.

Provided to the student; nothing here is an exercise. Add a MarkerArray display on ``/scene_markers`` in RViz
(fixed frame: ``lidar_link``, the frame of /scan) to see:

  * green lines beside the car: the distances to the walls on the left and on the right;
  * a red line ahead of the car: the wall ahead (``front_axis``, along the track);
  * a yellow dot: where the island wall ends (``island_end``).
"""
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .minisim import LIDAR_X

HALF_LENGTH = 0.25       # m: the side lines run this far ahead of and behind the LiDAR
HALF_WIDTH = 0.30        # m: the wall-ahead line is this wide on each side of the car


def _marker(index, kind, stamp, frame_id):
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame_id, stamp
    marker.ns, marker.id, marker.type = 'scene', index, kind
    marker.pose.orientation.w = 1.0
    return marker


def _line(index, colour, points, stamp, frame_id):
    marker = _marker(index, Marker.LINE_STRIP, stamp, frame_id)
    marker.scale.x = 0.012
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*colour, 1.0)
    marker.points = [Point(x=x, y=y, z=0.02) for x, y in points]
    return marker


def _hidden(marker):
    marker.action = Marker.DELETE            # the number is infinite: nothing to draw
    return marker


def scene_markers(scene, stamp=None, frame_id='lidar_link'):
    """MarkerArray with the four numbers of ``scene`` as drawn in the LiDAR's frame (x forward, y left).

    ``stamp`` is a builtin_interfaces/Time (use the scan's ``header.stamp``). A number that is infinite (nothing seen)
    is removed from RViz instead of drawn.
    """
    stamp = stamp if stamp is not None else Time()
    green, red, yellow = (0.1, 0.8, 0.2), (0.9, 0.1, 0.1), (1.0, 0.85, 0.0)
    left = _line(0, green, [(-HALF_LENGTH, scene.left), (HALF_LENGTH, scene.left)], stamp, frame_id)
    right = _line(1, green, [(-HALF_LENGTH, -scene.right), (HALF_LENGTH, -scene.right)], stamp, frame_id)
    front = _line(2, red, [(scene.front_axis, -HALF_WIDTH), (scene.front_axis, HALF_WIDTH)], stamp, frame_id)
    end = _marker(3, Marker.SPHERE, stamp, frame_id)
    end.scale.x = end.scale.y = end.scale.z = 0.04
    end.color.r, end.color.g, end.color.b, end.color.a = (*yellow, 1.0)
    end.pose.position.x = scene.island_end - LIDAR_X      # island_end is in the car's frame, the markers in the LiDAR's
    end.pose.position.y = scene.left if math.isfinite(scene.left) else 0.0
    end.pose.position.z = 0.02
    markers = [left, right, front, end]
    for marker, value in zip(markers, (scene.left, scene.right, scene.front_axis, scene.island_end)):
        if not math.isfinite(value):
            _hidden(marker)
    return MarkerArray(markers=markers)
