"""The signs (step 3): find them with the LiDAR, colour them with the camera, remember them by seat.

A sign is a 50 x 50 mm box: the LiDAR sees it as a short cluster of points standing apart from
the walls (``robotek_course.obstacle_vision.scan_clusters``), and the camera sees a red or green
block; the node pairs each block with the cluster at the same bearing and hands the strategy
``observations``: ``dict(x, y, color)`` in the car frame.

The layout does not change during a round (rules 9.4-9.5), and signs only stand on the 12 seats
(``lane.SEATS``, two rows). So every cluster is filed under its seat, with a count of sightings
and a vote for each colour seen; the memory lasts the whole round. A colour seen once, even a lap
earlier, is not lost when the sign leaves the camera's narrow view (66.7 degrees).
"""
import math

from . import lane

SECTOR = 0.2            # m from the lane centre a cluster may be and still be a sign (rows at +-0.1;
                        # the parking lot limits stick out to -0.3)
SEAT_SNAP = 0.15        # track units between a cluster and the seat it is filed under
SEAT_HITS = 2           # sightings before a seat counts as a sign
COLOUR_GATE = 0.12      # m between an observation and a cluster to give it the colour
COLOUR_AGE = 0.6        # s; older observations colour nothing
SWAP = {'red': 'green', 'green': 'red'}


def sign_position(cluster, pose, phi, sign, lookahead):
    """(progress, lateral) of a cluster on the lane, or None when it is behind the car or too far.

    ``cluster`` is in the car frame (x ahead, y left) of the real car; ``sign`` mirrors it for CW.
    """
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    lx, ly, lane_heading = lane.lane_pose(phi)
    tangent = (math.cos(lane_heading), math.sin(lane_heading))
    cluster_y = sign * cluster['y']
    wx = x + c * cluster['x'] - s * cluster_y
    wy = y + s * cluster['x'] + c * cluster_y
    along = (wx - lx) * tangent[0] + (wy - ly) * tangent[1]
    if not (-0.05 < along < lookahead):
        return None
    if math.hypot(cluster['x'], cluster_y) > lookahead + 0.5:
        return None
    sign_progress = lane.progress_of(wx, wy, phi)
    px, py, _ = lane.lane_pose(sign_progress)
    nx, ny = lane.lane_normal(sign_progress)
    lateral = (wx - px) * nx + (wy - py) * ny
    return sign_progress, lateral


def sign_colour(cluster, observations, observations_stamp, scan_stamp, sign):
    """Colour of the observation next to the cluster (mirrored for CW), or None."""
    if observations_stamp < scan_stamp - COLOUR_AGE:
        return None
    for observation in observations:
        if math.hypot(observation['x'] - cluster['x'], observation['y'] - cluster['y']) < COLOUR_GATE:
            colour = observation.get('color')
            return colour if sign > 0 else SWAP.get(colour, colour)
    return None


class SeatMemory:
    """Seats seen so far: half-unit seat index (0..15) -> [lateral, {colour: votes}, sightings]."""

    def __init__(self):
        self.seats = {}

    def remember(self, clusters, observations, observations_stamp, scan_stamp, pose, phi, sign, lookahead):
        """File the clusters of one scan under their seats and count the colours seen."""
        for cluster in clusters:
            position = sign_position(cluster, pose, phi, sign, lookahead)
            if position is None or abs(position[1]) > SECTOR:
                continue
            seat = round(position[0] * 2.0) / 2.0
            if abs(position[0] - seat) > SEAT_SNAP or seat % 2.0 > 1.0:
                continue                      # between seats, or in a corner: not a sign
            entry = self.seats.setdefault(int(round(seat * 2.0)) % 16, [0.0, {}, 0])
            entry[0] = lane.ROW if position[1] >= 0.0 else -lane.ROW
            entry[2] += 1
            colour = sign_colour(cluster, observations, observations_stamp, scan_stamp, sign)
            if colour is not None:
                entry[1][colour] = entry[1].get(colour, 0) + 1

    def colour(self, key):
        votes = self.seats[key][1]
        return max(votes, key=votes.get) if votes else None

    def next_sign(self, phi, lookahead):
        """Nearest sign ahead (seen twice or more): (lateral m, progress units, colour or None)."""
        best = None
        origin = phi - 0.05
        for key, (lateral, votes, hits) in self.seats.items():
            if hits < SEAT_HITS:
                continue
            progress = origin + (key / 2.0 - origin) % lane.UNITS_PER_LAP
            if progress - phi > lookahead:
                continue
            if best is None or progress < best[1]:
                colour = max(votes, key=votes.get) if votes else None
                best = (lateral, progress, colour)
        return best

    def ahead(self, phi, behind, horizon):
        """Signs from ``behind`` units back to ``horizon`` ahead, by progress (counting laps):
        {progress: (lateral, colour)}."""
        out = {}
        origin = phi - behind
        for key, (lateral, votes, hits) in self.seats.items():
            if hits < SEAT_HITS:
                continue
            progress = round(origin + (key / 2.0 - origin) % lane.UNITS_PER_LAP, 2)
            if progress - phi <= horizon:
                out[progress] = (lateral, max(votes, key=votes.get) if votes else None)
        return out
