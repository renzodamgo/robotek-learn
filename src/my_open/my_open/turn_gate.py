"""Step 6 - When to turn: three conditions at once."""
import math

TURN_FRONT = 1.19        # m: the wall ahead (along the track) must be this near
LOOKAHEAD = 0.35         # m: the island's end must be this near (when it is visible)
END_CLEARANCE = 0.08     # m: in a narrow lane the look-ahead shrinks to (distance to the island - this)
CONFIRMATIONS = 2        # consecutive scans that must see the island's end close
ARM_DISTANCE = 0.25      # m driven after a corner before the next one may start
ISLAND_OPEN = 1.0        # m: an island wall farther than this beside the car means the island has ended


class TurnGate:
    """Decides, every tick, whether the car should start a corner.

    Feed it the latest ``Scene`` (a NEW object for every LiDAR scan), ``inner`` (the distance to the island wall:
    ``scene.left`` on a counter-clockwise lap, ``scene.right`` on a clockwise one), the metres driven since the last
    corner, and whether a corner is in progress. The corner is due when ALL of these hold:

    1. the gate is armed (a corner has not just been started, and the island has been seen again since);
    2. at least ``ARM_DISTANCE`` metres have been driven since the last corner;
    3. the wall ahead is near: ``scene.front_axis <= TURN_FRONT``;
    4. the island has ended: the side is open (``inner > ISLAND_OPEN``) or its end was seen close in
       ``CONFIRMATIONS`` different scans in a row.
    """

    def __init__(self, turn_front=TURN_FRONT, lookahead=LOOKAHEAD):
        self.turn_front, self.lookahead = turn_front, lookahead
        self.armed = True
        self.hits = 0
        self._scene = None

    def update(self, scene, inner, travelled, arc):
        """Returns True when the corner should start now."""
        if scene is not self._scene:                 # one count per scan, however often the tick repeats
            self._scene = scene
            limit = min(self.lookahead, max(0.0, inner - END_CLEARANCE)) if math.isfinite(inner) else self.lookahead
            self.hits = self.hits + 1 if 0.0 <= scene.island_end <= limit else 0
        seen_again = math.isfinite(inner) and inner < ISLAND_OPEN and travelled >= ARM_DISTANCE
        self.armed = self.armed or (seen_again and not arc)
        island_open = math.isfinite(inner) and inner > ISLAND_OPEN
        return (self.armed and not arc and travelled >= ARM_DISTANCE
                and scene.front_axis <= self.turn_front
                and (island_open or self.hits >= CONFIRMATIONS))

    def corner_started(self):
        """Call it when the corner starts: the gate stays shut until the island is seen again."""
        self.armed = False
