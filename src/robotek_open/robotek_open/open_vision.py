"""Infer the first turn from the coloured field lines, without a world pose."""

import cv2
import numpy as np

from .open_constants import SENSORS


class OpenVision:
    """Debounce geometrical line evidence from independent camera frames."""

    def __init__(self, min_length=40.0, min_confidence=0.25, confirmations=5,
                 blue_hsv=(90, 135, 60, 60), orange_hsv=(5, 28, 60, 60),
                 floor_mask=False, wall_v_max=40, wall_min_rows=8):
        self.min_length = min_length
        self.min_confidence = min_confidence
        self.confirmations = confirmations
        # (h_min, h_max, s_min, v_min); h_min > h_max wraps through red (H 179 -> 0).
        self.blue_hsv = tuple(blue_hsv)
        self.orange_hsv = tuple(orange_hsv)
        self.floor_mask = floor_mask
        self.wall_v_max = wall_v_max
        self.wall_min_rows = wall_min_rows
        self.blue_length = 0.0
        self.orange_length = 0.0
        self.confidence = 0.0
        self.candidate = 0
        self.direction = 0
        self.hits = 0
        self.stamp = None
        self.overlay = None

    @classmethod
    def from_config(cls, cfg):
        """The detector with the camera settings of an OpenConfig (HSV ranges, floor mask, evidence)."""
        return cls(
            SENSORS.vision_min_length, SENSORS.vision_min_confidence, cfg.direction_confirmations,
            blue_hsv=(cfg.vision_blue_h_min, cfg.vision_blue_h_max,
                      cfg.vision_blue_s_min, cfg.vision_blue_v_min),
            orange_hsv=(cfg.vision_orange_h_min, cfg.vision_orange_h_max,
                        cfg.vision_orange_s_min, cfg.vision_orange_v_min),
            floor_mask=cfg.vision_floor_mask, wall_v_max=cfg.vision_wall_v_max,
            wall_min_rows=cfg.vision_wall_min_rows)

    @staticmethod
    def line_length(mask):
        """Measure the longest elongated colour component, in image pixels."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lengths = []
        for contour in contours:
            if cv2.contourArea(contour) < 25:
                continue
            _, dimensions, _ = cv2.minAreaRect(contour)
            short, long = sorted(dimensions)
            if long >= 3 * max(short, 1.0):
                lengths.append(long)
        return max(lengths, default=0.0)

    @staticmethod
    def colour_mask(hsv, h_min, h_max, s_min, v_min):
        if h_min <= h_max:
            return cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, 255, 255]))
        return (cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([179, 255, 255]))
                | cv2.inRange(hsv, np.array([0, s_min, v_min]), np.array([h_max, 255, 255])))

    def floor_rows(self, hsv):
        """First floor row per column: below the lowest dark band (the black field wall).

        Everything above it (wall, room, people) cannot be a field line. Columns
        without a wall band keep the whole image.
        """
        dark = (hsv[..., 2] <= self.wall_v_max).astype(np.uint8)
        # Keep only dark runs at least wall_min_rows tall: thin printed lines are not walls.
        dark &= cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((self.wall_min_rows, 1), np.uint8))
        rows = np.arange(dark.shape[0])[:, None]
        lowest = np.where(dark != 0, rows, -1).max(axis=0)
        # Bridge short gaps in the wall band (glare, the island's end) with neighbouring columns.
        lowest = cv2.dilate(lowest.astype(np.float32)[None, :], np.ones((1, 15), np.uint8))[0]
        return lowest.astype(int) + 1

    def update(self, bgr, stamp):
        """Use each frame once; absent or balanced lines cannot choose a turn."""
        if self.stamp is not None and stamp <= self.stamp:
            return
        self.stamp = stamp
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        blue = self.colour_mask(hsv, *self.blue_hsv)
        orange = self.colour_mask(hsv, *self.orange_hsv)
        kernel = np.ones((3, 5), dtype=np.uint8)
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel)
        orange = cv2.morphologyEx(orange, cv2.MORPH_CLOSE, kernel)
        # The downward camera sees the distant lines near the image's top.
        # Only remove the top strip; a lower-half ROI loses both lines.
        top = int(bgr.shape[0] * 0.08)
        blue[:top] = 0
        orange[:top] = 0
        floor = None
        if self.floor_mask:
            floor = self.floor_rows(hsv)
            above = np.arange(bgr.shape[0])[:, None] < floor[None, :]
            blue[above] = 0
            orange[above] = 0
        self.blue_length = self.line_length(blue)
        self.orange_length = self.line_length(orange)
        longest = max(self.blue_length, self.orange_length)
        self.confidence = abs(self.blue_length - self.orange_length) / max(longest, 1.0)
        candidate = 0
        if longest >= self.min_length and self.confidence >= self.min_confidence:
            candidate = 1 if self.blue_length > self.orange_length else -1
        self.hits = self.hits + 1 if candidate and candidate == self.candidate else 0
        if candidate and candidate != self.candidate:
            self.hits = 1
        self.candidate = candidate
        self.direction = candidate if self.hits >= self.confirmations else 0
        self.overlay = bgr.copy()
        # Darken what the detector ignores (the top strip and, with the floor mask, everything above the black wall),
        # so the image shows that only the floor below the wall counts.
        ignored = np.zeros(bgr.shape[:2], dtype=bool)
        ignored[:top] = True
        if floor is not None:
            ignored |= above
        self.overlay[ignored] //= 4
        self.overlay[blue != 0] = (255, 0, 0)
        self.overlay[orange != 0] = (0, 128, 255)
        cv2.line(self.overlay, (0, top), (bgr.shape[1] - 1, top), (0, 255, 0), 1)
        if floor is not None:
            points = np.stack([np.arange(bgr.shape[1]), np.minimum(floor, bgr.shape[0] - 1)], axis=1)
            cv2.polylines(self.overlay, [points.astype(np.int32)], False, (0, 255, 255), 2)
        text = (f'B={self.blue_length:.0f} O={self.orange_length:.0f} '
                f'conf={self.confidence:.2f} dir={self.direction:+d}')
        cv2.putText(self.overlay, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(self.overlay, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
