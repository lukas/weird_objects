"""Fast color-based hexapod detector for bench camera footage.

The robot is the only object in the bench scene that is a dense cluster of
red AND blue 3D-printed parts. Red alone (battery wires, tape measure) and
blue alone (tape on the floor sheet) both occur elsewhere, so the detector
scores connected regions of *red-blue proximity overlap*: dilate the red and
blue masks and look for places where both are present. Only the robot
produces a large overlap blob.

Runs at several hundred fps on a Mac CPU at the default work width.
"""

from dataclasses import dataclass

import cv2
import numpy as np

VERDICT_FULL = "full"          # robot found, bbox clear of frame edges
VERDICT_PARTIAL = "partial"    # robot found but touching a frame edge
VERDICT_NOT_VISIBLE = "not_visible"


@dataclass
class Detection:
    verdict: str
    # bbox in full-resolution pixel coords (x0, y0, x1, y1), or None
    box: tuple | None
    # overlap blob area in work-resolution pixels (confidence proxy)
    score: float


class RobotDetector:
    def __init__(
        self,
        work_width: int = 320,
        min_overlap_px: int = 40,
        edge_margin_frac: float = 0.015,
        sat_min: int = 110,
        val_min: int = 60,
    ):
        self.work_width = work_width
        self.min_overlap_px = min_overlap_px
        self.edge_margin_frac = edge_margin_frac
        self.sat_min = sat_min
        self.val_min = val_min
        # red/blue parts sit within ~1 body radius of each other
        self._prox_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17))
        self._grow_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41))
        self._open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    def masks(self, frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """Red and blue masks at work resolution, plus the scale factor."""
        scale = self.work_width / frame_bgr.shape[1]
        small = cv2.resize(
            frame_bgr,
            (self.work_width, max(1, round(frame_bgr.shape[0] * scale))),
            interpolation=cv2.INTER_AREA,
        )
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        sv = (s > self.sat_min) & (v > self.val_min)
        red = (((h <= 8) | (h >= 172)) & sv).astype(np.uint8)
        blue = ((h >= 100) & (h <= 130) & sv).astype(np.uint8)
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, self._open_kernel)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, self._open_kernel)
        return red, blue, scale

    def detect(self, frame_bgr: np.ndarray) -> Detection:
        red, blue, scale = self.masks(frame_bgr)

        red_d = cv2.dilate(red, self._prox_kernel)
        blue_d = cv2.dilate(blue, self._prox_kernel)
        overlap = red_d & blue_d

        n, labels, stats, _ = cv2.connectedComponentsWithStats(overlap, connectivity=8)
        if n < 2:
            return Detection(VERDICT_NOT_VISIBLE, None, 0.0)
        areas = stats[1:, cv2.CC_STAT_AREA]
        best = 1 + int(np.argmax(areas))
        score = float(areas[best - 1])
        if score < self.min_overlap_px:
            return Detection(VERDICT_NOT_VISIBLE, None, score)

        # grow the winning overlap blob and collect the raw red/blue pixels
        # that belong to the robot (legs stick out past the overlap zone)
        seed = (labels == best).astype(np.uint8)
        region = cv2.dilate(seed, self._grow_kernel)
        robot_px = ((red | blue) & region).astype(bool)
        ys, xs = np.nonzero(robot_px)
        x0, x1 = xs.min(), xs.max()
        y0, y1 = ys.min(), ys.max()

        hh, ww = red.shape
        margin = max(2, round(self.edge_margin_frac * ww))
        partial = x0 <= margin or y0 <= margin or x1 >= ww - 1 - margin or y1 >= hh - 1 - margin
        box = tuple(round(c / scale) for c in (x0, y0, x1 + 1, y1 + 1))
        return Detection(VERDICT_PARTIAL if partial else VERDICT_FULL, box, score)


def annotate(frame_bgr: np.ndarray, det: Detection) -> np.ndarray:
    out = frame_bgr.copy()
    color = {
        VERDICT_FULL: (0, 200, 0),
        VERDICT_PARTIAL: (0, 165, 255),
        VERDICT_NOT_VISIBLE: (0, 0, 255),
    }[det.verdict]
    if det.box is not None:
        x0, y0, x1, y1 = det.box
        cv2.rectangle(out, (x0, y0), (x1, y1), color, 2)
    cv2.putText(out, det.verdict, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    return out
