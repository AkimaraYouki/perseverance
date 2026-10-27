"""Camera step watch (2026-10-27, desktop): 'there is probably a step ~d m ahead' from the robot camera, for the automatic
step-detect mode while following (auto_step). No ROS / GPU; the same code runs on the robot (person_tracker) and in the
Isaac test (climb_test --auto_step).

Cue: a long, near-horizontal edge across the robot's path on the floor ahead (the foot of a step riser: plywood on the
floor). Each edge pixel -> ground ray with the calibrated lens model (tracking_core.BearingModel.ray) + the camera height
and body pitch -> distance. A step is reported when the same floor place (odometry x + distance) is seen in >= need of
the last window frames. It only has to be roughly right: the jump itself still starts from the wheel contact
(step-detect mode). Floor seams / shadows may slow the robot down for nothing — accepted.
"""
import math

import cv2
import numpy as np


class StepWatch:
    def __init__(self, bearing_model, max_dist=2.5, min_len_frac=0.25, max_tilt_deg=12.0, path_half_deg=10.0,
                 need=3, window=6, cluster_m=0.15, canny=(40, 120), contrast_min=28.0, band=(3, 9)):
        self.bm = bearing_model
        self.max_dist, self.min_len, self.max_tilt = max_dist, min_len_frac, math.radians(max_tilt_deg)
        self.path_half = math.radians(path_half_deg)
        self.need, self.window, self.cluster = need, window, cluster_m
        self.canny = canny
        # a step's foot separates two different surfaces (plywood above, floor below); a floor seam / grid line has the
        # same floor on both sides (2026-10-27 Isaac: the ground grid lines were taken for steps)
        self.contrast_min, self.band = contrast_min, band
        self.hist = []                 # per frame: list of world x of candidate step feet
        self.last = []                 # this frame's candidates (d, bearing span, (u0, v0, u1, v1)) for drawing

    def rays(self, bgr):
        """Candidate step feet this frame -> [(depression below the camera axis [rad], (b_left, b_right) [rad], line px)]."""
        H, W = bgr.shape[:2]
        g = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        e = cv2.Canny(g, *self.canny)
        e[: int(H * 0.35)] = 0                                     # sky / walls / the person's upper body
        lines = cv2.HoughLinesP(e, 1, np.pi / 180, threshold=50, minLineLength=int(self.min_len * W), maxLineGap=12)
        out = []
        for ln in (lines.reshape(-1, 4) if lines is not None else []):
            u0, v0, u1, v1 = [float(x) for x in ln]
            if abs(math.atan2(v1 - v0, u1 - u0 + 1e-9)) > self.max_tilt and abs(math.atan2(v1 - v0, u1 - u0 + 1e-9)) < math.pi - self.max_tilt:
                continue
            b0, _ = self.bm.ray(u0, v0)
            b1, _ = self.bm.ray(u1, v1)
            lo, hi = min(b0, b1), max(b0, b1)
            if lo > -self.path_half / 2 or hi < self.path_half / 2:    # must cross the robot's path
                continue
            if self._contrast(bgr, u0, v0, u1, v1) < self.contrast_min:
                continue
            _, dep = self.bm.ray(0.5 * (u0 + u1), 0.5 * (v0 + v1))
            out.append((dep, (hi, lo), (u0, v0, u1, v1)))
        return out

    def to_dist(self, rays, cam_h, pitch):
        """Rays (depression below the camera axis) -> ground distances with the camera height and body pitch (+ nose down)."""
        out = []
        for dep, span, ln in rays:
            dep += pitch
            if dep < math.radians(2.0):
                continue
            d = cam_h / math.tan(dep)
            if 0.15 < d < self.max_dist:
                out.append((d, span, ln))
        return out

    def _contrast(self, bgr, u0, v0, u1, v1):
        """Mean colour difference between thin bands just above and just below the line (BGR, 0-255)."""
        H, W = bgr.shape[:2]
        a, b = self.band
        up, dn = [], []
        for s_ in np.linspace(0.1, 0.9, 15):
            u, v = int(u0 + s_ * (u1 - u0)), int(v0 + s_ * (v1 - v0))
            if v - b < 0 or v + b >= H or not 0 <= u < W:
                continue
            up.append(bgr[v - b:v - a, u].reshape(-1, 3).mean(0))
            dn.append(bgr[v + a:v + b, u].reshape(-1, 3).mean(0))
        if len(up) < 5:
            return 0.0
        return float(np.linalg.norm(np.median(up, 0) - np.median(dn, 0)))

    def update(self, bgr, cam_h, pitch, odom_x):
        """Feed one frame. Returns the distance [m] to the nearest confirmed step foot ahead, or None."""
        c = self.to_dist(self.rays(bgr), cam_h, pitch)
        self.last = c
        return self.feed([d for d, _, _ in c], odom_x)

    def feed(self, dists, odom_x):
        """Temporal check of one frame's distances (robot: person_tracker publishes the rays, person_follow_cam the rest)."""
        self.hist.append([odom_x + d for d in dists])
        self.hist = self.hist[-self.window:]
        pts = np.array([x for fr in self.hist for x in fr])
        if len(pts) == 0:
            return None
        best = None
        for x in np.sort(pts):
            if x <= odom_x + 0.05:
                continue
            frames = sum(any(abs(y - x) < self.cluster for y in fr) for fr in self.hist)
            if frames >= self.need:
                best = x
                break                                              # nearest first
        return None if best is None else float(best - odom_x)

    def draw(self, bgr, d_conf=None):
        for d, _, (u0, v0, u1, v1) in self.last:
            cv2.line(bgr, (int(u0), int(v0)), (int(u1), int(v1)), (0, 200, 255), 2)
            cv2.putText(bgr, f"{d:.2f}", (int(u0), int(v0) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if d_conf is not None:
            cv2.putText(bgr, f"STEP {d_conf:.2f} m", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return bgr
