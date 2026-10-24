"""Camera-first person tracking core (no ROS, no GPU): ByteTrack multi-person tracker, designated-target manager with
appearance re-identification, camera bearing models. Used by person_tracker (robot GPU node); tested on the desktop
(test/test_tracking_core.py).

ByteTrack (Zhang et al., ECCV 2022): Kalman box tracks; detections split by score — high ones are matched first (IoU),
then the remaining tracks get a second chance with the low-score ones (a half-occluded / blurred person keeps its id).
Target: one track id is 'the person'. Its appearance (OSNet embedding, unit vector) is kept as a small gallery. While the
id lives it is followed without ReID; when the id is lost, or when another person overlaps it (ids may swap), the
gallery decides: the confirmed track most similar to the gallery (>= sim_min and clearly better than the second) becomes
the target again.
"""
import math

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:                                   # greedy fallback (few people: same answer almost always)
    linear_sum_assignment = None


# --------------------------------------------------------------------------------------------- geometry
def iou_matrix(a, b):
    """a (N, 4), b (M, 4) boxes x0 y0 x1 y1 -> IoU (N, M)."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    a, b = np.asarray(a, float), np.asarray(b, float)
    x0 = np.maximum(a[:, None, 0], b[None, :, 0]); y0 = np.maximum(a[:, None, 1], b[None, :, 1])  # noqa: E702
    x1 = np.minimum(a[:, None, 2], b[None, :, 2]); y1 = np.minimum(a[:, None, 3], b[None, :, 3])  # noqa: E702
    inter = np.clip(x1 - x0, 0, None) * np.clip(y1 - y0, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / np.maximum(area_a[:, None] + area_b[None, :] - inter, 1e-9)


def assign(cost, thresh):
    """Min-cost matching with cost <= thresh -> (matches [(i, j)], unmatched rows, unmatched cols)."""
    n, m = cost.shape
    if n == 0 or m == 0:
        return [], list(range(n)), list(range(m))
    if linear_sum_assignment is not None:
        r, c = linear_sum_assignment(cost)
        pairs = [(i, j) for i, j in zip(r, c) if cost[i, j] <= thresh]
    else:
        pairs, used_r, used_c = [], set(), set()
        for k in np.argsort(cost, axis=None):
            i, j = divmod(int(k), m)
            if cost[i, j] > thresh:
                break
            if i not in used_r and j not in used_c:
                pairs.append((i, j)); used_r.add(i); used_c.add(j)  # noqa: E702
    mr, mc = {i for i, _ in pairs}, {j for _, j in pairs}
    return pairs, [i for i in range(n) if i not in mr], [j for j in range(m) if j not in mc]


class BearingModel:
    """Image column -> horizontal bearing [rad], + left. 'pinhole': atan((cx - u) / fx). 'equidistant' (fisheye,
    e.g. 160 deg lenses): theta = (cx - u) / f, f = (W / 2) / (hfov / 2). 'opencv_fisheye': cv2.fisheye with K, D
    (calibrated). Wide lenses with the pinhole formula put people at the edges 10-30 deg off -> LiDAR/camera mismatch."""

    def __init__(self, model='pinhole', width=640, height=480, fx=530.0, cx=None, hfov_deg=160.0, K=None, D=None):
        self.model, self.W, self.H = model, width, height
        self.cx = width / 2.0 if cx is None else cx
        self.fx = fx
        self.f_eq = (width / 2.0) / (math.radians(hfov_deg) / 2.0)
        self.K = None if K is None else np.asarray(K, float).reshape(3, 3)
        self.D = None if D is None else np.asarray(D, float).reshape(4, 1)

    def bearing(self, u, v=None):
        if self.model == 'equidistant':
            return (self.cx - u) / self.f_eq
        if self.model == 'opencv_fisheye' and self.K is not None:
            import cv2
            p = cv2.fisheye.undistortPoints(np.array([[[u, self.H / 2.0 if v is None else v]]], np.float64), self.K, self.D)
            return -math.atan(float(p[0, 0, 0]))
        return math.atan2(self.cx - u, self.fx)


# --------------------------------------------------------------------------------------------- Kalman box
class KalmanBox:
    """ByteTrack's constant-velocity filter on (cx, cy, aspect w/h, h)."""
    std_pos, std_vel = 1.0 / 20, 1.0 / 160

    def __init__(self, box):
        z = self._z(box)
        self.x = np.r_[z, np.zeros(4)]
        h = z[3]
        s = np.array([2 * self.std_pos * h, 2 * self.std_pos * h, 1e-2, 2 * self.std_pos * h,
                      10 * self.std_vel * h, 10 * self.std_vel * h, 1e-5, 10 * self.std_vel * h])
        self.P = np.diag(s ** 2)
        self.F = np.eye(8)
        self.F[:4, 4:] = np.eye(4)
        self.Hm = np.eye(4, 8)

    @staticmethod
    def _z(b):
        w, h = b[2] - b[0], b[3] - b[1]
        return np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2, w / max(h, 1e-6), h])

    def predict(self):
        h = self.x[3]
        q = np.array([self.std_pos * h, self.std_pos * h, 1e-2, self.std_pos * h,
                      self.std_vel * h, self.std_vel * h, 1e-5, self.std_vel * h])
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + np.diag(q ** 2)

    def update(self, box):
        z = self._z(box)
        h = self.x[3]
        R = np.diag(np.array([self.std_pos * h, self.std_pos * h, 1e-1, self.std_pos * h]) ** 2)
        S = self.Hm @ self.P @ self.Hm.T + R
        K = self.P @ self.Hm.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.Hm @ self.x)
        self.P = (np.eye(8) - K @ self.Hm) @ self.P

    def box(self):
        cx, cy, a, h = self.x[:4]
        w = a * h
        return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])


class Track:
    _next = 1

    def __init__(self, box, score, frame, extra):
        self.id = Track._next
        Track._next += 1
        self.kf = KalmanBox(box)
        self.score, self.extra = score, extra
        self.hits, self.start, self.last = 1, frame, frame
        self.confirmed = False

    @property
    def box(self):
        return self.kf.box()


class ByteTracker:
    """ByteTrack: high (>= high) detections first, then low (>= low) for the leftovers; new tracks from unmatched high
    detections (confirmed after min_hits frames); tracks kept max_lost frames without a detection."""

    def __init__(self, high=0.5, low=0.1, new=0.6, match_iou=0.2, low_iou=0.5, max_lost=30, min_hits=2):
        self.high, self.low, self.new = high, low, new
        self.c1, self.c2 = 1.0 - match_iou, 1.0 - low_iou
        self.max_lost, self.min_hits = max_lost, min_hits
        self.tracks, self.frame = [], 0

    def update(self, boxes, scores, extras=None):
        """boxes (N, 4), scores (N,), extras: per detection payload (kept on the matched track). -> live tracks
        (matched this frame first)."""
        self.frame += 1
        boxes = np.asarray(boxes, float).reshape(-1, 4)
        scores = np.asarray(scores, float).reshape(-1)
        extras = extras if extras is not None else [None] * len(scores)
        for t in self.tracks:
            t.kf.predict()
        hi = np.flatnonzero(scores >= self.high)
        lo = np.flatnonzero((scores >= self.low) & (scores < self.high))
        T = self.tracks
        pairs, ut, ud = assign(1.0 - iou_matrix([t.box for t in T], boxes[hi]), self.c1)
        for i, j in pairs:
            self._hit(T[i], boxes[hi[j]], scores[hi[j]], extras[hi[j]])
        rest = [T[i] for i in ut]
        pairs2, ut2, _ = assign(1.0 - iou_matrix([t.box for t in rest], boxes[lo]), self.c2)
        for i, j in pairs2:
            self._hit(rest[i], boxes[lo[j]], scores[lo[j]], extras[lo[j]])
        for j in ud:
            d = hi[j]
            if scores[d] >= self.new:
                self.tracks.append(Track(boxes[d], scores[d], self.frame, extras[d]))
        self.tracks = [t for t in self.tracks if self.frame - t.last <= (self.max_lost if t.confirmed else 2)]
        return sorted([t for t in self.tracks if t.confirmed or t.hits >= self.min_hits],
                      key=lambda t: t.last != self.frame)

    def _hit(self, t, box, score, extra):
        t.kf.update(box)
        t.score, t.extra, t.last = score, extra, self.frame
        t.hits += 1
        if t.hits >= self.min_hits:
            t.confirmed = True


# --------------------------------------------------------------------------------------------- target
class TargetManager:
    """Which track is 'the person'. register(): the confirmed, currently seen track nearest to straight ahead (bearing),
    larger boxes preferred. Gallery = up to n_gallery unit embeddings (oldest replaced), added every gallery_every frames
    while the target is seen alone (no other box overlapping). Re-ID when lost or after an overlap."""

    def __init__(self, sim_min=0.65, sim_margin=0.08, sim_single=0.75, n_gallery=12, gallery_every=10, overlap_iou=0.15,
                 gate_deg=12.0, gate_rate_deg_s=40.0, gate_max_deg=70.0, fps=30.0):
        self.sim_min, self.sim_margin, self.sim_single = sim_min, sim_margin, sim_single   # single = only one person in view
        self.n_gallery, self.every, self.ov = n_gallery, gallery_every, overlap_iou
        # where it may reappear: within gate_deg + gate_rate x lost time of the bearing it was last seen at (same uniforms:
        # appearance alone cannot tell two soldiers apart, 2026-10-25 test sim 0.99 -> swapped to the other one)
        self.g0, self.grate, self.gmax, self.fps = math.radians(gate_deg), math.radians(gate_rate_deg_s), math.radians(gate_max_deg), fps
        self.clear()

    def clear(self):
        self.id, self.gallery, self.k, self.state = None, [], 0, 'NONE'
        self.crossed = False           # another person overlapped the target since the last ReID check
        self.lost_frames = 0
        self.want = False
        self.last_b = None

    def register(self):
        self.clear()
        self.want = True

    def similarity(self, e):
        if not self.gallery or e is None:
            return None
        s = np.sort(np.asarray(self.gallery) @ e)[::-1]
        return float(np.mean(s[:3]))                  # top-3 mean: one lucky gallery match is not enough

    def need_embeddings(self, tracks, frame):
        """Ids whose embedding the caller should compute this frame (keeps ReID cheap: only when it matters)."""
        if self.want:
            return []
        if self.id is None or self.state == 'LOST' or self.crossed:
            return [t.id for t in tracks if t.confirmed and t.last == frame]
        tg = next((t for t in tracks if t.id == self.id and t.last == frame), None)
        if tg is not None and (len(self.gallery) < 3 or self.k % self.every == 0) and not self._overlapped(tg, tracks):
            return [tg.id]
        return []

    def _overlapped(self, tg, tracks):
        others = [t.box for t in tracks if t.id != tg.id and t.last == tg.last]
        return bool(others) and float(iou_matrix([tg.box], others).max()) > self.ov

    def update(self, tracks, frame, bearing_of, emb=None):
        """tracks: live tracks; bearing_of(track) -> rad; emb: {id: unit vector} for the ids asked for.
        Returns the target track (seen this frame) or None."""
        emb = emb or {}
        seen = [t for t in tracks if t.confirmed and t.last == frame]
        self.k += 1
        if self.want:
            if seen:
                tg = min(seen, key=lambda t: abs(bearing_of(t)) - 0.002 * (t.box[3] - t.box[1]))
                self.id, self.state, self.want, self.lost_frames = tg.id, 'LOCK', False, 0
                return tg
            self.state = 'NONE'
            return None
        if self.id is None:
            return None
        tg = next((t for t in seen if t.id == self.id), None)
        if tg is not None and self._overlapped(tg, tracks):
            self.crossed = True
        # ReID decides when lost, or after a crossing (ids may have swapped)
        if (tg is None or self.crossed) and self.gallery and emb:
            gate = min(self.gmax, self.g0 + self.grate * self.lost_frames / self.fps)
            near = {t.id for t in seen if self.last_b is None or abs(bearing_of(t) - self.last_b) <= gate}
            sims = sorted(((self.similarity(e), i) for i, e in emb.items() if e is not None and i in near), reverse=True)
            if sims:
                best, bid = sims[0]
                second = sims[1][0] if len(sims) > 1 else -1.0
                tb = next((t for t in seen if t.id == bid), None)
                need = self.sim_min if len(sims) > 1 else self.sim_single   # alone in view: someone else must not look 'close enough'
                if tb is not None and best >= need and best - second >= self.sim_margin and not self._overlapped(tb, tracks):
                    if bid != self.id:
                        self.state = 'REACQUIRED'
                    self.id, tg, self.crossed = bid, tb, False
        if tg is None:
            self.lost_frames += 1
            self.state = 'LOST'
            return None
        self.lost_frames = 0
        self.last_b = bearing_of(tg)
        if self.state != 'REACQUIRED' or self.k % 15 == 0:
            self.state = 'LOCK'
        e = emb.get(tg.id)
        if e is not None and not self.crossed and not self._overlapped(tg, tracks):
            if len(self.gallery) < self.n_gallery:
                self.gallery.append(e)
            else:
                self.gallery[(self.k // max(1, self.every)) % self.n_gallery] = e
        return tg


def l2n(v):
    v = np.asarray(v, float)
    return v / max(float(np.linalg.norm(v)), 1e-9)
