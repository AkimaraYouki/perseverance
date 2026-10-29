"""Desktop test of tracking_core (no ROS / GPU): two people cross in front of the camera; the target id must survive the
crossing (ReID decides after the overlap), short detector dropouts and a 1 s full occlusion with re-acquisition."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'gen2_tools'))
import tracking_core as tc  # noqa: E402


def scene(T=240, seed=0):
    """Person A walks left -> right, person B right -> left (they cross at frame ~120); A hidden frames 170-200."""
    rng = np.random.default_rng(seed)
    for k in range(T):
        dets = []
        xa, xb = 100 + 3.0 * k, 560 - 2.0 * k
        if not (170 <= k < 200):
            dets.append(('A', [xa - 30, 120, xa + 30, 360], 0.85 if rng.random() > 0.1 else 0.3))
        dets.append(('B', [xb - 32, 110, xb + 32, 370], 0.8))
        yield k, dets


def run(b_vec, label):
    emb = {'A': tc.l2n(np.r_[1.0, 0.2, np.zeros(14)]), 'B': tc.l2n(b_vec)}
    rng = np.random.default_rng(1)
    bt, tm = tc.ByteTracker(), tc.TargetManager()
    bm = tc.BearingModel('equidistant', 640, 480, hfov_deg=160)
    who = {}
    ok = bad = lost = 0
    for k, dets in scene():
        boxes = [d[1] for d in dets]
        tracks = bt.update(boxes, [d[2] for d in dets], extras=[d[0] for d in dets])
        if k == 3:
            tm.register()                                  # A is nearer the centre at frame 3? (A at x 109, B at 554)
        ids = tm.need_embeddings(tracks, bt.frame)
        e = {t.id: tc.l2n(emb[t.extra] + 0.05 * rng.standard_normal(16)) for t in tracks if t.id in ids}
        tg = tm.update(tracks, bt.frame, lambda t: bm.bearing((t.box[0] + t.box[2]) / 2), e)
        if k == 4:
            who['target'] = tg.extra if tg is not None else None
        if k > 4:
            if tg is None:
                lost += 1
            elif tg.extra == who['target']:
                ok += 1
            else:
                bad += 1
    print(f"[{label}] registered {who['target']}  frames on target {ok}, on the wrong person {bad}, lost {lost}")
    assert who['target'] == 'A' and bad == 0, 'followed the wrong person'
    assert lost < 45, 'lost too long'


def scene_front(speed, T=200):
    """A far near the centre (small box); B near walks across in front (big box) and hides A completely (2026-10-29)."""
    for k in range(T):
        xa, xb, dets = 320 + 0.5 * k, 620 - speed * k, []
        if abs(xb - xa) >= 55:
            dets.append(('A', [xa - 20, 200, xa + 20, 320], 0.8))
        if -60 < xb < 700:
            dets.append(('B', [xb - 60, 90, xb + 60, 470], 0.85))
        yield k, dets


def run_front(b_vec, label, speed):
    emb = {'A': tc.l2n(np.r_[1.0, 0.2, np.zeros(14)]), 'B': tc.l2n(b_vec)}
    rng = np.random.default_rng(1)
    bt = tc.ByteTracker(max_lost=20, new=0.7)
    tm = tc.TargetManager(sim_min=0.72, sim_single=0.80, sim_margin=0.12, gate_deg=10, gate_rate_deg_s=30)
    bm = tc.BearingModel('equidistant', 640, 480, hfov_deg=124)
    bad = ok = 0
    for k, dets in scene_front(speed):
        tracks = bt.update([d[1] for d in dets], [d[2] for d in dets], extras=[d[0] for d in dets])
        if k == 2:
            tm.register()
        ids = tm.need_embeddings(tracks, bt.frame)
        e = {t.id: tc.l2n(emb[t.extra] + 0.05 * rng.standard_normal(16)) for t in tracks if t.id in ids}
        tg = tm.update(tracks, bt.frame, lambda t: bm.bearing((t.box[0] + t.box[2]) / 2), e)
        if k > 3 and tg is not None and tm.state in ('LOCK', 'REACQUIRED'):
            ok, bad = ok + (tg.extra == 'A'), bad + (tg.extra != 'A')
    print(f"[{label}, passer {speed} px/frame in front] frames on target {ok}, on the passer {bad}")
    assert bad == 0 and ok > 150, 'took the person who walked in front'


def main():
    run(np.r_[0.3, 1.0, np.zeros(14)], 'different clothes')
    run(np.r_[1.0, 0.35, np.zeros(14)], 'same uniform (sim 0.99)')      # only motion / bearing can tell them apart
    for sp in (4, 6, 10):                                                 # near passer hides a far target
        run_front(np.r_[0.2, 1.0, np.zeros(14)], 'different clothes', sp)
        run_front(np.r_[1.0, 0.25, np.zeros(14)], 'similar clothes', sp)
    bm = tc.BearingModel('equidistant', 640, 480, hfov_deg=160)
    # bearing models: 160 deg equidistant edge = 80 deg, pinhole fx 530 edge = 31 deg
    assert abs(np.degrees(bm.bearing(0)) - 80) < 1e-6
    assert abs(np.degrees(tc.BearingModel('pinhole', 640, 480, fx=530).bearing(0)) - 31.1) < 0.1
    print('ok')


if __name__ == '__main__':
    main()
