#!/usr/bin/env python3
"""Fit J, f, b from wheel_inertia.py CSVs: tau = J*a + f + b*w (w > 0 only, 3 < w < W_HI).
a: Savitzky-Golay derivative (5 points, 2 ms) inside each segment. tau from the command (1.27 * I_cmd) and,
separately, from the drive's reported torque (I_fb * 1.27). Coast segments: tau = 0."""
import sys
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

KT, W_LO, W_HI = 1.27, 3.0, 11.0


def rows(path):
    d = pd.read_csv(path)
    out = []
    for _, g in d.groupby((d.seg != d.seg.shift()).cumsum()):
        s = g.seg.iloc[0]
        if not (s.startswith('I') or s.startswith('coast')) or len(g) < 7:
            continue
        t, w = g.t.values, g.w.values
        dt = np.median(np.diff(t))
        a = savgol_filter(w, 5, 2, deriv=1, delta=dt)
        m = (w > W_LO) & (w < W_HI)
        m[:2] = m[-2:] = False                     # edge samples of the filter / the switch instant
        cur = 0.0 if s.startswith('coast') else float(s[1:5])
        for wi, ai, ic, ifb in zip(w[m], a[m], g.I_cmd.values[m], g.I_fb.values[m]):
            out.append((s.split('_')[0], cur, wi, ai, KT * ic, KT * ifb))
    return pd.DataFrame(out, columns=['seg', 'I', 'w', 'a', 'tau_cmd', 'tau_fb'])


def fit(df, col):
    A = np.column_stack([df.a, np.ones(len(df)), df.w])
    x, *_ = np.linalg.lstsq(A, df[col].values, rcond=None)
    res = df[col].values - A @ x
    return x, float(np.sqrt(np.mean(res ** 2)))


for path in sys.argv[1:]:
    df = rows(path)
    print(f'== {path.split("/")[-1]}  samples push {int((df.I > 0).sum())}, coast {int((df.I == 0).sum())}')
    for col in ('tau_cmd', 'tau_fb'):
        (J, f, b), rms = fit(df, col)
        print(f'  {col:7s}: J {J:.3e} kg.m2  f {f:+.3f} N.m  b {b:+.2e} N.m.s/rad  rms {rms:.3f} N.m')
    co = df[df.I == 0]
    for i in sorted(df.I[df.I > 0].unique()):
        sub = pd.concat([df[df.I == i], co])
        (J, f, b), rms = fit(sub, 'tau_cmd')
        (Jf, _, _), _ = fit(sub, 'tau_fb')
        p = df[df.I == i]
        print(f'  I {i:.1f} A: J(cmd) {J:.3e}  J(fb) {Jf:.3e}   push a mean {p.a.mean():6.0f} rad/s2, I_fb/I_cmd {(p.tau_fb / p.tau_cmd).mean():.2f}')
    c = co
    print(f'  coast: a mean {c.a.mean():.1f} rad/s2 over w {c.w.min():.1f}-{c.w.max():.1f}')


# ---- friction-cancelling estimate (used for the report): per push segment, slope of w(t) over W_LO < w < W_HI
# (linear fit, drive current rise excluded by the W_LO cut); coast decel a_c(w) = c0 + c1*w fitted on all coasts;
# J = tau / (a_push - a_c(w_mean)) -> Coulomb + viscous friction cancel at the same speed.
def seg_slopes(path):
    d = pd.read_csv(path)
    push, coast_w, coast_a = [], [], []
    for _, g in d.groupby((d.seg != d.seg.shift()).cumsum()):
        s = g.seg.iloc[0]
        m = (g.w > W_LO) & (g.w < W_HI)
        if m.sum() < 3:
            continue
        t, w = g.t.values[m], g.w.values[m]
        if s.startswith('I'):
            sl = np.polyfit(t, w, 1)[0]
            push.append((float(s[1:5]), sl, w.mean(), (g.I_fb.values[m] / g.I_cmd.values[m]).mean(), m.sum()))
        elif s.startswith('coast'):
            a = savgol_filter(g.w.values, 21, 2, deriv=1, delta=np.median(np.diff(g.t.values)))[m.values]
            coast_w += list(w); coast_a += list(a)
    c1, c0 = np.polyfit(coast_w, coast_a, 1)
    return pd.DataFrame(push, columns=['I', 'a', 'w', 'fb_ratio', 'n']), c0, c1


if __name__ == '__main__':
    print('\n==== friction-cancelling J = tau / (a_push - a_coast(w)) ====')
    for path in sys.argv[1:]:
        p, c0, c1 = seg_slopes(path)
        p['J_cmd'] = KT * p.I / (p.a - (c0 + c1 * p.w))
        p['J_fb'] = p.J_cmd * p.fb_ratio
        Jbar = p.J_cmd.mean()
        print(f'{path.split("/")[-1]}: coast a = {c0:.1f} {c1:+.2f}*w rad/s2 -> f/J {-c0:.1f}, b/J {-c1:.2f};'
              f' pushes {len(p)} ({p.n.mean():.1f} samples each)')
        for i, g in p.groupby('I'):
            print(f'   I {i:.1f} A: J_cmd {g.J_cmd.mean():.3e} +- {g.J_cmd.std():.1e}  (J_fb {g.J_fb.mean():.3e}, I_fb/I_cmd {g.fb_ratio.mean():.2f})')
        print(f'   all: J_cmd {Jbar:.3e} +- {p.J_cmd.std():.1e}, J_fb {p.J_fb.mean():.3e};  f = {-c0 * Jbar:.3f} N.m, b = {-c1 * Jbar:.2e} N.m.s/rad')
