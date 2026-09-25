"""학습 중인 런의 건강 상태와 수렴 여부를 텐서보드 이벤트만 읽어 판정한다 (Isaac Sim 을 안 띄운다).

오리 로봇 리포(open_duck_mini_isaaclab/scripts/train_health.py)에서 가져왔다. 바꾼 것:
  * 수렴 신호. 이 과제는 지형 커리큘럼이라 Train/mean_reward, mean_episode_length 가 **짧은 에피소드 쪽으로
    치우친다**(금방 넘어지는 소수 env 가 짧은 에피소드를 쏟아낸다 — 2026-09-25 완주 83 % 인데 길이 180).
    그래서 env 기준 신호 세 가지가 모두 평평할 때만 수렴으로 본다:
      - Curriculum/terrain_levels           (지형 단계, 상대 변화)
      - Episode_Termination/time_out         (마지막 에피소드를 20 s 완주한 env 비율, 절대 변화)
      - Episode_Reward/track_lin_vel + track_ang_vel  (명령 추종, 상대 변화)
  * 비교는 양 끝을 창(win) 평균으로 한다 (두 점만 보면 잡음에 속는다 — 오리에서 겪음).
  * 수렴이 아니라 **퇴행**(지형 단계가 span 동안 떨어짐)도 경고한다 (2026-09-25 rough_modes3 에서 겪음).

오리의 경고는 그대로: 학습률이 adaptive 하한에 붙음, 행동 std 붕괴/과대.
판정은 곡선 기반이다. 성공 판정은 끝난 뒤 pv measure(행동 지표)와 pv play(눈)로 한다.

    isaaclab.sh -p train_health.py --run <런 디렉터리> [--span 400 --win 50 --min_iters 500]
마지막 줄: "STATUS CONVERGED" / "STATUS RUNNING" / "STATUS WAIT" (표본 부족). 종료코드 10 = 수렴.
"""

import argparse
import glob
import os
import sys

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

LR_FLOOR = 1.1e-5          # rsl_rl adaptive 스케줄 하한 1e-5
STD_COLLAPSE = 0.08
STD_TOO_WIDE = 0.75
TERRAIN_REL = 0.03         # 지형 단계 상대 변화 3 % 미만이면 평평
TIMEOUT_ABS = 0.02         # 완주 비율 2 %p 미만이면 평평
TRACK_REL = 0.02           # 명령 추종 상대 변화 2 % 미만이면 평평
REGRESS_REL = -0.10        # 지형 단계가 span 동안 10 % 넘게 떨어지면 퇴행 경고


def scalars(acc, tag):
    return [(s.step, s.value) for s in acc.Scalars(tag)] if tag in acc.Tags()["scalars"] else []


def window_mean(series, lo, hi):
    v = [x for s, x in series if lo <= s <= hi]
    return sum(v) / len(v) if v else None


def span_change(series, span, win):
    """(앞 창 평균, 뒤 창 평균). 표본이 모자라면 (None, None)."""
    if not series:
        return None, None
    last = series[-1][0]
    a = window_mean(series, last - span - win, last - span)
    b = window_mean(series, last - win, last)
    return a, b


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True)
    p.add_argument("--span", type=int, default=400, help="이 iter 폭 동안 평평하면 수렴")
    p.add_argument("--win", type=int, default=50, help="양 끝 평균 창")
    p.add_argument("--min_iters", type=int, default=500, help="이 런에서 최소 이만큼은 돌린 뒤에만 수렴 판정")
    p.add_argument("--min_timeout", type=float, default=0.5,
                   help="20 s 완주 env 비율이 이 값 이상일 때만 수렴으로 본다 (처음부터 학습할 때 '아무것도 못 배워서 평평' 을 수렴으로 오판하지 않게)")
    a = p.parse_args()

    files = sorted(glob.glob(os.path.join(a.run, "events.out.tfevents.*")))
    if not files:
        print("이벤트 파일 없음"); print("STATUS WAIT"); return 0
    acc = EventAccumulator(files[-1], size_guidance={"scalars": 0})
    acc.Reload()

    terr = scalars(acc, "Curriculum/terrain_levels")
    tout = scalars(acc, "Episode_Termination/time_out")
    tv, tw = scalars(acc, "Episode_Reward/track_lin_vel"), scalars(acc, "Episode_Reward/track_ang_vel")
    alive = scalars(acc, "Episode_Reward/alive")
    # Episode_Reward/* = 에피소드 합 / 20 s 라 짧은 에피소드에 끌려간다. alive(가중치 2, 매 초 2) 로 나눠
    # **초당** 추종 보상으로 정규화한다: (track_lin + track_ang) / (alive / 2) = 가중치 x 평균 추종 (0 ~ 3.5)
    track = [(s, (v + w) / (al / 2.0)) for (s, v), (_, w), (_, al) in zip(tv, tw, alive) if al > 1e-3]
    std = scalars(acc, "Policy/mean_std")
    lr = scalars(acc, "Loss/learning_rate")
    if not tout:
        print("아직 기록 없음"); print("STATUS WAIT"); return 0

    first, last = tout[0][0], tout[-1][0]
    ran = last - first
    print(f"[{os.path.basename(a.run)}]  iter {last}  (이 런에서 {ran} iter)")
    cur = lambda s: s[-1][1] if s else float("nan")  # noqa: E731
    print(f"  지형 단계 {cur(terr):.2f} | 20 s 완주 env {cur(tout):.3f} | 초당 추종 {cur(track):.3f} / 3.5"
          f" | std {cur(std):.3f} | lr {cur(lr):.2e}")

    warn = []
    if lr:
        # 순간값이 아니라 최근 100 iter 중 바닥 비율로 본다. adaptive 스케줄은 KL 에 따라 출렁여서 잠깐 바닥에 닿았다가
        # 돌아오는 건 정상이다 (r4: 최근 200 iter 중 24 % 가 바닥, 곧 1e-3 대로 회복). 오리의 경고 뜻은 "붙어서 안 떨어짐".
        recent = [v for _, v in lr[-100:]]
        frac = sum(v <= LR_FLOOR for v in recent) / len(recent)
        print(f"  학습률 바닥 비율 (최근 {len(recent)} iter) {frac*100:.0f} %")
        if frac >= 0.8:
            warn.append(f"학습률이 adaptive 하한({LR_FLOOR:.0e})에 최근 {frac*100:.0f} % 붙어 있다 — 실질 학습이 거의 멈춘 상태다.")
    if std and cur(std) < STD_COLLAPSE:
        warn.append(f"행동 std 붕괴({cur(std):.3f}) — 탐색이 사라졌다.")
    if std and cur(std) > STD_TOO_WIDE:
        warn.append(f"행동 std 과대({cur(std):.3f}) — 잡음이 신호를 덮는다.")

    status = "WAIT"
    if ran >= max(a.min_iters, a.span + a.win):
        t0, t1 = span_change(terr, a.span, a.win)
        o0, o1 = span_change(tout, a.span, a.win)
        k0, k1 = span_change(track, a.span, a.win)
        rel = lambda x0, x1: (x1 - x0) / abs(x0) if x0 else float("inf")  # noqa: E731
        tr, oa, kr = rel(t0, t1), o1 - o0, rel(k0, k1)
        flat_t, flat_o, flat_k = abs(tr) < TERRAIN_REL, abs(oa) < TIMEOUT_ABS, abs(kr) < TRACK_REL
        print(f"  최근 {a.span} iter 변화 (양 끝 {a.win} iter 평균):")
        print(f"    지형 단계 {t0:.2f} -> {t1:.2f} ({tr*100:+.1f} %, 평평 기준 ±{TERRAIN_REL*100:.0f} %)  {'평평' if flat_t else '변화 중'}")
        print(f"    완주 env  {o0:.3f} -> {o1:.3f} ({oa*100:+.1f} %p, 기준 ±{TIMEOUT_ABS*100:.0f} %p)  {'평평' if flat_o else '변화 중'}")
        print(f"    초당 추종 {k0:.3f} -> {k1:.3f} ({kr*100:+.1f} %, 기준 ±{TRACK_REL*100:.0f} %)  {'평평' if flat_k else '변화 중'}")
        if tr < REGRESS_REL:
            warn.append(f"지형 단계가 최근 {a.span} iter 동안 {tr*100:+.1f} % — 퇴행. 이전 체크포인트와 비교 측정할 것.")
        status = "CONVERGED" if (flat_t and flat_o and flat_k and o1 >= a.min_timeout) else "RUNNING"
        if flat_t and flat_o and flat_k and o1 < a.min_timeout:
            warn.append(f"곡선은 평평한데 완주 env {o1:.2f} < {a.min_timeout} — 수렴이 아니라 학습이 안 되는 상태일 수 있다.")
    else:
        print(f"  수렴 판정은 이 런에서 {max(a.min_iters, a.span + a.win)} iter 이후 (지금 {ran})")

    for w in warn:
        print("  경고:", w)
    if status == "CONVERGED":
        print("  -> 수렴: 멈추고 측정할 시점. 단 곡선 수렴은 행동 수렴이 아니다 (pv measure, pv play 로 확인).")
    print(f"STATUS {status}")
    return 10 if status == "CONVERGED" else 0


if __name__ == "__main__":
    sys.exit(main())
