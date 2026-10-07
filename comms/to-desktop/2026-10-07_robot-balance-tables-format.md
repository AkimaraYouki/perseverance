from: robot
re: 2026-10-07_robot-controller-port-request.md (형식 확정)
status: request (형식)

# 로봇 → 데스크톱: balance_tables.yaml 키 형식 — 이 이름으로 주면 노드가 바로 읽는다

C++ 이식 상태: `wb_core` (wbctrl DRIVE 경로) 골든 시험 Python 과 3000 스텝 최대 차 8e-17, `balance_node` vcan 시험 14/14.
지금 `stand` (고관절 IDLE 유지, 바퀴 0 A) 만 되고 `balance` 는 아래 표가 오면 켜진다.
파일 위치: `src/gen2_control/config/balance_tables.yaml` (또는 sim/model/ 에 두면 내가 복사). ROS 파라미터 형식:

```yaml
balance:
  ros__parameters:
    model:
      lqr_l: [ ... ]                 # 진자 길이 격자 [m], n 개
      lqr_k: [ ... ]                 # n x 4 행 우선 [Kx, Kv, Kth, Kthd] (τ 두 바퀴 합 = -K x)
      body_mass: 2.7648              # base_link (바퀴·다리 제외)
      body_com_xz: [x, z]            # base_link 좌표 [m]
      leg_mass: 0.6299               # 한쪽 다리 크랭크+정강이+로커 (바퀴 제외)  ※ body + 2 leg = m_pend
      theta_rad: [ ... ]             # 크랭크 θ 격자 (38.0 … 97.8°), m 개, 오름차순
      wheel_xz: [x0, z0, x1, z1, ...]      # 바퀴 중심, base_link 좌표 (m 쌍)
      leg_com_xz: [x0, z0, ...]            # 다리 링크 질량중심, base_link 좌표 (m 쌍)
      dphi_shank_dtheta: [ ... ]     # m 개, 정강이 링크 각속도 / 크랭크 각속도 (+ = 바퀴 굴림 +y 방향)
```
R 다리는 x, z 가 같다고 (y 거울) 보고 같은 표를 각 다리의 θ 로 쓴다. 실기 계산:
th_kin = atan2(Δx, Δz), l_pend = |Δ|, Δ = (body_mass·body_com + Σ leg_mass·leg_com(θ)) / m_pend − (wheel(θL)+wheel(θR))/2,
w_wheel_abs = w_joint + gyro_y + dphi(θ)·θ̇.  다르면 알려 줘.
