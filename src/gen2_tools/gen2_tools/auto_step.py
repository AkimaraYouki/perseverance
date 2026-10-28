"""Automatic step-detect mode while following (2026-10-27, desktop). No ROS; used by person_follow_cam on the robot and by
climb_test --auto_step in Isaac.

FOLLOW (follow speed, up to 0.7 m/s)
  -> step seen by the camera closer than slow_dist          -> SLOW (min(follow, slow_v))
  -> closer than arm_dist (or the wheels blocked)           -> STEP: step-detect mode ON, forward command step_v
       (the robot then does its own approach -> blocked -> back off -> run -> jump: balance/step_mode as on the pad)
  -> the jump has landed + hold_s, or max_step_s            -> step mode OFF -> FOLLOW (camera ignores steps nearer
                                                               than ignore_m for ignore_s: the one just climbed)
  -> no block / jump within d0 + false_m (false alarm) or max_step_s without a jump -> step mode OFF -> SLOW, not FOLLOW
       (real robot 2026-10-29, 3 falls: the step was farther than the camera said, the old d0 + 0.35 m exit gave the
       robot the follow speed 0.65-0.7 m/s right in front of it, and the camera ignored it for ignore_s)
Fallback without the camera: FOLLOW and blocked (commanded > 0.15 m/s, measured < 0.05 m/s for block_s) and the LiDAR
corridor clear (a 6-8 cm step is below the LiDAR plane; a box is not) -> STEP.
"""


class AutoStep:
    def __init__(self, slow_dist=1.5, arm_dist=0.9, slow_v=0.3, step_v=0.3, hold_s=1.0, max_step_s=12.0,
                 block_s=0.3, lost_s=3.0, ignore_m=0.6, ignore_s=1.5, done_travel=1.3, false_m=0.8, land_stop_s=0.8):
        self.slow_dist, self.arm_dist, self.slow_v, self.step_v = slow_dist, arm_dist, slow_v, step_v
        self.hold_s, self.max_step_s, self.block_s, self.lost_s = hold_s, max_step_s, block_s, lost_s
        self.ignore_m, self.ignore_s = ignore_m, ignore_s
        self.false_m = false_m                                # STEP without a block this far past the step = false alarm
        self.slow_after, self.x_slow_end = 1.0, None
        # forward 0 for land_stop_s after each landing (real robot 2026-10-29 "한번 뛰고 뒤로 갈려고 함"): the landing overshoot
        # (0.4 m/s > approach 0.14) wound the balance slope integrator negative (v_ki 0.8 -> v_ref 0.05-0.07), the robot
        # crept below the stall line 0.5 x 0.14 and the core saw 'blocked' on flat ground -> back off / re-run / fall.
        # A zero command resets that integrator and the core's 'moved' timer (wb_core: |vx| <= 0.02, want false).
        self.land_stop_s = land_stop_s
        # 2-tier stairs (user 2026-10-27 "한 번 점프하고 2 단 점프니까 계속 턱 감지 켜 놓고 다 올라가면 해제"): after a jump
        # step mode stays ON; it is released once the robot has driven done_travel past the last landing without another
        # block / jump (the camera cannot see tier 2: plywood on plywood). Hitting tier 2 at follow speed fell in Isaac.
        self.done_travel = done_travel
        self.x_land = None
        self.state, self.t_state, self.t_seen, self.t_block = 'FOLLOW', 0.0, -1e9, None
        self.jumped, self.t_land, self.t_done = False, None, -1e9
        self.n_steps, self.false_alarms = 0, 0
        self.x0, self.d0 = None, 0.3
        self.x_step = None

    def _go(self, s, t):
        self.state, self.t_state = s, t

    def _give_up(self, t, odom_x, vx_follow):
        """STEP ended without a jump: step mode OFF but stay SLOW for slow_after more metres (the step may still be
        ahead: blocked at slow_v -> STEP again). No camera ignore window: nothing was climbed."""
        self.t_block, self.x_land, self.t_seen, self.x_step = None, None, t, None
        self.x_slow_end = None if odom_x is None else odom_x + self.slow_after
        self._go('SLOW', t)
        return min(vx_follow, self.slow_v), False, self.state

    def update(self, t, vx_follow, step_d, v_meas, jumping, lidar_clear=True, odom_x=None):
        """t [s], vx_follow: what following wants [m/s], step_d: camera step distance or None, v_meas [m/s],
        jumping: the jump state machine is not in DRIVE, odom_x: travelled distance [m] (false-alarm check).
        -> (vx command, step mode on?, state)."""
        if step_d is not None and t - self.t_done < self.ignore_s and step_d < self.ignore_m:
            step_d = None                                      # the step just climbed (still in view right after landing)
        if step_d is not None:
            self.t_seen = t
            if odom_x is not None:
                self.x_step = odom_x + step_d                     # remember where it is (odometry)
        elif odom_x is not None and getattr(self, 'x_step', None) is not None and self.state == 'SLOW':
            # close to the step its foot leaves the bottom of the image (2026-10-27 Isaac: lost -> back to 0.7 -> fell):
            # dead-reckon the remembered place instead of forgetting it
            step_d = self.x_step - odom_x if self.x_step - odom_x > -0.2 else None
        blocked = vx_follow > 0.15 and abs(v_meas) < 0.05
        self.t_block = (self.t_block if self.t_block is not None else t) if blocked else None
        blocked_long = self.t_block is not None and t - self.t_block >= self.block_s
        if self.state == 'FOLLOW':
            if step_d is not None and step_d < self.arm_dist:
                self._go('STEP', t)
            elif step_d is not None and step_d < self.slow_dist:
                self._go('SLOW', t)
            elif blocked_long and lidar_clear:
                self._go('STEP', t)
        elif self.state == 'SLOW':
            if (step_d is not None and step_d < self.arm_dist) or blocked_long:
                self._go('STEP', t)
            elif (t - self.t_seen > self.lost_s and step_d is None
                  and (self.x_slow_end is None or odom_x is None or odom_x > self.x_slow_end)):
                self.x_slow_end = None
                self._go('FOLLOW', t)
        if self.state == 'STEP':
            if self.t_state == t:
                self.jumped, self.t_land = False, None
                self.x0, self.d0 = odom_x, max(step_d if step_d is not None else 0.3, 0.3)   # dead-reckoned d can be < 0
            # false alarm (floor seam, shadow): drove past where the step should be without being blocked / jumping
            if (odom_x is not None and self.x0 is not None and not self.jumped and not jumping
                    and odom_x - self.x0 > self.d0 + self.false_m):
                self.false_alarms += 1
                return self._give_up(t, odom_x, vx_follow)
            if jumping:
                self.jumped, self.t_land = True, None
            elif self.jumped and self.t_land is None:
                self.t_land = t
            if self.t_land is not None and t - self.t_land >= self.hold_s:
                if self.x_land is None:                          # this landing counted once
                    self.n_steps += 1
                    self.x_land, self.t_state = odom_x, t          # timeout counts from the landing (approach to the next tier)
                if odom_x is None or odom_x - self.x_land > self.done_travel:   # nothing more to climb
                    self.t_done, self.t_block, self.x_land = t, None, None
                    self._go('FOLLOW', t)
                    return vx_follow, False, self.state
            if jumping and self.x_land is not None:              # next tier: a new jump -> count its landing again
                self.x_land, self.t_state = None, t
            if self.state == 'STEP' and t - self.t_state > self.max_step_s and not jumping:
                if not self.jumped:
                    return self._give_up(t, odom_x, vx_follow)
                self.n_steps += int(self.x_land is None)
                self.t_done, self.t_block, self.x_land = t, None, None
                self._go('FOLLOW', t)
        if self.state == 'FOLLOW':
            return vx_follow, False, self.state
        if self.state == 'SLOW':
            return min(vx_follow, self.slow_v), False, self.state
        if self.t_land is not None and t - self.t_land < self.land_stop_s:
            return 0.0, True, self.state                       # settle after the landing (see land_stop_s)
        return self.step_v, True, self.state                  # constant push (person_follow_cam stops it next to the person)
