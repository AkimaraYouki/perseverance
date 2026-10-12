#!/usr/bin/env python3
"""Follow ONE designated person (competition: no markers, 2 people, hand signals stop / restart only).

Fusion: LiDAR leg tracks (people/legs, laser frame: distance + bearing, works without light) are confirmed as people
when a camera person box (people/camera, YOLO on the GPU) lies at the same bearing (+-bearing_tol). The designated
person is registered on enable: the confirmed person closest to straight ahead; its clothes (HSV histogram of the
box) are kept and slowly updated, and every later choice prefers the track that continues + the best appearance.

States: IDLE (not enabled) -> FOLLOW (seen) -> OCCLUDED (lost < occl_s: drive to the last seen place, slowly)
        -> SEARCH (turn slowly until the person is seen and matches again). HOLD = stop hand signal: stand still.
Command: with Nav2 up (nav_follow.launch) a path to 'follow_dist in front of the person' is planned on the LiDAR
costmap every nav_replan_s and MPPI drives it (around obstacles); close to the person, without Nav2 or when the
controller aborts: wz = k_yaw * bearing, vx = k_dist * (distance - follow_dist) * cos(bearing). Both go to
cmd_vel/auto (cmd_mux; the pad always wins). Parameters can be changed live (ros2 param set). Services: follow/enable (SetBool), follow/hold (SetBool: true = stop, false = resume).
Topic: follow/status (String).
"""

import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from std_srvs.srv import SetBool

LASER_X = -0.0774          # laser in base_link (desktop CAD)


class PersonFollow(Node):
    def __init__(self):
        super().__init__('person_follow')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.d_ref = p('follow_dist_m', 0.8)
        self.k_d, self.k_yaw = p('k_dist', 0.8), p('k_yaw', 1.5)
        self.vx_max, self.wz_max = p('vx_max', 0.7), p('wz_max', 1.0)
        self.tol = math.radians(p('bearing_tol_deg', 10.0))
        self.cam_to = p('camera_timeout_s', 0.5)
        self.occl_s = p('occluded_s', 5.0)
        self.search_wz = p('search_wz', 0.4)
        self.app_min = p('appearance_min', 0.5)          # Bhattacharyya similarity needed to re-acquire
        self.enabled, self.hold = False, False
        self.state = 'IDLE'
        self.cam, self.cam_t = [], 0.0
        self.legs = []
        self.target = None                               # dict: id, p (base xy), t, hist
        self.pub = self.create_publisher(Twist, 'cmd_vel/auto', 10)
        self.st = self.create_publisher(String, 'follow/status', 10)
        self.create_subscription(String, 'people/camera', self._cam, 10)
        self.create_subscription(PoseArray, 'people/legs', self._legs, 10)
        self.create_service(SetBool, 'follow/enable', self._enable)
        self.create_service(SetBool, 'follow/hold', self._hold)
        # Nav2 (nav_follow.launch: planner + MPPI on the LiDAR costmap) drives around obstacles when available
        self.use_nav = p('use_nav', True)
        self.nav_period = p('nav_replan_s', 0.5)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self.ac_plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.ac_follow = ActionClient(self, FollowPath, 'follow_path')
        self.nav_goal_h = None          # FollowPath goal handle
        self.nav_t = 0.0
        self.nav_busy = False
        self.add_on_set_parameters_callback(self._set_params)
        self.create_timer(0.05, self._step)

    _LIVE = {'follow_dist_m': 'd_ref', 'k_dist': 'k_d', 'k_yaw': 'k_yaw', 'vx_max': 'vx_max', 'wz_max': 'wz_max',
             'occluded_s': 'occl_s', 'search_wz': 'search_wz', 'appearance_min': 'app_min', 'use_nav': 'use_nav',
             'nav_replan_s': 'nav_period'}

    def _set_params(self, params):
        """ros2 param set /person_follow <name> <value> takes effect immediately (tuning on the robot)."""
        from rcl_interfaces.msg import SetParametersResult
        for q in params:
            if q.name in self._LIVE:
                setattr(self, self._LIVE[q.name], q.value)
            elif q.name == 'bearing_tol_deg':
                self.tol = math.radians(q.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ inputs
    def _cam(self, m):
        self.cam, self.cam_t = json.loads(m.data)['people'], time.monotonic()

    def _legs(self, m):
        # PoseArray in the laser frame, ids are not carried: use positions (base frame)
        self.legs = [np.array([q.position.x + LASER_X, q.position.y]) for q in m.poses]

    def _enable(self, req, res):
        self.enabled = bool(req.data)
        self.hold = False
        self.target = None
        self.state = 'SEARCH' if self.enabled else 'IDLE'
        res.success, res.message = True, f'follow {"on: registering the person ahead" if self.enabled else "off"}'
        self.get_logger().info(res.message)
        if not self.enabled:
            self._nav_cancel()
            self.pub.publish(Twist())
        return res

    def _hold(self, req, res):
        self.hold = bool(req.data)
        if self.hold:
            self._nav_cancel()
        res.success, res.message = True, 'hold' if self.hold else 'resume'
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------------ fusion
    @staticmethod
    def _sim(a, b):
        return float(np.sum(np.sqrt(np.asarray(a) * np.asarray(b))))      # Bhattacharyya coefficient

    def _candidates(self):
        """Confirmed people: leg position + matching camera box (bearing within tol)."""
        fresh = time.monotonic() - self.cam_t < self.cam_to
        out = []
        for pl in self.legs:
            b = math.atan2(pl[1], pl[0])
            best = None
            if fresh:
                for c in self.cam:
                    d = abs(math.radians(c['bearing_deg']) - b)
                    if d < self.tol and (best is None or d < best[0]):
                        best = (d, c)
            out.append({'p': pl, 'bearing': b, 'dist': float(np.hypot(*pl)), 'cam': best[1] if best else None})
        return out

    def _step(self):
        now = time.monotonic()
        if not self.enabled:
            self._status('IDLE')
            return
        cands = self._candidates()
        tgt = self.target
        pick = None
        if tgt is None:
            # register: confirmed person (camera + legs) closest to straight ahead, within 2.5 m
            conf = [c for c in cands if c['cam'] is not None and c['dist'] < 2.5]
            if conf:
                pick = min(conf, key=lambda c: abs(c['bearing']))
                self.target = tgt = {'p': pick['p'], 't': now, 'hist': np.array(pick['cam']['hsv_hist'])}
                self.get_logger().info(f'registered person at {pick["dist"]:.2f} m, {math.degrees(pick["bearing"]):+.0f} deg')
        else:
            best, bs = None, -1e9
            for c in cands:
                jump = float(np.linalg.norm(c['p'] - tgt['p']))
                gate = 0.4 + 1.5 * (now - tgt['t'])          # how far the person may have walked since last seen
                if jump > gate:
                    continue
                sim = self._sim(c['cam']['hsv_hist'], tgt['hist']) if c['cam'] else None
                if sim is not None and sim < self.app_min:
                    continue                                 # someone else (different clothes)
                s = -jump + (0.5 * sim if sim is not None else 0.0)
                if s > bs:
                    best, bs = c, s
            pick = best
            if pick is not None:
                tgt['p'], tgt['t'] = pick['p'], now
                if pick['cam'] is not None:
                    tgt['hist'] = 0.9 * tgt['hist'] + 0.1 * np.array(pick['cam']['hsv_hist'])
        # between LiDAR scans (10 Hz) keep the bearing fresh with the camera (30 Hz): same person = matching
        # clothes within the bearing tolerance -> rotate the target to the camera bearing, keep the LiDAR range
        if tgt is not None and now - self.cam_t < 0.1 and now - tgt['t'] < 0.5:
            bt, dt_ = math.atan2(tgt['p'][1], tgt['p'][0]), float(np.hypot(*tgt['p']))
            best = None
            for c in self.cam:
                bc = math.radians(c['bearing_deg'])
                if abs(bc - bt) < self.tol and self._sim(c['hsv_hist'], tgt['hist']) >= self.app_min:
                    if best is None or abs(bc - bt) < abs(best - bt):
                        best = bc
            if best is not None:
                tgt['p'] = np.array([dt_ * math.cos(best), dt_ * math.sin(best)])
        t = Twist()
        if self.hold:
            state = 'HOLD'
        elif tgt is None:
            state = 'SEARCH'
            t.angular.z = self.search_wz
        elif pick is not None:
            state = 'FOLLOW'
        elif now - tgt['t'] < self.occl_s:
            state = 'OCCLUDED'
        else:
            state = 'SEARCH'
            t.angular.z = self.search_wz
        if state in ('FOLLOW', 'OCCLUDED') and self._nav(tgt, now, state == 'FOLLOW'):
            self._status(state + '+nav', tgt)
            return                                         # the Nav2 controller publishes cmd_vel/auto
        self._nav_cancel()
        if state in ('FOLLOW', 'OCCLUDED'):
            x, y = tgt['p']
            b, d = math.atan2(y, x), math.hypot(x, y)
            k = 1.0 if state == 'FOLLOW' else 0.5             # slower towards the last seen place
            t.angular.z = float(np.clip(self.k_yaw * b, -self.wz_max, self.wz_max))
            t.linear.x = float(np.clip(k * self.k_d * (d - self.d_ref) * max(0.0, math.cos(b)), -self.vx_max * 0.5, self.vx_max))
        self.pub.publish(t)
        self._status(state, tgt)

    # ------------------------------------------------------------------ Nav2
    def _nav(self, tgt, now, seen):
        """Plan to 'follow_dist in front of the person' (odom frame) every nav_replan_s; False = drive directly.
        Not seen (OCCLUDED): the target position is relative to where the robot was, so keep the last (odom) goal."""
        if not self.use_nav or not self.ac_plan.server_is_ready() or not self.ac_follow.server_is_ready():
            return False
        if now - self.nav_fail_t < 2.0:
            return False
        if not seen:
            return self.nav_goal_h is not None
        x, y = tgt['p']
        d = math.hypot(x, y)
        if d < self.d_ref + 0.15:                          # close enough: just face the person (direct)
            return False
        if self.nav_busy or now - self.nav_t < self.nav_period:
            return self.nav_goal_h is not None
        try:
            tf = self.tfb.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return False
        gx, gy = x - self.d_ref * x / d, y - self.d_ref * y / d          # goal in base_link
        q = tf.transform.rotation
        yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = tf.transform.translation.x + c * gx - s_ * gy
        g.pose.position.y = tf.transform.translation.y + s_ * gx + c * gy
        yaw = yaw0 + math.atan2(y, x)
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req = ComputePathToPose.Goal(goal=g, planner_id='GridBased', use_start=False)
        self.nav_busy, self.nav_t = True, now
        self.ac_plan.send_goal_async(req).add_done_callback(self._plan_accepted)
        return self.nav_goal_h is not None

    def _plan_accepted(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.nav_busy = False
            return
        h.get_result_async().add_done_callback(self._plan_done)

    def _plan_done(self, fut):
        self.nav_busy = False
        res = fut.result().result
        if not res.path.poses or not self.enabled or self.hold:
            return
        g = FollowPath.Goal(path=res.path, controller_id='FollowPath', goal_checker_id='goal_checker')
        self.ac_follow.send_goal_async(g).add_done_callback(self._follow_accepted)

    def _follow_accepted(self, fut):
        h = fut.result()
        self.nav_goal_h = h if h and h.accepted else None
        if self.nav_goal_h is not None:
            h.get_result_async().add_done_callback(lambda f, h=h: self._follow_done(h, f.result().status))

    def _follow_done(self, h, status):
        if status == GoalStatus.STATUS_ABORTED and self.nav_goal_h is h:
            self.get_logger().warn('Nav2 controller aborted: steering directly for 2 s')
            self.nav_goal_h, self.nav_fail_t = None, time.monotonic()

    def _nav_cancel(self):
        if self.nav_goal_h is not None:
            self.nav_goal_h.cancel_goal_async()
            self.nav_goal_h = None

    def _status(self, state, tgt=None):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
        s = state
        if tgt is not None:
            s += f' target {math.hypot(*tgt["p"]):.2f} m @ {math.degrees(math.atan2(tgt["p"][1], tgt["p"][0])):+.0f} deg'
        self.st.publish(String(data=s))


def main():
    rclpy.init()
    try:
        rclpy.spin(PersonFollow())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
