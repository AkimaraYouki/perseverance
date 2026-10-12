"""Offline check of person_follow (+ nav_follow.launch) in a separate DDS domain, no robot needed:
  ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: launch nav_follow, static TF base_link->laser, person_follow,
  then python3 sim_follow.py <occl_start_s> <occl_end_s> [turn] [obs] [far]
"""
"""Kinematic sim: robot integrates cmd_vel/auto, person A walks (occluded OCC_A..OCC_B s), person B stands aside."""
import rclpy, time, math, json, sys
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, Pose, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
OCC_A, OCC_B = float(sys.argv[1]), float(sys.argv[2]); TURN = 'turn' in sys.argv[3:]; OBS = 'obs' in sys.argv[3:]
OBS_C, OBS_R = np.array([1.3, 0.05]), 0.25
X0 = 2.4 if 'far' in sys.argv[3:] else 1.6
if X0 > 2: OBS_C = np.array([1.1, 0.05])
rclpy.init(); n = rclpy.create_node('sim'); tb = TransformBroadcaster(n)
cmd = [0.0, 0.0]; st = []
n.create_subscription(Twist, 'cmd_vel/auto', lambda m: cmd.__setitem__(slice(0, 2), [m.linear.x, m.angular.z]), 10)
n.create_subscription(String, 'follow/status', lambda m: st.append(m.data), 10)
pl = n.create_publisher(PoseArray, 'people/legs', 10); pc = n.create_publisher(String, 'people/camera', 10)
po = n.create_publisher(Odometry, 'odom', 10); ps = n.create_publisher(LaserScan, 'scan_level', 10)
hA = np.zeros(32); hA[3] = 1; hB = np.zeros(32); hB[20] = 1
rob = np.array([0.0, 0.0, 0.0]); v_act = 0.0; w_act = 0.0
def tf_pub(now):
    t = TransformStamped(); t.header.stamp = now; t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
    t.transform.translation.x, t.transform.translation.y = rob[0], rob[1]
    t.transform.rotation.z, t.transform.rotation.w = math.sin(rob[2] / 2), math.cos(rob[2] / 2); tb.sendTransform(t)
    o = Odometry(); o.header.stamp = now; o.header.frame_id = 'odom'; o.child_frame_id = 'base_link'
    o.pose.pose.position.x, o.pose.pose.position.y = rob[0], rob[1]; o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(rob[2] / 2), math.cos(rob[2] / 2); o.twist.twist.linear.x = v_act; o.twist.twist.angular.z = w_act; po.publish(o)
def rel(p):
    c, s = math.cos(rob[2]), math.sin(rob[2]); dx, dy = p[0] - rob[0], p[1] - rob[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy])
c = n.create_client(SetBool, 'follow/enable')
for k in range(40): tf_pub(n.get_clock().now().to_msg()); rclpy.spin_once(n, timeout_sec=0.05)
c.wait_for_service(timeout_sec=5)
t0 = time.time(); last_scan = 0; enabled = False; log = []
while time.time() - t0 < 16:
    T = time.time() - t0; dt = 0.02
    if not enabled and T > 0.5:
        n.get_logger().info('enable')
        c.call_async(SetBool.Request(data=True)); enabled = True
    # person A: stands 1.6 m ahead, from 2 s walks +x 0.6 m/s (TURN: turns left at 6 s)
    if T < 2: pA = np.array([X0, 0.0])
    elif not TURN or T < 6: pA = np.array([X0 + 0.6 * (T - 2), 0.0])
    else: pA = np.array([X0 + 0.6 * 4, 0.6 * (T - 6)])
    pB = np.array([2.0 + 0.0, 1.5])   # second person, different clothes, stands
    # robot: first-order speed response (balancing robot lag)
    v_act += (np.clip(cmd[0], -1, 1) - v_act) * dt / 0.4; w_act += (cmd[1] - w_act) * dt / 0.2
    rob[2] += w_act * dt; rob[0] += v_act * math.cos(rob[2]) * dt; rob[1] += v_act * math.sin(rob[2]) * dt
    now = n.get_clock().now().to_msg(); tf_pub(now)
    occl = OCC_A <= T < OCC_B
    if T - last_scan >= 0.1:
        last_scan = T; pa = PoseArray(); pa.header.frame_id = 'laser'; pa.header.stamp = now
        people = []
        for p, h, hide in ((pA, hA, occl), (pB, hB, False)):
            r = rel(p)
            if hide or r[0] < 0.1 and abs(r[1]) < 0.1: continue
            q = Pose(); q.position.x = r[0] + 0.0774; q.position.y = r[1]; pa.poses.append(q)
            b = math.degrees(math.atan2(r[1], r[0]))
            if abs(b) < 31: people.append({'bearing_deg': b, 'hsv_hist': list(h)})
        pl.publish(pa); pc.publish(String(data=json.dumps({'people': people})))
        s = LaserScan(); s.header.frame_id = 'laser'; s.header.stamp = now; s.angle_min = -3.14; s.angle_increment = 0.0175; s.range_min = 0.05; s.range_max = 12.0; rg = []
        for i in range(359):
            a = rob[2] + s.angle_min + i * s.angle_increment; dvec = np.array([math.cos(a), math.sin(a)]); o = rob[:2] + np.array([-0.0774 * math.cos(rob[2]), -0.0774 * math.sin(rob[2])])
            best = float('inf')
            if OBS:
                f = o - OBS_C; bq = f @ dvec; cq = f @ f - OBS_R ** 2; disc = bq * bq - cq
                if disc >= 0 and -bq - math.sqrt(disc) > 0: best = -bq - math.sqrt(disc)
            for pp, hide in ((pA, occl), (pB, False)):
                if hide: continue
                f = o - pp; bq = f @ dvec; cq = f @ f - 0.06 ** 2; disc = bq * bq - cq
                if disc >= 0 and -bq - math.sqrt(disc) > 0: best = min(best, -bq - math.sqrt(disc))
            rg.append(best)
        s.ranges = rg; ps.publish(s)
    if int(T * 2) != int((T - dt) * 2):
        log.append(f'{T:5.1f}s robot ({rob[0]:4.1f},{rob[1]:4.1f}) A ({pA[0]:4.1f},{pA[1]:4.1f}) dist {np.linalg.norm(pA - rob[:2]):4.2f} v {v_act:4.2f} cmd {cmd[0]:+.2f}/{cmd[1]:+.2f} {"OCCL " if occl else ""}{"HIT " if OBS and np.linalg.norm(rob[:2]-OBS_C) < OBS_R + 0.2 else ""}| {st[-1] if st else ""}')
    rclpy.spin_once(n, timeout_sec=0.0); time.sleep(dt)
c.call_async(SetBool.Request(data=False)); [rclpy.spin_once(n, timeout_sec=0.05) for _ in range(5)]
print('\n'.join(log))
