"""Runtime parameter changes for the GCS GUI (comms 2026-10-13_1300): only the names listed, only while the robot
is disarmed (user 2026-10-13: "실시간 = 디스암하고"). No controller gains here — only limits, tracking, perception.

    live.Live(node, {'vx_max': 'vx_max', 'bearing_tol_deg': lambda v: setattr(node, 'tol', math.radians(v))})

Call it after every declare_parameter of the node. Declares the read-only `live_params` (string array, the names);
any other name is refused ("read at start only"). The robot mode comes from controller/state, polled on a side node
(no 100 Hz executor wake-ups); refused while it is unknown or not 'disarmed'.
"""

import rclpy
from gen2_msgs.msg import ControllerState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


class Live:
    def __init__(self, node, mapping, always=()):
        # always: on/off switches (not tuning) accepted in any mode, e.g. person_follow use_nav (GCS 1330)
        self.node, self.mapping, self.mode, self.always = node, dict(mapping), None, set(always)
        node.declare_parameter('live_params', list(self.mapping),
                               ParameterDescriptor(read_only=True, description='set at runtime, robot disarmed only'))
        self._side = rclpy.create_node(node.get_name() + '_live')
        self._sub = self._side.create_subscription(ControllerState, 'controller/state', lambda m: None, 1)
        node.add_on_set_parameters_callback(self._cb)

    def robot_mode(self):
        with self._sub.handle:
            while True:
                r = self._sub.handle.take_message(ControllerState, False)
                if r is None:
                    break
                self.mode = (r[0] if isinstance(r, tuple) else r).mode
        return self.mode

    def _cb(self, params):
        for q in params:
            if q.name not in self.mapping:
                return SetParametersResult(successful=False, reason=f'{q.name}: read at start only')
        mode = self.robot_mode()
        if mode != 'disarmed' and not all(q.name in self.always for q in params):
            return SetParametersResult(successful=False, reason=f'only while the robot is disarmed (now {mode})')
        for q in params:
            t = self.mapping[q.name]
            if callable(t):
                t(q.value)
            else:
                setattr(self.node, t, q.value)
            self.node.get_logger().info(f'live {q.name} = {q.value}')
        return SetParametersResult(successful=True)
