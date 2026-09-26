"""Gen2 Motor Test UI (Tk + matplotlib).

The GUI never streams motor commands. It asks motor_test_node (C++) to run a bounded, guarded
test and sends a 10 Hz heartbeat from the Tk main loop: if this window freezes or closes, the
test aborts within heartbeat_timeout_s (0.5 s). STOP = big button, Space or Esc.
"""
import collections
import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import rclpy  # noqa: E402
from gen2_msgs.msg import MotorStateArray, MotorTestStatus  # noqa: E402
from gen2_msgs.srv import MotorTest  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

WINDOW_S = 10.0
BG, PANEL, FG, DIM = '#11151c', '#1b212b', '#e6e8ec', '#8a93a3'
GREEN, RED, AMBER, BLUE = '#2fb86a', '#d9443a', '#e8a93a', '#4f9dff'


class Ros:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('motor_test_gui')
        self.lock = threading.Lock()
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=2000))
        self.latest = {}
        self.state_t = 0.0
        self.status = None
        self.status_t = 0.0
        self.node.create_subscription(MotorStateArray, 'motors/state', self._on_state,
                                      qos_profile_sensor_data)
        self.node.create_subscription(MotorTestStatus, 'motor_test/status', self._on_status, 10)
        self.hb = self.node.create_publisher(Empty, 'motor_test/heartbeat', 10)
        self.start_cli = self.node.create_client(MotorTest, 'motor_test/start')
        self.stop_cli = self.node.create_client(Trigger, 'motor_test/stop')
        self.zero_cli = self.node.create_client(Trigger, 'motor_test/zero_all')
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    def _on_state(self, msg):
        t = time.monotonic()
        with self.lock:
            self.state_t = t
            for m in msg.motors:
                self.latest[m.name] = m
                if not m.stale:
                    self.hist[m.name].append((t, math.degrees(m.position_rad), m.velocity_rad_s,
                                              m.current_a, m.raw_position_deg))

    def _on_status(self, msg):
        with self.lock:
            self.status, self.status_t = msg, time.monotonic()

    def shutdown(self):
        self.ex.shutdown(timeout_sec=2.0)
        self.th.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()


class App:
    def __init__(self, root, ros):
        self.root, self.ros = root, ros
        root.title('Gen2 Motor Test')
        root.configure(bg=BG)
        root.geometry('1180x760')
        style = ttk.Style(root)
        style.theme_use('clam')
        for w in ('TFrame', 'TLabelframe'):
            style.configure(w, background=PANEL)
        style.configure('TLabelframe.Label', background=PANEL, foreground=BLUE, font=('DejaVu Sans', 11, 'bold'))
        style.configure('TLabel', background=PANEL, foreground=FG, font=('DejaVu Sans', 11))
        style.configure('Dim.TLabel', foreground=DIM)
        style.configure('Big.TLabel', font=('DejaVu Sans Mono', 13, 'bold'))
        style.configure('TRadiobutton', background=PANEL, foreground=FG, font=('DejaVu Sans', 11))
        style.configure('TCheckbutton', background=PANEL, foreground=AMBER, font=('DejaVu Sans', 11, 'bold'))
        style.configure('TCombobox', font=('DejaVu Sans', 11))

        left = ttk.Frame(root, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(root, padding=4)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # motor select + live
        sel = ttk.LabelFrame(left, text='Motor', padding=8)
        sel.pack(fill=tk.X, pady=4)
        self.motor = tk.StringVar()
        self.combo = ttk.Combobox(sel, textvariable=self.motor, state='readonly', width=18)
        self.combo.pack(anchor=tk.W)
        self.live = ttk.Label(sel, text='waiting for /motors/state ...', style='Big.TLabel',
                              justify=tk.LEFT)
        self.live.pack(anchor=tk.W, pady=(8, 0))

        # test panel
        tp = ttk.LabelFrame(left, text='Guarded test', padding=8)
        tp.pack(fill=tk.X, pady=4)
        self.mode = tk.StringVar(value='current')
        mrow = ttk.Frame(tp)
        mrow.pack(anchor=tk.W)
        ttk.Radiobutton(mrow, text='Current (A)', variable=self.mode, value='current',
                        command=self._limits).pack(side=tk.LEFT)
        ttk.Radiobutton(mrow, text='Velocity (rad/s)', variable=self.mode, value='velocity',
                        command=self._limits).pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(mrow, text='Position (deg)', variable=self.mode, value='position',
                        command=self._limits).pack(side=tk.LEFT)
        self.value = tk.DoubleVar(value=0.3)
        vrow = tk.Frame(tp, bg=PANEL)
        vrow.pack(anchor=tk.W, pady=4)
        self.scale = tk.Scale(vrow, variable=self.value, orient=tk.HORIZONTAL, length=250,
                              resolution=0.05, from_=-1, to=1, bg=PANEL, fg=FG,
                              highlightthickness=0, troughcolor='#2a3140', font=('DejaVu Sans', 10))
        self.scale.pack(side=tk.LEFT)
        self.value_entry = tk.Entry(vrow, textvariable=self.value, width=8, font=('DejaVu Sans', 11))
        self.value_entry.pack(side=tk.LEFT, padx=6, anchor=tk.S)
        self.value_unit = ttk.Label(vrow, text='A')
        self.value_unit.pack(side=tk.LEFT, anchor=tk.S)
        self.limit_lbl = ttk.Label(tp, text='', style='Dim.TLabel')
        self.limit_lbl.pack(anchor=tk.W)
        drow = ttk.Frame(tp)
        drow.pack(anchor=tk.W, pady=4)
        ttk.Label(drow, text='Duration s').pack(side=tk.LEFT)
        self.duration = tk.DoubleVar(value=1.0)
        self.dur_spin = tk.Spinbox(drow, from_=0.2, to=3.0, increment=0.2, width=6,
                                   textvariable=self.duration, font=('DejaVu Sans', 11))
        self.dur_spin.pack(side=tk.LEFT, padx=6)
        ttk.Label(drow, text='Speed rad/s').pack(side=tk.LEFT, padx=(10, 0))
        self.speed = tk.DoubleVar(value=2.0)
        self.speed_spin = tk.Spinbox(drow, from_=0.1, to=20.0, increment=0.5, width=6,
                                     textvariable=self.speed, font=('DejaVu Sans', 11),
                                     state=tk.DISABLED)
        self.speed_spin.pack(side=tk.LEFT, padx=6)
        self.lifted = tk.BooleanVar(value=False)
        ttk.Checkbutton(tp, text='Robot lifted / wheel free / hand on E-stop',
                        variable=self.lifted, command=self._buttons).pack(anchor=tk.W, pady=6)
        brow = tk.Frame(tp, bg=PANEL)
        brow.pack(fill=tk.X)
        self.start_btn = tk.Button(brow, text='START', bg=GREEN, fg='white', width=10,
                                   font=('DejaVu Sans', 13, 'bold'), command=self._start,
                                   state=tk.DISABLED, disabledforeground='#355')
        self.start_btn.pack(side=tk.LEFT)
        tk.Button(brow, text='Zero all', width=8, command=self._zero,
                  font=('DejaVu Sans', 11)).pack(side=tk.LEFT, padx=8)
        self.stop_btn = tk.Button(left, text='STOP  (Space / Esc)', bg=RED, fg='white', height=2,
                                  font=('DejaVu Sans', 18, 'bold'), command=self._stop,
                                  activebackground='#ff5040')
        self.stop_btn.pack(fill=tk.X, pady=8)

        # result / status
        rp = ttk.LabelFrame(left, text='Test status', padding=8)
        rp.pack(fill=tk.X, pady=4)
        self.status_lbl = ttk.Label(rp, text='motor_test_node: not seen', justify=tk.LEFT,
                                    wraplength=340)
        self.status_lbl.pack(anchor=tk.W)

        # scale check
        sp = ttk.LabelFrame(left, text='Hand-rotation scale check (read-only)', padding=8)
        sp.pack(fill=tk.X, pady=4)
        srow = tk.Frame(sp, bg=PANEL)
        srow.pack(anchor=tk.W)
        tk.Button(srow, text='Begin', width=8, command=self._scale_begin).pack(side=tk.LEFT)
        tk.Button(srow, text='Finish', width=8, command=self._scale_finish).pack(side=tk.LEFT, padx=6)
        self.scale_lbl = ttk.Label(sp, text='Mark the output, press Begin, turn N full turns, Finish.',
                                   style='Dim.TLabel', wraplength=340, justify=tk.LEFT)
        self.scale_lbl.pack(anchor=tk.W, pady=4)
        self._scale = None

        # plots
        self.fig = Figure(figsize=(7.5, 7), dpi=100, facecolor=BG)
        self.axes = [self.fig.add_subplot(3, 1, i + 1) for i in range(3)]
        labels = ('position [deg]', 'velocity [rad/s]', 'current [A]')
        colors = (BLUE, GREEN, AMBER)
        self.lines = []
        for ax, lab, col in zip(self.axes, labels, colors):
            ax.set_facecolor(PANEL)
            ax.tick_params(colors=DIM, labelsize=9)
            for sp_ in ax.spines.values():
                sp_.set_color('#2a3140')
            ax.set_ylabel(lab, color=FG, fontsize=10)
            ax.grid(True, color='#2a3140', linewidth=0.6)
            ax.set_xlim(-WINDOW_S, 0)
            self.lines.append(ax.plot([], [], color=col, linewidth=1.6)[0])
        self.axes[-1].set_xlabel('time [s]', color=FG)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        root.bind('<space>', lambda e: self._stop())
        root.bind('<Escape>', lambda e: self._stop())
        root.protocol('WM_DELETE_WINDOW', self._close)
        self._last_status_id = None
        self._heartbeat()
        self._refresh()

    # ------------------------------------------------------------------ helpers
    def _limits(self):
        st = self.ros.status
        name = self.motor.get()
        if st is None or name not in st.motor_names:
            return None
        i = list(st.motor_names).index(name)
        mode = self.mode.get()
        vlim = st.max_velocity_rad_s[i]
        if mode == 'current':
            lim, unit, res = st.max_current_a[i], 'A', 0.05
        elif mode == 'velocity':
            lim, unit, res = vlim, 'rad/s', 0.1
        else:
            lim, unit, res = st.max_position_move_deg, 'deg', 1.0
        self.scale.configure(from_=-lim, to=lim, resolution=res)
        try:
            v = max(-lim, min(lim, self.value.get()))
        except tk.TclError:
            v = 0.0
        self.value.set(v)
        self.value_unit.configure(text=unit)
        self.dur_spin.configure(to=st.max_duration_s)
        self.speed_spin.configure(to=vlim, state=tk.NORMAL if mode == 'position' else tk.DISABLED)
        extra = f', speed ≤ {vlim:.1f} rad/s (relative move)' if mode == 'position' else ''
        self.limit_lbl.configure(
            text=f'limit ±{lim:.2f} {unit}, duration ≤ {st.max_duration_s:.1f} s{extra}')
        return lim

    def _buttons(self):
        st = self.ros.status
        ok = (self.lifted.get() and st is not None and time.monotonic() - self.ros.status_t < 1.0
              and not st.running and bool(self.motor.get()))
        self.start_btn.configure(state=tk.NORMAL if ok else tk.DISABLED)

    def _call(self, cli, req, done):
        if not cli.service_is_ready():
            messagebox.showerror('Motor test', 'motor_test_node is not running')
            return
        fut = cli.call_async(req)

        def poll():
            if fut.done():
                done(fut.result())
            else:
                self.root.after(30, poll)
        poll()

    # ------------------------------------------------------------------ actions
    def _start(self):
        req = MotorTest.Request()
        req.motor = self.motor.get()
        req.mode = self.mode.get()
        try:
            req.value = float(self.value.get())
            req.duration_s = float(self.duration.get())
            if req.mode == 'position':
                req.speed_rad_s = float(self.speed.get())
        except (tk.TclError, ValueError):
            messagebox.showerror('Motor test', 'invalid number')
            return
        req.confirm_lifted = bool(self.lifted.get())
        if req.value == 0.0:
            messagebox.showerror('Motor test', 'value is 0')
            return

        def done(res):
            if not res.accepted:
                messagebox.showwarning('Motor test', f'Rejected: {res.message}')
        self._call(self.ros.start_cli, req, done)

    def _stop(self):
        if self.ros.stop_cli.service_is_ready():
            self.ros.stop_cli.call_async(Trigger.Request())

    def _zero(self):
        self._call(self.ros.zero_cli, Trigger.Request(), lambda r: None)

    def _scale_begin(self):
        name = self.motor.get()
        m = self.ros.latest.get(name)
        if m is None or m.stale:
            self.scale_lbl.configure(text='no fresh feedback')
            return
        self._scale = dict(name=name, prev=m.raw_position_deg, total=0.0)
        self.scale_lbl.configure(text='counting... turn the output by hand, then Finish')

    def _scale_finish(self):
        if not self._scale:
            return
        n = simpledialog.askfloat('Scale check', 'How many output turns did you make? (e.g. 1 or -1)',
                                  parent=self.root)
        s, self._scale = self._scale, None
        if not n:
            self.scale_lbl.configure(text='cancelled')
            return
        per_rev = abs(s['total'] / n)
        direction = 1 if s['total'] * n > 0 else -1
        self.scale_lbl.configure(
            text=f"raw delta {s['total']:.1f}° over {n:g} turn(s)\n"
                 f"=> raw_deg_per_output_rev: {per_rev:.1f}\n=> direction: {direction}\n"
                 f"Edit gen2_hardware/config/motors.yaml, then set verified flags.")

    def _close(self):
        self._stop()
        self.root.after(200, self.root.destroy)

    # ------------------------------------------------------------------ loops
    def _heartbeat(self):
        self.ros.hb.publish(Empty())   # dead-man: stops if the Tk loop stalls or the window closes
        self.root.after(100, self._heartbeat)

    def _refresh(self):
        now = time.monotonic()
        with self.ros.lock:
            names = sorted(self.ros.latest.keys())
            state_age = now - self.ros.state_t if self.ros.state_t else None
            st = self.ros.status
            st_age = now - self.ros.status_t if self.ros.status_t else None
        if st is not None and not names:
            names = list(st.motor_names)
        if names and list(self.combo['values']) != names:
            self.combo['values'] = names
            if not self.motor.get():
                self.motor.set(names[0])
                self._limits()
        name = self.motor.get()
        m = self.ros.latest.get(name)
        if m is None or state_age is None or state_age > 1.0:
            self.live.configure(text='NO /motors/state\n(is gen2-bench running?)', foreground=RED)
        else:
            col = RED if m.stale or m.error_code else FG
            self.live.configure(foreground=col, text=(
                f"pos   {math.degrees(m.position_rad):9.2f} deg\n"
                f"vel   {m.velocity_rad_s:9.3f} rad/s\n"
                f"cur   {m.current_a:9.2f} A\n"
                f"temp  {m.temperature_c:9.0f} °C\n"
                f"err   {m.error_code} {m.error_text}{'  STALE' if m.stale else ''}"))
            if self._scale and self._scale['name'] == name and not m.stale:
                d = m.raw_position_deg - self._scale['prev']
                d -= 6400.0 * round(d / 6400.0)   # int16 upload wraps at ±3200°
                self._scale['total'] += d
                self._scale['prev'] = m.raw_position_deg
                self.scale_lbl.configure(text=f"counting... raw delta {self._scale['total']:.1f}°")
        if st is None or st_age is None or st_age > 1.0:
            self.status_lbl.configure(text='motor_test_node: NOT RUNNING — tests unavailable',
                                      foreground=RED)
        else:
            if st.running:
                txt = (f"RUNNING {st.motor} {st.mode} {st.value:+.2f}  "
                       f"{st.elapsed_s:.1f}/{st.duration_s:.1f} s")
                col = AMBER
            else:
                txt = (f"last: {st.result}\n{st.motor} {st.mode} {st.value:+.2f}  "
                       f"moved {math.degrees(st.moved_rad):.1f}°  peak {st.peak_current_a:.2f} A, "
                       f"{st.peak_velocity_rad_s:.2f} rad/s  mean(2nd half) {st.mean_velocity_rad_s:.3f}")
                if st.mode == 'position':
                    txt += (f"\ntarget {math.degrees(st.target_rad):.1f}°  "
                            f"final error {math.degrees(st.position_error_rad):+.2f}°")
                col = GREEN if st.result in ('done', 'idle') else AMBER
            txt += f"\nheartbeat {'OK' if st.heartbeat_ok else 'LOST'}"
            self.status_lbl.configure(text=txt, foreground=col)
            if self._last_status_id is None:
                self._last_status_id = st.test_id
                self._limits()
        self._buttons()
        # plots
        with self.ros.lock:
            h = list(self.ros.hist.get(name, []))
        h = [x for x in h if now - x[0] <= WINDOW_S]
        if h:
            t = [x[0] - now for x in h]
            for i, (ax, line) in enumerate(zip(self.axes, self.lines)):
                y = [x[i + 1] for x in h]
                line.set_data(t, y)
                lo, hi = min(y), max(y)
                pad = max(0.05, 0.1 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)
        self.canvas.draw_idle()
        self.root.after(100, self._refresh)


def main():
    ros = Ros()
    root = tk.Tk()
    App(root, ros)
    try:
        root.mainloop()
    finally:
        ros.shutdown()


if __name__ == '__main__':
    main()
