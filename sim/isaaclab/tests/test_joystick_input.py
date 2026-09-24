# 가짜 joydev: FIFO 에 8바이트 이벤트를 써서 Gamepad 를 검증한다 (Isaac 불필요)
import os, struct, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "source", "wheeled_biped_isaaclab"))
import joystick_input as J
d = tempfile.mkdtemp(); fifo = os.path.join(d, "js"); os.mkfifo(fifo)
r = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK); w = os.open(fifo, os.O_WRONLY)
os.close(r)
pad = J.Gamepad(fifo)
def ev(t, n, v): os.write(w, struct.pack("IhBB", 0, v, t, n))
# 최신 hid 배치 초기상태: 0..3 스틱 0, 4/5 트리거 -1 (INIT 플래그)
for n in range(6): ev(J.JS_EVENT_AXIS | J.JS_EVENT_INIT, n, -32767 if n in (4, 5) else 0)
pad.poll(); print("배치:", pad.layout)
R = ((-0.45, 0.45), (-1.0, 1.0)); ok = True
def check(name, got, exp):
    global ok
    g = tuple(round(x, 3) if isinstance(x, float) else x for x in got)
    good = all(abs(a-b) < 1e-3 if isinstance(a, float) else a == b for a, b in zip(g, exp))
    ok &= good; print(("PASS" if good else "FAIL"), name, g, "기대", exp)
check("중립", J.command_from_gamepad(pad, *R), (0.0, 0.0, 0.0, False, False))
ev(J.JS_EVENT_AXIS, 1, -32767); pad.poll(); check("왼스틱 위 끝 = 전진 최대", J.command_from_gamepad(pad, *R), (0.45, 0.0, 0.0, False, False))
ev(J.JS_EVENT_AXIS, 1, 0); ev(J.JS_EVENT_AXIS, 0, -32767); pad.poll(); check("왼스틱 가로는 무시", J.command_from_gamepad(pad, *R), (0.0, 0.0, 0.0, False, False))
ev(J.JS_EVENT_AXIS, 0, 0); ev(J.JS_EVENT_AXIS, 2, -32767); pad.poll(); check("오른스틱 왼쪽 끝 = 좌회전(+wz) 최대", J.command_from_gamepad(pad, *R), (0.0, 1.0, 0.0, False, False))
ev(J.JS_EVENT_AXIS, 2, 0); ev(J.JS_EVENT_AXIS, 3, -32767); pad.poll(); check("오른스틱 세로는 무시", J.command_from_gamepad(pad, *R), (0.0, 0.0, 0.0, False, False))
ev(J.JS_EVENT_AXIS, 3, 0); ev(J.JS_EVENT_AXIS, 5, 32767); pad.poll(); check("RT 끝까지 = 높이 +1", J.command_from_gamepad(pad, *R), (0.0, 0.0, 1.0, False, False))
ev(J.JS_EVENT_AXIS, 5, -32767); ev(J.JS_EVENT_AXIS, 4, 32767); pad.poll(); check("LT 끝까지 = 높이 -1", J.command_from_gamepad(pad, *R), (0.0, 0.0, -1.0, False, False))
ev(J.JS_EVENT_AXIS, 4, -32767); ev(J.JS_EVENT_AXIS, 1, 2000); pad.poll(); check("데드존 안 = 0", J.command_from_gamepad(pad, *R), (0.0, 0.0, 0.0, False, False))
ev(J.JS_EVENT_AXIS, 1, -32767); ev(J.JS_EVENT_BUTTON, 0, 1); pad.poll(); check("A 누르면 비상정지", J.command_from_gamepad(pad, *R), (0.0, 0.0, 0.0, True, False))
print("=> 전부 PASS" if ok else "=> FAIL 있음")
