"""Xbox 컨트롤러로 휠-레그 로봇 명령(vx, wz, 높이)을 실시간 조종한다.

오리 프로젝트(SummerProject/source/open_duck_mini_isaaclab/joystick_input.py)의
Gamepad 를 그대로 가져왔다 — joydev 직접 읽기, 데드존, 축 배치 자동 판별, A = 비상정지.
바꾼 것은 맨 아래 명령 매핑뿐이다 (이 로봇은 옆으로 못 가고 대신 높이를 바꾼다).

--- 이하 원문 설명 ---


리눅스 joydev 인터페이스(`/dev/input/js0`)를 직접 읽는다. pygame이나 evdev를
쓰지 않는 이유는 두 가지다: Isaac 파이썬에 그 패키지들이 있으리라는 보장이 없고,
joydev 프로토콜은 8바이트 고정 구조라 표준 라이브러리 `struct`만으로 충분하다.
의존성이 없으니 Isaac Sim 없이도 이 파일 하나만 따로 테스트할 수 있다
(`tests/test_joystick_input.py`).

joydev 이벤트 (리눅스 커널 `linux/joystick.h`):

    uint32 time      밀리초 타임스탬프
    int16  value     축이면 -32767..32767, 버튼이면 0/1
    uint8  type      0x01 버튼, 0x02 축, 0x80 은 초기 상태 통보 플래그
    uint8  number    축/버튼 번호

장치를 열면 커널이 현재 상태를 JS_EVENT_INIT 플래그가 붙은 이벤트로 한 번씩
보내준다. 이걸 버리면 안 된다 — 버리면 사용자가 스틱을 건드리기 전까지 축이
0인지 아닌지 알 수 없다.

읽기는 논블로킹이다. 시뮬 루프는 60Hz로 돌아야 하는데, 조이스틱 입력이 없다고
루프가 멈추면 안 된다. 매 스텝 `poll()`로 밀린 이벤트만 훑고 즉시 반환한다.

**축 번호는 패드/드라이버마다 다르다.** 실제로 두 배치를 만났다:

    고전 xpad (유선)          최신 hid (Xbox Wireless, 이 PC에 물린 것)
    ------------------------  --------------------------------------
    0 왼쪽 X                  0 왼쪽 X
    1 왼쪽 Y                  1 왼쪽 Y
    2 LT                      2 오른쪽 X
    3 오른쪽 X                3 오른쪽 Y
    4 오른쪽 Y                4 LT
    5 RT                      5 RT
    6/7 D-pad                 6/7 D-pad

하드코딩했다가 실제로 틀렸다 — 오른쪽 스틱 X라고 믿은 3번이 이 패드에서는
오른쪽 스틱 **Y**였다. 그래서 배치를 자동으로 판별한다. 단서는 트리거다:
트리거는 안 누르면 -1을 내고 스틱은 0을 낸다. 쉴 때 -1인 축이 어디냐로
두 배치가 갈린다. `scripts/joystick_check.py --raw`로 눈으로도 볼 수 있다.
"""

from __future__ import annotations

import os
import struct

_EVENT_FORMAT = "IhBB"
_EVENT_SIZE = struct.calcsize(_EVENT_FORMAT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

#: 스틱을 놓아도 정확히 0으로 돌아오지 않는다. 이 값 이하는 0으로 본다.
#: 로봇 명령에 그대로 실리면 "정지" 명령이 영영 안 나온다.
DEFAULT_DEADZONE = 0.12

# 왼쪽 스틱은 두 배치가 같다. 오른쪽 스틱 X만 갈리므로 그것만 판별한다.
AXIS_LEFT_X, AXIS_LEFT_Y = 0, 1
AXIS_RIGHT_X_CLASSIC = 3  # 고전 xpad (2=LT, 5=RT)
AXIS_RIGHT_X_MODERN = 2  # 최신 hid (4=LT, 5=RT)

#: 트리거는 안 누르면 이 값 이하로 쉰다. 스틱은 0 근처다.
_TRIGGER_REST = -0.9

BUTTON_A = 0


class GamepadUnavailable(RuntimeError):
    """장치가 없거나 열 수 없을 때."""


class Gamepad:
    """joydev 장치 하나를 논블로킹으로 읽는다."""

    def __init__(self, path: str = "/dev/input/js0", deadzone: float = DEFAULT_DEADZONE):
        self.path = path
        self.deadzone = deadzone
        self._axes: dict[int, float] = {}
        self._buttons: dict[int, bool] = {}
        self.connected = False
        #: 판별 전까지는 고전 배치를 가정한다. 첫 poll()에서 확정된다.
        self.axis_right_x = AXIS_RIGHT_X_CLASSIC
        self.layout = "미판별"
        self._layout_done = False
        try:
            self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            raise GamepadUnavailable(
                f"{path} 를 열 수 없습니다 ({exc}). 컨트롤러가 연결되어 있는지 "
                "확인하세요 — 유선 Xbox 패드는 꽂으면 커널 xpad 드라이버가 바로 "
                "잡습니다. `ls /dev/input/js*` 로 확인할 수 있습니다."
            ) from exc
        self.connected = True

    def poll(self) -> None:
        """밀린 이벤트를 모두 소화한다. 없으면 바로 반환한다."""
        if not self.connected:
            return
        while True:
            try:
                data = os.read(self._fd, _EVENT_SIZE)
            except BlockingIOError:
                return  # 더 읽을 것이 없다 — 정상
            except OSError:
                # 실행 중 케이블이 빠지면 여기로 온다. 죽지 말고 중립으로 둔다.
                self.connected = False
                self._axes.clear()
                self._buttons.clear()
                return
            if not data or len(data) < _EVENT_SIZE:
                return
            self._apply(data)
            # 장치를 열면 커널이 현재 상태를 INIT 이벤트로 한 번씩 보내주므로,
            # 사용자가 아무것도 안 해도 첫 poll 에서 배치를 판별할 수 있다.
            if not self._layout_done and len(self._axes) >= 6:
                self.detect_layout()

    def detect_layout(self) -> str:
        """트리거 위치로 축 배치를 판별한다.

        트리거는 안 누르면 -1, 스틱은 0에서 쉰다. 두 배치에서 -1로 쉬는 자리가
        다르다 — 고전은 2번(LT), 최신은 4번(LT). 그래서 두 자리를 함께 본다.

        한 자리만 보면 틀린다. 연결 순간에 오른쪽 스틱을 왼쪽으로 밀고 있으면
        최신 배치의 2번이 -1이 되어 고전으로 오인한다(테스트에서 잡혔다).
        둘 다 -1이면 구분이 안 되므로 요즘 흔한 최신 배치로 가정하고,
        `joystick_check.py --raw` 로 확인하라고 알린다.
        """
        two_is_trigger = self._axes.get(2, 0.0) <= _TRIGGER_REST
        four_is_trigger = self._axes.get(4, 0.0) <= _TRIGGER_REST

        if four_is_trigger and not two_is_trigger:
            self.axis_right_x = AXIS_RIGHT_X_MODERN
            self.layout = "최신 hid (4=LT, 오른쪽 스틱 X=2)"
        elif two_is_trigger and not four_is_trigger:
            self.axis_right_x = AXIS_RIGHT_X_CLASSIC
            self.layout = "고전 xpad (2=LT, 오른쪽 스틱 X=3)"
        else:
            self.axis_right_x = AXIS_RIGHT_X_MODERN
            self.layout = (
                "판별 모호 -> 최신 hid 로 가정 (연결 순간 스틱/트리거를 잡고 "
                "있었을 수 있습니다. joystick_check.py --raw 로 확인하세요)"
            )
        self._layout_done = True
        return self.layout

    def _apply(self, data: bytes) -> None:
        _, value, ev_type, number = struct.unpack(_EVENT_FORMAT, data)
        # 초기 상태 통보도 실제 상태이므로 플래그만 떼고 똑같이 반영한다.
        ev_type &= ~JS_EVENT_INIT
        if ev_type == JS_EVENT_AXIS:
            self._axes[number] = max(-1.0, value / 32767.0)
        elif ev_type == JS_EVENT_BUTTON:
            self._buttons[number] = bool(value)

    def axis(self, number: int) -> float:
        """-1..1. 데드존 안이면 0. 데드존 바깥은 0에서 다시 시작하도록 재조정한다
        (안 하면 스틱을 살짝 밀자마자 명령이 뚝 튄다)."""
        raw = self._axes.get(number, 0.0)
        if abs(raw) <= self.deadzone:
            return 0.0
        scaled = (abs(raw) - self.deadzone) / (1.0 - self.deadzone)
        return scaled if raw > 0 else -scaled

    def button(self, number: int) -> bool:
        return self._buttons.get(number, False)

    def close(self) -> None:
        if self.connected:
            os.close(self._fd)
            self.connected = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()



#: 오른쪽 스틱 Y. 두 배치(고전 xpad / 최신 hid)에서 번호가 갈린다.
AXIS_RIGHT_Y_CLASSIC = 4
AXIS_RIGHT_Y_MODERN = 3
BUTTON_B = 1


def right_y_axis(pad: "Gamepad") -> int:
    return AXIS_RIGHT_Y_MODERN if pad.axis_right_x == AXIS_RIGHT_X_MODERN else AXIS_RIGHT_Y_CLASSIC


def command_from_gamepad(
    pad: "Gamepad",
    lin_vel_x_range: tuple[float, float],
    ang_vel_z_range: tuple[float, float],
) -> tuple[float, float, float, bool, bool]:
    """스틱 -> (vx, wz, 높이 변화 입력 -1..1, 비상정지, 높이 초기화).

    배치 (오리와 같은 손 위치를 유지했다):

        왼쪽 스틱  세로 = 전진/후진 (vx)
        왼쪽 스틱  가로 = 제자리 회전 (wz)
        오른쪽 스틱 세로 = 높이 올리기/내리기 (위로 밀면 키가 커진다)
        A = 비상정지 (vx = wz = 0, 높이는 유지)
        B = 높이를 기본값으로

    높이는 속도(rate) 입력이다. 스틱을 놓으면 그 높이에 머문다 — 실기에서 손을
    떼자마자 쪼그려 앉으면 위험하므로 위치 입력으로 하지 않았다.
    방향 규약: +x 앞, yaw 반시계 +. 스틱은 위/왼쪽이 음수라 부호를 뒤집는다.
    """
    estop = pad.button(BUTTON_A)
    reset_h = pad.button(BUTTON_B)
    if estop:
        return 0.0, 0.0, 0.0, True, reset_h
    vx = -pad.axis(AXIS_LEFT_Y)
    wz = -pad.axis(AXIS_LEFT_X)
    dh = -pad.axis(right_y_axis(pad))
    return _scale(vx, lin_vel_x_range), _scale(wz, ang_vel_z_range), dh, False, reset_h


def _scale(value: float, rng: tuple[float, float]) -> float:
    """-1..1 을 범위로 편다. 끝까지 밀면 학습 범위 상한에 정확히 닿는다."""
    lo, hi = rng
    if hi == lo:
        return lo
    return value * (hi if value >= 0 else -lo)
