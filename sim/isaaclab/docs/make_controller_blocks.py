#!/usr/bin/env python3
"""제어기 블록선도 페이지 생성 -> controller_blocks.html (인쇄용, 도면 SVG 5 장).

값은 scripts/climb_test.py TUNE 과 tasks/residual.py (2026-09-27, 바퀴 140 mm 모델) 에서 옮겼다.
블록을 고치면 이 파일을 고치고 다시 돌린다:  python3 make_controller_blocks.py
"""
import re
from pathlib import Path

OUT = Path(__file__).with_name("controller_blocks.html")


def fmt(t: str) -> str:
    """x_{ref} -> 아래첨자, x^{2} -> 위첨자."""
    t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    t = re.sub(r"_\{([^}]*)\}", r'<tspan baseline-shift="sub" font-size="72%">\1</tspan>', t)
    t = re.sub(r"\^\{([^}]*)\}", r'<tspan baseline-shift="super" font-size="72%">\1</tspan>', t)
    return t


class D:
    def __init__(self, w, h, title):
        self.w, self.h, self.title, self.e = w, h, title, []

    def add(self, s):
        self.e.append(s)

    def text(self, x, y, t, cls="sg", anchor="start"):
        self.add(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}">{fmt(t)}</text>')

    def box(self, x, y, w, h, title, sub="", cls="bx"):
        self.add(f'<rect class="{cls}" x="{x}" y="{y}" width="{w}" height="{h}" rx="3"/>')
        lines = [("t", l) for l in title.split("\n") if l] + [("ts", l) for l in sub.split("\n") if l]
        lh = 14.5
        y0 = y + h / 2 - (len(lines) - 1) * lh / 2 + 4.5
        for i, (c, l) in enumerate(lines):
            self.text(x + w / 2, y0 + i * lh, l, c, "middle")
        return x, y, w, h

    def gain(self, x, y, label, w=46, h=34):
        """삼각형 게인. (x, y) = 왼쪽 가운데."""
        self.add(f'<path class="bx" d="M{x},{y - h / 2} L{x + w},{y} L{x},{y + h / 2} Z"/>')
        self.text(x + w * 0.36, y + 4, label, "tg", "middle")
        return x + w

    def sum(self, cx, cy, signs):
        """합산점. signs: {'w': '+', 'n': '−', ...} 들어오는 쪽 부호."""
        self.add(f'<circle class="bx" cx="{cx}" cy="{cy}" r="9"/>')
        self.add(f'<path class="thin" d="M{cx - 6.4},{cy - 6.4} L{cx + 6.4},{cy + 6.4} M{cx - 6.4},{cy + 6.4} L{cx + 6.4},{cy - 6.4}"/>')
        off = {"w": (-15, -5), "n": (-11, -12), "s": (-11, 21), "e": (12, -5)}
        for k, s in signs.items():
            dx, dy = off[k]
            self.text(cx + dx, cy + dy, s, "pm", "middle")

    def arr(self, pts, cls="ln", head=True):
        d = "M" + " L".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        mk = "" if not head else (' marker-end="url(#ahl)"' if cls == "lg" else ' marker-end="url(#ah)"')
        self.add(f'<path class="{cls}" d="{d}"{mk}/>')

    def dot(self, x, y):
        self.add(f'<circle class="dot" cx="{x}" cy="{y}" r="2.6"/>')

    def svg(self):
        return (f'<svg class="dg" viewBox="0 0 {self.w} {self.h}" role="img" aria-label="{self.title}">'
                + "".join(self.e) + "</svg>")


# ============================================================================================
# 도면 1: 전체 구조
# ============================================================================================
def fig_overview():
    d = D(1200, 640, "전체 구조")
    # 입력
    d.box(20, 40, 140, 78, "패드 (사람)", "v_{cmd}  ω_{z,cmd}\nA: 다리 수동 h_{c}\nY/LT/RT: 점프")
    d.box(20, 262, 140, 58, "IMU", "iAHRS: g_{b}, ω_{b}, 비력")
    d.box(20, 344, 140, 72, "모터 CAN", "고관절 M, Ṁ, τ_{hip}\n바퀴 ω_{W}")
    # 추정
    d.box(210, 236, 170, 196, "상태 추정", "θ, θ̇  진자각\nv  바퀴 속도\nl  진자 길이\nφ, φ̇  roll\nω_{z}  yaw 속도\nc_{L}, c_{R}  접지")
    d.arr([(160, 291), (210, 291)]); d.arr([(160, 380), (210, 380)])
    # 속도 관리
    d.box(440, 40, 170, 78, "속도 관리", "최고 3 km/h · PI 브레이크\n가속 1.5 m/s² · 푸시백")
    d.arr([(160, 64), (440, 64)]); d.text(172, 58, "v_{cmd}")
    # LQR
    d.box(680, 130, 160, 82, "바퀴 LQR", "τ_{w} = −K(l) · x\nx = [x_{err}, e_{v}, θ, θ̇]")
    d.arr([(610, 94), (650, 94), (650, 160), (680, 160)]); d.text(618, 88, "v_{ref}, x_{err}")
    d.arr([(380, 262), (620, 262), (620, 190), (680, 190)]); d.text(392, 256, "θ, θ̇, v, l")
    d.arr([(430, 262), (430, 104), (440, 104)]); d.dot(430, 262); d.text(404, 100, "v", anchor="end")
    # 조향
    d.box(680, 244, 160, 58, "조향 P", "τ_{y} = 0.5 (ω_{z,cmd} − ω_{z})")
    d.arr([(160, 80), (190, 80), (190, 208), (650, 208), (650, 262), (680, 262)]); d.text(196, 202, "ω_{z,cmd}")
    d.arr([(380, 290), (680, 290)]); d.text(392, 284, "ω_{z}")
    # 좌우 분리
    d.box(890, 150, 120, 150, "좌우 분리", "τ_{L} = ½τ_{w} − τ_{y}\nτ_{R} = ½τ_{w} + τ_{y}\n|τ| ≤ 7 N·m")
    d.arr([(840, 171), (890, 171)]); d.text(848, 165, "τ_{w}")
    d.arr([(840, 273), (890, 273)]); d.text(848, 267, "τ_{y}")
    d.box(1050, 176, 130, 98, "바퀴 모터 ×2", "AK45-10\n지연 5 ms(+지터)\nLPF 20 Hz")
    d.arr([(1010, 225), (1050, 225)]); d.text(1016, 219, "τ_{L,R}")
    # roll · 다리
    d.box(440, 452, 170, 82, "roll PI", "Δ = h_{L} − h_{R}\nk_{p} 1.5 · k_{i} 15 · k_{d} 0.3")
    d.arr([(380, 400), (410, 400), (410, 480), (440, 480)]); d.text(386, 394, "φ, φ̇")
    d.arr([(190, 208), (190, 222), (400, 222), (400, 440), (425, 440), (425, 506), (440, 506)])
    d.text(404, 520, "φ_{ref}=atan(v_{ref}ω_{z}/g)", "sgs", "end")
    d.box(680, 452, 160, 82, "다리 높이", "h_{L,R} = h_{c} ± Δ/2\n122.5 ~ 242.5 mm")
    d.arr([(610, 493), (680, 493)]); d.text(618, 487, "Δ")
    d.arr([(160, 100), (176, 100), (176, 612), (760, 612), (760, 534)]); d.text(184, 606, "h_{c} (자동 182.5 mm / 수동)")
    d.box(890, 452, 120, 82, "VMC", "관절 PD 60 / 1\n+ Jᵀ(½ m g)")
    d.arr([(840, 493), (890, 493)]); d.text(846, 487, "h_{L,R}")
    d.box(1050, 452, 130, 82, "고관절 모터 ×2", "AK60-6\n지연 5 ms(+지터)\n|τ| ≤ 9 N·m")
    d.arr([(1010, 493), (1050, 493)]); d.text(1014, 487, "τ_{hip}")
    d.arr([(300, 432), (300, 560), (950, 560), (950, 534)]); d.text(560, 574, "M, Ṁ (고관절 각도·속도)", "sgs", "middle")
    # 들림 판정 (논리)
    d.box(440, 316, 170, 96, "들림 · 착지 판정", "들림: 무하중 0.3 s\n착지: 접지·비력 0.02 s\n들리면 균형 끔", cls="bxm")
    d.arr([(380, 364), (440, 364)], "lg"); d.text(386, 358, "c, 비력", "sgl")
    d.arr([(610, 336), (760, 336), (760, 302)], "lg"); d.text(620, 330, "τ_{w} → −0.05 ω_{W}", "sgl")
    d.arr([(610, 392), (760, 392), (760, 452)], "lg"); d.text(620, 408, "h → IDLE, 적분 비움", "sgl")
    # 로봇 (플랜트) 되먹임
    d.box(1050, 300, 130, 128, "로봇", "바퀴 역진자\n+ 4절 링크 다리\n4.04 kg\n바퀴 Ø140", cls="bxp")
    d.arr([(1115, 274), (1115, 300)]); d.arr([(1115, 452), (1115, 428)])
    d.arr([(1180, 364), (1192, 364), (1192, 628), (8, 628), (8, 291), (20, 291)], "fb")
    d.arr([(8, 380), (20, 380)], "fb"); d.dot(8, 380)
    d.text(600, 622, "센서", "sgs", "middle")
    return d


# ============================================================================================
# 도면 2: 바퀴 채널 (속도 관리 + LQR + 조향 + 분리)
# ============================================================================================
def fig_wheel():
    d = D(1200, 760, "바퀴 채널")
    # --- 최고 속도 ---
    d.text(20, 30, "① 최고 속도", "hd")
    d.box(20, 46, 150, 44, "ω_{max} · R · k_{motor}", ""); d.text(95, 104, "k_{motor}: 배터리 전압으로 추정", "sgs", "middle")
    x = d.gain(210, 68, "0.75")
    d.arr([(170, 68), (210, 68)])
    d.box(300, 46, 120, 44, "min( · , 3 km/h)", "")
    d.arr([(x, 68), (300, 68)])
    d.arr([(420, 68), (1010, 68), (1010, 159)]); d.text(480, 62, "v_{max}")
    # --- PI 브레이크 ---
    d.text(20, 132, "② 과속 PI 브레이크 (내리막)", "hd")
    d.text(20, 172, "v", anchor="start"); d.arr([(34, 168), (70, 168)])
    d.box(70, 146, 110, 44, "LPF 4 Hz", "1 / (τs + 1)")
    d.box(210, 146, 60, 44, "| · |", "")
    d.arr([(180, 168), (210, 168)])
    d.sum(310, 168, {"w": "+", "n": "−"}); d.arr([(270, 168), (301, 168)])
    d.arr([(460, 68), (460, 118), (310, 118), (310, 159)]); d.dot(460, 68)
    d.arr([(319, 168), (360, 168)]); d.text(328, 162, "e")
    d.dot(360, 168)
    d.box(380, 132, 110, 30, "max(e, 0)", ""); d.arr([(360, 168), (360, 147), (380, 147)])
    x = d.gain(520, 147, "1", 40, 28); d.arr([(490, 147), (520, 147)])
    d.box(380, 176, 110, 44, "∫ k_{i} = 5.5", "0 ≤ · ≤ v_{max}"); d.arr([(360, 168), (360, 198), (380, 198)])
    d.sum(620, 168, {"n": "+", "s": "+"})
    d.arr([(x, 147), (620, 147), (620, 159)]); d.arr([(490, 198), (620, 198), (620, 177)])
    d.sum(1010, 168, {"n": "+", "w": "−"}); d.arr([(629, 168), (1001, 168)]); d.text(640, 162, "제동량")
    d.dot(1010, 68)
    d.box(1050, 146, 120, 44, "max( · , 0)", ""); d.arr([(1019, 168), (1050, 168)])
    d.arr([(1110, 190), (1110, 232), (270, 232), (270, 262)]); d.text(1100, 226, "v_{lim}", anchor="end")
    # --- 명령 경로 ---
    d.text(20, 250, "③ 목표 속도", "hd")
    d.text(20, 290, "v_{cmd}"); d.arr([(62, 286), (220, 286)])
    d.box(220, 262, 110, 48, "포화 ± v_{lim}", "")
    d.box(370, 262, 120, 48, "변화율 제한", "± 1.5 m/s²")
    d.arr([(330, 286), (370, 286)])
    d.box(530, 256, 170, 60, "푸시백 전환", "바퀴 > 한계 80 % 이면\nv_{ref} = v(1 − 0.6·cut)", cls="bxm")
    d.arr([(490, 286), (540, 286)])
    d.box(540, 330, 150, 44, "max|ω_{W}| / ω_{max}", "")
    d.text(470, 356, "ω_{W,L/R}", anchor="end"); d.arr([(476, 352), (540, 352)])
    d.arr([(615, 330), (615, 316)], "lg")
    d.arr([(700, 286), (781, 286)]); d.text(708, 280, "v_{ref}")
    d.dot(740, 286)
    # --- LQR ---
    d.text(20, 408, "④ 바퀴 LQR (진자 길이 l 로 게인 보간)", "hd")
    d.sum(790, 286, {"w": "−", "n": "+"}); d.arr([(740, 250), (790, 250), (790, 277)]); d.text(746, 244, "v")
    d.arr([(799, 286), (840, 286)]); d.text(806, 280, "e_{v}"); d.dot(840, 286)
    d.box(860, 310, 140, 48, "∫ dt", "|x_{err}| ≤ 0.3 m\n속도 제한 중 0")
    d.arr([(840, 286), (840, 334), (860, 334)])
    ys = [440, 510, 580, 650]
    labels = [("x_{err}", "K_{x} −1.41"), ("e_{v}", "K_{v} −3.35"), ("θ", "K_{θ} −18.5"), ("θ̇", "K_{θ̇} −3.33")]
    d.arr([(1000, 334), (1030, 334), (1030, 400), (560, 400), (560, ys[0]), (600, ys[0])]); d.text(570, ys[0] - 6, labels[0][0])
    d.arr([(840, 286), (840, 390), (540, 390), (540, ys[1]), (600, ys[1])]); d.text(570, ys[1] - 6, labels[1][0])
    d.text(430, ys[2] + 4, "θ (추정)", anchor="end"); d.arr([(436, ys[2]), (600, ys[2])])
    d.text(430, ys[3] + 4, "θ̇ = 자이로 y", anchor="end"); d.arr([(436, ys[3]), (600, ys[3])])
    for (sig, g), y in zip(labels, ys):
        d.box(600, y - 20, 150, 40, g, "")
    d.arr([(750, ys[0]), (800, ys[0]), (800, 536)]); d.arr([(750, ys[1]), (800, ys[1])], head=False); d.dot(800, ys[1])
    d.arr([(750, ys[3]), (800, ys[3]), (800, 554)]); d.arr([(750, ys[2]), (800, ys[2])], head=False); d.dot(800, ys[2])
    d.sum(800, 545, {"n": "+", "s": "+"})
    d.box(600, 690, 150, 44, "K(l) 표 보간", "l = 0.12 ~ 0.40 m, 15 점")
    d.arr([(675, 690), (675, 670)], "lg"); d.text(470, 718, "l (진자 길이)", anchor="end"); d.arr([(476, 712), (600, 712)])
    x = d.gain(830, 545, "−1", 40, 30); d.arr([(809, 545), (830, 545)])
    d.arr([(x, 545), (905, 545)]); d.text(876, 539, "τ_{w}")
    # --- 조향 + 분리 ---
    d.text(930, 428, "⑤ 조향 · 좌우 분리", "hd")
    d.sum(960, 450, {"w": "+", "s": "−"}); d.text(860, 454, "ω_{z,cmd}"); d.arr([(912, 450), (951, 450)])
    d.text(960, 490, "ω_{z} 자이로", anchor="middle"); d.arr([(960, 478), (960, 459)])
    x = d.gain(975, 450, "0.5", 40, 28); d.arr([(969, 450), (975, 450)])
    d.arr([(x, 450), (1050, 450), (1050, 526)]); d.text(1022, 444, "τ_{y}")
    d.dot(905, 545)
    d.sum(1050, 535, {"w": "+", "n": "−"}); d.arr([(905, 535), (940, 535)]); xg = d.gain(940, 535, "½", 32, 24); d.arr([(xg, 535), (1041, 535)])
    d.sum(1050, 610, {"w": "+", "n": "+"}); d.arr([(905, 545), (905, 610), (940, 610)]); xg = d.gain(940, 610, "½", 32, 24); d.arr([(xg, 610), (1041, 610)])
    d.arr([(1050, 450), (1030, 450), (1030, 585), (1050, 585), (1050, 601)]); d.dot(1050, 450)
    d.box(1080, 516, 110, 38, "포화 ±7 N·m", "")
    d.box(1080, 591, 110, 38, "포화 ±7 N·m", "")
    d.arr([(1059, 535), (1080, 535)]); d.arr([(1059, 610), (1080, 610)])
    d.box(1070, 650, 124, 64, "지연 e^{−sT}", "T 5 ms (+5, 40 %)\n+ LPF 20 Hz")
    d.arr([(1135, 554), (1135, 574), (1060, 574), (1060, 666), (1070, 666)]); d.text(1054, 568, "τ_{L}", anchor="end")
    d.arr([(1135, 629), (1135, 640), (1100, 640), (1100, 650)]); d.text(1140, 646, "τ_{R}")
    d.arr([(1132, 714), (1132, 732)]); d.text(1132, 750, "바퀴 모터", "sgs", "middle")
    return d


# ============================================================================================
# 도면 3: 다리 채널 (roll PI + 높이 + VMC)
# ============================================================================================
def fig_legs():
    d = D(1200, 600, "다리 채널")
    d.text(20, 30, "① 회전 중 기울기 목표", "hd")
    d.text(20, 72, "v_{ref}, ω_{z,cmd}"); d.arr([(120, 68), (160, 68)])
    d.box(160, 46, 150, 44, "atan(v_{ref} ω_{z} / g)", "")
    d.box(340, 46, 100, 44, "포화 ±20°", "")
    d.arr([(310, 68), (340, 68)]); d.arr([(440, 68), (500, 68), (500, 131)]); d.text(450, 62, "φ_{ref}")
    d.text(20, 118, "② roll PI (좌우 다리 길이 차)", "hd")
    d.text(20, 144, "φ (IMU roll)"); d.arr([(100, 140), (491, 140)])
    d.sum(500, 140, {"w": "+", "n": "−"})
    d.box(530, 118, 130, 44, "b · sin( · )", "b = 0.198 m (바퀴 간격)")
    d.arr([(509, 140), (530, 140)]); d.arr([(660, 140), (690, 140)]); d.text(668, 134, "e_{r}"); d.dot(690, 140)
    x = d.gain(720, 110, "1.5", 44, 30); d.arr([(690, 140), (690, 110), (720, 110)])
    d.box(720, 150, 130, 48, "∫ k_{i} = 15", "새기 0.5 /s, |·| ≤ 0.10")
    d.arr([(690, 140), (690, 174), (720, 174)])
    d.text(20, 238, "−ω_{x} (자이로)"); d.arr([(110, 234), (530, 234)])
    d.box(530, 212, 110, 44, "LPF 8 Hz", "")
    x2 = d.gain(680, 234, "0.3b", 50, 30); d.arr([(640, 234), (680, 234)])
    d.sum(900, 174, {"n": "+", "w": "+", "s": "+"})
    d.arr([(x, 110), (900, 110), (900, 165)]); d.arr([(850, 174), (891, 174)]); d.arr([(x2, 234), (900, 234), (900, 183)])
    d.box(930, 152, 110, 44, "포화 ±0.10 m", ""); d.arr([(909, 174), (930, 174)])
    d.arr([(1040, 174), (1060, 174), (1060, 262), (1080, 262)]); d.text(1048, 168, "Δ")
    xg = d.gain(1080, 262, "½", 34, 26)
    d.arr([(xg, 262), (1150, 262), (1150, 375)], head=False); d.dot(1150, 375)
    d.arr([(1150, 375), (1150, 330), (1009, 330)]); d.arr([(1150, 375), (1150, 420), (1009, 420)])
    d.text(1120, 254, "Δ/2", anchor="start")
    d.box(700, 262, 200, 54, "적분 멈춤 조건", "한쪽 다리 무하중 (τ_{hip} < 0.8 N·m)\n또는 |φ| > 20°", cls="bxm")
    d.arr([(785, 262), (785, 198)], "lg")
    # --- 높이 가운데 ---
    d.text(20, 300, "③ 다리 높이", "hd")
    d.box(20, 318, 150, 44, "IDLE 182.5 mm", "자동")
    d.text(20, 398, "오른쪽 스틱 세로"); d.arr([(130, 394), (160, 394)])
    x = d.gain(160, 394, "0.08", 46, 30)
    d.box(236, 372, 120, 44, "∫", "142.5 ~ 222.5 mm"); d.arr([(x, 394), (236, 394)])
    d.box(400, 330, 110, 74, "A 버튼 전환", "자동 / 수동\n자동 복귀\n0.08 m/s", cls="bxm")
    d.arr([(170, 340), (400, 340)]); d.arr([(356, 394), (400, 394)])
    d.arr([(510, 367), (560, 367)]); d.text(520, 361, "h_{c}"); d.dot(560, 367)
    d.sum(1000, 330, {"w": "+", "e": "+"}); d.sum(1000, 420, {"w": "+", "e": "−"})
    d.arr([(560, 367), (560, 330), (991, 330)]); d.arr([(560, 367), (560, 420), (991, 420)])
    # --- VMC ---
    d.text(20, 470, "④ VMC (다리 하나씩, L·R 같음)", "hd")
    d.arr([(1000, 339), (1000, 350), (960, 350), (960, 480), (40, 480), (40, 520), (60, 520)]); d.text(66, 514, "h_{L}")
    d.box(60, 500, 110, 40, "포화", "122.5 ~ 242.5 mm")
    d.box(200, 500, 110, 40, "지연 e^{−sT}", "")
    d.box(340, 494, 150, 52, "M(h) 역기구학", "4절 링크 표 (leg_map)")
    d.arr([(170, 520), (200, 520)]); d.arr([(310, 520), (340, 520)])
    d.sum(540, 520, {"w": "+", "s": "−"}); d.arr([(490, 520), (531, 520)]); d.text(496, 514, "M_{ref}")
    d.text(540, 574, "M (엔코더)", anchor="middle"); d.arr([(540, 562), (540, 529)])
    x = d.gain(580, 520, "60", 44, 30); d.arr([(549, 520), (580, 520)])
    d.sum(700, 520, {"w": "+", "n": "−", "s": "+"}); d.arr([(x, 520), (691, 520)])
    d.box(640, 440, 120, 36, "k_{d} Ṁ, k_{d} = 1", ""); d.arr([(700, 476), (700, 511)])
    d.box(620, 560, 160, 36, "s · ½ m g · dh/dM", "자중 보상 Jᵀ F"); d.arr([(700, 560), (700, 529)])
    d.box(760, 500, 110, 40, "포화 ±9 N·m", ""); d.arr([(709, 520), (760, 520)])
    d.arr([(870, 520), (930, 520)]); d.text(880, 514, "τ_{hip,L}")
    d.text(940, 524, "→ 고관절 모터 (AK60-6)")
    d.text(1000, 440, "h_{R} 도 같은 경로", "sgs", "middle"); d.arr([(1000, 429), (1000, 432)], head=False)
    return d


# ============================================================================================
# 도면 4: 상태 추정 + 들림 판정
# ============================================================================================
def fig_est():
    d = D(1200, 540, "상태 추정")
    d.text(20, 30, "① 진자각 θ 와 진자 길이 l", "hd")
    d.text(20, 72, "IMU g_{b,x}"); d.arr([(96, 68), (130, 68)])
    d.box(130, 48, 90, 40, "asin", "")
    d.arr([(220, 68), (491, 68)]); d.text(240, 62, "pitch (바이어스 포함)")
    d.text(20, 146, "고관절 M_{L}, M_{R}"); d.arr([(120, 142), (150, 142)])
    d.box(150, 116, 190, 52, "4절 기구학 + 명목 질량", "몸통 좌표 무게중심 r")
    d.box(380, 100, 90, 34, "atan2(r_{x}, r_{z})", ""); d.box(380, 150, 90, 34, "|r|", "")
    d.arr([(340, 142), (360, 142), (360, 117), (380, 117)]); d.arr([(360, 142), (360, 167), (380, 167)])
    d.arr([(470, 117), (500, 117), (500, 77)]); d.text(508, 112, "θ_{kin}")
    d.arr([(470, 167), (560, 167)]); d.text(480, 161, "l → LQR 게인")
    d.sum(500, 68, {"w": "+", "s": "+", "n": "−"})
    d.arr([(509, 68), (640, 68)]); d.text(560, 62, "θ"); d.dot(600, 68)
    d.box(640, 44, 190, 64, "조건부 적분 (균형점 학습)", "θ̇_{bias} = 0.3 (θ − θ_{bias})\n|θ_{bias}| ≤ 8°", cls="bxm")
    d.arr([(830, 76), (860, 76), (860, 24), (500, 24), (500, 59)]); d.text(870, 50, "θ_{bias}")
    d.text(650, 128, "켜짐: |v| < 0.05, |v_{cmd}| < 0.02, |θ̇| < 0.3, |a| < 0.3 (서 있을 때만)", "sgl")
    d.arr([(600, 68), (600, 200), (1060, 200)]); d.text(1066, 204, "θ → LQR")
    d.text(20, 240, "자이로 ω_{y}, ω_{x}, ω_{z}"); d.arr([(160, 236), (1060, 236)]); d.text(1066, 240, "θ̇, φ̇, ω_{z}")
    d.text(20, 268, "IMU g_{b,y}"); d.box(96, 252, 70, 32, "asin", ""); d.arr([(166, 268), (1060, 268)]); d.text(1066, 272, "φ (roll)")
    # --- 접지 · 속도 ---
    d.text(20, 316, "② 접지와 바퀴 속도", "hd")
    d.text(20, 356, "τ_{hip,L}, τ_{hip,R}"); d.arr([(130, 352), (160, 352)])
    d.box(160, 332, 150, 40, "≥ 0.8 N·m ?", "+ = 다리가 몸을 받침")
    d.arr([(310, 352), (560, 352)]); d.text(320, 346, "c_{L}, c_{R}"); d.dot(390, 352)
    d.text(566, 356, "→ roll 적분 멈춤", "sgs")
    d.text(20, 420, "ω_{abs,L}, ω_{abs,R}"); d.text(20, 442, "(엔코더 + 링크 회전)", "sgs")
    d.arr([(140, 416), (430, 416)])
    d.box(430, 396, 180, 44, "R · Σ c_{i} ω_{i} / Σ c_{i}", "접지한 바퀴만 평균")
    d.arr([(390, 352), (390, 380), (520, 380), (520, 396)], "lg")
    d.box(650, 396, 150, 44, "LPF 10 Hz", "Σc = 0 이면 직전 값 유지")
    d.arr([(610, 418), (650, 418)]); d.arr([(800, 418), (1060, 418)]); d.text(1066, 422, "v → 속도 관리·LQR")
    # --- 들림 상태기계 ---
    d.text(20, 478, "③ 들림 / 착지", "hd")
    d.box(430, 468, 130, 50, "DRIVE", "균형 제어", cls="st")
    d.box(820, 468, 130, 50, "LIFT", "바퀴 −0.05 ω_{W}\n다리 IDLE, 적분 0", cls="st")
    d.arr([(560, 482), (820, 482)], "lg"); d.text(690, 476, "c_{L} = c_{R} = 0 이 0.3 s", "sgl", "middle")
    d.arr([(820, 506), (560, 506)], "lg"); d.text(690, 528, "(c_{L} or c_{R}) 이고 비력 > 0.4 g 가 0.02 s", "sgl", "middle")
    d.arr([(390, 352), (390, 494), (430, 494)], "lg")
    return d


# ============================================================================================
# 도면 5: 점프 상태기계
# ============================================================================================
def fig_jump():
    d = D(1200, 330, "점프 상태기계")
    st = [("DRIVE", "LQR + VMC"), ("RETRACT", "0.30 s\n다리 → 최저 (0.225 s)\nLQR θ_{ref} = 3°"),
          ("EXTRACT", "다리 → 최고\n바퀴 pitch PD\n30·θ + 3·ω_{y}"),
          ("FLY", "0.12 s 다리 접음\n바퀴 = 반작용 휠\n목표 −10°"),
          ("DESCEND", "다리 → 182.5 mm\n부드럽게 k_{p} 20\n바퀴 PD 계속"),
          ("LAND", "0.30 s\nk_{p} 20 → 60\nLQR 복귀")]
    xs = [20, 215, 410, 605, 800, 995]
    for (n, s), x in zip(st, xs):
        d.box(x, 60, 175, 110, n, s, cls="st")
    conds = ["Y / LT / RT\nv ≥ 0.2 m/s, |ω_{z}| ≤ 1", "0.30 s", "다리 ≥ 최고 − 8 mm\n또는 0.25 s",
             "0.12 s", "고관절 토크 > 2 N·m\n또는 이륙 뒤 0.45 s"]
    for i, c in enumerate(conds):
        x0, x1 = xs[i] + 175, xs[i + 1]
        d.arr([(x0, 115), (x1, 115)], "lg")
        for j, line in enumerate(c.split("\n")):
            d.text((x0 + x1) / 2, 196 + j * 15, line, "sgl", "middle")
    d.arr([(1082, 170), (1082, 280), (107, 280), (107, 170)], "lg"); d.text(600, 300, "0.30 s 뒤 DRIVE (다음 턱으로)", "sgl", "middle")
    d.text(20, 36, "창 모드(climb_test.py)와 시험 세트(wbctrl.py)에만 있다. 바퀴 PD 토크 배율 7, 공중 목표 pitch −10°, 착지 다리 182.5 mm", "sgs")
    return d


FIGS = [
    ("overview", "1. 전체 구조", "실선 = 신호, 점선 = 모드 전환·논리, 굵은 되먹임 = 센서. 모든 명령은 5 ms 제어 주기(200 Hz)로 나가고, 모터 앞에서 5 ms 지연(+40 % 확률로 5 ms 더)을 거친다.", fig_overview),
    ("wheel", "2. 바퀴 채널: 속도 관리 · LQR · 조향 · 좌우 분리", "LQR 게인은 l = 0.25 m 값 (Q = diag(2, 5, 100, 5), R = 1). 실제로는 매 스텝 진자 길이 l 로 표를 보간한다. 푸시백이 켜지면 x_err 도 0 으로 비운다.", fig_wheel),
    ("legs", "3. 다리 채널: roll PI · 높이 · VMC", "Δ 는 왼쪽이 길면 +. roll 은 왼쪽이 낮으면 + (asin g_y). 수동 모드에서도 roll 수평 맞추기는 그대로 돈다.", fig_legs),
    ("est", "4. 상태 추정과 들림 판정", "IMU 가속도 적분은 쓰지 않는다 (Ascento 2 논문과 같은 판단). 무게중심은 CAD 명목 질량으로 계산하므로 실제와의 차이는 균형점 학습이 잡는다.", fig_est),
    ("jump", "5. 점프 상태기계", "", fig_jump),
]

PARAMS = [
    ("바퀴 LQR", "Q = diag(2, 5, 100, 5), R = 1, l 격자 0.12~0.40 m 15 점", "lqr_vmc.py:45, TUNE lqr_*"),
    ("속도 관리", "v_max = min(3 km/h, 0.75 ω_max R), PI 1 / 5.5, LPF 4 Hz, 가속 1.5 m/s², 푸시백 0.8", "residual.py:185-211"),
    ("조향", "k_ψ = 0.5 N·m·s/rad, 회전 한계 끔 (사람이 조심)", "residual.py:216-221"),
    ("roll PI", "k_p 1.5, k_i 15, k_d 0.3 (LPF 8 Hz), 새기 0.5 /s, |Δ| ≤ 0.10 m, 멈춤 20°", "residual.py:236-244, lqr_vmc.py:57"),
    ("다리 VMC", "관절 PD 60 / 1, 자중 ½ m g Jᵀ, 행정 122.5~242.5 mm, IDLE 182.5 mm", "residual.py:245, 269-280"),
    ("상태 추정", "접지 0.8 N·m, 속도 LPF 10 Hz, 균형점 학습 0.3 /s ±8°", "residual.py:154-181"),
    ("들림 판정", "들림 0.3 s, 착지 0.02 s + 비력 0.4 g, 들린 동안 바퀴 −0.05 ω", "residual.py:222-250"),
    ("지연", "명령 5 ms + 40 % 확률 +5 ms, 바퀴 토크 LPF 20 Hz", "residual.py:258-270, cad.py:157"),
]


def page():
    figs = "\n".join(
        f'<section class="fig" id="{fid}"><h2>{title}</h2><div class="scroll">{fn().svg()}</div>'
        + (f'<p class="cap">{cap}</p>' if cap else "") + "</section>"
        for fid, title, cap, fn in FIGS)
    rows = "\n".join(f"<tr><td>{a}</td><td>{b}</td><td><code>{c}</code></td></tr>" for a, b, c in PARAMS)
    return TEMPLATE.replace("{{FIGS}}", figs).replace("{{ROWS}}", rows)


TEMPLATE = r"""<title>PERSEVERANCE 블록선도</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+KR:wght@400;500;600&family=JetBrains+Mono:wght@400;500&family=Noto+Serif:ital,wght@0,400;1,400&family=Noto+Serif+KR:wght@400&display=swap">
<style>
:root {
  --bg: #EEF1F4; --paper: #FFFFFF; --ink: #1A1E23; --muted: #5A636E; --rule: #D3D9E0;
  --blk: #FFFFFF; --blk2: #F6F2E8; --plant: #EAF1F7; --st: #EEF3F8;
  --sig: #1D5A86; --logic: #9A6400; --fb: #6B7480;
  --sans: "IBM Plex Sans KR", "Noto Sans KR", sans-serif; --mono: "JetBrains Mono", ui-monospace, monospace;
  --math: "Noto Serif", "Noto Serif KR", serif;
  color-scheme: light;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --bg: #0F1216; --paper: #161A1F; --ink: #E2E6EA; --muted: #9AA4AF; --rule: #2B323B;
    --blk: #1C2127; --blk2: #2A2519; --plant: #1A2530; --st: #1B242E;
    --sig: #86B8DE; --logic: #E3B35E; --fb: #8A939E; color-scheme: dark;
  }
}
:root[data-theme="dark"] {
  --bg: #0F1216; --paper: #161A1F; --ink: #E2E6EA; --muted: #9AA4AF; --rule: #2B323B;
  --blk: #1C2127; --blk2: #2A2519; --plant: #1A2530; --st: #1B242E;
  --sig: #86B8DE; --logic: #E3B35E; --fb: #8A939E; color-scheme: dark;
}
body { background: var(--bg); color: var(--ink); font-family: var(--sans); font-size: 14.5px; line-height: 1.6;
  padding-inline: 16px; padding-block: 24px 56px; }
.sheet { max-width: 1240px; margin: 0 auto; background: var(--paper); border: 1px solid var(--rule);
  padding-block: 36px 44px; padding-inline: clamp(14px, 3vw, 40px); }
h1 { font-size: 26px; margin: 0 0 4px; font-weight: 600; text-wrap: balance; }
h2 { font-size: 17px; font-weight: 600; margin: 0 0 8px; padding-top: 12px; border-top: 2px solid var(--ink); }
.meta { color: var(--muted); font-size: 13px; margin: 0 0 18px; }
.legend { display: flex; flex-wrap: wrap; gap: 8px 22px; font-size: 13px; color: var(--muted); margin: 0 0 22px; }
.legend span { display: inline-flex; align-items: center; gap: 7px; }
.legend svg { width: 42px; height: 20px; }
.fig { margin: 0 0 30px; break-inside: avoid; }
.scroll { overflow-x: auto; }
svg.dg { display: block; width: 100%; min-width: 900px; height: auto; }
.cap { color: var(--muted); font-size: 13px; margin: 6px 0 0; max-width: 90ch; }
table { border-collapse: collapse; width: 100%; font-size: 13px; }
th { text-align: left; font-weight: 600; border-bottom: 1.5px solid var(--ink); padding: 6px 8px; }
td { border-bottom: 1px solid var(--rule); padding: 6px 8px; vertical-align: top; }
code { font-family: var(--mono); font-size: 12px; }
.tblwrap { overflow-x: auto; }
/* 도면 */
.bx { fill: var(--blk); stroke: var(--ink); stroke-width: 1.3; }
.bxm { fill: var(--blk2); stroke: var(--logic); stroke-width: 1.3; stroke-dasharray: 5 3; }
.bxp { fill: var(--plant); stroke: var(--ink); stroke-width: 1.6; }
.st { fill: var(--st); stroke: var(--ink); stroke-width: 1.4; }
.ln { fill: none; stroke: var(--ink); stroke-width: 1.35; }
.lg { fill: none; stroke: var(--logic); stroke-width: 1.3; stroke-dasharray: 5 4; }
.fb { fill: none; stroke: var(--fb); stroke-width: 2.2; }
.thin { fill: none; stroke: var(--ink); stroke-width: 0.9; }
.dot { fill: var(--ink); }
.ah { fill: var(--ink); } .ahl { fill: var(--logic); }
text { font-family: var(--sans); fill: var(--ink); }
.t { font-size: 13px; font-weight: 600; }
.ts { font-size: 11px; fill: var(--muted); font-family: var(--mono); }
.tg { font-size: 11.5px; font-family: var(--mono); font-weight: 500; }
.sg { font-family: var(--math); font-style: italic; font-size: 13px; fill: var(--sig); }
.sgs { font-size: 11.5px; fill: var(--muted); }
.sgl { font-size: 11.5px; fill: var(--logic); }
.pm { font-size: 13px; font-weight: 600; font-family: var(--mono); }
.hd { font-size: 12px; font-weight: 600; fill: var(--muted); letter-spacing: 0.02em; }
@page { size: A4 landscape; margin: 10mm; }
@media print {
  :root, :root[data-theme="dark"] { --bg: #fff; --paper: #fff; --ink: #000; --muted: #444; --rule: #bbb;
    --blk: #fff; --blk2: #FBF6EA; --plant: #EEF4F9; --st: #F1F5F9; --sig: #1D5A86; --logic: #8A5A00; --fb: #666; color-scheme: light; }
  body { padding: 0; }
  .sheet { border: 0; padding: 0; max-width: none; }
  .fig { break-before: page; } .fig:first-of-type { break-before: auto; }
  svg.dg { min-width: 0; }
  * { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
}
</style>
<svg width="0" height="0" style="position:absolute" aria-hidden="true">
  <defs>
    <marker id="ah" viewBox="0 0 10 10" refX="9.5" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path class="ah" d="M0,0 L10,5 L0,10 z"/></marker>
    <marker id="ahl" viewBox="0 0 10 10" refX="9.5" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path class="ahl" d="M0,0 L10,5 L0,10 z"/></marker>
  </defs>
</svg>
<main class="sheet">
<h1>PERSEVERANCE 블록선도</h1>
<p class="meta">LQR + VMC 균형 제어기, 코드 그대로 · 바퀴 140 mm 모델 · 제어 200 Hz · climb_test.py TUNE / tasks/residual.py 기준 · 2026-09-27</p>
<div class="legend">
  <span><svg viewBox="0 0 42 20"><path class="ln" d="M2,10 L38,10" marker-end="url(#ah)"/></svg>신호</span>
  <span><svg viewBox="0 0 42 20"><path class="lg" d="M2,10 L38,10" marker-end="url(#ahl)"/></svg>모드 전환 · 논리</span>
  <span><svg viewBox="0 0 42 20"><path class="bx" d="M6,2 L36,10 L6,18 Z"/></svg>게인</span>
  <span><svg viewBox="0 0 42 20"><circle class="bx" cx="21" cy="10" r="8"/><path class="thin" d="M15.4,4.4 L26.6,15.6 M15.4,15.6 L26.6,4.4"/></svg>합산점 (들어오는 쪽 부호)</span>
  <span><svg viewBox="0 0 42 20"><rect class="bxm" x="3" y="2" width="36" height="16" rx="2"/></svg>조건·상태 전환</span>
</div>
{{FIGS}}
<section class="fig" id="params"><h2>6. 파라미터 요약</h2>
<div class="tblwrap"><table><thead><tr><th>부분</th><th>값</th><th>코드</th></tr></thead><tbody>
{{ROWS}}
</tbody></table></div>
<p class="cap">줄 번호는 perseverance 저장소 기준 (sim/isaaclab/tasks/residual.py, sim/isaaclab/scripts/lqr_vmc.py). 창 모드 값은 scripts/climb_test.py 맨 위 TUNE 블록.</p>
</section>
</main>
"""

if __name__ == "__main__":
    OUT.write_text(page(), encoding="utf-8")
    print(OUT)
