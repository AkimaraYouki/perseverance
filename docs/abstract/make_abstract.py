"""확장요약문 (B5 1쪽) PDF — reportlab 로 직접 조판.

양식 (확장요약문.pdf): B5 182x257 mm, 본문 영역 위 21 mm / 아래 24 mm / 좌우 14.8 mm.
본문 바탕 9.5 pt, 장평 95 %, 자간 -5 %, 들여쓰기 9.4 pt, 줄간격 150 % (14.25 pt). 표 8 pt, 장평 93 %, 자간 -6 %, 행간 110 %.
캡션: Table 은 표 위, Fig. 는 그림 아래, 영문. 바탕 -> 나눔명조, Times New Roman -> Liberation Serif (폭 호환).
줄은 띄어쓰기에서만 끊는다 (어절 단위), 문단 마지막 줄 외에는 양쪽 정렬.
저자·소속은 아래 AUTHORS_*, AFFIL_* 를 고치고 다시 실행: python3 docs/abstract/make_abstract.py (reportlab, 나눔명조, Liberation Serif)
"""
import os
import re

from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

HERE = os.path.dirname(os.path.abspath(__file__))
NANUM = "/usr/share/fonts/truetype/nanum"
LIB = "/usr/share/fonts/truetype/liberation"
pdfmetrics.registerFont(TTFont("KR", f"{NANUM}/NanumMyeongjo.ttf"))
pdfmetrics.registerFont(TTFont("KRB", f"{NANUM}/NanumMyeongjoExtraBold.ttf"))
pdfmetrics.registerFont(TTFont("TNR", f"{LIB}/LiberationSerif-Regular.ttf"))
pdfmetrics.registerFont(TTFont("TNRB", f"{LIB}/LiberationSerif-Bold.ttf"))

# ---- 내용 ---------------------------------------------------------------------------------------
TITLE_KR = "LQR과 가상 모델 제어를 이용한 4절 링크 바퀴형 이족 로봇의 균형 및 주행 제어"
TITLE_EN = "Balance and Locomotion Control of a Wheeled Bipedal Robot with Four-Bar Legs Using LQR and Virtual Model Control"
AUTHORS_KR = [("발표자성명", "#"), ("교신저자성명", "†"), ("공동저자성명", "*")]
AUTHORS_EN = "Author name, Author name and Author name"
AFFIL_LEFT = [("†", "교신저자; 정회원, (교신저자 소속)"), ("", "E-mail : (교신저자 E-mail)"), ("#", "(발표자의 소속)")]
AFFIL_RIGHT = [("*", "(공동저자의 소속)")]
KEYWORDS = ("Key Words : Wheeled bipedal robot(바퀴형 이족 로봇), LQR(선형 2차 레귤레이터), Virtual model control(가상 모델 제어), "
            "Four-bar linkage(4절 링크), Domain randomization(도메인 무작위화)")
BODY = [
    "바퀴형 이족 로봇은 두 바퀴 위의 역진자로 균형을 잡으면서 다리 길이를 바꿔 요철과 턱에 적응한다. 본 연구는 4절 링크 다리를 "
    "갖는 로봇(4.17 kg, 고관절 AK60-6, 바퀴 AK45-10)에 바퀴 LQR과 다리 가상 모델 제어(VMC)를 결합한 200 Hz 제어기를 설계하고 "
    "시뮬레이션으로 강건성을 평가했다.",
    "바퀴는 [위치 오차, 속도 오차, 진자각, 각속도]에 대한 LQR로 제어하며, 이득은 진자 길이 15개 점에서 구해 보간한다"
    "(Q = diag(2, 5, 100, 5), R = 1). 진자각은 IMU 자세에 기구학으로 구한 무게중심 방향을 더해 추정한다. 속도는 모터 한계의 "
    "75 %로 제한하고, 회전 속도는 바깥 바퀴가 한계를 넘지 않도록 전진 지령에 따라 줄인다.",
    "다리는 관절 기준 가상 스프링·댐퍼(60 N·m/rad, 1.0 N·m·s/rad)와 자중 피드포워드로 만든 가상 힘을 4절 링크의 dh/dθ로 "
    "고관절 토크로 바꾼다. 링크는 dh/dθ가 행정 120 mm 전체에서 110~119 mm/rad로 거의 일정하게 설계해 높이와 관계없이 같은 "
    "이득을 쓴다. 좌우 다리 길이 차는 roll PI로 정해 몸체를 수평으로 유지하고, 점프는 다리를 접었다 펴는 상태머신과 공중에서 "
    "바퀴 반작용을 쓰는 자세 제어로 한다.",
    "제어기는 로봇 N대를 배열로 계산하는 하나의 코드로, 실기(N = 1)와 Isaac Lab 평가(최대 4096대)에 같이 쓴다. 실측 질량, "
    "바퀴 드라이브 데드밴드(0.5 A)와 그 보상, 로봇마다 질량 ±15 %, 무게중심 ±2 cm, 센서 잡음, 5 ms 지연을 넣었을 때 9개 "
    "험지에서 4096대 중 98.9 %가 20 s를 주행했고, 11개 시나리오 176회 중 167회 성공했다(Table 1).",
]
TABLE_TITLE = "Success in simulation with randomized robot models"
TABLE = [("Test", "Success"),
         ("Rough terrain, 9 types (4096 robots, 20 s)", "98.9 %"),
         ("Flat: stop, slalom, fast turn", "48/48"),
         ("Stones, one-side bumps (8 cm), stones with turns", "48/48"),
         ("30 cm ramp up and down", "16/16"),
         ("Staggered 8 cm ridges", "15/16"),
         ("Jump onto 8 cm platform", "15/16"),
         ("Placed by hand at 10° / 20° tilt", "15/16, 10/16")]
FIG_TITLE = "(a) Robot model (CAD) and (b) LQR-VMC control structure"

# ---- 조판 ---------------------------------------------------------------------------------------
PW, PH = 182 * mm, 257 * mm
L, R = 14.8 * mm, PW - 14.8 * mm
W = R - L
BODY_STYLE = dict(font="KR", size=9.5, scale=95, cs=-0.05 * 9.5)
LEAD = 14.25


class Page:
    def __init__(self, path):
        self.c = canvas.Canvas(path, pagesize=(PW, PH))
        self.c.setTitle(TITLE_KR)
        self.c.setAuthor(", ".join(n for n, _ in AUTHORS_KR))

    # 한글 글꼴의 넓은 하이픈·θ 는 Times 계열로 찍는다
    FALLBACK = set("-\u03b8")

    @staticmethod
    def runs(s, font):
        s = s.replace("\u00a0", " ")                               # 묶은 공백은 보통 공백으로 찍는다
        if font not in ("KR", "KRB"):
            return [(s, font)]
        out = []
        for ch in s:
            f = "TNR" if ch in Page.FALLBACK else font
            if out and out[-1][1] == f:
                out[-1] = (out[-1][0] + ch, f)
            else:
                out.append((ch, f))
        return out

    # 폭: PDF 는 Tc 에도 장평(Th)을 곱한다
    @staticmethod
    def width(s, font, size, scale=100, cs=0.0):
        return sum(scale / 100.0 * (pdfmetrics.stringWidth(r, f, size) + cs * len(r)) for r, f in Page.runs(s, font))

    def word(self, x, y, s, font, size, scale=100, cs=0.0):
        t = self.c.beginText(x, y)
        t.setHorizScale(scale)
        t.setCharSpace(cs)
        for r, f in self.runs(s, font):
            t.setFont(f, size)
            t.textOut(r)
        self.c.drawText(t)

    def wrap(self, text, width, font, size, scale=100, cs=0.0, indent=0.0):
        # 숫자와 단위 (200 Hz, 15 %) 는 한 줄에: 묶은 공백 (U+00A0)
        text = re.sub(r"(\d) (kg|Hz|A|%|cm|ms|s|mm|N\u00b7m)(?=[\s,)/.\u00b7가-힣]|$)", "\\1\u00a0\\2", text)
        words, lines, cur = text.split(" "), [], []
        for w in words:
            trial = cur + [w]
            avail = width - (indent if not lines else 0.0)
            if cur and self.width(" ".join(trial), font, size, scale, cs) > avail:
                lines.append(cur)
                cur = [w]
            else:
                cur = trial
        if cur:
            lines.append(cur)
        return lines

    def para(self, y, text, x0, width, font, size, scale=100, cs=0.0, lead=None, indent=0.0, align="justify"):
        """y = 첫 줄 기준선. 반환: 다음 줄 기준선."""
        lead = lead or size * 1.2
        lines = self.wrap(text, width, font, size, scale, cs, indent)
        sp = self.width(" ", font, size, scale, cs)
        for i, ws in enumerate(lines):
            xi = x0 + (indent if i == 0 else 0.0)
            avail = width - (indent if i == 0 else 0.0)
            wsz = [self.width(w, font, size, scale, cs) for w in ws]
            nat = sum(wsz) + sp * (len(ws) - 1)
            if align == "center":
                x, gap = xi + (avail - nat) / 2, sp
            elif align == "justify" and i < len(lines) - 1 and len(ws) > 1:
                x, gap = xi, (avail - sum(wsz)) / (len(ws) - 1)
            else:
                x, gap = xi, sp
            for w, ww in zip(ws, wsz):
                self.word(x, y, w, font, size, scale, cs)
                x += ww + gap
            y -= lead
        return y


def arrow(c, x0, y0, x1, y1, head=1.3 * mm, lw=0.6):
    import math
    c.setLineWidth(lw)
    ang = math.atan2(y1 - y0, x1 - x0)
    xe, ye = x1 - head * 0.8 * math.cos(ang), y1 - head * 0.8 * math.sin(ang)
    c.line(x0, y0, xe, ye)
    p = c.beginPath()
    p.moveTo(x1, y1)
    for s in (+1, -1):
        p.lineTo(x1 - head * math.cos(ang) + s * head * 0.45 * math.sin(ang), y1 - head * math.sin(ang) - s * head * 0.45 * math.cos(ang))
    p.close()
    c.drawPath(p, fill=1, stroke=0)


def build(path):
    pg = Page(path)
    c = pg.c
    top = PH - 21 * mm                                             # 본문 영역 위 (21 mm)
    # 제목
    y = top - 15.5
    y = pg.para(y, TITLE_KR, L, W, "KRB", 16, scale=95, lead=20, align="center")
    y -= 2.0 * mm - 20 + 13
    y = pg.para(y, TITLE_EN, L, W, "TNR", 13, lead=15.5, align="center")
    # 저자 (위첨자 표식)
    y -= 4.5 * mm - 15.5 + 12
    parts, sup = [], []
    for i, (n, m) in enumerate(AUTHORS_KR):
        parts.append((n, "KR", 12, 95))
        parts.append((m, "KR", 7.5, 95))
        if i < len(AUTHORS_KR) - 1:
            parts.append((" · ", "KR", 12, 95))
    tot = sum(pg.width(s, f, z, sc) for s, f, z, sc in parts)
    x = L + (W - tot) / 2
    for s, f, z, sc in parts:
        pg.word(x, y + (4.2 if z < 10 else 0), s, f, z, sc)
        x += pg.width(s, f, z, sc)
    y -= 15
    y = pg.para(y, AUTHORS_EN, L, W, "TNR", 12, lead=15, align="center")
    # Key Words + 본문
    y -= 3.0 * mm - 15 + 9.5
    y = pg.para(y, KEYWORDS, L, W, lead=LEAD, align="justify", **BODY_STYLE)
    y -= LEAD                                                      # 빈 줄
    for t in BODY:
        y = pg.para(y, t, L, W, lead=LEAD, indent=9.4, align="justify", **BODY_STYLE)
    body_end = y + LEAD
    y -= LEAD * 0.6                                                # 그림·표 위 빈 줄 (기준선 -> 다음 블록 위)

    # ---- Table 1 (왼쪽 74 mm) -------------------------------------------------------------------
    colL, colW = L, 74 * mm
    yt = y
    cap = [("Table 1", "TNRB"), ("  " + TABLE_TITLE, "TNR")]
    yt = caption(pg, yt, cap, colL, colW, 9)
    yt -= 0.6 * mm
    tw = [51 * mm, 19 * mm]
    tx0 = colL + (colW - sum(tw)) / 2
    tf = dict(font="TNR", size=8, scale=93, cs=-0.06 * 8)
    pad, lead8 = 0.9 * mm, 8.8
    c.setStrokeColorRGB(0, 0, 0)
    yrow = yt
    c.setLineWidth(0.5)
    c.line(tx0, yrow, tx0 + sum(tw), yrow)                         # 위 이중선
    c.line(tx0, yrow - 1.2, tx0 + sum(tw), yrow - 1.2)
    yrow -= 1.2
    for i, row in enumerate(TABLE):
        cells = [pg.wrap(row[0], tw[0] - 2 * pad, **tf), [[row[1]]]]
        nl = max(len(cl) for cl in cells)
        h = nl * lead8 + 2 * pad
        if i == 0:
            c.setFillGray(0.86)
            c.rect(tx0, yrow - h, sum(tw), h, stroke=0, fill=1)
            c.setFillGray(0)
        base = yrow - pad - 6.2
        for j, cl in enumerate(cells):
            xj = tx0 + sum(tw[:j])
            for k, ln in enumerate(cl):
                s = " ".join(ln)
                if j == 0:
                    pg.word(xj + pad, base - k * lead8, s, **tf)
                else:
                    pg.word(xj + (tw[j] - pg.width(s, **tf)) / 2, base - k * lead8, s, **tf)
        yrow -= h
        c.setLineWidth(0.5)
        if i < len(TABLE) - 1:
            c.line(tx0, yrow, tx0 + sum(tw), yrow)
    c.line(tx0, yrow, tx0 + sum(tw), yrow)                         # 아래 이중선
    c.line(tx0, yrow - 1.2, tx0 + sum(tw), yrow - 1.2)
    c.line(tx0 + tw[0], yt - 1.2, tx0 + tw[0], yrow)               # 가운데 세로선
    table_bottom = yrow - 1.2

    # ---- Fig. 1 (오른쪽) ------------------------------------------------------------------------
    fx0 = L + 77 * mm
    fw = R - fx0
    ftop = y - 0.3 * mm
    img_w, img_h = 25.6 * mm, 32 * mm
    c.drawImage(os.path.join(HERE, "..", "media", "robot_cad_front.png"), fx0 + 0.5 * mm, ftop - img_h, img_w, img_h, mask="auto")
    lab = dict(font="TNR", size=7.5)
    lab_y = ftop - img_h - 3.6 * mm
    pg.word(fx0 + 0.5 * mm + img_w / 2 - pg.width("(a)", **lab) / 2, lab_y, "(a)", **lab)
    bx, bw, bh, gap = fx0 + 30 * mm, 28 * mm, 6.4 * mm, 2.2 * mm
    boxes = [("IMU, encoders", "→ state θ, dθ/dt, v", 1.0),
             ("speed / turn limits", "v*, ω*", 1.0),
             ("wheel LQR  K(l)", "+ yaw damping", 0.9),
             ("leg VMC + roll PI", "τ = F·dh/dθ", 0.9)]
    bt = dict(font="TNR", size=6.6)
    for i, (a, b, g) in enumerate(boxes):
        yb = ftop - 0.3 * mm - i * (bh + gap)                      # 상자 위
        c.setFillGray(g)
        c.setLineWidth(0.6)
        c.roundRect(bx, yb - bh, bw, bh, 0.7 * mm, stroke=1, fill=1)
        c.setFillGray(0)
        for k, s in enumerate((a, b)):
            pg.word(bx + bw / 2 - pg.width(s, **bt) / 2, yb - 2.65 * mm - k * 2.6 * mm, s, **bt)
        if i:
            arrow(c, bx + bw / 2, yb + gap - 0.05 * mm, bx + bw / 2, yb + 0.05 * mm)
        if i >= 2:
            arrow(c, bx + bw, yb - bh / 2, bx + bw + 3.4 * mm, yb - bh / 2)
            for k, s in enumerate(("wheel", "torque") if i == 2 else ("hip", "torque")):
                pg.word(bx + bw + 3.9 * mm, yb - 2.65 * mm - k * 2.6 * mm, s, **bt)
    pg.word(bx + bw / 2 - pg.width("(b)", **lab) / 2, lab_y, "(b)", **lab)
    fy = lab_y - 4.2 * mm
    fig_bottom = caption(pg, fy, [("Fig. 1", "TNRB"), ("  " + FIG_TITLE, "TNR")], fx0, fw, 9)

    # ---- 저자 소속 (아래) -----------------------------------------------------------------------
    yr = 24 * mm + 3 * 10.5 + 2.5 * mm                              # 본문 영역 아래 24 mm 위로 세 줄
    c.setLineWidth(0.5)
    c.line(L, yr, L + 70 * mm, yr)
    ft = dict(font="KR", size=8, scale=95)
    for col_x, items in ((L, AFFIL_LEFT), (L + 80 * mm, AFFIL_RIGHT)):
        yy = yr - 2.6 * mm - 4
        for mark, s in items:
            pg.word(col_x + 0.5 * mm, yy, mark, **ft)
            pg.word(col_x + 4.5 * mm, yy, s, **ft)
            yy -= 10.5
    c.showPage()
    c.save()
    return dict(body_end_mm=(PH - body_end) / mm, table_bottom_mm=(PH - table_bottom) / mm,
                fig_bottom_mm=(PH - fig_bottom) / mm, affil_rule_mm=(PH - yr) / mm)


def caption(pg, y, runs, x0, width, size):
    """굵은 'Table 1' + 제목, 가운데, 넘치면 줄바꿈. 반환: 마지막 줄 아래."""
    words = []
    for s, f in runs:
        if f == "TNRB":
            words.append((s.strip(), f))                            # "Table 1" 은 한 덩어리
        else:
            words.extend((w, f) for w in s.strip().split(" "))
    lines, cur = [], []
    sp = pg.width(" ", "TNR", size)
    for w in words:
        trial = cur + [w]
        if cur and sum(pg.width(a, f, size) for a, f in trial) + sp * len(trial) > width:
            lines.append(cur)
            cur = [w]
        else:
            cur = trial
    lines.append(cur)
    yb = y - size
    for ln in lines:
        tot = sum(pg.width(a, f, size) for a, f in ln) + sp * (len(ln) - 1) + (sp if len(ln) > 1 else 0)
        x = x0 + (width - tot) / 2
        for k, (a, f) in enumerate(ln):
            pg.word(x, yb, a, f, size)
            x += pg.width(a, f, size) + sp
            if k == 0 and ln is lines[0]:
                x += sp                                             # "Table 1" 뒤 두 칸
        yb -= size * 1.2
    return yb + size * 1.2 - 2.5


if __name__ == "__main__":
    out = os.path.join(HERE, "extended_abstract_lqr_vmc.pdf")
    print(build(out), out)
