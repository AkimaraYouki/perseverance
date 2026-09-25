"""Pure rendering of the 320x240 status screen (no ROS, no hardware) — testable offline.

Layout (landscape):   header: C-WANG by suhopark [robot state]   HH:MM
  +-----------------+-----------------+
  | 1 PC            | 2 SENSORS       |
  +-----------------+-----------------+
  | 3 MOTORS        | 4 NET & POWER   |
  +-----------------+-----------------+
"""
import math
import time

from PIL import Image, ImageDraw, ImageFont

W, H = 320, 240
# Black & white theme. Levels are shown by SHAPE, not colour:
#   OK = filled dot, WARN = hollow dot, BAD = inverted (white box, black text).
BG = (0, 0, 0)
PANEL = (0, 0, 0)
FG = (255, 255, 255)
DIM = (150, 150, 150)
OK = (255, 255, 255)
WARN = (200, 200, 200)
BAD = (255, 255, 254)       # distinct tuple so helpers can invert it
ACCENT = (255, 255, 253)
BAR_BG = (60, 60, 60)
TITLE = 'C-WANG'
BYLINE = 'by suhopark'

# Blink phases for the current render: errors blink fast (2 Hz), warnings slow (1 Hz).
_BLINK = {'fast': True, 'slow': True}
# Which blink phases the last render actually depended on (lets the node skip redraws).
USED = {'fast': False, 'slow': False}

_F = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
_FB = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
_MB = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf'
_M = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'


class _Draw(ImageDraw.ImageDraw):
    """ImageDraw whose text() pastes cached glyph masks. Most labels are identical between
    frames, so this removes most of the per-frame FreeType work."""
    _cache = {}

    def text(self, xy, text, fill=None, font=None, **kw):
        if kw or not text:
            return super().text(xy, text, fill=fill, font=font, **kw)
        key = (text, id(font))
        m = self._cache.get(key)
        if m is None:
            l, t, r, b = font.getbbox(text)
            mask = Image.new('L', (max(1, r - l), max(1, b - t)))
            ImageDraw.Draw(mask).text((-l, -t), text, font=font, fill=255)
            m = (mask, l, t)
            if len(self._cache) > 4000:
                self._cache.clear()
            self._cache[key] = m
        mask, l, t = m
        self.im.paste(fill if isinstance(fill, tuple) else (255, 255, 255),
                      (int(xy[0]) + l, int(xy[1]) + t,
                       int(xy[0]) + l + mask.width, int(xy[1]) + t + mask.height), mask.im)


def _font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


F_T = _font(_FB, 11)      # panel titles
F_S = _font(_M, 10)       # rows
F_SB = _font(_MB, 10)
F_BIG = _font(_MB, 15)
F_H = _font(_FB, 12)

ROW = 13


def _ok(v):
    return v is not None and not (isinstance(v, float) and not math.isfinite(v))


def _fmt(v, spec, missing='--'):
    return format(v, spec) if _ok(v) else missing


def _heat(t, warn=70.0, bad=85.0):
    return DIM if not _ok(t) else BAD if t >= bad else WARN if t >= warn else OK


def _panel(d, box, title, right=None, rcol=DIM):
    x0, y0, x1, y1 = box
    d.rounded_rectangle(box, radius=4, outline=FG, width=1)
    _txt(d, (x0 + 6, y0 + 3), title, F_T, FG)
    d.line([x0 + 1, y0 + 17, x1 - 1, y0 + 17], fill=DIM)
    if right:
        _txt(d, (x1 - 6 - d.textlength(right, font=F_S), y0 + 4), right, F_S, rcol)
    return x0 + 6, y0 + 21


def _bar(d, x, y, w, frac, col):
    d.rectangle([x, y, x + w, y + 5], fill=BAR_BG)
    if _ok(frac):
        d.rectangle([x, y, x + int(w * min(1.0, max(0.0, frac))), y + 5], fill=FG)


def _runtime(minutes):
    if minutes is None:
        return '--'
    if isinstance(minutes, float) and math.isinf(minutes):
        return 'idle'
    if minutes < 0:
        return '--'
    h, m = divmod(int(minutes), 60)
    return f"{h}h{m:02d}" if h else f"{m}m"


def _dot(d, x, y, col):
    if col == BAD:
        USED['fast'] = True
    elif col == WARN:
        USED['slow'] = True
    if col == BAD and not _BLINK['fast']:
        d.rectangle([x, y + 2, x + 7, y + 9], outline=DIM)
        return
    if col == WARN and not _BLINK['slow']:
        return
    if col == BAD:
        d.rectangle([x, y + 2, x + 7, y + 9], fill=FG)
        d.line([x + 1, y + 3, x + 6, y + 8], fill=BG, width=2)
        d.line([x + 1, y + 8, x + 6, y + 3], fill=BG, width=2)
    elif col == OK:
        d.ellipse([x, y + 2, x + 7, y + 9], fill=FG)
    elif col == WARN:
        d.ellipse([x, y + 2, x + 7, y + 9], outline=FG, width=1)
    else:
        d.ellipse([x + 2, y + 4, x + 5, y + 7], fill=DIM)


def _txt(d, xy, text, font, col):
    """Text; BAD is drawn inverted so it stands out without colour."""
    if col == BAD and text:
        USED['fast'] = True
    elif col == WARN and text:
        USED['slow'] = True
    if col == BAD and text and _BLINK['fast']:
        x, y = xy
        w = d.textlength(text, font=font)
        d.rectangle([x - 1, y, x + w + 1, y + font.size + 1], fill=FG)
        d.text((x, y), text, font=font, fill=BG)
    else:
        dim = col == DIM or (col == WARN and not _BLINK['slow'])  # warnings pulse slowly
        d.text(xy, text, font=font, fill=DIM if dim else FG)


# ------------------------------------------------------------------ panels
def _pc(d, box, sy):
    x, y = _panel(d, box, '1 PC', sy.get('power_mode') and f"{sy['power_mode']}")
    w = box[2] - x - 6
    for label, pct, temp in (('CPU', sy.get('cpu'), sy.get('t_cpu')),
                             ('GPU', sy.get('gpu'), sy.get('t_gpu'))):
        col = _heat(temp)
        _txt(d, (x, y), f"{label} {_fmt(pct, '3.0f')}%", F_SB, FG)
        t = f"{_fmt(temp, '.1f')}°C"
        _txt(d, (x + w - d.textlength(t, font=F_SB), y), t, F_SB, col)
        _bar(d, x, y + 12, w, (pct or 0) / 100.0, ACCENT)
        y += 19
    ru, rt = sy.get('ram_used'), sy.get('ram_total')
    _txt(d, (x, y), f"RAM {_fmt(ru, '.1f')}/{_fmt(rt, '.1f')}G", F_SB, FG)
    _bar(d, x, y + 12, w, (ru / rt) if _ok(ru) and rt else None, ACCENT)
    y += 19
    _txt(d, (x, y), f"TJ {_fmt(sy.get('t_tj'), '.0f')}°C", F_S, _heat(sy.get('t_tj')))
    fan = f"FAN {_fmt(sy.get('fan_rpm'), '.0f')}"
    _txt(d, (x + w - d.textlength(fan, font=F_S), y), fan, F_S, DIM)
    y += ROW
    _txt(d, (x, y), f"Jetson {_fmt(sy.get('jetson_w'), '.1f')} W", F_S, FG)


def _sensors(d, box, s):
    x, y = _panel(d, box, '2 SENSORS')
    w = box[2] - x - 6
    rows = []
    hub = s.get('hub', {})
    if hub.get('fresh'):
        col = OK if (hub.get('rate') or 0) > 80 and not hub.get('crc') else WARN
        rows.append(('HUB', col, f"{_fmt(hub.get('rate'), '.0f')}Hz crc{hub.get('crc', 0)}"))
    else:
        rows.append(('HUB', BAD, 'no data'))
    g = s.get('gps', {})
    if not hub.get('fresh'):
        rows.append(('GPS', DIM, '--'))
    elif not g.get('seen'):
        # not an alarm unless the GPS is supposed to be connected (gps_expected)
        rows.append(('GPS', WARN, 'no NAV-PVT') if s.get('gps_expected') else
                    ('GPS', DIM, 'not connected'))
    elif g.get('fix', 0) < 2:
        rows.append(('GPS', DIM, f"no fix sv{g.get('sv', 0)}"))  # normal indoors: no blink
    else:
        rows.append(('GPS', OK if g.get('usable') else WARN,
                     f"{g.get('fix')}D sv{g.get('sv')} {_fmt(g.get('hacc'), '.1f')}m"))
    mag = s.get('mag', {})
    rows.append(('MAG', OK if mag.get('ok') else (DIM if not hub.get('fresh') else WARN),
                 f"{_fmt(mag.get('rate'), '.0f')}Hz" if mag.get('ok') else '--'))
    for key, label in (('imu', 'IMU'), ('lidar', 'LIDAR'), ('camera', 'CAM')):
        st = s.get(key, {})
        rows.append((label, st.get('col', DIM), st.get('text', 'n/a')))
    for label, col, txt in rows:
        _dot(d, x, y, col)
        _txt(d, (x + 11, y), label, F_SB, FG)
        _txt(d, (x + w - d.textlength(txt, font=F_S), y), txt, F_S, col if col != OK else FG)
        y += ROW


def _motors(d, box, s):
    """One line per motor: configured motors, then drives found on the bus but not in
    motors.yaml ("id_70?", warning), then dim placeholders up to motors_expected (default 4)."""
    can = s.get('can_rate')
    x, y = _panel(d, box, '3 MOTORS', f"CAN {_fmt(can, '.0f')}/s", OK if can else BAD)
    w = box[2] - x - 6
    motors = list(s.get('motors', []))
    expected = int(s.get('motors_expected', 4))
    # right edges of the value columns
    e_temp = x + w
    e_cur = e_temp - d.textlength('999', font=F_S) - 5
    e_pos = e_cur - d.textlength('-99.9', font=F_S) - 5
    name_w = e_pos - d.textlength('-180', font=F_S) - 3 - (x + 11)
    for label, edge in (('deg', e_pos), ('A', e_cur), ('C', e_temp)):
        d.text((edge - d.textlength(label, font=F_S), y - 1), label, font=F_S, fill=DIM)
    y += ROW - 1
    rows = motors[:expected + 2]
    for m in rows:
        unconf = m.get('configured') is False
        fresh = m.get('fresh') and not m.get('stale')
        if not fresh:
            col = BAD
        elif m.get('error_code'):
            col = BAD
        elif unconf:
            col = WARN
        else:
            col = OK
        _dot(d, x, y, col)
        name = m.get('name', '?')
        if unconf and name.startswith('id_'):
            name = '#' + name[3:]          # "id_70?" -> "#70?"
        while name and d.textlength(name, font=F_SB) > name_w:
            name = name[:-1]
        _txt(d, (x + 11, y), name, F_SB, FG if not unconf else WARN)
        if not fresh:
            msg = 'STALE'
            _txt(d, (e_temp - d.textlength(msg, font=F_S), y), msg, F_S, BAD)
        elif m.get('error_code'):
            msg = (m.get('error_text') or 'fault')[:14]
            _txt(d, (e_temp - d.textlength(msg, font=F_S), y), msg, F_S, BAD)
        else:
            pos = m.get('pos_deg')
            if _ok(pos):                   # display only: wrap to [-180, 180) (wheels grow without bound)
                pos = (pos + 180.0) % 360.0 - 180.0
            vals = ((e_pos, _fmt(pos, '.0f')), (e_cur, _fmt(m.get('current'), '.1f')),
                    (e_temp, _fmt(m.get('temp'), '.0f')))
            for edge, v in vals:
                c = _heat(m.get('temp'), 65, 80) if edge == e_temp else FG
                _txt(d, (edge - d.textlength(v, font=F_S), y), v, F_S, c)
        y += ROW
    for _ in range(max(0, expected - len(rows))):
        _dot(d, x, y, None)
        d.text((x + 11, y), '-', font=F_S, fill=DIM)
        d.text((e_temp - d.textlength('waiting', font=F_S), y), 'waiting', font=F_S, fill=DIM)
        y += ROW
    if len(motors) > len(rows):
        d.text((x + 11, y), f'+{len(motors) - len(rows)} more', font=F_S, fill=DIM)


def _netpower(d, box, s):
    sy = s.get('sys', {})
    x, y = _panel(d, box, '4 NET & POWER')
    w = box[2] - x - 6
    if sy.get('wifi_up'):
        dbm = sy.get('wifi_dbm')
        col = OK if _ok(dbm) and dbm > -67 else WARN
        ssid = (sy.get('ssid') or '?')[:12]
        _txt(d, (x, y), f"WiFi {ssid}", F_S, FG)
        t = f"{_fmt(dbm, '.0f')}dBm"
        _txt(d, (x + w - d.textlength(t, font=F_S), y), t, F_S, col)
        y += ROW
        _txt(d, (x, y), f"IP {sy.get('wifi_ip') or '--'}", F_S, DIM)
    else:
        _txt(d, (x, y), 'WiFi DOWN', F_SB, BAD)
        y += ROW
        _txt(d, (x, y), f"ETH {sy.get('eth_ip') or '--'}" if sy.get('eth_up') else '', F_S, DIM)
    y += ROW
    ssh = sy.get('ssh', 0)
    _txt(d, (x, y), f"SSH {ssh}", F_S, ACCENT if ssh else DIM)
    eth = 'ETH up' if sy.get('eth_up') else 'ETH --'
    _txt(d, (x + w - d.textlength(eth, font=F_S), y), eth, F_S, OK if sy.get('eth_up') else DIM)
    y += ROW + 1
    d.line([x, y - 1, x + w, y - 1], fill=BAR_BG)

    def batt(label, b):
        nonlocal y
        if not b.get('fresh'):
            txt, col = 'no data', BAD
        elif not b.get('present'):
            txt, col = 'not connected', DIM
        else:
            cv = b.get('cell_v')
            col = BAD if cv < b.get('err', 3.3) else WARN if cv < b.get('warn', 3.5) else OK
            est = '~' if b.get('estimated') else ''
            txt = (f"{_fmt(b.get('voltage'), '4.1f')}V {est}{_fmt(b.get('power'), '.0f')}W "
                   f"{_fmt(b.get('soc'), '3.0f')}% {_runtime(b.get('runtime_min'))}")
        _txt(d, (x, y), label, F_SB, FG)
        _txt(d, (x + w - d.textlength(txt, font=F_S), y), txt, F_S, col)
        y += ROW

    comp, mot = s.get('compute', {}), s.get('motor', {})
    batt(f"C{comp.get('cells', 3)}S", comp)
    batt(f"M{mot.get('cells', 6)}S", mot)
    total = sum(b.get('power') or 0.0 for b in (comp, mot) if b.get('fresh') and b.get('present'))
    wh = sum(b.get('wh') or 0.0 for b in (comp, mot) if b.get('fresh'))
    any_batt = any(b.get('fresh') and b.get('present') for b in (comp, mot))
    _txt(d, (x, y + 1), 'TOTAL', F_SB, FG)
    t = f"{total:.0f}W {wh:.2f}Wh" if any_batt else '--'
    d.text((x + w - d.textlength(t, font=F_BIG), y - 2), t, font=F_BIG,
           fill=ACCENT if any_batt else DIM)


def render(s, fast=True, slow=True):
    """s: status dict built by the node; fast/slow: blink phases. Returns a 320x240 image."""
    _BLINK['fast'], _BLINK['slow'] = fast, slow
    USED['fast'], USED['slow'] = False, True  # the clock colon always ticks with 'slow'
    img = Image.new('RGB', (W, H), BG)
    d = _Draw(img)
    state = s.get('robot_state') or 'NO CTRL'
    scol = {'ACTIVE': OK, 'ARMED': WARN, 'READY': ACCENT, 'DISARMED': DIM}.get(
        state, BAD if ('FAULT' in state or 'SAFE_STOP' in state) else DIM)
    d.text((5, 3), TITLE, font=F_H, fill=FG)
    x = 5 + d.textlength(TITLE, font=F_H) + 4
    d.text((x, 5), BYLINE, font=F_S, fill=DIM)
    x += d.textlength(BYLINE, font=F_S) + 6
    tw = d.textlength(state, font=F_S)
    live = state in ('ARMED', 'ACTIVE')           # motors can move: badge blinks
    if scol == BAD:
        USED['fast'] = True
    if (scol == BAD and fast) or (live and slow):
        d.rectangle([x, 3, x + tw + 8, 17], fill=FG)
        d.text((x + 4, 5), state, font=F_S, fill=BG)
    else:
        d.rectangle([x, 3, x + tw + 8, 17], outline=FG)
        d.text((x + 4, 5), state, font=F_S, fill=FG)
    clock = time.strftime('%H:%M', time.localtime(s.get('wall_time', time.time())))
    if not slow:
        clock = clock.replace(':', ' ')  # colon ticks once a second: the display is alive
    _txt(d, (W - 6 - d.textlength(clock, font=F_S), 5), clock, F_S, DIM)

    L, M, R = 2, 160, 318
    T, C, B = 21, 128, 238
    _pc(d, (L, T, M - 2, C - 2), s.get('sys', {}))
    _sensors(d, (M + 2, T, R, C - 2), s)
    _motors(d, (L, C + 2, M - 2, B), s)
    _netpower(d, (M + 2, C + 2, R, B), s)
    return img
