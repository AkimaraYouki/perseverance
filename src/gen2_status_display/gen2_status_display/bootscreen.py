"""Boot status screen on the ST7789 (gen2-bootscreen.service, root, right after lcd-pinmux).

Shows where a boot is stuck before ROS is up:
  first 3 s: the PREVIOUS boot (how long it ran, clean or abrupt end, last systemd line, last error)
  then 1 Hz: time since power-on, systemd state, the jobs systemd is still waiting on, failed units,
  the last error in the journal, Wi-Fi/IP, IMU / LiDAR / hub / can0 / ttyTHS1 / camera, gen2 services.
It hands the panel to the status_display node when that node asks (lcd_handover), so a boot that
never reaches gen2-bench keeps this screen. No screen at all = stuck in the bootloader / kernel.

Run: python3 -m gen2_status_display.bootscreen [--png out.png]   (--png renders offline, no LCD)
"""
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

from PIL import Image, ImageChops

from . import lcd_handover
from .render import W, H, BG, FG, DIM, _Draw, _font, _FB, _M, _MB

F = _font(_M, 10)
FB = _font(_MB, 10)
F_HEAD = _font(_FB, 12)
F_BIG = _font(_MB, 15)
ROW = 13
PREV_S = 3.0
WIFI = 'wlP1p1s0'
SERVICES = [('can0-up', 'can0'), ('lcd-pinmux', 'lcd'), ('NetworkManager', 'net'),
            ('gen2-bench', 'bench'), ('gen2-balance', 'ctrl')]
DEVICES = [('IMU', '/dev/gen2_imu'), ('LIDAR', '/dev/gen2_lidar'), ('HUB', '/dev/gen2_hub'),
           ('THS1', '/dev/ttyTHS1'), ('CAM', '/dev/video0')]
# journal lines that are errors by priority but say nothing about a stuck boot
NOISE = ('ddsi_udp_conn_write', 'not multicast-capable', 'scan_gate', 'gnome-', 'pulseaudio',
         'bluetoothd', 'gdm', 'wpa_supplicant', 'kerneloops')
SHUTDOWN = re.compile(r'systemd-shutdown|Journal stopped|Reached target.*(Power-Off|Reboot|Shutdown)|Shutting down')  # = gen2_bootlog.sh


# ---------------------------------------------------------------- probes (never raise)
def _run(cmd, timeout=2.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ''


def _last_error(boot):
    out = _run(['journalctl', '-b', str(boot), '-p', 'err', '-n', '30', '-o', 'json', '--no-pager',
                '--output-fields=MESSAGE,SYSLOG_IDENTIFIER,_COMM,__MONOTONIC_TIMESTAMP'], 3.0)
    for line in reversed(out.splitlines()):
        try:
            e = json.loads(line)
        except ValueError:
            continue
        msg = e.get('MESSAGE')
        if not isinstance(msg, str):
            continue
        who = e.get('SYSLOG_IDENTIFIER') or e.get('_COMM') or 'kernel'
        if any(n in msg or n in who for n in NOISE):
            continue
        t = int(e.get('__MONOTONIC_TIMESTAMP', 0)) / 1e6
        return f'[{t:.0f}s] {who}: {msg}'
    return ''


def previous_boot():
    """Summary of the previous boot, or None (first boot / no persistent journal)."""
    # `journalctl -b -1` fails here ("no boot entry"): without an RTC every boot starts in 1970 or
    # 2026-06-06, so journald cannot order boots by time. Use the boot ID from --list-boots instead.
    boots = _run(['journalctl', '--list-boots', '--no-pager'], 4.0).splitlines()
    prev_id = next((ln.split()[1] for ln in boots if ln.split()[:1] == ['-1']), None)
    if prev_id is None:
        return None
    last = _run(['journalctl', '-b', prev_id, '-n', '1', '-o', 'json', '--no-pager',
                 '--output-fields=__MONOTONIC_TIMESTAMP'], 4.0).strip()
    try:
        dur = int(json.loads(last.splitlines()[-1]).get('__MONOTONIC_TIMESTAMP', 0)) / 1e6
    except (ValueError, IndexError):
        return None
    tail = _run(['journalctl', '-b', prev_id, '-n', '300', '-o', 'cat', '--no-pager'], 4.0)
    unit = _run(['journalctl', '-b', prev_id, '_PID=1', '-n', '1', '-o', 'cat', '--no-pager'], 4.0).strip()
    return {'duration_s': dur, 'clean': SHUTDOWN.search(tail) is not None, 'last_systemd': unit,
            'last_error': _last_error(prev_id), 'boots': len(boots) - 1}


def gather():
    s = dict(uptime_s=float((_read('/proc/uptime') or '0').split()[0]),
             state=_run(['systemctl', 'is-system-running']).strip() or '?')
    jobs = []
    for line in _run(['systemctl', 'list-jobs', '--no-legend', '--no-pager', '--plain']).splitlines():
        p = line.split()
        if len(p) >= 4:
            jobs.append((p[3], p[1]))  # (state, unit); running first
    jobs.sort(key=lambda j: j[0] != 'running')
    s['jobs'] = jobs
    s['failed'] = [line.split()[0] for line in
                   _run(['systemctl', '--failed', '--no-legend', '--no-pager', '--plain']).splitlines() if line.split()]
    s['error'] = _last_error(0)
    link = _run(['iw', 'dev', WIFI, 'link'], 1.0)
    ssid = next((ln.split(':', 1)[1].strip() for ln in link.splitlines() if 'SSID:' in ln), '')
    ip = ''
    for ln in _run(['ip', '-4', '-o', 'addr', 'show', 'dev', WIFI], 1.0).splitlines():
        ip = ln.split()[3].split('/')[0]
    s['wifi'] = (ssid, ip)
    s['dev'] = [(name, os.path.exists(path)) for name, path in DEVICES]
    can = _run(['ip', '-d', 'link', 'show', 'can0'], 1.0)
    st = ''
    if 'can state' in can:
        st = can.split('can state', 1)[1].split()[0]
    s['can'] = ('UP' if 'state UP' in can else ('down' if can else 'none'), st)
    svc = []
    out = _run(['systemctl', 'show', '--property=ActiveState,NRestarts', '--no-pager']
               + [u + '.service' for u, _ in SERVICES])
    blocks = [b for b in out.strip().split('\n\n')] if out.strip() else []
    for (u, short), b in zip(SERVICES, blocks + [''] * len(SERVICES)):
        kv = dict(ln.split('=', 1) for ln in b.splitlines() if '=' in ln)
        svc.append((short, kv.get('ActiveState', '?'), int(kv.get('NRestarts', '0') or 0)))
    s['svc'] = svc
    return s


# ---------------------------------------------------------------- pure rendering
def _clip(d, text, font, width):
    if d.textlength(text, font=font) <= width:
        return text
    while text and d.textlength(text + '~', font=font) > width:
        text = text[:-1]
    return text + '~'


def _wrap(d, text, font, width, lines):
    out = []
    while text and len(out) < lines:
        n = len(text)
        while n > 1 and d.textlength(text[:n], font=font) > width:
            n -= 1
        out.append(text[:n])
        text = text[n:].lstrip()
    if text and out:
        out[-1] = _clip(d, out[-1] + text, font, width)
    return out


def _dur(s):
    s = int(s)
    if s < 60:
        return f'{s}s'
    if s < 3600:
        return f'{s // 60}m{s % 60:02d}s'
    return f'{s // 3600}h{s % 3600 // 60:02d}m'


def _header(d, left, right):
    d.rectangle([0, 0, W - 1, 17], fill=FG)
    d.text((4, 2), left, font=F_HEAD, fill=BG)
    d.text((W - 4 - d.textlength(right, font=F_HEAD), 2), right, font=F_HEAD, fill=BG)


def _label(d, y, text):
    d.text((4, y), text, font=FB, fill=DIM)


def _dot(d, x, y, ok, bad=False):
    """OK = filled dot, missing = hollow dot, BAD = filled box (same shapes as the dashboard)."""
    if bad:
        d.rectangle([x, y + 2, x + 7, y + 9], fill=FG)
    elif ok:
        d.ellipse([x, y + 2, x + 7, y + 9], fill=FG)
    else:
        d.ellipse([x, y + 2, x + 7, y + 9], outline=DIM)


def _inverted(d, x, y, text, font=FB):
    w = d.textlength(text, font=font)
    d.rectangle([x - 2, y, x + w + 1, y + ROW - 1], fill=FG)
    d.text((x, y), text, font=font, fill=BG)


def render_previous(p):
    img = Image.new('RGB', (W, H), BG)
    d = _Draw(img)
    _header(d, 'C-WANG  previous boot', f'boot #{p["boots"] - 1}')
    y = 30
    _label(d, y, 'RAN FOR')
    d.text((80, y - 3), _dur(p['duration_s']), font=F_BIG, fill=FG)
    y += 24
    _label(d, y, 'ENDED')
    if p['clean']:
        d.text((80, y), 'clean shutdown / reboot', font=F, fill=FG)
    else:
        _inverted(d, 80, y, 'ABRUPT: power loss / hang')
    y += ROW + 8
    _label(d, y, 'LAST SYSTEMD LINE')
    y += ROW
    for ln in _wrap(d, p['last_systemd'] or '-', F, W - 12, 3):
        d.text((8, y), ln, font=F, fill=FG)
        y += ROW
    y += 6
    _label(d, y, 'LAST ERROR')
    y += ROW
    for ln in _wrap(d, p['last_error'] or 'none', F, W - 12, 4):
        d.text((8, y), ln, font=F, fill=FG)
        y += ROW
    d.text((4, H - ROW - 1), 'this boot follows ...', font=F, fill=DIM)
    return img


def render_boot(s, prev=None, handover=False):
    img = Image.new('RGB', (W, H), BG)
    d = _Draw(img)
    _header(d, 'C-WANG  booting', f'{s["uptime_s"]:.0f} s')
    y = 21
    _label(d, y, 'SYSTEM')
    state = s['state']
    if state in ('degraded', 'maintenance', 'stopping'):
        _inverted(d, 62, y, state)
    else:
        d.text((62, y), state, font=FB, fill=FG)
    d.text((170, y), f'{len(s["jobs"])} jobs left', font=F, fill=DIM if s['jobs'] else FG)
    y += ROW + 2
    _label(d, y, 'WAITING ON')
    y += ROW
    for st, unit in s['jobs'][:3]:
        d.text((8, y), _clip(d, f'{"> " if st == "running" else "  "}{unit}', F, W - 70), font=F, fill=FG)
        d.text((W - 60, y), st, font=F, fill=DIM)
        y += ROW
    if not s['jobs']:
        d.text((8, y), 'nothing: boot finished', font=F, fill=FG)
        y += ROW
    y += 2
    _label(d, y, 'FAILED')
    if s['failed']:
        _inverted(d, 62, y, _clip(d, ' '.join(s['failed']), FB, W - 70))
    else:
        d.text((62, y), 'none', font=F, fill=FG)
    y += ROW + 2
    _label(d, y, 'LAST ERROR')
    y += ROW
    for ln in _wrap(d, s['error'] or 'none', F, W - 12, 2):
        d.text((8, y), ln, font=F, fill=FG)
        y += ROW
    y = max(y + 2, 150)
    ssid, ip = s['wifi']
    _label(d, y, 'WI-FI')
    if ssid or ip:
        d.text((62, y), _clip(d, f'{ssid or "?"}  {ip or "no IP yet"}', F, W - 66), font=F, fill=FG)
    else:
        d.text((62, y), 'not connected', font=F, fill=DIM)
    y += ROW + 2
    _label(d, y, 'DEVICES')
    x = 62
    for name, ok in s['dev']:
        _dot(d, x, y, ok)
        d.text((x + 10, y), name, font=F, fill=FG if ok else DIM)
        x += 14 + d.textlength(name, font=F)
    y += ROW
    up, cst = s['can']
    x = 62
    _dot(d, x, y, up == 'UP' and cst == 'ERROR-ACTIVE', bad=cst in ('BUS-OFF', 'ERROR-PASSIVE'))
    d.text((x + 10, y), f'can0 {up} {cst}'.strip(), font=F, fill=FG if up == 'UP' else DIM)
    y += ROW + 2
    _label(d, y, 'SERVICES')
    x = 62
    for short, active, restarts in s['svc']:
        _dot(d, x, y, active == 'active', bad=active == 'failed' or restarts > 0)
        t = short + (f'x{restarts}' if restarts else '')
        d.text((x + 10, y), t, font=F, fill=FG if active == 'active' else DIM)
        x += 14 + d.textlength(t, font=F)
    if handover:
        foot = 'status screen starting ...'
    elif prev:
        foot = f'prev boot: {_dur(prev["duration_s"])} {"clean" if prev["clean"] else "ABRUPT end"}'
    else:
        foot = 'prev boot: no record'
    d.text((4, H - ROW - 1), foot, font=F, fill=DIM)
    return img


# ---------------------------------------------------------------- LCD loop
def _lcd_config():
    cfg = dict(spi_dev='/dev/spidev0.0', speed_hz=20_000_000, dc=22, rst=15, bl=18,
               invert=True, bgr=False, rotate_180=False)
    try:
        import yaml
        prefix = Path(__file__).resolve().parents[4]   # <install>/gen2_status_display
        y = yaml.safe_load(open(prefix / 'share/gen2_status_display/config/status_display.yaml'))
        p = y['status_display']['ros__parameters']
        cfg.update(spi_dev=p.get('spi_device', cfg['spi_dev']), speed_hz=p.get('spi_speed_hz', cfg['speed_hz']),
                   dc=p.get('pin_dc', 22), rst=p.get('pin_rst', 15), bl=p.get('pin_bl', 18),
                   invert=p.get('invert_colors', True), bgr=p.get('bgr', False),
                   rotate_180=p.get('rotate_180', False))
    except Exception:  # defaults match the Seengreat wiring
        pass
    return cfg


def main():
    if '--png' in sys.argv:   # offline preview: no LCD, no lock
        out = sys.argv[sys.argv.index('--png') + 1]
        prev = previous_boot()
        render_boot(gather(), prev).save(out)
        if prev:
            render_previous(prev).save(out.replace('.png', '_prev.png'))
        return
    lock = lcd_handover.try_own()
    if lock is None:
        print('bootscreen: LCD already owned (status screen running), exiting')
        return
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    from .st7789 import ST7789
    lcd = ST7789(**_lcd_config())
    shown = None

    def push(img):
        nonlocal shown
        if shown is None:
            lcd.show(img)
        else:
            box = ImageChops.difference(img, shown).getbbox()
            if box:
                lcd.show_region(img, box)
        shown = img

    prev = previous_boot()
    t0 = time.monotonic()
    if prev:
        push(render_previous(prev))
    next_t = t0 + (PREV_S if prev else 0.0)
    while not stop:
        if lcd_handover.requested():
            try:
                push(render_boot(gather(), prev, handover=True))
            except Exception:
                pass
            break
        now = time.monotonic()
        if now >= next_t:
            next_t = now + 1.0
            try:
                push(render_boot(gather(), prev))
            except Exception as e:  # keep the last frame rather than die
                print(f'bootscreen: {e}', file=sys.stderr)
        time.sleep(0.1)
    lcd.close()
    # the lock is released when this process exits, together with the GPIO lines


if __name__ == '__main__':
    main()
