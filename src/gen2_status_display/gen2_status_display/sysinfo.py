"""Cheap 1 Hz Jetson / network probes from sysfs & procfs (no root, no heavy subprocesses)."""
import fcntl
import glob
import os
import socket
import struct
import subprocess
import time


def _read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def _num(path, scale=1.0):
    v = _read(path)
    try:
        return float(v) * scale
    except (TypeError, ValueError):
        return None


class SysInfo:
    def __init__(self, wifi_iface='auto', eth_iface='auto'):
        self._cpu_prev = None
        self.zones = {}
        for z in glob.glob('/sys/class/thermal/thermal_zone*'):
            self.zones[_read(z + '/type', '')] = z + '/temp'
        self.gpu_load = next((p for p in ('/sys/devices/platform/bus@0/17000000.gpu/load',
                                          '/sys/devices/platform/17000000.gpu/load')
                              if os.path.exists(p)), None)
        self.ina = None
        for h in glob.glob('/sys/class/hwmon/hwmon*'):
            if _read(h + '/name') == 'ina3221':
                for i in range(1, 4):
                    if _read(f'{h}/in{i}_label') == 'VDD_IN':
                        self.ina = (f'{h}/in{i}_input', f'{h}/curr{i}_input')
        self.fan = next((h + '/rpm' for h in glob.glob('/sys/class/hwmon/hwmon*')
                         if _read(h + '/name') == 'pwm_tach'), None)
        nets = [os.path.basename(p) for p in glob.glob('/sys/class/net/*')]
        self.wifi = wifi_iface if wifi_iface != 'auto' else next(
            (n for n in sorted(nets) if n.startswith('wl')), None)
        self.eth = eth_iface if eth_iface != 'auto' else next(
            (n for n in sorted(nets) if n.startswith(('en', 'eth'))), None)
        self.power_mode = None
        try:
            out = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True, timeout=2)
            for line in out.stdout.splitlines():
                if 'Power Mode' in line:
                    self.power_mode = line.split(':')[-1].strip()
        except Exception:
            pass
        self._ssid, self._ssid_t = None, 0.0

    def _cpu_percent(self):
        f = _read('/proc/stat', '').splitlines()[0].split()[1:]
        v = [int(x) for x in f]
        idle, total = v[3] + v[4], sum(v)
        pct = None
        if self._cpu_prev:
            di, dt = idle - self._cpu_prev[0], total - self._cpu_prev[1]
            pct = 100.0 * (1.0 - di / dt) if dt > 0 else None
        self._cpu_prev = (idle, total)
        return pct

    def _temp(self, name):
        p = self.zones.get(name)
        return _num(p, 1e-3) if p else None

    @staticmethod
    def _ipv4(iface):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            r = fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', iface[:15].encode()))
            s.close()
            return socket.inet_ntoa(r[20:24])
        except OSError:
            return None

    def _wifi_signal(self):
        for line in (_read('/proc/net/wireless', '') or '').splitlines()[2:]:
            parts = line.split()
            if parts and parts[0].rstrip(':') == self.wifi:
                try:
                    return float(parts[3].rstrip('.'))  # dBm
                except (IndexError, ValueError):
                    return None
        return None

    def _ssid_now(self):
        if time.monotonic() - self._ssid_t > 10.0:  # SSID rarely changes: poll every 10 s
            self._ssid_t = time.monotonic()
            self._ssid = None
            try:
                out = subprocess.run(['iw', 'dev', self.wifi, 'link'], capture_output=True,
                                     text=True, timeout=1)
                for line in out.stdout.splitlines():
                    if line.strip().startswith('SSID:'):
                        self._ssid = line.split(':', 1)[1].strip()
            except Exception:
                pass
        return self._ssid

    @staticmethod
    def _ssh_sessions():
        n = 0
        for f in ('/proc/net/tcp', '/proc/net/tcp6'):
            for line in (_read(f, '') or '').splitlines()[1:]:
                p = line.split()
                if len(p) > 3 and p[1].endswith(':0016') and p[3] == '01':
                    n += 1
        return n

    def sample(self):
        mem = {}
        for line in (_read('/proc/meminfo', '') or '').splitlines():
            k, v = line.split(':', 1)
            mem[k] = float(v.split()[0]) / 1048576.0  # GiB
        jetson_w = None
        if self.ina:
            v, a = _num(self.ina[0], 1e-3), _num(self.ina[1], 1e-3)
            jetson_w = v * a if v is not None and a is not None else None
        wifi_up = self.wifi and _read(f'/sys/class/net/{self.wifi}/operstate') == 'up'
        return dict(
            cpu=self._cpu_percent(),
            gpu=_num(self.gpu_load, 0.1) if self.gpu_load else None,
            t_cpu=self._temp('cpu-thermal'), t_gpu=self._temp('gpu-thermal'),
            t_tj=self._temp('tj-thermal'),
            ram_used=mem.get('MemTotal', 0) - mem.get('MemAvailable', 0),
            ram_total=mem.get('MemTotal'),
            fan_rpm=_num(self.fan) if self.fan else None,
            jetson_w=jetson_w, power_mode=self.power_mode,
            wifi_iface=self.wifi, wifi_up=bool(wifi_up),
            ssid=self._ssid_now() if wifi_up else None,
            wifi_dbm=self._wifi_signal() if wifi_up else None,
            wifi_ip=self._ipv4(self.wifi) if wifi_up else None,
            eth_up=bool(self.eth and _read(f'/sys/class/net/{self.eth}/carrier') == '1'),
            eth_ip=self._ipv4(self.eth) if self.eth else None,
            ssh=self._ssh_sessions(),
            video0=os.path.exists('/dev/video0'),
            lidar_port=bool(glob.glob('/dev/serial/by-id/usb-Silicon_Labs_CP2102*')),
        )
