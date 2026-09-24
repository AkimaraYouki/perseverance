#!/usr/bin/env python3
"""
SHR1 패킷 테스트 — biped_sensor_hub_esp32_c3 (ESP32-C3) USB CDC 스트림 검증기.

펌웨어: ~/Downloads/biped_sensor_hub_esp32_c3_gpio8_fixed.ino
프레임: 136 byte packed, magic 0x31524853("SHR1" LE), version 1,
        CRC32-IEEE(poly 0xEDB88320, init/final 0xFFFFFFFF) over 앞 132 byte,
        100 Hz 송신.

사용법:
    python3 shr1_packet_test.py                 # 자동 포트 탐지 + 라이브 대시보드
    python3 shr1_packet_test.py -p /dev/ttyACM1 -d 10
    python3 shr1_packet_test.py --once          # 한 프레임 전체 디코드 후 종료
    python3 shr1_packet_test.py --selftest      # 보드 없이 파서 검증
    python3 shr1_packet_test.py --csv log.csv   # CSV 기록
    python3 shr1_packet_test.py --json          # 프레임당 JSON 한 줄 (파이프용)
"""
import argparse
import collections
import json
import math
import os
import struct
import sys
import time
import zlib

# ---------------------------------------------------------------- 프레임 정의
FRAME_MAGIC = 0x31524853  # "SHR1" little-endian
FRAME_VERSION = 1
FRAME_SIZE = 136
MAGIC_BYTES = struct.pack("<I", FRAME_MAGIC)

# .ino 의 struct __attribute__((packed)) HostFrame 과 1:1 대응 (순서 변경 금지)
FIELDS = [
    ("magic", "I"), ("version", "H"), ("frame_size", "H"),
    ("sequence", "I"), ("esp_time_us", "Q"), ("flags", "I"),
    ("usb_drop_count", "I"), ("gps_checksum_error_count", "I"),
    # Power
    ("compute_mV", "H"), ("compute_mA", "i"),
    ("motor_mV", "H"), ("motor_mA", "i"),
    # GNSS epoch / UTC
    ("gps_iTOW_ms", "I"), ("year", "H"),
    ("month", "B"), ("day", "B"), ("hour", "B"),
    ("minute", "B"), ("second", "B"), ("gps_valid_flags", "B"),
    ("gps_tAcc_ns", "I"), ("gps_nano_ns", "i"),
    # GNSS solution
    ("gps_fix_type", "B"), ("gps_fix_flags", "B"),
    ("gps_num_sv", "B"), ("reserved0", "B"),
    ("lon_e7", "i"), ("lat_e7", "i"), ("height_mm", "i"), ("hMSL_mm", "i"),
    ("hAcc_mm", "I"), ("vAcc_mm", "I"),
    ("velN_mms", "i"), ("velE_mms", "i"), ("velD_mms", "i"),
    ("gSpeed_mms", "i"), ("headMot_e5", "i"),
    ("sAcc_mms", "I"), ("headAcc_e5", "I"),
    ("pDOP_centi", "H"),
    # 펌웨어 2026-09-24 판부터 reserved1 자리에 GPS UART 수신 바이트 하위 16비트.
    # 구 펌웨어는 0 이다. 크기·오프셋(122)은 같아 프레임 호환.
    ("gps_rx_bytes_lo16", "H"),
    # Magnetometer, raw IST8310 counts
    ("mag_x", "h"), ("mag_y", "h"), ("mag_z", "h"), ("reserved2", "H"),
    ("crc32", "I"),
]
FMT = "<" + "".join(c for _, c in FIELDS)
NAMES = [n for n, _ in FIELDS]
CRC_LEN = FRAME_SIZE - 4  # crc32 필드 직전까지 = 132

assert struct.calcsize(FMT) == FRAME_SIZE, \
    f"파이썬 레이아웃 {struct.calcsize(FMT)} != 펌웨어 {FRAME_SIZE}"

FLAG_BITS = [
    (1 << 0, "PM1_VALID"),
    (1 << 1, "PM2_VALID"),
    (1 << 2, "GPS_PACKET_SEEN"),
    (1 << 3, "GPS_FIX_OK"),
    (1 << 4, "GPS_3D"),
    (1 << 5, "GPS_HACC_OK"),
    (1 << 6, "GPS_USABLE"),
    (1 << 7, "GPS_TIME_VALID"),
    (1 << 8, "MAG_VALID"),
]
FIX_TYPE = {0: "no-fix", 1: "dead-reckoning", 2: "2D", 3: "3D",
            4: "GNSS+DR", 5: "time-only"}
IST8310_UT_PER_LSB = 0.3  # 데이터시트 표준 감도, 참고용 환산


def crc_of(buf):
    """앞 132 byte 에 대한 CRC32-IEEE. 펌웨어 crc32_ieee() 와 동일."""
    return zlib.crc32(buf[:CRC_LEN]) & 0xFFFFFFFF


def decode(buf):
    return dict(zip(NAMES, struct.unpack(FMT, buf)))


def flag_names(flags):
    return [n for b, n in FLAG_BITS if flags & b]


# ------------------------------------------------------------------- 파서 FSM
class Parser:
    """바이트 스트림 -> 검증된 프레임. 쓰레기/부분 프레임에서 자동 재동기."""

    def __init__(self):
        self.buf = bytearray()
        self.n_ok = 0
        self.n_crc_err = 0
        self.n_bad_header = 0   # frame_size/version 불일치
        self.n_resync = 0       # 재동기 이벤트 수
        self.n_discarded = 0    # 프레임 밖으로 버린 바이트

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(MAGIC_BYTES)
            if i < 0:
                # magic 일부가 꼬리에 걸쳐 있을 수 있으니 3 byte 만 남긴다
                keep = min(len(self.buf), 3)
                self.n_discarded += len(self.buf) - keep
                if len(self.buf) > keep:
                    self.n_resync += 1
                del self.buf[:len(self.buf) - keep]
                return out
            if i > 0:
                self.n_discarded += i
                self.n_resync += 1
                del self.buf[:i]
            if len(self.buf) < FRAME_SIZE:
                return out                       # 더 받아야 함
            raw = bytes(self.buf[:FRAME_SIZE])
            f = decode(raw)
            if f["frame_size"] != FRAME_SIZE or f["version"] != FRAME_VERSION:
                self.n_bad_header += 1
                del self.buf[:4]                 # 이 magic 은 가짜, 재탐색
                continue
            if f["crc32"] != crc_of(raw):
                self.n_crc_err += 1
                del self.buf[:4]
                continue
            self.n_ok += 1
            del self.buf[:FRAME_SIZE]
            out.append((f, raw))


# ------------------------------------------------------------------- 통계 누적
class Stats:
    def __init__(self):
        self.t0 = time.monotonic()
        self.first = None
        self.last = None
        self.prev_seq = None
        self.seq_missing = 0     # 빠진 프레임 개수 총합
        self.seq_gaps = 0        # 불연속 이벤트 수
        self.seq_backwards = 0
        self.dt_us = []          # esp_time_us 간격
        self.host_dt = []        # 호스트 도착 간격
        self.prev_host_t = None
        self.flags_hist = collections.Counter()
        self.usb_drop_start = None
        self.usb_drop_last = 0
        self.gps_crcerr_start = None
        self.gps_crcerr_last = 0

    def add(self, f):
        now = time.monotonic()
        if self.first is None:
            self.first = f
            self.usb_drop_start = f["usb_drop_count"]
            self.gps_crcerr_start = f["gps_checksum_error_count"]
        else:
            d = (f["sequence"] - self.prev_seq) & 0xFFFFFFFF
            if d == 0 or d > 0x7FFFFFFF:
                self.seq_backwards += 1
            elif d > 1:
                self.seq_gaps += 1
                self.seq_missing += d - 1
            self.dt_us.append(f["esp_time_us"] - self.last["esp_time_us"])
            if self.prev_host_t is not None:
                self.host_dt.append((now - self.prev_host_t) * 1e6)
        self.prev_host_t = now
        self.prev_seq = f["sequence"]
        self.last = f
        self.flags_hist[f["flags"]] += 1
        self.usb_drop_last = f["usb_drop_count"]
        self.gps_crcerr_last = f["gps_checksum_error_count"]

    @property
    def elapsed(self):
        return time.monotonic() - self.t0

    @property
    def expected(self):
        """seq 기준으로 보드가 보냈어야 할 총 프레임 수."""
        if self.first is None:
            return 0
        return ((self.last["sequence"] - self.first["sequence"]) & 0xFFFFFFFF) + 1

    @property
    def usb_dropped(self):
        if self.usb_drop_start is None:
            return 0
        return (self.usb_drop_last - self.usb_drop_start) & 0xFFFFFFFF

    @property
    def gps_crc_errors(self):
        if self.gps_crcerr_start is None:
            return 0
        return (self.gps_crcerr_last - self.gps_crcerr_start) & 0xFFFFFFFF


def _spread(v):
    if not v:
        return (0.0, 0.0, 0.0, 0.0)
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return (min(v), m, max(v), sd)


# --------------------------------------------------------------------- 출력부
def dump_frame(f, raw):
    L = []
    a = L.append
    a("── 원시 바이트 (136) " + "─" * 40)
    for off in range(0, FRAME_SIZE, 16):
        chunk = raw[off:off + 16]
        a(f"  {off:3d}  {chunk.hex(' '):<48}")
    a("")
    a("── 헤더 " + "─" * 53)
    a(f"  magic       0x{f['magic']:08X}  ('SHR1')   version {f['version']}   frame_size {f['frame_size']}")
    a(f"  sequence    {f['sequence']}")
    a(f"  esp_time    {f['esp_time_us']} us  ({f['esp_time_us']/1e6:.3f} s 부팅 후)")
    a(f"  flags       0x{f['flags']:04X}  {' | '.join(flag_names(f['flags'])) or '(없음)'}")
    a(f"  usb_drop    {f['usb_drop_count']}     gps_ck_err {f['gps_checksum_error_count']}")
    a(f"  crc32       0x{f['crc32']:08X}  (재계산 0x{crc_of(raw):08X})  "
      f"{'OK' if f['crc32'] == crc_of(raw) else '불일치'}")
    a("")
    a("── 전력 (PM02D) " + "─" * 45)
    pm1 = "유효" if f["flags"] & 1 else "무효(플래그 off)"
    pm2 = "유효" if f["flags"] & 2 else "무효(플래그 off)"
    a(f"  compute  {f['compute_mV']/1000:8.3f} V  {f['compute_mA']/1000:8.3f} A  "
      f"{f['compute_mV']*f['compute_mA']/1e6:7.2f} W   [{pm1}]")
    a(f"  motor    {f['motor_mV']/1000:8.3f} V  {f['motor_mA']/1000:8.3f} A  "
      f"{f['motor_mV']*f['motor_mA']/1e6:7.2f} W   [{pm2}]")
    a("")
    a("── GNSS " + "─" * 53)
    a(f"  fix         {FIX_TYPE.get(f['gps_fix_type'], f['gps_fix_type'])}"
      f"   sv {f['gps_num_sv']}   fix_flags 0x{f['gps_fix_flags']:02X}")
    a(f"  UTC         {f['year']:04d}-{f['month']:02d}-{f['day']:02d} "
      f"{f['hour']:02d}:{f['minute']:02d}:{f['second']:02d}"
      f"  valid 0x{f['gps_valid_flags']:02X}  iTOW {f['gps_iTOW_ms']} ms")
    a(f"  lat/lon     {f['lat_e7']/1e7:.7f}, {f['lon_e7']/1e7:.7f}")
    a(f"  height      ellip {f['height_mm']/1000:.3f} m   MSL {f['hMSL_mm']/1000:.3f} m")
    a(f"  정확도      hAcc {f['hAcc_mm']/1000:.3f} m   vAcc {f['vAcc_mm']/1000:.3f} m   "
      f"pDOP {f['pDOP_centi']/100:.2f}")
    a(f"  속도 NED    {f['velN_mms']/1000:.3f} / {f['velE_mms']/1000:.3f} / "
      f"{f['velD_mms']/1000:.3f} m/s   지상속도 {f['gSpeed_mms']/1000:.3f} m/s")
    a(f"  heading     {f['headMot_e5']/1e5:.3f} deg  (±{f['headAcc_e5']/1e5:.3f})   "
      f"sAcc {f['sAcc_mms']/1000:.3f} m/s")
    a(f"  UART 수신   {f['gps_rx_bytes_lo16']} byte (하위 16비트, 0 이면 구 펌웨어 또는 무수신)")
    a("")
    a("── 지자기 (IST8310 raw) " + "─" * 37)
    mx, my, mz = f["mag_x"], f["mag_y"], f["mag_z"]
    mag = math.sqrt(mx * mx + my * my + mz * mz)
    a(f"  x/y/z       {mx} / {my} / {mz} counts   "
      f"[{'유효' if f['flags'] & (1 << 8) else '무효'}]")
    a(f"  크기        {mag:.1f} counts  ≈ {mag*IST8310_UT_PER_LSB:.1f} uT "
      f"(0.3 uT/LSB 공칭, 지구자기장 25~65 uT)")
    a(f"  방위각 xy   {math.degrees(math.atan2(my, mx)) % 360:.1f} deg (미보정)")
    a("")
    a("── 예약 필드 (0 이어야 정상) " + "─" * 32)
    a(f"  reserved0 {f['reserved0']}  reserved2 {f['reserved2']}")
    return "\n".join(L)


def report(st, ps, wallclock=True):
    L = []
    a = L.append
    el = st.elapsed
    if not wallclock and ps.n_ok >= 2:
        span = (st.last["esp_time_us"] - st.first["esp_time_us"]) / 1e6
        el = span if span > 0 else el
    a("")
    a("=" * 68)
    a(f"  SHR1 패킷 테스트 결과   ({el:.1f} s"
      f"{'' if wallclock else ', ESP 타임스탬프 기준'})")
    a("=" * 68)
    a("")
    a("[ 프레임 무결성 ]")
    a(f"  정상 수신        {ps.n_ok}")
    a(f"  CRC 오류         {ps.n_crc_err}")
    a(f"  헤더 오류        {ps.n_bad_header}   (frame_size/version 불일치)")
    a(f"  재동기 이벤트    {ps.n_resync}")
    a(f"  버린 바이트      {ps.n_discarded}")
    a(f"  잔여 버퍼        {len(ps.buf)} byte")
    if ps.n_ok:
        integrity = 100.0 * ps.n_ok / max(1, ps.n_ok + ps.n_crc_err + ps.n_bad_header)
        a(f"  → 무결성         {integrity:.4f} %")
    a("")
    a("[ 수신율 / 연속성 ]")
    if ps.n_ok >= 2:
        a(f"  실측 레이트      {ps.n_ok/el:.2f} Hz   (펌웨어 목표 100 Hz)")
        a(f"  seq 범위         {st.first['sequence']} → {st.last['sequence']}  "
          f"(보드 송신 {st.expected})")
        a(f"  누락 프레임      {st.seq_missing}  ({st.seq_gaps} 회 끊김)"
          f"   역행/중복 {st.seq_backwards}")
        loss = 100.0 * st.seq_missing / max(1, st.expected)
        a(f"  → 손실률         {loss:.4f} %")
        lo, mean, hi, sd = _spread(st.dt_us)
        a(f"  ESP 송신 간격    min {lo/1000:.2f} / 평균 {mean/1000:.2f} / "
          f"max {hi/1000:.2f} ms   σ {sd/1000:.3f}")
        if wallclock:
            lo, mean, hi, sd = _spread(st.host_dt)
            a(f"  호스트 도착 간격 min {lo/1000:.2f} / 평균 {mean/1000:.2f} / "
              f"max {hi/1000:.2f} ms   σ {sd/1000:.3f}  (USB 버퍼링 포함)")
        a(f"  대역폭           {ps.n_ok*FRAME_SIZE/el/1024:.1f} KiB/s  "
          f"(921600 baud ≈ 90 KiB/s 상한의 "
          f"{100*ps.n_ok*FRAME_SIZE*10/el/921600:.1f} %)")
    a("")
    a("[ 보드 자체 카운터 (측정 구간 증분) ]")
    a(f"  usb_drop_count           +{st.usb_dropped}"
      f"   (호스트가 못 읽어 보드가 버린 프레임)")
    a(f"  gps_checksum_error_count +{st.gps_crc_errors}")
    if st.last:
        a(f"  누적값: usb_drop {st.last['usb_drop_count']}, "
          f"gps_ck_err {st.last['gps_checksum_error_count']}")
    a("")
    a("[ 플래그 분포 ]")
    for fl, cnt in st.flags_hist.most_common(8):
        names = " | ".join(flag_names(fl)) or "(없음)"
        a(f"  0x{fl:04X}  {cnt:6d}  {100*cnt/max(1,ps.n_ok):5.1f} %   {names}")
    a("")
    a("[ 판정 ]")
    ok = True
    def chk(cond, good, bad):
        nonlocal ok
        a(f"  {'PASS' if cond else 'FAIL'}  {good if cond else bad}")
        if not cond:
            ok = False
    chk(ps.n_ok > 0, "프레임 수신됨", "프레임을 하나도 못 받음")
    chk(ps.n_crc_err == 0, "CRC 오류 없음", f"CRC 오류 {ps.n_crc_err} 건")
    chk(ps.n_bad_header == 0, "헤더 정상", f"헤더 오류 {ps.n_bad_header} 건")
    chk(st.seq_missing == 0, "시퀀스 누락 없음", f"프레임 {st.seq_missing} 개 누락")
    if ps.n_ok >= 2:
        rate = ps.n_ok / el
        chk(95 <= rate <= 105, f"레이트 정상 ({rate:.1f} Hz)",
            f"레이트 이탈 ({rate:.1f} Hz, 목표 100)")
    chk(st.usb_dropped == 0, "보드측 USB 드랍 없음",
        f"보드가 {st.usb_dropped} 프레임 드랍 (호스트 소비 속도 부족)")
    a("")
    a("  " + ("=> 종합 PASS" if ok else "=> 종합 FAIL"))
    a("=" * 68)
    return "\n".join(L)


# --------------------------------------------------------------------- 셀프테스트
def build_frame(seq, t_us, flags=0x1FF, **over):
    vals = {n: 0 for n in NAMES}
    vals.update(magic=FRAME_MAGIC, version=FRAME_VERSION, frame_size=FRAME_SIZE,
                sequence=seq, esp_time_us=t_us, flags=flags,
                compute_mV=12345, compute_mA=2500, motor_mV=24000, motor_mA=8000,
                gps_fix_type=3, gps_num_sv=14, lat_e7=374566000, lon_e7=1269780000,
                hAcc_mm=1200, pDOP_centi=110, year=2026, month=9, day=24,
                mag_x=120, mag_y=-45, mag_z=310)
    vals.update(over)
    raw = bytearray(struct.pack(FMT, *[vals[n] for n in NAMES]))
    raw[CRC_LEN:] = struct.pack("<I", crc_of(raw))
    return bytes(raw)


def selftest():
    print("=" * 68)
    print("  오프라인 셀프테스트 — 보드 없이 파서/CRC/재동기 검증")
    print("=" * 68)
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail else ""))
        if not cond:
            fails.append(name)

    check("struct 크기 = 136", struct.calcsize(FMT) == FRAME_SIZE,
          f"계산값 {struct.calcsize(FMT)}")
    check("magic 바이트열 = 53 48 52 31", MAGIC_BYTES.hex(' ') == "53 48 52 31",
          MAGIC_BYTES.hex(' '))
    # 펌웨어 crc32_ieee 와 zlib 동치 확인 (알려진 벡터)
    check("CRC32 알고리즘 일치 (\"123456789\" -> 0xCBF43926)",
          zlib.crc32(b"123456789") & 0xFFFFFFFF == 0xCBF43926,
          f"0x{zlib.crc32(b'123456789') & 0xFFFFFFFF:08X}")

    # 왕복
    f0 = build_frame(1, 1_000_000)
    d = decode(f0)
    check("빌드→디코드 왕복", d["sequence"] == 1 and d["crc32"] == crc_of(f0))

    # 깨끗한 스트림
    p, s = Parser(), Stats()
    stream = b"".join(build_frame(i, i * 10_000) for i in range(1, 101))
    for f, _ in p.feed(stream):
        s.add(f)
    check("연속 100 프레임 전량 파싱", p.n_ok == 100 and s.seq_missing == 0,
          f"ok={p.n_ok} missing={s.seq_missing}")

    # 바이트 단위로 쪼개 넣기 (부분 프레임 복원)
    p2, got = Parser(), 0
    for i in range(0, len(stream), 7):
        got += len(p2.feed(stream[i:i + 7]))
    check("7 byte 씩 분할 입력해도 동일", got == 100, f"ok={got}")

    # 쓰레기 삽입 → 재동기
    p3 = Parser()
    noisy = (build_frame(1, 0) + b"\xde\xad\xbe\xef" * 5 + build_frame(2, 10_000)
             + b"SHR1garbage" + build_frame(3, 20_000))
    n = len(p3.feed(noisy))
    check("쓰레기 사이에서 재동기", n == 3 and p3.n_resync >= 1,
          f"ok={n} resync={p3.n_resync} discarded={p3.n_discarded}")

    # CRC 손상 검출
    p4 = Parser()
    bad = bytearray(build_frame(10, 0))
    bad[40] ^= 0x01            # 페이로드 1비트 뒤집기
    n = len(p4.feed(build_frame(9, 0) + bytes(bad) + build_frame(11, 0)))
    check("CRC 오류 검출 + 건너뛰기", n == 2 and p4.n_crc_err == 1,
          f"ok={n} crc_err={p4.n_crc_err}")

    # 잘못된 frame_size
    p5 = Parser()
    n = len(p5.feed(build_frame(1, 0, frame_size=999) + build_frame(2, 0)))
    check("헤더(frame_size) 오류 검출", n == 1 and p5.n_bad_header == 1,
          f"ok={n} bad_header={p5.n_bad_header}")

    # 시퀀스 누락 집계
    p6, s6 = Parser(), Stats()
    for f, _ in p6.feed(build_frame(1, 0) + build_frame(2, 10_000)
                        + build_frame(7, 60_000) + build_frame(8, 70_000)):
        s6.add(f)
    check("시퀀스 누락 집계 (4 개, 1 회)",
          s6.seq_missing == 4 and s6.seq_gaps == 1,
          f"missing={s6.seq_missing} gaps={s6.seq_gaps}")

    # seq 32bit 랩어라운드
    p7, s7 = Parser(), Stats()
    for f, _ in p7.feed(build_frame(0xFFFFFFFE, 0) + build_frame(0xFFFFFFFF, 1)
                        + build_frame(0, 2) + build_frame(1, 3)):
        s7.add(f)
    check("seq 32bit 랩어라운드 처리",
          s7.seq_missing == 0 and s7.seq_backwards == 0,
          f"missing={s7.seq_missing} back={s7.seq_backwards}")

    # 필드 오프셋 표 (펌웨어와 수동 대조용)
    print()
    print("  ── 필드 오프셋 (펌웨어 struct 와 대조) " + "─" * 22)
    off = 0
    line = []
    for n_, c in FIELDS:
        line.append(f"{n_}@{off}")
        off += struct.calcsize("<" + c)
        if len(line) == 4:
            print("    " + "  ".join(f"{x:<26}" for x in line))
            line = []
    if line:
        print("    " + "  ".join(f"{x:<26}" for x in line))
    print(f"    총 {off} byte")
    print()
    if fails:
        print(f"  => 셀프테스트 FAIL: {', '.join(fails)}")
        return 1
    print("  => 셀프테스트 전부 PASS. 파서는 펌웨어 규격과 일치.")
    return 0


# ------------------------------------------------------------------ 포트 탐지
def find_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    cands = []
    for p in list_ports.comports():
        vid, pid = (p.vid or 0), (p.pid or 0)
        if (vid, pid) == (0x303A, 0x1001):      # Espressif USB JTAG/serial
            cands.insert(0, p.device)
        elif vid == 0x303A or "ACM" in p.device:
            cands.append(p.device)
    return cands[0] if cands else None


def port_holders(dev):
    """이 포트를 이미 열고 있는 다른 프로세스 (바이트를 훔쳐감)."""
    out = []
    try:
        real = os.path.realpath(dev)
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == os.getpid():
                continue
            fddir = f"/proc/{pid}/fd"
            try:
                for fd in os.listdir(fddir):
                    if os.path.realpath(os.path.join(fddir, fd)) == real:
                        cmd = open(f"/proc/{pid}/cmdline", "rb").read()
                        out.append((pid, cmd.replace(b"\0", b" ").decode(errors="replace").strip()))
                        break
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                continue
    except Exception:
        pass
    return out



# ----------------------------------------------------------- 녹화 재생 / 루프백
def replay(path):
    """--record 로 저장한 원시 바이트를 그대로 파서에 통과시킨다."""
    parser, stats = Parser(), Stats()
    total = 0
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(65536)
            if not chunk:
                break
            total += len(chunk)
            for f, _ in parser.feed(chunk):
                stats.add(f)
    print(f"재생: {path}  ({total} byte)")
    print(report(stats, parser, wallclock=False))
    return 0 if (parser.n_ok and not parser.n_crc_err and not stats.seq_missing) else 1


def loopback(seconds=3.0, rate_hz=100.0, inject_noise=False):
    """가상 시리얼(pty)에 펌웨어를 흉내 낸 100 Hz 송신기를 띄우고
    실제 수신 경로 전체(포트 열기 → 읽기 → 파싱 → 통계)를 검증한다."""
    import pty
    import random
    import threading

    try:
        import serial
    except ImportError:
        print("pyserial 필요: pip install pyserial", file=sys.stderr)
        return 2

    master, slave = pty.openpty()
    dev = os.ttyname(slave)
    print(f"가상 보드: {dev} @ {rate_hz:.0f} Hz, {seconds:.0f} s"
          + ("  (노이즈/손상 주입)" if inject_noise else ""))

    stop = threading.Event()

    def emitter():
        seq, t_us, period = 1, 0, 1.0 / rate_hz
        drops = 0
        nxt = time.monotonic()
        while not stop.is_set():
            nxt += period
            t_us += int(period * 1e6)
            pkt = build_frame(seq, t_us, flags=0x1FF, usb_drop_count=drops,
                              compute_mV=12000 + (seq % 100),
                              motor_mV=24000, motor_mA=3000 + (seq % 500),
                              mag_x=(seq % 200) - 100, mag_y=50, mag_z=-300)
            if inject_noise:
                r = random.random()
                if r < 0.01:                      # 1 % 프레임 유실
                    seq += 1
                    continue
                if r < 0.02:                      # 1 % CRC 손상
                    b = bytearray(pkt); b[60] ^= 0x20; pkt = bytes(b)
                elif r < 0.03:                    # 1 % 앞쪽 쓰레기
                    pkt = os.urandom(random.randint(1, 9)) + pkt
            try:
                os.write(master, pkt)
            except OSError:
                return
            seq = (seq + 1) & 0xFFFFFFFF
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    th = threading.Thread(target=emitter, daemon=True)
    th.start()
    time.sleep(0.2)

    ser = serial.Serial(dev, 921600, timeout=0.05)
    parser, stats = Parser(), Stats()
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        data = ser.read(8192)
        if data:
            for f, _ in parser.feed(data):
                stats.add(f)
    stop.set()
    th.join(timeout=1.0)
    ser.close()
    os.close(master)

    print(report(stats, parser))
    if inject_noise:
        print("  (노이즈 주입 모드이므로 CRC 오류/누락 FAIL 은 정상 — "
              "검출 자체가 되는지가 확인 대상)")
        return 0 if parser.n_crc_err > 0 and stats.seq_missing > 0 else 1
    return 0 if (parser.n_ok and not parser.n_crc_err and not stats.seq_missing) else 1



def safe_read(ser, n=65536):
    """CDC 버퍼 오버런 시 pyserial 이 던지는 예외를 흡수한다."""
    try:
        return ser.read(n)
    except Exception:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        return b""


def stall_test(dev, baud=921600,
               stalls=(10, 20, 50, 100, 200, 500)):
    """호스트가 read 를 멈춘 동안 프레임이 언제부터 유실되는지 측정.

    젯슨 소비자 스레드가 얼마나 늦어도 되는지(스케줄링 여유)를 알려준다.
    경로: ESP TX 버퍼 -> USB -> 커널 tty 버퍼 -> 애플리케이션.
    """
    import serial
    ser = serial.Serial(dev, baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    def drain(sec):
        p = Parser()
        last = None
        t = time.monotonic() + sec
        while time.monotonic() < t:
            for f, _ in p.feed(safe_read(ser)):
                last = f
        return p, last

    print(f"포트 {dev} — 호스트 정지 내성 측정")
    print("주의: 1 s 이상 정지시키면 ESP32-C3 의 USB CDC 링크가 물려서")
    print("      장치가 USB 에서 떨어지고 스스로 복귀하지 않는다 (실측 확인).")
    print("      그래서 기본 시퀀스는 500 ms 에서 끝난다.")
    print("경로: ESP TX 버퍼 → USB → 커널 tty 버퍼 → 앱")
    print()
    print(f"{'정지':>7}  {'보드드랍':>8}  {'seq누락':>7}  {'tty잔량':>8}  판정")
    print("-" * 52)
    drain(1.0)
    worst_ok = 0
    for ms in stalls:
        _, before = drain(0.8)
        if before is None:
            print(f"{ms:5d}ms  {'?':>8}  {'?':>7}  {'?':>8}  프레임 없음")
            continue
        t0 = time.monotonic()
        while time.monotonic() - t0 < ms / 1000.0:
            time.sleep(0.001)          # read 는 하지 않음
        try:
            inwait = ser.in_waiting
        except Exception:
            inwait = -1
        p2, after = drain(0.8)
        if after is None:
            print(f"{ms:5d}ms  {'?':>8}  {'?':>7}  {inwait:7d}B  복구 실패")
            continue
        drop = (after["usb_drop_count"] - before["usb_drop_count"]) & 0xFFFFFFFF
        # 정지 구간을 가로지르는 seq 누락 = 보드가 보낸 수 - 우리가 받은 수
        sent = ((after["sequence"] - before["sequence"]) & 0xFFFFFFFF)
        miss = max(0, sent - p2.n_ok)
        verdict = "OK" if (drop == 0 and miss == 0) else "유실"
        if verdict == "OK":
            worst_ok = ms
        print(f"{ms:5d}ms  {drop:8d}  {miss:7d}  {inwait:7d}B  {verdict}")
    ser.close()
    print()
    print(f"  => 무손실 허용 정지시간 ≥ {worst_ok} ms "
          f"(젯슨 소비 스레드가 이보다 오래 멈추면 프레임이 사라짐)")
    return 0


# ----------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="SHR1 센서허브 패킷 테스트")
    ap.add_argument("-p", "--port", help="시리얼 포트 (기본: 자동 탐지)")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-d", "--duration", type=float, default=10.0, help="측정 시간 [s]")
    ap.add_argument("--once", action="store_true", help="첫 프레임 전체 디코드 후 종료")
    ap.add_argument("--selftest", action="store_true", help="보드 없이 파서 검증")
    ap.add_argument("--csv", help="CSV 기록 파일")
    ap.add_argument("--json", action="store_true", help="프레임당 JSON 한 줄")
    ap.add_argument("--quiet", action="store_true", help="라이브 대시보드 끄기")
    ap.add_argument("--record", help="수신 원시 바이트를 그대로 파일에 저장")
    ap.add_argument("--replay", help="녹화 파일을 재생해 파서 검증 (보드 불필요)")
    ap.add_argument("--loopback", nargs="?", const=3.0, type=float,
                    metavar="SEC", help="pty 가상보드로 수신 경로 전체 검증")
    ap.add_argument("--stall", action="store_true",
                    help="호스트 정지 내성 측정 (버퍼 여유)")
    ap.add_argument("--noise", action="store_true",
                    help="--loopback 에 손상/유실 주입 (검출 능력 확인)")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.replay:
        return replay(args.replay)
    if args.loopback is not None:
        return loopback(args.loopback, inject_noise=args.noise)

    try:
        import serial
    except ImportError:
        print("pyserial 필요:  pip install pyserial", file=sys.stderr)
        return 2

    dev = args.port or find_port()
    if not dev:
        print("ESP32 시리얼 포트를 못 찾음. -p /dev/ttyACMx 로 지정.", file=sys.stderr)
        return 2

    holders = port_holders(dev)
    if holders and not args.json:
        print(f"경고: {dev} 를 이미 열고 있는 프로세스가 있음 "
              f"(바이트를 나눠 가져가 손실로 보일 수 있음):", file=sys.stderr)
        for pid, cmd in holders:
            print(f"   pid {pid}  {cmd[:90]}", file=sys.stderr)

    if args.stall:
        return stall_test(dev, args.baud)

    try:
        ser = serial.Serial(dev, args.baud, timeout=0.05)
    except Exception as e:
        print(f"{dev} 열기 실패: {e}", file=sys.stderr)
        return 2

    time.sleep(0.2)
    ser.reset_input_buffer()

    parser, stats = Parser(), Stats()
    recf = open(args.record, "wb") if args.record else None
    csvf = None
    if args.csv:
        csvf = open(args.csv, "w")
        csvf.write("host_t," + ",".join(NAMES) + "\n")

    live = (not args.quiet) and (not args.json) and (not args.once) and sys.stdout.isatty()
    if not args.json:
        print(f"포트 {dev} @ {args.baud} baud, {args.duration:.0f} 초 측정"
              f"{' (라이브)' if live else ''}...  Ctrl-C 로 중단")
    last_draw = 0.0
    lines_drawn = 0
    t_end = time.monotonic() + args.duration
    first_raw = None

    try:
        while time.monotonic() < t_end:
            data = safe_read(ser, 8192)
            if not data:
                continue
            if recf:
                recf.write(data)
            for f, raw in parser.feed(data):
                stats.add(f)
                if first_raw is None:
                    first_raw = raw
                    if args.once:
                        raise KeyboardInterrupt
                if csvf:
                    csvf.write(f"{time.time():.6f}," +
                               ",".join(str(f[n]) for n in NAMES) + "\n")
                if args.json:
                    f2 = dict(f)
                    f2["flag_names"] = flag_names(f["flags"])
                    print(json.dumps(f2), flush=True)
            now = time.monotonic()
            if live and now - last_draw > 0.5 and stats.last:
                f = stats.last
                el = stats.elapsed
                blk = [
                    f"  수신 {parser.n_ok:7d}  {parser.n_ok/max(el,1e-9):6.1f} Hz   "
                    f"seq {f['sequence']}",
                    f"  CRC오류 {parser.n_crc_err}  헤더오류 {parser.n_bad_header}  "
                    f"누락 {stats.seq_missing}  재동기 {parser.n_resync}  "
                    f"보드드랍 +{stats.usb_dropped}",
                    f"  전력  compute {f['compute_mV']/1000:6.3f} V "
                    f"{f['compute_mA']/1000:6.3f} A   "
                    f"motor {f['motor_mV']/1000:6.3f} V {f['motor_mA']/1000:6.3f} A",
                    f"  자기  {f['mag_x']:6d} {f['mag_y']:6d} {f['mag_z']:6d}   "
                    f"GPS {FIX_TYPE.get(f['gps_fix_type'],'?')} sv={f['gps_num_sv']}",
                    f"  플래그 0x{f['flags']:04X}  {' | '.join(flag_names(f['flags'])) or '(없음)'}",
                    f"  남은 시간 {max(0, t_end-now):.1f} s",
                ]
                if lines_drawn:
                    sys.stdout.write(f"\033[{lines_drawn}A")
                for ln in blk:
                    sys.stdout.write("\033[2K" + ln + "\n")
                sys.stdout.flush()
                lines_drawn = len(blk)
                last_draw = now
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if csvf:
            csvf.close()
        if recf:
            recf.close()

    if args.once:
        if first_raw is None:
            print("프레임을 못 받음.", file=sys.stderr)
            return 1
        print(dump_frame(decode(first_raw), first_raw))
        return 0

    if not args.json:
        print(report(stats, parser))
        if args.csv:
            print(f"\nCSV 저장: {args.csv}  ({parser.n_ok} 행)")
        if args.record:
            print(f"원시 녹화: {args.record}  "
                  f"({os.path.getsize(args.record)} byte)  "
                  f"→ --replay {args.record} 로 재검증 가능")
    return 0


if __name__ == "__main__":
    sys.exit(main())
