#!/usr/bin/env python3
"""ST7789V 2inch (240x320) test for Jetson Orin Nano.
Wiring (BOARD, Seengreat wiki): DIN=19, SCK=23, CS=24 (/dev/spidev0.0), RST=15, DC=22, BL=18
"""
import os, sys, time, fcntl, struct
import numpy as np
import Jetson.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont

DC, RST, BL = 22, 15, 18
W, H = 240, 320                      # native portrait
SPI_DEV = "/dev/spidev0.0"
SPEED = int(sys.argv[1]) if len(sys.argv) > 1 else 20_000_000
CHUNK = 4096                          # spidev bufsiz

SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04

fd = os.open(SPI_DEV, os.O_RDWR)
fcntl.ioctl(fd, SPI_IOC_WR_MODE, struct.pack("B", 0))
fcntl.ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", SPEED))

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup([DC, RST, BL], GPIO.OUT, initial=GPIO.HIGH)

def write(data):
    for i in range(0, len(data), CHUNK):
        os.write(fd, data[i:i + CHUNK])

def cmd(c, *args):
    GPIO.output(DC, GPIO.LOW);  write(bytes([c]))
    if args:
        GPIO.output(DC, GPIO.HIGH); write(bytes(args))

def init():
    GPIO.output(RST, GPIO.HIGH); time.sleep(0.01)
    GPIO.output(RST, GPIO.LOW);  time.sleep(0.01)
    GPIO.output(RST, GPIO.HIGH); time.sleep(0.12)
    cmd(0x01); time.sleep(0.15)          # SWRESET
    cmd(0x11); time.sleep(0.12)          # SLPOUT
    cmd(0x36, 0x00)                      # MADCTL: portrait, RGB
    cmd(0x3A, 0x05)                      # COLMOD: 16bit RGB565
    cmd(0x21)                            # INVON (IPS panel)
    cmd(0x13)                            # NORON
    cmd(0x29); time.sleep(0.05)          # DISPON

def show(img):
    if img.size == (H, W):               # landscape image -> rotate
        img = img.rotate(90, expand=True)
    a = np.asarray(img.convert("RGB"), dtype=np.uint16)
    px = ((a[..., 0] & 0xF8) << 8) | ((a[..., 1] & 0xFC) << 3) | (a[..., 2] >> 3)
    cmd(0x2A, 0, 0, (W - 1) >> 8, (W - 1) & 0xFF)
    cmd(0x2B, 0, 0, (H - 1) >> 8, (H - 1) & 0xFF)
    cmd(0x2C)
    GPIO.output(DC, GPIO.HIGH)
    write(px.astype(">u2").tobytes())

def step(msg, t=1.5):
    print(msg, flush=True); time.sleep(t)

try:
    print(f"SPI {SPI_DEV} @ {SPEED/1e6:.0f} MHz")
    for i in range(3):                   # 1) backlight blink
        GPIO.output(BL, GPIO.LOW);  time.sleep(0.3)
        GPIO.output(BL, GPIO.HIGH); time.sleep(0.3)
    step("[1] 백라이트 3회 깜빡임 완료", 0.5)

    init()
    step("[2] 초기화 완료 (RST/DC/SPI)", 0.2)

    for name, c in [("RED", (255, 0, 0)), ("GREEN", (0, 255, 0)),
                    ("BLUE", (0, 0, 255)), ("WHITE", (255, 255, 255)),
                    ("BLACK", (0, 0, 0))]:
        t0 = time.time(); show(Image.new("RGB", (W, H), c))
        step(f"[3] 전체 {name} ({(time.time()-t0)*1000:.0f} ms)", 1.0)

    img = Image.new("RGB", (H, W), "black")   # landscape 320x240
    d = ImageDraw.Draw(img)
    bars = [(255,255,255),(255,255,0),(0,255,255),(0,255,0),
            (255,0,255),(255,0,0),(0,0,255),(0,0,0)]
    for i, c in enumerate(bars):
        d.rectangle([i * 40, 0, i * 40 + 39, 150], fill=c)
    d.rectangle([0, 0, H - 1, W - 1], outline=(255, 0, 0), width=2)
    try:
        f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
    except OSError:
        f = ImageFont.load_default()
    d.text((10, 160), "ST7789V 320x240", fill="white", font=f)
    d.text((10, 195), "Jetson Orin Nano OK", fill=(0, 255, 0), font=f)
    d.text((3, 3), "TL", fill=(255, 0, 0), font=f)
    show(img)
    step("[4] 컬러바 + 텍스트 표시 (좌상단 'TL')", 0)
finally:
    os.close(fd)
