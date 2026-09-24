"""ST7789V 240x320 SPI driver (spidev ioctl + Jetson.GPIO), no extra pip packages.

Requires the 40-pin pinmux from lcd-pinmux.service (SPI1 SFIO + DC/RST/BL tristate off).
"""
import fcntl
import os
import struct
import time

import numpy as np

SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
CHUNK = 4096  # spidev bufsiz


class ST7789:
    WIDTH, HEIGHT = 240, 320  # native portrait

    def __init__(self, spi_dev='/dev/spidev0.0', speed_hz=20_000_000,
                 dc=22, rst=15, bl=18, invert=True, bgr=False, rotate_180=False):
        import Jetson.GPIO as GPIO
        self._gpio = GPIO
        self.dc, self.rst, self.bl = dc, rst, bl
        self.flip = rotate_180
        self.fd = os.open(spi_dev, os.O_RDWR)
        fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack('B', 0))
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack('I', speed_hz))
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([dc, rst, bl], GPIO.OUT, initial=GPIO.HIGH)
        self._init(invert, bgr)

    def _write(self, data):
        for i in range(0, len(data), CHUNK):
            os.write(self.fd, data[i:i + CHUNK])

    def _cmd(self, c, *args):
        self._gpio.output(self.dc, self._gpio.LOW)
        self._write(bytes([c]))
        if args:
            self._gpio.output(self.dc, self._gpio.HIGH)
            self._write(bytes(args))

    def _init(self, invert, bgr):
        g = self._gpio
        g.output(self.rst, g.LOW)
        time.sleep(0.01)
        g.output(self.rst, g.HIGH)
        time.sleep(0.12)
        self._cmd(0x01)
        time.sleep(0.15)                      # SWRESET
        self._cmd(0x11)
        time.sleep(0.12)                      # SLPOUT
        self._cmd(0x36, 0x08 if bgr else 0x00)  # MADCTL
        self._cmd(0x3A, 0x05)                 # RGB565
        self._cmd(0x21 if invert else 0x20)   # IPS panel needs INVON
        self._cmd(0x13)
        self._cmd(0x29)
        time.sleep(0.05)

    def backlight(self, on):
        self._gpio.output(self.bl, self._gpio.HIGH if on else self._gpio.LOW)

    def show(self, img):
        """img: PIL image, 240x320 or 320x240 (landscape is rotated)."""
        if img.size == (self.HEIGHT, self.WIDTH):
            img = img.rotate(-90 if self.flip else 90, expand=True)
        a = np.asarray(img.convert('RGB'), dtype=np.uint16)
        px = ((a[..., 0] & 0xF8) << 8) | ((a[..., 1] & 0xFC) << 3) | (a[..., 2] >> 3)
        w, h = self.WIDTH - 1, self.HEIGHT - 1
        self._cmd(0x2A, 0, 0, w >> 8, w & 0xFF)
        self._cmd(0x2B, 0, 0, h >> 8, h & 0xFF)
        self._cmd(0x2C)
        self._gpio.output(self.dc, self._gpio.HIGH)
        self._write(px.astype('>u2').tobytes())

    def show_region(self, img, box):
        """Update only box=(x0, y0, x1, y1) (x1/y1 exclusive) of a 320x240 landscape image.

        show() rotates landscape 90° CCW onto the 240x320 panel, so landscape (x, y) lands on
        panel column y, row (319 - x). The cropped region is rotated the same way and written
        into the matching panel window — roughly (area / 76800) of a full-frame write.
        """
        x0, y0, x1, y1 = box
        region = img.crop(box).rotate(-90 if self.flip else 90, expand=True)
        a = np.asarray(region.convert('RGB'), dtype=np.uint16)
        px = ((a[..., 0] & 0xF8) << 8) | ((a[..., 1] & 0xFC) << 3) | (a[..., 2] >> 3)
        if self.flip:   # rotate(-90): landscape (x, y) -> panel column 239 - y, row x
            c0, c1 = self.WIDTH - y1, self.WIDTH - 1 - y0
            r0, r1 = x0, x1 - 1
        else:           # rotate(+90): landscape (x, y) -> panel column y, row 319 - x
            c0, c1 = y0, y1 - 1
            r0, r1 = self.HEIGHT - x1, self.HEIGHT - 1 - x0
        self._cmd(0x2A, c0 >> 8, c0 & 0xFF, c1 >> 8, c1 & 0xFF)
        self._cmd(0x2B, r0 >> 8, r0 & 0xFF, r1 >> 8, r1 & 0xFF)
        self._cmd(0x2C)
        self._gpio.output(self.dc, self._gpio.HIGH)
        self._write(px.astype('>u2').tobytes())

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass
