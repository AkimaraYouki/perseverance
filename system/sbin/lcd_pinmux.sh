#!/bin/sh
# 40-pin pinmux for ST7789 LCD on Jetson Orin Nano (resets on reboot)
# SPI1 (SFIO, output enabled): SCK pin23, MOSI pin19, CS0 pin24
for a in 0x0243d028 0x0243d040 0x0243d008; do busybox devmem $a 32 0x400; done
# GPIO (tristate off, input enabled): RST pin15 PN.01, DC pin22 PY.01, BL pin18 PY.03
for a in 0x02440020 0x0243d000 0x0243d010; do busybox devmem $a 32 0x40; done
for a in 0x0243d028 0x0243d040 0x0243d008 0x02440020 0x0243d000 0x0243d010; do echo "$a = $(busybox devmem $a 32)"; done
