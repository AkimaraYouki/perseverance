// Temporary GPS UART diagnostic for the ESP32-C3 hub (GPIO20 = RX from GPS TX, GPIO21 = TX to GPS RX).
// For each baud: dump what the GPS sends, then poll UBX-MON-VER and report whether it answers.
#include <Arduino.h>

static const uint32_t BAUDS[] = {115200, 38400, 9600, 57600, 230400};
static const uint8_t MON_VER_POLL[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

void dump(uint32_t ms, bool ascii) {
  const uint32_t t0 = millis();
  uint32_t n = 0, printable = 0;
  char line[161];
  uint8_t li = 0;
  bool ubx_hdr = false;
  uint8_t prev = 0;
  while (millis() - t0 < ms) {
    while (Serial1.available()) {
      const uint8_t b = Serial1.read();
      ++n;
      if (prev == 0xB5 && b == 0x62) ubx_hdr = true;
      prev = b;
      if ((b >= 32 && b < 127) || b == '\r' || b == '\n') ++printable;
      if (ascii && li < 160) line[li++] = (b >= 32 && b < 127) ? (char)b : '.';
    }
  }
  line[li] = 0;
  Serial.printf("  bytes=%lu printable=%lu%% ubx_sync_seen=%d\n", (unsigned long)n,
                (unsigned long)(n ? printable * 100 / n : 0), ubx_hdr);
  if (ascii && li) Serial.printf("  first: %s\n", line);
}

void setup() {
  Serial.begin(921600);
  delay(1500);
}

void loop() {
  Serial.println("=== GPS PROBE START ===");
  for (uint32_t baud : BAUDS) {
    Serial1.begin(baud, SERIAL_8N1, 20, 21);
    delay(50);
    while (Serial1.available()) Serial1.read();
    Serial.printf("baud %lu: listen 1.5 s\n", (unsigned long)baud);
    dump(1500, true);
    Serial1.write(MON_VER_POLL, sizeof(MON_VER_POLL));
    Serial1.flush();
    Serial.printf("baud %lu: after UBX-MON-VER poll (1 s)\n", (unsigned long)baud);
    dump(1000, true);
    Serial1.end();
  }
  Serial.println("=== GPS PROBE END ===");
  delay(3000);
}
