#include <RoboController.h>

RoboController rc;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("==== RoboController BasicRead ===="));

#if defined(ESP32)
  // ====== ESP32: DÙNG UART CỨNG (Serial2) ======
  // Chân dưới đây bạn TỰ ĐIỀU CHỈNH cho đúng phần cứng của bạn.
  // Ví dụ DevKit: RX2 = GPIO16, TX2 = GPIO17
  const int RX2_PIN = 16;
  const int TX2_PIN = 17;

  Serial.println(F("[INIT] ESP32: using Serial2 for controller"));
  Serial2.begin(ROBOCTRL_FIXED_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // Gắn thư viện vào Serial2 (đã begin sẵn)
  rc.begin(Serial2);

#elif defined(ESP8266)
  // ====== ESP8266: ưu tiên SoftwareSerial D7(DTR) / D8(DRX) ======
  Serial.println(F("[INIT] ESP8266: try SoftwareSerial D7(D8)"));
  if (!rc.beginSoftware(D7, D8)) {
    Serial.println(F("[WARN] SoftSerial FAIL, fallback to Serial"));
    rc.begin();
  }

#else
  // ====== AVR / các board khác: thử SoftwareSerial chân 7/8, nếu fail dùng Serial ======
  Serial.println(F("[INIT] Other MCU: try SoftwareSerial 7/8"));
  if (!rc.beginSoftware(7, 8)) {
    Serial.println(F("[WARN] SoftSerial FAIL, fallback to Serial"));
    rc.begin();
  }
#endif

  rc.setTimeout(200);
  Serial.println(F("[READY] RoboController example - FULL 24 buttons (one line)"));
}

void loop() {
  // Khi poll() trả về true => vừa có frame mới
  if (rc.poll()) {
    // tạo 1 dòng để gom hết
    String line;

    // ===== JOYSTICK =====
    line += F("[JOY] j1=");
    line += rc.joy1();
    line += F(" j2=");
    line += rc.joy2();

    // ===== NÚT TRÁI (0..7) =====
    if (rc.isL1()) line += F(" | L1");
    if (rc.isL2()) line += F(" | L2");
    if (rc.isL3()) line += F(" | L3");
    if (rc.isL4()) line += F(" | L4");
    if (rc.isL5()) line += F(" | L5");
    if (rc.isL6()) line += F(" | L6");
    if (rc.isL7()) line += F(" | L7");
    if (rc.isL8()) line += F(" | L8");

    // ===== NÚT PHẢI (8..15) =====
    if (rc.isR1()) line += F(" | R1");
    if (rc.isR2()) line += F(" | R2");
    if (rc.isR3()) line += F(" | R3");
    if (rc.isR4()) line += F(" | R4");
    if (rc.isR5()) line += F(" | R5");
    if (rc.isR6()) line += F(" | R6");
    if (rc.isR7()) line += F(" | R7");
    if (rc.isR8()) line += F(" | R8");

    // ===== JOYSTICK CLICK (16,17) =====
    if (rc.isJ1()) line += F(" | J1");
    if (rc.isJ2()) line += F(" | J2");

    // ===== NÚT CÒN LẠI (18..23) =====
    for (uint8_t i = 18; i < 24; ++i) {
      if (rc.is(i)) {
        line += F(" | B");
        line += i;
      }
    }

    // in CẢ dòng 1 lần
    Serial.println(line);
  }

  // mất tay cầm
  if (!rc.isConnected()) {
    // ở đây có thể dừng robot / chớp LED, hiện tại chỉ in ra
    Serial.println(F("Controller disconnected"));
    delay(200);
  }
}
