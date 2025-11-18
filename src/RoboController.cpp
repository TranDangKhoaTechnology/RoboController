#include "RoboController.h"

// ===================== CTOR =====================
RoboController::RoboController()
: _uartMode(RC_UART_AUTO),
  _proto(RC_PROTO_ASCII),
  _s(nullptr),
#if ROBOCTRL_HAS_SOFT
  _soft(nullptr),
#endif
  _timeout(ROBOCTRL_DEFAULT_TIMEOUT),
  _lastRecv(0),
  _flags(0),
  _buttons(0),
  _joy1(0),
  _joy2(0)
{
  _lineLen = 0;
}

// ===================== begin() =====================
// Dùng Serial mặc định @115200
void RoboController::begin() {
  Serial.begin(ROBOCTRL_FIXED_BAUD);
  _s = &Serial;
  _uartMode = RC_UART_AUTO;
  _lastRecv = 0;
}

// ===================== beginHardware() =====================
// Chọn UART cứng theo số, ví dụ:
//   ESP32: 0=Serial, 1=Serial1, 2=Serial2
//   AVR Mega: 0..3
bool RoboController::beginHardware(uint8_t uartNo) {
  Stream *hs = getHWSerial(uartNo);
  if (!hs) return false;   // không có UART này

  // tùy kiến trúc mà ta gọi begin() được
#if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_AVR)
  ((HardwareSerial*)hs)->begin(ROBOCTRL_FIXED_BAUD);
#endif

  _s = hs;
  _uartMode = RC_UART_HARD;
  _lastRecv = 0;
  return true;
}

// ===================== beginSoftware() =====================
// Tạo đối tượng SoftwareSerial mới theo chân RX/TX
// Chỉ hoạt động nếu ROBOCTRL_HAS_SOFT = 1
bool RoboController::beginSoftware(int8_t rxPin, int8_t txPin) {
#if ROBOCTRL_HAS_SOFT
  if (rxPin < 0 || txPin < 0) return false;

  // nếu trước đó đã tạo rồi thì xóa đi để tạo lại
  if (_soft) {
    delete _soft;
    _soft = nullptr;
  }

  // tạo cổng mềm
  _soft = new SoftwareSerial(rxPin, txPin);
  if (!_soft) return false;

  _soft->begin(ROBOCTRL_FIXED_BAUD);
  _s = _soft;
  _uartMode = RC_UART_SOFT;
  _lastRecv = 0;
  return true;
#else
  // nếu nền tảng không hỗ trợ SoftSerial thì trả false
  (void)rxPin; (void)txPin;
  return false;
#endif
}

// ===================== beginStream() =====================
// Dùng 1 stream đã có sẵn (không tự begin baud nữa)
// -> Dùng cho các trường hợp như Serial2 đã được user .begin()
//    hoặc 1 lớp Stream khác (BLE, WiFi, RS485, ...)
void RoboController::beginStream(Stream &s) {
  _s = &s;
  _uartMode = RC_UART_AUTO;
  _lastRecv = 0;
}

// ===================== isConnected() =====================
// Nếu quá _timeout ms không nhận frame thì coi như mất kết nối
bool RoboController::isConnected() const {
  if (_lastRecv == 0) return false;  // chưa từng nhận
  uint32_t now = millis();
  return (now - _lastRecv) <= _timeout;
}

// ===================== poll() =====================
// Gọi liên tục trong loop(). Khi return true => có frame mới.
bool RoboController::poll() {
  if (!_s) return false;

  if (!readLine()) return false;

  // Có 1 dòng đầy đủ -> parse theo protocol
  bool ok = false;
  switch (_proto) {
    case RC_PROTO_ASCII:
    default:
      ok = parseAsciiLine(_lineBuf);
      break;
  }

  if (ok) {
    _lastRecv = millis();
  }
  return ok;
}

// ===================== readLine() =====================
// Đọc từng ký tự đến khi gặp '\n' hoặc '\r'
// Lưu trong _lineBuf, kết thúc bằng '\0'
// Return true khi có 1 dòng hoàn chỉnh.
bool RoboController::readLine() {
  if (!_s) return false;

  while (_s->available()) {
    char c = (char)_s->read();

    if (c == '\r') {
      // bỏ qua, chờ '\n'
      continue;
    }
    if (c == '\n') {
      // kết thúc dòng
      if (_lineLen == 0) {
        // dòng rỗng -> bỏ
        continue;
      }
      _lineBuf[_lineLen] = '\0';
      _lineLen = 0;
      return true;
    }

    // ký tự thường
    if (_lineLen < (BUF_LEN - 1)) {
      _lineBuf[_lineLen++] = c;
    } else {
      // tràn buffer -> reset
      _lineLen = 0;
    }
  }

  return false;
}

// ===================== parseAsciiLine() =====================
// Dòng dạng: "AA,BB,CC,DD,EE,FF,GG"
bool RoboController::parseAsciiLine(const char *line) {
  // tạo bản copy để dùng strtok
  char buf[BUF_LEN];
  strncpy(buf, line, BUF_LEN);
  buf[BUF_LEN - 1] = '\0';

  // tách 7 trường
  char *tok[7];
  uint8_t count = 0;

  char *p = strtok(buf, ",");
  while (p && count < 7) {
    tok[count++] = p;
    p = strtok(nullptr, ",");
  }

  if (count != 7) {
    return false;
  }

  uint8_t data[7];
  for (uint8_t i = 0; i < 7; ++i) {
    data[i] = parseHexByte(tok[i]);
  }

  // CRC8 trên 6 byte đầu
  uint8_t crc = crc8_0x07(data, 6);
  if (crc != data[6]) {
    return false;
  }

  _flags   = data[0];
  uint32_t b0 = data[1];
  uint32_t b1 = data[2];
  uint32_t b2 = data[3];
  _buttons = (b0) | (b1 << 8) | (b2 << 16);
  _joy1    = data[4];
  _joy2    = data[5];

  return true;
}

// ===================== parseHexByte() =====================
// chuyển "AF" -> 0xAF
static uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0;
}

uint8_t RoboController::parseHexByte(const char *p) {
  if (!p) return 0;
  uint8_t hi = hexVal(p[0]);
  uint8_t lo = hexVal(p[1]);
  return (hi << 4) | lo;
}

// ===================== crc8_0x07() =====================
// CRC8 với đa thức 0x07, init = 0x00, không xor out
uint8_t RoboController::crc8_0x07(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else            crc <<= 1;
    }
  }
  return crc;
}

// ===================== speedLevel() =====================
// Gợi ý: nếu L2/L3/L4 là các nút chọn tốc độ, thì:
uint8_t RoboController::speedLevel(uint8_t defaultIdx) const {
  // ví dụ: L2 -> level 1, L3 -> level 2, L4 -> level 3
  if (isL2()) return 1;
  if (isL3()) return 2;
  if (isL4()) return 3;
  return defaultIdx;
}

// ===================== printState() =====================
// In toàn bộ frame để debug
void RoboController::printState(Stream &out) const {
  out.print(F("[STATE] flags=0x"));
  if (_flags < 0x10) out.print('0');
  out.print(_flags, HEX);

  out.print(F(" btn=0x"));
  out.print(_buttons, HEX);

  out.print(F(" joy1="));
  out.print(_joy1);

  out.print(F(" joy2="));
  out.println(_joy2);
}

// ===================== getHWSerial() =====================
// Trả về HardwareSerial* tương ứng uartNo (tùy kiến trúc)
Stream* RoboController::getHWSerial(uint8_t uartNo) {
#if defined(ESP8266)
  // ESP8266 chỉ có Serial đọc được
  if (uartNo == 0) return &Serial;
  return nullptr;

#elif defined(ESP32)
  // ESP32 có 3 UART
  switch (uartNo) {
    case 0: return &Serial;
    case 1: return &Serial1;
    case 2: return &Serial2;
    default: return nullptr;
  }

#elif defined(ARDUINO_ARCH_AVR)
  // AVR: tùy board mà có bao nhiêu UART
  // UNO/Nano: chỉ có Serial
  // Mega: Serial, Serial1, Serial2, Serial3
  if (uartNo == 0) return &Serial;
  #if defined(HAVE_HWSERIAL1)
    if (uartNo == 1) return &Serial1;
  #endif
  #if defined(HAVE_HWSERIAL2)
    if (uartNo == 2) return &Serial2;
  #endif
  #if defined(HAVE_HWSERIAL3)
    if (uartNo == 3) return &Serial3;
  #endif
  return nullptr;

#else
  // Các nền tảng khác: cứ trả về Serial mặc định
  (void)uartNo;
  return &Serial;
#endif
}
