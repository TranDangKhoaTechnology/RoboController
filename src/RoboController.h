#pragma once
#include <Arduino.h>

/*
 * RoboController.h
 *
 * Mục đích:
 * - Nhận dữ liệu từ tay cầm / bộ điều khiển gửi qua UART ở dạng ASCII 7 trường:
 *     flags,b0,b1,b2,joy1,joy2,crc
 *   Trong đó:
 *     flags : 1 byte trạng thái (tùy ý, ví dụ bit báo kết nối, chế độ...)
 *     b0    : byte thấp của 24 nút
 *     b1    : byte giữa của 24 nút
 *     b2    : byte cao của 24 nút  (=> tổng 3 byte = 24 nút, từ bit 0..23)
 *     joy1  : giá trị joystick 1 (0..255)
 *     joy2  : giá trị joystick 2 (0..255)
 *     crc   : CRC8 đa thức 0x07 tính trên 6 byte đầu
 *
 * - Baud cố định: 115200
 * - Timeout mặc định: 200 ms (nếu lâu hơn không nhận được frame => coi như mất kết nối)
 *
 * 4 cách dùng cơ bản:
 *   RoboController rc;
 *
 *   1) rc.begin();
 *      -> dùng Serial mặc định (UART0) @115200
 *
 *   2) rc.beginHardware(0);
 *      -> chọn UART cứng theo số (Serial, Serial1, Serial2...)
 *
 *   3) rc.beginSoftware(rx, tx);
 *      -> dùng SoftwareSerial (chỉ có trên AVR/ESP8266, KHÔNG dùng trên ESP32)
 *
 *   4) rc.beginStream(myStream);
 *      -> đã có một Stream khác (ví dụ WiFiSerial, BLESerial, RS485...) thì đưa vào
 *
 *   5) rc.begin(Serial2);
 *      -> sugar cho beginStream(), tiện nhất trên ESP32: tự bạn cấu hình Serial2 trước.
 *
 * Trong loop():
 *   if (rc.poll()) {
 *     // Có frame mới -> dùng dữ liệu
 *     rc.printState(Serial);
 *   }
 *
 * Lưu ý:
 * - Thư viện KHÔNG quản lý nguồn nuôi / an toàn điện, chỉ đọc frame.
 * - Bạn tự quyết định logic điều khiển robot dựa trên dữ liệu đọc được.
 */

// ===== chọn SoftwareSerial theo chip =====
#if defined(ESP8266)
// ESP8266 core đã có EspSoftwareSerial tên là "SoftwareSerial"
  #include <SoftwareSerial.h>
  #define ROBOCTRL_HAS_SOFT 1

#elif defined(ESP32)
// ESP32: KHÔNG dùng SoftwareSerial trong thư viện này.
// Chỉ dùng UART cứng (Serial / Serial1 / Serial2) hoặc bất kỳ Stream nào bên ngoài.
  #define ROBOCTRL_HAS_SOFT 0

#elif defined(ARDUINO_ARCH_AVR)
// UNO/Nano/Mega: cho phép dùng SoftwareSerial
  #include <SoftwareSerial.h>
  #define ROBOCTRL_HAS_SOFT 1

#else
  // các nền tảng khác không hỗ trợ SoftSerial trong thư viện này
  #define ROBOCTRL_HAS_SOFT 0
#endif

// ===== chế độ UART đang dùng =====
enum {
  RC_UART_AUTO = 0,  // dùng Serial mặc định
  RC_UART_HARD = 1,  // dùng UART cứng khác: Serial1/2/3...
  RC_UART_SOFT = 2   // dùng SoftwareSerial (chỉ khi ROBOCTRL_HAS_SOFT = 1)
};

// ===== loại protocol (hiện mới có 1 loại) =====
enum {
  RC_PROTO_ASCII = 0,  // frame ASCII: "AA,BB,CC,DD,EE,FF,GG"
};

// cấu hình cố định
#define ROBOCTRL_FIXED_BAUD      115200UL
#define ROBOCTRL_DEFAULT_TIMEOUT 200

class RoboController {
public:
  // Khởi tạo đối tượng, chưa gắn Serial nào
  RoboController();

  // 1) dùng Serial mặc định @115200
  //    gọi trong setup()
  void begin();

  // 2) dùng UART cứng theo số
  //    ESP8266 chỉ có Serial (0); ESP32/AVR có thể có 1/2/3
  //    return false nếu không có UART đó
  bool beginHardware(uint8_t uartNo);

  // 3) dùng UART mềm (SoftwareSerial) với chân RX/TX chỉ định
  //    chỉ hoạt động nếu ROBOCTRL_HAS_SOFT = 1
  bool beginSoftware(int8_t rxPin, int8_t txPin);

  // 4) dùng stream bên ngoài đã có sẵn
  //    ví dụ: một lớp Serial tùy biến
  void beginStream(Stream &s);

  // 5) sugar: rc.begin(Serial2); // hoặc bất kỳ Stream nào đã begin sẵn
  void begin(Stream &s) { beginStream(s); }

  // đặt lại thời gian timeout mất kết nối
  void setTimeout(uint16_t ms) { _timeout = ms; }

  // đặt loại protocol (tương lai có thêm kiểu khác)
  void setProtocol(uint8_t proto) { _proto = proto; }

  // gọi trong loop() để xử lý UART
  // return true khi vừa nhận xong 1 frame hợp lệ
  bool poll();

  // có kết nối (không timeout)
  bool isConnected() const;

  // ===== truy cập dữ liệu hiện tại =====
  uint8_t  flags()   const { return _flags; }
  uint32_t buttons() const { return _buttons; } // 24 nút thấp (0..23)
  uint8_t  joy1()    const { return _joy1; }    // hướng/giá trị joystick 1 (0..255)
  uint8_t  joy2()    const { return _joy2; }    // joystick 2

  // đọc 1 nút theo index 0..23
  bool btn(uint8_t idx) const {
    if (idx >= 24) return false;
    return (_buttons >> idx) & 0x1u;
  }

  // alias cho btn()
  bool is(uint8_t idx)    const { return btn(idx); }
  bool isBtn(uint8_t idx) const { return btn(idx); }

  // ===== các nhóm nút đặt tên sẵn để code điều khiển dễ đọc hơn =====
  // nhóm L: 0..7
  bool isL1() const { return btn(0); }
  bool isL2() const { return btn(1); }
  bool isL3() const { return btn(2); }
  bool isL4() const { return btn(3); }
  bool isL5() const { return btn(4); }
  bool isL6() const { return btn(5); }
  bool isL7() const { return btn(6); }
  bool isL8() const { return btn(7); }

  // nhóm R: 8..15
  bool isR1() const { return btn(8); }
  bool isR2() const { return btn(9); }
  bool isR3() const { return btn(10); }
  bool isR4() const { return btn(11); }
  bool isR5() const { return btn(12); }
  bool isR6() const { return btn(13); }
  bool isR7() const { return btn(14); }
  bool isR8() const { return btn(15); }

  // joystick click (nếu tay cầm có): 16,17
  bool isJ1() const { return btn(16); }
  bool isJ2() const { return btn(17); }

  // hàm gợi ý: mức tốc độ dựa trên L2/L3/L4
  // trả về 0..3 (tùy bạn map sang tốc độ thật)
  uint8_t speedLevel(uint8_t defaultIdx) const;

  // debug: in toàn bộ frame ra Serial nào đó
  void printState(Stream &out) const;

private:
  uint8_t  _uartMode;
  uint8_t  _proto;
  Stream  *_s;
#if ROBOCTRL_HAS_SOFT
  SoftwareSerial *_soft;
#endif
  uint16_t _timeout;
  uint32_t _lastRecv;

  // dữ liệu hiện tại
  uint8_t  _flags;
  uint32_t _buttons;  // 24 bit thấp
  uint8_t  _joy1;
  uint8_t  _joy2;

  // buffer đọc 1 dòng ASCII
  static const uint8_t BUF_LEN = 64;
  char _lineBuf[BUF_LEN];
  uint8_t _lineLen;

  // đọc 1 dòng đầy đủ (kết thúc bằng \n hoặc \r\n)
  bool readLine();

  // parse 1 dòng kiểu "AA,BB,CC,DD,EE,FF,GG"
  bool parseAsciiLine(const char *line);

  // chuyển 2 ký tự hex thành 1 byte
  static uint8_t parseHexByte(const char *p);

  // CRC8 đa thức 0x07
  static uint8_t crc8_0x07(const uint8_t *data, uint8_t len);

  // lấy HardwareSerial tương ứng với số uart
  Stream* getHWSerial(uint8_t uartNo);
};
