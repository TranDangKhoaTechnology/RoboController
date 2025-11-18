# 🚀 RoboController – UART Gamepad Frame Parser

Read controller frame over UART (115200 baud). Parse ASCII format:
`flags,b0,b1,b2,joy1,joy2,crc` with CRC8 check.

## 📌 Features
- Read **24 buttons** + **2 joysticks** (0–255)
- CRC8 (0x07) validation (data error check)
- **Connection timeout**: auto detect disconnect if no frame in N ms
- Supports ALL Arduino chips:
  | Chip | HardwareSerial | SoftwareSerial |
  |------|----------------|----------------|
  | AVR (UNO/Mega) | ✔ | ✔ |
  | ESP8266 | ✔ | ✔ |
  | ESP32 | ✔ | ❌ |

Frame is sent as 7 ASCII hex fields separated by commas:
```text
flags,b0,b1,b2,joy1,joy2,crc
```

---

## 📦 Installation

Put files in a folder named `RoboController` inside your Arduino `libraries` directory:

```text
RoboController/
 ├── RoboController.h
 ├── RoboController.cpp
 ├── library.properties
 ├── LICENSE
 ├── README.md   ← this file
 └── examples/
      ├── BasicRead/BasicRead.ino
      └── Run/Run.ino
```

Restart Arduino IDE → Library will appear as **RoboController**.

---

## 🧪 Basic Usage

```cpp
#include <RoboController.h>

RoboController rc;

void setup() {
  Serial.begin(115200);

  // 1) simplest: use default Serial (UART0)
  rc.begin();

  // optional: custom timeout (ms)
  rc.setTimeout(300); // default 200 ms
}

void loop() {
  // Call poll() as often as possible
  if (rc.poll()) {
    // A valid frame has just been received
    rc.printState(Serial); // debug: print flags, buttons, joy1, joy2

    if (rc.isL1()) {
      Serial.println("L1 pressed");
    }
  }

  // You can also check connection state
  if (!rc.isConnected()) {
    // no frame in timeout window (e.g. 200 ms)
    // → treat as disconnected
    // Serial.println("Controller disconnected!");
  }
}
```

---

## 🌐 UART Setup Modes

Library supports **4 ways** to attach a serial interface:

```cpp
#include <RoboController.h>
RoboController rc;
```

### 1️⃣ Use default Serial (UART0)
```cpp
void setup() {
  Serial.begin(115200);
  rc.begin();       // use Serial as source
}
```

### 2️⃣ Use hardware UART by index
```cpp
// AVR Mega: 0=Serial, 1=Serial1, 2=Serial2, 3=Serial3
// ESP32:    0=Serial, 1=Serial1, 2=Serial2

bool ok = rc.beginHardware(3); // use Serial3 on MEGA
if (!ok) {
  // invalid UART index
}
```

### 3️⃣ Use SoftwareSerial (AVR / ESP8266 only)
```cpp
// NOT available on ESP32
rc.beginSoftware(8, 9); // RX=8, TX=9
```

### 4️⃣ Use any existing Stream
```cpp
// You configure Serial2 or any custom Stream:
Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
rc.begin(Serial2);  // sugar for beginStream(Serial2);
```

---

## 📤 Frame Format (from sender side)

Data is sent as ASCII hex:

```text
flags,b0,b1,b2,joy1,joy2,crc
```

Example:
```text
01,00,01,00,7F,20,A5
```

- `flags` : 1 byte status
- `b0,b1,b2` : 3 bytes = 24 buttons (bit 0..23)
- `joy1` : joystick 1 (0..255)
- `joy2` : joystick 2 (0..255)
- `crc` : CRC8(0x07) of first 6 bytes

Library takes care of parsing and CRC checking internally.

---

## 🔘 Buttons API

```cpp
// Direct bit index (0..23)
bool b0  = rc.btn(0);
bool b5  = rc.btn(5);
bool b23 = rc.btn(23);

// Aliases
bool b7  = rc.is(7);
bool b10 = rc.isBtn(10);

// Named helpers
if (rc.isL1()) { /* left group button 1  (bit 0)  */ }
if (rc.isL2()) { /* left group button 2  (bit 1)  */ }
if (rc.isL8()) { /* left group button 8  (bit 7)  */ }

if (rc.isR1()) { /* right group button 1 (bit 8)  */ }
if (rc.isR4()) { /* right group button 4 (bit 11) */ }
if (rc.isR8()) { /* right group button 8 (bit 15) */ }

// Joystick-click buttons (if controller has them)
if (rc.isJ1()) { /* joystick 1 click (bit 16) */ }
if (rc.isJ2()) { /* joystick 2 click (bit 17) */ }

// Use L2/L3/L4 as speed selector
uint8_t level = rc.speedLevel(1); // e.g. 0..3, your mapping
```

You can also read raw flags + button bit mask:

```cpp
uint8_t  f  = rc.flags();    // raw flags byte
uint32_t bm = rc.buttons();  // 24-bit mask in lower bits
```

---

## 🕹️ Joystick API (joy1 / joy2)

Library returns **raw 0–255** values for each joystick:

```cpp
uint8_t j1 = rc.joy1();  // joystick 1 value
uint8_t j2 = rc.joy2();  // joystick 2 value
```

### 1️⃣ Digital-style joystick
App sends discrete values:
- `0` = center
- `1` = up
- `2` = right
- `3` = down
- `4` = left

```cpp
void handleJoy1(uint8_t j1) {
  switch (j1) {
    case 0:  stopRobot();          break; // center
    case 1:  moveForward();        break; // up
    case 2:  moveRight();          break; // right
    case 3:  moveBackward();       break; // down
    case 4:  moveLeft();           break; // left
    default: /* other codes */     break;
  }
}

void loop() {
  if (rc.poll()) {
    handleJoy1(rc.joy1());
  }
}
```

### 2️⃣ Analog-style joystick 0–255 (with deadzone)

```cpp
void handleJoyAnalog() {
  const uint8_t CENTER = 128;
  const uint8_t DEAD   = 20;   // deadzone

  int j1 = rc.joy1();          // 0..255
  int dx = j1 - CENTER;        // -128..+127

  if (dx > DEAD) {
    // joystick 1 → right
    moveRightWithSpeed(map(dx, 0, 127, 0, 255));
  } else if (dx < -DEAD) {
    // joystick 1 → left
    moveLeftWithSpeed(map(-dx, 0, 128, 0, 255));
  } else {
    // inside deadzone
    stopSideMotion();
  }
}
```

### 3️⃣ Two-joystick robot control (tank / mecanum)

Example mapping:
- `joy1` controls **forward/backward** (Y axis)
- `joy2` controls **left/right turn** (X axis)

```cpp
void handleTwoJoy() {
  const uint8_t CENTER = 128;
  const uint8_t DEAD   = 15;

  int y = rc.joy1() - CENTER; // forward/back
  int x = rc.joy2() - CENTER; // left/right turn

  // apply deadzone
  if (abs(y) < DEAD) y = 0;
  if (abs(x) < DEAD) x = 0;

  // convert to motor speed (-255..255)
  int vF = map(y, -128, 127, -255, 255); // forward/back speed
  int vS = map(x, -128, 127, -255, 255); // turn speed

  // Example for 2-wheel tank robot
  int left  = vF - vS;
  int right = vF + vS;

  setLeftMotor(left);
  setRightMotor(right);
}
```

You can change mixing logic to fit **Mecanum / Omni / 4-wheel** robots.

---

## 🔌 Connection State & Timeout

The library tracks last time a valid frame was received:

```cpp
// Set timeout (in ms)
rc.setTimeout(200); // default 200 ms

// Check connection
if (rc.isConnected()) {
  // recently received frame
} else {
  // no frame within timeout → treat as disconnected
}
```

Typical usage pattern:

```cpp
void loop() {
  rc.poll(); // always call

  if (!rc.isConnected()) {
    // stop robot for safety
    stopAllMotors();
    return;
  }

  // Safe: controller is alive
  if (rc.isL1()) forward();
  if (rc.isL2()) backward();
}
```

---

## ⚙️ Protocol Selection

Currently library supports **1 protocol**: ASCII frame.

```cpp
rc.setProtocol(RC_PROTO_ASCII);  // default
```

In future you can extend to other protocols without changing user API.

---

## 📚 Public API Summary

Everything you can use from this library:

### Constructors
```cpp
RoboController rc;
```

### Attach serial / stream
```cpp
void begin();                    // use default Serial
bool beginHardware(uint8_t id);  // Serial / Serial1 / Serial2 / Serial3...
bool beginSoftware(int8_t rx, int8_t tx); // AVR/ESP8266 only
void beginStream(Stream &s);     // generic stream
void begin(Stream &s);           // sugar for beginStream()
```

### Runtime / protocol
```cpp
bool     poll();                 // call often, returns true when a valid frame is parsed
bool     isConnected() const;    // connection alive (no timeout)
void     setTimeout(uint16_t ms);
void     setProtocol(uint8_t proto); // currently RC_PROTO_ASCII
```

### Data access
```cpp
uint8_t  flags()   const;
uint32_t buttons() const;        // 24-bit mask
uint8_t  joy1()    const;
uint8_t  joy2()    const;

bool     btn(uint8_t idx) const; // 0..23
bool     is(uint8_t idx)  const;
bool     isBtn(uint8_t idx) const;
```

### Named buttons
```cpp
// Left group (0..7)
bool isL1() const; bool isL2() const; bool isL3() const; bool isL4() const;
bool isL5() const; bool isL6() const; bool isL7() const; bool isL8() const;

// Right group (8..15)
bool isR1() const; bool isR2() const; bool isR3() const; bool isR4() const;
bool isR5() const; bool isR6() const; bool isR7() const; bool isR8() const;

// Joystick click (16,17)
bool isJ1() const; bool isJ2() const;
```

### Helpers
```cpp
uint8_t speedLevel(uint8_t defaultIdx) const; // map L2/L3/L4 to speed level
void    printState(Stream &out) const;        // debug print
```

---

## 📌 Using on ESP32 (example)

```cpp
#include <RoboController.h>

RoboController rc;

void setup() {
  Serial.begin(115200);

  const int RX_PIN = 16;
  const int TX_PIN = 17;
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  rc.begin(Serial2); // controller on Serial2
  rc.setTimeout(300);
}

void loop() {
  if (rc.poll()) {
    rc.printState(Serial);
  }

  if (!rc.isConnected()) {
    stopAllMotors();
    return;
  }

  // your control logic here...
}
```

---

Made by **TranDangKhoaTechnology**  
GitHub: https://github.com/TranDangKhoaTechnology  
Email: trandangkhoa31122006@gmail.com
