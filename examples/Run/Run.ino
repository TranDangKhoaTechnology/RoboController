#include <Arduino.h>
#include <RoboController.h>

RoboController rc;

// ================== CẤU HÌNH CHÂN MOTOR (L298N) ==================
// theo đúng mapping bạn đưa
#define ENA 5     // PWM cho motor A (bánh trái)
#define IN1 18
#define IN2 19

#define IN3 21
#define IN4 22
#define ENB 23    // PWM cho motor B (bánh phải)

// Đổi DIR_A / DIR_B = -1 nếu motor quay ngược
const int DIR_A = +1;
const int DIR_B = +1;

// ================== TỐC ĐỘ & TRẠNG THÁI ==================
// 0: dừng, 1: chậm, 2: TB, 3: nhanh
int speedBase[] = { 0, 80, 120, 180 };
const int SPEED_COUNT = sizeof(speedBase) / sizeof(speedBase[0]);

int  speedIndex   = 2;      // tốc độ mặc định (mức 2)
bool driveLocked  = false;  // true => khóa chuyển động

// lưu trạng thái nút lần trước để bắt cạnh
bool prevL5 = false;  // tăng tốc
bool prevL6 = false;  // giảm tốc
bool prevR1 = false;  // khóa/mở khóa

// ================== PWM (analogWrite cho mọi board) ==================
void pwmWriteA(int val) {
  val = constrain(val, 0, 255);
  analogWrite(ENA, val);
}

void pwmWriteB(int val) {
  val = constrain(val, 0, 255);
  analogWrite(ENB, val);
}

// ================== MOTOR ==================
void motorA(int spd) {
  spd *= DIR_A;

  if (spd > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    pwmWriteA(spd);
  } else if (spd < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    pwmWriteA(-spd);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    pwmWriteA(0);
  }
}

void motorB(int spd) {
  spd *= DIR_B;

  if (spd > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    pwmWriteB(spd);
  } else if (spd < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    pwmWriteB(-spd);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    pwmWriteB(0);
  }
}

void stopAll() {
  motorA(0);
  motorB(0);
}

// ================== NÚT TỐC ĐỘ & LOCK ==================
void handleSpeedButtons() {
  bool curL5 = rc.isL5();   // tăng tốc
  bool curL6 = rc.isL6();   // giảm tốc
  bool curR1 = rc.isR1();   // khóa/mở khóa

  // bắt cạnh lên
  if (!prevL5 && curL5) {
    if (speedIndex < SPEED_COUNT - 1) speedIndex++;
    Serial.print("[SPEED] ++ -> ");
    Serial.println(speedBase[speedIndex]);
  }
  if (!prevL6 && curL6) {
    if (speedIndex > 0) speedIndex--;
    Serial.print("[SPEED] -- -> ");
    Serial.println(speedBase[speedIndex]);
  }
  if (!prevR1 && curR1) {
    driveLocked = !driveLocked;
    Serial.print("[LOCK] = ");
    Serial.println(driveLocked ? "ON" : "OFF");
  }

  prevL5 = curL5;
  prevL6 = curL6;
  prevR1 = curR1;
}

// ================== GIẢI MÃ HƯỚNG TỪ 1 JOY ==================
//
// joy code:
//   0 = giữa
//   1 = lên
//   2 = phải
//   3 = xuống
//   4 = trái
//
// Mỗi joy đóng góp 1 vector (v,h):
//   v: +1 (tiến), -1 (lùi), 0 (không)
//   h: +1 (phải), -1 (trái), 0 (không)
//
void joyContribution(uint8_t code, int &v, int &h) {
  v = 0;
  h = 0;
  switch (code) {
    case 1: v = +1; break;   // lên -> tiến
    case 3: v = -1; break;   // xuống -> lùi
    case 2: h = +1; break;   // phải
    case 4: h = -1; break;   // trái
    default: break;          // 0 hoặc khác -> không
  }
}

// ================== ĐIỀU KHIỂN BẰNG 2 JOY (GIỐNG NHAU) ==================
//
// - Lấy hướng của cả 2 joy, cộng vector:
//     v_total = v1 + v2  (trong [-2..+2])
//     h_total = h1 + h2  (trong [-2..+2])
// - Chuẩn hóa lại về -1/0/+1 để lái mềm.
// - V=0, H=0: dừng.
// - V!=0, H=0: đi thẳng / lùi.
// - V=0, H!=0: quay tại chỗ.
// - V!=0, H!=0: đi chéo / rẽ:
//     + có thành phần tiến + lệch => rẽ tiến
//     + có thành phần lùi + lệch => rẽ lùi.
//
void controlMoveCombinedJoy() {
  if (driveLocked) {
    stopAll();
    return;
  }

  int baseSpeed = speedBase[speedIndex];
  if (baseSpeed <= 0) {
    stopAll();
    return;
  }

  // đọc 2 joy từ thư viện
  uint8_t j1 = rc.joy1();  // 0..4
  uint8_t j2 = rc.joy2();  // 0..4

  int v1, h1, v2, h2;
  joyContribution(j1, v1, h1);
  joyContribution(j2, v2, h2);

  int v_total = v1 + v2;   // -2..+2
  int h_total = h1 + h2;   // -2..+2

  // chuẩn hóa về -1/0/+1 (nếu cả 2 cùng hướng thì vẫn coi là 1, chỉ mạnh hơn tí)
  if (v_total > 0) v_total = +1;
  else if (v_total < 0) v_total = -1;

  if (h_total > 0) h_total = +1;
  else if (h_total < 0) h_total = -1;

  // nếu không có input -> dừng
  if (v_total == 0 && h_total == 0) {
    stopAll();
    return;
  }

  int leftPWM  = 0;
  int rightPWM = 0;

  // CHỈ tiến/lùi (không rẽ)
  if (v_total != 0 && h_total == 0) {
    leftPWM  =  v_total * baseSpeed;
    rightPWM =  v_total * baseSpeed;
  }
  // CHỈ quay (không tiến/lùi)
  else if (v_total == 0 && h_total != 0) {
    // quay tại chỗ: trái +, phải -
    leftPWM  =  h_total * baseSpeed;
    rightPWM = -h_total * baseSpeed;
  }
  // VỪA tiến/lùi VỪA lệch -> rẽ / đi chéo
  else {
    // ví dụ: 1 joy tiến + 1 joy trái/phải -> v_total=1, h_total=±1
    // mình cho 1 bánh nhanh, 1 bánh chậm để nó rẽ cong.
    int fast =  v_total * baseSpeed;
    int slow =  v_total * (baseSpeed * 50 / 100); // ~0.5 * baseSpeed

    if (h_total < 0) {
      // lệch TRÁI: bánh trái chậm, bánh phải nhanh
      leftPWM  = slow;
      rightPWM = fast;
    } else {
      // lệch PHẢI: bánh trái nhanh, bánh phải chậm
      leftPWM  = fast;
      rightPWM = slow;
    }
  }

  motorA(leftPWM);
  motorB(rightPWM);

  // debug cho dễ tune
  Serial.print("[JOY] J1=");
  Serial.print(j1);
  Serial.print(" J2=");
  Serial.print(j2);
  Serial.print(" | v1=");
  Serial.print(v1);
  Serial.print(" h1=");
  Serial.print(h1);
  Serial.print(" v2=");
  Serial.print(v2);
  Serial.print(" h2=");
  Serial.print(h2);
  Serial.print(" | vT=");
  Serial.print(v_total);
  Serial.print(" hT=");
  Serial.print(h_total);
  Serial.print(" | L=");
  Serial.print(leftPWM);
  Serial.print(" R=");
  Serial.println(rightPWM);
}

// ================== CHỌN UART ĐỌC TAY CẦM ==================
// 1: dùng Serial2 (ESP32 RX2/TX2)
// 0: dùng Serial (USB)
#define RC_USE_SERIAL2 1

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==== Run.ino – 2 JOY giống nhau, cộng vector, có rẽ/chéo ====");

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopAll();

#if RC_USE_SERIAL2 && defined(ESP32)
  // Nếu farm / tay cầm nối vào UART2
  const int RX2_PIN = 16;
  const int TX2_PIN = 17;
  Serial.println("[INIT] ESP32: dùng Serial2 đọc controller");
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

  rc.begin(Serial2);   // dùng hàm begin(Stream&) trong thư viện

#else
  // Nếu farm gửi thẳng vào USB (Serial)
  Serial.println("[INIT] Dùng Serial đọc controller");
  rc.begin(Serial);
#endif

  rc.setTimeout(200);
  Serial.println("[READY] 2 joy điều khiển, nút L5/L6 tốc, R1 lock.");
}

// ================== LOOP ==================
void loop() {
  // log cho biết loop vẫn chạy
  static uint32_t lastPing = 0;
  if (millis() - lastPing > 1000) {
    lastPing = millis();
    Serial.println("[LOOP] alive");
  }

  // Chỉ dùng hàm thư viện:
  //   rc.poll(), rc.joy1(), rc.joy2(), rc.isL5(), rc.isL6(), rc.isR1()
  if (rc.poll()) {
    handleSpeedButtons();
    controlMoveCombinedJoy();
  }

  if (!rc.isConnected()) {
    stopAll();
  }
}
