/*
  Robot Mecanum 4WD
  ESP32-C6 + 4x BTS7960 (chip kustom) + 4x motor DC encoder (chip kustom)

  Input (simulasi Wokwi): Serial Monitor, karena Wokwi tidak mensimulasikan
  Bluetooth dan ESP32-C6 tidak punya Bluetooth Classic (dibutuhkan PS4).

    w / s   : maju / mundur
    a / d   : geser kiri / kanan (strafe)
    q / e   : putar kiri / kanan
    x       : berhenti
    j LX LY RX : mode analog seperti stick, nilai -1..1
                 contoh:  j 0.5 1 0   (maju sambil geser kanan)

  Pemetaan stick PS4 (untuk robot asli, lihat bagian USE_PS4):
    Stick kiri Y = maju/mundur, stick kiri X = geser, stick kanan X = putar
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== IMU BNO055 =====================
#define IMU_SDA 12
#define IMU_SCL 13

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool imuReady = false;

float heading = 0;
float roll = 0;
float pitch = 0;

const uint32_t IMU_MS = 100;

// #define USE_PS4   // aktifkan HANYA di ESP32 klasik (bukan C6), lihat catatan di bawah

// ===================== PIN =====================
struct WheelPins { uint8_t rpwm, lpwm, enca, encb; };

const WheelPins PINS[4] = {
  { 0,  1,  2,  3},   // 0 = FL (depan kiri)
  { 6,  7, 10, 11},   // 1 = FR (depan kanan)
  {18, 19, 20, 21},   // 2 = RL (belakang kiri)
  {22, 23,  4,  5},   // 3 = RR (belakang kanan)
};
const char *NAMES[4] = {"FL", "FR", "RL", "RR"};

// ===================== PARAMETER =====================
const float   MAX_RPM    = 200.0f;   // sama dengan atribut maxRpm chip motor
const float   PPR        = 11.0f;    // sama dengan atribut ppr chip motor
const float   CPR        = PPR * 4;  // dekode x4
const int8_t  MOTOR_DIR[4] = {1, 1, 1, 1};  // ubah ke -1 jika motor terpasang terbalik (robot asli)

const int     PWM_FREQ   = 1000;     // 1 kHz agar simulasi ringan. Robot asli: 10-20 kHz
const int     PWM_BITS   = 8;
const int     PWM_MAX    = 255;

const uint32_t CTRL_MS   = 50;       // periode kontrol kecepatan
const float   KP         = 0.30f;    // PWM per rpm
const float   KI         = 1.00f;    // PWM per (rpm*detik)
#define USE_PID 1                    // 0 = open-loop (feedforward saja)

const float   DEADZONE   = 0.08f;
const float   SERIAL_SPEED = 0.6f;   // kecepatan untuk perintah w/a/s/d/q/e

// ===================== ENCODER (kuadratur x4) =====================
struct Encoder {
  uint8_t a, b;
  volatile long count;
  volatile uint8_t last;
};
Encoder enc[4];

// Urutan maju: 00 -> 10 -> 11 -> 01 (A mendahului B), state = (A<<1)|B
const int8_t QTAB[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encISR(void *arg) {
  Encoder *e = (Encoder *)arg;
  uint8_t s = (digitalRead(e->a) << 1) | digitalRead(e->b);
  e->count += QTAB[(e->last << 2) | s];
  e->last = s;
}

// ===================== MOTOR (BTS7960) =====================
// ESP32-C6 hanya punya 6 kanal LEDC, sedangkan 4 driver butuh 8 pin PWM.
// Solusi: hanya pin arah yang aktif (RPWM atau LPWM) yang dipasangi PWM;
// pin satunya dijadikan LOW digital.
struct Wheel {
  int8_t dir;        // -1, 0, +1
  float  target;     // rpm (ruang motor)
  float  meas;       // rpm terukur (difilter)
  float  integ;      // integral PI (satuan PWM)
  long   lastCount;
};
Wheel wheel[4];

void releasePin(uint8_t p) {
  ledcDetach(p);
  pinMode(p, OUTPUT);
  digitalWrite(p, LOW);
}

void setMotor(int i, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  int dir = (pwm > 0) - (pwm < 0);

  if (dir != wheel[i].dir) {
    if (wheel[i].dir ==  1) releasePin(PINS[i].rpwm);
    if (wheel[i].dir == -1) releasePin(PINS[i].lpwm);
    if (dir ==  1) ledcAttach(PINS[i].rpwm, PWM_FREQ, PWM_BITS);
    if (dir == -1) ledcAttach(PINS[i].lpwm, PWM_FREQ, PWM_BITS);
    wheel[i].dir = dir;
  }
  if (dir ==  1) ledcWrite(PINS[i].rpwm,  pwm);
  if (dir == -1) ledcWrite(PINS[i].lpwm, -pwm);
}

// ===================== KINEMATIKA MECANUM =====================
// vx: geser kanan (+), vy: maju (+), wz: putar searah jarum jam (+)
// Konfigurasi roller "X" standar. Hasil dinormalisasi ke -1..1.
void mecanum(float vx, float vy, float wz, float out[4]) {
  out[0] = vy + vx + wz;   // FL
  out[1] = vy - vx - wz;   // FR
  out[2] = vy - vx + wz;   // RL
  out[3] = vy + vx - wz;   // RR

  float m = 0;
  for (int i = 0; i < 4; i++) if (fabsf(out[i]) > m) m = fabsf(out[i]);
  if (m > 1.0f) for (int i = 0; i < 4; i++) out[i] /= m;
}

// ===================== INPUT =====================
struct Stick { float lx, ly, rx; } in = {0, 0, 0};

float shape(float v) {                       // deadzone + skala ulang
  if (fabsf(v) < DEADZONE) return 0;
  float s = (fabsf(v) - DEADZONE) / (1.0f - DEADZONE);
  return (v < 0 ? -s : s);
}

#ifdef USE_PS4
// Hanya untuk ESP32 klasik. Pasang library "PS4Controller" dan pairing MAC
// lewat SixaxisPairTool. Periksa kompatibilitas dengan versi core Arduino-ESP32 Anda.
#include <PS4Controller.h>
bool readPS4() {
  if (!PS4.isConnected()) { in = {0, 0, 0}; return false; }
  in.lx = shape(PS4.LStickX() / 127.0f);
  in.ly = shape(PS4.LStickY() / 127.0f);
  in.rx = shape(PS4.RStickX() / 127.0f);
  return true;
}
#endif

void handleLine(char *line) {
  char c = tolower(line[0]);
  const float v = SERIAL_SPEED;
  switch (c) {
    case 'w': in = { 0,  v, 0}; break;
    case 's': in = { 0, -v, 0}; break;
    case 'a': in = {-v,  0, 0}; break;
    case 'd': in = { v,  0, 0}; break;
    case 'q': in = { 0,  0, -v}; break;
    case 'e': in = { 0,  0,  v}; break;
    case 'x': in = { 0,  0, 0}; break;
    case 'j': {
      float a = 0, b = 0, r = 0;
      if (sscanf(line + 1, "%f %f %f", &a, &b, &r) == 3) {
        in.lx = constrain(a, -1.0f, 1.0f);
        in.ly = constrain(b, -1.0f, 1.0f);
        in.rx = constrain(r, -1.0f, 1.0f);
      }
      break;
    }
    default: return;
  }
  Serial.printf("Input  LX=%.2f  LY=%.2f  RX=%.2f\n", in.lx, in.ly, in.rx);
}

void readSerialInput() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) { buf[n] = 0; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

// ===================== KONTROL KECEPATAN RODA =====================
void controlStep(float dt) {
  float cmd[4];
  mecanum(in.lx, in.ly, in.rx, cmd);

  bool moving = false;
  for (int i = 0; i < 4; i++) {
    long c;
    noInterrupts();
    c = enc[i].count;
    interrupts();

    long dc = c - wheel[i].lastCount;
    wheel[i].lastCount = c;
    float rpm = (dc / CPR) / dt * 60.0f;
    wheel[i].meas = 0.6f * wheel[i].meas + 0.4f * rpm;

    wheel[i].target = cmd[i] * MOTOR_DIR[i] * MAX_RPM;

    float ff = wheel[i].target / MAX_RPM * PWM_MAX;
    float out = ff;

    if (fabsf(cmd[i]) < 0.001f) {
      wheel[i].integ = 0;
      out = 0;
    } else {
      moving = true;
#if USE_PID
      float err = wheel[i].target - wheel[i].meas;
      wheel[i].integ += KI * err * dt;
      wheel[i].integ = constrain(wheel[i].integ, -0.3f * PWM_MAX, 0.3f * PWM_MAX);
      out = ff + KP * err + wheel[i].integ;
#endif
    }
    setMotor(i, (int)out);
  }

  static uint32_t lastPrint = 0;
  bool spinning = false;
  for (int i = 0; i < 4; i++) if (fabsf(wheel[i].meas) > 2) spinning = true;
  if ((moving || spinning) && millis() - lastPrint >= 250) {
    lastPrint = millis();
    Serial.printf("rpm  T: %4.0f %4.0f %4.0f %4.0f | M: %4.0f %4.0f %4.0f %4.0f   (FL FR RL RR)\n",
                  wheel[0].target, wheel[1].target, wheel[2].target, wheel[3].target,
                  wheel[0].meas, wheel[1].meas, wheel[2].meas, wheel[3].meas);
  }
}

// ===================== PEMBACAAN IMU =====================
void readIMU() {
  static uint32_t lastIMU = 0;

  if (!imuReady) return;

  if (millis() - lastIMU >= IMU_MS) {
    lastIMU = millis();

    sensors_event_t event;
    bno.getEvent(&event);

    heading = event.orientation.x;
    roll = event.orientation.y;
    pitch = event.orientation.z;

    Serial.printf(
      "IMU | Heading: %.2f | Roll: %.2f | Pitch: %.2f\n",
      heading, roll, pitch
    );
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);

  // ===================== INISIALISASI IMU =====================
  Wire.begin(IMU_SDA, IMU_SCL);

  if (bno.begin()) {
    imuReady = true;

    bno.setExtCrystalUse(true);

    Serial.println("BNO055 berhasil terhubung!");
  } else {
    imuReady = false;

    Serial.println("BNO055 tidak terdeteksi!");
  }
  
  for (int i = 0; i < 4; i++) {
    pinMode(PINS[i].rpwm, OUTPUT); digitalWrite(PINS[i].rpwm, LOW);
    pinMode(PINS[i].lpwm, OUTPUT); digitalWrite(PINS[i].lpwm, LOW);

    enc[i].a = PINS[i].enca;
    enc[i].b = PINS[i].encb;
    enc[i].count = 0;
    pinMode(enc[i].a, INPUT);
    pinMode(enc[i].b, INPUT);
    enc[i].last = (digitalRead(enc[i].a) << 1) | digitalRead(enc[i].b);
    attachInterruptArg(digitalPinToInterrupt(enc[i].a), encISR, &enc[i], CHANGE);
    attachInterruptArg(digitalPinToInterrupt(enc[i].b), encISR, &enc[i], CHANGE);

    wheel[i] = {0, 0, 0, 0, 0};
  }

#ifdef USE_PS4
  PS4.begin("00:00:00:00:00:00");   // ganti dengan MAC Bluetooth ESP32 Anda
#endif

  Serial.println("Mecanum siap. Perintah: w s a d q e x  |  j LX LY RX");
}

void loop() {

#ifdef USE_PS4
  readPS4();
#else
  readSerialInput();
#endif

  // Membaca data IMU
  readIMU();

  static uint32_t lastCtrl = 0;

  uint32_t now = millis();

  if (now - lastCtrl >= CTRL_MS) {

    float dt = (now - lastCtrl) / 1000.0f;

    lastCtrl = now;

    controlStep(dt);
  }
}
