#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <MPU9250_asukiaaa.h>

// --- CẤU HÌNH WIFI & UDP ---
WiFiUDP udp;
const int udpPort = 4210;           
const char* udpAddress = "255.255.255.255";

// --- CẤU HÌNH UART GỬI ARDUINO ---
#define RXD2 16
#define TXD2 17

// --- CẤU HÌNH OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ========================================================
// --- CẤU HÌNH MPU9250 & THUẬT TOÁN YAW NÂNG CAO ---
// ========================================================
MPU9250_asukiaaa mySensor;

static const float GYRO_Z_LPF_BETA = 0.10f;
static const float STILL_GYRO_DPS  = 0.18f;
static const float STILL_ACC_G     = 0.06f;
static const float MEAN_BETA       = 0.02f;
static const float BIAS_ADAPT      = 0.020f;
static const float BIAS_Z_MAX_STEP = 0.02f;
static const bool FREEZE_YAW_WHEN_STILL = true;
static const float YAW_DB_DPS  = 0.05f;
static const float YAW_MAX_DPS = 250.0f;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float axF = 0, ayF = 0, azF = 1.0f;
float gzFilt = 0;
float gzStillMean = 0;
float angleYaw = 0;

// Góc Yaw cuối cùng

// Hàm hỗ trợ toán học cho Yaw
static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static inline float soft_deadband(float x, float db) {
  float ax = fabsf(x);
  if (ax <= db) return 0.0f;
  return (x > 0) ? (ax - db) : -(ax - db);
}

// --- CẤU HÌNH ĐIỀU KHIỂN & PID ---
#define ENCODER_PPR 7392.0 
double target_speed = 25.0; 
char currentCmd = 'S';
double sp1 = 0, sp2 = 0, sp3 = 0, sp4 = 0;

struct MotorController {
  String name;
  int pinA, pinB;
  double Kp, Ki, Kd;
  volatile long encoderCount = 0;
  int lastEncoded = 0;
  
  double input = 0;
  int pwm = 0;         

  double integral = 0, lastError = 0;
  MotorController(String n, int a, int b, double p, double i, double d) 
    : name(n), pinA(a), pinB(b), Kp(p), Ki(i), Kd(d) {}

  void begin(void (*isr)()) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    lastEncoded = (digitalRead(pinA) << 1) | digitalRead(pinB);
    attachInterrupt(digitalPinToInterrupt(pinA), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isr, CHANGE);
  }

  void computePID(double dt, double target) {
    noInterrupts();
    long delta = encoderCount;
    encoderCount = 0;
    interrupts();
    input = ((double)delta / ENCODER_PPR) / (dt / 60.0);
    
    double error = target - input;
    double P = Kp * error;
    integral += error * dt;
    
    double maxI = 255.0;
    if (Ki > 0) maxI = 255.0 / Ki;
    if (integral > maxI) integral = maxI;
    if (integral < -maxI) integral = -maxI;
    
    double I = Ki * integral;
    double D = Kd * ((error - lastError) / dt);
    lastError = error;
    double pidOut = P + I + D;
    
    if (pidOut > 255) pidOut = 255;
    if (pidOut < -255) pidOut = -255;
    
    pwm = (int)round(pidOut);
  }
};
// --- BỘ SỐ KP, KI, KD ---
MotorController m1("M1", 26, 25, 9.0, 30.0, 0.0);
MotorController m2("M2", 19, 18, 9.0, 30.0, 0.0); 
MotorController m3("M3", 34, 35, 9.5, 30.0, 0.0);
MotorController m4("M4", 14, 27, 10.0, 30.0, 0.0);

// --- NGẮT ENCODER ---
void IRAM_ATTR isr1() { int encoded = (digitalRead(m1.pinA) << 1) | digitalRead(m1.pinB); int sum = (m1.lastEncoded << 2) | encoded; if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) m1.encoderCount++; if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) m1.encoderCount--; m1.lastEncoded = encoded; }
void IRAM_ATTR isr2() { int encoded = (digitalRead(m2.pinA) << 1) | digitalRead(m2.pinB); int sum = (m2.lastEncoded << 2) | encoded; if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) m2.encoderCount++; if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) m2.encoderCount--; m2.lastEncoded = encoded; }
void IRAM_ATTR isr3() { int encoded = (digitalRead(m3.pinA) << 1) | digitalRead(m3.pinB); int sum = (m3.lastEncoded << 2) | encoded; if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) m3.encoderCount++; if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) m3.encoderCount--; m3.lastEncoded = encoded; }
void IRAM_ATTR isr4() { int encoded = (digitalRead(m4.pinA) << 1) | digitalRead(m4.pinB); int sum = (m4.lastEncoded << 2) | encoded; if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) m4.encoderCount++; if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) m4.encoderCount--; m4.lastEncoded = encoded; }

// --- BIẾN THỜI GIAN ---
unsigned long lastPidTime = 0;
unsigned long lastUdpSend = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 500;
void setup() {
  Serial.begin(115200); 
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
  Wire.begin();
  Wire.setClock(400000L);

  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  m1.begin(isr1); m2.begin(isr2); m3.begin(isr3); m4.begin(isr4);
// KẾT NỐI WIFI
  display.setCursor(0, 0); display.println("Ket noi WiFi..."); display.display();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500);
  }
  udp.begin(udpPort);
  
  // KHỞI TẠO MPU VÀ BẮT ĐẦU CALIB 10s
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
// Bắt buộc mở Accel để phát hiện đứng yên
  mySensor.beginGyro();
  
  display.clearDisplay();
  display.setCursor(0, 0); display.println("Hieu Chinh Gyro");
  display.setCursor(0, 20);
  display.println("GIU YEN XE 10s!");
  display.display();

  double sumGx = 0, sumGy = 0, sumGz = 0;
  int calibCount = 0;
  unsigned long calibStartTime = millis();

  // Vòng lặp calib 10s (Đọc cả 3 trục)
  while (millis() - calibStartTime < 10000) {
    mySensor.gyroUpdate();
    sumGx += mySensor.gyroX();
    sumGy += mySensor.gyroY();
    sumGz += mySensor.gyroZ();
    calibCount++;
// In đếm ngược
    if (calibCount % 50 == 0) {
      display.fillRect(0, 40, 128, 24, SH110X_BLACK);
      display.setCursor(0, 40); display.setTextSize(2);
      display.print(10 - (millis() - calibStartTime)/1000); display.print(" s");
      display.display();
    }
    delay(20);
  }

  // Chốt Bias ban đầu cho 3 trục
  gyroBiasX = sumGx / calibCount;
  gyroBiasY = sumGy / calibCount;
  gyroBiasZ = sumGz / calibCount;
  lastUpdateTime = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("San Sang!");
  display.print("IP: "); display.println(WiFi.localIP());
  display.display();
  delay(2000);
}

void loop() {
  unsigned long now = millis();
  
  // ==========================================================
  // 1. NHẬN LỆNH TỪ PYTHON -> CẬP NHẬT BIẾN NHỚ
  // ==========================================================
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[256]; 
    int len = udp.read(packetBuffer, 255);
    
    if (len > 0) {
      packetBuffer[len] = '\0'; // Đóng chuỗi lại cho an toàn
      currentCmd = packetBuffer[0]; // Lấy chữ cái đầu tiên
      lastCommandTime = now;
      
      // [DEBUG]: In toàn bộ chuỗi nhận được từ Laptop ra màn hình
      Serial.print("-> Nhan tu PC: ");
      Serial.println(packetBuffer);

      // NẾU LÀ LỆNH TỰ HÀNH 'M' TỪ PYTHON (Bóc tách chuỗi ra 4 vận tốc)
      if (currentCmd == 'M') {
        sscanf(packetBuffer, "M,%lf,%lf,%lf,%lf", &sp1, &sp2, &sp3, &sp4);
      }
    }
    udp.flush(); // Xóa sạch bộ đệm để đợi gói tin tiếp theo
  }


  // Watchdog: nếu mất lệnh từ máy tính, dừng robot thay vì giữ lệnh cũ.
  if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {
    currentCmd = 'S';
    sp1 = sp2 = sp3 = sp4 = 0;
    m1.integral = m2.integral = m3.integral = m4.integral = 0;
  }

  // ==========================================================
  // 2. DỊCH LỆNH BẰNG TAY (ĐI 8 HƯỚNG + XOAY TẠI CHỖ)
  // ==========================================================
  // 4 Hướng Cơ Bản (Trượt thẳng và ngang)
  if      (currentCmd == 'F') { sp1 =  target_speed; sp2 =  target_speed; sp3 =  target_speed; sp4 =  target_speed; } 
  else if (currentCmd == 'B') { sp1 = -target_speed; sp2 = -target_speed; sp3 = -target_speed; sp4 = -target_speed; } 
  else if (currentCmd == 'L') { sp1 = -target_speed; sp2 =  target_speed; sp3 = -target_speed; sp4 =  target_speed; } 
  else if (currentCmd == 'R') { sp1 =  target_speed; sp2 = -target_speed; sp3 =  target_speed; sp4 = -target_speed; } 
  
  // 4 Hướng Chéo (Mecanum/Omni 45 độ)
  else if (currentCmd == 'Q') { sp1 = 0;             sp2 =  target_speed; sp3 = 0;             sp4 =  target_speed; } 
  else if (currentCmd == 'E') { sp1 =  target_speed; sp2 = 0;             sp3 =  target_speed; sp4 = 0;             } 
  else if (currentCmd == 'Z') { sp1 = -target_speed; sp2 = 0;             sp3 = -target_speed; sp4 = 0;             } 
  else if (currentCmd == 'C') { sp1 = 0;             sp2 = -target_speed; sp3 = 0;             sp4 = -target_speed; } 

  // 2 Lệnh Xoay (Quay đầu)
  else if (currentCmd == 'U') { sp1 = -target_speed; sp2 = -target_speed; sp3 =  target_speed; sp4 =  target_speed; } 
  else if (currentCmd == 'O') { sp1 =  target_speed; sp2 =  target_speed; sp3 = -target_speed; sp4 = -target_speed; } 

  // Dừng lại
  else if (currentCmd == 'S') { sp1 = 0;             sp2 = 0;             sp3 = 0;             sp4 = 0;             }

  // ==========================================================
  // 3. CẬP NHẬT GÓC YAW (ADVANCED ALGORITHM)
  // ==========================================================
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  double dt_imu = (double)(now - lastUpdateTime) / 1000.0;
  
  if (dt_imu > 0) {
    lastUpdateTime = now;
    
    float ax = mySensor.accelX();
    float ay = mySensor.accelY();
    float az = mySensor.accelZ();
    
    float gx = mySensor.gyroX() - gyroBiasX;
    float gy = mySensor.gyroY() - gyroBiasY;
    float gz = mySensor.gyroZ() - gyroBiasZ;
    
    const float ACC_LPF = 0.12f;
    axF += ACC_LPF * (ax - axF);
    ayF += ACC_LPF * (ay - ayF);
    azF += ACC_LPF * (az - azF);
    
    gzFilt += GYRO_Z_LPF_BETA * (gz - gzFilt);
    gzFilt = clampf(gzFilt, -YAW_MAX_DPS, YAW_MAX_DPS);

    float accMag = sqrtf(axF * axF + ayF * ayF + azF * azF);
    bool still = (fabsf(accMag - 1.0f) < STILL_ACC_G) &&
                 (fabsf(gx) < STILL_GYRO_DPS) &&
                 (fabsf(gy) < STILL_GYRO_DPS) &&
                 (fabsf(gzFilt) < STILL_GYRO_DPS);
                 
    if (still) {
      gzStillMean += MEAN_BETA * (gzFilt - gzStillMean);
      float step = BIAS_ADAPT * gzStillMean;
      step = clampf(step, -BIAS_Z_MAX_STEP, BIAS_Z_MAX_STEP);
      gyroBiasZ += step;
      
      gz = mySensor.gyroZ() - gyroBiasZ;
      gzFilt += GYRO_Z_LPF_BETA * (gz - gzFilt);
      
      if (FREEZE_YAW_WHEN_STILL) {
        gzFilt = 0.0f; 
      }
    } else {
      gzStillMean *= 0.995f;
    }

    float gzUse = soft_deadband(gzFilt, YAW_DB_DPS);
    angleYaw += gzUse * dt_imu;
    angleYaw = wrap180(angleYaw);
  }

  // ==========================================================
  // 4. TÍNH PID VÀ GỬI XUỐNG ARDUINO (Mỗi 50ms)
  // ==========================================================
  if (now - lastPidTime >= 50) {
    double dt_pid = (now - lastPidTime) / 1000.0;
    lastPidTime = now;

    m1.computePID(dt_pid, sp1); 
    m2.computePID(dt_pid, sp2); 
    m3.computePID(dt_pid, sp3); 
    m4.computePID(dt_pid, sp4);
    
    String data = String(m1.pwm) + "," + String(m2.pwm) + "," + String(m3.pwm) + "," + String(m4.pwm);
    Serial2.println(data);

    // [DEBUG]: In chuỗi PWM vừa đẩy xuống Arduino ra màn hình
    Serial.print("<- Gui xuong Arduino: ");
    Serial.println(data);
  }

  // ==========================================================
  // 5. HIỂN THỊ OLED & GỬI UDP VỀ LAB (Mỗi 100ms - 10Hz)
  // ==========================================================
  if (now - lastUdpSend >= 100) {
    lastUdpSend = now;
    
    String labData = String(m1.input, 1) + "," + String(m2.input, 1) + "," + 
                     String(m3.input, 1) + "," + String(m4.input, 1) + "," + 
                     String(angleYaw, 1);
                     
    udp.beginPacket(udpAddress, udpPort);
    udp.print(labData);
    udp.endPacket();

    display.clearDisplay();
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.print("YAW: "); display.print(angleYaw, 1);
    display.setCursor(80, 0);  display.print("CMD: "); display.print(currentCmd);

    display.setCursor(0, 20);  display.print("M1: "); display.print(m1.input, 1);
    display.setCursor(64, 20); display.print("M2: "); display.print(m2.input, 1);
    
    display.setCursor(0, 40);  display.print("M4: "); display.print(m4.input, 1);
    display.setCursor(64, 40); display.print("M3: "); display.print(m3.input, 1);

    display.display();
  }
}