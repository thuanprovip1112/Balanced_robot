#include <Wire.h>
#include <AFMotor_R4.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

// ===================================================
// HỆ SỐ PID CHÍNH THỨC (HARD ONLY)
// ===================================================
const float KP_HARD = 21.5;  // PID dùng duy nhất
const float KI_HARD = 0.8;
const float KD_HARD = 2.2;

// ===================================================
// CẤU HÌNH ROBOT
// ===================================================
float setpoint = 0.0; // Điểm góc cân bằng thực tế của xe

const float INTEGRAL_LIMIT  = 50.0;
const int   MAX_SPEED        = 255;
const int   DEAD_ZONE        = 0.3;  
const int   MOTOR_DEADBAND   = 40;   // Bù ma sát tĩnh trực tiếp trên PWM
const float FALL_ANGLE       = 25.0; 
const float ALPHA            = 0.98; // Hệ số bộ lọc bù

const bool INVERT_LEFT  = true;  // Giữ theo cấu hình đã chạy được ở file auto
const bool INVERT_RIGHT = true; 
const bool INVERT_ANGLE = true;

// ===================================================
// BIẾN RUNTIME
// ===================================================
float angle        = 0.0;
float zeroOffset   = 0.0;
float integralTerm = 0.0;
float lastError    = 0.0;
float lastDeriv    = 0.0;
unsigned long lastTime = 0;
int16_t gyroXoffset = 0;

// Khai báo các hàm chức năng
void calibrateMPU();
void calibrateZeroAngle();
void driveMotors(float control);
void stopMotors();

// ===================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  delay(100);
  lcd.init();
  delay(100);
  lcd.backlight();
  delay(100);
  lcd.clear();
  delay(100);
  lcd.setCursor(0, 0); lcd.print("Balancing Robot");
  lcd.setCursor(0, 1); lcd.print("Initializing...");

  stopMotors();

  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("MPU6050 ERROR!");
    while (1) delay(100);
  }

  // Cân bằng cảm biến tĩnh lúc khởi động
  calibrateMPU();

  // Đọc góc ban đầu để đồng bộ bộ lọc bù
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle = atan2((float)ay, (float)az) * 180.0 / PI;
  if (INVERT_ANGLE) angle = -angle;

  // Hiệu chuẩn góc zero theo vị trí đứng thẳng hiện tại
  calibrateZeroAngle();

  // Đếm ngược 3 giây ra hiệu cho người dùng dựng xe sẵn sàng
  for (int i = 1; i > 0; i--) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("DUNG XE THANG!");
    lcd.setCursor(0, 1); lcd.print("Kich hoat sau: "); lcd.print(i); lcd.print("s");
    delay(1000);
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("PID RUNNING...");
  
  lastTime = millis();
  delay(1000);
}

// ===================================================
// VÒNG LẶP ĐIỀU KHIỂN THỜI GIAN THỰC
// ===================================================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z') {
      calibrateZeroAngle();
    }
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  
  // Khóa chu kỳ lấy mẫu lý tưởng ~10ms (100Hz) để bộ PID chạy ổn định
  if (dt < 0.001) {
    return; 
  }
  lastTime = now;

  // Đọc dữ liệu thô từ cảm biến
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  float gyroRate = ((float)gx - gyroXoffset) / 131.0;
  if (INVERT_ANGLE) { 
    accAngle = -accAngle; 
    gyroRate = -gyroRate; 
  }

  // Bộ lọc bù tính toán góc nghiêng hiện tại
  angle = ALPHA * (angle + gyroRate * dt) + (1.0 - ALPHA) * accAngle;

  float calibratedAngle = angle - zeroOffset;

  // Kiểm tra trạng thái ngã an toàn
  if (fabs(calibratedAngle - setpoint) > FALL_ANGLE) {
    stopMotors();
    integralTerm = 0; // Reset tích phân khi ngã để tránh tích lực ảo
    lcd.setCursor(0, 1); lcd.print("STATUS: FALLEN  ");
    
    // In trạng thái ra Serial để theo dõi
    Serial.print("Angle:"); Serial.print(calibratedAngle);
    Serial.println(", STATUS: FALLEN!");
    return; 
  }

  // Thuật toán PID hard-only
  float error = calibratedAngle - setpoint;
  float Kp = KP_HARD;
  float Ki = KI_HARD;
  float Kd = KD_HARD;

  // Cập nhật trạng thái LCD bình thường
  lcd.setCursor(0, 1); 
  lcd.print("G:"); lcd.print(calibratedAngle, 1);
  lcd.print("   ");

  // Khâu tích phân (I)
  integralTerm += error * dt;
  integralTerm  = constrain(integralTerm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Khâu vi phân (D) + Bộ lọc nhiễu tần số cao
  float rawDeriv = (error - lastError) / dt;
  lastDeriv = 0.7 * lastDeriv + 0.3 * rawDeriv;
  lastError = error;

  // Tính tổng lực điều khiển
  float control = (Kp * error) + (Ki * integralTerm) + (Kd * lastDeriv);

  // Xuất lực kéo động cơ
  driveMotors(control);

  // Gửi dữ liệu lên Serial Plotter nếu bạn muốn vẽ đồ thị trực quan
  Serial.print("Angle:"); Serial.print(calibratedAngle);
  Serial.print(",Error:"); Serial.print(error);
  Serial.print(",Control:"); Serial.println(control);
}

// ===================================================
// CÁC HÀM BỔ TRỢ PHẦN CỨNG
// ===================================================
void driveMotors(float control) {
  if (fabs(control) < DEAD_ZONE) {
    stopMotors();
    return;
  }

  int spd = (int)fabs(control);
  // Bù deadband tĩnh: cộng trực tiếp vào PWM
  if (spd > 0) {
    spd += MOTOR_DEADBAND;
  }
  if (spd > MAX_SPEED) spd = MAX_SPEED;

  uint8_t dirL = (control > 0) ? FORWARD : BACKWARD;
  uint8_t dirR = (control > 0) ? FORWARD : BACKWARD;

  if (INVERT_LEFT)  dirL = (dirL == FORWARD) ? BACKWARD : FORWARD;
  if (INVERT_RIGHT) dirR = (dirR == FORWARD) ? BACKWARD : FORWARD;

  motorL.setSpeed(spd); motorL.run(dirL);
  motorR.setSpeed(spd); motorR.run(dirR);
}

void stopMotors() {
  motorL.setSpeed(0); motorL.run(RELEASE);
  motorR.setSpeed(0); motorR.run(RELEASE);
}

void calibrateZeroAngle() {
  zeroOffset = angle;
  angle = 0.0;
  integralTerm = 0.0;
  lastError = 0.0;
  lastDeriv = 0.0;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Zero calibrated");
  lcd.setCursor(0, 1); lcd.print("Offset:"); lcd.print(zeroOffset, 2);
  Serial.print("Zero offset set to: "); Serial.println(zeroOffset, 2);
  delay(800);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("PID RUNNING...");
}

void calibrateMPU() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calibrating...");
  lcd.setCursor(0, 1); lcd.print("KEEP STILL!");
  delay(1500);

  const int samples = 500;
  long gx_sum = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx_sum += gx;
    delay(3);
  }

  gyroXoffset = (int16_t)(gx_sum / samples);
  
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calib done!");
  delay(500);
}