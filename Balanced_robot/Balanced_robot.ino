#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

// ===================================================
// CẤU HÌNH CHÂN ĐỘNG CƠ CẬP NHẬT (TRỤC X & TRỤC Y)
// ===================================================
// Motor LEFT (Trục X): D2(STEP), D5(DIR)
#define MOTOR_LEFT_STEP 2
#define MOTOR_LEFT_DIR 5

// Motor RIGHT (Trục Y): D3(STEP), D6(DIR) -- ĐÃ CẬP NHẬT CHÍNH XÁC
#define MOTOR_RIGHT_STEP 3
#define MOTOR_RIGHT_DIR 6

// SRF05 Ultrasonic Sensor
#define TRIG_PIN 9
#define ECHO_PIN 10

// CNC V3 Shield Enable Pin tổng
#define ENABLE_PIN 8

// Khởi tạo đối tượng động cơ bước (1 = Driver interface)
AccelStepper motorL(1, MOTOR_LEFT_STEP, MOTOR_LEFT_DIR);
AccelStepper motorR(1, MOTOR_RIGHT_STEP, MOTOR_RIGHT_DIR);

LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

// ===================================================
// HỆ SỐ PID CHÍNH THỨC (Tinh chỉnh cho vi bước mịn)
// ===================================================
const float KP_HARD = 45.0;  
const float KI_HARD = 1.5;   
const float KD_HARD = 5.5;   

// ===================================================
// CẤU HÌNH HỆ THỐNG ĐỘNG CƠ BƯỚC
// ===================================================
float setpoint = 0.0; // Góc cân bằng mong muốn (độ)

const float INTEGRAL_LIMIT  = 50.0;
const float DEAD_ZONE        = 0.3;  
const float FALL_ANGLE       = 25.0; // Góc ngã an toàn để ngắt động cơ
const float ALPHA            = 0.98; // Hệ số bộ lọc bù cho MPU6050

// Đảo chiều quay động cơ và cảm biến nếu cần thiết để xe tiến/lùi đúng hướng
const bool INVERT_LEFT  = true;
const bool INVERT_RIGHT = true; 
const bool INVERT_ANGLE = true;

// Cấu hình vi bước cơ khí (Đã gạt Jumper trên mạch thành 1/4 step)
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 4;  

// Ngưỡng phát xung an toàn và mượt mà cho vi xử lý ATmega328 (Arduino Uno)
const long MAX_STEPPER_SPEED = 6000;  // Tốc độ tối đa (steps/sec)
const long MAX_ACCELERATION  = 12000; // Gia tốc thích ứng (steps/sec²)

// ===================================================
// BIẾN ĐIỀU KHIỂN RUNTIME
// ===================================================
float angle        = 0.0;
float zeroOffset   = 0.0;
float integralTerm = 0.0;
float lastError    = 0.0;
float lastDeriv    = 0.0;

unsigned long lastPIDTime = 0;
unsigned long lastLCDTime = 0;
int16_t gyroXoffset = 0;

// Biến điều khiển cảm biến siêu âm SRF05
float currentDistance = 0.0;
unsigned long lastDistanceTime = 0;
const unsigned long DISTANCE_INTERVAL = 150; // Chu kỳ đo khoảng cách 150ms

// Khai báo nguyên mẫu các hàm chức năng
void calibrateMPU();
void calibrateZeroAngle();
void driveMotors(float control);
void stopMotors();
float measureDistanceNonBlocking();
void updateDisplay(float currentAngle);

// ===================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Ép xung I2C lên Tốc độ cao (400kHz) để tăng tốc xử lý dữ liệu
  Wire.setClock(400000); 
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Kích hoạt toàn bộ Driver trên CNC Shield
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Mức LOW = Bật dòng giữ motor
  
  delay(100);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Balancing Robot");
  lcd.setCursor(0, 1); lcd.print("Axis: X & Y Ok");

  stopMotors();
  
  // Thiết lập giới hạn vận hành cho động cơ bước
  motorL.setMaxSpeed(MAX_STEPPER_SPEED);
  motorL.setAcceleration(MAX_ACCELERATION);
  motorR.setMaxSpeed(MAX_STEPPER_SPEED);
  motorR.setAcceleration(MAX_ACCELERATION);

  // Khởi động cảm biến gia tốc
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("MPU6050 ERROR!");
    while (1) delay(100);
  }

  // Khử sai số tĩnh cho Gyroscope
  calibrateMPU();

  // Đọc hướng ban đầu đồng bộ bộ lọc
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle = atan2((float)ax, (float)az) * 180.0 / PI;
  if (INVERT_ANGLE) angle = -angle;

  // Lấy mốc cân bằng hiện tại thực tế
  calibrateZeroAngle();

  // Đếm ngược an toàn trước khi kích hoạt xe
  for (int i = 3; i > 0; i--) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("DUNG XE THANG!");
    lcd.setCursor(0, 1); lcd.print("Kich hoat: "); lcd.print(i); lcd.print("s");
    delay(1000);
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("PID RUNNING...");
  
  lastPIDTime = micros(); 
}

// ===================================================
// VÒNG LẶP ĐIỀU KHIỂN THỜI GIAN THỰC (REAL-TIME LOOP)
// ===================================================
void loop() {
  // LỆNH ƯU TIÊN SỐ 1: Phải gọi liên tục ở mỗi vòng lặp để sinh xung mượt
  motorL.run();
  motorR.run();

  // Lắng nghe lệnh đặt lại mốc 0 từ Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z') {
      calibrateZeroAngle();
    }
  }

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastPIDTime) / 1000000.0;
  
  // Giới hạn chu kỳ vòng lặp PID cố định 10ms (100Hz) giúp bộ lọc & bộ tích phân chạy chuẩn xác
  if (dt < 0.010) {
    return; // Chưa đủ thời gian chu kỳ -> Tiếp tục quay lại đầu loop phát xung cho motor
  }
  lastPIDTime = nowMicros;

  // Đọc dữ liệu MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Tính toán góc nghiêng dựa trên trọng lực và vận tốc góc
  float accAngle = atan2((float)ax, (float)az) * 180.0 / PI;
  float gyroRate = ((float)gx - gyroXoffset) / 131.0;
  if (INVERT_ANGLE) { 
    accAngle = -accAngle; 
    gyroRate = -gyroRate; 
  }

  // Bộ lọc bù (Complementary Filter) loại bỏ nhiễu rung và trôi góc
  angle = ALPHA * (angle + gyroRate * dt) + (1.0 - ALPHA) * accAngle;
  float calibratedAngle = angle - zeroOffset;

  // Bảo vệ hệ thống: Tự ngắt nguồn động cơ khi robot ngã quá góc cho phép
  if (fabs(calibratedAngle - setpoint) > FALL_ANGLE) {
    stopMotors();
    integralTerm = 0; 
    if (millis() - lastLCDTime > 200) {
      lcd.setCursor(0, 1); lcd.print("STATUS: FALLEN  ");
      lastLCDTime = millis();
    }
    return; 
  }

  // Đo khoảng cách siêu âm bất đồng bộ (Không gây trễ luồng PID)
  if (millis() - lastDistanceTime >= DISTANCE_INTERVAL) {
    lastDistanceTime = millis();
    currentDistance = measureDistanceNonBlocking();
  }

  // Tính toán sai số PID
  float error = calibratedAngle - setpoint;

  // Khâu tích phân (Integral Term) kèm chống bão hòa tích phân (Anti-windup)
  integralTerm += error * dt;
  integralTerm  = constrain(integralTerm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Khâu vi phân (Derivative Term) đi kèm bộ lọc thông thấp (Low-pass filter) tần số cao
  float rawDeriv = (error - lastError) / dt;
  lastDeriv = 0.7 * lastDeriv + 0.3 * rawDeriv;
  lastError = error;

  // Tính toán tổng lực điều khiển đầu ra
  float control = (KP_HARD * error) + (KI_HARD * integralTerm) + (KD_HARD * lastDeriv);

  // Gửi lệnh điều tốc sang Driver động cơ bước
  driveMotors(control);
  
  // Cập nhật giao diện màn hình LCD định kỳ (200ms một lần)
  if (millis() - lastLCDTime >= 200) {
    lastLCDTime = millis();
    updateDisplay(calibratedAngle);
  }

  // Xuất dữ liệu đồ thị lên Serial Plotter
  Serial.print("Angle:"); Serial.print(calibratedAngle);
  Serial.print(",Distance:"); Serial.print(currentDistance);
  Serial.print(",Control:"); Serial.println(control);
}

// ===================================================
// HÀM ĐIỀU KHIỂN PHẦN CỨNG CHI TIẾT
// ===================================================
void driveMotors(float control) {
  if (fabs(control) < DEAD_ZONE) {
    stopMotors();
    return;
  }

  // Quy đổi giá trị điều khiển PID sang tốc độ phát xung thực tế
  float motorSpeed = (control / 20.0) * MAX_STEPPER_SPEED;
  motorSpeed = constrain(motorSpeed, -MAX_STEPPER_SPEED, MAX_STEPPER_SPEED);

  // Áp dụng hướng quay logic cho cả 2 bên bánh xe
  if (INVERT_LEFT)  motorL.setSpeed(-motorSpeed);
  else              motorL.setSpeed(motorSpeed);
  
  if (INVERT_RIGHT) motorR.setSpeed(-motorSpeed);
  else              motorR.setSpeed(motorSpeed);
}

void stopMotors() {
  motorL.setSpeed(0);
  motorR.setSpeed(0);
}

// Hàm đo khoảng cách siêu âm an toàn bằng cách giới hạn thời gian chờ tối đa 3ms (~50cm)
float measureDistanceNonBlocking() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 3000); 
  
  if (duration == 0) return -1.0; // Trả về -1 nếu không phát hiện vật cản gần
  return (duration * 0.034) / 2.0;  
}

// Hàm in dữ liệu lên LCD
void updateDisplay(float currentAngle) {
  lcd.setCursor(0, 1);
  lcd.print("G:"); 
  lcd.print(currentAngle, 1);
  lcd.print("   ");

  lcd.setCursor(10, 0);
  lcd.print("D:");
  if (currentDistance > 0 && currentDistance < 100) {
    lcd.print((int)currentDistance);
    lcd.print("cm ");
  } else {
    lcd.print("N/A ");
  }
}

void calibrateZeroAngle() {
  zeroOffset = angle;
  angle = 0.0;
  integralTerm = 0.0;
  lastError = 0.0;
  lastDeriv = 0.0;
  
  motorL.setCurrentPosition(0);
  motorR.setCurrentPosition(0);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Zero calibrated");
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
    delay(2);
  }

  gyroXoffset = (int16_t)(gx_sum / samples);
  
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calib done!");
  delay(500);
}