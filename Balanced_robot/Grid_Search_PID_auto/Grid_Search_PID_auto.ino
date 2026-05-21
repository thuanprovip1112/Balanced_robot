#include <Wire.h>
#include <AFMotor_R4.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

// ===================================================
// CẤU HÌNH GRID SEARCH (Lược SOFT - Vùng nhẹ)
// ===================================================
const float KP_SOFT_START = 4.0;
const float KP_SOFT_END   = 10.0;
const float KP_SOFT_STEP  = 1.0;

const float KD_SOFT_START = 0.5;
const float KD_SOFT_END   = 2.0;
const float KD_SOFT_STEP  = 0.25;

const float KI_SOFT_FIXED = 0.4;

// ===================================================
// CẤU HÌNH GRID SEARCH (Lược HARD - Vùng lớn)
// ===================================================
const float KP_HARD_START = 14.0;
const float KP_HARD_END   = 22.0;
const float KP_HARD_STEP  = 1.0;

const float KD_HARD_START = 2.5;
const float KD_HARD_END   = 4.0;
const float KD_HARD_STEP  = 0.25;

const float KI_HARD_FIXED = 0.8;

// Thời gian chạy test cho mỗi bộ số (ms)
const unsigned long TEST_TIME = 4000; 

// ===================================================
// CẤU HÌNH ROBOT
// ===================================================
float setpoint = 0.0;

const float INTEGRAL_LIMIT  = 50.0;
const int   MAX_SPEED        = 255;
const float DEAD_ZONE        = 0.3;  
const int   MOTOR_DEADBAND   = 40;   // Bù ma sát tĩnh trực tiếp trên PWM
const float FALL_ANGLE       = 25.0;
const float ALPHA            = 0.98;
const float SCHEDULE_ANGLE     = 3.0;   // Chuyển vùng tại 3 độ
const float SCHEDULE_HYSTERESIS = 0.5;  // Hysteresis

const bool INVERT_LEFT  = true;
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
bool aggressiveMode = false;
unsigned long lastTime = 0;
int16_t gyroXoffset = 0;

// Lưu kết quả tối ưu nhất
float bestKp_Soft    = 0;
float bestKd_Soft    = 0;
float bestScore_Soft = 999999.0;

float bestKp_Hard    = 0;
float bestKd_Hard    = 0;
float bestScore_Hard = 999999.0;

// Khai báo các hàm chức năng
void calibrateMPU();
void calibrateZeroAngle();
void driveMotors(float control);
void stopMotors();
float readAngle();
float testPID(float testKp, float testKi, float testKd, bool &isFallen);
void waitRobotToBeVertical();

// ===================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Grid Search PID");
  lcd.setCursor(0, 1); lcd.print("Initializing...");

  stopMotors();

  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("MPU6050 ERROR!");
    while (1) delay(100);
  }

  // Cân bằng cảm biến tĩnh
  calibrateMPU();

  // Khởi tạo góc ban đầu từ gia tốc trọng trường
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle = atan2((float)ay, (float)az) * 180.0 / PI;
  if (INVERT_ANGLE) angle = -angle;

  // Hiệu chuẩn góc zero từ vị trí đứng thẳng
  calibrateZeroAngle();

  // ===================================================
  // GRID SEARCH - VÙNG SOFT
  // ===================================================
  Serial.println("\n================================================");
  Serial.println("TUNING SOFT PID (Vùng gần 0 độ)");
  Serial.println("Kp     | Kd     | SCORE    | STATUS");
  Serial.println("================================================");

  int testNum = 0;
  int totalTests_Soft = ((KP_SOFT_END - KP_SOFT_START) / KP_SOFT_STEP + 1) *
                        ((KD_SOFT_END - KD_SOFT_START) / KD_SOFT_STEP + 1);
  int totalTests_Hard = ((KP_HARD_END - KP_HARD_START) / KP_HARD_STEP + 1) *
                        ((KD_HARD_END - KD_HARD_START) / KD_HARD_STEP + 1);

  // for (float testKp = KP_SOFT_START; testKp <= KP_SOFT_END + 0.01; testKp += KP_SOFT_STEP) {
  //   for (float testKd = KD_SOFT_START; testKd <= KD_SOFT_END + 0.01; testKd += KD_SOFT_STEP) {
  //     testNum++;

  //     if (testNum < 39) continue;

  //     waitRobotToBeVertical();

  //     lcd.clear();
  //     lcd.setCursor(0, 0);
  //     lcd.print("Soft "); lcd.print(testNum); lcd.print("/"); lcd.print(totalTests_Soft);
  //     lcd.setCursor(0, 1);
  //     lcd.print("Kp:"); lcd.print(testKp, 1); lcd.print(" Kd:"); lcd.print(testKd, 2);

  //     Serial.print("Kp="); Serial.print(testKp, 1);
  //     Serial.print(" | Kd="); Serial.print(testKd, 2);
  //     Serial.print(" -> ");

  //     bool isFallen = false;
  //     float score = testPID(testKp, KI_SOFT_FIXED, testKd, isFallen);

  //     Serial.print("Score="); Serial.print(score, 1);

  //     if (!isFallen && score < bestScore_Soft) {
  //       bestScore_Soft = score;
  //       bestKp_Soft    = testKp;
  //       bestKd_Soft    = testKd;
  //       Serial.print(" *** TOT NHAT ***");
  //     }

  //     if (isFallen) Serial.print(" [NGA]");
  //     Serial.println();

  //     stopMotors();
  //   }
  // }

  // ===================================================
  // GRID SEARCH - VÙNG HARD
  // ===================================================
  Serial.println("\n================================================");
  Serial.println("TUNING HARD PID (Vùng cứu đổ > 3 độ)");
  Serial.println("Kp     | Kd     | SCORE    | STATUS");
  Serial.println("================================================");

  testNum = 0;

  for (float testKp = KP_HARD_START; testKp <= KP_HARD_END + 0.01; testKp += KP_HARD_STEP) {
    for (float testKd = KD_HARD_START; testKd <= KD_HARD_END + 0.01; testKd += KD_HARD_STEP) {
      testNum++;
      
      if (testNum < 55) continue; // Bỏ qua một số bộ số đầu để tập trung vào vùng có khả năng tốt hơn (theo kết quả từ lần chạy trước)
      
      waitRobotToBeVertical();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Hard "); lcd.print(testNum); lcd.print("/"); lcd.print(totalTests_Hard);
      lcd.setCursor(0, 1);
      lcd.print("Kp:"); lcd.print(testKp, 1); lcd.print(" Kd:"); lcd.print(testKd, 2);

      Serial.print("Kp="); Serial.print(testKp, 1);
      Serial.print(" | Kd="); Serial.print(testKd, 2);
      Serial.print(" -> ");

      bool isFallen = false;
      float score = testPID(testKp, KI_HARD_FIXED, testKd, isFallen);

      Serial.print("Score="); Serial.print(score, 1);

      if (!isFallen && score < bestScore_Hard) {
        bestScore_Hard = score;
        bestKp_Hard    = testKp;
        bestKd_Hard    = testKd;
        Serial.print(" *** TOT NHAT ***");
      }

      if (isFallen) Serial.print(" [NGA]");
      Serial.println();

      stopMotors();
    }
  }

  // ===================================================
  // HIỂN THỊ KẾT QUẢ CUỐI CÙNG
  // ===================================================
  Serial.println("\n================================================");
  Serial.println("KET QUA TOT NHAT - SOFT PID:");
  Serial.print("Kp_Soft = "); Serial.println(bestKp_Soft, 1);
  Serial.print("Ki_Soft = "); Serial.println(KI_SOFT_FIXED, 1);
  Serial.print("Kd_Soft = "); Serial.println(bestKd_Soft, 1);
  Serial.print("Score = "); Serial.println(bestScore_Soft, 1);

  Serial.println("\nKET QUA TOT NHAT - HARD PID:");
  Serial.print("Kp_Hard = "); Serial.println(bestKp_Hard, 1);
  Serial.print("Ki_Hard = "); Serial.println(KI_HARD_FIXED, 1);
  Serial.print("Kd_Hard = "); Serial.println(bestKd_Hard, 1);
  Serial.print("Score = "); Serial.println(bestScore_Hard, 1);
  Serial.println("================================================");

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("DONE! Copy PID:");
  lcd.setCursor(0, 1);
  lcd.print("S:"); lcd.print(bestKp_Soft, 0); lcd.print(" H:"); lcd.print(bestKp_Hard, 0);

  while (1) {
    delay(100);
  }
}

void loop() {
  // Không dùng vòng loop chính, toàn bộ tiến trình chạy một lần trong setup()
}

// ===================================================
// Hàm đọc góc thô phục vụ khởi tạo hoặc đồng bộ nhanh
// ===================================================
float readAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float a = atan2((float)ay, (float)az) * 180.0 / PI;
  if (INVERT_ANGLE) a = -a;
  return a;
}

// ===================================================
// Hàm khóa tiến trình, đợi người dùng dựng xe đứng thẳng
// ===================================================
void waitRobotToBeVertical() {
  stopMotors();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("DUNG XE THANG!");
  
  float currentAngle = readAngle();
  
  // Đợi cho đến khi góc lệch so với điểm setpoint nhỏ hơn 2 độ
  while (fabs(currentAngle - setpoint) > 2.0) {
    currentAngle = readAngle();
    lcd.setCursor(0, 1);
    lcd.print("Goc hien tai: "); lcd.print(currentAngle, 1); lcd.print("  ");
    delay(200);
  }
  
  // Xe đã thẳng đứng, đếm ngược ngắn chuẩn bị thả tay
  for (int i = 2; i > 0; i--) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Chuan bi tha...");
    lcd.setCursor(0, 1); lcd.print(i); lcd.print("s");
    delay(1000);
  }
}

// ===================================================
// Hàm chạy thử nghiệm 1 bộ số PID
// Trả về điểm phạt (Càng thấp càng tốt)
// ===================================================
float testPID(float testKp, float testKi, float testKd, bool &isFallen) {
  float score       = 0;
  float intTerm     = 0;
  float lError      = 0;
  float lDeriv      = 0;
  float localAngle  = readAngle(); // Đồng bộ góc ban đầu sát thực tế nhất
  
  unsigned long start = millis();
  unsigned long lTime = start;

  while (millis() - start < TEST_TIME) {
    unsigned long now = millis();
    float dt = (now - lTime) / 1000.0;
    lTime = now;
    if (dt <= 0 || dt > 0.1) dt = 0.01;

    // Đọc dữ liệu cảm biến
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accAngle = atan2((float)ay, (float)az) * 180.0 / PI;
    float gyroRate = ((float)gx - gyroXoffset) / 131.0;
    if (INVERT_ANGLE) { 
      accAngle = -accAngle; 
      gyroRate = -gyroRate; 
    }

    // Bộ lọc bù kết hợp Gia tốc và Tốc độ góc
    localAngle = ALPHA * (localAngle + gyroRate * dt) + (1.0 - ALPHA) * accAngle;

    // Áp dụng zero offset (hiệu chuẩn)
    float calibratedAngle = localAngle - zeroOffset;

    // Nếu xe nghiêng vượt ngưỡng cho phép -> Xác định ngã -> Dừng khẩn cấp
    if (fabs(calibratedAngle - setpoint) > FALL_ANGLE) {
      stopMotors();
      isFallen = true;
      return 999999.0; // Trả về điểm phạt cực lớn
    }

    // Tính toán sai số
    float error = calibratedAngle - setpoint;

    // Thành phần Tích phân (I)
    intTerm += error * dt;
    intTerm  = constrain(intTerm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    // Thành phần Vi phân (D) tích hợp bộ lọc nhiễu tần số cao (Low-pass filter)
    float rawDeriv = (error - lError) / dt;
    lDeriv = 0.7 * lDeriv + 0.3 * rawDeriv;
    lError = error;

    // Tổng hợp lực điều khiển
    float control = (testKp * error) + (testKi * intTerm) + (testKd * lDeriv);

    // Xuất tín hiệu điều khiển động cơ
    driveMotors(control);

    // Thuật toán chấm điểm tối ưu (IAE + Phạt vận tốc điều khiển để xe hạn chế phóng nhanh)
    score += (fabs(error) * dt) + (fabs(control) * 0.002 * dt);

    delay(10); // Tạo khoảng nghỉ ngắn để chu kỳ lấy mẫu dt ổn định quanh mức 10ms
  }

  stopMotors();
  return score;
}

// ===================================================
// Hàm điều khiển động cơ mượt, triệt tiêu lỗi giật xung của map()
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

// ===================================================
// Hàm dừng động cơ an toàn
// ===================================================
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
  lcd.setCursor(0, 0); lcd.print("Ready to search...");
}

// ===================================================
// Hàm lấy mẫu tĩnh loại bỏ sai số lệch (Offset) của Gyro
// ===================================================
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
  lcd.setCursor(0, 1); lcd.print("Offset X: "); lcd.print(gyroXoffset);
  delay(1000);
}