#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

#define MOTOR_LEFT_STEP 2
#define MOTOR_LEFT_DIR 5
#define MOTOR_RIGHT_STEP 3
#define MOTOR_RIGHT_DIR 6
#define ENABLE_PIN 8

LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

const float ALPHA = 0.90;            // Bộ lọc bù độ nhạy cao
float setpoint = 0.0;
const float INTEGRAL_LIMIT = 50.0;   
const float FALL_ANGLE = 25.0;   

// Đảo chiều phần cứng nếu xe chạy ngược hướng đổ (Đổi true/false nếu cần)
const bool INVERT_LEFT  = true;
const bool INVERT_RIGHT = true; 
const bool INVERT_ANGLE = true;

// ===================================================
// DẢI QUÉT TUNING MỚI - TĂNG KP BẮT ĐẦU TỪ 200
// ===================================================
const float KP_START = 600.0;  // Bắt đầu từ 200 theo yêu cầu của bạn
const float KP_END   = 800.0;  // Tăng trần lên 400 để dải quét rộng hơn
const float KP_STEP  = 40.0;

const float KD_START = 130.0;   // Tăng nhẹ KD nền để tương thích với KP lớn
const float KD_END   = 200.0;   
const float KD_STEP  = 6.0;

const float KI_FIXED = 2.0;     
const unsigned long TEST_TIME_PER_SET = 3000; // Thời gian test mỗi cặp tham số (ms)

// Biến điều khiển hệ thống
float angle = 0.0;
float zeroOffset = 0.0;
float integralTerm = 0.0;
int16_t gyroXoffset = 0;
float globalGyroRate = 0.0;

// Máy trạng thái Tuning
enum TuningState { STATE_WAIT_VERTICAL, STATE_COUNTDOWN, STATE_RUNNING, STATE_DONE };
TuningState currentState = STATE_WAIT_VERTICAL;

float currentKp = KP_START;
float currentKd = KD_START;
unsigned long stateTimer = 0;
float currentScore = 0;
int countdownCount = 0;

float bestKp = 0.0; 
float bestKd = 0.0; 
float bestScore = 999999.0;

void calibrateMPU();
void calibrateZeroAngle();
void setMotorSpeeds(float leftControl, float rightControl);
void moveToNextParam();
float updateAngle(float dt);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Tăng tốc độ I2C để đọc dữ liệu MPU nhanh nhất
  
  pinMode(MOTOR_LEFT_STEP, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_STEP, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Bật Driver

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Tuning Kp>=200");

  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear(); lcd.print("MPU6050 ERROR!");
    while (1) delay(100);
  }
  calibrateMPU();

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  angle = atan2((float)ax, (float)az) * 180.0 / PI;
  if (INVERT_ANGLE) angle = -angle;
  calibrateZeroAngle();

  Serial.println("\n========================================================");
  Serial.println("  START GRID SEARCH TUNING - Kp BẮT ĐẦU TỪ 200.0");
  Serial.println("========================================================");

  stateTimer = millis();
}

void loop() {
  static unsigned long lastPIDTime = 0;
  unsigned long now = millis();
  
  // Chu kỳ PID chạy cố định 10ms (100Hz)
  if (now - lastPIDTime < 10) return; 
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  float calibratedAngle = updateAngle(dt);

  switch (currentState) {
    case STATE_WAIT_VERTICAL:
      setMotorSpeeds(0, 0);
      if (fabs(calibratedAngle - setpoint) < 0.8) {
        currentState = STATE_COUNTDOWN;
        stateTimer = millis();
        countdownCount = 2; 
      } else {
        static unsigned long lcdPrintTimer = 0;
        if (millis() - lcdPrintTimer > 200) {
          lcdPrintTimer = millis();
          lcd.clear();
          lcd.setCursor(0, 0); lcd.print("GIU XE THANG!");
          lcd.setCursor(0, 1); lcd.print("Goc: "); lcd.print(calibratedAngle, 1);
        }
      }
      break;

    case STATE_COUNTDOWN:
      setMotorSpeeds(0, 0);
      if (fabs(calibratedAngle - setpoint) > 1.5) {
        currentState = STATE_WAIT_VERTICAL;
        break;
      }
      if (millis() - stateTimer >= 1000) {
        stateTimer = millis();
        countdownCount--;
        if (countdownCount <= 0) {
          integralTerm = 0; currentScore = 0;
          lcd.clear();
          lcd.setCursor(0, 0); lcd.print("REALTIME TUNING");
          lcd.setCursor(0, 1); lcd.print("P:"); lcd.print(currentKp,0); lcd.print(" D:"); lcd.print(currentKd,1);
          
          currentState = STATE_RUNNING;
          stateTimer = millis(); 
        } else {
          lcd.clear();
          lcd.setCursor(0, 0); lcd.print("BUONG TAY SAU...");
          lcd.setCursor(0, 1); lcd.print(countdownCount); lcd.print("s");
        }
      }
      break;

    case STATE_RUNNING:
      if (fabs(calibratedAngle - setpoint) > FALL_ANGLE) {
        Serial.print("Kp="); Serial.print(currentKp, 1); 
        Serial.print(" | Kd="); Serial.print(currentKd, 1); 
        Serial.println(" -> [XE NGÃ]");
        moveToNextParam();
        break;
      }

      if (millis() - stateTimer >= TEST_TIME_PER_SET) {
        Serial.print("Kp="); Serial.print(currentKp, 1);
        Serial.print(" | Kd="); Serial.print(currentKd, 1);
        Serial.print(" -> ĐẠT! Score: "); Serial.println(currentScore, 2);

        if (currentScore < bestScore) {
          bestScore = currentScore; 
          bestKp = currentKp; 
          bestKd = currentKd;
        }
        moveToNextParam();
        break;
      }

      {
        float error = calibratedAngle - setpoint;
        integralTerm += error * dt;
        integralTerm = constrain(integralTerm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        float deriv = globalGyroRate; 

        float control = (currentKp * error) + (KI_FIXED * integralTerm) + (currentKd * deriv);
        
        setMotorSpeeds(control, control);
        currentScore += (fabs(error) * dt);
      }
      break;

    case STATE_DONE:
      setMotorSpeeds(0, 0);
      break;
  }
}

// ===================================================
// HÀM CẬP NHẬT GÓC NGHIÊNG
// ===================================================
float updateAngle(float dt) {
  int16_t ax, ay, az, gx;
  mpu.getAcceleration(&ax, &ay, &az);
  gx = mpu.getRotationX();

  float accAngle = atan2((float)ax, (float)az) * 180.0 / PI;
  globalGyroRate = ((float)gx - gyroXoffset) / 131.0;
  
  if (INVERT_ANGLE) { 
    accAngle = -accAngle; 
    globalGyroRate = -globalGyroRate; 
  }

  angle = ALPHA * (angle + globalGyroRate * dt) + (1.0 - ALPHA) * accAngle;
  return angle - zeroOffset;
}

// ===================================================
// HÀM PHÁT XUNG ĐIỀU TỐC TRỰC TIẾP QUA VÒNG LẶP
// ===================================================
void setMotorSpeeds(float leftControl, float rightControl) {
  if (fabs(leftControl) < 0.5) {
    return;
  }

  bool dirL = (leftControl > 0);
  bool dirR = (rightControl > 0);

  if (INVERT_LEFT)  dirL = !dirL;
  if (INVERT_RIGHT) dirR = !dirR;

  digitalWrite(MOTOR_LEFT_DIR, dirL);
  digitalWrite(MOTOR_RIGHT_DIR, dirR);

  // Tính chu kỳ trễ dựa vào lực xuất từ PID
  long pulseDelay = 1000000 / (fabs(leftControl) * 20);
  
  // Chặn đáy ở 400us để xe giật đảo chiều nhanh mà không bị khóa/mất bước động cơ cơ khí
  pulseDelay = constrain(pulseDelay, 400, 5000); 

  // Tạo 1 chu kỳ xung vuông bằng delay mềm
  digitalWrite(MOTOR_LEFT_STEP, HIGH);
  digitalWrite(MOTOR_RIGHT_STEP, HIGH);
  delayMicroseconds(pulseDelay);
  digitalWrite(MOTOR_LEFT_STEP, LOW);
  digitalWrite(MOTOR_RIGHT_STEP, LOW);
  delayMicroseconds(pulseDelay);
}

void moveToNextParam() {
  setMotorSpeeds(0, 0);
  currentKd += KD_STEP;
  if (currentKd > KD_END) {
    currentKd = KD_START;
    currentKp += KP_STEP;
  }

  if (currentKp > KP_END) {
    currentState = STATE_DONE;
    Serial.println("\n========================================================");
    Serial.println("  QUÁ TRÌNH TỰ ĐỘNG TUNING HOÀN TẤT TUYỆT ĐỐI! ");
    Serial.print(" => Kp TỐT NHẤT: "); Serial.println(bestKp);
    Serial.print(" => Kd TỐT NHẤT: "); Serial.println(bestKd);
    Serial.print(" => Sai số thấp nhất: "); Serial.println(bestScore);
    Serial.println("========================================================");

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("TUNING DONE!");
    lcd.setCursor(0, 1); lcd.print("P:"); lcd.print(bestKp,1); lcd.print(" D:"); lcd.print(bestKd,1);
  } else {
    currentState = STATE_WAIT_VERTICAL;
  }
}

void calibrateZeroAngle() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float a = atan2((float)ax, (float)az) * 180.0 / PI;
  if (INVERT_ANGLE) a = -a;
  zeroOffset = a;
  angle = 0.0;
}

void calibrateMPU() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calibrating MPU...");
  delay(1000);
  const int samples = 300;
  long gx_sum = 0;
  for (int i = 0; i < samples; i++) {
    gx_sum += mpu.getRotationX();
    delay(2);
  }
  gyroXoffset = (int16_t)(gx_sum / samples);
}