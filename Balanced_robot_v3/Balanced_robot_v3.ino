/**
Self-Balancing Robot - Optimized for Arduino Uno R3
Hardware: Arduino Uno R3 + CNC Shield V3 + 2x DRV8825 + 2x NEMA17 + MPU6050
Optimizations vs reference code:
  Timer1 ISR for step generation -> non-blocking, precise timing
  Manual PID (no library) -> D-term low-pass filter built-in
  Anti-windup on integral
  Dead band to cut micro-jitter near balance
  Fall detection -> auto disable motors
  Alpha = 0.1 (lower than reference 0.3) -> less vibration noise
CNC Shield V3 Pin Mapping:
  X axis -> Left motor:  STEP=2, DIR=5
  Y axis -> Right motor: STEP=3, DIR=6
  Enable (shared, active LOW): pin 8
MPU6050: SDA=A4, SCL=A5 (standard Uno I2C)
DRV8825 Microstepping (set via jumpers on CNC Shield):
  No jumper  = Full step (1/1)
  M0 only    = Half step (1/2)
  M1 only    = 1/4 step
  M0+M1      = 1/8 step  <- Recommended
  M2 only    = 1/16 step
  All jumpers= 1/32 step
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// PIN DEFINITIONS (CNC Shield V3)
// ============================================================
#define STEP_PIN_L 2  // X axis STEP
#define DIR_PIN_L 5   // X axis DIR
#define STEP_PIN_R 3  // Y axis STEP
#define DIR_PIN_R 6   // Y axis DIR
#define ENABLE_PIN 8  // Shared enable, active LOW

// LCD I2C
// Common addresses: 0x27 (PCF8574T) or 0x3F (PCF8574AT)
// Run an I2C scanner sketch if unsure which address your module uses
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// ============================================================
// TUNING PARAMETERS - Adjust these first
// ============================================================

// PID gains - 2-level gain scheduling
// LOW: |error| < GAIN_SWITCH_ANGLE  -> near balance, soft response, less jitter
float Kp_low = 1900.0f;
float Ki_low = 80.0f;
float Kd_low = 150.0f;

// HIGH: |error| >= GAIN_SWITCH_ANGLE -> far from balance, aggressive recovery
float Kp_high = 4000.0f;
float Ki_high = 120.0f;
float Kd_high = 350.0f;

// Angle threshold to switch between LOW and HIGH gains (degrees)
#define GAIN_SWITCH_ANGLE 2.0f

// Balance point: angle where robot is truly vertical
// Set to 0.0, run, observe which way robot falls, adjust by +/-1 steps
float setpoint = 0.0f;

// Low-pass filter for angle (0.0-1.0, lower = smoother but more lag)
// 0.1 is much safer than reference 0.3 to prevent motor vibration feedback
#define ANGLE_LPF_ALPHA 0.10f

// Low-pass filter for D-term (prevent derivative kick amplifying noise)
#define DTERM_LPF_ALPHA 0.25f

// Microstepping mode — must match jumper setting on CNC Shield
// 1=Full, 2=Half, 4=1/4, 8=1/8, 16=1/16, 32=1/32
#define MICROSTEP 16

// Dead band: ignore PID output below this (steps/sec)
// Scales with MICROSTEP so physical threshold stays the same
#define DEAD_BAND ((60 * MICROSTEP) / 8)

// Max motor speed in steps/sec
// DRV8825 + NEMA17 base: 2000 steps/sec at 1/8 -> scales with MICROSTEP
#define MAX_SPEED (500 * MICROSTEP)

// Anti-windup clamp for integral term
#define INTEGRAL_LIMIT 300.0f

// Disable motors if robot tilts beyond this angle (fallen over)
#define FALL_ANGLE 45.0f

// PID update interval in milliseconds (10ms = 100Hz)
#define PID_INTERVAL_MS 10

// Right motor direction flip (set true if right motor spins wrong way)
#define REVERSE_RIGHT true

// ---- Position hold ----
// Outer P-loop: converts net displacement (steps) to setpoint angle nudge
// Too large -> robot oscillates back-and-forth; start at 0.002 and increase slowly
#define POS_HOLD_GAIN 0.006f
// Max angle correction the position loop can demand (degrees)
#define POS_HOLD_MAX 1.0f

// ============================================================
// TIMER1 ISR VARIABLES (volatile = shared with interrupt)
// ============================================================
// Timer1 fires every 100us (10kHz). Each ISR tick is 1 unit.
// To step at N steps/sec: fire STEP every (10000/N) ticks
volatile int isrMotorSpeed = 0;  // Updated from PID loop
volatile int isrTickCount1 = 0;  // Tick counter motor L
volatile int isrTickCount2 = 0;  // Tick counter motor R
volatile bool isrDirLeft = true;
volatile bool isrDirRight = true;
volatile bool stepPinLState = false;
volatile bool stepPinRState = false;
volatile long positionSteps = 0;  // Net steps from start (+ = forward)

// ============================================================
// APPLICATION VARIABLES
// ============================================================
MPU6050 mpu(Wire);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

float smoothedAngle = 0.0f;
float prevError = 0.0f;
float integral = 0.0f;
float dFiltered = 0.0f;
unsigned long lastPIDTime = 0;
unsigned long lastLCDTime = 0;
bool motorsEnabled = false;
bool hasFallen = false;

// ============================================================
// TIMER1 SETUP (CTC mode, 10kHz)
// ============================================================
void setupTimer1() {
  // Stop interrupts during setup
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // CTC mode (WGM12), prescaler 8 -> 16MHz/8 = 2MHz tick
  // Compare value: 2MHz / 10kHz - 1 = 199
  OCR1A = 199;
  TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC + prescaler 8

  // Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  sei();
}

// ============================================================
// TIMER1 ISR - Runs every 100us, handles step pulses
// ============================================================
ISR(TIMER1_COMPA_vect) {
  int spd = isrMotorSpeed;
  if (spd == 0) return;

  // Ticks per step = 10000 / |speed|  (10000 ticks = 1 second at 10kHz)
  int ticksPerStep = 10000 / spd;  // spd is already abs value here

  // Motor L
  isrTickCount1++;
  if (isrTickCount1 >= ticksPerStep) {
    isrTickCount1 = 0;
    // Generate step pulse (HIGH then LOW next iteration)
    if (!stepPinLState) {
      digitalWrite(STEP_PIN_L, HIGH);
      stepPinLState = true;
      positionSteps += isrDirLeft ? 1 : -1;  // Count on rising edge
    } else {
      digitalWrite(STEP_PIN_L, LOW);
      stepPinLState = false;
    }
  }

  // Motor R (identical timing, independent counter for jitter isolation)
  isrTickCount2++;
  if (isrTickCount2 >= ticksPerStep) {
    isrTickCount2 = 0;
    if (!stepPinRState) {
      digitalWrite(STEP_PIN_R, HIGH);
      stepPinRState = true;
    } else {
      digitalWrite(STEP_PIN_R, LOW);
      stepPinRState = false;
    }
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(STEP_PIN_L, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(STEP_PIN_R, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Disable motors during init (DRV8825: HIGH = disabled)
  digitalWrite(ENABLE_PIN, HIGH);

  // LCD init (before MPU so it shows errors too)
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Self-Balance Bot");
  lcd.setCursor(0, 1);
  lcd.print("  Initializing  ");

  // MPU6050 init
  Wire.begin();
  Wire.setClock(400000);  // Fast I2C 400kHz -> reduce read latency
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("[ERROR] MPU6050 not found. Check wiring.");
    lcd.setCursor(0, 0);
    lcd.print("MPU6050  ERROR! ");
    lcd.setCursor(0, 1);
    lcd.print(" Check wiring!  ");
    while (true)
      ;
  }

  // Calibrate - keep robot perfectly still on flat surface
  Serial.println("[INFO] Calibrating MPU6050... Do NOT move robot.");
  lcd.setCursor(0, 0);
  lcd.print(" Calibrating... ");
  lcd.setCursor(0, 1);
  lcd.print(" Do NOT move!   ");
  delay(500);
  mpu.calcOffsets(true, true);  // Calibrate both gyro and accel
  Serial.println("[INFO] Calibration done.");

  // Warm up MPU complementary filter — must loop update() for ~1.5s to converge.
  // A single mpu.update() right after calcOffsets() gives an inaccurate angle
  // because the gyro integral hasn't had time to blend with the accelerometer.
  Serial.println("[INFO] Warming up angle filter... hold robot still.");
  lcd.setCursor(0, 0);
  lcd.print(" Warming up...  ");
  lcd.setCursor(0, 1);
  lcd.print(" Hold still!    ");
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 1500) {
      mpu.update();
      delay(5);
    }
  }
  smoothedAngle = mpu.getAngleY();
  Serial.print("[INFO] Settled angle: ");
  Serial.println(smoothedAngle, 2);
  {
    char abuf[8];
    dtostrf(smoothedAngle, 6, 2, abuf);
    char lbuf[17];
    snprintf(lbuf, sizeof(lbuf), "Angle:%s deg ", abuf);
    lcd.setCursor(0, 0);
    lcd.print("    Ready!      ");
    lcd.setCursor(0, 1);
    lcd.print(lbuf);
  }

  // Start Timer1 ISR for step generation
  setupTimer1();

  // Final pre-arm read: seed all filter states so PID has clean initial values
  mpu.update();
  smoothedAngle = mpu.getAngleY();
  prevError = setpoint - smoothedAngle;  // Pre-seed D-term — no derivative kick
  integral = 0.0f;
  dFiltered = 0.0f;

  // Enable motors
  digitalWrite(ENABLE_PIN, LOW);
  motorsEnabled = true;
  lastPIDTime = millis();
  lastLCDTime = millis();
  Serial.println("[INFO] Running.");
}

// ============================================================
// LCD UPDATE — called every 250ms, non-blocking
// ============================================================
void updateLCD(float angle, float error, int speed, long pos, bool fallen) {
  char buf[17];
  char abuf[8];

  lcd.setCursor(0, 0);
  if (fallen) {
    lcd.print("  ** FALLEN **  ");
    lcd.setCursor(0, 1);
    lcd.print(" Tilt to reset  ");
    return;
  }

  // Line 1: "A:-1.23 G:LOW  " (16 chars)
  dtostrf(angle, 6, 2, abuf);
  bool isLow = (abs(error) < GAIN_SWITCH_ANGLE);
  snprintf(buf, sizeof(buf), "A:%s G:%-3s  ", abuf, isLow ? "LOW" : "HI");
  lcd.print(buf);

  // Line 2: "S:4000  P:02756 " (16 chars)
  lcd.setCursor(0, 1);
  snprintf(buf, sizeof(buf), "S:%-4d  P:%-5ld ", speed, pos);
  lcd.print(buf);
}

// ============================================================
// MOTOR DIRECTION HELPER
// ============================================================
void setMotorDirections(int speed) {
  bool goForward = (speed > 0);

  isrDirLeft = goForward;  // Sync to ISR so step counter knows direction
  isrDirRight = goForward;

  digitalWrite(DIR_PIN_L, goForward ? HIGH : LOW);

  // Right motor is mechanically mirrored -> flip direction if needed
  if (REVERSE_RIGHT) {
    digitalWrite(DIR_PIN_R, goForward ? LOW : HIGH);
  } else {
    digitalWrite(DIR_PIN_R, goForward ? HIGH : LOW);
  }
}

// ============================================================
// MAIN LOOP - PID runs here, motor steps in ISR
// ============================================================
void loop() {
  unsigned long now = millis();

  if (now - lastPIDTime < PID_INTERVAL_MS) return;

  float dt = (now - lastPIDTime) / 1000.0f;
  lastPIDTime = now;

  // --- Read & filter angle ---
  mpu.update();
  float rawAngle = mpu.getAngleY();
  smoothedAngle += ANGLE_LPF_ALPHA * (rawAngle - smoothedAngle);

  // --- Fall detection ---
  if (abs(smoothedAngle) > FALL_ANGLE) {
    if (!hasFallen) {
      hasFallen = true;
      motorsEnabled = false;
      digitalWrite(ENABLE_PIN, HIGH);  // Cut motor power
      isrMotorSpeed = 0;
      integral = 0.0f;  // Reset integral on fall
      Serial.println("[WARN] Robot fallen. Re-balance to resume.");
    }
    return;
  }

  // --- Re-enable after recovery ---
  if (hasFallen && abs(smoothedAngle) < 5.0f) {
    hasFallen = false;
    motorsEnabled = true;
    cli();
    positionSteps = 0;
    sei();  // Reset position origin on recovery
    digitalWrite(ENABLE_PIN, LOW);
  }

  if (!motorsEnabled) return;

  // --- Position hold: nudge setpoint to pull robot back to origin ---
  cli();
  long posSnap = positionSteps;
  sei();
  float posCorrection = constrain(-POS_HOLD_GAIN * (float)posSnap,
                                  -POS_HOLD_MAX, POS_HOLD_MAX);

  // --- PID calculation ---
  float error = (setpoint + posCorrection) - smoothedAngle;

  // Integral with anti-windup clamp
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Derivative with low-pass filter (prevents noise amplification)
  float dRaw = (error - prevError) / dt;
  dFiltered += DTERM_LPF_ALPHA * (dRaw - dFiltered);
  prevError = error;

  // --- Gain scheduling: pick LOW or HIGH set based on error magnitude ---
  float Kp_active = (abs(error) < GAIN_SWITCH_ANGLE) ? Kp_low : Kp_high;
  float Ki_active = (abs(error) < GAIN_SWITCH_ANGLE) ? Ki_low : Ki_high;
  float Kd_active = (abs(error) < GAIN_SWITCH_ANGLE) ? Kd_low : Kd_high;

  float pidOut = Kp_active * error + Ki_active * integral + Kd_active * dFiltered;
  pidOut = constrain(pidOut, -(float)MAX_SPEED, (float)MAX_SPEED);

  // --- Dead band: cut tiny outputs that only cause jitter ---
  if (abs(pidOut) < DEAD_BAND) {
    pidOut = 0;
  }

  int newSpeed = (int)abs(pidOut);

  // --- Update direction pins (outside ISR for safety) ---
  setMotorDirections((int)pidOut);

  // --- Atomically update ISR speed (disable interrupt briefly) ---
  cli();
  isrMotorSpeed = newSpeed;
  isrTickCount1 = 0;  // Reset counters on speed change to avoid glitch
  isrTickCount2 = 0;
  sei();

  // --- LCD update (every 250ms — non-blocking) ---
  if (now - lastLCDTime >= 250) {
    lastLCDTime = now;
    updateLCD(smoothedAngle, error, newSpeed, posSnap, hasFallen);
  }

  // --- Debug output (comment out to reduce loop time) ---
  Serial.print("A:");
  Serial.print(smoothedAngle, 2);
  Serial.print(" E:");
  Serial.print(error, 2);
  Serial.print(" I:");
  Serial.print(integral, 2);
  Serial.print(" D:");
  Serial.print(dFiltered, 2);
  Serial.print(" S:");
  Serial.print(newSpeed);
  Serial.print(" P:");
  Serial.print(posSnap);
  Serial.print(" G:");
  Serial.println((abs(error) < GAIN_SWITCH_ANGLE) ? "LOW" : "HIGH");
}
