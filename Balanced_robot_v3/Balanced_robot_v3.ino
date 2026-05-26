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

// SRF05 ultrasonic sensor
#define SRF05_TRIG_PIN      10   // D10 -> TRIG
#define SRF05_ECHO_PIN      9    // D9  -> ECHO
// Only measure up to this distance (cm) — caps pulseIn() blocking time
// 100cm -> max blocking = 100*58 = 5800us (~5.8ms) every SRF05_INTERVAL_MS
#define SRF05_MAX_CM        400
#define SRF05_TIMEOUT_US    ((long)SRF05_MAX_CM * 58L)
#define SRF05_INTERVAL_MS   100   // Read sensor every 100ms

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
float Kp_low = 1800.0f;
float Ki_low = 120.0f;
float Kd_low = 259.0f;

// HIGH: |error| >= GAIN_SWITCH_ANGLE -> far from balance, aggressive recovery
float Kp_high = 4200.0f;
float Ki_high = 180.0f;
float Kd_high = 500.0f;

// Angle threshold to switch between LOW and HIGH gains (degrees)
#define GAIN_SWITCH_ANGLE 1.1f

// Hysteresis band to prevent oscillation between gain modes (degrees)
// HIGH: |error| > GAIN_SWITCH_ANGLE + HYSTERESIS_BAND
// LOW:  |error| < GAIN_SWITCH_ANGLE - HYSTERESIS_BAND
#define HYSTERESIS_BAND 0.5f

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
#define MAX_SPEED (1000 * MICROSTEP)

// Anti-windup clamp for integral term
#define INTEGRAL_LIMIT 300.0f

// Disable motors if robot tilts beyond this angle (fallen over)
#define FALL_ANGLE 30.0f

// PID update interval in milliseconds (10ms = 100Hz)
#define PID_INTERVAL_MS 10

// Right motor direction flip (set true if right motor spins wrong way)
#define REVERSE_RIGHT true

// Wheel diameter (mm) — used to convert step count to real distance
// Measure your wheel's outer diameter and update this value
#define WHEEL_DIAMETER_MM   65.0f

// ---- Return to home (outer PI cascade) ----
// Controls the robot's position by nudging the balance setpoint.
// P: immediate lean proportional to distance from home
// I: slowly builds lean to overcome friction and steady-state drift
// Both gains are in degrees/cm (P) and degrees/(cm*s) (I)
#define HOME_KP         0.01f  // smaller = smoother return (less overshoot/faster speed)
#define HOME_KI         0.02f  // helps overcome friction; 0 to disable
#define HOME_I_LIMIT    25.0f   // anti-windup clamp (cm*s)
#define HOME_MAX_DEG    1.0f    // max angle correction — robot returns home gradually

// ---- Upright detection (startup) ----
// Set true : robot starts lying flat, lifts to vertical to arm
// Set false: robot starts already upright, arms immediately after warm-up
#define LIFT_TO_START       true
// Robot must rotate this many degrees from its lying angle to be considered upright
// e.g. lying at ~0 deg, upright at ~90 deg -> set 60 as safe midpoint
#define UPRIGHT_DELTA_DEG   60.0f
// Robot must hold upright steadily for this long before PID arms (ms)
#define UPRIGHT_HOLD_MS     1500

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
unsigned long lastSRFTime = 0;
int           distanceCm  = 0;   // Latest SRF05 reading (0 = out of range)
float         homeIntegral = 0.0f; // Integral for return-to-home PI loop
bool          useHighGain = false; // Gain scheduling state (HIGH/LOW)
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
  pinMode(SRF05_TRIG_PIN, OUTPUT);
  pinMode(SRF05_ECHO_PIN, INPUT);
  digitalWrite(SRF05_TRIG_PIN, LOW);

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

#if LIFT_TO_START
  // ── Phase 1: Calibrate GYRO ONLY while flat ──────────────────
  // Gyro offsets are orientation-independent — safe to calibrate while lying.
  // We skip accel calibration here; doing it while flat would make 0°=lying
  // instead of 0°=upright, which would break the PID setpoint.
  Serial.println("[INFO] Calibrating gyro while flat...");
  lcd.setCursor(0, 0); lcd.print(" Calibrating... ");
  lcd.setCursor(0, 1); lcd.print("  Keep flat!    ");
  delay(500);
  mpu.calcOffsets(true, false);   // GYRO ONLY
  Serial.println("[INFO] Gyro calibrated.");

  // ── Phase 2: Warm up filter, record lying angle as reference ───
  Serial.println("[INFO] Warming up filter while flat...");
  lcd.setCursor(0, 0); lcd.print(" Warming up...  ");
  lcd.setCursor(0, 1); lcd.print("  Keep flat!    ");
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 1500) {
      mpu.update();
      delay(5);
    }
  }
  smoothedAngle = mpu.getAngleY();
  float lyingAngle = smoothedAngle;        // Reference: angle when lying
  Serial.print("[INFO] Lying angle: ");
  Serial.println(lyingAngle, 2);

  // ── Phase 3: Wait for robot to be lifted upright ────────────────
  Serial.println("[INFO] Lift robot upright to start!");
  lcd.setCursor(0, 0); lcd.print(" Lift to START! ");
  lcd.setCursor(0, 1); lcd.print(" (hold steady)  ");
  setupTimer1();   // Start ISR now so step-pulse logic is ready when we arm
  {
    unsigned long uprightSince = 0;
    bool uprightDetected = false;
    while (true) {
      mpu.update();
      float ang = mpu.getAngleY();
      smoothedAngle += ANGLE_LPF_ALPHA * (ang - smoothedAngle);

      bool isUpright = (fabs(smoothedAngle - lyingAngle) > UPRIGHT_DELTA_DEG);
      if (isUpright) {
        if (!uprightDetected) {
          uprightDetected = true;
          uprightSince    = millis();
          Serial.println("[INFO] Upright detected, hold steady...");
          lcd.setCursor(0, 0); lcd.print("Stand detected! ");
          lcd.setCursor(0, 1); lcd.print(" Hold steady... ");
        }
        if (millis() - uprightSince >= UPRIGHT_HOLD_MS) {
          break;   // Held upright and stable long enough
        }
      } else {
        if (uprightDetected) {
          uprightDetected = false;
          Serial.println("[INFO] Lost upright, keep lifting...");
          lcd.setCursor(0, 0); lcd.print(" Lift to START! ");
          lcd.setCursor(0, 1); lcd.print(" (hold steady)  ");
        }
      }
      delay(10);
    }
  }

  // ── Phase 4: Full recalibration at upright position ─────────────
  // Robot is now vertical and stable. Recalibrate accel+gyro so that
  // upright = 0° — exactly what the PID setpoint expects.
  Serial.println("[INFO] Recalibrating at upright...");
  lcd.setCursor(0, 0); lcd.print(" Recalibrating  ");
  lcd.setCursor(0, 1); lcd.print("  Stay still!   ");
  mpu.calcOffsets(true, true);
  Serial.println("[INFO] Upright calibration done.");

#else
  // ── LIFT_TO_START disabled: calibrate upright immediately ────────
  Serial.println("[INFO] Calibrating while upright...");
  lcd.setCursor(0, 0); lcd.print(" Calibrating... ");
  lcd.setCursor(0, 1); lcd.print(" Hold upright!  ");
  delay(500);
  mpu.calcOffsets(true, true);   // Full calibration at upright
  Serial.println("[INFO] Calibration done.");
  setupTimer1();

#endif

  // ── Warm-up / final settle (both modes) ──────────────────────────
  lcd.setCursor(0, 0); lcd.print(" Warming up...  ");
  lcd.setCursor(0, 1); lcd.print(" Hold still!    ");
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 500) {
      mpu.update();
      delay(5);
    }
  }
  smoothedAngle = mpu.getAngleY();
  Serial.print("[INFO] Settled angle: ");
  Serial.println(smoothedAngle, 2);
  lcd.setCursor(0, 0); lcd.print("    Ready!      ");
  {
    char abuf[8]; dtostrf(smoothedAngle, 6, 2, abuf);
    char lbuf[17]; snprintf(lbuf, sizeof(lbuf), "Angle:%s deg ", abuf);
    lcd.setCursor(0, 1); lcd.print(lbuf);
  }

  // ── Phase 5: Arm PID ─────────────────────────────────────────────
  mpu.update();
  smoothedAngle = mpu.getAngleY();
  prevError = setpoint - smoothedAngle;   // Pre-seed D-term — no derivative kick
  integral  = 0.0f;
  dFiltered = 0.0f;
  useHighGain = false;  // Reset gain scheduling on startup

  digitalWrite(ENABLE_PIN, LOW);
  motorsEnabled = true;
  lastPIDTime   = millis();
  lastLCDTime   = millis();
  Serial.println("[INFO] Running.");
}

// ============================================================
// SRF05 — returns distance in cm, 0 if out of range / no echo
// Uses micros() instead of pulseIn() — pulseIn() uses a tight assembly
// counting loop that gets corrupted by the Timer1 ISR firing every 100us.
// micros() is Timer0-based and works correctly regardless of Timer1.
// ============================================================
int readSRF05() {
  // 10us trigger pulse
  digitalWrite(SRF05_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SRF05_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SRF05_TRIG_PIN, LOW);

  // Wait for ECHO to go HIGH (timeout 5ms — sensor not responding)
  unsigned long t0 = micros();
  while (digitalRead(SRF05_ECHO_PIN) == LOW) {
    if ((micros() - t0) > 5000UL) return 0;
  }

  // Measure how long ECHO stays HIGH
  unsigned long echoStart = micros();
  while (digitalRead(SRF05_ECHO_PIN) == HIGH) {
    if ((micros() - echoStart) > (unsigned long)SRF05_TIMEOUT_US) return 0;
  }

  long duration = (long)(micros() - echoStart);
  return (int)(duration / 58L);   // us -> cm
}

// ============================================================
// LCD UPDATE — called every 250ms, non-blocking
// ============================================================
void updateLCD(float angle, float odo, int dist, bool fallen, bool highGain, char motorDir) {
  char buf[17];
  char abuf[8];

  lcd.setCursor(0, 0);
  if (fallen) {
    lcd.print("  ** FALLEN **  ");
    lcd.setCursor(0, 1);
    lcd.print(" Tilt to reset  ");
    return;
  }

  // Line 1: "A:-1.23 G:L M:F" (16 chars)
  dtostrf(angle, 6, 2, abuf);
  snprintf(buf, sizeof(buf), "A:%s G:%c M:%c", abuf, highGain ? 'H' : 'L', motorDir);
  lcd.print(buf);

  // Line 2: "O:-123cm D: 45cm" (16 chars)
  //   O: odometry (cm from home), D: SRF05 distance
  lcd.setCursor(0, 1);
  if (dist > 0) {
    snprintf(buf, sizeof(buf), "O:%4dcm D:%3dcm", (int)odo, dist);
  } else {
    snprintf(buf, sizeof(buf), "O:%4dcm D:---cm", (int)odo);
  }
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

  // --- SRF05: read every SRF05_INTERVAL_MS (decoupled from PID rate) ---
  if (now - lastSRFTime >= SRF05_INTERVAL_MS) {
    lastSRFTime = now;
    distanceCm  = readSRF05();
  }

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
    hasFallen     = false;
    motorsEnabled = true;
    cli(); positionSteps = 0; sei();  // Reset position origin on recovery
    homeIntegral  = 0.0f;            // Reset home PI integral
    useHighGain   = false;           // Reset gain scheduling on recovery
    digitalWrite(ENABLE_PIN, LOW);
  }

  if (!motorsEnabled) return;

  // --- Return to home: outer PI cascade controller ---
  // odoCm > 0 = robot ahead of home; < 0 = robot behind home
  cli();
  long posSnap = positionSteps;
  sei();
  float odoCm = (float)posSnap * (PI * WHEEL_DIAMETER_MM)
                / (200.0f * MICROSTEP * 10.0f);

  homeIntegral += odoCm * dt;
  homeIntegral  = constrain(homeIntegral, -HOME_I_LIMIT, HOME_I_LIMIT);
  float posCorrection = constrain(
    +(HOME_KP * odoCm + HOME_KI * homeIntegral),
    -HOME_MAX_DEG, HOME_MAX_DEG
  );

  // --- PID calculation ---
  float error = (setpoint + posCorrection) - smoothedAngle;

  // Integral with anti-windup clamp
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Derivative with low-pass filter (prevents noise amplification)
  float dRaw = (error - prevError) / dt;
  dFiltered += DTERM_LPF_ALPHA * (dRaw - dFiltered);
  prevError = error;

  // --- Gain scheduling with hysteresis ---
  // Prevents oscillation between LOW and HIGH near the threshold
  // HIGH: when |error| crosses GAIN_SWITCH_ANGLE + HYSTERESIS_BAND
  // LOW:  when |error| drops below GAIN_SWITCH_ANGLE - HYSTERESIS_BAND
  float absError = abs(error);
  if (absError > (GAIN_SWITCH_ANGLE + HYSTERESIS_BAND)) {
    useHighGain = true;   // Far from balance -> aggressive
  } else if (absError < (GAIN_SWITCH_ANGLE - HYSTERESIS_BAND)) {
    useHighGain = false;  // Close to balance -> soft
  }
  // else: stay in current state (hysteresis band)

  float Kp_active = useHighGain ? Kp_high : Kp_low;
  float Ki_active = useHighGain ? Ki_high : Ki_low;
  float Kd_active = useHighGain ? Kd_high : Kd_low;

  float pidOut = Kp_active * error + Ki_active * integral + Kd_active * dFiltered;
  pidOut = constrain(pidOut, -(float)MAX_SPEED, (float)MAX_SPEED);

  // --- Dead band: cut tiny outputs that only cause jitter ---
  if (abs(pidOut) < DEAD_BAND) {
    pidOut = 0;
  }

  char motorDir = 'S';
  if (pidOut > 0) {
    motorDir = 'F';
  } else if (pidOut < 0) {
    motorDir = 'B';
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
    updateLCD(smoothedAngle, odoCm, distanceCm, hasFallen, useHighGain, motorDir);
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
  Serial.print(" Odo:");
  Serial.print(odoCm, 1);
  Serial.print("cm SRF:");
  Serial.print(distanceCm);
  Serial.print("cm G:");
  Serial.print(useHighGain ? "HIGH" : "LOW");
  Serial.println();
}
