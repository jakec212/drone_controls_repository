// ============================================================
//  Flight Controller — Teensy + nRF
24L01 + MPU6050
//  X-frame quadcopter, rate (acro) mode
//  Loop target: 4 ms (250 Hz)
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>


// ============================================================
//  HARDWARE PINS
// ============================================================
// nRF24:  CE=9, CSN=10  (hardware SPI: MOSI=11, MISO=12, SCK=13)
// MPU6050: SDA/SCL via Wire (Teensy default: SDA=18, SCL=19)
// Motors:  pins 1, 2, 3, 4  (Teensy PWM capable)
// Battery: analog pin A0 (voltage divider), A1 (current sensor)
// LED:     pin 13 (built-in)


// ============================================================
//  RATE LIMITS  (degrees / second)
// ============================================================
const float MAX_ROLL_RATE  = 60.0f;
const float MAX_PITCH_RATE = 60.0f;
const float MAX_YAW_RATE   = 30.0f;


// ============================================================
//  PID GAINS  — tune these after the drone hovers
//  Start with I=0 and D=0, increase P until it oscillates,
//  then back off ~30%.  Then add I to remove drift.
//  D is optional; leave at 0 until P and I are solid.
// ============================================================
float P_Roll  = 0.6f,  I_Roll  = 3.5f,  D_Roll  = 0.03f;
float P_Pitch = 0.6f,  I_Pitch = 3.5f,  D_Pitch = 0.03f;
float P_Yaw   = 2.0f,  I_Yaw   = 12.0f, D_Yaw   = 0.0f;


// ============================================================
//  PID STATE  — one struct per axis keeps state isolated
// ============================================================
struct PIDState {
  float prevError;
  float iterm;
};
PIDState pidRoll  = {0, 0};
PIDState pidPitch = {0, 0};
PIDState pidYaw   = {0, 0};


// ============================================================
//  GYRO STATE  (degrees/second, calibration-corrected)
// ============================================================
float currentRoll  = 0;
float currentPitch = 0;
float currentYaw   = 0;


// ============================================================
//  DESIRED RATES  (set by pilot sticks, degrees/second)
// ============================================================
float desiredRoll  = 0;
float desiredPitch = 0;
float desiredYaw   = 0;
float desiredThrottle = 0;   // raw 0–1023, used directly in mixer


// ============================================================
//  RADIO SETUP
// ============================================================
RF24 radio(9, 10);                        // CE, CSN
const byte RADIO_ADDRESS[6] = "00001";

struct __attribute__((packed)) ControlData {
  int16_t throttle;   // 0 – 1023
  int16_t yaw;        // 0 – 1023
  int16_t pitch;      // 0 – 1023
  int16_t roll;       // 0 – 1023
};
ControlData rxData;

// Joystick center values captured during calibration
int16_t centerThrottle = 512;
int16_t centerYaw      = 512;
int16_t centerPitch    = 512;
int16_t centerRoll     = 512;

const int DEADZONE = 15;    // ADC counts either side of center treated as zero


// ============================================================
//  MPU6050 SETUP
// ============================================================
MPU6050 mpu(Wire);

// Gyro sensitivity divisors — must match the register config below
// Config 0 => ±250  °/s  => divisor 131.0
// Config 1 => ±500  °/s  => divisor  65.5
// Config 2 => ±1000 °/s  => divisor  32.8
// Config 3 => ±2000 °/s  => divisor  16.4
// MPU6050_light's getGyro*() already returns deg/s, but we set the
// range so fast flips don't clip.  Config 1 (±500) is a good default.
const uint8_t GYRO_CONFIG = 1;


// ============================================================
//  BATTERY MONITORING
// ============================================================
const int    BAT_VOLTAGE_PIN = A0;   // after a resistor divider
const int    BAT_CURRENT_PIN = A1;   // current sensor (e.g. ACS712)

const float  BAT_CAPACITY_MAH = 1300.0f;  // change to your pack
float        batteryStartMah  = 0;
float        currentConsumedMah = 0;

// Voltage divider ratio: if you use 10k + 3.3k to bring ~12 V into 3.3 V range
// Vbat = analogRead * (3.3 / 1023) * ((10000+3300) / 3300)
// Simplify to a single scale factor — calibrate against a multimeter
const float  VOLTAGE_SCALE   = 0.01565f;   // (3.3/1023) * (13300/3300)
// ACS712-5A: Vout = 2.5 + 0.185*I  →  I = (Vout - 2.5) / 0.185
// For a Teensy 3.3 V ADC: Vout = analogRead * (3.3/1023)
const float  CURRENT_SCALE   = 0.089f;     // matches reference (adjust for your sensor)

// ============================================================
//  TIMING
// ============================================================
const uint32_t LOOP_US   = 4000;     // 4 ms = 250 Hz
const float    LOOP_DT   = 0.004f;   // seconds  — baked into PID math

uint32_t loopTimer = 0;


// ============================================================
//  MISC
// ============================================================
const int LED_PIN = 13;
uint32_t  serialTimer = 0;

// PWM scale factor: analogWriteResolution(12) gives 0-4095.
// Standard ESC protocol expects 1000-2000 µs.
// At 400 Hz: period = 2500 µs.  1000 µs = 4096 * (1000/2500) ≈ 1638
// However, the reference code writes values in the 1000-2000 range
// directly and multiplies by 1.024 to account for the 12-bit scaling.
// We keep that convention so your throttle/PID values stay intuitive.
const float PWM_SCALE = 1.024f;


// ============================================================
//  FUNCTION: read_radio()
//  Reads the latest nRF24 packet (if available) and maps the
//  raw 0–1023 ADC values into desired deg/s rates.
//  Throttle is mapped to a 1000–2000 PWM-equivalent range.
//  Updates globals: desiredRoll, desiredPitch, desiredYaw,
//                   desiredThrottle
// ============================================================
void read_radio() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
  }

  // --- Throttle: map 0-1023 to 1000-2000 (PWM-equivalent) ---
  // We keep throttle in the 1000-2000 range so it feeds directly
  // into the motor mixer alongside PID outputs (which are ±400).
  desiredThrottle = map(rxData.throttle, 0, 1023, 1000, 2000);

  // --- Roll ---
  float rawRoll = rxData.roll - centerRoll;
  if (abs(rawRoll) < DEADZONE) rawRoll = 0;
  desiredRoll = (rawRoll / (float)centerRoll) * MAX_ROLL_RATE;

  // --- Pitch ---
  float rawPitch = rxData.pitch - centerPitch;
  if (abs(rawPitch) < DEADZONE) rawPitch = 0;
  // Invert if needed so forward stick = nose down (positive pitch rate)
  desiredPitch = -(rawPitch / (float)centerPitch) * MAX_PITCH_RATE;

  // --- Yaw ---
  float rawYaw = rxData.yaw - centerYaw;
  if (abs(rawYaw) < DEADZONE) rawYaw = 0;
  desiredYaw = (rawYaw / (float)centerYaw) * MAX_YAW_RATE;
}


// ============================================================
//  FUNCTION: read_gyro()
//  Fetches calibration-corrected gyro rates from MPU6050_light.
//  MPU6050_light::getGyroX/Y/Z() already returns deg/s after
//  calcOffsets() is called in setup.
//  Updates globals: currentRoll, currentPitch, currentYaw
//
//  IMPORTANT: verify axis mapping against your physical mount.
//  If the IMU is rotated 90°, swap or negate axes here.
// ============================================================
void read_gyro() {
  mpu.update();
  // MPU6050_light axis convention (flat mount, USB toward nose):
  //   getGyroX() = pitch rate
  //   getGyroY() = roll rate
  //   getGyroZ() = yaw rate
  // Adjust signs if the drone corrects in the wrong direction.
  currentPitch = mpu.getGyroX();
  currentRoll  = mpu.getGyroY();
  currentYaw   = mpu.getGyroZ();
}


// ============================================================
//  FUNCTION: calculate_PID()
//  Runs one PID iteration for a single axis.
//
//  Parameters:
//    desired   — target rate (deg/s)
//    current   — measured rate (deg/s)
//    kP, kI, kD — gains
//    state     — persistent iterm + prevError for this axis
//
//  Returns: correction value, clamped to ±400
//           (±400 keeps corrections within the ~1000-unit PWM
//            headroom above the idle throttle — same rationale
//            as the reference code)
// ============================================================
float calculate_PID(float desired, float current,
                    float kP, float kI, float kD,
                    PIDState &state) {

  float error = desired - current;

  // Proportional
  float pTerm = kP * error;

  // Integral — trapezoidal rule, anti-windup clamp ±400
  // Using (error + prevError)/2 gives a better area estimate
  // than a simple rectangle, matching the reference implementation.
  state.iterm += kI * (error + state.prevError) * LOOP_DT / 2.0f;
  state.iterm  = constrain(state.iterm, -400.0f, 400.0f);

  // Derivative — on error (not measurement), so a sudden stick
  // input won't spike D.  Divide by dt so gain is time-invariant.
  float dTerm = kD * (error - state.prevError) / LOOP_DT;

  state.prevError = error;

  float output = pTerm + state.iterm + dTerm;
  return constrain(output, -400.0f, 400.0f);
}


// ============================================================
//  FUNCTION: reset_pid()
//  Zero all integrators and previous errors.
//  Call this whenever the throttle is cut (sticks down) so
//  accumulated I-term doesn't cause a violent flip on re-arm.
// ============================================================
void reset_pid() {
  pidRoll  = {0, 0};
  pidPitch = {0, 0};
  pidYaw   = {0, 0};
}


// ============================================================
//  FUNCTION: mix_motors()
//  Combines throttle + PID corrections into 4 motor commands
//  using a standard X-frame mixer.
//
//  X-frame layout (top-down view, props spinning inward at front):
//
//      M1 (CCW) -------- M4 (CW)
//         \    FRONT    /
//          \           /
//          /           \
//         /    REAR     \
//      M2 (CW)  ------- M3 (CCW)
//
//  Roll  right  → M1/M2 speed up, M3/M4 slow down
//  Pitch forward → M1/M4 speed up, M2/M3 slow down (CHECK sign)
//  Yaw   CW     → CCW motors (M1,M3) speed up, CW (M2,M4) slow down
//
//  Output is in the 1000-2000 PWM-equivalent range.
//  PWM_SCALE (1.024) maps that range into the 12-bit analogWrite
//  resolution at 400 Hz (same approach as the reference code).
// ============================================================
struct MotorOutputs { float m1, m2, m3, m4; };

MotorOutputs mix_motors(float throttle,
                        float rollOut, float pitchOut, float yawOut) {
  MotorOutputs m;
  m.m1 = PWM_SCALE * (throttle - rollOut - pitchOut - yawOut); // Front-Left  CCW
  m.m2 = PWM_SCALE * (throttle - rollOut + pitchOut + yawOut); // Rear-Left   CW
  m.m3 = PWM_SCALE * (throttle + rollOut + pitchOut - yawOut); // Rear-Right  CCW
  m.m4 = PWM_SCALE * (throttle + rollOut - pitchOut + yawOut); // Front-Right CW
  return m;
}


// ============================================================
//  FUNCTION: write_motors()
//  Writes the four motor values to the ESCs via PWM.
//  Also enforces:
//    - Idle floor (1180): motors keep spinning when armed so
//      the ESCs don't disarm mid-flight.
//    - Hard ceiling (1999): leaves 1 count of headroom.
// ============================================================
const float THROTTLE_IDLE   = 1180.0f;
const float THROTTLE_CUTOFF = 1000.0f;   // ESC disarm signal

void write_motors(MotorOutputs &m) {
  // Enforce idle floor so ESCs stay armed
  m.m1 = max(m.m1, THROTTLE_IDLE);
  m.m2 = max(m.m2, THROTTLE_IDLE);
  m.m3 = max(m.m3, THROTTLE_IDLE);
  m.m4 = max(m.m4, THROTTLE_IDLE);

  // Hard ceiling
  m.m1 = min(m.m1, 1999.0f);
  m.m2 = min(m.m2, 1999.0f);
  m.m3 = min(m.m3, 1999.0f);
  m.m4 = min(m.m4, 1999.0f);

  analogWrite(1, (int)m.m1);
  analogWrite(2, (int)m.m2);
  analogWrite(3, (int)m.m3);
  analogWrite(4, (int)m.m4);
}


// ============================================================
//  FUNCTION: check_failsafe()
//  If throttle drops below the cutoff threshold (stick fully
//  down, or radio link lost and receiver outputs low), cut all
//  motors and reset PID state.
//
//  Returns true if failsafe is active (caller should skip
//  the rest of the loop body).
// ============================================================
bool check_failsafe() {
  // desiredThrottle is already mapped to 1000-2000
  if (desiredThrottle < 1050) {
    // Kill all motors
    analogWrite(1, (int)THROTTLE_CUTOFF);
    analogWrite(2, (int)THROTTLE_CUTOFF);
    analogWrite(3, (int)THROTTLE_CUTOFF);
    analogWrite(4, (int)THROTTLE_CUTOFF);
    reset_pid();
    return true;
  }
  return false;
}


// ============================================================
//  FUNCTION: read_battery()
//  Reads voltage and current from analog pins and updates the
//  running current-consumed tally.
//
//  Call once per loop (every 4 ms).
//  batteryRemaining (0-100%) is updated as a global.
// ============================================================
float batteryVoltage   = 0;
float batteryCurrent   = 0;
float batteryRemaining = 100.0f;

void read_battery() {
  // --- Voltage ---
  batteryVoltage = analogRead(BAT_VOLTAGE_PIN) * VOLTAGE_SCALE;

  // --- Current  (ACS712 or similar) ---
  // Convert ADC reading to voltage, then to amps
  float currentVoltage = analogRead(BAT_CURRENT_PIN) * (3.3f / 1023.0f);
  batteryCurrent = currentVoltage * (1.0f / CURRENT_SCALE);
  // Note: if your sensor has an offset (e.g. ACS712 idles at 2.5 V)
  // subtract it: batteryCurrent = (currentVoltage - 2.5f) / 0.185f;

  // --- Coulomb counting (mAh) ---
  // Current (A) × time (hours) = charge (Ah)
  // time per loop = LOOP_DT seconds = LOOP_DT/3600 hours
  currentConsumedMah += batteryCurrent * 1000.0f * (LOOP_DT / 3600.0f);

  // --- Remaining percentage ---
  batteryRemaining = ((batteryStartMah - currentConsumedMah) / BAT_CAPACITY_MAH) * 100.0f;
  batteryRemaining = constrain(batteryRemaining, 0.0f, 100.0f);
}


// ============================================================
//  FUNCTION: estimate_starting_charge()
//  Called once in setup() to estimate how full the battery
//  already is from its resting voltage, rather than assuming
//  it's always 100%.  Matches the reference code's approach.
//  Returns initial charge in mAh.
// ============================================================
float estimate_starting_charge() {
  // Read voltage a few times and average to settle the ADC
  float v = 0;
  for (int i = 0; i < 10; i++) {
    v += analogRead(BAT_VOLTAGE_PIN) * VOLTAGE_SCALE;
    delay(5);
  }
  v /= 10.0f;
  batteryVoltage = v;

  float startMah;
  if (v > 8.3f) {
    // Fully charged (>4.15 V/cell on a 2S)
    startMah = BAT_CAPACITY_MAH;
    digitalWrite(LED_PIN, LOW);   // LED off = battery OK
  } else if (v < 7.5f) {
    // Low battery — warn and start at 30%
    startMah = 0.30f * BAT_CAPACITY_MAH;
    digitalWrite(LED_PIN, HIGH);  // LED on = low battery warning
  } else {
    // Partial charge: linear interpolation between 7.5 V and 8.3 V
    // (82 * v - 580) / 100  gives 0.0-1.0 fraction over that range
    startMah = ((82.0f * v - 580.0f) / 100.0f) * BAT_CAPACITY_MAH;
    digitalWrite(LED_PIN, LOW);
  }

  Serial.print("Battery voltage: "); Serial.print(v, 2); Serial.println(" V");
  Serial.print("Starting charge: "); Serial.print(startMah, 0); Serial.println(" mAh");
  return startMah;
}


// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);   // 400 kHz I2C for faster gyro reads

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // LED on during init

  // ---- Radio ----
  Serial.println("Initializing radio...");
  if (!radio.begin()) {
    Serial.println("ERROR: Radio not responding. Check wiring.");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200); }
  }
  radio.openReadingPipe(0, RADIO_ADDRESS);
  radio.setPALevel(RF24_PA_MIN);     // increase to RF24_PA_HIGH outdoors
  radio.setDataRate(RF24_250KBPS);   // most reliable at short range
  radio.startListening();
  Serial.println("Radio OK.");

  // ---- Joystick center calibration ----
  // The transmitter must have all sticks centered during this window.
  Serial.println("Calibrating joysticks — center all sticks...");
  delay(2000);   // give the user time to center sticks

  long sumT = 0, sumR = 0, sumP = 0, sumY = 0;
  int  samples = 0;
  uint32_t calStart = millis();

  while (millis() - calStart < 1000) {   // collect for 1 second
    if (radio.available()) {
      radio.read(&rxData, sizeof(rxData));
      sumT += rxData.throttle;
      sumR += rxData.roll;
      sumP += rxData.pitch;
      sumY += rxData.yaw;
      samples++;
    }
    delay(4);
  }

  if (samples > 0) {
    centerThrottle = sumT / samples;
    centerRoll     = sumR / samples;
    centerPitch    = sumP / samples;
    centerYaw      = sumY / samples;
  }

  // Report centers back over serial (read with Serial Monitor or
  // send to an OLED / LCD if you wire one up)
  Serial.println("--- Joystick centers ---");
  Serial.print("  Throttle: "); Serial.println(centerThrottle);
  Serial.print("  Roll:     "); Serial.println(centerRoll);
  Serial.print("  Pitch:    "); Serial.println(centerPitch);
  Serial.print("  Yaw:      "); Serial.println(centerYaw);

  // ---- MPU6050 ----
  Serial.println("Initializing MPU6050...");
  // setGyroConfig: 0=±250, 1=±500, 2=±1000, 3=±2000 deg/s
  mpu.setGyroConfig(GYRO_CONFIG);

  byte mpuStatus = mpu.begin();
  if (mpuStatus != 0) {
    Serial.print("ERROR: MPU6050 init failed, code: "); Serial.println(mpuStatus);
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(500); }
  }

  Serial.println("Calibrating gyro — keep drone perfectly still...");
  delay(500);
  mpu.calcOffsets();   // MPU6050_light samples ~3000 pts internally
  Serial.println("Gyro calibration done.");

  // ---- PWM / ESC setup ----
  analogWriteFrequency(1, 400);
  analogWriteFrequency(2, 400);
  analogWriteFrequency(3, 400);
  analogWriteFrequency(4, 400);
  analogWriteResolution(12);

  // Send disarm signal while ESCs boot
  analogWrite(1, (int)THROTTLE_CUTOFF);
  analogWrite(2, (int)THROTTLE_CUTOFF);
  analogWrite(3, (int)THROTTLE_CUTOFF);
  analogWrite(4, (int)THROTTLE_CUTOFF);
  delay(1000);   // wait for ESCs to arm at low throttle

  // ---- Battery ----
  batteryStartMah = estimate_starting_charge();

  digitalWrite(LED_PIN, LOW);   // init complete
  Serial.println("Ready. Arm by raising throttle.");

  loopTimer = micros();
}


// ============================================================
//  LOOP
// ============================================================
void loop() {

  // 1. Read sensors
  read_gyro();    // updates currentRoll, currentPitch, currentYaw
  read_radio();   // updates desiredRoll, desiredPitch, desiredYaw, desiredThrottle

  // 2. Failsafe — cut motors if throttle is at the bottom
  if (check_failsafe()) {
    // Still burn the rest of the 4 ms so our timing stays consistent
    while (micros() - loopTimer < LOOP_US);
    loopTimer = micros();
    return;
  }

  // 3. PID — one call per axis
  float rollOutput  = calculate_PID(desiredRoll,  currentRoll,
                                    P_Roll,  I_Roll,  D_Roll,  pidRoll);
  float pitchOutput = calculate_PID(desiredPitch, currentPitch,
                                    P_Pitch, I_Pitch, D_Pitch, pidPitch);
  float yawOutput   = calculate_PID(desiredYaw,   currentYaw,
                                    P_Yaw,   I_Yaw,   D_Yaw,   pidYaw);

  // 4. Mix corrections into motor commands
  MotorOutputs motors = mix_motors(desiredThrottle,
                                   rollOutput, pitchOutput, yawOutput);

  // 5. Write to ESCs
  write_motors(motors);

  // 6. Battery monitoring (every loop, cheap analog reads)
  read_battery();

  // 7. LED: blink faster when battery below 30%
  if (batteryRemaining <= 30.0f) digitalWrite(LED_PIN, HIGH);
  else                           digitalWrite(LED_PIN, LOW);

  // 8. Serial telemetry (50 Hz — every 20 ms)
  if (millis() - serialTimer > 20) {
    Serial.print("GYRO  R:"); Serial.print(currentRoll,  1);
    Serial.print(" P:");      Serial.print(currentPitch, 1);
    Serial.print(" Y:");      Serial.print(currentYaw,   1);
    Serial.print("  |  DESIRED R:"); Serial.print(desiredRoll,  1);
    Serial.print(" P:");             Serial.print(desiredPitch, 1);
    Serial.print(" Y:");             Serial.print(desiredYaw,   1);
    Serial.print("  |  THR:");       Serial.print(desiredThrottle, 0);
    Serial.print("  |  BAT:");       Serial.print(batteryRemaining, 1);
    Serial.println("%");
    serialTimer = millis();
  }

  // 9. Loop timing — spin until exactly 4 ms have elapsed
  //    This is critical: LOOP_DT in calculate_PID() MUST match
  //    real elapsed time, or I and D terms will be wrong.
  while (micros() - loopTimer < LOOP_US);
  loopTimer = micros();
}
