#include <SPI.h>
// #include <nRF24L01.h> ----- going to see if this compiles without this
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>

//=============================== DEFINE VARIABLES AND CONSTANTS ===============================

// ===== MAX PITCH, ROLL, AND YAW RATES ======
const float MAX_RP_RATE = 60.0; // Max Roll/Pitch rate
const float MAX_Y_RATE = 30.0;  // Max Yaw rate

//====== PID CONSTANTS ===========
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

//====== PID STATE VARIABLES ==========
float desiredRoll = 0.0f;
float desiredPitch = 0.0f;
float desiredYaw = 0.0f;
float desiredThrottle = 0.0f;
float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float ItermRoll, ItermPitch, ItermYaw;
float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
float PIDRoll, PIDPitch, PIDYaw;
float throttleCommand = 1000.0f;                        //THROTTLE COMMAND TO BE SENT TO THE MOTORS

//======= CONTROL VARIABLES =========
const int DEADZONE = 15;
const uint8_t RF_CHANNEL = 108;
const uint16_t PRINT_INTERVAL_MS = 20;  // 50 Hz DEBUGGING telemetry
const uint8_t GYRO_CONFIG = 3;          // 0=250, 1=500, 2=1000, 3=2000 deg/s

const float PID_DT = 0.004f;            // 250 Hz control loop timing
const float PID_LIMIT = 400.0f;

const uint32_t LOOP_US = 4000;          // 4 ms loop target
const int MOTOR_PWM_FREQ_HZ = 200;
const int PWM_RES_BITS = 12;
const int PWM_MAX_COUNTS = (1 << PWM_RES_BITS) - 1; // 4095 for 12-bit
uint32_t loopTimerUs = 0;

//========================= MAX AND MIN PWM SIGNALS THAT ARE TO BE SENT TO THE MOTORS ============================
const int THROTTLE_MIN_CMD = 1000;
const int THROTTLE_MAX_CMD = 2000;

//========================= DEFINING THE MOTOR PINS ON THE TEENSY 4.0 =====================
const int MOTOR1_PIN = 1;
const int MOTOR2_PIN = 2;
const int MOTOR3_PIN = 3;
const int MOTOR4_PIN = 4;

int16_t rollCenter = 512;
int16_t pitchCenter = 512;
int16_t yawCenter = 512;




//======================================== RADIO SETUP ==========================================
RF24 radio(9, 10);
const byte address[6] = "00001";

struct __attribute__((packed)) ControlData {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};
ControlData incomingData = {512, 512, 512, 512};

//=========================================== MOTOR OUTPUTS STRUCT DATA TYPE ================================
struct MotorOutputs {
  float m1;
  float m2;
  float m3;
  float m4;
};
MotorOutputs motors = {1000, 1000, 1000, 1000};


//======================================= MPU6050 SETUP =================================================
MPU6050 mpu(Wire);
unsigned long timer = 0;
unsigned long lastRadioPacketMs = 0;

//================================ GENERIC FUNCTIONS FOR READABILITY =======================================

//=========== MAP ANALOG TO DEGREES/SECOND ===========
float mapStickToRate(int16_t stick, int16_t center, float maxRate) {
  int16_t delta = stick - center;

  if (abs(delta) < DEADZONE) {
    return 0.0f;
  }

  return ((float)delta / 511.0f) * maxRate;
}

//========== READ GYRO FUNCTION ==============
void read_gyro() {
  mpu.update();

  // Axis convention used here:
  // X -> roll rate, Y -> pitch rate, Z -> yaw rate
  currentPitch = mpu.getGyroY();
  currentRoll  = mpu.getGyroX();
  currentYaw   = -mpu.getGyroZ(); //this negative accounts for ccw being positive and cw being negative naturally with the drone orientation
}

//========== READ RADIO FUNCTION ==============
void read_radio() {
  // If multiple packets are waiting, keep the newest one.
  while (radio.available()) {
    radio.read(&incomingData, sizeof(incomingData));
  }

  desiredRoll  = mapStickToRate(incomingData.roll,  rollCenter,  MAX_RP_RATE);
  desiredPitch = mapStickToRate(incomingData.pitch, pitchCenter, MAX_RP_RATE);
  desiredYaw   = mapStickToRate(incomingData.yaw,   yawCenter,   MAX_Y_RATE);
  desiredThrottle = incomingData.throttle;
}

//========== CALCULATE PID (ONE AXIS) ==============
float calculate_PID(float desired, float current,
                    float kP, float kI, float kD,
                    float &prevError, float &iTerm) {
  float error = desired - current;

  float pTerm = kP * error;

  // Trapezoidal integral + anti-windup clamp
  iTerm += kI * (error + prevError) * PID_DT * 0.5f;
  iTerm = constrain(iTerm, -PID_LIMIT, PID_LIMIT);

  float dTerm = kD * (error - prevError) / PID_DT;
  prevError = error;

  float output = pTerm + iTerm + dTerm;
  return constrain(output, -PID_LIMIT, PID_LIMIT);
}

//========== MIX MOTORS FUNCTION ==============
MotorOutputs mix_motors(float throttle, float rollOutput, float pitchOutput, float yawOutput) {
  MotorOutputs m;
  // Motor spin directions:
  // M1, M4 = CCW   M2, M3 = CW
  // Positive yaw command -> CW body yaw.
  // So increase CCW motors (M1,M4) and decrease CW motors (M2,M3).
  m.m1 = throttle + rollOutput + pitchOutput + yawOutput;
  m.m2 = throttle - rollOutput + pitchOutput - yawOutput;
  m.m3 = throttle - rollOutput - pitchOutput - yawOutput;
  m.m4 = throttle + rollOutput - pitchOutput + yawOutput;
  return m;
}

//========== PWM MATH FOR 200 HZ OUTPUT ==============
int pwmCommandToDutyCounts(float pwmCommandUs) {
  float clampedUs = constrain(pwmCommandUs, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);
  float periodUs = 1000000.0f / (float)MOTOR_PWM_FREQ_HZ;
  float dutyCounts = (clampedUs / periodUs) * (float)PWM_MAX_COUNTS;
  return (int)constrain(dutyCounts, 0.0f, (float)PWM_MAX_COUNTS);
}

//========== WRITE MIXED VALUES TO MOTORS ==============
void write_motors(const MotorOutputs &m) {
  float m1Cmd = constrain(m.m1, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);
  float m2Cmd = constrain(m.m2, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);
  float m3Cmd = constrain(m.m3, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);
  float m4Cmd = constrain(m.m4, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);

  analogWrite(MOTOR1_PIN, pwmCommandToDutyCounts(m1Cmd));
  analogWrite(MOTOR2_PIN, pwmCommandToDutyCounts(m2Cmd));
  analogWrite(MOTOR3_PIN, pwmCommandToDutyCounts(m3Cmd));
  analogWrite(MOTOR4_PIN, pwmCommandToDutyCounts(m4Cmd));
}

//========== WRITE SAME PWM COMMAND TO ALL MOTORS ==============
void write_all_motors_us(float pwmUs) {
  int duty = pwmCommandToDutyCounts(pwmUs);
  analogWrite(MOTOR1_PIN, duty);
  analogWrite(MOTOR2_PIN, duty);
  analogWrite(MOTOR3_PIN, duty);
  analogWrite(MOTOR4_PIN, duty);
}

//========== ESC CALIBRATION SEQUENCE ==============
void calibrate_escs() {
  Serial.println("ESC calibration: sending 2000us for 5s...");
  write_all_motors_us(2000.0f);
  delay(5000);

  Serial.println("ESC calibration: sending 1000us for 3s...");
  write_all_motors_us(1000.0f);
  delay(3000);

  Serial.println("ESC calibration done.");
}

//=========== JOYSTICK CALIBRATION FUNCTION =================
bool calibrate_radio_centers(uint16_t sampleCount = 100, uint32_t timeoutMs = 3000) {
  Serial.println("Center all sticks now. Calibrating radio centers...");

  long sumRoll = 0;
  long sumPitch = 0;
  long sumYaw = 0;
  uint16_t got = 0;
  uint32_t t0 = millis();

  while (got < sampleCount && (millis() - t0) < timeoutMs) {
    if (radio.available()) {
      // Keep newest packet in case multiple are queued
      while (radio.available()) {
        radio.read(&incomingData, sizeof(incomingData));
      }

      sumRoll  += incomingData.roll;
      sumPitch += incomingData.pitch;
      sumYaw   += incomingData.yaw;
      got++;
    }
  }

  if (got == 0) {
    Serial.println("Calibration failed: no radio packets received.");
    return false;
  }

  rollCenter  = (int16_t)(sumRoll  / got);
  pitchCenter = (int16_t)(sumPitch / got);
  yawCenter   = (int16_t)(sumYaw   / got);

  Serial.print("Center values -> Roll: ");  Serial.print(rollCenter);
  Serial.print(" Pitch: ");                 Serial.print(pitchCenter);
  Serial.print(" Yaw: ");                   Serial.println(yawCenter);

  return true;
}





//======================================= ARDUINO SETUP FUNCTION ======================================
void setup() {
    //===== SETUP SERIAL COMMUNICATION =======
    Serial.begin(230400);

    //===== SETUP I2C COMMUNICATION FOR THE MPU5060 GYRO =====
    Wire.begin();
    Wire.setClock(400000);

    //===== SETUP SPI RADIO COMMUNICATION ======= (SOME CONSTANTS MUST BE THE SAME AS THE TRANSMITTER)
    if (!radio.begin()) {
        Serial.println("Radio Error");
        while (1);
    }
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(RF_CHANNEL);
    radio.startListening();

    //====== RADIO JOYSTICK CALIBRATION ========
    delay(1500);  // give you a moment to release sticks to center
    if (!calibrate_radio_centers(100, 4000)) {
        // Fallback if calibration fails
        rollCenter = 512;
        pitchCenter = 512;
        yawCenter = 512;
    }

    //======== SETUP AND CONFIGURE GYRO ===========
    // GYRO_SENSITIVITY 0 = +/-250 deg/s, 1 = +/-500 deg/s, 2 = +/-1000 deg/s, 3 = +/-2000 deg/s
    mpu.setGyroConfig(GYRO_CONFIG); 
    byte mpuStatus = mpu.begin();
    if (mpuStatus != 0) {
      Serial.print("MPU Error: ");
      Serial.println(mpuStatus);
      while (1);
    }

    //======== CALIBRATE THE GYRO(MPU6050) ==========
    Serial.println("Calibrating MPU6050. Keep drone still...");
    delay(1000);
    mpu.calcOffsets();

    Serial.println("Flight controller sensor/radio test ready.");
    timer = millis();

    //================== SETTING UP PWM FREQUENCY FOR EACH OF THE MOTOR PINS ====================
    analogWriteFrequency(MOTOR1_PIN, MOTOR_PWM_FREQ_HZ);
    analogWriteFrequency(MOTOR2_PIN, MOTOR_PWM_FREQ_HZ);
    analogWriteFrequency(MOTOR3_PIN, MOTOR_PWM_FREQ_HZ);
    analogWriteFrequency(MOTOR4_PIN, MOTOR_PWM_FREQ_HZ);
    analogWriteResolution(PWM_RES_BITS);

    //================== ESC CALIBRATION: HIGH THEN LOW ====================
    calibrate_escs();

    //=========== INITIALIZING THE PID LOOP TIMER COUNTER ============
    loopTimerUs = micros();
}

//======================================= ARDUINO LOOP FUNCTION =======================================
void loop() {
  //======== STEP 1 - READ GYRO SENSOR DATA ========
  read_gyro();

  //======== STEP 2 - READ PILOT COMMANDS FROM RADIO =========
  bool gotPacket = false;
  while (radio.available()) {
    radio.read(&incomingData, sizeof(incomingData));
    gotPacket = true;
    lastRadioPacketMs = millis();
  }

  if (gotPacket) {
    desiredRoll  = mapStickToRate(incomingData.roll,  rollCenter,  MAX_RP_RATE);
    desiredPitch = mapStickToRate(incomingData.pitch, pitchCenter, MAX_RP_RATE);
    desiredYaw   = mapStickToRate(incomingData.yaw,   yawCenter,   MAX_Y_RATE);
    desiredThrottle = incomingData.throttle;
  }

  //======== STEP 3 - APPLY PID FOR ROLL/PITCH/YAW =========
  PIDRoll = calculate_PID(desiredRoll, currentRoll,
                          PRateRoll, IRateRoll, DRateRoll,
                          PrevErrorRoll, ItermRoll);
  PIDPitch = calculate_PID(desiredPitch, currentPitch,
                           PRatePitch, IRatePitch, DRatePitch,
                           PrevErrorPitch, ItermPitch);
  PIDYaw = calculate_PID(desiredYaw, currentYaw,
                         PRateYaw, IRateYaw, DRateYaw,
                         PrevErrorYaw, ItermYaw);

  //======== STEP 4 - MIX PID OUTPUTS INTO MOTOR COMMANDS ================ (THIS IS ALSO WHERE THE THROTTLE IS MAPPED FROM ITS LOW VALUE OF 33 ANALOG TO 990 ANALOG)
  throttleCommand = map((int)desiredThrottle, 33, 990, THROTTLE_MIN_CMD, THROTTLE_MAX_CMD);
  motors = mix_motors(throttleCommand, PIDRoll, PIDPitch, PIDYaw);

  //======== STEP 5 - WRITE PWM SIGNALS TO THE MOTORS =========
  write_motors(motors);





  //======= SERIAL PRINT AT 50Hz FOR DEBUGGING PURPOSES ============
  if (millis() - timer >= PRINT_INTERVAL_MS) {
    bool radioLinkOk = (millis() - lastRadioPacketMs) < 250;
    Serial.print("R:");
    Serial.print(radioLinkOk ? "OK" : "NO_PACKET");
    Serial.print(" Raw T:");
    Serial.print(incomingData.throttle);
    Serial.print(" ThrCmd:");
    Serial.print(throttleCommand, 1);
    Serial.print(" Y:");
    Serial.print(incomingData.yaw);
    Serial.print(" P:");
    Serial.print(incomingData.pitch);
    Serial.print(" R:");
    Serial.print(incomingData.roll);

    Serial.print(" Cmd YPR:");
    Serial.print(desiredYaw, 1);
    Serial.print(",");
    Serial.print(desiredPitch, 1);
    Serial.print(",");
    Serial.print(desiredRoll, 1);

    Serial.print(" GyroDPS XYZ:");
    Serial.print(currentPitch, 1);
    Serial.print(",");
    Serial.print(currentRoll, 1);
    Serial.print(",");
    Serial.print(currentYaw, 1);

    Serial.print(" PID RPY:");
    Serial.print(PIDRoll, 1);
    Serial.print(",");
    Serial.print(PIDPitch, 1);
    Serial.print(",");
    Serial.print(PIDYaw, 1);

    Serial.print(" Mix M1-4:");
    Serial.print(motors.m1, 1);
    Serial.print(",");
    Serial.print(motors.m2, 1);
    Serial.print(",");
    Serial.print(motors.m3, 1);
    Serial.print(",");
    Serial.println(motors.m4, 1);

    timer = millis();
  }

  //======== STEP 6 - LOCK LOOP RATE TO 4 ms (250 Hz) =========
  while ((micros() - loopTimerUs) < LOOP_US) {
    // wait
  }

  //======== RESET THE LOOP TIMER AND THEN RERUN THE PID LOOP ========
  loopTimerUs = micros();
}
