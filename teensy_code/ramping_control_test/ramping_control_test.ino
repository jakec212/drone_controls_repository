#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>

// =============================== SETTINGS ===============================
const uint8_t RF_CHANNEL = 108;
const uint16_t PRINT_INTERVAL_MS = 20;
const uint8_t GYRO_CONFIG = 3; // 0=250, 1=500, 2=1000, 3=2000 deg/s

const int THROTTLE_MIN_CMD = 1000;
const int THROTTLE_MAX_CMD = 2000;

//starting motor value
float start_speed = 0;
float end_speed = 0;
// Match flight_controller mapping bounds exactly.
const int THROTTLE_MAP_IN_MIN = 33;
const int THROTTLE_MAP_IN_MAX = 990;

const int MOTOR_PWM_FREQ_HZ = 200;
const int PWM_RES_BITS = 12;
const int PWM_MAX_COUNTS = (1 << PWM_RES_BITS) - 1; // 4095

const int MOTOR1_PIN = 1;
const int MOTOR2_PIN = 2;
const int MOTOR3_PIN = 3;
const int MOTOR4_PIN = 4;

// =============================== RADIO ===============================
RF24 radio(9, 10);
const byte address[6] = "00001";

struct __attribute__((packed)) ControlData {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};
ControlData incomingData = {512, 512, 512, 512};

int16_t rollCenter = 512;
int16_t pitchCenter = 512;
int16_t yawCenter = 512;

// =============================== IMU ===============================
MPU6050 mpu(Wire);

// =============================== STATE ===============================
unsigned long timer = 0;
unsigned long lastRadioPacketMs = 0;
float desiredThrottle = 0.0f;
float throttleCommand = 1000.0f;

// =============================== HELPERS ===============================
bool calibrate_radio_centers(uint16_t sampleCount = 100, uint32_t timeoutMs = 3000) {
  Serial.println("Center all sticks now. Calibrating radio centers...");

  long sumRoll = 0;
  long sumPitch = 0;
  long sumYaw = 0;
  uint16_t got = 0;
  uint32_t t0 = millis();

  while (got < sampleCount && (millis() - t0) < timeoutMs) {
    if (radio.available()) {
      while (radio.available()) {
        radio.read(&incomingData, sizeof(incomingData));
      }
      sumRoll += incomingData.roll;
      sumPitch += incomingData.pitch;
      sumYaw += incomingData.yaw;
      got++;
    }
  }

  if (got == 0) {
    Serial.println("Calibration failed: no radio packets received.");
    return false;
  }

  rollCenter = (int16_t)(sumRoll / got);
  pitchCenter = (int16_t)(sumPitch / got);
  yawCenter = (int16_t)(sumYaw / got);

  Serial.print("Center values -> Roll: "); Serial.print(rollCenter);
  Serial.print(" Pitch: ");                Serial.print(pitchCenter);
  Serial.print(" Yaw: ");                  Serial.println(yawCenter);
  return true;
}

int pwmCommandToDutyCounts(float pwmCommandUs) {
  float clampedUs = constrain(pwmCommandUs, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);
  float periodUs = 1000000.0f / (float)MOTOR_PWM_FREQ_HZ;
  float dutyCounts = (clampedUs / periodUs) * (float)PWM_MAX_COUNTS;
  return (int)constrain(dutyCounts, 0.0f, (float)PWM_MAX_COUNTS);
}

void write_all_motors_us(float pwmUs) {
  int duty = pwmCommandToDutyCounts(pwmUs);
  analogWrite(MOTOR1_PIN, duty);
  analogWrite(MOTOR2_PIN, duty);
  analogWrite(MOTOR3_PIN, duty);
  analogWrite(MOTOR4_PIN, duty);
}

void calibrate_escs() {
  Serial.println("ESC calibration: sending 2000us for 5s...");
  write_all_motors_us(2000.0f);
  delay(5000);

  Serial.println("ESC calibration: sending 1000us for 3s...");
  write_all_motors_us(1000.0f);
  delay(3000);

  Serial.println("ESC calibration done.");
}

void read_radio() {
  bool gotPacket = false;
  while (radio.available()) {
    radio.read(&incomingData, sizeof(incomingData));
    gotPacket = true;
    lastRadioPacketMs = millis();
  }

  if (gotPacket) {
    desiredThrottle = incomingData.throttle;
  }
}

// =============================== ARDUINO ===============================
void setup() {
  Serial.begin(230400);

  Wire.begin();
  Wire.setClock(400000);

  if (!radio.begin()) {
    Serial.println("Radio Error");
    while (1) {}
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RF_CHANNEL);
  radio.startListening();

  delay(1500);
  if (!calibrate_radio_centers(100, 4000)) {
    rollCenter = 512;
    pitchCenter = 512;
    yawCenter = 512;
  }

  mpu.setGyroConfig(GYRO_CONFIG);
  byte mpuStatus = mpu.begin();
  if (mpuStatus != 0) {
    Serial.print("MPU Error: ");
    Serial.println(mpuStatus);
    while (1) {}
  }

  Serial.println("Calibrating MPU6050. Keep drone still...");
  delay(1000);
  mpu.calcOffsets();

  analogWriteFrequency(MOTOR1_PIN, MOTOR_PWM_FREQ_HZ);
  analogWriteFrequency(MOTOR2_PIN, MOTOR_PWM_FREQ_HZ);
  analogWriteFrequency(MOTOR3_PIN, MOTOR_PWM_FREQ_HZ);
  analogWriteFrequency(MOTOR4_PIN, MOTOR_PWM_FREQ_HZ);
  analogWriteResolution(PWM_RES_BITS);

  calibrate_escs();
  write_all_motors_us((float)THROTTLE_MIN_CMD);

  Serial.println("Throttle passthrough ready.");
  timer = millis();
}

void loop() {
  read_radio();

  throttleCommand = map((int)desiredThrottle,
                        THROTTLE_MAP_IN_MIN,
                        THROTTLE_MAP_IN_MAX,
                        THROTTLE_MIN_CMD,
                        THROTTLE_MAX_CMD);
  throttleCommand = constrain(throttleCommand, (float)THROTTLE_MIN_CMD, (float)THROTTLE_MAX_CMD);
  
  
  end_speed = throttleCommand;
  ramp(start_speed, throttleCommand);
  write_all_motors_us(throttleCommand);
  start_speed = end_speed;

  if (millis() - timer >= PRINT_INTERVAL_MS) {
    bool radioLinkOk = (millis() - lastRadioPacketMs) < 250;
    Serial.print("R:");
    Serial.print(radioLinkOk ? "OK" : "NO_PACKET");
    Serial.print(" RawT:");
    Serial.print(incomingData.throttle);
    Serial.print(" ThrCmd:");
    Serial.println(throttleCommand, 1);
    timer = millis();
  }
}


void ramp(float start_speed, float end_speed){
  if(start_speed > end_speed){
    for(float i = start_speed; i >= end_speed; i --){
    write_all_motors_us(i);
    Serial.print(i);
    //little delay
    delay(1);
    }
  }
  else{
    for(float i = start_speed; i <= end_speed; i ++){
    write_all_motors_us(i);
    Serial.print(i);
    //little delay
    delay(1);
    }
  }
}

