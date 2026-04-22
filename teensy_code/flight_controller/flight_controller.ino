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

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float ItermRoll, ItermPitch, ItermYaw;
float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
float PIDRoll, PIDPitch, PIDYaw;

//======= CONTROL VARIABLES =========
const int DEADZONE = 10;
const uint8_t RF_CHANNEL = 108;
const uint16_t PRINT_INTERVAL_MS = 20;  // 50 Hz telemetry

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

    //===== SETUP MPU6050 =====
    mpu.setGyroConfig(1); // +/-500 deg/s
    byte mpuStatus = mpu.begin();
    if (mpuStatus != 0) {
      Serial.print("MPU Error: ");
      Serial.println(mpuStatus);
      while (1);
    }
    Serial.println("Calibrating MPU6050. Keep drone still...");
    delay(1000);
    mpu.calcOffsets();

    Serial.println("Flight controller sensor/radio test ready.");
    timer = millis();
}

//======================================= ARDUINO LOOP FUNCTION =======================================
void loop() {
  // Always update gyro data each loop.
  mpu.update();

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

  // Print at 50 Hz for near real-time axis feedback.
  if (millis() - timer >= PRINT_INTERVAL_MS) {
    bool radioLinkOk = (millis() - lastRadioPacketMs) < 250;
    Serial.print("R:");
    Serial.print(radioLinkOk ? "OK" : "NO_PACKET");
    Serial.print(" Raw T:");
    Serial.print(incomingData.throttle);
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

    Serial.print(" Gyro XYZ:");
    Serial.print(mpu.getGyroX(), 1);
    Serial.print(",");
    Serial.print(mpu.getGyroY(), 1);
    Serial.print(",");
    Serial.println(mpu.getGyroZ(), 1);

    timer = millis();
  }
}
