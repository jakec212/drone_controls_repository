#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>


//variables like within range function for the joystick values
//max and min roll and pitch Rate degrees/s (60 degrees/s max)(connect the 1023 from the joystick to what the input should be)
//max and min yaw rate degrees/s (30 degrees/s max)

// --- Target Rates (Degrees per Second) ---
const float MAX_RP_RATE = 60.0; // Max Roll/Pitch rate
const float MAX_Y_RATE = 30.0;  // Max Yaw rate

// --- PID Gains (Start with these, then tune) ---
float P_RateRoll = 0.6, I_RateRoll = 3.5, D_RateRoll = 0.03;
float P_RatePitch = 0.6, I_RatePitch = 3.5, D_RatePitch = 0.03;
float P_RateYaw = 2.0, I_RateYaw = 12.0, D_RateYaw = 0.0;

// --- PID State Variables ---
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float ItermRoll, ItermPitch, ItermYaw;
float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
float PIDRoll, PIDPitch, PIDYaw;

// --- Joystick Calibration ---
int16_t rollCenter = 512, pitchCenter = 512, yawCenter = 512;
const int DEADZONE = 10; // Ignore small stick movements near center

// --- Timing ---
uint32_t LoopTimer;





// --- Radio Setup ---
RF24 radio(9, 10);
const byte address[6] = "00001";

struct __attribute__((packed)) ControlData {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

ControlData incomingData;

// --- MPU6050 Setup ---
MPU6050 mpu(Wire);
unsigned long timer = 0;




void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // 1. Initialize Radio
  if (!radio.begin()) {
    Serial.println("Radio Error");
    while (1);
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();


  //Calibrate the radio readings for the joystick values to get a "center value"
  //read like 100 values and take the average and store that as a center value
  //plus or minus 5 or 10 of that center value should be a zero as well.

  // --- Joystick Calibration ---
  Serial.println("Calibrating Joysticks... Leave sticks centered.");
  long sumRoll = 0, sumPitch = 0, sumYaw = 0;
  for (int i = 0; i < 100; i++) {
    if (radio.available()) {
      radio.read(&incomingData, sizeof(incomingData));
    }
    sumRoll += incomingData.roll;
    sumPitch += incomingData.pitch;
    sumYaw += incomingData.yaw;
    delay(10);
  }
  rollCenter = sumRoll / 100;
  pitchCenter = sumPitch / 100;
  yawCenter = sumYaw / 100;
  
  // Set Teensy PWM frequency for ESCs (standard is 400Hz or 490Hz)
  analogWriteFrequency(1, 400); // Pins for motors
  analogWriteFrequency(2, 400);
  analogWriteFrequency(3, 400);
  analogWriteFrequency(4, 400);
  analogWriteResolution(12); // 0-4095 range for finer control
  
  LoopTimer = micros(); // Start the flight loop timer



  // 2. Initialize MPU6050

  // GYRO_SENSITIVITY 0 = +/-250 deg/s, 1 = +/-500 deg/s, 2 = +/-1000 deg/s, 3 = +/-2000 deg/s
  mpu.setGyroConfig(3);
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU Error: ");
    Serial.println(status);
    while (1);
  }

  // Calibrate to find the "zero" velocity (offset)
  // Ensure the drone is perfectly still on the desk
  Serial.println("Calibrating Gyro... Keep still.");
  delay(1000);
  mpu.calcOffsets(); 
  Serial.println("Ready.");
}




void loop() {
  // Read sensor data
  mpu.update();

  // Read radio data
  if (radio.available()) {
    radio.read(&incomingData, sizeof(incomingData));
  }


//Map radio data 0-1023to max and min gyro target values for roll, pitch, and yaw
//center values of the joysticks should mark 0degrees/s anything less is turning left for roll and yaw. less than the middle joystick value is doing a backflip for the drone

//Compare current values with Target values
//Current values is a scale of 0degrees/s to 2000degrees/s mapped in a range of +/- 32,767
//its honestly a pretty accurate analog reading for the rate that you are spinning

//Apply PID control loop

//Determine the corrective motor speeds and update the PWM signals for the drone









  // Print raw velocity (deg/sec) every 20ms (50Hz)
  // This frequency is more common for flight data monitoring
  if (millis() - timer > 20) {
    
    // Format: Pitch Rate | Roll Rate | Yaw Rate
    Serial.print("G_VEL -> P:"); Serial.print(mpu.getGyroX()); 
    Serial.print(" | R:");         Serial.print(mpu.getGyroY());
    Serial.print(" | Y:");         Serial.print(mpu.getGyroZ());

    // Joystick Comparison
    Serial.print("  ||  STICK -> P:"); Serial.print(incomingData.pitch);
    Serial.print(" R:");             Serial.println(incomingData.roll);
    
    timer = millis();
  }
}