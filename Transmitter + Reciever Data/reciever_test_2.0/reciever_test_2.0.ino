#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>


//variables like within range function for the joystick values
//max and min roll and pitch Rate degrees/s (60 degrees/s max)(connect the 1023 from the joystick to what the input should be)
//max and min yaw rate degrees/s (30 degrees/s max)



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

  //set center joystick values with a safe range


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