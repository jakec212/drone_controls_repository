//Transmitter test to see if we can send the joystick values to the drone reciever


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- PIN ASSIGNMENTS ---
// Left Joystick: Throttle & Yaw
#define PIN_YAW      A3 
#define PIN_THROTTLE A2 
// Right Joystick: Pitch & Roll
#define PIN_PITCH    A0 
#define PIN_ROLL     A1 

// --- CALIBRATION (Adjust these based on your specific joysticks) ---
#define A_LOW 0    
#define A_CENTER 512   
#define A_HIGH 1023

// Radio Setup: CE on D9, CSN on D10 [cite: 17]
RF24 radio(9, 10);
const byte address[6] = "00001"; // Must match the receiver [cite: 19]

// Define a structure to hold all 4 joystick values
struct ControlData {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

ControlData data; // Create an instance of our data structure

// Existing floating point mapping from your simulator code [cite: 1, 2]
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Maps stick to a standard 0-1023 range [cite: 3, 4]
int mapStick(int val, int low, int center, int high) {
  val = constrain(val, low, high);
  if (val < center)
    return (int)mapFloat(val, low, center, 0, 512);
  else
    return (int)mapFloat(val, center, high, 512, 1023);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Radio [cite: 20]
  if (!radio.begin()) {
    Serial.println("Error: Radio hardware not responding!");
    while (1); 
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN); // Keep power low for desk testing [cite: 21]
  radio.stopListening();         // Set as Transmitter [cite: 21]
  Serial.println("Transmitter Initialized.");
}

void loop() {
  // 1. Read Raw Values [cite: 7, 8]
  int rawYaw      = analogRead(PIN_YAW);
  int rawThrottle = analogRead(PIN_THROTTLE);
  int rawPitch    = analogRead(PIN_PITCH);
  int rawRoll     = analogRead(PIN_ROLL);

  // 2. Map and Store in Structure [cite: 9, 10]
  data.yaw      = mapStick(rawYaw,      A_LOW, A_CENTER, A_HIGH);
  data.throttle = mapStick(rawThrottle, A_LOW, A_CENTER, A_HIGH);
  data.pitch    = mapStick(rawPitch,    A_LOW, A_CENTER, A_HIGH);
  data.roll     = mapStick(rawRoll,     A_LOW, A_CENTER, A_HIGH);

  // 3. Send the entire structure via Radio [cite: 23]
  bool report = radio.write(&data, sizeof(data));

  // 4. Debug Output (optional) [cite: 13]
  if (report) {
    Serial.print("Sent -> T: "); Serial.print(data.throttle);
    Serial.print(" | Y: "); Serial.print(data.yaw);
    Serial.print(" | P: "); Serial.print(data.pitch);
    Serial.print(" | R: "); Serial.println(data.roll);
  } else {
    Serial.println("Transmission Failed");
  }
  
  delay(10); // Short delay for stability [cite: 16]
}