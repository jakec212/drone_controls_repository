/*
 * Teensy 4.0 FPV SkyDive Controller
 * Optimized for smoothness and simulator compatibility
 */

// --- CALIBRATION VALUES ---
#define A0_LOW 0    
#define A0_CENTER 740   
#define A0_HIGH 1023

#define A1_LOW 0    
#define A1_CENTER 774   
#define A1_HIGH 1023

#define A2_LOW 0    
#define A2_CENTER 778   
#define A2_HIGH 1023

#define A3_LOW 0   
#define A3_CENTER 778   
#define A3_HIGH 1023

// Helper function for floating point mapping (avoids integer truncation)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Maps stick to 0-1023 range (Standard HID resolution)
float mapStick(float val, float low, float center, float high) {
  // Constrain input to prevent math errors if stick over-travels
  val = constrain(val, low, high);
  
  if (val < center)
    return mapFloat(val, low, center, 0, 512); 
  else
    return mapFloat(val, center, high, 512, 1023);
}

void setup() {
  Serial.begin(115200);
  // Ensure "Joystick" is enabled in Tools -> USB Type
  analogReadResolution(10); // Matches the 1023 max value
}

void loop() {
  // Read Raw Analog Values
  float rawYaw      = analogRead(A0);
  float rawThrottle = analogRead(15); // Pin 15 (A1)
  float rawPitch    = analogRead(A2);
  float rawRoll     = analogRead(A3);

  // Map to 10-bit Joystick Values (0 to 1023)
  int outYaw      = (int)mapStick(rawYaw,      A0_LOW, A0_CENTER, A0_HIGH);
  int outThrottle = (int)mapStick(rawThrottle, A1_LOW, A1_CENTER, A1_HIGH);
  int outPitch    = (int)mapStick(rawPitch,    A2_LOW, A2_CENTER, A2_HIGH);
  int outRoll     = (int)mapStick(rawRoll,     A3_LOW, A3_CENTER, A3_HIGH);

  // Update HID Joystick (Simulator sees these)
  Joystick.X(outRoll);
  Joystick.Y(outPitch);
  Joystick.Z(outThrottle);
  Joystick.Zrotate(outYaw);
  
  // Optional: Explicitly send the packet
  Joystick.send_now();

  // --- DEBUG SERIAL OUTPUT ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) { // Print every 100ms so it doesn't lag the loop
    Serial.print("RAW (Y T P R): ");
    Serial.print(rawYaw); Serial.print(" | ");
    Serial.print(rawThrottle); Serial.print(" | ");
    Serial.print(rawPitch); Serial.print(" | ");
    Serial.println(rawRoll);

    Serial.print("MAP (Y T P R): ");
    Serial.print(outYaw); Serial.print(" | ");
    Serial.print(outThrottle); Serial.print(" | ");
    Serial.print(outPitch); Serial.print(" | ");
    Serial.println(outRoll);
    Serial.println("-----------------------");
    lastPrint = millis();
  }

  delay(5); // Fast enough for low-latency flight (200Hz)
}