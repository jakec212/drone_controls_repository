// Observed values from your Serial data
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

float mapFloat(float val, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (val - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

float mapStick(float val, float low, float center, float high) {
  if (val < center)
    return mapFloat(val, low, center, -32767, 0);
  else
    return mapFloat(val, center, high, 0, 32767);
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read ONCE, use everywhere
  float rawA0 = analogRead(A0);
  float rawA1 = analogRead(15);
  float rawA2 = analogRead(A2);
  float rawA3 = analogRead(A3);

  float yaw      = mapStick(rawA0, A0_LOW, A0_CENTER, A0_HIGH);
  float throttle = mapStick(rawA1, A1_LOW, A1_CENTER, A1_HIGH);
  float pitch    = mapStick(rawA2, A2_LOW, A2_CENTER, A2_HIGH);
  float roll     = mapStick(rawA3, A3_LOW, A3_CENTER, A3_HIGH);

  Joystick.X(roll);
  Joystick.Y(pitch);
  Joystick.Z(throttle);
  Joystick.Zrotate(yaw);
  Joystick.send_now();

  Serial.print("RAW: ");
  Serial.print(rawA0); Serial.print(" ");
  Serial.print(rawA1); Serial.print(" ");
  Serial.print(rawA2); Serial.print(" ");
  Serial.println(rawA3);

  Serial.print("MAP: ");
  Serial.print(yaw);      Serial.print(" ");
  Serial.print(throttle); Serial.print(" ");
  Serial.print(pitch);    Serial.print(" ");
  Serial.println(roll);
  Serial.println("---");

  delay(10);
}