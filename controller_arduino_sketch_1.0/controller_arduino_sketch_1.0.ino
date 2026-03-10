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

int mapStick(int val, int low, int center, int high) {
  if (val < center)
    return map(val, low, center, -512, 0);
  else
    return map(val, center, high, 0, 512);
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  int yaw      = mapStick(analogRead(A0), A0_LOW, A0_CENTER, A0_HIGH);
  int throttle = mapStick(analogRead(15),  A1_LOW, A1_CENTER, A1_HIGH);
  int pitch    = mapStick(analogRead(A2), A2_LOW, A2_CENTER, A2_HIGH);
  int roll     = mapStick(analogRead(A3), A3_LOW, A3_CENTER, A3_HIGH);

  Joystick.X(      roll);
  Joystick.Y(      pitch);
  Joystick.Z(      throttle);
  Joystick.Zrotate(yaw);
  Joystick.send_now();

  Serial.print("Y:"); Serial.print(yaw);
  Serial.print("  T:"); Serial.print(throttle);
  Serial.print("  P:"); Serial.print(pitch);
  Serial.print("  R:"); Serial.println(roll);

  delay(10);
}