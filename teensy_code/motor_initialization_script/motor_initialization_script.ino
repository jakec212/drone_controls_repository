// Motor pins connected to the ESC signal inputs
const int motorPins[] = {1, 2, 3, 4};
const int numMotors = 4;

int start = 819;
int end = 0;
int val =0;
//prototype

void setup() {
  Serial.begin(9600);
  
  // Initialize all pins
  for (int i = 0; i < numMotors; i++) {
    // 200Hz frequency (5000us period)
    analogWriteFrequency(motorPins[i], 200); 
    // 12-bit resolution (0-4095)
    analogWriteResolution(12);
    pinMode(motorPins[i], OUTPUT);
    
    // Start with a safe 1000us signal immediately
    analogWrite(motorPins[i], 1638); 
  }

  Serial.println("--- QUAD MOTOR ESC MENU ---");
  Serial.println("Press '1' -> 2000us (All Motors - Calibration High)");
  Serial.println("Press '2' -> 1000us (All Motors - Arming/Idle)");
  Serial.println("Press '3' -> 1400us (All Motors - Low Throttle Test)");
  Serial.println("Press '4' -> 1700us (All Motors - Arming/Idle)");
  Serial.println("Press '5' -> 1150us (All Motors - Low Throttle Test)");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '1') {
      end = 1638;
      ramp(start, end);
      updateAllMotors(end, "2000us (HIGH)");
      start = end;
    } 
    else if (c == '2') {
      end = 819;
      ramp(start, end);
      updateAllMotors(819, "1000us (LOW)");
      start = 819;
    }
    else if (c == '3') {
      end = 1100;
      ramp(start, end);
      updateAllMotors(end, "1400us (HIGH)");
      start = end;
    } 
    else if (c == '4') {
      end = 1400;
      ramp(start, end);
      updateAllMotors(end, "1700us (LOW)");
      start = end;
    }
    else if (c == '5') {
      end = 942;
      ramp(start, end);
      updateAllMotors(end, "1150us (TEST)");
      start = end;
    }
  }
}

// Helper function to update all 4 pins at once
void updateAllMotors(int value, String label) {
  Serial.print("Sending ");
  Serial.println(label);
  for (int i = 0; i < numMotors; i++) {
    analogWrite(motorPins[i], value);
  }
}

void ramp(int start, int end){
  for(int i = start; i < end; i ++){
    updateAllMotors(i, "Ramping");
    Serial.print(i);
    //little delay
    delay(100);
  }
  

}
