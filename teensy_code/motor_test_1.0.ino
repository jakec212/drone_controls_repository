const int motorPins[4] = {1, 2, 3, 4};
const int PWM_FREQ = 50; 
const int PWM_RES = 12;

void setup() {
  Serial.begin(9600);
  analogWriteResolution(PWM_RES);
  
  for(int i=0; i<4; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWriteFrequency(motorPins[i], PWM_FREQ);
  }

  Serial.println("Starting Calibration Sweep...");
  Serial.println("Listen for the 'Low-Medium-High' chime.");

  // We will sweep from 900us to 1100us to find the ESC's "Zero"
  for (int us = 900; us <= 1100; us += 5) {
    int rawValue = (us * 4096) / 20000;
    
    for(int i=0; i<4; i++) {
      analogWrite(motorPins[i], rawValue);
    }
    
    Serial.print("Testing Signal: ");
    Serial.print(us);
    Serial.println("us");
    delay(200); // Give the ESC time to react
  }
}

void loop() {
  // If the sweep worked, the ESC should now be armed.
  // This will give a tiny nudge to Motor 1 only.
  int testPulse = (1150 * 4096) / 20000;
  analogWrite(motorPins[0], testPulse);
  delay(500);
  
  int idlePulse = (1000 * 4096) / 20000;
  analogWrite(motorPins[0], idlePulse);
  delay(2000);
}
