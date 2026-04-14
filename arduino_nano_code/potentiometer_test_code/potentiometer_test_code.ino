/*
  Potentiometer Range Tester
  Reads analog input from pin A5 and prints to Serial Monitor.
*/

const int potPin = A5; // Pin connected to the slider wiper

void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  Serial.println("--- Potentiometer Range Test Started ---");
  Serial.println("Slide the pot from end to end to find your min/max values.");
}

void loop() {
  // Read the input on analog pin 5:
  int sensorValue = analogRead(potPin);
  
  // Calculate voltage for reference (5V scale)
  float voltage = sensorValue * (5.0 / 1023.0);

  // Print the results to the Serial Monitor:
  Serial.print("Raw Value: ");
  Serial.print(sensorValue);
  Serial.print(" \t Voltage: ");
  Serial.println(voltage);

  // Small delay for readability
  delay(50);
}