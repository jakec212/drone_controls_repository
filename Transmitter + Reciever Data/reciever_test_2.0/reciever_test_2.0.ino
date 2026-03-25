//Reciever code for joystick control and data sending

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte address[6] = "00001";

struct ControlData {
  int throttle;
  int yaw;
  int pitch;
  int roll;
};

ControlData data;

void setup() {
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println("Error: Radio hardware not responding!");
    while (1);
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Receiver Initialized. Waiting for data...");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    
    Serial.print("Received -> T: "); Serial.print(data.throttle);
    Serial.print(" | Y: ");         Serial.print(data.yaw);
    Serial.print(" | P: ");         Serial.print(data.pitch);
    Serial.print(" | R: ");         Serial.println(data.roll);
  }
}