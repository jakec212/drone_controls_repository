//Reciever code for the ping test


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// CE on D9, CSN on D10
RF24 radio(9, 10);

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  
  if (!radio.begin()) {
    Serial.println("Error: Radio hardware not responding!");
    while (1);
  }

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); // Set as Receiver
  Serial.println("Receiver Initialized. Waiting for Sophia...");
}

void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    
    Serial.print("Data Received: ");
    Serial.println(text);
  }
}