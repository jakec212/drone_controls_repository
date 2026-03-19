//Transmitter Code for the ping test


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// CE on D9, CSN on D10
RF24 radio(9, 10); 

int counter = 0; // Initialize counter at the top of your sketch

const byte address[6] = "00001"; // Must match the receiver

void setup() {
  Serial.begin(9600);
  
  // Check if the Nano can actually see the NRF24L01 chip
  if (!radio.begin()) {
    Serial.println("Error: Radio hardware not responding! Check your wiring.");
    while (1); 
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN); // Keep power low for desk testing
  radio.stopListening();         // Set as Transmitter
  Serial.println("Transmitter Initialized.");
}

void loop() {
  char text[32]; 
  // This "prints" the text and the counter into the 'text' buffer
  sprintf(text, "Hello Sophia %d", counter); 

  bool report = radio.write(&text, sizeof(text));

  if (report) {
    Serial.print("Ping Sent: ");
    Serial.print(text);
    Serial.println(" - Success!");
    counter++; // Only increment if the message actually sent
  } else {
    Serial.println("Ping Sent: Failed (No Ack)");
  }
  
  delay(0); 
}