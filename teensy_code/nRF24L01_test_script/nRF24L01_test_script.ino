#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

// Update these pins to match your wiring (CE, CSN)
RF24 radio(9, 10); 

void setup() {
  Serial.begin(9600);
  printf_begin();
  
  Serial.println(F("--- nRF24L01 Hardware Test ---"));

  // Start the radio hardware
  if (!radio.begin()) {
    Serial.println(F("CRITICAL: Radio hardware not responding!"));
    Serial.println(F("Check your wiring (VCC, GND, MISO, MOSI, SCK, CE, CSN)."));
    while (1); // Stop execution
  }

  // Test 1: Check SPI Connectivity
  if (radio.isChipConnected()) {
    Serial.println(F("SUCCESS: nRF24L01 is communicating over SPI."));
  } else {
    Serial.println(F("FAILED: nRF24L01 is NOT connected or damaged."));
  }

  // Test 2: Print Register Details
  // Healthy chips show specific addresses and settings.
  // Fried chips usually show all 0x00 or all 0xFF.
  Serial.println(F("\n--- Register Details ---"));
  radio.printDetails(); 
  
  Serial.println(F("\nDiagnostic Complete."));
}

void loop() {
  // Nothing to do here
}