// /*********
//   Rui Santos & Sara Santos - Random Nerd Tutorials
//   Complete instructions at https://RandomNerdTutorials.com/esp32-uart-communication-serial-arduino/
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
//   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// *********/
#include <Arduino.h>
// // Define TX and RX pins for UART (change if needed)
#define TXD1 16
#define RXD1 17

// Use Serial1 for UART communication
HardwareSerial mySerial(1);
// 
int counter = 0;
// 
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  // 
  Serial.println("ESP32 UART Transmitter");
}
// 
void loop() {
  // 
  // Send message over UART
  mySerial.println(String(counter));
  // 
  Serial.println("Sent: " + String(counter));
  
  while (mySerial.readStringUntil('\n').compareTo(String(counter))==0)
  {
    Serial.println(mySerial.readStringUntil('\n'));
  }  
  counter++;
}