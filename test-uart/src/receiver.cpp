#include <Arduino.h>
#include <HardwareSerial.h>
HardwareSerial SerialPort(1);
char bitOfMessage = ' ';
std::string message = " ";
int msg = 0;

void setup() {
  // put your setup code here, to run once:
  SerialPort.begin(9600, SERIAL_8N1, 18, 19);
}

void loop() {
  // put your main code here, to run repeatedly:
  bitOfMessage = char(SerialPort.read());
  if (bitOfMessage != '\n'){
    message += bitOfMessage;
  }
  msg = std::stoi(message.substr(2,1));
  Serial.println(msg);
}
