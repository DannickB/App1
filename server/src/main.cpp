#include <Arduino.h>
#include <HardwareSerial.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(34, INPUT);

}

void loop() {
  delay(200);
  Serial.println(analogRead(34));
  // put your main code here, to run repeatedly:
}
