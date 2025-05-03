/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#define FLAG_SERVER

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "sensor.h"
#include "enum.h"

//BLE server name
#define bleServerName "Base Station Server"
#define TX 4
#define RX 13

HardwareSerial mySerial(1);
Sensor sensor;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 3000;
bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"


BLECharacteristic notificationCharacteristic("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeNotificationDescriptor(BLEUUID((uint16_t)0x2902));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};


void setup() {
  // Start serial communication 
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RX, TX);
  sensor.Init();

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *notificationService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // Temperature
  notificationService->addCharacteristic(&notificationCharacteristic);
  bmeNotificationDescriptor.setValue("BME temperature Celsius");
  notificationCharacteristic.addDescriptor(&bmeNotificationDescriptor);
  
  // Start the service
  notificationService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  int counter = 0;
  while (!deviceConnected) {
    delay(1000);
    if (counter > 20) { 
      Serial.println("ESP could not find client, restarting...");
      ESP.restart();
    }
  }

  /*if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      
      String str = "42";
      value = 42;
      // Notify temperature reading from BME sensor
      static char valueTemp[6];
      dtostrf(value, 6, 2, valueTemp);
      // Set temperature Characteristic value and notify connected client
      notificationCharacteristic.setValue("str");
      notificationCharacteristic.notify();
      Serial.println("Sending notification to client");

      lastTime = millis();
    }
  }*/

  if (deviceConnected) {

    // Update sensorial data
    delay(3000);

    // Send ping containing sensor data opcode so that the client knows which data to except
    String UARTsendData;
    static char buffer[6];

    unsigned long lastSentPing = millis();
    for (int i = 0; i < sensor_enum::enum_max; i++) {

      sensor.UpdateSensors();
      sensor.ToSerial();
      
      // dtostrf needs a float value, so we need to cast the int opcode to float
      dtostrf(static_cast<float>(i), 6, 2, buffer);
      // Setting opcode value, send ping, remember when the ping was sent
      notificationCharacteristic.setValue(buffer);
      lastSentPing = millis();
      
      // Send ping
      notificationCharacteristic.notify();
      Serial.println("Sending notification to client : " + String(i));
      delay(100);

      switch (i) {
        case sensor_enum::Barometer_Temperature:
          UARTsendData = String(sensor_enum::Barometer_Temperature + sensor.barometer_temperature);
          break;
        case sensor_enum::Pressure:
          UARTsendData = String(sensor_enum::Pressure + sensor.barometer_pressure);
          break;
        case sensor_enum::Huminidy:
          UARTsendData = String(sensor_enum::Huminidy +sensor.humidity);
          break;
        case sensor_enum::Temperature:
          UARTsendData = String(sensor_enum::Temperature +sensor.temperature);
          break;
        case sensor_enum::Ligh_level:
          UARTsendData = String(sensor_enum::Ligh_level +sensor.light_level);
          break;
        case sensor_enum::Rainfall:
          UARTsendData = String(sensor_enum::Rainfall + sensor.rain_count);
          break;
        case sensor_enum::Wind_direction:
          UARTsendData = String(sensor_enum::Wind_direction +sensor.wind_direction);
          break;
        case sensor_enum::Wind_Speed:
          UARTsendData = String(sensor_enum::Wind_Speed +sensor.wind_speed);
          break;
        default:
          UARTsendData = "ERROR";
          break;
      }
      mySerial.println(UARTsendData);

      // while (!mySerial.available()) {
      // }

      UARTsendData = mySerial.readStringUntil('\n');
      Serial.println("Received from client: " + UARTsendData);
      if (UARTsendData.toInt() == i) {
        Serial.println("ACK received : " + String(i));
        break;
      }
      else {
        Serial.println("ERROR, received: " + UARTsendData + " expected: " + String(i));
      }

      // Wait for opcode ACK trough 


      delay(500);
    }
  }
}