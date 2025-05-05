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

  if (deviceConnected) {

    // Send ping containing sensor data opcode so that the client knows which data to except
    String UARTsendData;
    static char buffer[6];

    sensor.UpdateSensors();
    sensor.ToSerial();

    unsigned long lastSentPing = millis();
    for (int i = 0; i < sensor_enum::enum_max; i++) {
      
      // dtostrf needs a float value, so we need to cast the int opcode to float
      dtostrf(static_cast<float>(i), 6, 2, buffer);
      // Setting opcode value, send ping, remember when the ping was sent
      notificationCharacteristic.setValue(buffer);
      lastSentPing = millis();
      
      // Send ping
      notificationCharacteristic.notify();

      switch (i) {
        case sensor_enum::Barometer_Temperature:
          UARTsendData = String(sensor.barometer_temperature);
          break;
        case sensor_enum::Pressure:
          UARTsendData = String(sensor.barometer_pressure);
          break;
        case sensor_enum::Huminidy:
          UARTsendData = String(sensor.humidity);
          break;
        case sensor_enum::Temperature:
          UARTsendData = String(sensor.temperature);
          break;
        case sensor_enum::Ligh_level:
          UARTsendData = String(sensor.light_level_voltage);
          break;
        case sensor_enum::Rainfall:
          UARTsendData = String(sensor.rain_count);
          break;
        case sensor_enum::Wind_direction:
          UARTsendData = String(sensor.wind_direction);
          break;
        case sensor_enum::Wind_Speed:
          UARTsendData = String(sensor.wind_speed);
          break;
        default:
          UARTsendData = "ERROR";
          break;
      }
      UARTsendData = String(i) + UARTsendData;
      Serial.println("Sending data to client: " + UARTsendData);
      mySerial.println(UARTsendData);

      delay(50);

    } // End of for loop
    delay(1000);
  } // Endif deviceConnected

  else {
    Serial.println("Client disconnected, waiting for new connection...");
    delay(1000);
  }
}