#include "BLEDevice.h"
#include <Wire.h>
#include <Arduino.h>
#include "enum.h"
#include <HardwareSerial.h>

//Rx Tx pins
#define TXD1 12
#define RXD1 14

//Struct containing our sensors info
struct {
  String Barometer_Temperature;
  String Pressure;
  String Humididy;
  String Temperature;
  String Light_level;
  String Rainfall;
  String Wind_direction;
  String Wind_Speed;
} sensorStruc;

// Use Serial1 for UART communication
HardwareSerial SerialPort(2);

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "Base Station Server"

// BLE Service and characteristic
static BLEUUID bmeServiceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");
static BLEUUID notificationCharacteristicUUID("cba1d466-344c-4be3-ab3f-189f80dd7518");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* notificationCharacteristic;

//Activate notify callback
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//Flags to check whether new temperature and humidity readings are available
boolean newNotification = false;

//Read data from Uart port and store it in appropriate variable
void readUartData(){
  String message;
  while (SerialPort.available() > 0) {
    message = SerialPort.readStringUntil('\n');
  }
  int upcode = (int)message[0] - 48;
  String data = message.substring(1);
  switch(upcode) {
    case sensorEnum::Barometer_Temperature:
      sensorStruc.Barometer_Temperature = data;
      break;
    case sensorEnum::Pressure:
      sensorStruc.Pressure = data;
      break;
    case sensorEnum::Humididy:
      sensorStruc.Humididy = data;
      break;
    case sensorEnum::Temperature:
      sensorStruc.Temperature = data;
      break;
    case sensorEnum::Light_level:
      sensorStruc.Light_level = data;
      break;
    case sensorEnum::Rainfall:
      sensorStruc.Rainfall = data;
      break;
    case sensorEnum::Wind_direction:
      sensorStruc.Wind_direction = data;
      break;
    case sensorEnum::Wind_Speed:
      sensorStruc.Wind_Speed = data;
      break;
    default:
      Serial.println("\nUpcode invalid\n\n");
  }

}
//Print Received variables
void printValues(){
  Serial.println("Barometer Temperature(°C): " + sensorStruc.Barometer_Temperature);
  Serial.println("Barometer Pressure(kPa): " + sensorStruc.Pressure );
  Serial.println("Humidity(%): " + sensorStruc.Humididy);
  Serial.println("Temperature(°C): " + sensorStruc.Temperature);
  Serial.println("Light level(V): " + sensorStruc.Light_level);
  Serial.println("Total rainfall(mm): " + sensorStruc.Rainfall);
  Serial.println("Wind direction(°): " + sensorStruc.Wind_direction);
  Serial.println("Wind speed(km/h): " + sensorStruc.Wind_Speed);
  Serial.println("*************************");
  Serial.println("");

}

//Receive notification
static void NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
  uint8_t* pData, size_t length, bool isNotify) {
newNotification = true;
readUartData();
}


//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Get server ref
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
 
  // Get characteristic ref
  notificationCharacteristic = pRemoteService->getCharacteristic(notificationCharacteristicUUID);

  if (notificationCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  notificationCharacteristic->registerForNotify(NotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); 
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); 
      doConnect = true; //Set flag, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

void setup() {
  
  //Start serial communication
  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  Serial.println("Starting Arduino BLE Client application...");

  //Init BLE device
  BLEDevice::init("");
 
  // Scans for BLE server
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  // See if we need to connect to BLE server and subscrib to it's characteristic
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of our Characteristic
      notificationCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  //Reset variable to wait for new notification
  if (newNotification){
    newNotification = false;
    Serial.println("Waiting for new com...");
  }
  printValues();
  delay(1000); // Delay a second between loops.
}