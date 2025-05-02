#include "BLEDevice.h"
#include <Wire.h>
#include <Arduino.h>
#include "enum.h"
#include <HardwareSerial.h>

#define TXD1 12
#define RXD1 14

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

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID bmeServiceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");

// BLE Characteristics
static BLEUUID notificationCharacteristicUUID("cba1d466-344c-4be3-ab3f-189f80dd7518");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* notificationCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//Flags to check whether new temperature and humidity readings are available
boolean newNotification = false;

int sensor;
void readUartData(int sensor){
  String message;
  while (SerialPort.available() > 0) {
    message = SerialPort.readStringUntil('\n');
  }
  switch(sensor) {
    case sensorEnum::Barometer_Temperature:
      sensorStruc.Barometer_Temperature = message;
      break;
    case sensorEnum::Pressure:
      sensorStruc.Pressure = message;
      break;
    case sensorEnum::Humididy:
      sensorStruc.Humididy = message;
      break;
    case sensorEnum::Temperature:
      sensorStruc.Temperature = message;
      break;
    case sensorEnum::Light_level:
      sensorStruc.Light_level = message;
      break;
    case sensorEnum::Rainfall:
      sensorStruc.Rainfall = message;
      break;
    case sensorEnum::Wind_direction:
      sensorStruc.Wind_direction = message;
      break;
    case sensorEnum::Wind_Speed:
      sensorStruc.Wind_Speed = message;
      break;
    default:
      Serial.print("Upcode invalid");
  }
  delay(50);
  SerialPort.println(sensor);

}

void printValues(){
  Serial.println("Barometer Temperature: " + sensorStruc.Barometer_Temperature + "°C");
  Serial.println("Barometer Pressure: " + sensorStruc.Pressure + "kPa");
  Serial.println("Humidity: " + sensorStruc.Humididy + "%");
  Serial.println("Temperature: " + sensorStruc.Temperature + "°C");
  Serial.println("Light level: " + sensorStruc.Light_level);
  Serial.println("Total rainfall: " + sensorStruc.Rainfall + "mm");
  Serial.println("Wind direction: " + sensorStruc.Wind_Speed + "°");
  Serial.println("Wind speed: " + sensorStruc.Wind_direction + "km/h");
  Serial.println("*************************");
  Serial.println("");

}

//When the BLE Server sends a new temperature reading with the notify property
static void NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
  uint8_t* pData, size_t length, bool isNotify) {
sensor = std::atoi((char*)pData);
newNotification = true;
readUartData(sensor);
}


//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
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
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
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
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of each Characteristic
      notificationCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  //if new temperature readings are available, print in the OLED
  if (newNotification){
    newNotification = false;
    Serial.println("Waiting for new com...");
  }
  printValues();
  delay(1000); // Delay a second between loops.
}