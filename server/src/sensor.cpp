#include "sensor.h"

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <SparkFun_Weather_Meter_Kit_Arduino_Library.h>

/// @file sensor.cpp
/// @brief Sensor class implementation
/// @details Integrates various sensors including a barometer, light meter, rain meter, anemometer, and humidometer.
/// @author Gabriel Labrecque, Dannick Bilodeau
/// @date 2025-05-02
/// @version 1.0

Sensor::Sensor() {};

Sensor::~Sensor() {};

void Sensor::Init() {

    // Initialize the barometer sensor
    if (! dps.begin_I2C()) {
      Serial.println("Failed to find DPS");
      while (1) yield();
    } 
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    Serial.println("DPS310 initialized");

    // Initialize the weather meter kit
    weatherMeterKit.begin();
    Serial.println("WeatherMeterKit initialized");

    pinMode(GPIO_LIGHTMETER, INPUT);
    Serial.println("gpio" + String(GPIO_LIGHTMETER) + " set to INPUT");
};

void Sensor::UpdateSensors() {

    // Reads the barometer sensor if available and store result
    if (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
        Serial.println("Barometer sensor not available");
        while (1) yield();
    }
    dps.getEvents(&barometer_temperature_event, &barometer_pressure_event);
    barometer_pressure = barometer_pressure_event.pressure / 10;
    barometer_temperature = barometer_temperature_event.temperature;

    // Reads the humidometer sensor if available and store result
    int i, j;
    int duree[42];
    unsigned long pulse;

    pinMode(GPIO_HUMIDOMETER, OUTPUT_OPEN_DRAIN);
    digitalWrite(GPIO_HUMIDOMETER, HIGH);
    delay(250);
    digitalWrite(GPIO_HUMIDOMETER, LOW);
    delay(20);
    digitalWrite(GPIO_HUMIDOMETER, HIGH);
    delayMicroseconds(40);
    pinMode(GPIO_HUMIDOMETER, INPUT_PULLUP);
    
    while (digitalRead(GPIO_HUMIDOMETER) == HIGH);
    i = 0;

    do {
            pulse = pulseIn(GPIO_HUMIDOMETER, HIGH);
            duree[i] = pulse;
            i++;
    } while (pulse != 0);
    
    if (i != 42) 
        Serial.printf(" Erreur timing \n"); 

    for (i=0; i<5; i++) {
        data[i] = 0;
        for (j = ((8*i)+1); j < ((8*i)+9); j++) {
        data[i] = data[i] * 2;
        if (duree[j] > 50) {
            data[i] = data[i] + 1;
        }
        }
    }

    if ( (data[0] + data[1] + data[2] + data[3]) != data[4] ) 
        Serial.println(" Erreur checksum");

    humidity = data[0] + (data[1] / 256.0);
    temperature = data [2] + (data[3] / 256.0);

    // Reads the lightmeter sensor if available and store result
    light_level = analogRead(GPIO_LIGHTMETER);
    light_level_voltage = static_cast<float>(analogRead(GPIO_LIGHTMETER)) / 4095.0 * 3.3;

    // Reads the rainmeter sensor if available and store result
    //wind_direction = static_cast<float>(analogRead(GPIO_ANOMEMETER_DIRECTION)) / 4095.0 * 360.0;
    wind_direction = weatherMeterKit.getWindDirection();
    wind_speed = weatherMeterKit.getWindSpeed();
    rain_count = weatherMeterKit.getTotalRainfall();
};

void Sensor::ToSerial() {
    Serial.println("");
    Serial.println("Barometer Temperature: " + String(barometer_temperature) + "°C");
    Serial.println("Barometer Pressure: " + String(barometer_pressure) + "kPa");
    Serial.println("Humidity: " + String(humidity) + "%");
    Serial.println("Temperature: " + String(temperature) + "°C");
    Serial.println("Light level: " + String(light_level) + ", (" + String(light_level_voltage) + "V)");
    Serial.println("Total rainfall: " + String(rain_count) + "mm");
    Serial.println("Wind direction: " + String(wind_direction) + "°");
    Serial.println("Wind speed: " + String(wind_speed) + "km/h");
    Serial.println("");
};