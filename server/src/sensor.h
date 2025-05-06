/// @file sensor.h
/// @brief Sensor class header
/// @details Integrates various sensors including a barometer, light meter, rain meter, anemometer, and humidometer.
/// @author Gabriel Labrecque labg0902, Dannick Bilodeau bild2707
/// @date 2025-05-02
/// @version 1.0

#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <SparkFun_Weather_Meter_Kit_Arduino_Library.h>

#define GPIO_LIGHTMETER 34
#define GPIO_RAINMETER 23
#define GPIO_ANOMEMETER_DIRECTION 35
#define GPIO_ANOMEMETER_SPEED 27
#define GPIO_HUMIDOMETER 16

class Sensor {
public:

    // Members
    Adafruit_DPS310 dps;
    sensors_event_t barometer_pressure_event, barometer_temperature_event;
    float barometer_pressure, barometer_temperature;

    byte data[5];
    float wind_speed, wind_direction;
    float humidity, temperature;
    int light_level;
    float light_level_voltage;
    int rain_count;

    SFEWeatherMeterKit weatherMeterKit = SFEWeatherMeterKit(GPIO_ANOMEMETER_DIRECTION, GPIO_ANOMEMETER_SPEED, GPIO_RAINMETER);

    // Constructor and Destructor
    Sensor();
    ~Sensor();

    // Methods
    void Init();
    void UpdateSensors();
    void ToSerial();
};

#endif