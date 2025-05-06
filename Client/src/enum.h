/// @file enum.h
/// @brief Client side enumeration header of sensor opcodes
/// @details Integrates various sensors including a barometer, light meter, rain meter, anemometer, and humidometer.
/// @author Gabriel Labrecque labg0902, Dannick Bilodeau bild2707
/// @date 2025-05-02
/// @version 1.0

#ifndef ENUM_H
#define ENUM_H


enum sensorEnum
{
    Barometer_Temperature = 0,
    Pressure = 1,
    Humididy = 2,
    Temperature = 3,
    Light_level = 4,
    Rainfall = 5,
    Wind_direction = 6,
    Wind_Speed = 7,
};

#endif