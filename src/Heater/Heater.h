#ifndef HEATER_H__
#define HEATER_H__

#include <Arduino.h>

class Heater
{
public:
    Heater(byte heater_pin, float rated_power);
    ~Heater();
    byte getPin();

    void setOutputHeat(float power);
    float getHeatingLevel();

private:
    byte  pin;
    float heater_rated_power;
    float power_to_byte;
    float heating_level;
};

#endif /* HEATER_H__ */
