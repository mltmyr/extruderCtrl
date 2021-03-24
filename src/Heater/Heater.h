#ifndef HEATER_H__
#define HEATER_H__

#include <Arduino.h>

class Heater
{
public:
    Heater(byte heater_pin);
    ~Heater();
    byte getPin();

    void setOutputHeat(byte heat_level);
    int getHeatingLevel();

private:
    byte pin;

    byte heating_level;
};

#endif /* HEATER_H__ */
