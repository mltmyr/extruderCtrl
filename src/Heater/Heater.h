#ifndef HEATER_H__
#define HEATER_H__

#include <Arduino.h>

class Heater
{
public:
    Heater(uint8_t heater_pin, float rated_power);
    ~Heater();
    uint8_t getPin();

    void setOutputHeat(float power);
    float getHeatingLevel();

private:
    uint8_t pin;
    float   heater_rated_power;
    float   power_to_byte;
    float   heating_level;
};

#endif /* HEATER_H__ */
