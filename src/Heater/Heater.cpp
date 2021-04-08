#include "Heater.h"

Heater::Heater(byte heater_pin, float rated_power)
{
    this->pin = heater_pin;
    this->heater_rated_power = rated_power;
    this->power_to_byte = (float)(255.0/rated_power);
    this->heating_level = 0;
}

Heater::~Heater()
{
    analogWrite(this->pin, 0);
}

byte Heater::getPin()
{
    return this->pin;
}

void Heater::setOutputHeat(float power)
{
    this->heating_level = power;

    analogWrite(this->pin, (byte)(this->heating_level*this->power_to_byte));
}

float Heater::getHeatingLevel()
{
    return this->heating_level;
}
