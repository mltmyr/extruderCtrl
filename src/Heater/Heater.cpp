#include "Heater.h"

Heater::Heater(byte heater_pin)
{
    this->pin = heater_pin;
    this->heating_level = 0;
}

Heater::~Heater()
{

}

byte Heater::getPin()
{
    return this->pin;
}

void Heater::setOutputHeat(byte heat_level)
{
    this->heating_level = heat_level;

    analogWrite(this->pin, this->heating_level);
}

int Heater::getHeatingLevel()
{
    return this->heating_level;
}
