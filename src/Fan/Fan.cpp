#include <Arduino.h>

#include "Fan.h"

Fan::Fan(uint8_t fan_pin)
{
    this->pin = fan_pin;
    pinMode(this->pin, OUTPUT);

    this->speed = 0;
}

Fan::~Fan()
{

}

uint8_t Fan::getPin()
{
    return this->pin;
}

void Fan::setSpeed(uint8_t fan_speed)
{
    this->speed = fan_speed;

    analogWrite(this->pin, this->speed);

    return;
}

uint8_t Fan::getSpeed()
{
    return this->speed;
}
