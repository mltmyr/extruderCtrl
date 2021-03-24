#include <Arduino.h>

#include "Fan.h"

Fan::Fan(byte fan_pin)
{
    this->pin = fan_pin;
    pinMode(this->pin, OUTPUT);

    this->speed = 0;
}

Fan::~Fan()
{

}

byte Fan::getPin()
{
    return this->pin;
}

void Fan::setSpeed(byte fan_speed)
{
    this->speed = fan_speed;

    analogWrite(this->pin, this->speed);

    return;
}

byte Fan::getSpeed()
{
    return this->speed;
}
