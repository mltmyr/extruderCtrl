#ifndef FAN_H__
#define FAN_H__

#include <Arduino.h>

class Fan
{
public:
    Fan(byte fan_pin);
    ~Fan();
    byte getPin();

    void setSpeed(byte fan_speed);
    byte getSpeed();

private:
    byte speed;
    byte pin;
};

#endif /* FAN_H__ */
