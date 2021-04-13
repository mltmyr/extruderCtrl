#ifndef FAN_H__
#define FAN_H__

#include <Arduino.h>

class Fan
{
public:
    Fan(uint8_t fan_pin);
    ~Fan();
    uint8_t getPin();

    void setSpeed(uint8_t fan_speed);
    uint8_t getSpeed();

private:
    uint8_t speed;
    uint8_t pin;
};

#endif /* FAN_H__ */
