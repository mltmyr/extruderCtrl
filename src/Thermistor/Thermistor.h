#ifndef THERMISTOR_H__
#define THERMISTOR_H__

#include <Arduino.h>

typedef struct
{
    float R_vdiv; // Voltage division resistor, 4.7kOhm
    float R_T1;   // Thermistor Resistance at T1 Celsius, 100kOhm
    float T1;     // Temperature in Celsius giving R_T1 Resistance, 25Celsius
    float B;      // Thermistor coefficient, 4267Kelvin
} Thermistor_config_t;

class Thermistor
{
public:
    Thermistor(byte thermistor_pin, float read_freq);
    Thermistor(byte thermistor_pin, Thermistor_config_t* config, float read_freq);
    ~Thermistor();

    void configure(Thermistor_config_t* config);
    void setReadFreq(float read_freq);

    float ReadTemp();

    void PeriodicReadTemp();
    float getTemp();

private:
    byte pin;

    float R_T;
    float T;

    float c1;
    float c2;
    float c3;
    float R_vdiv;

    float period; // in ms
    boolean read_periodically;
    unsigned long next_read_time; // in ms

};

#endif /*THERMISTOR_H__*/
