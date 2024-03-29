#ifndef THERMISTOR_H__
#define THERMISTOR_H__

#include <Arduino.h>

#include "../GlobalConfig.h"

#if USE_THERMISTOR_TABLE

#else
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
    Thermistor(uint8_t thermistor_pin, float read_freq);
    Thermistor(uint8_t thermistor_pin, Thermistor_config_t* config, float read_freq);
    ~Thermistor();

    void configure(Thermistor_config_t* config);
    void setReadFreq(float read_freq);

    float ReadTemp();

    void PeriodicReadTemp();
    float getTemp();

private:
    uint8_t pin;

    float R_T;
    float T;

    float c1;
    float c2;
    float c3;
    float R_vdiv;

    float period; // in ms
    boolean read_periodically;
    uint32_t next_read_time; // in ms

};
#endif

#endif /*THERMISTOR_H__*/
