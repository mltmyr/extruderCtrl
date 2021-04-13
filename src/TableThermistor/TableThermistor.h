#ifndef TABLE_THERMISTOR_H__
#define TABLE_THERMISTOR_H__

#include <Arduino.h>

#include "../GlobalConfig.h"

#if USE_THERMISTOR_TABLE

typedef struct
{
    int16_t value;
    int16_t celsius;
} temperature_entry_t;

class Thermistor
{
public:
    Thermistor(uint8_t thermistor_pin, const temperature_entry_t thermistor_table[], const uint8_t thermistor_table_length, float read_freq);
    ~Thermistor();
    void setReadFreq(float read_freq);

    void PeriodicReadTemp();
    float getTemp();
private:
    void ReadTemp();

    uint8_t pin;
    const temperature_entry_t* tt;
    const uint8_t tt_len;
    float   T;
    int16_t a;

    float    period; // in ms
    boolean  read_periodically;
    uint32_t next_read_time; // in ms
};

#endif

#endif /*TABLE_THERMISTOR_H__*/
