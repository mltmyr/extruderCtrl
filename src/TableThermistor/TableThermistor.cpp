#include "TableThermistor.h"

#if USE_THERMISTOR_TABLE
#warning "Calculating temperature by lookup table and interpolation!"

#define MAX_READ_FREQUENCY           (50.0)

Thermistor::Thermistor(byte thermistor_pin, const temperature_entry_t thermistor_table[], const uint8_t thermistor_table_length, float read_freq)
    : pin(thermistor_pin), T(0), tt(thermistor_table), tt_len(thermistor_table_length)
{
    this->setReadFreq(read_freq);
}

Thermistor::~Thermistor()
{

}

void Thermistor::setReadFreq(float read_freq)
{   
    if (read_freq <= 0.0)
    {
        this->read_periodically = false;
    }
    else if (read_freq > MAX_READ_FREQUENCY)
    {
        this->period = 1000.0/(MAX_READ_FREQUENCY);
        this->read_periodically = true;
    }
    else
    {
        this->period = 1000.0/read_freq;
        this->read_periodically = true;
    }

    this->next_read_time = 0;

    return;
}

void Thermistor::ReadTemp()
{
    //int a
    this->a = analogRead(this->pin);
    
    uint8_t l = 0;
    uint8_t r = this->tt_len;
    uint8_t m;

    for (;;)
    {
        m = (l + r) >> 1;
        
        if (!m)
        {
            this->T = int16_t(pgm_read_word(&(this->tt)[0].celsius));
            break;
        }
        if (m == l || m == r)
        {
            this->T = int16_t(pgm_read_word(&(this->tt)[this->tt_len-1].celsius));
            break;
        }
        
        int16_t a0 = int16_t(pgm_read_word(&(this->tt)[m-1].value));
        int16_t a1 = int16_t(pgm_read_word(&(this->tt)[m-0].value));
        if (a < a0)
        {
            r = m;
        }
        else if (a > a1)
        {
            l = m;
        }
        else
        {
            int16_t T0 = int16_t(pgm_read_word(&(this->tt)[m-1].celsius));
            int16_t T1 = int16_t(pgm_read_word(&(this->tt)[m-0].celsius));

            this->T = T0 + (a - a0) * float(T1 - T0)/float(a1 - a0);
            break;
        }
    }

    return;
}

void Thermistor::PeriodicReadTemp()
{
    if (!(this->read_periodically))
    {
        return;
    }

    unsigned long time = millis();
    if (time >= this->next_read_time)
    {
        this->ReadTemp();
        this->next_read_time = time + this->period;
    }

    return;
}

float Thermistor::getTemp()
{
    return this->T;
}

#endif
