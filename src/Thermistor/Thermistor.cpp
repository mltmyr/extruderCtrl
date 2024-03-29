#include "Thermistor.h"

#include <math.h>

#if USE_THERMISTOR_TABLE

#else
#warning "Calculating temperature by expression!"

#define KELVIN_AT_0_CELSIUS          (273.15)
#define THERMISTOR_STD_CONFIG_VCC    (5.0)

#define MAX_READ_FREQUENCY           (50.0)

#define THERMISTOR_STD_CONFIG_R_VDIV (4700.0)
#define THERMISTOR_STD_CONFIG_R_T1   (100000.0)
#define THERMISTOR_STD_CONFIG_T1     (25)
#define THERMISTOR_STD_CONFIG_B      (4267)


Thermistor::Thermistor(uint8_t thermistor_pin, float read_freq)
{
    this->pin = thermistor_pin;

    Thermistor_config_t cfg;
    cfg.R_vdiv = THERMISTOR_STD_CONFIG_R_VDIV;
    cfg.R_T1   = THERMISTOR_STD_CONFIG_R_T1;
    cfg.T1     = THERMISTOR_STD_CONFIG_T1;
    cfg.B      = THERMISTOR_STD_CONFIG_B;
    this->configure(&cfg);

    this->setReadFreq(read_freq);
}

Thermistor::Thermistor(uint8_t thermistor_pin, Thermistor_config_t* config, float read_freq)
{
    this->pin = thermistor_pin;

    pinMode(this->pin, INPUT);

    this->configure(config);

    this->setReadFreq(read_freq);
}

Thermistor::~Thermistor()
{

}

void Thermistor::configure(Thermistor_config_t* config)
{
    this->c1 = 1.0/(config->T1+KELVIN_AT_0_CELSIUS) - (log(config->R_T1))/(config->B);
    this->c2 = 1.0/(config->B);
    this->c3 = KELVIN_AT_0_CELSIUS;

    this->R_T = config->R_T1;
    this->T   = config->T1;

    this->R_vdiv = config->R_vdiv;
    return;
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

float Thermistor::ReadTemp()
{
    int disc_T = analogRead(this->pin);
    
    this->R_T = (this->R_vdiv) * ((float)disc_T)/((float)(1023.0 - disc_T));

    float lnR_T = log(this->R_T);
    this->T = 1.0/(this->c1 + this->c2*lnR_T) - this->c3;

    return this->T;
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
