#include "Heater_control.h"

#define HEATER_CONTROL_MIN_FREQ   5
#define HEATER_CONTROL_MAX_FREQ  50

#define HEATER_CONTROL_TEMP_REF_INIT  0 // Celsius

HeaterControl::HeaterControl(Thermistor* thrm, Pid* reg, Heater* htr, float ctrl_freq)
{
    this->thermistor = thrm;
    this->regulator  = reg;
    this->heater     = htr;

    this->temperatureReference = HEATER_CONTROL_TEMP_REF_INIT;

    this->setCtrlFreq(ctrl_freq);
}

HeaterControl::~HeaterControl()
{

}

void HeaterControl::setCtrlFreq(float ctrl_freq)
{
    if (ctrl_freq <= HEATER_CONTROL_MIN_FREQ)
    {
        this->period = 1.0/HEATER_CONTROL_MIN_FREQ;
    }
    else if (ctrl_freq >= HEATER_CONTROL_MAX_FREQ)
    {
        this->period = 1.0/HEATER_CONTROL_MAX_FREQ;
    }
    else
    {
        this->period = 1.0/ctrl_freq;
    }
    
    this->next_read_time = 0;

    return;
}

void HeaterControl::setTempRef(float ref)
{
    this->temperatureReference = ref;
    return;
}

void HeaterControl::processControl()
{
    long int time = millis();
    if (time >= this->next_read_time)
    {
        /* Read temperature */
        float temperature = this->thermistor->getTemp();
        
        /* Calculate current error in temperature */
        float temperatureError = this->temperatureReference - temperature;

        /* Calculate control signal */
        float u = this->regulator->get_u(temperatureError);

        /* Output control signal to heater block */
        this->heater->setOutputHeat(round(u));

        /* Set next time for control calculations */
        this->next_read_time = time + this->period;
    }

    return;
}
