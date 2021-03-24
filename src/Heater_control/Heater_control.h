#ifndef HEATER_CONTROL_H__
#define HEATER_CONTROL_H__

#include <Arduino.h>

#include "../Thermistor/Thermistor.h"
#include "../PID/PID.h"
#include "../Heater/Heater.h"

class HeaterControl
{
public:
    HeaterControl(Thermistor* thrm, Pid* reg, Heater* htr, float ctrl_freq);
    ~HeaterControl();

    void setCtrlFreq(float ctrl_freq);
    void setTempRef(float ref);
    
    void processControl();

private:
    Thermistor* thermistor;
    Pid*        regulator;
    Heater*     heater;

    float temperatureReference;

    float period; // in ms
    unsigned long next_read_time; // in ms
};

#endif /* HEATER_CONTROL_H__ */
