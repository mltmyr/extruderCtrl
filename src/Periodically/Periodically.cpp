#include "Periodically.h"

#define PERIODIC_ACTION_MIN_FREQ 0.01
#define PERIODIC_ACTION_MAX_FREQ 1000

Periodically::Periodically(periodic_action_cb action, void* context, byte context_length, float frequency)
{
    this->action = action;
    this->context = context;
    this->length = context_length;

    this->setFrequency(frequency);

    this->running = false;
}

Periodically::~Periodically()
{

}

void Periodically::setFrequency(float frequency)
{
    if (this->period <= PERIODIC_ACTION_MIN_FREQ)
    {
        this->period = 1.0/PERIODIC_ACTION_MIN_FREQ;
    }
    else if (this->period >= PERIODIC_ACTION_MAX_FREQ)
    {
        this->period = 1.0/PERIODIC_ACTION_MAX_FREQ;
    }
    else
    {
        this->period = 1.0/frequency;
    }
    
    this->next_read_time = 0;
}

void Periodically::start()
{
    this->running = true;
    return;
}

void Periodically::stop()
{
    this->running = false;
    return;
}

void Periodically::process()
{
    if (this->running == true)
    {
        long int time = millis();
        if (time >= this->next_read_time)
        {
            this->action(this->context, this->length);
            
            this->next_read_time = time + this->period;
        }
    }
    return;
}
