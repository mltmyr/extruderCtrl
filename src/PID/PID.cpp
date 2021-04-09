#include <Arduino.h>

#include "PID.h"

Pid::Pid(pid_cfg_t* cfg)
{
    this->P = cfg->Kp;
    this->I = cfg->Ki;
    this->D = cfg->Kd;
    this->i_sum = cfg->i_sum0;

    this->u_lim_low  = cfg->u_lim_low;
    this->u_lim_high = cfg->u_lim_high;

    this->u = 0;
    this->e = 0;
    this->e_prev = 0;
    this->t = 0;
    this->t_prev = 0;
}

Pid::Pid(float Kp, float Ki, float Kd, float i_sum0, float u_lim_low, float u_lim_high)
{
    this->P = Kp;
    this->I = Ki;
    this->D = Kd;
    this->i_sum = i_sum0;

    this->u_lim_low  = u_lim_low;
    this->u_lim_high = u_lim_high;

    this->u = 0;
    this->e = 0;
    this->e_prev = 0;
    this->t = 0;
    this->t_prev = 0;
}

Pid::~Pid()
{

}

float Pid::get_u(float error)
{
    this->e = error;

    this->t = millis();
    float T = (float)(this->t - this->t_prev)/1000.0;

    if (this->t_prev == 0.0)
    {
        this->t_prev = this->t;
        return this->u;
    }
    this->t_prev = this->t;

    float i_sum_temp;
    if (this->u_lim_low <= this->u && this->u <= this->u_lim_high) // Stop integration when at output limits
    {
        i_sum_temp = this->i_sum + (this->I)*T*(this->e);
    }

    this->u = (this->P)*(this->e) + (this->i_sum) + (this->D)*(this->e - this->e_prev)/T;
    this->e_prev = e;

    if (this->u <= this->u_lim_low) // Saturate output to output limits
    {
        this->u = this->u_lim_low;
    }
    else if (this->u >= this->u_lim_high)
    {
        this->u = this->u_lim_high;
    }
    else
    {
        this->i_sum = i_sum_temp;
    }

    return this->u;
}

void Pid::set_iSum(float i_sum0)
{
    this->i_sum = i_sum0;
    return;
}
