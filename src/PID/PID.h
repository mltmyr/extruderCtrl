#ifndef PID_H__
#define PID_H__

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float i_sum0;
    float u_lim_low;
    float u_lim_high;
} pid_cfg_t;

class Pid
{
public:
    Pid(pid_cfg_t* cfg);
    Pid(float Kp, float Ki, float Kd, float i_sum0, float u_lim_low, float u_lim_high);
    ~Pid();

    float get_u(float error);
    void  set_iSum(float i_sum0);

private:
    float P;
    float I;
    float D;

    float i_sum;

    float u_lim_low;
    float u_lim_high;

    float e;
    float e_prev;

    uint32_t t;
    uint32_t t_prev;

    float u;
};

#endif /* PID_H__ */
