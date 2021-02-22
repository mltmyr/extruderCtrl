#ifndef STEPPER_H__
#define STEPPER_H__

#include <arduino.h>

#define STEP_FREQ_SIG_MIN (-3840.0)
#define STEP_FREQ_SIG_MAX  (3840.0)

#define STEPPER_FREQ_ACCELERATION_MAX (10000)
#define PROCESS_RAMPING_FREQ_MAX (100)

void stepper_init(byte step_pin, byte dir_pin, byte enable_pin, float process_freq);
void stepper_deinit();

void stepper_enable();
void stepper_disable();

void  stepper_setSteppingFrequency(float step_freq);
float stepper_getSteppingFrequency();

void stepper_process();

#endif /* STEPPER_H__ */
