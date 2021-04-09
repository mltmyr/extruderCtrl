#include "Stepper.h"

#define OCR3A_MIN (8)
#define OCR3A_MAX (65535)

#define TIMER_3_PRESCALER   (8)
#define MCU_CLOCK_FREQUENCY (16000000UL)
#define TIMER_TICK_LENGTH   ((float)TIMER_3_PRESCALER/(float)MCU_CLOCK_FREQUENCY)

class Ramper
{
public:    
    Ramper(float max_accel)
    {
        this->a_max = max_accel;
        /*this->x0 = 0.0;
        this->x1 = 0.0;
        this->t0 = 0;
        this->t1 = 0;*/
    }

    ~Ramper() {}

    void targetChanged(float r, float x0, unsigned long t0)
    {
        this->x1 = r;
        this->x0 = x0;
        this->t0 = t0;

        this->t1 = (unsigned long)(abs(this->x1 - this->x0)/((float)(this->a_max))) + this->t0;
        /*Serial.print("x0: ");
        Serial.print(this->x0, DEC);
        Serial.print(", x1: ");
        Serial.print(this->x1, DEC);*/
        /*Serial.print(", t0: ");
        Serial.print(this->t0, DEC);
        Serial.print(", t1: ");
        Serial.println(this->t1, DEC);*/
    }

    float getVal(unsigned long t)
    {
        float out;
        if (t > this->t1)
        {
            out = (this->x1);
        }
        else
        {
            out = (this->x0)*(this->t1 - t)/(this->t1 - this->t0) + (this->x1)*(t - this->t0)/(this->t1 - this->t0);
        }
        /*Serial.print("t: ");
        Serial.print(t, DEC);
        Serial.print(", out: ");*/
        //Serial.println(out, DEC);
        
        return out;
    }

private:
    float a_max;
    float x0;
    float x1;
    unsigned long t0;
    unsigned long t1;
};

typedef enum
{
    FORWARD    =  1,
    STANDSTILL =  0,
    BACKWARD   = -1,
} dir_t;

boolean stepper_is_initialized_m = false;

byte enable_pin_m;
byte   step_pin_m;
byte    dir_pin_m;
boolean current_step_state_m;
float   targetSteppingFreq_m;
float   steppingFreq_m;

dir_t   current_dir_m;
boolean change_dir_m;

Ramper* rmpr;

float         process_ramp_period_m; // in ms
unsigned long next_process_ramp_time_m; // in ms

void startTimer()
{
    TCCR3B |= (0 << CS32) | (1 << CS31) | (0 << CS30); // Enable timer with prescaler 8.
    return;
}

void stopTimer()
{
    TCCR3B |= (0 << CS32) | (0 << CS31) | (0 << CS30); // Disable timer.
    return;
}

void setOCR3A(unsigned long period)
{
    OCR3AH = 0x00FF & (period >> 8);
    OCR3AL = 0x00FF & period;
    return;
}

void clearTimer()
{
    TCNT3H = 0x00;
    TCNT3L = 0x00;
    return;
}

void setProcessFreq(float process_freq)
{
    if (process_freq > PROCESS_RAMPING_FREQ_MAX)
    {
        process_ramp_period_m = 1000.0/PROCESS_RAMPING_FREQ_MAX;
    }
    else
    {
        process_ramp_period_m = 1000.0/process_freq;
    }

    next_process_ramp_time_m = 0;
    return;
}

void stepper_init(byte step_pin, byte dir_pin, byte enable_pin, float process_freq)
{
    if (stepper_is_initialized_m == true)
    {
        return;
    }

    // Set module variables
    enable_pin_m = enable_pin;
    step_pin_m   = step_pin;
    dir_pin_m    = dir_pin;
    current_step_state_m = digitalRead(step_pin_m);
    
    targetSteppingFreq_m = 0.0;
    steppingFreq_m       = 0.0;

    current_dir_m = STANDSTILL;
    change_dir_m  = false;

    rmpr = new Ramper(STEPPER_FREQ_ACCELERATION_MAX);
    rmpr->targetChanged(0.0, 0.0, 0);

    PRR1 &= ~(1 << PRTIM3); // Enable TC3 in the Power Reduction Register 1.

    TCCR3A |= (1 << WGM31) | (1 << WGM30); // Operation: Fast PWM with TOP = OCR3A
    TCCR3B |= (1 << WGM33) | (1 << WGM32);


    pinMode(enable_pin_m, OUTPUT);
    pinMode(step_pin_m,   OUTPUT);
    pinMode(dir_pin_m,    OUTPUT);

    digitalWrite(enable_pin_m, HIGH);
    digitalWrite(dir_pin_m, LOW);
    digitalWrite(step_pin_m, HIGH);
    delay(1);
    digitalWrite(step_pin_m, LOW);

    

    setProcessFreq(process_freq);

    stepper_is_initialized_m = true;
    return;
}

void stepper_deinit()
{
    if (stepper_is_initialized_m == false)
    {
        return;
    }

    stopTimer();
    stepper_disable();
    delete rmpr;

    stepper_is_initialized_m = false;
    return;
}

void stepper_enable()
{
    if (stepper_is_initialized_m == false)
    {
        return;
    }

    digitalWrite(enable_pin_m, LOW); // Inverted => LOW is enable, HIGH is disable.
    //TIMSK3 |= (1 << OCIE3A); // Enable OC3A interrupt
    return;
}

void stepper_disable()
{
    if (stepper_is_initialized_m == false)
    {
        return;
    }

    TIMSK3 &= ~(1 << OCIE3A); // Disable OC3A interrupt
    digitalWrite(enable_pin_m, HIGH); // Inverted => LOW is enable, HIGH is disable.
    return;
}

void stepWithFreq(float freq)
{   
    dir_t new_dir;
    if (freq > 0)
    {
        new_dir = FORWARD;
    }
    else if (freq < 0)
    {
        new_dir = BACKWARD;
    }
    else
    {
        new_dir = STANDSTILL;
    }

    unsigned long OCR_val = (unsigned long)abs(1.0/(2*TIMER_TICK_LENGTH*freq));
    if (OCR_val > OCR3A_MAX)
    {
        OCR_val = OCR3A_MAX;
    }
    else if (OCR_val < OCR3A_MIN)
    {
        OCR_val = OCR3A_MIN;
    }

    switch (current_dir_m)
    {
    case FORWARD:
        switch (new_dir)
        {
        case FORWARD:
            setOCR3A(OCR_val);
            break;

        case BACKWARD:
            current_dir_m = BACKWARD;
            change_dir_m = true;
            setOCR3A(OCR_val);
            break;

        case STANDSTILL:
            TIMSK3 &= ~(1 << OCIE3A);
            stopTimer();
            //setOCR3A(OCR3A_MAX);
            current_dir_m = STANDSTILL;
            break;

        default:
            /* Should not hit! Do nothing. */
            break;
        }
        break;

    case BACKWARD:
        switch (new_dir)
        {
        case BACKWARD:
            setOCR3A(OCR_val);
            break;

        case FORWARD:
            current_dir_m = FORWARD;
            change_dir_m = true;
            setOCR3A(OCR_val);
            break;

        case STANDSTILL:
            TIMSK3 &= ~(1 << OCIE3A);
            stopTimer();
            //setOCR3A(OCR3A_MAX);
            current_dir_m = STANDSTILL;
            break;

        default:
            /* Should not hit! Do nothing. */
            break;
        }
        break;

    case STANDSTILL:
        switch (new_dir)
        {
        case STANDSTILL:
            /* Do nothing */
            break;

        case FORWARD:
            current_dir_m = FORWARD;
            change_dir_m = true;
            setOCR3A(OCR_val);
            clearTimer();
            TIMSK3 |= (1 << OCIE3A);
            startTimer();
            break;

        case BACKWARD:
            current_dir_m = BACKWARD;
            change_dir_m = true;
            setOCR3A(OCR_val);
            clearTimer();
            TIMSK3 |= (1 << OCIE3A);
            startTimer();
            break;

        default:
            /* Should not hit! Do nothing. */
            break;
        }
        break;

    default:
        /* Should not hit! Do nothing. */
        break;
    }
    
    return;
}

void stepper_setSteppingFrequency(float step_freq)
{
    if (stepper_is_initialized_m == false)
    {
        return;
    }

    if (step_freq >= STEP_FREQ_SIG_MAX)
    {
        targetSteppingFreq_m = STEP_FREQ_SIG_MAX;
    }
    else if (step_freq <= STEP_FREQ_SIG_MIN)
    {
        targetSteppingFreq_m = STEP_FREQ_SIG_MIN;
    }
    else
    {
        targetSteppingFreq_m = step_freq;
    }
    
    rmpr->targetChanged(targetSteppingFreq_m, steppingFreq_m, millis());

    return;
}

float stepper_getSteppingFrequency()
{
    if (stepper_is_initialized_m == false)
    {
        return 0;
    }

    return steppingFreq_m;
}

void stepper_process()
{
    if (stepper_is_initialized_m == false)
    {
        return;
    }

    unsigned long time = millis();
    if (time >= next_process_ramp_time_m)
    {
        steppingFreq_m = rmpr->getVal(time); // Get ramped stepping frequency

        stepWithFreq(steppingFreq_m);

        next_process_ramp_time_m = time + (unsigned long)process_ramp_period_m;
    }
    return;
}


ISR(TIMER3_COMPA_vect)
{
    uint8_t old_SREG = SREG;
    cli();

    if (change_dir_m == true)
    {
        if (current_dir_m == FORWARD)
        {
            digitalWrite(dir_pin_m, LOW);
        }
        else if (current_dir_m == BACKWARD)
        {
            digitalWrite(dir_pin_m, HIGH);
        }
        change_dir_m = false;
    }

    if (current_step_state_m == false)
    {
        digitalWrite(step_pin_m, HIGH);
    }
    else
    {
        digitalWrite(step_pin_m, LOW);
    }
    current_step_state_m = !current_step_state_m;

    SREG = old_SREG;
}
