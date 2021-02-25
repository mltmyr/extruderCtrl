#include "DebugLED.h"

void LED_startup_blink(byte led_pin)
{
    pinMode(led_pin, OUTPUT);

    digitalWrite(led_pin, HIGH);
    delay(500);
    digitalWrite(led_pin, LOW);
    delay(500);
    digitalWrite(led_pin, HIGH);
    delay(500);
    digitalWrite(led_pin, LOW);
    delay(500);

    pinMode(led_pin, INPUT);
    return;
}

void LED_init_done_blink(byte led_pin)
{
    pinMode(led_pin, OUTPUT);

    digitalWrite(led_pin, HIGH);
    delay(250);
    digitalWrite(led_pin, LOW);
    delay(250);
    digitalWrite(led_pin, HIGH);
    delay(250);
    digitalWrite(led_pin, LOW);
    delay(250);
    digitalWrite(led_pin, HIGH);
    delay(250);
    digitalWrite(led_pin, LOW);
    delay(250);
    digitalWrite(led_pin, HIGH);
    delay(250);
    digitalWrite(led_pin, LOW);
    delay(250);
    
    pinMode(led_pin, INPUT);
    return;
}

#define PROCESS_LED_FREQ 10
unsigned long next_process_time_m = 0;
boolean blink_ones_m = false;
byte blink_ones_led_pin = 0;
byte counter_m = 0;

void LED_debug_blink(byte led_pin)
{
    if (blink_ones_m == true)
    {
        return;
    }
    blink_ones_m = true;
    blink_ones_led_pin = led_pin;

    pinMode(blink_ones_led_pin, OUTPUT);
    return;
}

void stop_debug_blink()
{
    blink_ones_m = false;
    counter_m = 0;
    pinMode(blink_ones_led_pin, INPUT);
    return;
}

void LED_process()
{
    unsigned long time = millis();
    if (time >= next_process_time_m)
    {
        if (blink_ones_m == true)
        {
            counter_m++;
            switch (counter_m)
            {
            case 5:
                digitalWrite(blink_ones_led_pin, HIGH);
                break;
            
            case 10:
                digitalWrite(blink_ones_led_pin, LOW);
                stop_debug_blink();
                break;

            default:
                break;
            }
            
        }
        
        next_process_time_m = time + (unsigned long)(1000.0/PROCESS_LED_FREQ);
    }
    return;
}