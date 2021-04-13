#ifndef DEBUG_LED_H__
#define DEBUG_LED_H__

#include <Arduino.h>

void LED_startup_blink(uint8_t led_pin);

void LED_init_done_blink(uint8_t led_pin);

void LED_debug_blink(uint8_t led_pin);

void LED_process();

#endif /* DEBUG_LED_H__ */
