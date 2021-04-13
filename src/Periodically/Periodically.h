#ifndef PERIODICALLY_H__
#define PERIODICALLY_H__

#include <Arduino.h>

typedef void (*periodic_action_cb)(void*, uint8_t);

class Periodically
{
public:
    Periodically(periodic_action_cb action, void* context, uint8_t context_length, float frequency);
    ~Periodically();

    void setFrequency(float frequency);
    void start();
    void stop();

    void process();
private:
    periodic_action_cb action;
    void* context;
    uint8_t length;
    boolean running;

    float    period; // in ms
    uint32_t next_read_time; // in ms
};

#endif /* PERIODICALLY_H__ */
