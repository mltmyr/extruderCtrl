#ifndef PERIODICALLY_H__
#define PERIODICALLY_H__

#include <Arduino.h>

typedef void (*periodic_action_cb)(void*, byte);

class Periodically
{
public:
    Periodically(periodic_action_cb action, void* context, byte context_length, float frequency);
    ~Periodically();

    void setFrequency(float frequency);
    void start();
    void stop();

    void process();
private:
    periodic_action_cb action;
    void* context;
    byte length;
    boolean running;

    float period; // in ms
    unsigned long next_read_time; // in ms
};

#endif /* PERIODICALLY_H__ */
