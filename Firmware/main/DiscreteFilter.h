#ifndef DISCRETE_FILTER_H
#define DISCRETE_FILTER_H

#include <Arduino.h>
#include <stdexcept>

class DiscreteFilter
{
public:
    DiscreteFilter(float max_val, float min_val, float windup);
    DiscreteFilter(void);
    void Update(float input);
    void Reset();

    // set functions
    void SetGain(int index, float value);
    void SetButterFilter(int order, float Ts, float cutOff);
    void SetHighPass(int order, float Ts, float cutOff);
    void SetUnity();
    void SetWindup(float max_val, float min_val, float windup);
    void DisableWindup(void);
    // get functions
    float GetLatest();
    float GetGain(int index);

private:
    int _size = 4;
    float _gains[8];
    float _inputs[4];
    float _outputs[4];

    int _en_windup;
    float _max_val, _min_val, _windup;
};

#endif