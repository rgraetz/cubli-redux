#ifndef DISCRETE_FILTER_H
#define DISCRETE_FILTER_H

#include <Arduino.h>
#include <stdexcept>

class DiscreteFilter
{
public:
    DiscreteFilter(void);
    void Update(float input);
    void Reset();

    // get functions
    void SetGain(int index, float value);
    void SetButterFilter(int order, float Ts, float cuttOff);
    // set functions
    float GetLatest();

private:
    int _size = 4;
    float _gains[8];
    float _inputs[4];
    float _outputs[4];
};

#endif