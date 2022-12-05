#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "common.h"

#define TIMER_BIT 8
#define PWM_BASE_FREQ 15000
#define EN_RAMP 200

class ControlAlgo
{
public:
    ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum);
    int ControlUpdate(float angle);
    void Disable();
    void SetTarget(float target);
    void SetGains(float gains[8]);
    void SetGain(float gain, int i, int relative);

    float GetTarget();
    float GetGain(int i);
    void CalcGains(float wc, float wi, float wlp, float gain, float phase, int print);
    void SetOutput(float output);

    float _error;
    float _out_calc, _out, _pwm;

private:
    int _pwmChannel, _motorNum;
    int _DIR_GPIO, _PWM_GPIO;
    float _MIN_OUT, _MAX_OUT, _PWM_MAX;
    float _WINDUP;

    int _invertDir, _invertOut;

    float _gains[8];
    float _inputs[4];
    float _outputs[4];

    float _target;
    float _ramp_time;

    void UpdateOutput(float output);
};

#endif