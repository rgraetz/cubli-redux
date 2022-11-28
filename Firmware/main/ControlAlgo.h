#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

#define TIMER_BIT 8
#define PWM_BASE_FREQ 2000

class ControlAlgo
{
public:
    ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum);
    void ControlUpdate(float angle);
    void Disable();
    void SetTarget(float target);
    void SetGains(float gains[8]);
    void SetGain(float gain, int i, int relative);

    float GetTarget();
    float GetGain(int i);
    void CalcGains(float wc, float wi, float wlp, float gain, float phase);

    float _error, _out, _out_full, _pwm, _diff_error, _int_error;
    float _gains[8];

private:
    int _pwmChannel, _motorNum;
    int _DIR_GPIO, _PWM_GPIO;
    float _MIN_OUT, _MAX_OUT;
    float _WINDUP;

    int _invertDir, _invertOut;

    float _inputs[4];
    float _outputs[4];

    float _target;

    void UpdateOutput(float output);
};

#endif