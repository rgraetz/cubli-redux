#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

#define TIMER_BIT 8
#define PWM_BASE_FREQ 20000

class ControlAlgo
{
public:
    ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum);
    void ControlUpdate(float angle);
    void Disable();
    void SetTarget(float target);
    void SetGains(float gains[8]);
    void SetGain(float gain, int i, int relative);

    float _error, _out, _pwm;
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