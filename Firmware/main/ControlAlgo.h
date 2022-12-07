#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "common.h"

#define TIMER_BIT 8
#define PWM_BASE_FREQ 15000
#define EN_RAMP 200

#define CONTROL_GYRO 0
#define CONTROL_ACCEL 1

class ControlAlgo
{
public:
    ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum);
    int ControlUpdate(float gyro_angle, float acc_angle);
    void Disable();
    void SetTarget(float target);
    void SetGains(float gains[8], int type);
    void SetGain(float gain, int i, int relative, int type);

    float GetTarget();
    float GetGain(int i, int type);
    void CalcGains(float wc, float wi, float wlp, float gain, float phase, int type, int print);
    void LeakGains(float wc, float wlp, float gain);
    void SetOutput(float output);

    float _error1, _error2;
    float _accel_out, _velocity_out, _pwm;

private:
    int _pwmChannel, _motorNum;
    int _DIR_GPIO, _PWM_GPIO;
    float _MIN_OUT, _MAX_OUT, _PWM_MAX;
    float _WINDUP;

    int _invertDir, _invertOut;

    float _gains1[8];
    float _inputs1[4];
    float _outputs1[4];

    float _gains2[8];
    float _inputs2[4];
    float _outputs2[4];

    float _target;
    float _ramp_time;

    void UpdateOutput(float output);
};

#endif