#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "common.h"
#include "DiscreteFilter.h"

#define TIMER_BIT 8
#define PWM_BASE_FREQ 15000
#define EN_RAMP 200

#define CONTROL_VEL 0
#define CONTROL_POS 1
#define CONTROL_BAL 2
#define CONTROL_FULL 3

class ControlAlgo
{
public:
    ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum);
    int ControlUpdate(float position, float velocity, int set_out);
    int ControlUpdate(float position, float velocity, int set_out, int en_pos_loop, int en_bal_loop, int en_dither);
    void Disable();
    void SetTarget(float target);
    void SetGains(float gains[8], int type);

    float GetTarget(int type);
    float GetGain(int i, int type);
    void CalcGains(float fc, float fi, float flp, float gain, float phase, int type, int print);
    void LeakGains(float wc, float wlp, float gain);
    void SetOutput(float output);

    float _pos_error, _vel_error, _bal_error;
    float _accel_out, _velocity_out, _pwm;

private:
    int _pwmChannel, _motorNum;
    int _DIR_GPIO, _PWM_GPIO;
    float _MIN_OUT, _MAX_OUT, _PWM_MAX, _MAX_SPEED, _MIN_SPEED;
    float _WINDUP;

    int _invertDir, _invertOut;

    float _bal_target, _pos_target, _vel_target;
    float _ramp_time;
    float _dither;

    DiscreteFilter _bal_loop, _pos_loop, _vel_loop;

    void UpdateOutput(float output);
};

#endif