#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "common.h"

#define TIMER_BIT 8
#define PWM_BASE_FREQ 15000
#define EN_RAMP 200

#define CONTROL_VEL 0
#define CONTROL_POS 1

class ControlAlgo
{
public:
    ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum);
    int ControlUpdate(float position, float velocity);
    void Disable();
    void SetTarget(float target);
    void SetGains(float gains[8], int type);
    void SetGain(float gain, int i, int relative, int type);

    float GetTarget(int type);
    float GetGain(int i, int type);
    void CalcGains(float wc, float wi, float wlp, float gain, float phase, int type, int print);
    void LeakGains(float wc, float wlp, float gain);
    void SetOutput(float output);

    float _pos_error, _vel_error;
    float _accel_out, _velocity_out, _pwm;

private:
    int _pwmChannel, _motorNum;
    int _DIR_GPIO, _PWM_GPIO;
    float _MIN_OUT, _MAX_OUT, _PWM_MAX;
    float _WINDUP;

    int _invertDir, _invertOut;

    float _gains_pos[8];
    float _inputs_pos[4];
    float _outputs_pos[4];

    float _gains_vel[8];
    float _inputs_vel[4];
    float _outputs_vel[4];

    float _pos_target;
    float _vel_target;
    float _ramp_time;

    void UpdateOutput(float output);
};

#endif