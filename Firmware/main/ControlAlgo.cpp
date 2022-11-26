#include "ControlAlgo.h"

ControlAlgo::ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum)
{
    _pwmChannel = motorNum - 1;
    _motorNum = motorNum;
    _invertDir = invertDir;
    _invertOut = 1;

    _DIR_GPIO = DIR_GPIO;
    _PWM_GPIO = PWM_GPIO;

    _MAX_OUT = (1 << TIMER_BIT) - 1;
    _MIN_OUT = -_MAX_OUT;

    _WINDUP = 0.05;

    pinMode(DIR_GPIO, OUTPUT);
    ledcSetup(_pwmChannel, PWM_BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM_GPIO, _pwmChannel);

    // default values
    _target = 0;
    _error = 0;
    _out = 0;
    for (int i = 0; i < 8; i++)
        _gains[i] = 0;
}

void ControlAlgo::SetTarget(float target)
{
    _target = target;
}

void ControlAlgo::SetGains(float gains[8])
{
    for (int i = 0; i < 8; i++)
        _gains[i] = gains[i];
    // _gains = gains;
}

void ControlAlgo::SetGain(float gain, int i, int relative)
{
    if (relative == 1)
        _gains[i] += gain;
    else
        _gains[i] = gain;
}

void ControlAlgo::Disable()
{
    _out = 0;
    UpdateOutput(_out);
    _inputs[0] = 0.0;
    _inputs[1] = 0.0;
    _inputs[2] = 0.0;
    _inputs[3] = 0.0;

    _outputs[0] = 0.0;
    _outputs[1] = 0.0;
    _outputs[2] = 0.0;
    _outputs[3] = 0.0;
}

void ControlAlgo::ControlUpdate(float angle)
{
    _error = (angle - _target);
    _inputs[3] = _inputs[2];
    _inputs[2] = _inputs[1];
    _inputs[1] = _inputs[0];
    _inputs[0] = _error;

    _outputs[3] = _outputs[2];
    _outputs[2] = _outputs[1];
    _outputs[1] = _outputs[0];
    _outputs[0] = (-_gains[1] * _outputs[1] - _gains[2] * _outputs[2] - _gains[3] * _outputs[3] + _gains[4] * _inputs[0] + _gains[5] * _inputs[1] + _gains[6] * _inputs[2] + _gains[7] * _inputs[3]) / _gains[0];

    // wind-up protection
    float scale = 1;
    if (_outputs[0] < _MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs[0];
    else if (_outputs[0] > _MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs[0];

    if (scale != 1)
    {
        _inputs[0] *= scale;
        _inputs[1] *= scale;
        _inputs[2] *= scale;
        _inputs[3] *= scale;

        _outputs[0] *= scale;
        _outputs[1] *= scale;
        _outputs[2] *= scale;
        _outputs[3] *= scale;
    }
    _out = _outputs[0];

    UpdateOutput(_outputs[0]);
}

void ControlAlgo::UpdateOutput(float output)
{
    // set direction GPIO
    if ((output > 0) ^ (_invertDir == 1))
        digitalWrite(_DIR_GPIO, HIGH);
    else
        digitalWrite(_DIR_GPIO, LOW);

    // set PWM duty cycle
    if (output > _MAX_OUT)
        ledcWrite(_pwmChannel, (_invertOut == 0) ? _MAX_OUT : 0);
    else if (output < _MIN_OUT)
        ledcWrite(_pwmChannel, (_invertOut == 0) ? _MAX_OUT : 0);
    else
        ledcWrite(_pwmChannel, (_invertOut == 0) ? abs(output) : (uint8_t)(_MAX_OUT - abs(output)));
    _pwm = ledcRead(_pwmChannel);
}