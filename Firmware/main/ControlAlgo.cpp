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
    _diff_error = 0;
    _int_error = 0;
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

void ControlAlgo::CalcGains(float wc, float wi, float wlp, float gain, float phase)
{
    double Ts = 0.01;
    double Ts2 = Ts * Ts;
    double Ts3 = Ts * Ts;
    double a = phase;

    double a0, a1, a2, a3;
    double b0, b1, b2, b3;
    double K = gain * wi;
    if (phase > 0)
    {
        a0 = (2 * Ts2 * a * wc * wi * wlp + 4 * Ts * a * wc * wi + 4 * Ts * wi * wlp + 8 * wi);
        a1 = (2 * Ts2 * a * wc * wi * wlp - 4 * Ts * a * wc * wi - 4 * Ts * wi * wlp - 24 * wi);
        a2 = (-2 * Ts2 * a * wc * wi * wlp - 4 * Ts * a * wc * wi - 4 * Ts * wi * wlp + 24 * wi);
        a3 = (-2 * Ts2 * a * wc * wi * wlp + 4 * Ts * a * wc * wi + 4 * Ts * wi * wlp - 8 * wi);

        b0 = (K * Ts3 * a * wc * wi * wlp + 2 * K * Ts2 * a * a * wi * wlp + 2 * K * Ts2 * a * wc * wlp + 4 * K * Ts * a * a * wlp);
        b1 = (3 * K * Ts3 * a * wc * wi * wlp + 2 * K * Ts2 * a * a * wi * wlp + 2 * K * Ts2 * a * wc * wlp - 4 * K * Ts * a * a * wlp);
        b2 = (3 * K * Ts3 * a * wc * wi * wlp - 2 * K * Ts2 * a * a * wi * wlp - 2 * K * Ts2 * a * wc * wlp - 4 * K * Ts * a * a * wlp);
        b3 = (K * Ts3 * a * wc * wi * wlp - 2 * K * Ts2 * a * a * wi * wlp - 2 * K * Ts2 * a * wc * wlp + 4 * K * Ts * a * a * wlp);
    }
    else
    {
        b0 = (K * Ts2 * wi * wlp + 2 * K * Ts * wlp);
        b1 = 2 * K * Ts2 * wi * wlp;
        b2 = K * Ts2 * wi * wlp - 2 * K * Ts * wlp;
        b3 = 0;

        a0 = (2 * Ts * wi * wlp + 4 * wi);
        a1 = -8 * wi;
        a2 = -2 * Ts * wi * wlp + 4 * wi;
        a3 = 0;
    }

    _out = 0;
    _diff_error = 0;
    _int_error = 0;
    UpdateOutput(_out);
    _inputs[0] = 0.0;
    _inputs[1] = 0.0;
    _inputs[2] = 0.0;
    _inputs[3] = 0.0;

    _outputs[0] = 0.0;
    _outputs[1] = 0.0;
    _outputs[2] = 0.0;
    _outputs[3] = 0.0;

    _gains[0] = a0 / a0;
    _gains[1] = a1 / a0;
    _gains[2] = a2 / a0;
    _gains[3] = a3 / a0;

    _gains[4] = b0 / a0;
    _gains[5] = b1 / a0;
    _gains[6] = b2 / a0;
    _gains[7] = b3 / a0;

    // Serial.print("Inputs = ");
    // Serial.print(wc, 8);
    // Serial.print(",");
    // Serial.print(wi, 8);
    // Serial.print(",");
    // Serial.print(wlp, 8);
    // Serial.print(",");
    // Serial.print(gain, 8);
    // Serial.print(",");
    // Serial.print(phase, 8);
    // Serial.println("");

    // Serial.print("Gains = ");
    // Serial.print(_gains[0], 8);
    // Serial.print(",");
    // Serial.print(_gains[1], 8);
    // Serial.print(",");
    // Serial.print(_gains[2], 8);
    // Serial.print(",");
    // Serial.print(_gains[3], 8);
    // Serial.print(",");
    // Serial.print(_gains[4], 8);
    // Serial.print(",");
    // Serial.print(_gains[5], 8);
    // Serial.print(",");
    // Serial.print(_gains[6], 8);
    // Serial.print(",");
    // Serial.print(_gains[7], 8);
    // Serial.println("");
    // Serial.print("Calc = ");
    // Serial.print(K, 8);
    // Serial.print(",");
    // Serial.print(a0, 8);
    // Serial.print(",");
    // Serial.print(a1, 8);
    // Serial.print(",");
    // Serial.print(a2, 8);
    // Serial.print(",");
    // Serial.print(a3, 8);
    // Serial.print(",");
    // Serial.print(b0, 8);
    // Serial.print(",");
    // Serial.print(b1, 8);
    // Serial.print(",");
    // Serial.print(b2, 8);
    // Serial.print(",");
    // Serial.print(b3, 8);
    // Serial.println("");
}

float ControlAlgo::GetTarget()
{
    return _target;
}

void ControlAlgo::SetGain(float gain, int i, int relative)
{
    if (relative == 1)
        _gains[i] *= gain;
    else
        _gains[i] = gain;
}

float ControlAlgo::GetGain(int i)
{
    return _gains[i];
}

void ControlAlgo::Disable()
{
    _out = 0;
    _diff_error = 0;
    _int_error = 0;
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
    // target control loop
    // - sets target to achieve low motor speed
    // _target += 0.000025 * _out;
    // _target += 0.0005 * _error;

    // target dither
    float dither = 0; // 0.05 * sin(2 * PI * millis() / 2000);

    // angle control loop
    // - sets motor speed to achieve angle
    _error = (angle - _target - dither);
    _inputs[3] = _inputs[2];
    _inputs[2] = _inputs[1];
    _inputs[1] = _inputs[0];
    _inputs[0] = _error;

    _outputs[3] = _outputs[2];
    _outputs[2] = _outputs[1];
    _outputs[1] = _outputs[0];
    _outputs[0] = (-_gains[1] * _outputs[1] - _gains[2] * _outputs[2] - _gains[3] * _outputs[3] + _gains[4] * _inputs[0] + _gains[5] * _inputs[1] + _gains[6] * _inputs[2] + _gains[7] * _inputs[3]) / _gains[0];
    // _diff_error = 0.1 * (_inputs[0] - _inputs[1]) + 0.9 * _diff_error;
    // _int_error = constrain(_int_error + _gains[2] * _error, -255, 255);
    // _outputs[0] = _gains[0] * _inputs[0] + _gains[1] * _diff_error + _int_error;

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
    _out_full = _outputs[0];
    // if (abs(_error) > 0.5)
    //     _out = constrain(_out + _outputs[0], _MIN_OUT, _MAX_OUT); // cumulative output, converting accel/force to speed
    // _out = constrain(_outputs[0], _MIN_OUT, _MAX_OUT); // cumulative output, converting accel/force to speed
    // if (abs(_error) < 1)                                      // TODO: configurable threshold
    //     _out = 0.95 * _out;                                   // leaky always tries to zero speed
    _out = constrain(_out + _outputs[0], -100, 100); // cumulative output, converting accel/force to speed

    UpdateOutput(_out);
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