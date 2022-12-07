#include "ControlAlgo.h"

ControlAlgo::ControlAlgo(int DIR_GPIO, int invertDir, int PWM_GPIO, int motorNum)
{
    _pwmChannel = motorNum - 1;
    _motorNum = motorNum;
    _invertDir = invertDir;
    _invertOut = 1;

    _DIR_GPIO = DIR_GPIO;
    _PWM_GPIO = PWM_GPIO;

    _PWM_MAX = (1 << TIMER_BIT) - 1;

    _MAX_OUT = 255;
    _MIN_OUT = -_MAX_OUT;

    _WINDUP = 0.05;

    pinMode(DIR_GPIO, OUTPUT);
    ledcSetup(_pwmChannel, PWM_BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM_GPIO, _pwmChannel);

    // default values
    _target = 0;
    _accel_out = 0;
    _velocity_out = 0;
    _error1 = 0;
    _error2 = 0;
    _ramp_time = 0;
    for (int i = 0; i < 8; i++)
    {
        _gains1[i] = 0;
        _gains2[i] = 0;
    }
}

void ControlAlgo::SetTarget(float target)
{
    _target = target;
}

void ControlAlgo::SetGains(float gains[8], int type)
{
    for (int i = 0; i < 8; i++)
        if (type == CONTROL_GYRO)
            _gains1[i] = gains[i];
        else if (type == CONTROL_ACCEL)
            _gains2[i] = gains[i];
}

void ControlAlgo::CalcGains(float wc, float wi, float wlp, float gain, float phase, int type, int print)
{
    float integralTol = 0.000001;

    double Ts = CONTROL_PERIOD_USEC / 1E6;
    double Ts2 = Ts * Ts;
    double Ts3 = Ts * Ts;
    double a = phase;

    double a0, a1, a2, a3;
    double b0, b1, b2, b3;
    double K;

    if (type == CONTROL_GYRO)
    {
        // double whp = wlp;
        // Serial.println("Gyroscope Controller -> Lead Lag + High pass");
        // K = gain / a;
        // b0 = 2 * K * Ts * a * wc + 4 * K * a * a;
        // b1 = -8 * K * a * a;
        // b2 = -2 * K * Ts * a * wc + 4 * K * a * a;
        // b3 = 0;

        // a0 = Ts * Ts * a * wc * whp + 2 * Ts * a * wc + 2 * Ts * whp + 4;
        // a1 = 2 * Ts * Ts * a * wc * whp - 8;
        // a2 = Ts * Ts * a * wc * whp - 2 * Ts * a * wc - 2 * Ts * whp + 4;
        // a3 = 0;
        if (phase > 0)
        {
            Serial.println("Gyroscope Controller -> Lead Lag + Low Pass");
            K = gain / a;
            b0 = K * Ts * Ts * a * wc * wlp + 2 * K * Ts * a * a * wlp;
            b1 = 2 * K * Ts * Ts * a * wc * wlp;
            b2 = K * Ts * Ts * a * wc * wlp - 2 * K * Ts * a * a * wlp;
            b3 = 0.0;

            a0 = Ts * Ts * a * wc * wlp + 2 * Ts * a * wc + 2 * Ts * wlp + 4;
            a1 = 2 * Ts * Ts * a * wc * wlp - 8;
            a2 = Ts * Ts * a * wc * wlp - 2 * Ts * a * wc - 2 * Ts * wlp + 4;
            a3 = 0.0;
        }
        else
        {
            Serial.println("Gyroscope Controller -> Lead Lag");
            K = gain;
            a0 = Ts * wlp + 2;
            a1 = Ts * wlp - 2;
            a2 = 0.0;
            a3 = 0.0;

            b0 = K * Ts * wlp;
            b1 = K * Ts * wlp;
            b2 = 0.0;
            b3 = 0.0;
        }

        Disable();
        _gains1[0] = a0 / a0;
        _gains1[1] = a1 / a0;
        _gains1[2] = a2 / a0;
        _gains1[3] = a3 / a0;

        _gains1[4] = b0 / a0;
        _gains1[5] = b1 / a0;
        _gains1[6] = b2 / a0;
        _gains1[7] = b3 / a0;
    }
    else if (type == CONTROL_ACCEL)
    {
        // Serial.println("Accelerometer Controller -> Integral + Low Pass");
        // K = gain * wi;
        // b0 = (K * Ts2 * wi * wlp + 2 * K * Ts * wlp);
        // b1 = 2 * K * Ts2 * wi * wlp;
        // b2 = K * Ts2 * wi * wlp - 2 * K * Ts * wlp;
        // b3 = 0;

        // a0 = (2 * Ts * wi * wlp + 4 * wi);
        // a1 = -8 * wi;
        // a2 = -2 * Ts * wi * wlp + 4 * wi;
        // a3 = 0;

        if (phase > 0)
        {
            Serial.println("Accelerometer Controller -> Lead Lag + Low Pass");
            K = gain / a;
            b0 = K * Ts * Ts * a * wc * wlp + 2 * K * Ts * a * a * wlp;
            b1 = 2 * K * Ts * Ts * a * wc * wlp;
            b2 = K * Ts * Ts * a * wc * wlp - 2 * K * Ts * a * a * wlp;
            b3 = 0.0;

            a0 = Ts * Ts * a * wc * wlp + 2 * Ts * a * wc + 2 * Ts * wlp + 4;
            a1 = 2 * Ts * Ts * a * wc * wlp - 8;
            a2 = Ts * Ts * a * wc * wlp - 2 * Ts * a * wc - 2 * Ts * wlp + 4;
            a3 = 0.0;
        }
        else
        {
            Serial.println("Accelerometer Controller -> Lead Lag");
            K = gain;
            a0 = Ts * wlp + 2;
            a1 = Ts * wlp - 2;
            a2 = 0.0;
            a3 = 0.0;

            b0 = K * Ts * wlp;
            b1 = K * Ts * wlp;
            b2 = 0.0;
            b3 = 0.0;
        }

        Disable();
        _gains2[0] = a0 / a0;
        _gains2[1] = a1 / a0;
        _gains2[2] = a2 / a0;
        _gains2[3] = a3 / a0;

        _gains2[4] = b0 / a0;
        _gains2[5] = b1 / a0;
        _gains2[6] = b2 / a0;
        _gains2[7] = b3 / a0;
    }
    else
        throw std::invalid_argument("Invalid control type");
}

float ControlAlgo::GetTarget()
{
    return _target;
}

void ControlAlgo::SetGain(float gain, int i, int relative, int type)
{
    if (relative == 1)
    {
        if (type == CONTROL_GYRO)
            _gains1[i] *= gain;
        else if (type == CONTROL_ACCEL)
            _gains2[i] *= gain;
        else
            throw std::invalid_argument("Invalid control type");
    }
    else
    {
        if (type == CONTROL_GYRO)
            _gains1[i] = gain;
        else if (type == CONTROL_ACCEL)
            _gains2[i] = gain;
        else
            throw std::invalid_argument("Invalid control type");
    }
}

float ControlAlgo::GetGain(int i, int type)
{
    if (type == CONTROL_GYRO)
        return _gains1[i];
    else if (type == CONTROL_ACCEL)
        return _gains2[i];
    else
        throw std::invalid_argument("Invalid control type");
}

void ControlAlgo::Disable()
{
    _accel_out = 0;
    _velocity_out = 0;
    _error1 = 0;
    _error2 = 0;
    _ramp_time = 0;
    UpdateOutput(_velocity_out);

    for (int i = 0; i < 4; i++)
    {
        _inputs1[i] = 0.0;
        _inputs2[i] = 0.0;
        _outputs1[i] = 0.0;
        _outputs2[i] = 0.0;
    }
}

int ControlAlgo::ControlUpdate(float gyro_angle, float acc_angle)
{
    // target control loop
    // - sets target to achieve low motor speed
    // _target -= 0.000001 * _out;
    // _target += 0.00001 * _out;

    // target dither
    float dither = 0.0; // 0.25 * sin(2 * PI * millis() / 5000);

    // control error
    _error1 = (gyro_angle);
    _error2 = (acc_angle - _target - dither);

    // GYROSCOPE CONTROL LOOP
    _inputs1[3] = _inputs1[2];
    _inputs1[2] = _inputs1[1];
    _inputs1[1] = _inputs1[0];
    _inputs1[0] = _error1;

    _outputs1[3] = _outputs1[2];
    _outputs1[2] = _outputs1[1];
    _outputs1[1] = _outputs1[0];
    _outputs1[0] = (-_gains1[1] * _outputs1[1] - _gains1[2] * _outputs1[2] - _gains1[3] * _outputs1[3] + _gains1[4] * _inputs1[0] + _gains1[5] * _inputs1[1] + _gains1[6] * _inputs1[2] + _gains1[7] * _inputs1[3]) / _gains1[0];

    // - wind-up protection
    float scale = 1;
    if (_outputs1[0] < _MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs1[0];
    else if (_outputs1[0] > _MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs1[0];

    if (scale != 1)
    {
        _inputs1[0] *= scale;
        _inputs1[1] *= scale;
        _inputs1[2] *= scale;
        _inputs1[3] *= scale;

        _outputs1[0] *= scale;
        _outputs1[1] *= scale;
        _outputs1[2] *= scale;
        _outputs1[3] *= scale;
    }

    // ACCELEROMETER CONTROL LOOP
    _inputs2[3] = _inputs2[2];
    _inputs2[2] = _inputs2[1];
    _inputs2[1] = _inputs2[0];
    _inputs2[0] = _error2;

    _outputs2[3] = _outputs2[2];
    _outputs2[2] = _outputs2[1];
    _outputs2[1] = _outputs2[0];
    _outputs2[0] = (-_gains2[1] * _outputs2[1] - _gains2[2] * _outputs2[2] - _gains2[3] * _outputs2[3] + _gains2[4] * _inputs2[0] + _gains2[5] * _inputs2[1] + _gains2[6] * _inputs2[2] + _gains2[7] * _inputs2[3]) / _gains2[0];

    // - wind-up protection
    scale = 1;
    if (_outputs2[0] < _MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs2[0];
    else if (_outputs2[0] > _MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs2[0];

    if (scale != 1)
    {
        _inputs2[0] *= scale;
        _inputs2[1] *= scale;
        _inputs2[2] *= scale;
        _inputs2[3] *= scale;

        _outputs2[0] *= scale;
        _outputs2[1] *= scale;
        _outputs2[2] *= scale;
        _outputs2[3] *= scale;
    }

    // GYROSCOPE+ACCELEROMETER CONTROL OUTPUT
    // acceleration output
    _accel_out = _outputs1[0] + _outputs2[0];

    // velocity output with enable ramp
    if (_ramp_time < EN_RAMP)
        _ramp_time++;
    _velocity_out = (_ramp_time / EN_RAMP) * constrain(_velocity_out + _accel_out, -100, 100); // cumulative output, converting accel/force to speed

    UpdateOutput(_velocity_out);

    // TODO: add max velocity protection
    return 0;
}

void ControlAlgo::UpdateOutput(float output)
{
    // output = constrain(output, _MIN_OUT, _MAX_OUT);
    // set direction GPIO
    if ((output > 0) ^ (_invertDir == 1))
        digitalWrite(_DIR_GPIO, HIGH);
    else
        digitalWrite(_DIR_GPIO, LOW);

    // set PWM duty cycle
    if (output > _MAX_OUT || output < _MIN_OUT)
        _pwm = (_invertOut == 0) ? _MAX_OUT : _PWM_MAX - _MAX_OUT;
    else
        _pwm = (_invertOut == 0) ? abs(output) : (uint8_t)(_PWM_MAX - abs(output));
    ledcWrite(_pwmChannel, _pwm);
}

void ControlAlgo::SetOutput(float output) { this->UpdateOutput(output); };