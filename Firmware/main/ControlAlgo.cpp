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
    _pos_target = 0;
    _accel_out = 0;
    _velocity_out = 0;
    _pos_error = 0;
    _vel_error = 0;
    _ramp_time = 0;
    for (int i = 0; i < 8; i++)
    {
        _gains_pos[i] = 0;
        _gains_vel[i] = 0;
    }
}

void ControlAlgo::SetTarget(float target)
{
    _pos_target = target;
}

void ControlAlgo::SetGains(float gains[8], int type)
{
    for (int i = 0; i < 8; i++)
        if (type == CONTROL_VEL)
            _gains_vel[i] = gains[i];
        else if (type == CONTROL_POS)
            _gains_pos[i] = gains[i];
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

    if (type == CONTROL_VEL)
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
            Serial.println("Velocity Controller -> Lead Lag + Low Pass");
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
            Serial.println("Velocity Controller -> Lead Lag");
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
        _gains_vel[0] = a0 / a0;
        _gains_vel[1] = a1 / a0;
        _gains_vel[2] = a2 / a0;
        _gains_vel[3] = a3 / a0;

        _gains_vel[4] = b0 / a0;
        _gains_vel[5] = b1 / a0;
        _gains_vel[6] = b2 / a0;
        _gains_vel[7] = b3 / a0;
    }
    else if (type == CONTROL_POS)
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

        // if (phase > 0)
        // {
        //     Serial.println("Position Controller -> Lead Lag + Low Pass");
        //     K = gain / a;
        //     b0 = K * Ts * Ts * a * wc * wlp + 2 * K * Ts * a * a * wlp;
        //     b1 = 2 * K * Ts * Ts * a * wc * wlp;
        //     b2 = K * Ts * Ts * a * wc * wlp - 2 * K * Ts * a * a * wlp;
        //     b3 = 0.0;

        //     a0 = Ts * Ts * a * wc * wlp + 2 * Ts * a * wc + 2 * Ts * wlp + 4;
        //     a1 = 2 * Ts * Ts * a * wc * wlp - 8;
        //     a2 = Ts * Ts * a * wc * wlp - 2 * Ts * a * wc - 2 * Ts * wlp + 4;
        //     a3 = 0.0;
        // }
        // else
        // {
        //     Serial.println("Position Controller -> Lead Lag");
        //     K = gain;
        //     a0 = Ts * wlp + 2;
        //     a1 = Ts * wlp - 2;
        //     a2 = 0.0;
        //     a3 = 0.0;

        //     b0 = K * Ts * wlp;
        //     b1 = K * Ts * wlp;
        //     b2 = 0.0;
        //     b3 = 0.0;
        // }

        if (phase > 0)
        {
            Serial.println("Position Controller -> Integral + Lead Lag + Low Pass");
            K = gain * wi / a;
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
            Serial.println("Position Controller -> Integral + Low Pass");
            K = gain * wi;
            b0 = (K * Ts2 * wi * wlp + 2 * K * Ts * wlp);
            b1 = 2 * K * Ts2 * wi * wlp;
            b2 = K * Ts2 * wi * wlp - 2 * K * Ts * wlp;
            b3 = 0;

            a0 = (2 * Ts * wi * wlp + 4 * wi);
            a1 = -8 * wi;
            a2 = -2 * Ts * wi * wlp + 4 * wi;
            a3 = 0;
        }

        Disable();
        _gains_pos[0] = a0 / a0;
        _gains_pos[1] = a1 / a0;
        _gains_pos[2] = a2 / a0;
        _gains_pos[3] = a3 / a0;

        _gains_pos[4] = b0 / a0;
        _gains_pos[5] = b1 / a0;
        _gains_pos[6] = b2 / a0;
        _gains_pos[7] = b3 / a0;
    }
    else
        throw std::invalid_argument("Invalid control type");
}

float ControlAlgo::GetTarget(int type)
{
    if (type == CONTROL_POS)
        return _pos_target;
    else if (type == CONTROL_VEL)
        return _vel_target;
    else
        throw std::invalid_argument("Invalid control type");
}

void ControlAlgo::SetGain(float gain, int i, int relative, int type)
{
    if (relative == 1)
    {
        if (type == CONTROL_VEL)
            _gains_vel[i] *= gain;
        else if (type == CONTROL_POS)
            _gains_pos[i] *= gain;
        else
            throw std::invalid_argument("Invalid control type");
    }
    else
    {
        if (type == CONTROL_VEL)
            _gains_vel[i] = gain;
        else if (type == CONTROL_POS)
            _gains_pos[i] = gain;
        else
            throw std::invalid_argument("Invalid control type");
    }
}

float ControlAlgo::GetGain(int i, int type)
{
    if (type == CONTROL_VEL)
        return _gains_vel[i];
    else if (type == CONTROL_POS)
        return _gains_pos[i];
    else
        throw std::invalid_argument("Invalid control type");
}

void ControlAlgo::Disable()
{
    _accel_out = 0;
    _velocity_out = 0;
    _vel_error = 0;
    _pos_error = 0;
    _ramp_time = 0;
    UpdateOutput(_velocity_out);

    for (int i = 0; i < 4; i++)
    {
        _inputs_pos[i] = 0.0;
        _inputs_vel[i] = 0.0;
        _outputs_pos[i] = 0.0;
        _outputs_vel[i] = 0.0;
    }
}

int ControlAlgo::ControlUpdate(float position, float velocity)
{
    // BALANCING TARGET CONTROL LOOP
    // - sets position target to achieve low motor speed and balance
    _pos_target -= 0.000005 * _velocity_out;

    // target dither
    float dither = 0.0; // 0.25 * sin(2 * PI * millis() / 5000);

    // OUTER POSITION CONTROL LOOP
    // - sets velocity to achieve position
    _pos_error = (position - _pos_target - dither);

    _inputs_pos[3] = _inputs_pos[2];
    _inputs_pos[2] = _inputs_pos[1];
    _inputs_pos[1] = _inputs_pos[0];
    _inputs_pos[0] = _pos_error;

    _outputs_pos[3] = _outputs_pos[2];
    _outputs_pos[2] = _outputs_pos[1];
    _outputs_pos[1] = _outputs_pos[0];
    _outputs_pos[0] = (-_gains_pos[1] * _outputs_pos[1] - _gains_pos[2] * _outputs_pos[2] - _gains_pos[3] * _outputs_pos[3] + _gains_pos[4] * _inputs_pos[0] + _gains_pos[5] * _inputs_pos[1] + _gains_pos[6] * _inputs_pos[2] + _gains_pos[7] * _inputs_pos[3]) / _gains_pos[0];

    // TODO: convert windup to position
    // - wind-up protection
    float scale = 1;
    if (_outputs_pos[0] < _MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs_pos[0];
    else if (_outputs_pos[0] > _MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs_pos[0];

    if (scale != 1)
    {
        _inputs_pos[0] *= scale;
        _inputs_pos[1] *= scale;
        _inputs_pos[2] *= scale;
        _inputs_pos[3] *= scale;

        _outputs_pos[0] *= scale;
        _outputs_pos[1] *= scale;
        _outputs_pos[2] *= scale;
        _outputs_pos[3] *= scale;
    }

    _vel_target = -_outputs_pos[0];

    // INNER VELOCITY CONTROL LOOP
    _vel_error = (velocity - _vel_target - dither);

    _inputs_vel[3] = _inputs_vel[2];
    _inputs_vel[2] = _inputs_vel[1];
    _inputs_vel[1] = _inputs_vel[0];
    _inputs_vel[0] = _vel_error;

    _outputs_vel[3] = _outputs_vel[2];
    _outputs_vel[2] = _outputs_vel[1];
    _outputs_vel[1] = _outputs_vel[0];
    _outputs_vel[0] = (-_gains_vel[1] * _outputs_vel[1] - _gains_vel[2] * _outputs_vel[2] - _gains_vel[3] * _outputs_vel[3] + _gains_vel[4] * _inputs_vel[0] + _gains_vel[5] * _inputs_vel[1] + _gains_vel[6] * _inputs_vel[2] + _gains_vel[7] * _inputs_vel[3]) / _gains_vel[0];

    // - wind-up protection
    scale = 1;
    if (_outputs_vel[0] < _MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MIN_OUT - _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs_vel[0];
    else if (_outputs_vel[0] > _MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT))
        scale = (_MAX_OUT + _WINDUP * (_MAX_OUT - _MIN_OUT)) / _outputs_vel[0];

    if (scale != 1)
    {
        _inputs_vel[0] *= scale;
        _inputs_vel[1] *= scale;
        _inputs_vel[2] *= scale;
        _inputs_vel[3] *= scale;

        _outputs_vel[0] *= scale;
        _outputs_vel[1] *= scale;
        _outputs_vel[2] *= scale;
        _outputs_vel[3] *= scale;
    }

    // GYROSCOPE+ACCELEROMETER CONTROL OUTPUT
    // acceleration output
    _accel_out = _outputs_vel[0];

    // velocity output with enable ramp
    if (_ramp_time < EN_RAMP)
        _ramp_time++;
    _velocity_out = (_ramp_time / EN_RAMP) * constrain(_velocity_out + _accel_out, -200, 200); // cumulative output, converting accel/force to speed

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