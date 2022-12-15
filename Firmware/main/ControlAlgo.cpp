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
    _MAX_SPEED = 100;
    _MIN_SPEED = -_MAX_SPEED;

    _WINDUP = 0.05;

    pinMode(DIR_GPIO, OUTPUT);
    ledcSetup(_pwmChannel, PWM_BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM_GPIO, _pwmChannel);

    // control loop settings
    _bal_loop.SetWindup(-1.5, 1.5, _WINDUP);
    _pos_loop.SetWindup(-0.01, 0.01, _WINDUP);
    _vel_loop.SetWindup(_MIN_OUT, _MAX_OUT, _WINDUP); // TODO: set to max speed instead?

    // default values
    _pos_target = 0;
    _accel_out = 0;
    _velocity_out = 0;
    _pos_error = 0;
    _vel_error = 0;
    _bal_error = 0;
    _ramp_time = 0;
}

void ControlAlgo::SetTarget(float target)
{
    _pos_target = target;
}

void ControlAlgo::SetGains(float gains[8], int type)
{
    for (int i = 0; i < 8; i++)
        if (type == CONTROL_VEL)
            _vel_loop.SetGain(i, gains[i]);
        else if (type == CONTROL_POS)
            _pos_loop.SetGain(i, gains[i]);
        else if (type == CONTROL_BAL)
            _bal_loop.SetGain(i, gains[i]);
        else
            throw std::invalid_argument("Invalid control type");
}

void ControlAlgo::CalcGains(float fc, float fi, float flp, float gain, float phase, int type, int print)
{
    float wc = fc * 2 * PI;
    float wi = fi * 2 * PI;
    float wlp = flp * 2 * PI;

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
        if (phase > 0)
        {
            if (print == 1)
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
            if (print == 1)
                Serial.println("Velocity Controller -> Low Pass");
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
        _vel_loop.SetGain(0, a0 / a0);
        _vel_loop.SetGain(1, a1 / a0);
        _vel_loop.SetGain(2, a2 / a0);
        _vel_loop.SetGain(3, a3 / a0);
        _vel_loop.SetGain(4, b0 / a0);
        _vel_loop.SetGain(5, b1 / a0);
        _vel_loop.SetGain(6, b2 / a0);
        _vel_loop.SetGain(7, b3 / a0);
    }
    else if (type == CONTROL_POS)
    {
        if (phase > 0)
        {
            if (print == 1)
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
            if (print == 1)
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
        _pos_loop.SetGain(0, a0 / a0);
        _pos_loop.SetGain(1, a1 / a0);
        _pos_loop.SetGain(2, a2 / a0);
        _pos_loop.SetGain(3, a3 / a0);
        _pos_loop.SetGain(4, b0 / a0);
        _pos_loop.SetGain(5, b1 / a0);
        _pos_loop.SetGain(6, b2 / a0);
        _pos_loop.SetGain(7, b3 / a0);
    }
    else if (type == CONTROL_BAL)
    {
        if (print == 1)
            Serial.println("Balance Controller -> Integral + Low Pass");
        K = gain * wi;
        b0 = (K * Ts2 * wi * wlp + 2 * K * Ts * wlp);
        b1 = 2 * K * Ts2 * wi * wlp;
        b2 = K * Ts2 * wi * wlp - 2 * K * Ts * wlp;
        b3 = 0;

        a0 = (2 * Ts * wi * wlp + 4 * wi);
        a1 = -8 * wi;
        a2 = -2 * Ts * wi * wlp + 4 * wi;
        a3 = 0;

        Disable();
        _bal_loop.SetGain(0, a0 / a0);
        _bal_loop.SetGain(1, a1 / a0);
        _bal_loop.SetGain(2, a2 / a0);
        _bal_loop.SetGain(3, a3 / a0);
        _bal_loop.SetGain(4, b0 / a0);
        _bal_loop.SetGain(5, b1 / a0);
        _bal_loop.SetGain(6, b2 / a0);
        _bal_loop.SetGain(7, b3 / a0);
    }
    else
        throw std::invalid_argument("Invalid control type");

    if (print == 1)
    {
        Serial.print("Inputs = ");
        Serial.print(wc, 8);
        Serial.print(",");
        Serial.print(wi, 8);
        Serial.print(",");
        Serial.print(wlp, 8);
        Serial.print(",");
        Serial.print(gain, 8);
        Serial.print(",");
        Serial.print(phase, 8);
        Serial.println("");

        Serial.print("Gains = ");
        for (int i = 0; i < 8; i++)
        {
            if (type == CONTROL_POS)
                Serial.print(_pos_loop.GetGain(i), 8);
            else if (type == CONTROL_VEL)
                Serial.print(_vel_loop.GetGain(i), 8);
            else if (type == CONTROL_BAL)
                Serial.print(_bal_loop.GetGain(i), 8);
            Serial.print(",");
        }

        Serial.println("");
        Serial.print("Calc = ");
        Serial.print(K, 8);
        Serial.print(",");
        Serial.print(a0, 8);
        Serial.print(",");
        Serial.print(a1, 8);
        Serial.print(",");
        Serial.print(a2, 8);
        Serial.print(",");
        Serial.print(a3, 8);
        Serial.print(",");
        Serial.print(b0, 8);
        Serial.print(",");
        Serial.print(b1, 8);
        Serial.print(",");
        Serial.print(b2, 8);
        Serial.print(",");
        Serial.print(b3, 8);
        Serial.println("");
    }
}

float ControlAlgo::GetTarget(int type)
{
    if (type == CONTROL_POS)
        return _pos_target;
    else if (type == CONTROL_VEL)
        return _vel_target;
    else if (type == CONTROL_BAL)
        return _bal_target;
    else if (type == CONTROL_FULL)
        return _pos_target + _bal_target + _dither;
    else
        throw std::invalid_argument("Invalid control type");
}

float ControlAlgo::GetGain(int i, int type)
{
    if (type == CONTROL_VEL)
        return _vel_loop.GetGain(i);
    else if (type == CONTROL_POS)
        return _pos_loop.GetGain(i);
    else if (type == CONTROL_BAL)
        return _bal_loop.GetGain(i);
    else
        throw std::invalid_argument("Invalid control type");
}

void ControlAlgo::Disable()
{
    _accel_out = 0;
    _velocity_out = 0;
    _vel_error = 0;
    _bal_error = 0;
    _pos_error = 0;
    _ramp_time = 0;

    UpdateOutput(0);

    _bal_loop.Reset();
    _pos_loop.Reset();
    _vel_loop.Reset();
}

int ControlAlgo::ControlUpdate(float position, float velocity, int set_out)
{
    return ControlUpdate(position, velocity, set_out, 1, 1, 1);
}

int ControlAlgo::ControlUpdate(float position, float velocity, int set_out, int en_pos_loop, int en_bal_loop, int en_dither)
{
    // BALANCING TARGET CONTROL LOOP
    // - sets position target to achieve low motor speed and balance
    if (en_bal_loop == 1)
    {
        _bal_error = (_invertDir == 1) ? _velocity_out : -_velocity_out;
        _bal_loop.Update(_bal_error);
        _bal_target = _bal_loop.GetLatest();
    }
    else
        _bal_target = 0.0;

    // target dither
    if (en_dither)
        _dither = 0.25 * sin(2 * PI * millis() / 5000);
    else
        _dither = 0;

    // OUTER POSITION CONTROL LOOP
    // - sets velocity to achieve position
    if (en_pos_loop == 1)
    {
        _pos_error = (position - (_pos_target + _bal_target + _dither));
        _pos_loop.Update(_pos_error);

        _vel_target = (_invertDir == 1) ? _pos_loop.GetLatest() : -_pos_loop.GetLatest();
    }
    else
        _vel_target = 0.0;

    // INNER VELOCITY CONTROL LOOP
    _vel_error = (velocity - _vel_target);
    _vel_loop.Update(_vel_error);

    // acceleration output
    _accel_out = _vel_loop.GetLatest();

    // velocity output with enable ramp
    if (_ramp_time < EN_RAMP)
        _ramp_time++;
    _velocity_out = (_ramp_time / EN_RAMP) * (_velocity_out + _accel_out); // cumulative output, converting accel/force to speed
    _velocity_out = constrain(_velocity_out, _MIN_SPEED, _MAX_SPEED);      // prevent wind-up of velocity

    if (set_out == 1)
        UpdateOutput(_velocity_out);
    // else if (_pos_error < 1.0)
    //     _velocity_out = 0.975 * _velocity_out;

    // TODO: add max velocity protection
    return 0;
}

void ControlAlgo::UpdateOutput(float output)
{
    output = constrain(output, _MIN_OUT, _MAX_OUT);
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