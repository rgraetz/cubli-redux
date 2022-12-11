#include "DiscreteFilter.h"

DiscreteFilter::DiscreteFilter(void)
{
    _en_windup = 0;
    Reset();
}

void DiscreteFilter::Reset()
{
    for (int i; i < _size; i++)
    {
        _inputs[i] = 0.0;
        _outputs[i] = 0.0;
    }
}

void DiscreteFilter::Update(float input)
{
    // shift previous input values
    _inputs[3] = _inputs[2];
    _inputs[2] = _inputs[1];
    _inputs[1] = _inputs[0];
    _inputs[0] = input;

    // shift previous output values
    _outputs[3] = _outputs[2];
    _outputs[2] = _outputs[1];
    _outputs[1] = _outputs[0];

    // calculate current output value
    _outputs[0] = (-_gains[1] * _outputs[1] - _gains[2] * _outputs[2] - _gains[3] * _outputs[3] + _gains[4] * _inputs[0] + _gains[5] * _inputs[1] + _gains[6] * _inputs[2] + _gains[7] * _inputs[3]) / _gains[0];

    if (_en_windup)
    {
        // - wind-up protection
        float scale = 1;
        if (_outputs[0] < _min_val - _windup * (_max_val - _min_val))
            scale = (_min_val - _windup * (_max_val - _min_val)) / _outputs[0];
        else if (_outputs[0] > _max_val + _windup * (_max_val - _min_val))
            scale = (_max_val + _windup * (_max_val - _min_val)) / _outputs[0];

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
    }
}

void DiscreteFilter::SetUnity()
{
    _gains[0] = 1.0;
    _gains[1] = 0.0;
    _gains[2] = 0.0;
    _gains[3] = 0.0;

    _gains[4] = 1.0;
    _gains[5] = 0.0;
    _gains[6] = 0.0;
    _gains[7] = 0.0;
}

void DiscreteFilter::SetHighPass(int order, float Ts, float cutOff)
{
    if (order == 2)
    {
        float wc = cutOff * 2 * PI;

        _gains[0] = (Ts * Ts * wc * wc + 2.82842712474619 * Ts * wc + 4);
        _gains[1] = (2 * Ts * Ts * wc * wc - 8);
        _gains[2] = Ts * Ts * wc * wc - 2.82842712474619 * Ts * wc + 4;
        _gains[3] = 0.0;

        _gains[4] = 4.0;
        _gains[5] = -8.0;
        _gains[6] = +4.0;
        _gains[7] = 0.0;
    }
    else
        throw std::invalid_argument("ButterWorth filter of this order is not implemented");
}

void DiscreteFilter::SetButterFilter(int order, float Ts, float cutOff)
{
    if (order == 2)
    {
        float wc = cutOff * 2 * PI;
        _gains[0] = (Ts * Ts * wc * wc + 2.82842712474619 * Ts * wc + 4);
        _gains[1] = (2 * Ts * Ts * wc * wc - 8);
        _gains[2] = Ts * Ts * wc * wc - 2.82842712474619 * Ts * wc + 4;
        _gains[3] = 0.0;

        _gains[4] = 1.0 * Ts * Ts * wc * wc;
        _gains[5] = 2.0 * Ts * Ts * wc * wc;
        _gains[6] = 1.0 * Ts * Ts * wc * wc;
        _gains[7] = 0.0;
    }
    else
        throw std::invalid_argument("ButterWorth filter of this order is not implemented");
}

float DiscreteFilter::GetLatest() { return this->_outputs[0]; };
float DiscreteFilter::GetGain(int index) { return this->_gains[index]; };
void DiscreteFilter::SetGain(int index, float value) { this->_gains[index] = value; };
void DiscreteFilter::DisableWindup(void) { this->_en_windup = 0; };
void DiscreteFilter::SetWindup(float min_val, float max_val, float windup)
{
    this->_en_windup = 1;
    this->_max_val = max_val;
    this->_min_val = min_val;
    this->_windup = windup;
};