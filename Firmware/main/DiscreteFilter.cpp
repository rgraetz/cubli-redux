#include "DiscreteFilter.h"

DiscreteFilter::DiscreteFilter(void)
{
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
}

void DiscreteFilter::SetButterFilter(int order, float Ts, float cuttOff)
{
    if (order == 2)
    {
        float wc = cuttOff * 2 * PI;
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

void DiscreteFilter::SetGain(int index, float value) { this->_gains[index] = value; };