#ifndef DIGENC_H
#define DIGENC_H

#include <Arduino.h>

class DigitalEncoder
{
public:
    DigitalEncoder(int CHA_GPIO, int CHB_GPIO, int motorNum);
    void UpdatePosition();
    int _position;
    int _previous_state, _current_state;

private:
    int GetState();
    int _CHA_GPIO;
    int _CHB_GPIO;
    int _motorNum;
};

#endif