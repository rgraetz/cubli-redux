#include "DigitalEncoder.h"

DigitalEncoder::DigitalEncoder(int CHA_GPIO, int CHB_GPIO, int motorNum)
{
    pinMode(CHA_GPIO, INPUT);
    pinMode(CHB_GPIO, INPUT);
    _CHA_GPIO = CHA_GPIO;
    _CHB_GPIO = CHB_GPIO;
    _current_state = GetState();
    _previous_state = _current_state;
    _position = 0;
    _motorNum = motorNum;
}

void DigitalEncoder::UpdatePosition()
{
    _current_state = GetState();
    switch (_previous_state)
    {
    case 0: // CHA = 0, CHB = 0
        if (_current_state == 2)
            _position--;
        else if (_current_state == 1)
            _position++;
        else if (_current_state != 0)
        {
            Serial.print("ERROR invalid state transition on encoder ");
            Serial.print(_motorNum);
            Serial.print(", ");
            Serial.print(_current_state);
            Serial.print(", ");
            Serial.println(_previous_state);
        }
        break;
    case 1: // CHA = 1, CHB = 0
        if (_current_state == 0)
            _position--;
        else if (_current_state == 3)
            _position++;
        else if (_current_state != 1)
        {
            Serial.print("ERROR invalid state transition on encoder ");
            Serial.print(_motorNum);
            Serial.print(", ");
            Serial.print(_current_state);
            Serial.print(", ");
            Serial.println(_previous_state);
        }
        break;
    case 3: // CHA = 1, CHB = 1
        if (_current_state == 1)
            _position--;
        else if (_current_state == 2)
            _position++;
        else if (_current_state != 3)
        {
            Serial.print("ERROR invalid state transition on encoder ");
            Serial.print(_motorNum);
            Serial.print(", ");
            Serial.print(_current_state);
            Serial.print(", ");
            Serial.println(_previous_state);
        }
        break;
    case 2: // CHA = 0, CHB = 1
        if (_current_state == 3)
            _position--;
        else if (_current_state == 0)
            _position++;
        else if (_current_state != 2)
        {
            Serial.print("ERROR invalid state transition on encoder ");
            Serial.print(_motorNum);
            Serial.print(", ");
            Serial.print(_current_state);
            Serial.print(", ");
            Serial.println(_previous_state);
        }
        break;
    default:
        Serial.print("ERROR invalid previous state ");
        Serial.println(_previous_state);
        break;
    }
    _previous_state = _current_state;
}

int DigitalEncoder::GetState()
{
    return (digitalRead(_CHA_GPIO) << 1) + digitalRead(_CHB_GPIO);
}