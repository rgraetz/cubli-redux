#ifndef ANGLE_TRANSFORM_H
#define ANGLE_TRANSFORM_H

#include <Arduino.h>

class Euler
{
public:
    Euler(float roll, float pitch, float yaw);
    // Quaternion ToQuaternion(Euler e);
    float _roll, _pitch, _yaw;

private:
};

class Quaternion
{
public:
    Quaternion(float u[3], float angle);
    Euler ToEuler(int diag);
    float _q_w, _q_x, _q_y, _q_z;

private:
};

class AngleTransformation
{
public:
    AngleTransformation(float u1[3], float u2[3], float u3[3]);
    void Transform(float roll, float pitch, float yaw);
    float _roll_t, _pitch_t, _yaw_t;

private:
    float _u1[3], _u2[3], _u3[3];
    // Euler _input, _output;
    // Quaternion q1, q2, q3;
    // Euler e1, e2, e3;
};

#endif