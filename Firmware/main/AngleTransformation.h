#ifndef ANGLE_TRANSFORM_H
#define ANGLE_TRANSFORM_H

#include <Arduino.h>
#include "common.h"
#include "DiscreteFilter.h"

#define ACC_GYRO_FILTER 0.99
#define ACC_FILTER 5.0
#define GYRO_FILTER 0.5

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
};

class RobotCS
{
public:
    RobotCS(int type);
    void MPU2Robot(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);
    float roll, pitch, yaw;
    float accAngleX, accAngleY, accAngleZ;
    float GetAccX();
    float GetAccY();
    float GetAccZ();
    float GetGyroX();
    float GetGyroY();
    float GetGyroZ();

private:
    int _type;
    int _initialized;
    int _val;

    DiscreteFilter accXf, accYf, accZf, gyroXf, gyroYf, gyroZf;
};

#endif