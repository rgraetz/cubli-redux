#include "AngleTransformation.h"

Euler::Euler(float roll_deg, float pitch_deg, float yaw_deg)
{
    _roll = roll_deg;
    _pitch = pitch_deg;
    _yaw = yaw_deg;
}

Quaternion::Quaternion(float u[3], float angle_deg)
{
    float half_angle_rad = angle_deg / 2.0 * PI / 180.0;
    float sina = sin(half_angle_rad);
    _q_w = cos(half_angle_rad);
    _q_x = sina * u[0];
    _q_y = sina * u[1];
    _q_z = sina * u[2];
}

Euler Quaternion::ToEuler(int diag)
{
    float roll, pitch, yaw;
    float sinr_cosp = 2 * (_q_w * _q_x + _q_y * _q_z);
    float cosr_cosp = 1 - 2 * (_q_x * _q_x + _q_y * _q_y);
    roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (_q_w * _q_y - _q_z * _q_x);
    if (abs(sinp) >= 1)
        pitch = copysign(PI / 2.0, sinp);
    else
        pitch = asin(sinp);

    float siny_cosp = 2 * (_q_w * _q_z + _q_x * _q_y);
    float cosy_cosp = 1 - 2 * (_q_y * _q_y + _q_z * _q_z);
    yaw = atan2(siny_cosp, cosy_cosp);

    // if (diag >= 1)
    // {
    //     Serial.print(diag);
    //     Serial.print(",");
    //     Serial.print(_q_w);
    //     Serial.print(",");
    //     Serial.print(_q_x);
    //     Serial.print(",");
    //     Serial.print(_q_y);
    //     Serial.print(",");
    //     Serial.print(_q_z);
    //     Serial.print(",");
    //     Serial.print(roll * 180.0 / PI);
    //     Serial.print(",");
    //     Serial.print(pitch * 180.0 / PI);
    //     Serial.print(",");
    //     Serial.print(yaw * 180.0 / PI);
    //     Serial.println("");
    // }

    return Euler(roll * 180.0 / PI, pitch * 180.0 / PI, yaw * 180.0 / PI);
}

AngleTransformation::AngleTransformation(float u1[3], float u2[3], float u3[3])
{
    // unit vector inputs
    for (int i = 0; i < 3; i++)
    {
        _u1[i] = u1[i];
        _u2[i] = u2[i];
        _u3[i] = u3[i];
    }
    // TODO: add validation of unit vector
}

void AngleTransformation::Transform(float roll, float pitch, float yaw)
{
    // note MPU and board difference causes roll, pitch, yaw difference in q1,q2,q3
    Quaternion q1(_u1, roll);
    Quaternion q2(_u2, pitch);
    Quaternion q3(_u3, yaw);

    Euler e1 = q1.ToEuler(1);
    Euler e2 = q2.ToEuler(2);
    Euler e3 = q3.ToEuler(3);

    _roll_t = e1._roll + e2._roll + e3._roll;
    _pitch_t = e1._pitch + e2._pitch + e3._pitch;
    _yaw_t = e1._yaw + e2._yaw + e3._yaw;
}