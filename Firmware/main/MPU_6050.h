#ifndef MPU_6050_H
#define MPU_6050_H

#include <Arduino.h>
#include <Wire.h>

#define ACC_CONFIG_ADDR 0x1C
#define ACC_CONFIG_2G 0x00
#define ACC_CONFIG_4G 0x08
#define ACC_CONFIG_8G 0x10
#define ACC_CONFIG_16G 0x18
#define ACC_DATA_BASE_ADDR 0x3B

#define GYRO_CONFIG_ADDR 0x1B
#define GYRO_CONFIG_250 0x00
#define GYRO_CONFIG_500 0x08
#define GYRO_CONFIG_1000 0x10
#define GYRO_CONFIG_2000 0x18
#define GYRO_DATA_BASE_ADDR 0x43

#define ACC_GYRO_FILTER 0.96

#define CAL_NUM 200

class MPU_6050
{
public:
    MPU_6050(int MPU_addr, int acc_config, int gyro_config);
    void Init();
    void GetMeasurement();
    float roll, pitch, yaw;

private:
    int _MPU_addr;
    int _acc_config, _gyro_config;
    int _initialized;
    float _accel_scale;
    float _gyro_scale;
    float _previousTime, _currentTime;
    float _gyroAngleX, _gyroAngleY, _gyroAngleZ;
    float _AcX, _AcY, _AcZ;
    float _GyX, _GyY, _GyZ;
    float _accAngleX, _accAngleY;
    // calibration offset(s)
    // float _AcX_Off, _AcY_Off, _AcZ_Off;
    float _accAngleX_Off, _accAngleY_Off;
    float _GyX_Off, _GyY_Off, _GyZ_Off;
    void Calibrate();
    void WriteTo(byte device, byte address, byte value);
};

#endif