#include "MPU_6050.h"

MPU_6050::MPU_6050(int MPU_addr, int acc_config, int gyro_config)
{
    _MPU_addr = MPU_addr;
    _acc_config = acc_config;
    _gyro_config = gyro_config;
    _initialized = 0;

    // default calibration offset values
    _accAngleX_Off = 0.0;
    _accAngleY_Off = 0.0;

    _GyX_Off = 0.0;
    _GyY_Off = 0.0;
    _GyZ_Off = 0.0;
}

void MPU_6050::Init()
{
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(_MPU_addr); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B - DEVICE_RESET
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        // end the transmission

    // Range and Sensitivity Configuration
    // - gyroscope
    if (_gyro_config == 0)
    {
        WriteTo(_MPU_addr, GYRO_CONFIG_ADDR, GYRO_CONFIG_250);
        _gyro_scale = 131.0;
    }
    else if (_gyro_config == 1)
    {
        WriteTo(_MPU_addr, GYRO_CONFIG_ADDR, GYRO_CONFIG_500);
        _gyro_scale = 65.5;
    }
    else if (_gyro_config == 2)
    {
        WriteTo(_MPU_addr, GYRO_CONFIG_ADDR, GYRO_CONFIG_1000);
        _gyro_scale = 32.8;
    }
    else if (_gyro_config == 0)
    {
        WriteTo(_MPU_addr, GYRO_CONFIG_ADDR, GYRO_CONFIG_2000);
        _gyro_scale = 16.4;
    }
    else
    {
        Serial.print("MPU_6050::Init(): ERROR invalid gyroscope configuration provided ");
        Serial.println(_gyro_config);
    }

    // - accelerometer
    if (_acc_config == 0)
    {
        WriteTo(_MPU_addr, ACC_CONFIG_ADDR, ACC_CONFIG_2G);
        _accel_scale = 16384.0;
    }
    else if (_acc_config == 1)
    {
        WriteTo(_MPU_addr, ACC_CONFIG_ADDR, ACC_CONFIG_4G);
        _accel_scale = 8192.0;
    }
    else if (_acc_config == 2)
    {
        WriteTo(_MPU_addr, ACC_CONFIG_ADDR, ACC_CONFIG_8G);
        _accel_scale = 4096.0;
    }
    else if (_acc_config == 3)
    {
        WriteTo(_MPU_addr, ACC_CONFIG_ADDR, ACC_CONFIG_16G);
        _accel_scale = 2048.0;
    }
    else
    {
        Serial.print("MPU_6050::Init(): ERROR invalid accelerometer configuration provided ");
        Serial.println(_acc_config);
    }
    // TODO: coinfigure filter
    WriteTo(_MPU_addr, 0x1A, 0x00);

    _gyroAngleX = 0.0;
    _gyroAngleY = 0.0;
    _gyroAngleZ = 0.0;
    _currentTime = millis();

    _initialized = 1;
    // run sensor offset calibration
    Calibrate();

    // initial value
    _gyroAngleX = _accAngleX;
    _gyroAngleY = _accAngleY;
}

void MPU_6050::GetMeasurement()
{
    if (_initialized == 1)
    {
        Wire.beginTransmission(_MPU_addr);                               // Start communication with MPU6050 // MPU=0x68
        Wire.write(ACC_DATA_BASE_ADDR);                                  // acceleration base address
        Wire.endTransmission(false);                                     //
        Wire.requestFrom(_MPU_addr, 6, true);                            // Read 6 registers total, each axis value is stored in 2 registers
        _AcX = (short)((Wire.read() << 8) | Wire.read()) / _accel_scale; // X-axis value [gees]
        _AcY = (short)((Wire.read() << 8) | Wire.read()) / _accel_scale; // Y-axis value [gees]
        _AcZ = (short)((Wire.read() << 8) | Wire.read()) / _accel_scale; // Z-axis value [gees]

        _previousTime = _currentTime;
        _currentTime = millis();
        float elapsedTime = (_currentTime - _previousTime) / 1000.0;

        Wire.beginTransmission(_MPU_addr);                                         // Start communication with MPU6050 // MPU=0x68
        Wire.write(GYRO_DATA_BASE_ADDR);                                           // acceleration base address
        Wire.endTransmission(false);                                               //
        Wire.requestFrom(_MPU_addr, 6, true);                                      // Read 6 registers total, each axis value is stored in 2 registers
        _GyX = (short)((Wire.read() << 8) | Wire.read()) / _gyro_scale + _GyX_Off; // X-axis value [deg/s]
        _GyY = (short)((Wire.read() << 8) | Wire.read()) / _gyro_scale + _GyY_Off; // Y-axis value [deg/s]
        _GyZ = (short)((Wire.read() << 8) | Wire.read()) / _gyro_scale + _GyZ_Off; // Z-axis value [deg/s]

        _gyroAngleX = _GyX * elapsedTime;
        _gyroAngleY = _GyY * elapsedTime;
        _gyroAngleZ = _GyZ * elapsedTime;
    }
    else
        Serial.println("MPU_6050::GetMeasurement(): ERROR cannot take measurement before class is initialized");
}

void MPU_6050::Calibrate()
{
    Serial.println("MPU_6050::Calibrate(): Calibration Started");
    // reset calibration to zero
    // _AcX_Off = 0.0;
    // _AcY_Off = 0.0;
    // _AcZ_Off = 0.0;

    _accAngleX_Off = 0.0;
    _accAngleY_Off = 0.0;

    _GyX_Off = 0.0;
    _GyY_Off = 0.0;
    _GyZ_Off = 0.0;

    float temp_accAngleX_Off = 0.0;
    float temp_accAngleY_Off = 0.0;
    float temp_GyX_Off = 0.0;
    float temp_GyY_Off = 0.0;
    float temp_GyZ_Off = 0.0;
    for (int i = 0; i < CAL_NUM; i++)
    {
        GetMeasurement();
        temp_accAngleX_Off += _accAngleX;
        temp_accAngleY_Off += _accAngleY;

        temp_GyX_Off += _GyX;
        temp_GyY_Off += _GyY;
        temp_GyZ_Off += _GyZ;
    }

    // set calibration
    _accAngleX_Off = -temp_accAngleX_Off / CAL_NUM;
    _accAngleY_Off = -temp_accAngleY_Off / CAL_NUM;

    _GyX_Off = -temp_GyX_Off / CAL_NUM;
    _GyY_Off = -temp_GyY_Off / CAL_NUM;
    _GyZ_Off = -temp_GyZ_Off / CAL_NUM;

    Serial.print("MPU_6050::Calibrate(): Acc AngleX Offset = ");
    Serial.println(_accAngleX_Off);
    Serial.print("MPU_6050::Calibrate(): Acc AngleY Offset = ");
    Serial.println(_accAngleY_Off);
    Serial.print("MPU_6050::Calibrate(): GyX Offset = ");
    Serial.println(_GyX_Off);
    Serial.print("MPU_6050::Calibrate(): GyY Offset = ");
    Serial.println(_GyY_Off);
    Serial.print("MPU_6050::Calibrate(): GyZ Offset = ");
    Serial.println(_GyZ_Off);
    Serial.println("MPU_6050::Calibrate(): Calibration Complete");
}

void MPU_6050::WriteTo(byte device, byte address, byte value)
{
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
}