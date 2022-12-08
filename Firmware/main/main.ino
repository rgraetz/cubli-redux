#include "main.h"

DigitalEncoder M1_ENC(M1_CHA_GPIO, M1_CHB_GPIO, 1);
DigitalEncoder M2_ENC(M2_CHA_GPIO, M2_CHB_GPIO, 2);
DigitalEncoder M3_ENC(M3_CHA_GPIO, M3_CHB_GPIO, 3);

MPU_6050 MPU(MPU_ADDR, 0, 1);
RobotCS RobotAngles(1);
float u1[3] = {-0.81649658, 0.40824829, 0.40824829};
float u2[3] = {0.0, -0.70710678, 0.70710678};
float u3[3] = {0.57735027, 0.57735027, 0.57735027};
AngleTransformation MotorAngles(u1, u2, u3);

// VELOCITY CONTROL GAINS
float fc1 = 10.0;
float fi1 = 0.0;
float flp1 = 200.0;
float gain1 = 1000.0;
float phase1 = 3.0;

// POSITION CONTROL GAINS
float fc2 = 1.0;
float fi2 = 0.001;
float flp2 = 10.0;
float gain2 = 0.01;
float phase2 = 0.0;

ControlAlgo M1_Control(M1_DIR_GPIO, 1, M1_PWM_GPIO, 1); // Z-direction motor "black"
ControlAlgo M2_Control(M2_DIR_GPIO, 0, M2_PWM_GPIO, 2); // Y-direction motor "silver"
ControlAlgo M3_Control(M3_DIR_GPIO, 0, M3_PWM_GPIO, 3); // X-direction motor "purple"

int current_State = 0;
int next_State = 0;
int control_Error = 0;
int late = 0;
int mode = -1;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int diag = 0;
float frf_amp = 50;
float frf_freq = 1.0;

void setup()
{
    Serial.begin(115200);
    Serial.print("State,Error*10,Pitch,AccY,GyroY,Out,PWM,Target");
    SerialBT.begin("ESP32_Cube");
    Serial.println("Application started");

    pinMode(EMO_GPIO, OUTPUT);
    digitalWrite(EMO_GPIO, LOW);

    pinMode(BUZZ_GPIO, OUTPUT);
    digitalWrite(BUZZ_GPIO, LOW);
    pinMode(BATT_AIN, INPUT);

    // MPU initialization
    beep(1);
    MPU.Init();
    beep(1);
}

void beep(int num)
{
    for (int i = 0; i < num; i++)
    {
        digitalWrite(BUZZ_GPIO, HIGH);
        delay(70);
        digitalWrite(BUZZ_GPIO, LOW);
        if (i < num - 1)
            delay(70);
    }
}

void Stop()
{
    // disable and reset control
    M1_Control.Disable();
    M2_Control.Disable();
    M3_Control.Disable();
    digitalWrite(EMO_GPIO, LOW); // brake on
}

void loop()
{
    // CONTROL STATE MACHINE
    current_State = next_State;
    if (current_State == INIT) // initialization state
    {
        // update transition timing
        edge1_t = millis();
        edge2_t = millis();
        edge3_t = millis();

        // move to next state
        next_State = IDLE;
    }
    else if (current_State == IDLE) // wait for control inputs
    {
        if ((abs(RobotAngles.roll - EDGE1_R) > EDGE_TOLERANCE) || (abs(RobotAngles.pitch - EDGE1_P) > EDGE_TOLERANCE) || (digitalRead(START_GPIO) == 0))
            edge1_t = millis();
        if ((abs(RobotAngles.roll - EDGE2_R) > EDGE_TOLERANCE) || (abs(RobotAngles.pitch - EDGE2_P) > EDGE_TOLERANCE) || (digitalRead(START_GPIO) == 0))
            edge2_t = millis();
        if ((abs(RobotAngles.roll - EDGE3_R) > EDGE_TOLERANCE) || (abs(RobotAngles.pitch - EDGE3_P) > EDGE_TOLERANCE) || (digitalRead(START_GPIO) == 0))
            edge3_t = millis();
        if ((abs(RobotAngles.roll - CORNER_R) > EDGE_TOLERANCE) || (abs(RobotAngles.pitch - CORNER_P) > EDGE_TOLERANCE) || (digitalRead(START_GPIO) == 0))
            corner_t = millis();

        if (millis() - edge1_t > EN_PERIOD_MSEC)
        {
            Serial.println("Edge 1 Control Enabled!");
            beep(1);
            next_State = EDGE1;
        }
        else if (millis() - edge2_t > EN_PERIOD_MSEC)
        {
            Serial.println("Edge 2 Control Enabled!");
            beep(2);
            next_State = EDGE2;
        }
        else if (millis() - edge3_t > EN_PERIOD_MSEC)
        {
            Serial.println("Edge 3 Control Enabled!");
            beep(3);
            next_State = EDGE3;
        }
        else if (millis() - corner_t > EN_PERIOD_MSEC)
        {
            Serial.println("Corner Control Enabled!");
            beep(4);
            next_State = CORNER;
        }
    }
    else
    {
        if ((control_Error > 0) || (digitalRead(START_GPIO) == 0))
        {
            Serial.println("Control Disabled...");
            beep(1);
            next_State = INIT;
        }
    }

    // MEASUREMENT UPDATE
    if (micros() - measurement_t >= MEASUREMENT_PERIOD_USEC)
    {
        // measurement update time
        measurement_late = micros() - measurement_t - MEASUREMENT_PERIOD_USEC;
        measurement_t = micros();

        // update measurements
        MPU.GetMeasurement();                                                                                   // sensor coordinate system
        RobotAngles.MPU2Robot(MPU._AcX, MPU._AcY, MPU._AcZ, MPU._gyroAngleX, MPU._gyroAngleY, MPU._gyroAngleZ); // robot coordinate system
        // MotorAngles.Transform(RobotAngles.roll, RobotAngles.pitch, RobotAngles.yaw);                            // motor coordinate system
    }

    // CONTROL UPDATE
    if (micros() - control_t >= CONTROL_PERIOD_USEC)
    {
        // control update time
        control_late = micros() - control_t - CONTROL_PERIOD_USEC;
        control_t = micros();
        if (current_State == INIT)
        {
            // set control gains
            M1_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
            M2_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
            M3_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

            M1_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
            M2_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
            M3_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
            // stop motors
            Stop();
        }
        else if (current_State == IDLE)
        {
            // stop motors
            Stop();
            // set control target
            M1_Control.SetTarget(RobotAngles.pitch);
            // M2_Control.SetTarget(-target);
            // M3_Control.SetTarget(-target);
        }
        else if (current_State == EDGE1)
        {
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            // control_Error = M1_Control.ControlUpdate(RobotAngles.pitch);
            control_Error = M1_Control.ControlUpdate(RobotAngles.pitch, RobotAngles.gyroAngleY);
            M2_Control.Disable();
            M3_Control.Disable();
            // M1_Control.SetOutput(frf_amp * sin(frf_freq * millis() / 1000 * 2 * PI));
        }
        else if (current_State == EDGE2)
        {
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            M1_Control.Disable();
            // control_Error = M2_Control.ControlUpdate(-RobotAngles.pitch);
            M3_Control.Disable();
        }
        else if (current_State == EDGE3)
        {
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            M1_Control.Disable();
            M2_Control.Disable();
            // control_Error = M3_Control.ControlUpdate(-RobotAngles.pitch);
        }
        else if (current_State == CORNER)
        {
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            // control_Error = M1_Control.ControlUpdate(-RobotAngles.pitch) +
            //                 M2_Control.ControlUpdate(-RobotAngles.pitch) +
            //                 M3_Control.ControlUpdate(-RobotAngles.pitch);
        }
        else
        {
            Serial.println("Error: Incorrect control state...");
            throw std::invalid_argument("Incorrect control state");
        }
    }

    //     startNow = digitalRead(START_GPIO);
    //     if (control_State == 0) // control setup
    //     {                       // disabled, init control gains
    //         control_State += 1;
    //         enTime = millis();
    //         Stop();
    //     }
    //     else if (control_State == 1)
    //     { // wait for start rise
    //         if ((millis() - enTime > EN_PERIOD) && (startNow == 1) && (startLast == 0))
    //         {
    //             M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase, 0);
    //             M2_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase, 0);
    //             M3_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase, 0);

    //             beep();
    //             float target = RobotAngles.pitch;
    //             Serial.println("Control Enabled!");
    //             Serial.print("- target = ");
    //             Serial.println(target);

    //             // M3_Control.SetTarget(target);
    //             M1_Control.SetTarget(-target);
    //             control_State++;
    //             digitalWrite(EMO_GPIO, HIGH); // brake off
    //         }
    //     }
    //     else
    //     {
    //         // int retVal = M3_Control.ControlUpdate(RobotAngles.pitch);
    //         int retVal = M1_Control.ControlUpdate(-RobotAngles.pitch);
    //         if (retVal < 0 || startNow == 0)
    //         {
    //             SerialBT.println("Control Disabled...");
    //             control_State = 0;
    //         }
    //     }

    //     startLast = startNow;
    // }

    // DIAGNOSTIC LOGGING
    if (millis() - diagnostic_t >= DIAGNOSTIC_PERIOD_MSEC)
    {
        // diagnostic update time
        diagnostic_late = millis() - diagnostic_t - DIAGNOSTIC_PERIOD_MSEC;
        diagnostic_t = millis();

        Tuning();
        monitorVoltage(1);
        if (diag == 1)
        {
            if ((current_State == EDGE1)) // || (current_State == IDLE))
            {
                Serial.print("ErrorPos:");
                Serial.print(M1_Control._pos_error, 6);
                Serial.print(",");
                Serial.print("ErrorVel:");
                Serial.print(M1_Control._vel_error, 6);
                Serial.print(",");
                Serial.print("Angle:");
                Serial.print(RobotAngles.pitch, 6);
                Serial.print(",");
                Serial.print("AngleGyro:");
                Serial.print(RobotAngles.gyroAngleY, 6);
                Serial.print(",");
                Serial.print("MotorAcc:");
                Serial.print(M1_Control._accel_out, 6);
                Serial.print(", ");
                Serial.print("MotorVel:");
                Serial.print(M1_Control._velocity_out, 6);
                // Serial.print(", ");
                // Serial.print("PWM:");
                // Serial.print(M1_Control._pwm, 6);
                Serial.print(", ");
                Serial.print("TargetVel:");
                Serial.print(M1_Control.GetTarget(CONTROL_VEL), 6);
                Serial.print(", ");
                Serial.print("TargetPos:");
                Serial.print(M1_Control.GetTarget(CONTROL_POS), 6);
                Serial.print(", ");
                Serial.print("MeasLate:");
                Serial.print(measurement_late, 6);

                Serial.println("");
            }
            else
            {
                // Serial.print("AccX:");
                // Serial.print(MPU._AcX, 6);
                // Serial.print(", ");
                // Serial.print("AccY:");
                // Serial.print(MPU._AcY, 6);
                // Serial.print(", ");
                // Serial.print("AccZ:");
                // Serial.print(MPU._AcZ, 6);
                // Serial.print(", ");
                Serial.print("AccXf:");
                Serial.print(RobotAngles.GetAccX(), 6);
                Serial.print(", ");
                Serial.print("AccYf:");
                Serial.print(RobotAngles.GetAccY(), 6);
                Serial.print(", ");
                Serial.print("AccZf:");
                Serial.print(RobotAngles.GetAccZ(), 6);
                Serial.print(", ");
                Serial.print("AccAngleX:");
                Serial.print(RobotAngles.accAngleX, 6);
                Serial.print(", ");
                Serial.print("AccAngleY:");
                Serial.print(RobotAngles.accAngleY, 6);
                Serial.print(", ");
                Serial.print("Roll:");
                Serial.print(RobotAngles.roll, 6);
                Serial.print(", ");
                Serial.print("Pitch:");
                Serial.print(RobotAngles.pitch, 6);
                // Serial.print(", ");
                // Serial.print("PWM:");
                // Serial.print(M1_Control._pwm, 6);
                Serial.print(", ");
                Serial.print("MeasLate:");
                Serial.print(measurement_late, 6);

                Serial.println("");
            }
            // Serial.print(M1_Control._error, 6);
            // Serial.print(", ");
            // // Serial.print(RobotAngles.roll, 6);
            // // Serial.print(", ");
            // Serial.print(-RobotAngles.pitch, 6);
            // Serial.print(", ");
            // // Serial.print(-RobotAngles.accAngleY, 6);
            // // Serial.print(", ");

            // Serial.print(MPU._AcX, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcY, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcZ, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcZ - 0.001  * M1_Control._out_calc, 6);
            // Serial.print(", ");

            // // Serial.print(M1_Control._out, 6);
            // // Serial.print(", ");
            // Serial.print(M1_Control._out_calc, 6);
            // Serial.print(", ");
            // Serial.print(M1_Control._pwm, 6);
            // Serial.print(", ");
            // Serial.print(M1_Control.GetTarget(), 6);
            // Serial.print(", ");

            // Serial.println("");

            // accel filter validation
            // Serial.print(MPU._AcX, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcY, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcZ, 6);
            // Serial.print(", ");

            // Serial.print(RobotAngles.GetAccX(), 6);
            // Serial.print(", ");
            // Serial.print(RobotAngles.GetAccY(), 6);
            // Serial.print(", ");
            // Serial.print(RobotAngles.GetAccZ(), 6);
            // Serial.print(", ");

            // Serial.println("");

            // SerialBT.print(RobotAngles.roll, 6);
            // SerialBT.print(", ");
            // SerialBT.print(RobotAngles.pitch, 6);
            // SerialBT.print(", ");
            // SerialBT.print(RobotAngles.yaw, 6);
            // SerialBT.print(", ");
            // SerialBT.println("");
        }
    }
}

void monitorVoltage(int buzz)
{ // ESP32 12bit ADC with 0-3.3V input range
    // 4:1 voltage divider
    double voltage = (double)analogRead(BATT_AIN) * 3.3 / 4096.0 * 4.0;
    // SerialBT.print("monitorVoltage(): ");
    // SerialBT.print(voltage);
    // SerialBT.println(" V");
    if (voltage > 8 && voltage < 9.5 && buzz == 1)
        beep(1);
}

void Tuning()
{
    if (!Serial.available())
        return;
    char cmd = Serial.read();
    switch (cmd)
    {
    case 'd':
        if (diag == 1)
            diag = 0;
        else
            diag = 1;
        break;
    case '1':
        mode = 1;
        break;
    case '2':
        mode = 2;
        break;
    case '3':
        mode = 3;
        break;
    case '4':
        mode = 4;
        break;
    case '5':
        mode = 5;
        break;
    case '6':
        mode = 6;
        break;
    case '7':
        mode = 7;
        break;
    case '8':
        mode = 8;
        break;
    case '9':
        mode = 9;
        break;
    case '0':
        mode = 0;
        break;
    case '+':
        if (mode == 1)
        {
            fc1 *= 1.1;
            Serial.print("increased fc1 = ");
            Serial.println(fc1);
        }
        if (mode == 2)
        {
            fi1 *= 1.1;
            Serial.print("increased fi1 = ");
            Serial.println(fi1);
        }
        if (mode == 3)
        {
            flp1 *= 1.1;
            Serial.print("increased flp1 = ");
            Serial.println(flp1);
        }
        if (mode == 4)
        {
            gain1 *= 1.1;
            Serial.print("increased gain1 = ");
            Serial.println(gain1);
        }
        if (mode == 5)
        {
            if (phase1 < 1.0)
                phase1 = 1.0;
            else
                phase1 += 0.25;
            Serial.print("increased phase1 = ");
            Serial.println(phase1);
        }
        if (mode == 6)
        {
            fc2 *= 1.1;
            Serial.print("increased fc2 = ");
            Serial.println(fc2);
        }
        if (mode == 7)
        {
            fi2 *= 1.1;
            Serial.print("increased fi2 = ");
            Serial.println(fi2);
        }
        if (mode == 8)
        {
            flp2 *= 1.1;
            Serial.print("increased flp2 = ");
            Serial.println(flp2);
        }
        if (mode == 9)
        {
            gain2 *= 1.1;
            Serial.print("increased gain2 = ");
            Serial.println(gain2);
        }
        if (mode == 0)
        {
            if (phase2 < 1.0)
                phase2 = 1.0;
            else
                phase2 += 0.25;
            Serial.print("increased phase2 = ");
            Serial.println(phase2);
        }

        M1_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
        M2_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
        M3_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

        M1_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
        M2_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
        M3_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

        break;
    case '-':
        if (mode == 1)
        {
            fc1 *= 0.9;
            Serial.print("decreased fc1 = ");
            Serial.println(fc1);
        }
        if (mode == 2)
        {
            fi1 *= 0.9;
            Serial.print("decreased fi1 = ");
            Serial.println(fi1);
        }
        if (mode == 3)
        {
            flp1 *= 0.9;
            Serial.print("decreased flp1 = ");
            Serial.println(flp1);
        }
        if (mode == 4)
        {
            gain1 *= 0.9;
            Serial.print("decreased gain1 = ");
            Serial.println(gain1);
        }
        if (mode == 5)
        {
            phase1 -= 0.25;
            if (phase1 < 1.0)
                phase1 = 0.0;
            Serial.print("decreased phase1 = ");
            Serial.println(phase1);
        }
        if (mode == 6)
        {
            fc2 *= 0.9;
            Serial.print("decreased fc2 = ");
            Serial.println(fc2);
        }
        if (mode == 7)
        {
            fi2 *= 0.9;
            Serial.print("decreased fi2 = ");
            Serial.println(fi2);
        }
        if (mode == 8)
        {
            flp2 *= 0.9;
            Serial.print("decreased flp2 = ");
            Serial.println(flp2);
        }
        if (mode == 9)
        {
            gain2 *= 0.9;
            Serial.print("decreased gain2 = ");
            Serial.println(gain2);
        }
        if (mode == 0)
        {
            phase2 -= 0.25;
            if (phase2 < 1.0)
                phase2 = 0.0;
            Serial.print("decreased phase2 = ");
            Serial.println(phase2);
        }

        M1_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
        M2_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
        M3_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

        M1_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
        M2_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
        M3_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

        break;
    case 's':
        M1_Control.Disable();
        M2_Control.Disable();
        M3_Control.Disable();
        delay(2000);
        break;
    }
}