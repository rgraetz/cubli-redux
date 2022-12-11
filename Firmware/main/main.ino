#include "main.h"

DigitalEncoder M1_ENC(M1_CHA_GPIO, M1_CHB_GPIO, 1);
DigitalEncoder M2_ENC(M2_CHA_GPIO, M2_CHB_GPIO, 2);
DigitalEncoder M3_ENC(M3_CHA_GPIO, M3_CHB_GPIO, 3);

MPU_6050 MPU(MPU_ADDR, 0, 1);
RobotCS RobotAngles(1);
float u1[3] = {-0.81649658, 0.40824829, 0.40824829};
float u2[3] = {0.0, -0.70710678, 0.70710678};
float u3[3] = {0.57735027, 0.57735027, 0.57735027};
// AngleTransformation MotorAngles(u1, u2, u3);

float m1_pos, m1_vel;
float m2_pos, m2_vel;
float m3_pos, m3_vel;

// EDGE
// // VELOCITY CONTROL GAINS
// float fc1 = 10.0;
// float fi1 = 0.0;
// float flp1 = 200.0;
// float gain1 = 1000.0;
// float phase1 = 3.0;

// // POSITION CONTROL GAINS
// float fc2 = 1.0;
// float fi2 = 0.001;
// float flp2 = 10.0;
// float gain2 = 0.01;
// float phase2 = 0.0;

// // BALANCE CONTROL GAINS
// float fc3 = 1.0;
// float fi3 = 0.025;
// float flp3 = 1.0;
// float gain3 = 0.001;
// float phase3 = 0.0;

// CORNER
// VELOCITY CONTROL GAINS
float fc1 = 10.0;
float fi1 = 0.0;
float flp1 = 200.0;
float gain1 = 300.0;
float phase1 = 3.0;

// POSITION CONTROL GAINS
float fc2 = 1.0;
float fi2 = 0.001;
float flp2 = 10.0;
float gain2 = 0.01;
float phase2 = 0.0;

// BALANCE CONTROL GAINS
float fc3 = 1.0;
float fi3 = 0.001;
float flp3 = 1.0;
float gain3 = 0.001;
float phase3 = 0.0;

ControlAlgo M1_Control(M1_DIR_GPIO, 0, M1_PWM_GPIO, 1); // Z-direction motor "black"
ControlAlgo M2_Control(M2_DIR_GPIO, 0, M2_PWM_GPIO, 2); // Y-direction motor "silver"
ControlAlgo M3_Control(M3_DIR_GPIO, 0, M3_PWM_GPIO, 3); // X-direction motor "purple"

ControlAlgo R_Control(M1_DIR_GPIO, 0, M1_PWM_GPIO, 1);
ControlAlgo P_Control(M2_DIR_GPIO, 0, M2_PWM_GPIO, 2);
ControlAlgo Y_Control(M3_DIR_GPIO, 0, M3_PWM_GPIO, 3);

int current_State = 0;
int next_State = 0;
int control_Error = 0;
int late = 0;
int mode = -1;
int cont = -1;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int diag = 1;
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
    R_Control.Disable();
    P_Control.Disable();
    Y_Control.Disable();
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

        // single edge control
        m1_pos = -0.5 * (RobotAngles.roll + RobotAngles.pitch);
        m1_vel = -0.5 * (RobotAngles.GetGyroX() + RobotAngles.GetGyroY());

        m2_pos = RobotAngles.roll;
        m2_vel = RobotAngles.GetGyroX();

        m3_pos = 0.5 * (RobotAngles.pitch + RobotAngles.roll);
        m3_vel = 0.5 * (RobotAngles.GetGyroY() + RobotAngles.GetGyroX());

        // // roll control
        // // - combination 1
        // m1_pos = -RobotAngles.roll;
        // m1_vel = -RobotAngles.GetGyroX();

        // m2_pos = RobotAngles.roll;
        // m2_vel = RobotAngles.GetGyroX();

        // m3_pos = 0.0;
        // m3_vel = 0.0;

        // // - combination 2
        // m1_pos = -RobotAngles.roll;
        // m1_vel = -RobotAngles.GetGyroX();

        // m2_pos = 0.0;
        // m2_vel = 0.0;

        // m3_pos = -RobotAngles.roll;
        // m3_vel = -RobotAngles.GetGyroX();

        // // - total
        // m1_pos = -0.25 * RobotAngles.roll;
        // m1_vel = -0.25 * RobotAngles.GetGyroX();

        // m2_pos = 0.5 * RobotAngles.roll;
        // m2_vel = 0.5 * RobotAngles.GetGyroX();

        // m3_pos = -0.25 * RobotAngles.roll;
        // m3_vel = -0.25 * RobotAngles.GetGyroX();

        // pitch control
        // m1_pos = -0.5 * RobotAngles.pitch;
        // m1_vel = -0.5 * RobotAngles.GetGyroY();

        // m2_pos = 0.0;
        // m2_vel = 0.0;

        // m3_pos = 0.5 * RobotAngles.pitch;
        // m3_vel = 0.5 * RobotAngles.GetGyroY();

        // yaw control
        // m1_pos = 0.0;
        // m1_vel = -RobotAngles.GetGyroZ() / 3.0;
        // m2_pos = 0.0;
        // m2_vel = -RobotAngles.GetGyroZ() / 3.0;
        // m3_pos = 0.0;
        // m3_vel = -RobotAngles.GetGyroZ() / 3.0;
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
            M1_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
            M2_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
            M3_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

            M1_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
            M2_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
            M3_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

            M1_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 1);
            M2_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
            M3_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);

            // set control gains
            R_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
            P_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
            Y_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

            R_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
            P_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
            Y_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

            R_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 1);
            P_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
            Y_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
            // stop motors
            Stop();
        }
        else if (current_State == IDLE)
        {
            // stop motors
            Stop();
            // set control target
            M1_Control.SetTarget(m1_pos);
            M2_Control.SetTarget(m2_pos);
            M3_Control.SetTarget(m3_pos);
            R_Control.SetTarget(0.0);
            P_Control.SetTarget(0.0);
            Y_Control.SetTarget(0.0);
        }
        else if (current_State == EDGE1)
        { // black edge
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            // control calculations
            control_Error = M1_Control.ControlUpdate(m1_pos, m1_vel, 1);
            M2_Control.Disable();
            M3_Control.Disable();
        }
        else if (current_State == EDGE2)
        { // silver edge
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            // control calculations
            M1_Control.Disable();
            control_Error = M2_Control.ControlUpdate(m2_pos, m2_vel, 1);
            M3_Control.Disable();
        }
        else if (current_State == EDGE3)
        { // purple edge
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            // control calculations
            M1_Control.Disable();
            M2_Control.Disable();
            control_Error = M3_Control.ControlUpdate(m3_pos, m3_vel, 1);
        }
        else if (current_State == CORNER)
        {
            // brake off
            digitalWrite(EMO_GPIO, HIGH);
            // M1_Control.Disable();
            // M2_Control.Disable();
            // M3_Control.Disable();
            // control_Error = M1_Control.ControlUpdate(m1_pos, m1_vel) +
            //                 M2_Control.ControlUpdate(m2_pos, m2_vel) +
            //                 M3_Control.ControlUpdate(m3_pos, m3_vel);

            R_Control.ControlUpdate(RobotAngles.roll, RobotAngles.GetGyroX(), 0, 1, 0);
            P_Control.ControlUpdate(RobotAngles.pitch, RobotAngles.GetGyroY(), 0, 1, 0);
            Y_Control.ControlUpdate(0.0, RobotAngles.GetGyroZ(), 0, 0, 0);

            // roll only
            // R_Control.SetOutput(-0.25 * R_Control._velocity_out);
            // P_Control.SetOutput(0.5 * R_Control._velocity_out);
            // Y_Control.SetOutput(-0.25 * R_Control._velocity_out);

            // roll + yaw
            // R_Control.SetOutput(-0.25 * R_Control._velocity_out - 0.33 * Y_Control._velocity_out);
            // P_Control.SetOutput(0.5 * R_Control._velocity_out - 0.33 * Y_Control._velocity_out);
            // Y_Control.SetOutput(-0.25 * R_Control._velocity_out - 0.33 * Y_Control._velocity_out);

            // pitch only
            // R_Control.SetOutput(-0.5 * P_Control._velocity_out);
            // P_Control.SetOutput(0.0 * P_Control._velocity_out);
            // Y_Control.SetOutput(0.5 * P_Control._velocity_out);

            // yaw only
            // R_Control.SetOutput(-0.33 * Y_Control._velocity_out);
            // P_Control.SetOutput(-0.33 * Y_Control._velocity_out);
            // Y_Control.SetOutput(-0.33 * Y_Control._velocity_out);

            // roll + pitch
            R_Control.SetOutput(-0.25 * R_Control._velocity_out - 0.5 * P_Control._velocity_out);
            P_Control.SetOutput(0.5 * R_Control._velocity_out);
            Y_Control.SetOutput(-0.25 * R_Control._velocity_out + 0.5 * P_Control._velocity_out);
        }
        else
        {
            Serial.println("Error: Incorrect control state...");
            throw std::invalid_argument("Incorrect control state");
        }
    }

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
                Serial.print(m1_pos, 6);
                Serial.print(",");
                Serial.print("Velocity:");
                Serial.print(m1_pos, 6);
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
                Serial.print(M1_Control.GetTarget(CONTROL_POS) + M1_Control.GetTarget(CONTROL_BAL), 6);
                Serial.print(", ");
                Serial.print("MeasLate:");
                Serial.print(measurement_late, 6);

                Serial.println("");
            }
            else if (current_State == EDGE2)
            {
                Serial.print("ErrorPos:");
                Serial.print(M2_Control._pos_error, 6);
                Serial.print(",");
                Serial.print("ErrorVel:");
                Serial.print(M2_Control._vel_error, 6);
                Serial.print(",");
                Serial.print("Angle:");
                Serial.print(m2_pos, 6);
                Serial.print(",");
                Serial.print("Velocity:");
                Serial.print(m2_pos, 6);
                Serial.print(",");
                Serial.print("MotorAcc:");
                Serial.print(M2_Control._accel_out, 6);
                Serial.print(", ");
                Serial.print("MotorVel:");
                Serial.print(M2_Control._velocity_out, 6);
                // Serial.print(", ");
                // Serial.print("PWM:");
                // Serial.print(M2_Control._pwm, 6);
                Serial.print(", ");
                Serial.print("TargetVel:");
                Serial.print(M2_Control.GetTarget(CONTROL_VEL), 6);
                Serial.print(", ");
                Serial.print("TargetPos:");
                Serial.print(M2_Control.GetTarget(CONTROL_POS) + M2_Control.GetTarget(CONTROL_BAL), 6);
                Serial.print(", ");
                Serial.print("MeasLate:");
                Serial.print(measurement_late, 6);

                Serial.println("");
            }
            else if (current_State == EDGE3)
            {
                Serial.print("ErrorPos:");
                Serial.print(M3_Control._pos_error, 6);
                Serial.print(",");
                Serial.print("ErrorVel:");
                Serial.print(M3_Control._vel_error, 6);
                Serial.print(",");
                Serial.print("Angle:");
                Serial.print(m3_pos, 6);
                Serial.print(",");
                Serial.print("Velocity:");
                Serial.print(m3_pos, 6);
                Serial.print(",");
                Serial.print("MotorAcc:");
                Serial.print(M3_Control._accel_out, 6);
                Serial.print(", ");
                Serial.print("MotorVel:");
                Serial.print(M3_Control._velocity_out, 6);
                // Serial.print(", ");
                // Serial.print("PWM:");
                // Serial.print(M3_Control._pwm, 6);
                Serial.print(", ");
                Serial.print("TargetVel:");
                Serial.print(M3_Control.GetTarget(CONTROL_VEL), 6);
                Serial.print(", ");
                Serial.print("TargetPos:");
                Serial.print(M3_Control.GetTarget(CONTROL_POS) + M3_Control.GetTarget(CONTROL_BAL), 6);
                Serial.print(", ");
                Serial.print("MeasLate:");
                Serial.print(measurement_late, 6);

                Serial.println("");
            }
            else if (current_State == CORNER)
            {
                Serial.print("ErrorRPos:");
                Serial.print(R_Control._pos_error, 6);
                Serial.print(",");
                Serial.print("ErrorRVel:");
                Serial.print(R_Control._vel_error, 6);
                Serial.print("TargetRVel:");
                Serial.print(R_Control.GetTarget(CONTROL_VEL), 6);
                Serial.print(",");
                Serial.print("TargetRPos:");
                Serial.print(R_Control.GetTarget(CONTROL_POS) + R_Control.GetTarget(CONTROL_BAL), 6);
                Serial.print(",");

                Serial.print("ErrorPPos:");
                Serial.print(P_Control._pos_error, 6);
                Serial.print(",");
                Serial.print("ErrorPVel:");
                Serial.print(P_Control._vel_error, 6);
                Serial.print(",");
                Serial.print("TargetPVel:");
                Serial.print(P_Control.GetTarget(CONTROL_VEL), 6);
                Serial.print(",");
                Serial.print("TargetPPos:");
                Serial.print(P_Control.GetTarget(CONTROL_POS) + P_Control.GetTarget(CONTROL_BAL), 6);

                Serial.print("ErrorYPos:");
                Serial.print(Y_Control._pos_error, 6);
                Serial.print(",");
                Serial.print("ErrorYVel:");
                Serial.print(Y_Control._vel_error, 6);
                Serial.print(",");
                Serial.print("TargetYVel:");
                Serial.print(Y_Control.GetTarget(CONTROL_VEL), 6);
                Serial.print(",");
                Serial.print("TargetYPos:");
                Serial.print(Y_Control.GetTarget(CONTROL_POS) + Y_Control.GetTarget(CONTROL_BAL), 6);

                // Serial.print("ErrorPPos:");
                // Serial.print(P_Control._pos_error, 6);
                // Serial.print(",");
                // Serial.print("ErrorPVel:");
                // Serial.print(P_Control._vel_error, 6);
                // Serial.print(",");
                // Serial.print("ErrorYPos:");
                // Serial.print(Y_Control._pos_error, 6);
                // Serial.print(",");
                // Serial.print("ErrorYVel:");
                // Serial.print(Y_Control._vel_error, 6);

                Serial.println("");
            }
            else
            {
                // Serial.print("AccXf:");
                // Serial.print(RobotAngles.GetAccX(), 6);
                // Serial.print(", ");
                // Serial.print("AccYf:");
                // Serial.print(RobotAngles.GetAccY(), 6);
                // Serial.print(", ");
                // Serial.print("AccZf:");
                // Serial.print(RobotAngles.GetAccZ(), 6);
                // Serial.print(",");
                Serial.print("AngleGyroX:");
                Serial.print(RobotAngles.GetGyroX(), 6);
                Serial.print(",");
                Serial.print("AngleGyroY:");
                Serial.print(RobotAngles.GetGyroY(), 6);
                Serial.print(", ");
                Serial.print("AngleGyroZ:");
                Serial.print(RobotAngles.GetGyroZ(), 6);
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
                // Serial.print(", ");
                // Serial.print("MeasLate:");
                // Serial.print(measurement_late, 6);

                Serial.println("");

                SerialBT.print(RobotAngles.roll, 6);
                SerialBT.print(", ");
                SerialBT.print(RobotAngles.pitch, 6);
                SerialBT.print(", ");
                SerialBT.print(RobotAngles.yaw, 6);
                SerialBT.println("");
            }
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
    case 'v':
        cont = CONTROL_VEL;
        break;
    case 'p':
        cont = CONTROL_POS;
        break;
    case 'b':
        cont = CONTROL_BAL;
        break;
    case '+':
        if (mode == 1)
        {
            if (cont == CONTROL_VEL)
            {
                fc1 *= 1.1;
                Serial.print("increased fc1 = ");
                Serial.println(fc1);
            }
            else if (cont == CONTROL_POS)
            {
                fc2 *= 1.1;
                Serial.print("increased fc2 = ");
                Serial.println(fc2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                fc3 *= 1.1;
                Serial.print("increased fc3 = ");
                Serial.println(fc3, 8);
            }
        }
        else if (mode == 2)
        {
            if (cont == CONTROL_VEL)
            {
                fi1 *= 1.1;
                Serial.print("increased fi1 = ");
                Serial.println(fi1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                fi2 *= 1.1;
                Serial.print("increased fi2 = ");
                Serial.println(fi2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                fi3 *= 1.1;
                Serial.print("increased fi3 = ");
                Serial.println(fi3, 8);
            }
        }
        else if (mode == 3)
        {
            if (cont == CONTROL_VEL)
            {
                flp1 *= 1.1;
                Serial.print("increased flp1 = ");
                Serial.println(flp1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                flp2 *= 1.1;
                Serial.print("increased flp2 = ");
                Serial.println(flp2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                flp3 *= 1.1;
                Serial.print("increased flp3 = ");
                Serial.println(flp3, 8);
            }
        }
        else if (mode == 4)
        {
            if (cont == CONTROL_VEL)
            {
                gain1 *= 1.1;
                Serial.print("increased gain1 = ");
                Serial.println(gain1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                gain2 *= 1.1;
                Serial.print("increased gain2 = ");
                Serial.println(gain2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                gain3 *= 1.1;
                Serial.print("increased gain3 = ");
                Serial.println(gain3, 8);
            }
        }
        else if (mode == 5)
        {
            if (cont == CONTROL_VEL)
            {
                phase1 *= 1.1;
                if (phase1 == 0.0)
                    phase1 = 1.0;
                Serial.print("increased phase1 = ");
                Serial.println(phase1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                phase2 *= 1.1;
                if (phase2 == 0.0)
                    phase2 = 1.0;
                Serial.print("increased phase2 = ");
                Serial.println(phase2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                phase3 *= 1.1;
                if (phase3 == 0.0)
                    phase3 = 1.0;
                Serial.print("increased phase3 = ");
                Serial.println(phase3, 8);
            }
        }
        M1_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
        M2_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
        M3_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

        M1_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
        M2_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
        M3_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

        M1_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 1);
        M2_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
        M3_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);

        R_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
        P_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
        Y_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

        R_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
        P_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
        Y_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

        R_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 1);
        P_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
        Y_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);

        break;
    case '-':
        if (mode == 1)
        {
            if (cont == CONTROL_VEL)
            {
                fc1 *= 0.9;
                Serial.print("increased fc1 = ");
                Serial.println(fc1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                fc2 *= 0.9;
                Serial.print("increased fc2 = ");
                Serial.println(fc2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                fc3 *= 0.9;
                Serial.print("increased fc3 = ");
                Serial.println(fc3, 8);
            }
        }
        else if (mode == 2)
        {
            if (cont == CONTROL_VEL)
            {
                fi1 *= 0.9;
                Serial.print("increased fi1 = ");
                Serial.println(fi1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                fi2 *= 0.9;
                Serial.print("increased fi2 = ");
                Serial.println(fi2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                fi3 *= 0.9;
                Serial.print("increased fi3 = ");
                Serial.println(fi3, 8);
            }
        }
        else if (mode == 3)
        {
            if (cont == CONTROL_VEL)
            {
                flp1 *= 0.9;
                Serial.print("increased flp1 = ");
                Serial.println(flp1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                flp2 *= 0.9;
                Serial.print("increased flp2 = ");
                Serial.println(flp2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                flp3 *= 0.9;
                Serial.print("increased flp3 = ");
                Serial.println(flp3, 8);
            }
        }
        else if (mode == 4)
        {
            if (cont == CONTROL_VEL)
            {
                gain1 *= 0.9;
                Serial.print("increased gain1 = ");
                Serial.println(gain1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                gain2 *= 0.9;
                Serial.print("increased gain2 = ");
                Serial.println(gain2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                gain3 *= 0.9;
                Serial.print("increased gain3 = ");
                Serial.println(gain3, 8);
            }
        }
        else if (mode == 5)
        {
            if (cont == CONTROL_VEL)
            {
                phase1 *= 0.9;
                if (phase1 < 0.0)
                    phase1 = 0.0;
                Serial.print("increased phase1 = ");
                Serial.println(phase1, 8);
            }
            else if (cont == CONTROL_POS)
            {
                phase2 *= 0.9;
                if (phase2 < 0.0)
                    phase2 = 0.0;
                Serial.print("increased phase2 = ");
                Serial.println(phase2, 8);
            }
            else if (cont == CONTROL_BAL)
            {
                phase3 *= 0.9;
                if (phase3 < 0.0)
                    phase3 = 0.0;
                Serial.print("increased phase3 = ");
                Serial.println(phase3, 8);
            }
        }

        M1_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
        M2_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
        M3_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

        M1_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
        M2_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
        M3_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

        M1_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 1);
        M2_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
        M3_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);

        R_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 1);
        P_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);
        Y_Control.CalcGains(fc1 * 2 * PI, fi1 * 2 * PI, flp1 * 2 * PI, gain1, phase1, CONTROL_VEL, 0);

        R_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 1);
        P_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);
        Y_Control.CalcGains(fc2 * 2 * PI, fi2 * 2 * PI, flp2 * 2 * PI, gain2, phase2, CONTROL_POS, 0);

        R_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 1);
        P_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);
        Y_Control.CalcGains(fc3 * 2 * PI, fi3 * 2 * PI, flp3 * 2 * PI, gain3, phase3, CONTROL_BAL, 0);

        break;
    case 's':
        M1_Control.Disable();
        M2_Control.Disable();
        M3_Control.Disable();
        delay(2000);
        break;
    }
}