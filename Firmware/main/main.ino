#include "main.h"
#include "Kalman.h"

Kalman kalman;
float pitch;

DigitalEncoder M1_ENC(M1_CHA_GPIO, M1_CHB_GPIO, 1);
DigitalEncoder M2_ENC(M2_CHA_GPIO, M2_CHB_GPIO, 2);
DigitalEncoder M3_ENC(M3_CHA_GPIO, M3_CHB_GPIO, 3);

MPU_6050 MPU(MPU_ADDR, 0, 1);
RobotCS RobotAngles(1);
float u1[3] = {-0.81649658, 0.40824829, 0.40824829};
float u2[3] = {0.0, -0.70710678, 0.70710678};
float u3[3] = {0.57735027, 0.57735027, 0.57735027};
AngleTransformation MotorAngles(u1, u2, u3);

// angle_acc control gains
float fc = 10.0;
float fi = 0.01;
float flp = 500.0;
float gain = 875.0;
float phase = 8.0;
float fc = 6.5;
float fi = 0.01;
float flp = 500.0;
float gain = 400.0;
float phase = 10.0;
ControlAlgo M1_Control(M1_DIR_GPIO, 0, M1_PWM_GPIO, 1);

int control_State = 0;
int late = 0;
int mode = -1;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int diag = 1;

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
    beep();
    MPU.Init();
    beep();
}

void beep()
{
    digitalWrite(BUZZ_GPIO, HIGH);
    delay(70);
    digitalWrite(BUZZ_GPIO, LOW);
}

void loop()
{
    // M1_ENC.UpdatePosition();
    // M2_ENC.UpdatePosition();
    // M3_ENC.UpdatePosition();

    // CONTROL ENABLE/DISABLE LOGIC
    if (millis() - lastTime >= UPDATE_PERIOD)
    {
        late = millis() - lastTime - UPDATE_PERIOD;
        lastTime = millis();
        // update measurements
        MPU.GetMeasurement();                                                                                   // sensor coordinate system
        RobotAngles.MPU2Robot(MPU._AcX, MPU._AcY, MPU._AcZ, MPU._gyroAngleX, MPU._gyroAngleY, MPU._gyroAngleZ); // robot coordinate system
        // MotorAngles.Transform(RobotAngles.roll, RobotAngles.pitch, RobotAngles.yaw);                            // motor coordinate system
        // pitch = kalman.getAngle(RobotAngles.accAngleY, MPU._gyroAngleY, 1.0);

        startNow = digitalRead(START_GPIO);
        if (control_State == 0)
        { // disabled, init control gains
            control_State += 1;

            enTime = millis();
            M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase);
            M1_Control.Disable();        // disable and reset control
            digitalWrite(EMO_GPIO, LOW); // brake on
        }
        else if (control_State == 1)
        { // wait for start rise
            if ((millis() - enTime > EN_PERIOD) && (startNow == 1) && (startLast == 0))
            {
                beep();
                float target = RobotAngles.pitch;
                Serial.println("Control Enabled!");
                Serial.print("- target = ");
                Serial.println(target);

                M1_Control.SetTarget(target);
                control_State++;
                digitalWrite(EMO_GPIO, HIGH); // brake off
            }
        }
        else
        {
            // int retVal = M1_Control.ControlUpdate((RobotAngles.roll + RobotAngles.pitch) / 2.0);
            int retVal = M1_Control.ControlUpdate(RobotAngles.pitch);
            if (retVal < 0 || startNow == 0)
            {
                SerialBT.println("Control Disabled...");
                control_State = 0;
            }
        }

        startLast = startNow;
    }

    // DIAGNOSTIC LOGGING
    currentT = millis();
    if (currentT - previousT >= pos_time)
    {
        Tuning();
        monitorVoltage(1);
        previousT = currentT;
        if (diag == 1)
        {
            Serial.print(M1_Control._error, 6);
            Serial.print(", ");

            // Serial.print(MPU._AcX, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcY, 6);
            // Serial.print(", ");
            // Serial.print(MPU._AcZ, 6);
            // Serial.print(", ");

            Serial.print(RobotAngles.pitch, 6);
            Serial.print(", ");
            Serial.print(RobotAngles.accAngleY, 6);
            Serial.print(", ");
            Serial.print(MPU._gyroAngleY, 6);
            Serial.print(", ");

            Serial.print(M1_Control._out, 6);
            Serial.print(", ");
            Serial.print(M1_Control._out_calc, 6);
            Serial.print(", ");
            Serial.print(M1_Control._pwm, 6);
            Serial.print(", ");
            Serial.print(M1_Control.GetTarget(), 6);
            Serial.print(", ");

            Serial.println("");
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
        beep();
}

void Tuning()
{
    if (!Serial.available())
        return;
    char cmd = Serial.read();
    switch (cmd)
    {
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
            fc *= 1.1;
            Serial.print("increased fc = ");
            Serial.println(fc);
        }
        if (mode == 2)
        {
            fi *= 1.1;
            Serial.print("increased fi = ");
            Serial.println(fi);
        }
        if (mode == 3)
        {
            flp *= 1.1;
            Serial.print("increased flp = ");
            Serial.println(flp);
        }
        if (mode == 4)
        {
            gain *= 1.1;
            Serial.print("increased gain = ");
            Serial.println(gain);
        }
        if (mode == 5)
        {
            phase += 0.25;
            Serial.print("increased phase = ");
            Serial.println(phase);
        }
        M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase);
        break;
    case '-':
        if (mode == 1)
        {
            fc *= 0.9;
            Serial.print("decreased fc = ");
            Serial.println(fc);
        }
        if (mode == 2)
        {
            fi *= 0.9;
            Serial.print("decreased fi = ");
            Serial.println(fi);
        }
        if (mode == 3)
        {
            flp *= 0.9;
            Serial.print("decreased flp = ");
            Serial.println(flp);
        }
        if (mode == 4)
        {
            gain *= 0.9;
            Serial.print("decreased gain = ");
            Serial.println(gain);
        }
        if (mode == 5)
        {
            phase -= 0.25;
            if (phase < 0)
                phase = 0.0;
            Serial.print("decreased phase = ");
            Serial.println(phase);
        }
        M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase);
        break;
    case 's':
        M1_Control.Disable();
        delay(2000);
        break;
    }
}