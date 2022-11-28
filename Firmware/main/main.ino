#include "main.h"

DigitalEncoder M1_ENC(M1_CHA_GPIO, M1_CHB_GPIO, 1);
DigitalEncoder M2_ENC(M2_CHA_GPIO, M2_CHB_GPIO, 2);
DigitalEncoder M3_ENC(M3_CHA_GPIO, M3_CHB_GPIO, 3);

MPU_6050 MPU(MPU_ADDR, 0, 1);

float u1[3] = {-0.81649658, 0.40824829, 0.40824829};
float u2[3] = {0.0, -0.70710678, 0.70710678};
float u3[3] = {0.57735027, 0.57735027, 0.57735027};
AngleTransformation MotorAngles(u1, u2, u3);

float fc = 10;
float fi = 0.1;
float flp = 500;
float gain = 100.0;
float phase = 11.0;
ControlAlgo M1_Control(M1_DIR_GPIO, 1, M1_PWM_GPIO, 1);

float offset1 = 0;
float offset2 = 0;
float offset3 = 0;

int control_State = 0;
int late = 0;
int mode = 0;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int useBT = 1;

void setup()
{
    Serial.begin(115200);
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
        MPU.GetMeasurement();                                          // sensor coordinate system
        MotorAngles.Transform(-MPU.roll, -MPU.pitch + 90.0, -MPU.yaw); // motor coordinate system, Note: negative to get RHR and offset of 90

        startNow = digitalRead(START_GPIO);
        if (control_State == 0)
        { // disabled, init control gains
            control_State += 1;

            enTime = millis();

            // float M1_gains[8] = {1.000000, -1.120198, 0.120198, 0.000000, 0.439970, 0.000138, 0.000000, 0.000000};
            // float M1_gains[8] = {50.000000, 10000.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // M1_Control.SetGains(M1_gains);
            M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase);

            M1_Control.Disable();        // disable and reset control
            digitalWrite(EMO_GPIO, LOW); // brake on
        }
        else if (control_State == 1)
        { // wait for start rise
            if ((millis() - enTime > EN_PERIOD) && (startNow == 1) && (startLast == 0))
            {
                offset1 = MPU.roll;
                offset2 = MPU.pitch;
                offset3 = MPU.yaw;

                beep();
                // float target = MotorAngles._yaw_t;
                float target = MPU.yaw;
                SerialBT.println("Control Enabled!");
                SerialBT.print("- target = ");
                SerialBT.println(target);

                M1_Control.SetTarget(target);
                control_State++;
                digitalWrite(EMO_GPIO, HIGH); // brake off
            }
        }
        else
        {
            // M1_Control.ControlUpdate(MotorAngles._pitch_t); // MotorAngles._yaw_t);
            M1_Control.ControlUpdate(MPU.yaw); // + MPU.pitch);

            if (startNow == 0)
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
        // MPU.GetMeasurement();
        monitorVoltage(1);
        previousT = currentT;
        if (useBT == 1)
        {
            // Serial.print(MotorAngles._roll_t);
            // Serial.print(", ");
            Serial.print(M1_Control._error * 100);
            Serial.print(", ");
            // Serial.print(MotorAngles._yaw_t);
            // Serial.print(", ");
            Serial.print(MPU.roll - offset1);
            Serial.print(", ");
            Serial.print(MPU.pitch - offset2);
            Serial.print(", ");
            Serial.print(MPU.yaw - offset3);
            Serial.println("");
            // SerialBT.print(M1_ENC._position);
            // SerialBT.print(", ");
            // SerialBT.print(M2_ENC._position);
            // SerialBT.print(", ");
            // SerialBT.print(M3_ENC._position);
            // SerialBT.print(", ");
            SerialBT.print(MPU.roll);
            SerialBT.print(", ");
            SerialBT.print(MPU.pitch);
            SerialBT.print(", ");
            SerialBT.print(MPU.yaw);

            // SerialBT.print(", ");
            // SerialBT.print(MotorAngles._roll_t);
            // SerialBT.print(", ");
            // SerialBT.print(MotorAngles._pitch_t);
            // SerialBT.print(", ");
            // SerialBT.print(MotorAngles._yaw_t);

            // SerialBT.print(", ");
            // SerialBT.print(M1_Control._error);
            SerialBT.print(", ");
            SerialBT.print(M1_Control._out);
            SerialBT.print(", ");
            SerialBT.print(M1_Control._out_full);
            SerialBT.print(", ");
            SerialBT.print(M1_Control._pwm);
            SerialBT.print(", ");
            SerialBT.print(M1_Control.GetTarget());
            // SerialBT.print(", ");
            // SerialBT.print(late);
            // SerialBT.print(", ");
            // SerialBT.print(lastTime);

            SerialBT.println("");
        }
        else
        {
            Serial.print(MPU.roll);
            Serial.print(", ");
            Serial.print(MPU.pitch);
            Serial.print(", ");
            Serial.print(MPU.yaw);
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
    if (!SerialBT.available())
        return;
    char cmd = SerialBT.read();
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
    case '+':
        if (mode == 1)
        {
            fc *= 1.1;
            SerialBT.print("increased fc = ");
            SerialBT.println(fc);
        }
        if (mode == 2)
        {
            fi *= 1.1;
            SerialBT.print("increased fi = ");
            SerialBT.println(fi);
        }
        if (mode == 3)
        {
            flp *= 1.1;
            SerialBT.print("increased flp = ");
            SerialBT.println(flp);
        }
        if (mode == 4)
        {
            gain *= 1.1;
            SerialBT.print("increased gain = ");
            SerialBT.println(gain);
        }
        if (mode == 5)
        {
            phase += 0.25;
            SerialBT.print("increased phase = ");
            SerialBT.println(phase);
        }
        M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase);
        break;
    case '-':
        if (mode == 1)
        {
            fc *= 0.9;
            SerialBT.print("increased fc = ");
            SerialBT.println(fc);
        }
        if (mode == 2)
        {
            fi *= 0.9;
            SerialBT.print("increased fi = ");
            SerialBT.println(fi);
        }
        if (mode == 3)
        {
            flp *= 0.9;
            SerialBT.print("increased flp = ");
            SerialBT.println(flp);
        }
        if (mode == 4)
        {
            gain *= 0.9;
            SerialBT.print("increased gain = ");
            SerialBT.println(gain);
        }
        if (mode == 5)
        {
            phase -= 0.25;
            if (phase < 0)
                phase = 0;
            SerialBT.print("increased phase = ");
            SerialBT.println(phase);
        }
        M1_Control.CalcGains(fc * 2 * PI, fi * 2 * PI, flp * 2 * PI, gain, phase);
        break;
    case 's':
        M1_Control.Disable();
        delay(2000);
        break;
    }
}