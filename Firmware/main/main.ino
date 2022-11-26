#include "main.h"

DigitalEncoder M1_ENC(M1_CHA_GPIO, M1_CHB_GPIO, 1);
DigitalEncoder M2_ENC(M2_CHA_GPIO, M2_CHB_GPIO, 2);
DigitalEncoder M3_ENC(M3_CHA_GPIO, M3_CHB_GPIO, 3);

MPU_6050 MPU(MPU_ADDR, 0, 0);

ControlAlgo M1_Control(M1_DIR_GPIO, 1, M1_PWM_GPIO, 1);
float K1 = 100.0;

int control_State = 0;

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
    if (millis() - lastTime > UPDATE_PERIOD)
    {
        lastTime = millis();
        // update measurements
        MPU.GetMeasurement();
        startNow = digitalRead(START_GPIO);

        if (control_State == 0)
        { // disabled, init control gains
            control_State += 1;

            enTime = millis();

            float M1_gains[8] = {1.0, 0.0, 0.0, 0.0, K1, 0.0, 0.0, 0.0};
            M1_Control.SetGains(M1_gains);

            M1_Control.Disable();        // disable and reset control
            digitalWrite(EMO_GPIO, LOW); // brake on
        }
        else if (control_State == 1)
        { // wait for start rise
            if ((millis() - enTime > EN_PERIOD) && (startNow == 1) && (startLast == 0))
            {
                beep();
                float target = MPU.roll;

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
            M1_Control.ControlUpdate(MPU.roll);

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
        monitorVoltage(0);
        previousT = currentT;
        if (useBT == 1)
        {
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
            // SerialBT.print(M1_Control._error);
            // SerialBT.print(", ");
            // SerialBT.print(M1_Control._out);
            SerialBT.print(", ");
            SerialBT.print(M1_Control._pwm);
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
    char param = SerialBT.read();
    if (!SerialBT.available())
        return;
    char cmd = SerialBT.read();
    switch (param)
    {
    case 'p':
        if (cmd == '+' || cmd == '=')
        {
            M1_Control.SetGain(1.0, 4, 1);
            SerialBT.println("p incremented");
        }
        if (cmd == '-')
        {
            M1_Control.SetGain(-1.0, 4, 1);
            SerialBT.println("p decremented");
        }
        break;
    }
}