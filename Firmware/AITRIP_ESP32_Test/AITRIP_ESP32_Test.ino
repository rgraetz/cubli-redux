#include "test.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32-Cube"); //Bluetooth device name
  Serial.println("Application started");
  // put your setup code here, to run once:
  pinMode(test_GPIO, OUTPUT);
  //  pinMode(test_GPIO, INPUT);

  // test motor
  pinMode(M1_BRK_GPIO, OUTPUT);
  pinMode(M1_DIR_GPIO, OUTPUT);
  // pinMode(M1_PWM_GPIO, OUTPUT);
  ledcSetup(M1_PWM_CH, PWM_BASE_FREQ, TIMER_BIT);
  ledcAttachPin(M1_PWM_GPIO, M1_PWM_CH);

  // test encoder
  pinMode(M1_CHA_GPIO, INPUT);
  pinMode(M1_CHB_GPIO, INPUT);
  pinMode(START_GPIO, INPUT);
  pinMode(BATT_AIN, INPUT);

  // buzzer/alarm test
  ledcSetup(3, 2300, 8);
  ledcAttachPin(18, 3);
}

void loop()
{
//  M1_CHA = digitalRead(M1_CHA_GPIO);
//  M1_CHB = digitalRead(M1_CHB_GPIO);
//  Serial.print("ENC A/B = ");
//  Serial.print(M1_CHA);
//  Serial.println(M1_CHB);

  // start switch test
  int start = digitalRead(START_GPIO);
  Serial.println(start);

  // analog voltage measurement test
  int vin = analogRead(BATT_AIN);
  Serial.println(vin);

//  ledcWrite(3, 128);

//  // MOTOR TEST BEGIN
//  digitalWrite( M1_BRK_GPIO, HIGH );
//  //Serial.println("setting brake off -> I/O High");
//  
//  currentT1 = millis();
//  if ( currentT1 - previousT1 >= pwm_time ) {
//    previousT1 = currentT1;
//    M1_PWM = (M1_PWM >= 255) ? 0 : M1_PWM + 1;
//  
//    ledcWrite(M1_PWM_CH, M1_PWM);
//    SerialBT.print("pwm setting = ");
//    SerialBT.println(M1_PWM);
//  }
//  currentT1 = millis();
//  if ( currentT1 - previousT1 >= flip_time ) {
//    previousT1 = currentT1;
//    if ( value == LOW )
//    {
//      value = HIGH;
//      digitalWrite( M1_BRK_GPIO, HIGH );
//      SerialBT.println("flipping brake I/O High");
//    }
//    else if ( value == HIGH )
//    {
//      value = LOW;
//      digitalWrite( M1_BRK_GPIO, LOW );
//      SerialBT.println("flipping brake I/O Low");
//    }
//  }
//  
//  currentT2 = millis();
//  if ( currentT2 - previousT2 >= dir_time ) {
//    previousT2 = currentT2;
//    if ( value == LOW )
//    {
//      value = HIGH;
//      digitalWrite( M1_DIR_GPIO, HIGH );
//      SerialBT.println("flipping dir I/O High");
//    }
//    else if ( value == HIGH )
//    {
//      value = LOW;
//      digitalWrite( M1_DIR_GPIO, LOW );
//      SerialBT.println("flipping dir I/O Low");
//    }
//  }
//  // MOTOR TEST END
//
//  // put your main code here, to run repeatedly:
//  //  int newValue = digitalRead(test_GPIO);
//  //  if ( value != newValue )
//  //   {
//  //      value = newValue;
//  //      Serial.print("Input changed to ");
//  //      Serial.println(value);
//  //   }
//  //  Serial.println(analogRead(34));
//  //  delay(100);
//
//  currentT1 = millis();
//  if (currentT1 - previousT1 >= flip_time)
//  {
//    previousT1 = currentT1;
//    if (value == LOW)
//    {
//      value = HIGH;
//      digitalWrite(test_GPIO, HIGH);
//      SerialBT.println("flipping I/O High");
//    }
//    else if (value == HIGH)
//    {
//      value = LOW;
//      digitalWrite(test_GPIO, LOW);
//      SerialBT.println("flipping I/O Low");
//    }
//  }
}
