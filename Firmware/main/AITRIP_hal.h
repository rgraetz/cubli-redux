#ifndef HAL_H
#define HAL_H

// GPIO DEFINITIONS
// motor direction outputs
#define M1_DIR_GPIO 32
#define M2_DIR_GPIO 25
#define M3_DIR_GPIO 27

// motor PWM speed control outputs
#define M1_PWM_GPIO 33
#define M2_PWM_GPIO 26
#define M3_PWM_GPIO 14

// motor encoder inputs
#define M1_CHA_GPIO 5
#define M1_CHB_GPIO 17

#define M2_CHA_GPIO 16
#define M2_CHB_GPIO 4

#define M3_CHA_GPIO 2
#define M3_CHB_GPIO 15

// brake
#define EMO_GPIO 13

// alarm
#define ALARM_GPIO 18

// MPU addr
#define MPU_ADDR 0x68

// buzzer/alarm
#define BUZZ_GPIO 18

// start switch
#define START_GPIO 19

// battery voltage measurement
#define BATT_AIN 34

//
#define CONTROL_RATE 100

#endif
