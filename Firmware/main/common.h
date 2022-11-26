#ifndef COMMON_H
#define COMMON_H

//
#define MOTOR_NUM 3

// ENCODER POSITION / STATE
#define CURRENT 1
#define PREVIOUS 0
#define VEL_PERIOD 1000

// int M1_POS = 0;
// int M2_POS = 0;
// int M3_POS = 0;

// int M1_VEL = 0;
// int M2_VEL = 0;
// int M3_VEL = 0;

// int M1_ENC_STATE = 0;
// int M2_ENC_STATE = 0;
// int M3_ENC_STATE = 0;

long currentT, previousT = 0;
int pos_time = 250;

#endif