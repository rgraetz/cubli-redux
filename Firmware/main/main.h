#ifndef MAIN_H
#define MAIN_H

#include "BluetoothSerial.h"

#include "AITRIP_hal.h"
#include "AngleTransformation.h"
#include "common.h"
#include "ControlAlgo.h"
#include "DigitalEncoder.h"
#include "MPU_6050.h"

#define UPDATE_PERIOD 10
#define EN_PERIOD 4000

int lastTime = 0;
int enTime = 0;

int startNow = 0;
int startLast = 0;

#endif