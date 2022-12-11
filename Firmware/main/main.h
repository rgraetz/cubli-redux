#ifndef MAIN_H
#define MAIN_H

#include "BluetoothSerial.h"

#include "AITRIP_hal.h"
#include "AngleTransformation.h"
#include "common.h"
#include "ControlAlgo.h"
#include "DigitalEncoder.h"
#include "MPU_6050.h"

#define EN_PERIOD_MSEC 4000

#define INIT 0
#define IDLE 1
#define EDGE1 2
#define EDGE2 3
#define EDGE3 4
#define CORNER 5

#define EDGE_TOLERANCE 2.0
#define EDGE1_R 33.0   // 28.0   // M1 edge roll
#define EDGE1_P -19.75 //-15.55 // M1 edge pitch
#define EDGE2_R 2.75   // M2 edge roll
#define EDGE2_P 38.0   // M2 edge pitch
#define EDGE3_R -31.8  // M3 edge roll
#define EDGE3_P -18.0  // M3 edge pitch
#define CORNER_R 0.0   // corner roll
#define CORNER_P 0.0   // corner pitch

int edge1_t, edge2_t, edge3_t, corner_t;

int measurement_t,
    measurement_late;
int control_t, control_late;
int output_t, output_late;
int diagnostic_t, diagnostic_late;

int lastTime = 0;
int enTime = 0;

int startNow = 0;
int startLast = 0;

#endif