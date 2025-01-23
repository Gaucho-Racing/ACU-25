
#ifndef BATTERY_H
#define BATTERY_H

#include "config.h"

typedef enum {
    OV, UV, OT, UT
} Battery_Status;

typedef struct
{
    float cell_soc[NUM_CELL_IC*NUM_TOTAL_IC];
    float bal_temp[NUM_CELL_IC*NUM_TOTAL_IC]; 
    float cell_temp[NUM_CELL_IC*NUM_TOTAL_IC];
    float cell_volt[NUM_CELL_IC*NUM_TOTAL_IC];

    uint8_t cell_balancing[NUM_CELL_IC*NUM_TOTAL_IC];
    
    // stacks
    float stackVoltage[10];
    float icTemp[10];

    // stats
    float minCellVolt, maxCellVolt;
    float minChargeVolt, maxChargeVolt;

} Battery;


void read_battery();
void send_battery();
Battery_Status battery_status();
Battery_Status cell_balancing();

#endif