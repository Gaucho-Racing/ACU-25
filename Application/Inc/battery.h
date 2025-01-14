
#ifndef BATTERY_H
#define BATTERY_H

#include "config.h"
#include <stdint.h>

typedef struct {
    uint8_t device_count; //  num devices
    uint16_t cellCnt[BCC_DEVICE_CNT_MAX]; // # connected cells to each BCC. [0] BCC with CID=1, [1] BCC with CID=2, etc.

    // driver data
    uint16_t cellMap[BCC_DEVICE_CNT_MAX]; 
    uint8_t msgCntr[BCC_DEVICE_CNT_MAX + 1]; 
    uint8_t rxBuf[BCC_RX_BUF_SIZE_TPL];

} config;

typedef struct
{
    float cell_soc[140];
    float bal_temp[140]; 
    float cell_temp[140];
    float cell_volt[140];
    uint8_t cell_balancing[140];
    
    // stacks
    float stackVoltage[10];
    float icTemp[10];

    // stats
    float minCellVolt, maxCellVolt;
    float minChargeVolt, maxChargeVolt;

    // configuration
    config driver_config;
} battery;

#endif