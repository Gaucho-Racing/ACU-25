#ifndef ACU_H
#define ACU_H
#include "battery.h"
#include "state.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct {
    uint8_t id;
} Imu;

typedef struct {
    uint8_t id;
} Charger;

typedef struct {
    
    Battery * bty;
    Charger * chgr;
    Imu * imu;

    uint8_t fd_canBuff[8];      // connection to ECU

    uint8_t tx_buff_1[8];       // full duplex master, NSS disabled ==> Battery
    uint8_t rx_buff_1[8];       // full duplex master, NSS disabled ==> Battery

    uint8_t tx_buff_2[8];       // full duplex slave, NSS enabled   ==> Charger
    uint8_t rx_buff_2[8];       // full duplex slave, NSS enabled   ==> Charger
} ACU;

void init();

void can_read();
void can_send();

#endif