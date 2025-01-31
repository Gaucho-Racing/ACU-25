#ifndef ACU_H
#define ACU_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "battery.h"

typedef struct {
    uint8_t id;
} Imd;

typedef struct {
    uint8_t id;
} Charger;

typedef struct {
    
    Battery * bty;
    Charger * chgr;
    Imd * imd;

    uint32_t fd_canBuff[8];      // connection to ECU

    uint32_t * tx_buff;       // full duplex master, NSS disabled ==> BCC
    uint32_t * rx_buff;       // full duplex slave, NSS enabled   ==> BCC
} ACU;

void acu_init();
void can_read();
void can_send();
void get_ts_current();

#endif