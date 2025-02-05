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

} ACU;

void acu_init();
void can_read();
void can_send();
void get_ts_current();

#endif