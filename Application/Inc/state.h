#ifndef STATE_H
#define STATE_H

#include "acu.h"

// Typedef
typedef enum {
    STANDBY,
    PRECHARGE,
    CHARGE,
    NORMAL, 
    SHITDOWN
} State;

// State Functions
void shitdown();
void standby();
void precharge();
void charge();
void normal();
bool state_system_check(bool full_check, bool startup);

// Other Functions
void check_charge();

// Externs
extern ACU acu;
extern Battery battery;
extern uint16_t bcc_faults;
extern State state;
#endif