#ifndef STATE_H
#define STATE_H

typedef enum {
    STANDBY,
    PRECHARGE,
    CHARGE,
    NORMAL, 
    SHITDOWN
} State;

void shitdown();
void standby();
void precharge();
void charge();
void normal();
void system_check();
void check_charge();

#endif