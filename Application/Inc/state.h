#ifndef STATE_H
#define STATE_H
typedef enum {
    SHUTDOWN, 
    CHARGE, 
    PRE_CHARGE,
    NORMAL, 
    STANDBY
} State;
#endif