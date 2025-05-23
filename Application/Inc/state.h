#ifndef STATE_H
#define STATE_H

// Typedef
typedef enum {
    INIT,       // 🙏
    STANDBY,    // 🏠
    PRECHARGE,  // 🙏
    CHARGE,     // 🛌
    NORMAL,     // 💃
    SHITDOWN    // 🪦
} State;

// State Functions
void init();
void shitdown();
void standby();
void precharge();
void charge();
void normal();

// State main function
bool state_system_check(bool full_check, bool startup);
float constrain(float value, float lowerBound, float upperBound);

// State Mods & Accs
void print_state();
uint8_t get_state();
void set_state(uint8_t value);
#endif