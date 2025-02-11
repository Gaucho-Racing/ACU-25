#ifndef ACU_H
#define ACU_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "battery.h"

typedef struct {
    uint8_t id;
} IMU;

typedef struct {
    uint8_t id;
} Charger;

typedef struct {
    
    Battery * bty;
    Charger * chgr;
    IMU * imu;

    uint8_t relay_state;
    float glv_voltage; 
    float ts_voltage; 
    float ts_current;
    float shutdown_volt; 
    float dcdc_current; 
    float temps[2]; // thermistor readings
    float fan_Ref;

    float max_output_current;
    float max_temp;

    float cell_OT_Threshold, cell_UT_Threshold;

} ACU;

// extern Battery battery;
// extern CAN_message_t msg;
// extern States state;

// void mailboxSetup();
// void sendCANData(uint32_t ID);

void acu_init(ACU * acu);

void check_acu(ACU * acu);
void can_read(ACU * acu);
void can_send(ACU * acu);
// void can_parse(ACU * acu);
// void can_dump(ACU *acu);


// modifiers
void set_max_charge_voltage(ACU * acu, float current);
void set_max_charge_current(ACU * acu, float current);
void set_max_output_current(ACU * acu, float current);
void set_max_temp(ACU * acu, float temp);

void update_shitdown_voltage(ACU * acu);
void update_glv_voltage(ACU * acu);
void update_ts_voltage(ACU * acu);
void update_ts_current(ACU * acu);
void update_dc_dc_current(ACU * acu);

void update_temp_one(ACU * acu);
void update_temp_two(ACU * acu);
void update_temp_three(ACU * acu);

void update_fan_ref(ACU * acu);
void update_relay_state(ACU * acu);

void update_all(ACU * acu);

float V2T_f(float voltage, float B); // default = 4390
float V2T_i(int16_t voltage, float B); // default = 4390
float V2T_complex(float vdd, float voltage, float B, float Rsns, float Rref);

#endif