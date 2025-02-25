#ifndef ACU_H
#define ACU_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "battery.h"
#include "fdcan.h"
#include "adc.h"
#include "stm32g4xx_hal_fdcan.h"

typedef struct {
    uint8_t id;
} IMU;

typedef struct {
    uint8_t id;
    uint8_t chgr_errs; /* Map:  Bit 0: Hardware Failure: 0-Normal, 1-Error
                                Bit 1: OverTemp: 0-Normal, 1-Error
                                Bit 2: Input Voltage: 0-Normal, 1-Wrong input voltage
                                Bit 3: Starting State: 0-Correct, 1-Wrong polarity or NC
                                Bit 4: Communication State: 0-Normal, 1-Timeout
                        */
} Charger;

typedef struct {
    
    Battery * bty;
    Charger * chgr;
    IMU * imu;
    ADC * adc;

    // Charger stuff
    float volt_request;
    float temp_request;
    uint8_t chg_ctrl;

    // DC-DC Related Information
    float input_voltage; // ~20v for LV (LV only. Send 0 for HV)
    float ouput_voltage; // ~12v for LV and ~20v for HV
    float input_current; // Input current (LV only. Send 0 for HV)
    float ouput_current; // Output current
    float dc_dc_temp; // Temp of DC-DC converter => should be converted to u8 during can_send()

    // ACU General Stuff
    uint8_t relay_state;
    uint8_t acu_SOC; // Accumulator SOC	Accumulator state of charge (Based on lowest cell)
    uint8_t glv_SOC; // GLV SOC	GLV state of charge	 
    float glv_voltage; 
    float ts_voltage;  // output terminal voltage of ACU
    float ts_current; // current output of ACU
    float shutdown_volt; 
    float dcdc_current; 
    float temps[2]; // thermistor readings
    float fan_Ref;

    // ACU-Scoped data
    float hv_input_voltage; 
    float hv_output_voltage; 
    float hv_input_current; 
    float hv_output_current; 
    float cell_OT_Threshold;
    float cell_UT_Threshold;

    float cur_ref;
    float dcdc_ref;

    // voltages voltages voltages...
    float volt_20v; // 20v GLV voltage
    float volt_12v; // 12v supply voltage
    float volt_sdc; // voltage b4 ACU latch

    float max_output_current;
    float max_temp;

    // ACU errors/warnings
    uint8_t acuErrCount;
    uint8_t acu_errors[6];          /* Map: [OT, OV, UV, OC, UC, Precharge] */
    uint8_t acu_volt_warnings[3];   /* Map: [UV 20v, UV 12v, UV SDC] */

    // ACU states
    uint8_t ir_precharge_state; // 0: open, 1: closed
    uint8_t ir_state;           // 0: open, 1: closed
    uint8_t software_latch;     // 0: open, 1: closed

    // Cmmands send to ACU
    uint8_t ts_active; // 0: shutdown, 1: go TS Active/Precharge

    
} ACU;

void acu_init(ACU * acu);
void acu_check(ACU * acu, uint8_t state, bool startup);
bool can_polling(ACU * acu);
void can_read(ACU * acu, uint32_t id);
void can_send(ACU * acu, uint32_t id);
void can_dump(ACU *acu);



// modifiers
void set_max_charge_voltage(ACU * acu, float value);
void set_max_charge_current(ACU * acu, float value);
void set_max_output_current(ACU * acu, float value);
void set_max_temp(ACU * acu, float temp);
void set_ts_active(ACU * acu, uint8_t set_bit);

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

float get_total_voltage(ACU* acu);
#endif