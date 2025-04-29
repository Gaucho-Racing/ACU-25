#ifndef ACU_H
#define ACU_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "battery.h"
#include "fdcan.h"
#include "adc.h"
#include "math.h"
#include "stm32g4xx_hal_fdcan.h"

typedef struct {
    uint8_t data[64];
    FDCAN_GlobalTypeDef* instance; // Options: FDCAN1, FDCAN2, FDCAN3
    uint32_t identifier; // CANID
    uint32_t length; // Bytes
} CAN_RX_message;

typedef struct {
    float em_current;
    float em_voltage;
    float energy;

    uint8_t min_temp;
    uint8_t max_temp;
    uint8_t num_sensors;

    uint8_t status; /* Map: 0: violation, 1: logging*/
    float temps[32];

} EM;

typedef struct {
    uint8_t id;

    float hv_system_voltage;

    uint8_t r_iso_status;
    uint8_t r_iso_meas_count; 
    uint8_t isolation_quality;
    uint8_t status_device_activity; // 0 - initialization, 1 - Normal Operation, 2 - Self Test 
    uint16_t r_iso_corrected;
    uint16_t status_warnings_alarms; // 11 bits long

    uint16_t r_iso_negative;
    uint16_t r_iso_positive;
    uint16_t r_iso_original;
    uint8_t iso_meas_count;
    
} IMD;

typedef struct {

    // Charger Data (ACU reads from RX)
    uint8_t chgr_status; /* Map:    Bit 0: Hardware Failure: 0-Normal, 1-Error
                                    Bit 1: OverTemp: 0-Normal, 1-Error
                                    Bit 2: Input Voltage: 0-Normal, 1-Wrong input voltage
                                    Bit 3: Starting State: 0-Correct, 1-Wrong polarity or NC
                                    Bit 4: Communication State: 0-Normal, 1-Timeout
                        */
    uint16_t charger_output_voltage; // data from charger (via Rx)
    uint16_t charger_output_current; // data from charger (via Rx)
} Charger;

typedef struct {
    
    Battery * bty;
    Charger * chgr;
    IMD * imd;
    EM * em;

    // Config Charge Parameters (ACU to sends via TX to Charger)
    float target_voltage; 
    float target_current;

    uint8_t chg_ctrl; // 0b01 : Start Charging, 0b10: Stop Charging => to be sent from ACU to others via Tx

    // ACU_Status_1
    uint8_t acu_SOC; // Accumulator state of charge (Based on lowest cell)
    uint8_t glv_SOC; // GLV state of charge

    float acu_current; // current output of ACU

    // ADC Stuff
    // 0:ts_current, 1:ts_voltage, 2:sdc_volt_w, 
    // 3:sdc_volt_v, 4:voltage_12v,5:water_sense
    float ts_voltage;  // ts_v
    float ts_current;  // uc_ts_i
    float sdc_volt_w;  // sdcw_w => BEFORE ACU latch
    float sdc_volt_v;  // sdcv_v => AFTER ACU latch
    float voltage_12v; // glv voltage
    float water_sense; // uc_water

    float glv_voltage;  // from adc_data = JUST SET TO 0?
    
    // ACU-Status 3
    float hv_input_voltage;  // 600v input voltage => Apparantly not needed anymore
    float hv_output_voltage; // 20v output voltage => Apparantly not needed anymore
    float hv_input_current;  // 600v input current => Apparantly not needed anymore
    float hv_output_current;  // 20v output current => Apparantly not needed anymore

    // from GR24 => not sure if we still need to use
    uint32_t cur_LastHighTime;
    uint32_t lastChrgRecieveTime;

    // 0: AIR+ | 1: AIR- | 2: Precharge
    uint8_t relay_state; 
    uint8_t acu_latch;


    // ACU errors/warnings
    uint8_t acuErrCount;
    uint16_t acu_err_warns; // [ 0:OT, OV, UV, OC, UC, UV_20v, UV_GLV, 7:UV_SDC, 8:Precharge, 0, 0, 0, 0, 0, 0, 0]        

    // ACU states => THIS IS NOT USED
    // uint8_t ir_precharge_state; // 0: open, 1: closed
    // uint8_t ir_state;           // 0: open, 1: closed
    // uint8_t software_latch;     // 0: open, 1: closed

    // ACU Precharge via Tx
    uint8_t ts_active; // 0: shutdown, 1: go TS Active/Precharge
} ACU;

void acu_init(ACU * acu);
bool acu_check(ACU * acu, uint8_t state, bool startup);

// Send CAN Messages

void dequeue(ACU* acu);
void enqueue(uint32_t id, FDCAN_GlobalTypeDef * which_can);

// Receive CAN Messages

void can_read_handler(ACU* acu);
void can_read(ACU * acu, FDCAN_GlobalTypeDef * which_can, uint32_t id, uint32_t size, uint8_t * data);

// Other Messages
void can_dump(ACU *acu);

void reset_latch(ACU *acu);
void update_adc_data(ACU* acu);
void update_all(ACU * acu);

float get_total_voltage(ACU* acu);

// helpers
uint8_t fconstrain(float value);

// TO IMPLEMENT
void charger_check(ACU* acu); // check chgr_status
uint8_t calculate_acu_soc(ACU* acu); // sets acu_SOC based on lowest cell voltage
uint8_t calculate_glv_soc(ACU* acu); // sets acu_GLV based on ???
#endif