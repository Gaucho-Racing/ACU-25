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
    uint8_t data[64];
    FDCAN_RxHeaderTypeDef* RxHeader;
    FDCAN_HandleTypeDef* hfdcan;
} CAN_RX_message;

typedef struct {
    uint8_t data[64];
    uint32_t identifier;
    uint32_t data_length;
    FDCAN_HandleTypeDef* hfdcan;
} CAN_TX_message;

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
    uint8_t status_device_activity; //0 - initialization, 1 - Normal Operation, 2 - Self Test 
    uint16_t r_iso_corrected;
    uint16_t status_warnings_alarms; // 11 bits long

    uint16_t r_iso_negative;
    uint16_t r_iso_positive;
    uint16_t r_iso_original;
    uint8_t iso_meas_count;
    
} IMD;

typedef struct {
    uint8_t id;

    // Charger Data (ACU reads from RX)
    uint8_t chgr_status; /* Map:    Bit 0: Hardware Failure: 0-Normal, 1-Error
                                    Bit 1: OverTemp: 0-Normal, 1-Error
                                    Bit 2: Input Voltage: 0-Normal, 1-Wrong input voltage
                                    Bit 3: Starting State: 0-Correct, 1-Wrong polarity or NC
                                    Bit 4: Communication State: 0-Normal, 1-Timeout
                        */
    uint16_t charger_output_voltage;
    uint16_t charger_output_current;
} Charger;

typedef struct {
    
    Battery * bty;
    Charger * chgr;
    IMD * imd;
    EM * em;

    // Config Charge Parameters (ACU to sends via TX to Charger)
    float target_voltage;
    float target_current;
    float target_temp;

    uint8_t chg_ctrl; // 1 : Start Charging, 0: Stop Charging

    // ACU_Status_1
    uint8_t acu_SOC; // % charged of the Accumulator (Based on lowest cell)
    uint8_t glv_SOC; // % charged of the Low Voltage Bat

    float ts_voltage;  // output terminal voltage of ACU
    float ts_current; // current output of ACU

    // not used for CAN
    float shutdown_volt; // preset voltage threshold
    
    // ACU-Status 3
    float hv_input_voltage;  // 600v input voltage
    float hv_output_voltage; // 20v output voltage
    float hv_input_current;  // 600v input current
    float hv_output_current;  // 20v output current

    // voltages voltages voltages...
    float sdc_voltage; // voltage b4 ACU latch
    float glv_voltage;  // from adc_data

    // Config_Operational_ACU
    float config_min_cell_volt;
    float config_max_cell_temp;

    // from GR24 => not sure if we still need to use
    uint32_t cur_LastHighTime;
    uint32_t lastChrgRecieveTime;
    uint8_t relay_state; /* Bit 0: AIR+ State (1=closed)
                            Bit 1: AIR- State (1=closed)
                            Bit 2: Precharging (1=in progress, useless)
                            Bit 3: nothing
                            Bit 4: Shutdown (1 = is shutdown)
                            Bit 5: LatchNotClosed
                            */


    // ACU errors/warnings
    uint8_t acuErrCount;
    uint16_t acu_err_warns; /*[ OT, OV, UV, OC, 
                                UC, UV_20v, UV_GLV, UV_SDC, 
                                Precharge, 0, 0, 0, 
                                0, 0, 0, 0]*/         

    // ACU states
    uint8_t ir_precharge_state; // 0: open, 1: closed
    uint8_t ir_state;           // 0: open, 1: closed
    uint8_t software_latch;     // 0: open, 1: closed

    // Cmmands send to ACU
    uint8_t ts_active; // 0: shutdown, 1: go TS Active/Precharge
} ACU;

void acu_init(ACU * acu);
void acu_check(ACU * acu, uint8_t state, bool startup);
void can_read_all(ACU* acu);
void can_read(ACU * acu, uint32_t id, uint8_t * data);
void can_send(ACU * acu, uint32_t id);
void can_dump(ACU *acu);
void reset_latch(ACU *acu);

// modifiers
void update_adc_array_data(ACU* acu);
void update_all(ACU * acu);

float get_total_voltage(ACU* acu);
float V2T_f(float voltage, float B); // default = 4390
float V2T_i(int16_t voltage, float B); // default = 4390
float V2T_complex(float vdd, float voltage, float B, float Rsns, float Rref);
#endif