
#ifndef BATTERY_H
#define BATTERY_H

#include "config.h"
#include "stm32g474xx.h"
#include "bcc_communication.h"
#include "string.h"
#include "debug.h"

typedef struct
{
    const uint8_t address;
    const uint16_t defaultVal;
    const float value;
} bcc_init_reg_t;

typedef struct
{
    float cell_soc[NUM_CELL_IC*REAL_NUM_TOTAL_IC];
    float bal_temp[NUM_CELL_IC*REAL_NUM_TOTAL_IC]; 
    float cell_temp[NUM_CELL_IC*REAL_NUM_TOTAL_IC];
    float cell_volt[NUM_CELL_IC*REAL_NUM_TOTAL_IC];
    uint8_t cell_balancing[NUM_CELL_IC*REAL_NUM_TOTAL_IC]; // 0 = off, 255 = on, error = anything else
    
    // stacks
    float stackVoltage[NUM_TOTAL_IC];
    float icTemp[NUM_TOTAL_IC];

    // CAN data stuff
    float min_cell_volt;
    float max_cell_volt;
    float min_cell_temp;
    float max_cell_temp;

    float min_charge_volt;
    float max_charge_volt;

    float batVoltage, batSOC;
    float max_chg_current;

    // config
    bcc_drv_config_t drvConfig;
    bcc_fault_status_t faults;
    bcc_fault_status_t cid_faults[NUM_TOTAL_IC];

} Battery;

static const bcc_init_reg_t init_regs[INIT_REG_CNT] = {
    {MC33771C_GPIO_CFG1_OFFSET, MC33771C_GPIO_CFG1_POR_VAL, GPIO_CFG1},
    {MC33771C_GPIO_CFG2_OFFSET, MC33771C_GPIO_CFG2_POR_VAL, GPIO_CFG2},
    {MC33771C_TH_ALL_CT_OFFSET, MC33771C_TH_ALL_CT_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT14_OFFSET, MC33771C_TH_CT14_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT13_OFFSET, MC33771C_TH_CT13_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT12_OFFSET, MC33771C_TH_CT12_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT11_OFFSET, MC33771C_TH_CT11_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT10_OFFSET, MC33771C_TH_CT10_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT9_OFFSET, MC33771C_TH_CT9_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT8_OFFSET, MC33771C_TH_CT8_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT7_OFFSET, MC33771C_TH_CT7_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT6_OFFSET, MC33771C_TH_CT6_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT5_OFFSET, MC33771C_TH_CT5_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT4_OFFSET, MC33771C_TH_CT4_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT3_OFFSET, MC33771C_TH_CT3_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT2_OFFSET, MC33771C_TH_CT2_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_CT1_OFFSET, MC33771C_TH_CT1_POR_VAL, CELL_MAX_VOLT},
    {MC33771C_TH_AN6_OT_OFFSET, MC33771C_TH_AN6_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN5_OT_OFFSET, MC33771C_TH_AN5_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN4_OT_OFFSET, MC33771C_TH_AN4_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN3_OT_OFFSET, MC33771C_TH_AN3_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN2_OT_OFFSET, MC33771C_TH_AN2_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN1_OT_OFFSET, MC33771C_TH_AN1_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN0_OT_OFFSET, MC33771C_TH_AN0_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN6_UT_OFFSET, MC33771C_TH_AN6_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN5_UT_OFFSET, MC33771C_TH_AN5_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN4_UT_OFFSET, MC33771C_TH_AN4_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN3_UT_OFFSET, MC33771C_TH_AN3_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN2_UT_OFFSET, MC33771C_TH_AN2_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN1_UT_OFFSET, MC33771C_TH_AN1_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN0_UT_OFFSET, MC33771C_TH_AN0_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_CB1_CFG_OFFSET, MC33771C_CB1_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB2_CFG_OFFSET, MC33771C_CB2_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB3_CFG_OFFSET, MC33771C_CB3_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB4_CFG_OFFSET, MC33771C_CB4_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB5_CFG_OFFSET, MC33771C_CB5_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB6_CFG_OFFSET, MC33771C_CB6_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB7_CFG_OFFSET, MC33771C_CB7_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB8_CFG_OFFSET, MC33771C_CB8_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB9_CFG_OFFSET, MC33771C_CB9_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB10_CFG_OFFSET, MC33771C_CB10_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB11_CFG_OFFSET, MC33771C_CB11_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB12_CFG_OFFSET, MC33771C_CB12_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB13_CFG_OFFSET, MC33771C_CB13_CFG_POR_VAL, CBX_SET},
    {MC33771C_CB14_CFG_OFFSET, MC33771C_CB14_CFG_POR_VAL, CBX_SET}
};

bcc_status_t init_registers(Battery * bty);
bcc_status_t read_device_measurements(Battery * bty, uint8_t read_volt, uint8_t read_temp);
bcc_status_t config_cell_balancing(Battery * bty, bcc_cid_t cid, uint8_t cellIndex, bool all, bool enable);
bcc_status_t check_volt(Battery *bty);
bcc_status_t check_temp(Battery *bty);
bcc_status_t check_fuse(Battery *bty);

void battery_check(Battery *bty, bool full_check);
bool check_faults(Battery *bty);
void clear_faults(bcc_drv_config_t * drvConfig);
bool do_cell_balancing(Battery * bty, bool all);
void update_cell_voltages(Battery * bty);

// print individuals
void print_volt(const float * voltages, const uint8_t cid, uint8_t index);
void print_temp(const float * temperatures, const uint8_t cid, uint8_t index);

// print group
void print_cell_balancing(Battery *bty);
void print_temperature(Battery * bty);
void reset_discharge(Battery * bty);
void print_voltage(Battery *bty);

#endif