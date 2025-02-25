#ifndef CONFIG_H
#define CONFIG_H

#define NUM_TOTAL_IC 1
#define REAL_NUM_TOTAL_IC 10 // only used in defining struct so no mem leaks LOL
#define NUM_CELL_IC 14

#define INIT_REG_CNT 45
#define GPIO_CFG1 0.0
#define GPIO_CFG2 0.0
#define CBX_SET 1.0
#define RX_BUF_SIZE 8

#define SPI_LOOP_TIMEOUT 500

#define KEY_PRESSED     GPIO_PIN_RESET
#define NOT_PRESSED     GPIO_PIN_SET
    
#define CELL_MAX_VOLT 4.2f
#define CELL_MIN_VOLT 0.9f

#define CELL_MIN_TEMP 0.0 // to set later => when printing multiply by 0.1 to get Celcius
#define CELL_MAX_TEMP 1000.0 // to set later => when printing multiply by 0.1 to get Celcius

#define MIN_BALL_TEMP 0.0 // to set later
#define MAX_BALL_TEMP 1000.0 // to set later

//acu limits
#define MAX_HV_CURRENT 135
#define MAX_DCDC_TEMP 60
#define MAX_DCDC_CURRENT 10
#define MIN_GLV_VOLT 10
#define MAX_GLV_VOLT 14.4

#define MAX_READ_COUNT 8
#define TRIES 5 // defines how many times we can retry an action

// error margins
#define ERRMG_GLV_SDC 4
#define ERRMG_ISNS_VREF 0.2
#define ERRMG_5V 0.2
#define ERRMG_CELL_VOLT_ERR 20
#define ERRMG_CELL_TEMP_ERR 50
#define ERRMG_ACU_ERR 50

// ADC1283 connections
#define ADC_MUX_GLV_VOLT 7
#define ADC_MUX_HV_VOLT 6
#define ADC_MUX_HV_CURRENT 0
#define ADC_MUX_SHDN_POW 1
#define ADC_MUX_DCDC_CURRENT 5
#define ADC_MUX_TEMP1 2
#define ADC_MUX_TEMP2 3
#define ADC_MUX_FAN_REF 4

#define PRECHARGE_THRESHOLD 0.96 // fraction of total cell voltage

#define ACU_DEBUG 0

// Send
#define ACU_Debug_2             0x300001
#define ACU_Debug_FD            0x300101
#define ACU_Ping_Debug          0x300201
#define ACU_Ping_ECU            0x300202
#define ACU_Status_1            0x300702
#define ACU_Status_2            0x300802
#define ACU_Status_3            0x300902
#define ACU_Cell_Data_1         0x300DFF
#define ACU_Cell_Data_2         0x300EFF
#define ACU_Cell_Data_3         0x300FFF
#define ACU_Cell_Data_4         0x3010FF
#define ACU_Cell_Data_5         0x3011FF
#define ACU_DC_DC_Status        0x301202
#define ACU_Charger_Control     0x1806E5F4

// Receive
#define Charger_Data_ACU        0x18FF50E5
#define Debug_2_ACU             0x100003
#define Debug_FD_ACU            0x100103
#define Ping_ACU                0x100203
#define Precharge_ACU           0x200A03
#define Config_Charge_ACU       0x200B03
#define Config_Operational_ACU  0x200C03
#define EM_Measurements_ACU     0x10D
#define EM_Data_1_ACU           0x30D
#define EM_Data_2_ACU           0x30E
#define EM_Status_ACU           0x40D
#define EM_Temperature_ACU      0x60D
#define IMD_Response_ACU        0x23
#define IMD_Isolation_ACU       0x18EFF4FE
#define IMD_Voltage_ACU         0x18EFF4FE
#define IMD_IT_System_ACU       0x18EFF4FE
#define IMD_Request_ACU         0x18EFF4FE
#define IMD_General_ACU         0x18FF01F4

#endif