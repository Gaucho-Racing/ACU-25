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

//acu limits
#define MAX_HV_CURRENT 135
#define MIN_GLV_VOLT 10
#define MAX_GLV_VOLT 14.4

#define MAX_READ_COUNT 8
#define TRIES 5 // defines how many times we can retry an action

// error masks
#define ACU_ERR_OVER_TEMP   0b1000000000000000
#define ACU_ERR_OVER_VOLT   0b0100000000000000
#define ACU_ERR_UNDER_VOLT  0b0010000000000000
#define ACU_ERR_OVER_CURR   0b0001000000000000
#define ACU_ERR_UV_20_V     0b0000100000000000 // warning
#define ACU_ERR_UV_12_V     0b0000010000000000 // warning
#define ACU_ERR_UV_SDC      0b0000001000000000 // warning
#define ACU_PRECHARGE       0b0000000100000000
#define ACU_CLEAR_WARN      0b0000111000000000
#define ACU_CLEAR_ERRR      0b1111000100000000

// charger error masks
#define CHARGER_HW_FAIL     0b10000
#define CHARGER_OV_TEMP     0b01000
#define CHARGER_IN_VOLT     0b00100
#define CHARGER_CONNECT     0b00010
#define CHARGER_COOMMMM     0b00001

// precharge cmd
#define PLS_CHARGE 0b01000000
#define NO_CHARGE  0b10000000

// relay state muxes
#define AIR_PLUS   0b10000000 // IR- : 0: Open, 1: Closed
#define AIR_MINUS  0b01000000 // IR+ : 0: Open, 1: Closed
#define RELAY_PRE  0b00100000 // Precharge 0: Open, 1: Closed

// IMD error masks
#define IMD_ERROR_ACTIVE    0b10000000000
#define HV_POS_CONN_FAIL    0b01000000000
#define HV_NEG_CONN_FAIL    0b00100000000
#define EARTH_CONNN_FAIL    0b00010000000
#define ISO_ALARM_ERRROR    0b00001000000
#define ISO_WARN_ERRRROR    0b00000100000
#define ISO_OUTDATED_ERR    0b00000010000
#define UN_BALANCE_ALARM    0b00000001000
#define UNDERVOLTG_ALARM    0b00000000100
#define UNSAFE_TOO_START    0b00000000010
#define EARTH_LIFT_OPENN    0b00000000001

// error margins
#define GLV_SDC_LOW 1.0f
#define SDC_HIGH 9.0f
// #define ERRMG_5V 0.2
// #define ERRMG_CELL_VOLT_ERR 20
// #define ERRMG_CELL_TEMP_ERR 50
#define ERRMG_ACU_ERR 50

// ADC Warning Thresholds
#define UNDER_VOLTAGE_20V 15
#define UNDER_VOLTAGE_GLV 10
#define UNDER_VOLTAGE_SDCV 9

#define PRECHARGE_THRESHOLD 0.96 // fraction of total cell voltage
#define SAFE_V_TO_TURN_OFF 60

/* Send ***********************************************************/
// CAN1
#define ACU_Debug_2_Debug       0x300001    // FLAG = 1 (CAN1)
#define ACU_Ping_Debug          0x300201    // FLAG = 2 (CAN1/CAN2)
#define ACU_Ping_ECU            0x300202    // FLAG = 3 (CAN1/CAN2)

#define ACU_Status_1            0x300702    // FLAG = 5
#define ACU_Status_2            0x300802    // FLAG = 6
#define ACU_Status_3            0x300902    // FLAG = 7
#define ACU_DC_DC_Status        0x301202    // DEPRECATED

// CAN2
// ACU_Ping_Debug               0x300201    // FLAG = 1 (CAN1/CAN2)
// ACU_Ping_ECU                 0x300202    // FLAG = 2 (CAN1/CAN2)
#define ACU_Debug_FD            0x300101    // FLAG = 3 (CAN2)
#define ACU_Cell_Data_1         0x300DFF    // FLAG = 4 (CAN2)
#define ACU_Cell_Data_2         0x300EFF    // FLAG = 5 (CAN2)
#define ACU_Cell_Data_3         0x300FFF    // FLAG = 6 (CAN2)
#define ACU_Cell_Data_4         0x3010FF    // FLAG = 7 (CAN2)
#define ACU_Cell_Data_5         0x3011FF    // FLAG = 8 (CAN2)

// CAN3
#define ACU_Charger_Control     0x1806E5F4  // FLAG = 1

/* Receive ********************************************************/
// CAN1
#define Debug_2_ACU             0x100003    // Debugger sends Debug 2.0 to ACU (CAN1)
#define Debug_Ping_ACU          0x100203    // Debugger sends PING to ACU (CAN1/CAN2)
#define Precharge_ACU           0x200A03    // ECU sends PRECHARGE to ACU (CAN1)
#define Config_Charge_ACU       0x200B03    // ECU sends Charger Config to ACU (CAN1)
#define Config_Ops_ACU          0x200C03    // ECU sends Operation Config to ACU (CAN1)

// CAN2
#define Debug_FD_ACU            0x100103    // Debugger sends Debug FD to ACU (CAN2)
#define ECU_Ping_ALL            0x2002FF    // ECU sends PING to ALL (CAN1/CAN2)

// CAN3
#define Charger_Data_ACU        0x18FF50E5  // Charger sends Charger Data to ACU (CAN3)

#define EM_Measurements_ACU     0x10D       // EM sends EM Measurements to ACU (CAN3)
#define EM_Data_1_ACU           0x30D       // EM sends EM Team Data 1 to ACU (CAN3)
#define EM_Data_2_ACU           0x30E       // EM sends EM Team Data 2 to ACU (CAN3)
#define EM_Status_ACU           0x40D       // EM sends EM Status to ACU (CAN3)
#define EM_Temperature_ACU      0x60D       // EM sends EM Temperature to ACU (CAN3)

#define IMD_Response_ACU        0x23        // IMD sends IMD response to ACU (CAN3)
#define IMD_Isolation_ACU       0x18FF02F4  // IMD sends IMD iso info to ACU (CAN3) (prev 0x18EFF4FE)
#define IMD_Voltage_ACU         0x18FF03F4  // IMD sends IMD voltage to ACU (CAN3) (prev 0x18EFF4FE)
#define IMD_IT_System_ACU       0x18FF04F4  // IMD sends IMD it-system to ACU (CAN3) (prev 0x18EFF4FE)
#define IMD_Request_ACU         0x18EFF4FE  // IMD sends IMD request to ACU (CAN3) (Should be 1CEF00F4)
#define IMD_General_ACU         0x18FF01F4  // IMD sends IMD general to ACU (CAN3)

#endif