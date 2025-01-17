#ifndef CONFIG_H
#define CONFIG_H

#define NUM_TOTAL_IC 1
#define NUM_CELL_IC 14

#define INIT_REG_CNT 45
#define GPIO_CFG1 0.0
#define GPIO_CFG2 0.0
#define CBX_SET 1.0
#define RX_BUF_SIZE 8

#define KEY_PRESSED     GPIO_PIN_RESET
#define NOT_PRESSED     GPIO_PIN_SET

#define BCC_GET_TH_CTX(threshold) \
    (uint16_t)(((((threshold) * 10U) / 195U) > 0xFF) ? \
    0xFF : (((threshold) * 10U) / 195U))
    
#define CELL_MAX_VOLT 4.2f
#define CELL_MIN_VOLT 0.9f
#define PRM_CELL_MAX_VOLT BCC_GET_TH_CTX((uint32_t)(CELL_MAX_VOLT * 1000))
#define PRM_CELL_MIN_VOLT BCC_GET_TH_CTX((uint32_t)(CELL_MIN_VOLT * 1000))

#define CELL_MIN_TEMP 0.0 // to set later => when printing multiply by 0.1 to get Celcius
#define CELL_MAX_TEMP 1000.0 // to set later => when printing multiply by 0.1 to get Celcius

#define MIN_BALL_TEMP 0.0 // to set later
#define MAX_BALL_TEMP 1000.0 // to set later

#define D_PRINT(x) printf(x) // DEBUG MODE enabled
#define PRINT(x) Serial.printf(x)

#define BCC_DEVICE_CNT_MAX 63U
#define BCC_RX_BUF_SIZE_TPL (6U * (0x7FU + 1U))

#endif