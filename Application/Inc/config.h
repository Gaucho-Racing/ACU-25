#ifndef CONFIG_H
#define CONFIG_H

#define NUM_TOTAL_IC 1
#define NUM_CELL_IC 14

#define INIT_REG_CNT 45
#define GPIO_CFG1 0.0
#define GPIO_CFG2 0.0
#define CBX_SET 1.0
#define RX_BUF_SIZE 8

#define PIN_BCC_TX_CS 0
#define PIN_BCC_TX_RST -1 // likely not used
#define PIN_BCC_EN 6
#define PIN_BCC_INT 9

#define PIN_BCC_TX_CS 0
#define PIN_BCC_TX_RST -1 // likely not used
#define PIN_BCC_EN 6
#define PIN_BCC_INT 9

#define SPI_LOOP_TIMEOUT 500

#define KEY_PRESSED     GPIO_PIN_RESET
#define NOT_PRESSED     GPIO_PIN_SET
    
#define CELL_MAX_VOLT 4.2f
#define CELL_MIN_VOLT 0.9f

#define CELL_MIN_TEMP 0.0 // to set later => when printing multiply by 0.1 to get Celcius
#define CELL_MAX_TEMP 1000.0 // to set later => when printing multiply by 0.1 to get Celcius

#define MIN_BALL_TEMP 0.0 // to set later
#define MAX_BALL_TEMP 1000.0 // to set later

#define TRIES 5 // defines how many times we can retry an action

typedef enum {
    VOLTAGE, 
    TEMPERATURE,
    BALL_TEMP,
    SOC
} bcc_measurements;

#endif