#include "debug.h"
#include "state.h"

extern ACU acu;
extern Battery battery;
extern uint8_t cycle;
extern State state;

extern volatile uint8_t TPL_RxBuffer[256]; // Array to store received SPI data
extern volatile uint8_t TPL_RxBufferLevel; // Number of bytes to be read
extern volatile uint8_t TPL_RxBufferBottom; // Index of oldest data
extern volatile uint8_t TPL_RxBufferTop; // Index of newest data
extern void print_lpuart(char* arr);

#define TARG_VOLT 0
#define TARG_CURR 0
#define TARG_TEMP 0
#define CHARG_CTL 0
#define SHUTDOWN_VOLT 0

#define IN_VOLTAGE 0 
#define IN_CURRENT 0

#define OUT_VOLTAGE 0 
#define OUT_CURRENT 0

#define DC_DC_TEMP 0
#define DC_DC_CURR 0

#define ACU_SOC 0
#define GLV_SOC 0
#define GLV_VOLTAGE 0
#define TS_VOLTAGE 0
#define ACU_CURRENT 0

#define HV_INPUT_VOLT 0
#define HV_INPUT_CURR 0
#define HV_OUTPUT_VOLT 0
#define HV_OUTPUT_CURR 0

#define VOLT_SDC 0

#define CONFIG_MIN_CELL_VOLT 0
#define CONFIG_MAX_CELL_TEMP 0

#define PRINT_BATTERY_TEMP 0
#define PRINT_BATTERY_VOLTS 0

#define PRINT_BATTERY_STATUS 0
#define PRINT_BATTERY_FAULTS 0

#define ACU_ERROR_WARNS 0

#define PRINT_IMD_DATA 0
#define PRINT_ENERGY_MEASURE_DATA 0
#define PRINT_CHARGER_DATA 0

void debug(){
  print_lpuart("-----------------------Debug-----------------------\n");
  #if PRINT_IMD_DATA == 0
    char buffering[512];
    print_lpuart(buffering);
    bzero(buffering, sizeof(buffering));

  #endif
  print_lpuart("-------------------- End Debug --------------------\n");
}
uint8_t spi_send_string(const uint8_t *data, uint16_t length) {
  uint32_t counter = 0;
  BCC_MCU_WriteCsbPin(0, 0); // CS LOW
  BCC_MCU_WaitUs(2); // delay required by MC33664
  while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
    if(counter++ > SPI_LOOP_TIMEOUT) return 1;
    BCC_MCU_WaitUs(1);
  }
  for (uint16_t i = 0; i < length; i++) {
    LL_SPI_TransmitData8(SPI1, data[i]);
    BCC_MCU_WaitUs(3); // don't know why but seems we need this
  }
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  BCC_MCU_WaitUs(1); // delay required by MC33664
  BCC_MCU_WriteCsbPin(0, 1); // CS HIGH
  return 0;
}
uint8_t spi_read_string(uint8_t *buffer, uint16_t length){
  for (uint16_t i = 0; i < length; i++) {
    uint32_t counter = 0;
    while (TPL_RxBufferLevel == 0) {
      if(counter++ > SPI_LOOP_TIMEOUT) {
        return 1;
      }
      BCC_MCU_WaitUs(1);
    }
    buffer[i] = TPL_RxBuffer[TPL_RxBufferBottom];
    TPL_RxBufferBottom++;
    TPL_RxBufferLevel--;
  }
  return 0;
}

