#include "debug.h"

extern void print_lpuart(char* arr);

extern volatile uint8_t TPL_RxBuffer[256]; // Array to store received SPI data
extern volatile uint8_t TPL_RxBufferLevel; // Number of bytes to be read
extern volatile uint8_t TPL_RxBufferBottom; // Index of oldest data
extern volatile uint8_t TPL_RxBufferTop; // Index of newest data

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
        // print_lpuart("|timeout\n");
        return 1;
      }
      BCC_MCU_WaitUs(1);
    }
    buffer[i] = TPL_RxBuffer[TPL_RxBufferBottom];
    TPL_RxBufferBottom++;
    TPL_RxBufferLevel--;
    // char printBuffer[8];
    // sprintf(printBuffer, "|%u", TPL_RxBufferLevel);
    // print_lpuart(printBuffer);
  }
  // print_lpuart("\n");
  return 0;
}
/***************** end communication functions */
