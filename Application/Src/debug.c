#include "debug.h"

extern void print_lpuart(char* arr);

/******************** communication functions */
void print(char* arr) {
  uint32_t idx = 0;
  while (arr[idx]) {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1));
    LL_LPUART_TransmitData8(LPUART1, arr[idx]);
    idx++;
  }
}

void print_float(float value){
  char buffer[8];
  char *sign = (value < 0) ? "-": ""; // get sign
  float signedFloat = (value < 0) ? -value : value;

  int upper = (int)signedFloat;
  float diff = signedFloat-upper;
  int lower = (int)trunc(1000 * diff);

  sprintf(buffer, "%s%d.%.03d\n", sign, upper, lower);
  print(buffer);
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
    BCC_MCU_WaitUs(4); // don't know why but seems we need this
  }
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  BCC_MCU_WaitUs(1); // delay required by MC33664
  BCC_MCU_WriteCsbPin(0, 1); // CS HIGH
  return 0;
}

uint8_t spi_read_string(uint8_t *buffer, uint16_t length){
  for (uint16_t i = 0; i < length; i++) {
    uint32_t counter = 0;
    while (!(SPI2->SR&1U)) {
      if(counter++ > SPI_LOOP_TIMEOUT) {
        print_lpuart("|timeout\n");
        return 1;
      }
      BCC_MCU_WaitUs(1);
    }
    buffer[i] = LL_SPI_ReceiveData8(SPI2);
    char printBuffer[32];
    sprintf(printBuffer, "|%u", buffer[i]);
    print_lpuart(printBuffer);
  }
  print_lpuart("\n");
  return 0;
}
/***************** end communication functions */
