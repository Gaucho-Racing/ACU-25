#include "debug.h"
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
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {if(counter++ > SPI_LOOP_TIMEOUT) return 1;}
    for (uint16_t i = 0; i < length; i++) {
      LL_SPI_TransmitData8(SPI1, (uint8_t)(data[i]));
    }
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    return 0;
}
uint8_t spi_read_string(uint8_t *buffer, uint16_t length){
    uint32_t counter = 0;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI2)) {if(counter++ > SPI_LOOP_TIMEOUT) return 1;}
    for (uint16_t i = 0; i < length; i++) {
      buffer[i] = LL_SPI_ReceiveData8(SPI2);
    }
    return 0;
}
/***************** end communication functions */
