#ifndef ACU_H
#define ACU_H
#include "battery.h"

typedef struct {
    battery bty;

    uint8_t fdcan1Buff[8];      // to be changed
    uint8_t tx_buff_1[8];       // full duplex master, NSS disabled
    uint8_t rx_buff_1[8];       // full duplex master, NSS disabled
    uint8_t tx_buff_2[8];       // full duplex slave, NSS enabled
    uint8_t rx_buff_2[8];       // full duplex slave, NSS enabled
    uint8_t lpuart1[8];         // async, rs232 hardware flow control (disabled)

} ACU;

#endif