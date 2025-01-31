#include "mcu.h"

extern uint32_t BCC_MCU_Timeout_Start;

void BCC_MCU_WaitMs(uint16_t delay) {
    LL_mDelay(delay);
}

// NEED TO FIX THIS
void BCC_MCU_WaitUs(uint32_t delay) {
    LL_mDelay(1); 
}

// NEED TO FIX THIS
bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs){
//   BCC_MCU_Timeout_Start = micros();
  return BCC_STATUS_SUCCESS;
}

bool BCC_MCU_TimeoutExpired(void){
    return false;
//   return (micros() - BCC_MCU_Timeout_Start) > 1000;
}

void BCC_MCU_Assert(const bool x) {
  if (!x) print("BCC_MCU_Assert failed\n");
}

bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[]){
    return BCC_STATUS_SUCCESS;
}

bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt) {
    
    // send
    if(spi_send_string(txBuf, BCC_MSG_SIZE) != 0) return BCC_STATUS_COM_TIMEOUT;

    // receive
    uint8_t buffer[BCC_MSG_SIZE];
    for (uint16_t rxCount = 0; rxCount < recvTrCnt; rxCount++) {
        memset(buffer, '\0', BCC_MSG_SIZE);
        if (spi_read_string(buffer, BCC_MSG_SIZE) != 0){
            return BCC_STATUS_COM_TIMEOUT;
        }
        for (uint8_t i = 0; i < BCC_MSG_SIZE; i++) {
            rxBuf[rxCount*BCC_MSG_SIZE + i] = buffer[i];
        }
    }
    return BCC_STATUS_SUCCESS;
}

void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value) {
    // digitalWriteFast(PIN_BCC_TX_CS, value);
}

// not using
void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value) {
    // digitalWriteFast(PIN_BCC_TX_RST, value);
}

void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value) {
    // digitalWriteFast(PIN_BCC_EN, value);
}

uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance) {
    // return digitalReadFast(PIN_BCC_INT);
    return 1;
}