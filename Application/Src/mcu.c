#include "mcu.h"

extern uint32_t BCC_MCU_Timeout_Start;
extern uint32_t spiRx[10];

// use LL_mDelay() to set delay
void BCC_MCU_WaitMs(uint16_t delay) {
    LL_mDelay(delay);
    LL_mDelay(delay);
}

// unfortunately no LL_uDelay() :( so we do this heinous stuff
void BCC_MCU_WaitUs(uint32_t delay) {
    uint32_t clk_cycle_start = DWT->CYCCNT;

     // Convert microseconds to CPU cycles
    uint32_t clk_cycles = (SystemCoreClock / 1e6) * delay; 
    while ((DWT->CYCCNT - clk_cycle_start) < clk_cycles);
}

bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs){
    BCC_MCU_Timeout_Start = DWT->CYCCNT;
    BCC_MCU_WaitUs(timeoutUs);
    bool result = BCC_MCU_TimeoutExpired();
    if(result == false){
        return BCC_STATUS_TIMEOUT_START;
    }
    return BCC_STATUS_SUCCESS;
}

// i think?
bool BCC_MCU_TimeoutExpired(void){
    uint32_t time = (SystemCoreClock / 1e6) * 1000;
    return (DWT->CYCCNT - BCC_MCU_Timeout_Start) < time;
}

void BCC_MCU_Assert(const bool x) {
  if (!x) print("BCC_MCU_Assert failed\n");
  if (!x) print("BCC_MCU_Assert failed\n");
}

// ideally won't be using this
bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[]){
    return BCC_STATUS_SUCCESS;
    return BCC_STATUS_SUCCESS;
}

// comm channel with bcc & acu
bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt) {
    
    // move data to buffer
    uint8_t buffer[BCC_MSG_SIZE];
    memcpy(buffer, txBuf, 6);

    // send
    if(spi_send_string(txBuf, BCC_MSG_SIZE) != 0) return BCC_STATUS_SPI_FAIL;
    BCC_MCU_WaitUs(1);
    // receive
    for (uint16_t rxCount = 0; rxCount < recvTrCnt; rxCount++) {
        memset(buffer, '\0', BCC_MSG_SIZE);

        if (spi_read_string(buffer, BCC_MSG_SIZE) != 0){
            return BCC_STATUS_SPI_FAIL;
        }
        BCC_MCU_WaitUs(1);
        for (uint8_t i = 0; i < BCC_MSG_SIZE; i++) {
            rxBuf[rxCount*BCC_MSG_SIZE + i] = buffer[i];
        }
    }
    return BCC_STATUS_SUCCESS;
}

void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value) {
    if (value) {
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4); // idk
    }
    else {
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4); // idk
    }
    return;
}

// use GPIOC7
void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value) {
    if (value) {
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);
        return;
    }
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7); 
}

// use GPIOC6
void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value) {
    if (value) {
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
        return;
    }
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6); 
}

// use GPIOB11
uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance) {
    return (LL_GPIO_ReadInputPort(GPIOB)&(1U<<11)) != 0;
    return 1;
}