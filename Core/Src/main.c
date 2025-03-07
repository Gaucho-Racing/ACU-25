/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "acu.h" // fetch bcc & other componentes here
#include "mcu.h" // might not need
#include "state.h" // fetch states
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
  uint8_t can_type;
  uint8_t data[64];
  FDCAN_RxHeaderTypeDef RxHeader;
  FDCAN_HandleTypeDef* hfdcan;
} CAN_RX_message;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// cycle tracker
uint8_t cycle;

// Devices & Sensors
ACU acu;
Battery battery;

// ADC Data
float cur_ref = 0;
uint16_t adc_data[3]; // 0: ts_voltage, 1: glv voltage, 2: sdc_voltage

// BCC
uint8_t bcc_cooked_count = 0;
uint16_t bcc_faults;
uint32_t BCC_MCU_Timeout_Start;
uint32_t BCC_MCU_Timeout_length = 0;
bcc_status_t bcc_error;

// communication BCC - TPL
volatile uint8_t TPL_RxBuffer[256]; // Array to store received SPI data => Replace with struct holding CAN RxBuffer
volatile uint8_t TPL_RxBufferLevel = 0; // Number of bytes to be read
volatile uint8_t TPL_RxBufferBottom = 0; // Index of oldest data
volatile uint8_t TPL_RxBufferTop = 0; // Index of newest data

// communication - FDCAN
extern FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

volatile CAN_RX_message CAN_RxBuffer[256]; // Array to store received CAN data
volatile uint8_t CAN_RxBufferLevel = 0; // Number of bytes to be read
volatile uint8_t CAN_RxBufferBottom = 0; // Index of oldest data
volatile uint8_t cAN_RxBufferTop = 0; // Index of newest data

// extern FDCAN_HandleTypeDef hfdcan2; ==> separate this
// FDCAN_TxHeaderTypeDef TxHeader_Data;
// FDCAN_RxHeaderTypeDef RxHeader_Data;

// extern FDCAN_HandleTypeDef hfdcan3; ==> separate this
// FDCAN_TxHeaderTypeDef TxHeader_Charger;
// FDCAN_RxHeaderTypeDef RxHeader_Charger;

// SHARED BUFFER
uint8_t CAN_TxData[64];
uint8_t CAN_RxData[64];
uint8_t readCount = 0;

// stateful things
State state;
extern void debug();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void DWT_Delay_Init();
void print_lpuart(char* arr);
void print_bcc_status(bcc_status_t bccStatus);
void print_bcc_fault(bcc_fault_status_t fault);
int16_t Read_ADC1_Channel(uint32_t channel);

// to delete functions
void print_can_msg(uint8_t * arr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_lpuart(char* arr) {
  uint32_t idx = 0; // index
  while (arr[idx]) {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1));
    LL_LPUART_TransmitData8(LPUART1, arr[idx]);
    idx++;
  }
}

void print_can_msg(uint8_t * arr){
  uint32_t idx = 0; // index
  while (arr[idx]) {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1));
    LL_LPUART_TransmitData8(LPUART1, arr[idx]);
    idx++;
  }
}



// setup for bcc
int setup(){

  // setup ADC
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_data);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 3);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_StartConversion(ADC1);

  // setup battery configuring
  print_lpuart("configuring bcc...\n");
  battery.drvConfig.commMode = BCC_MODE_TPL;
  battery.drvConfig.drvInstance = 0U;
  battery.drvConfig.devicesCnt = NUM_TOTAL_IC;
  battery.drvConfig.loopBack = false;
  for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
    battery.drvConfig.device[i] = BCC_DEVICE_MC33771C;
    battery.drvConfig.cellCnt[i] = NUM_CELL_IC;
  }
  
  // init bcc
  bcc_error = BCC_Init(&(battery.drvConfig));
  uint8_t counter = TRIES;
  while (bcc_error != BCC_STATUS_SUCCESS && counter > 0){
    print_lpuart("failed BCC_Init...");
    print_bcc_status(bcc_error);
    LL_mDelay(1000);
    print_lpuart("retrying...\n");
    bcc_error = BCC_Init(&(battery.drvConfig));
  }
  if (counter == 0){
    state = SHITDOWN;
    print_lpuart("you goon...\n");
    return -1;
  }

  bcc_error = init_registers(&battery);
  while (bcc_error != BCC_STATUS_SUCCESS) {
    print_lpuart("failed init_registers...");
    LL_mDelay(1000);
    bcc_error = init_registers(&battery);
  }
  clear_faults(&(battery.drvConfig));
  print_lpuart("successful BCC_Init...\n");

  // configure cell balancing
  for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
  {
      if((BCC_CB_Enable(&(battery.drvConfig), (bcc_cid_t)(i+1),  true)) != BCC_STATUS_SUCCESS) {
          print_lpuart("failed BCC_CB_Enable for cid [insert] ...\n");
          state = SHITDOWN;
          for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            battery.cell_balancing[i*NUM_CELL_IC+j] = 100;
          }
          print_lpuart("you goon...\n");
          return -1;
      }
      for(uint8_t j = 0; j < NUM_CELL_IC; j++){
        battery.cell_balancing[i*NUM_CELL_IC+j] = 0;
      }
  }
  print_lpuart("successful BCC_CB_Enable...\n");
  state = STANDBY;
  
  // setup acu
  acu_init(&acu);
  acu.bty = &battery;

  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();
  MX_LPUART1_UART_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  /* Enable the SPI peripherals */
  BCC_MCU_WriteCsbPin(0, 1);
  LL_SPI_Enable(SPI1);
  LL_SPI_Enable(SPI2);
  LL_SPI_EnableIT_RXNE(SPI2);

  /* Enable the CAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    print_lpuart("failed to HAL_FDCAN_Start");
    Error_Handler();
  }

  // Activate interrupting capabilities
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // enable microsecond timer
  LL_TIM_EnableCounter(TIM5);
  LL_mDelay(1000);

  if(setup() != 0) state = SHITDOWN;



  // default: cell balancing is off
  reset_discharge(&battery);

  // Configure TxHeader
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; 

  // COnfigure RxHeader
  RxHeader.Identifier = 0;
  RxHeader.IdType = FDCAN_EXTENDED_ID;
  RxHeader.RxFrameType = FDCAN_DATA_FRAME;
  RxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  RxHeader.DataLength = FDCAN_DLC_BYTES_8;
  RxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  RxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  RxHeader.RxTimestamp = 0;/* Specifies the timestamp counter value captured on start of frame reception. Between 0 and 0xFFFF  */           

  cur_ref = 0; // ACU_ADC.readVoltageTot(ADC_MUX_HV_CURRENT,256);

  if(!state_system_check(true, true)){
    state = SHITDOWN;
     print_lpuart("System check failed, shutting down\n");
  }
  else{
    state = STANDBY;
    print_lpuart("System check passed, entering standby\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(acu.ts_active && state == STANDBY) state = PRECHARGE;
    else if(acu.ts_active && state > STANDBY) state = SHITDOWN;
    
    acu.ts_voltage = adc_data[0];
    acu.glv_voltage = adc_data[1];
    acu.sdc_voltage = adc_data[2];

    BCC_MCU_WaitMs(500);
    print_lpuart("State: ");
    switch(state){
      case (STANDBY):
        print_lpuart("STANDBY...\n");
        standby();
        break;
      case (PRECHARGE):
        print_lpuart("PRECHARGE...\n");
        precharge();
        break;
      case (CHARGE):
        print_lpuart("CHARGE...\n");
        charge();
        break;
      case (NORMAL):
        print_lpuart("NORMAL...\n");
        normal();
        break;
      case (SHITDOWN):
        print_lpuart("SHITDOWN...\n");
        shitdown();
        break;
      default:
        break;
    }

    // system checks & cooked counter
    battery_check(&battery, false);

    // send ACU ping
    can_send(&acu, ACU_Ping_Debug);

    // poll for can messages
    can_read_all(&acu);
    can_dump(&acu);
    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  LL_SPI_Disable(SPI1);
  LL_SPI_Disable(SPI2);
  HAL_FDCAN_Stop(&hfdcan1);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 16, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(128000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Enable DWT_Delay
void DWT_Delay_Init(void){
  // do we need to check (!CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk)?
  // don't know but it works so I'm not touching this
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;  // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable counter
}

void print_bcc_fault(bcc_fault_status_t fault){
  switch (fault)
  {
  case BCC_FS_CELL_OV:
      print_lpuart("CT overvoltage fault (register CELL_OV_FLT).\n");
      break;
  case BCC_FS_CELL_UV:
      print_lpuart("CT undervoltage fault (register CELL_UV_FLT).\n");
      break;
  case BCC_FS_CB_OPEN:
      print_lpuart("Open CB fault (register CB_OPEN_FLT).\n");
      break;
  case BCC_FS_CB_SHORT:
      print_lpuart("Short CB fault (register CB_SHORT_FLT).\n");
      break;
  case BCC_FS_GPIO_STATUS:
      print_lpuart("GPIO status (register GPIO_STS).\n");
      break;
  case BCC_FS_AN_OT_UT:
      print_lpuart("AN over and undertemperature (register AN_OT_UT_FLT). \n");
      break;
  case BCC_FS_GPIO_SHORT:
      print_lpuart("Short GPIO/open AN diagnostic (register GPIO_SHORT_ANx_OPEN_STS). \n");
      break;
  case BCC_FS_COMM:
      print_lpuart("Number of communication errors detected (register COM_STATUS).\n");
      break;
  case BCC_FS_FAULT1:
      print_lpuart("Fault status (register FAULT1_STATUS).\n");
      break;
  case BCC_FS_FAULT2:
      print_lpuart("Fault status (register FAULT2_STATUS).\n");
      break;
  case BCC_FS_FAULT3:
      print_lpuart("Fault status (register FAULT3_STATUS).\n");
      break;
  default:
      print_lpuart("Unknown status\n");
      break;
  }
}

void print_bcc_status(bcc_status_t bccStatus){
  switch (bccStatus)
  {
  case BCC_STATUS_SUCCESS:
      print_lpuart("Success\n");
      break;
  case BCC_STATUS_PARAM_RANGE:
      print_lpuart("Parameter out of range\n");
      break;
  case BCC_STATUS_SPI_FAIL:
      print_lpuart("SPI failed\n");
      break;
  case BCC_STATUS_COM_TIMEOUT:
      print_lpuart("communication timeout\n");
      break;
  case BCC_STATUS_COM_ECHO:
      print_lpuart("Echo frame doesn't correspond to sent frame\n");
      break;
  case BCC_STATUS_COM_CRC:
      print_lpuart("CRC error\n");
      break;
  case BCC_STATUS_COM_MSG_CNT:
      print_lpuart("Message counter mismatch\n");
      break;
  case BCC_STATUS_COM_NULL:
      print_lpuart("NULL message\n");
      break;
  case BCC_STATUS_DIAG_FAIL:
      print_lpuart("Diagnoctic mode not allowed\n");
      break;
  case BCC_STATUS_EEPROM_ERROR:
      print_lpuart("EEPROM communication error\n");
      break;
  case BCC_STATUS_EEPROM_PRESENT:
      print_lpuart("EEPROM device not detected\n");
      break;
  case BCC_STATUS_DATA_RDY:
      print_lpuart("New convertion already running\n");
      break;
  case BCC_STATUS_TIMEOUT_START:
      print_lpuart("BCC_MCU_StartTimeout function error\n");
      break;
  default:
      print_lpuart("Unknown status\n");
      break;
  }
}

// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
//   FDCAN_RxHeaderTypeDef RxHeader;
//   uint8_t RxData[8]; // Assuming a maximum data length of 8 bytes

//   if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
//     /* Retrieve Rx messages from RX FIFO0 */
//     if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
//       Error_Handler(); // Handle error if message retrieval fails
//     } else {
//       // Process the received message
//       // ...
//       // Example: Check identifier and data length
//       if (RxHeader.Identifier == 0x123 && RxHeader.DataLength == 4) {
//         // Process the data in RxData
//         // ...
//       }
//     }
//   }
// }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    print_lpuart("Error has occured!\n");
    LL_mDelay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
