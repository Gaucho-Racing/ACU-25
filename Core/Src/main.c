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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Devices & Sensors
ACU acu;
Battery battery;

uint16_t bcc_faults;
bcc_status_t bcc_error;
uint32_t BCC_MCU_Timeout_Start;
uint32_t BCC_MCU_Timeout_length = 0;

// communication stuff - bcc
uint32_t spiRx[10]; // Array to store received SPI data.
volatile int spiRxIdx; //  Index for received SPI data
volatile int spiRxComplete = 0; // Flag to indicate if SPI reception is complete

// Theoretical stuff
State state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// to delete functions
void print_lpuart(char* arr);

// void print_float_LPUART(float value);
// void print_measurement_LPUART(bcc_measurements type, float value);
// uint8_t spi_send(const uint8_t *data, uint16_t length);
// uint8_t spi_read(uint8_t *buffer, uint16_t length);
void DWT_Delay_Init();
void print_bcc_status(bcc_status_t bccStatus);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**************************************** printing
  void print_float_LPUART(float value){
    char buffer[8];
    char *sign = (value < 0) ? "-": ""; // get sign
    float signedFloat = (value < 0) ? -value : value;

    int upper = (int)signedFloat;
    float diff = signedFloat-upper;
    int lower = (int)trunc(1000 * diff);

    sprintf(buffer, "%s%d.%.03d\n", sign, upper, lower);
    print_LPUART(buffer);
  }
  void print_measurement_LPUART(bcc_measurements type, float value){
    
    switch (type){
      case VOLTAGE:
        print_LPUART("Volts: ");
        break;
      case TEMPERATURE:
        print_LPUART("Temp: ");
        break;
      default:
        print_LPUART("Error: ");
        break;
      }
      print_float_LPUART(value);
  }
*/
/**************************************** communication
  
  uint8_t spi_read(uint8_t *buffer, uint16_t length){

      uint32_t counter = 0;
      while (!LL_SPI_IsActiveFlag_RXNE(SPI2)) {if(counter++ > SPI_LOOP_TIMEOUT) return 1;}

      for (uint16_t i = 0; i < length; i++) {
        buffer[i] = LL_SPI_ReceiveData8(SPI2);
      }
      return 0;
  }

  uint8_t spi_send(const uint8_t *data, uint16_t length) {

    uint32_t counter = 0;
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {if(counter++ > SPI_LOOP_TIMEOUT) return 1;}

    for (uint16_t i = 0; i < length; i++) {
      LL_SPI_TransmitData8(SPI1, (uint8_t)(data[i]));
    }
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    return 0;
} */

void print_lpuart(char* arr) {
  uint32_t idx = 0; // index
  while (arr[idx]) {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1));
    LL_LPUART_TransmitData8(LPUART1, arr[idx]);
    idx++;
  }
}




// setup for bcc
int setup(){
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
  print_lpuart("calling BCC_Init...\n");
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

  init_registers(&battery);
  clear_faults(&(battery.drvConfig));
  print_lpuart("successful BCC_Init...\n");

  // configure cell balancing
  for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
  {
      if((BCC_CB_Enable(&(battery.drvConfig), (bcc_cid_t)i+1,  true)) != BCC_STATUS_SUCCESS) {
          print_lpuart("failed BCC_CB_Enable for cid [insert] ...\n");
          state = SHITDOWN;
          print_lpuart("you goon...\n");
          return -1;
      }
  }
  print_lpuart("successful BCC_CB_Enable...\n");
  
  print_lpuart("reading device measurements...\n");
  if((bcc_error = read_device_measurements(&battery)) != BCC_STATUS_SUCCESS){
      state = SHITDOWN;
      print_lpuart("failed read_device_measurements...\n");
      return -1;
  }
  BCC_MCU_WaitUs(500);
  if((bcc_faults = check_volt(&battery)) != BCC_FS_FAULT3){ // default BCC_FS_FAULT3 to be no fault FOR NOW
      state = SHITDOWN;
      print_lpuart("failed check_volt...\n");
      return -1;
  }
  
  if((bcc_faults = check_temp(&battery)) != BCC_FS_FAULT3){
      state = SHITDOWN;
      print_lpuart("failed check_temp...\n");
      return -1;
  }
  
  print_lpuart("passed initial checks...\n");
  // configure state
  state = STANDBY;

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** Disable the Internal Voltage Reference buffer
  */
  LL_VREFBUF_Disable();

  /** Configure the internal voltage reference buffer high impedance mode
  */
  LL_VREFBUF_EnableHIZ();

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  /* Enable the SPI peripherals */
  BCC_MCU_WriteCsbPin(0, 1);
  LL_SPI_Enable(SPI1);
  LL_SPI_Enable(SPI2);

  // enable microsecond timer
  LL_TIM_EnableCounter(TIM5);

  print_lpuart("Hello World\n");
  // print_measurement_LPUART(TEMPERATURE, 3.14);
  // print_measurement_LPUART(TEMPERATURE, -43120.14);
  // print_LPUART("\n\n");
  LL_mDelay(1000);

  if(setup() != 0) state = SHITDOWN;
  
  // setup acu
  acu.bty = &battery;
  acu.rx_buff = spiRx;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch(state){
      case (STANDBY):
        standby();
        break;
      case (PRECHARGE):
        precharge();
        break;
      case (CHARGE):
        charge();
        break;
      case (NORMAL):
        normal();
        break;
      case (SHITDOWN):
        shitdown();
        break;
      default:
        break;
    }
    print_lpuart("Hello World\n");
    // print_measurement_LPUART(TEMPERATURE, 3.14);
    // print_measurement_LPUART(TEMPERATURE, -43120.14);
    LL_mDelay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  LL_SPI_Disable(SPI1);
  LL_SPI_Disable(SPI2);
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

  LL_Init1msTick(128000000);

  LL_SetSystemCoreClock(128000000);
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
    print_LPUART("Error has occured!\n");
    LL_mDelay(100);
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
