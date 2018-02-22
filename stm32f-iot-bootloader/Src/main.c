
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


UART_HandleTypeDef huart5;

osThreadId defaultTaskHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void const * argument);
/**********************************Extract Frame Data*******************/
struct __packed FW_StartPacket{
	                             char stx[4];          // start packet, len = 54byte
	                             char packet_id[9];
	                             char packet_len[9];
	                             char sf_location[11];
	                             char mf_location[11];
	                             char crc[6];
	                             char etx[4];
                              }startPacket;

struct __packed FW_ConfigPacket{
	                             char stx[4];          // config packet, len = 32bytes
	                             char packet_id[9];
	                             char packet_len[9];
	                             char crc[6];
	                             char etx[4];
                              }configPacket;
struct __packed FW_DataPacket{
	                             char stx[4];         // data packet, len = 1056 full packet 
	                             char packet_id[9];
	                             char packet_len[9];
	                             char packet_data[1024];
	                             char crc[6];
	                             char etx[4];
                              }dataPacket;
struct __packed FW_EndPacket{                         // end packet, len = 41 bytes
	                             char stx[4];
	                             char packet_id[9];
	                             char packet_len[9];
	                             char firmwareLen[9];
	                             char crc[6];
	                             char etx[4];
                              }endPacket;

struct __packed FW_RespnsePacket{
	                             char stx[4];           // response packet, len = 34bytes
	                             char packet_id[9];
	                             char packet_len[9];
	                             char Status[2];
	                             char crc[6];
	                             char etx[4];
                              }responsePacket;

/********************************** Frame Def **************************/
typedef  __packed struct __FRAME_DATA{
	uint16_t Id;
	uint16_t Length;
	uint8_t  Data[1024];
	uint8_t  Status;  
	uint8_t  Read_Write;
} CreateFrame;
uint16_t frameCharCounter;

CreateFrame thisFrame;
/********************************* Def End ******************************/
int main(void)
{
   HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_UART5_Init();
  
 
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osKernelStart();
  
  while (1)
  {
       UART5_IRQHandler();
  

  }
  

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
