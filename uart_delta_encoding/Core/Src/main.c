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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for data_handler_Ta */
osThreadId_t data_handler_TaHandle;
const osThreadAttr_t data_handler_Ta_attributes = {
  .name = "data_handler_Ta",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for binary_semaphore_01 */
osSemaphoreId_t binary_semaphore_01Handle;
const osSemaphoreAttr_t binary_semaphore_01_attributes = {
  .name = "binary_semaphore_01"
};
/* USER CODE BEGIN PV */
// REQUISITO: Buffer de 8 bytes según especificación (NO 64)
volatile uint8_t rx_buffer[8] = {0};
volatile uint8_t tx_buffer[16] = {0};  // Buffer más grande para debug
volatile uint8_t data_ready = 0;  // Flag para sincronización
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void data_handler_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Callback de recepción UART completa (ISR)
 * REQUISITO: Solo copia datos y libera semáforo, NO hace delta encoding
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		// Marcar que hay datos listos
		data_ready = 1;

		// REQUISITO: Liberar semáforo binario para señalar tarea de procesamiento
		osSemaphoreRelease(binary_semaphore_01Handle);

		// REQUISITO: Configurar UART para recibir próximos 8 bytes
		HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, 8);
	}
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // REQUISITO: Iniciar recepción UART de 8 bytes
  HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, 8);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binary_semaphore_01 */
  binary_semaphore_01Handle = osSemaphoreNew(1, 0, &binary_semaphore_01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of data_handler_Ta */
  data_handler_TaHandle = osThreadNew(data_handler_task, NULL, &data_handler_Ta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blue_led_Pin */
  GPIO_InitStruct.Pin = LED_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_blue_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t tick_2s = pdMS_TO_TICKS(2000);
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LED_blue_GPIO_Port,LED_blue_Pin);
	HAL_UART_Transmit(&huart3, (uint8_t*)"not taken\r\n", sizeof("not taken\r\n")-1, 2000);
    vTaskDelay(tick_2s);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_data_handler_task */
/**
* @brief Function implementing the data_handler_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_data_handler_task */
void data_handler_task(void *argument)
{
  /* USER CODE BEGIN data_handler_task */
	uint8_t i;
	uint8_t local_buffer[8];  // Buffer local para evitar problemas de concurrencia
	int8_t delta_values[8];   // Buffer para valores delta con signo
	char debug_msg[100];      // Buffer para mensajes de debug
  /* Infinite loop */
  for(;;)
  {
    // REQUISITO: Usar osSemaphoreAcquire con espera indefinida (sin busy waits)
	osStatus_t status = osSemaphoreAcquire(binary_semaphore_01Handle, osWaitForever);

	if (status == osOK && data_ready) {
        data_ready = 0;  // Limpiar flag

        HAL_UART_Transmit(&huart3, (uint8_t*)"Sem: Taken\r\n", sizeof("Sem: Taken\r\n")-1, 1000);
        HAL_UART_Transmit(&huart3, (uint8_t*)"Delta Encoding\r\n", sizeof("Delta Encoding\r\n")-1, 1000);

        for (i = 0; i < 8; i++) {
            local_buffer[i] = rx_buffer[i];
        }

        // Mostrar datos recibidos para debug
        HAL_UART_Transmit(&huart3, (uint8_t*)"Recibido: ", 10, 1000);
        for (i = 0; i < 8; i++) {
            sprintf(debug_msg, "%c", local_buffer[i]);
            HAL_UART_Transmit(&huart3, (uint8_t*)debug_msg, 1, 1000);
        }
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 1000);

        // REQUISITO: Delta encoding DEBE aplicarse en tarea diferida, NO en ISR
        // Primer byte se envía como está
        delta_values[0] = (int8_t)local_buffer[0];

        // Bytes subsecuentes son diferencias del anterior
        for (i = 1; i < 8; i++) {
            delta_values[i] = (int8_t)((int16_t)local_buffer[i] - (int16_t)local_buffer[i-1]);
        }

        // Mostrar delta encoding para debug
        HAL_UART_Transmit(&huart3, (uint8_t*)"Delta: [", 8, 1000);
        for (i = 0; i < 8; i++) {
            sprintf(debug_msg, "%d", (int)delta_values[i]);
            HAL_UART_Transmit(&huart3, (uint8_t*)debug_msg, strlen(debug_msg), 1000);
            if (i < 7) {
                HAL_UART_Transmit(&huart3, (uint8_t*)", ", 2, 1000);
            }
        }
        HAL_UART_Transmit(&huart3, (uint8_t*)"]\r\n", 3, 1000);

        // REQUISITO: Transmisión UART desde tarea diferida, no desde ISR
        // Convertir valores con signo a formato transmisible
        for (i = 0; i < 8; i++) {
            tx_buffer[i] = (uint8_t)delta_values[i];
        }

        HAL_UART_Transmit(&huart3, (uint8_t*)"Enviando bytes: ", 16, 1000);
        HAL_UART_Transmit(&huart3, tx_buffer, 8, 1000);
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n\r\n", 4, 1000);
    }
  }
  /* USER CODE END data_handler_task */
}

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
