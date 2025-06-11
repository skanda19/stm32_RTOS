/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Automotive Dashboard Data Logging System
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct dashboard_buffer {
    int speed;           // Speed in km/h
    char GPS[128];       // NMEA GPS string (GPGGA format)
    int temperature;     // Temperature in Kelvin
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

/* Definitions for speedTask */
osThreadId_t speedTaskHandle;
const osThreadAttr_t speedTask_attributes = {
  .name = "speedTask",
  .stack_size = 128 * 8,  // Increased stack size
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 128 * 8,  // Increased stack size
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for temperatureTask */
osThreadId_t temperatureTaskHandle;
const osThreadAttr_t temperatureTask_attributes = {
  .name = "temperatureTask",
  .stack_size = 128 * 8,  // Increased stack size
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for bufferRenderingTask */
osThreadId_t bufferRenderingTaskHandle;
const osThreadAttr_t bufferRenderingTask_attributes = {
  .name = "bufferRenderingTask",
  .stack_size = 128 * 16,  // Much larger stack for printf operations
  .priority = (osPriority_t) osPriorityHigh,
};

/* USER CODE BEGIN PV */
// Shared dashboard buffer
struct dashboard_buffer dashboard_data = {0};

// Mutex for protecting shared buffer access
osMutexId_t dashboardMutexHandle;
const osMutexAttr_t dashboardMutex_attributes = {
  .name = "dashboardMutex"
};

// Global time counter for GPS timestamps (in seconds)
uint32_t gps_time_counter = 123519; // Starting time 12:35:19

// Random seed counter
uint32_t random_seed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void speedTaskHook(void *argument);
void gpsTaksHook(void *argument);
void temperatureTaskHook(void *argument);
void bufferRenderingTaskHook(void *argument);

/* USER CODE BEGIN PFP */
// Utility functions
uint32_t generate_random(uint32_t min, uint32_t max);
float generate_random_float(float min, float max);
void generate_gps_string(char* gps_buffer, size_t buffer_size);
int uart_printf(const char* format, ...);
uint8_t calculate_nmea_checksum(const char* sentence);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Simple pseudo-random number generator
 * @param min: minimum value (inclusive)
 * @param max: maximum value (inclusive)
 * @retval Random number between min and max
 */
uint32_t generate_random(uint32_t min, uint32_t max) {
    if (max <= min) return min;  // Safety check

    random_seed = (random_seed * 1103515245 + 12345) & 0x7fffffff;
    uint32_t range = max - min + 1;
    return min + (random_seed % range);
}

/**
 * @brief Generate random float number
 * @param min: minimum value
 * @param max: maximum value
 * @retval Random float between min and max
 */
float generate_random_float(float min, float max) {
    uint32_t rand_int = generate_random(0, 1000);
    float ratio = (float)rand_int / 1000.0f;
    return min + ratio * (max - min);
}

/**
 * @brief Calculate NMEA sentence checksum
 * @param sentence: NMEA sentence (without $ and *)
 * @retval Calculated checksum
 */
uint8_t calculate_nmea_checksum(const char* sentence) {
    uint8_t checksum = 0;
    while (*sentence) {
        checksum ^= *sentence;
        sentence++;
    }
    return checksum;
}

/**
 * @brief Generate synthetic GPS GPGGA string
 * @param gps_buffer: Buffer to store the GPS string
 * @param buffer_size: Size of the buffer
 */
void generate_gps_string(char* gps_buffer, size_t buffer_size) {
    char temp_sentence[128];

    if (gps_buffer == NULL || buffer_size < 50) {
        return;  // Safety check
    }

    // Convert time counter to HHMMSS format
    uint32_t hours = (gps_time_counter / 10000) % 24;
    uint32_t minutes = (gps_time_counter / 100) % 100;
    uint32_t seconds = gps_time_counter % 100;

    // Generate random values
    uint8_t satellites = generate_random(3, 10);
    float hdop = generate_random_float(0.5f, 2.0f);
    float altitude = generate_random_float(100.0f, 1000.0f);

    // Create the sentence without checksum
    int ret = snprintf(temp_sentence, sizeof(temp_sentence),
             "GPGGA,%02lu%02lu%02lu,4807.038,N,01131.000,E,1,%02d,%.1f,%.1f,M,46.9,M,,",
             hours, minutes, seconds, satellites, hdop, altitude);

    if (ret < 0 || ret >= sizeof(temp_sentence)) {
        strcpy(gps_buffer, "$GPGGA,120000,4807.038,N,01131.000,E,1,04,1.0,100.0,M,46.9,M,,*5E");
        return;
    }

    // Calculate checksum
    uint8_t checksum = calculate_nmea_checksum(temp_sentence);

    // Create final GPS string with checksum
    snprintf(gps_buffer, buffer_size, "$%s*%02X", temp_sentence, checksum);

    // Increment time for next call (simulate 1 second increment)
    gps_time_counter++;
    if ((gps_time_counter % 100) >= 60) {
        gps_time_counter += 40; // Skip to next minute
    }
    if ((gps_time_counter % 10000) >= 6000) {
        gps_time_counter += 4000; // Skip to next hour
    }
}

/**
 * @brief Printf function for UART output
 * @param format: printf-style format string
 * @retval Number of characters transmitted
 */
int uart_printf(const char* format, ...) {
    char buffer[512];  // Increased buffer size
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Ensure we don't exceed buffer size
    if (length >= sizeof(buffer)) {
        length = sizeof(buffer) - 1;
        buffer[length] = '\0';
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t*)buffer, length, 5000);
    if (status != HAL_OK) {
        // UART transmission failed
        return -1;
    }
    return length;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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

  // Initialize random seed with a pseudo-random value
  random_seed = HAL_GetTick();
  if (random_seed == 0) {
    random_seed = 12345;  // Fallback seed
  }

  // Initialize dashboard buffer with default values
  dashboard_data.speed = 0;
  dashboard_data.temperature = 273;
  strcpy(dashboard_data.GPS, "Initializing...");

  // Send initial debug message
  uart_printf("System Initialization Complete\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Create dashboard mutex */
  dashboardMutexHandle = osMutexNew(&dashboardMutex_attributes);
  if (dashboardMutexHandle == NULL) {
    // Handle error - mutex creation failed
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of speedTask */
  speedTaskHandle = osThreadNew(speedTaskHook, NULL, &speedTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(gpsTaksHook, NULL, &gpsTask_attributes);

  /* creation of temperatureTask */
  temperatureTaskHandle = osThreadNew(temperatureTaskHook, NULL, &temperatureTask_attributes);

  /* creation of bufferRenderingTask */
  bufferRenderingTaskHandle = osThreadNew(bufferRenderingTaskHook, NULL, &bufferRenderingTask_attributes);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_speedTaskHook */
/**
  * @brief  Function implementing the speedTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_speedTaskHook */
void speedTaskHook(void *argument)
{
  /* USER CODE BEGIN 5 */
  uart_printf("Speed Task Started\r\n");

  /* Infinite loop */
  for(;;)
  {
    // Generate synthetic speed data (0-200 km/h)
    int new_speed = generate_random(0, 200);

    // Acquire mutex to safely update shared buffer
    if (osMutexAcquire(dashboardMutexHandle, 1000) == osOK) {
      dashboard_data.speed = new_speed;
      osMutexRelease(dashboardMutexHandle);
    }

    // Wait 100ms for 10Hz polling rate
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_gpsTaksHook */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gpsTaksHook */
void gpsTaksHook(void *argument)
{
  /* USER CODE BEGIN gpsTaksHook */
  uart_printf("GPS Task Started\r\n");

  /* Infinite loop */
  for(;;)
  {
    char gps_string[128];

    // Generate synthetic GPS data
    generate_gps_string(gps_string, sizeof(gps_string));

    // Acquire mutex to safely update shared buffer
    if (osMutexAcquire(dashboardMutexHandle, 1000) == osOK) {
      strncpy(dashboard_data.GPS, gps_string, sizeof(dashboard_data.GPS) - 1);
      dashboard_data.GPS[sizeof(dashboard_data.GPS) - 1] = '\0'; // Ensure null termination
      osMutexRelease(dashboardMutexHandle);
    }

    // Wait 500ms for 2Hz polling rate
    osDelay(500);
  }
  /* USER CODE END gpsTaksHook */
}

/* USER CODE BEGIN Header_temperatureTaskHook */
/**
* @brief Function implementing the temperatureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_temperatureTaskHook */
void temperatureTaskHook(void *argument)
{
  /* USER CODE BEGIN temperatureTaskHook */
  uart_printf("Temperature Task Started\r\n");

  /* Infinite loop */
  for(;;)
  {
    // Generate synthetic temperature data (273-320 Kelvin = 0-47°C)
    int new_temperature = generate_random(273, 320);

    // Acquire mutex to safely update shared buffer
    if (osMutexAcquire(dashboardMutexHandle, 1000) == osOK) {
      dashboard_data.temperature = new_temperature;
      osMutexRelease(dashboardMutexHandle);
    }

    // Wait 200ms for 5Hz polling rate
    osDelay(200);
  }
  /* USER CODE END temperatureTaskHook */
}

/* USER CODE BEGIN Header_bufferRenderingTaskHook */
/**
* @brief Function implementing the bufferRenderingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bufferRenderingTaskHook */
void bufferRenderingTaskHook(void *argument)
{
  /* USER CODE BEGIN bufferRenderingTaskHook */
  struct dashboard_buffer local_copy;

  // Send startup message
  uart_printf("\r\n=== Automotive Dashboard System Started ===\r\n");
  uart_printf("Speed Task: 10Hz | GPS Task: 2Hz | Temperature Task: 5Hz\r\n");
  uart_printf("Dashboard Refresh Rate: 1Hz\r\n");
  uart_printf("================================================\r\n\r\n");

  uart_printf("Dashboard Rendering Task Started\r\n");

  // Wait a bit for other tasks to start
  osDelay(2000);

  /* Infinite loop */
  for(;;)
  {
    // Acquire mutex to safely read shared buffer
    if (osMutexAcquire(dashboardMutexHandle, 1000) == osOK) {
      // Make a local copy to minimize mutex holding time
      memcpy(&local_copy, &dashboard_data, sizeof(struct dashboard_buffer));
      osMutexRelease(dashboardMutexHandle);

      // Print dashboard data to UART
      uart_printf("=== Dashboard Update [%lu ms] ===\r\n", HAL_GetTick());
      uart_printf("Speed: %d km/h\r\n", local_copy.speed);
      uart_printf("Temperature: %d K (%dC)\r\n",
                  local_copy.temperature,
                  local_copy.temperature - 273);
      uart_printf("GPS: %s\r\n", local_copy.GPS);
      uart_printf("----------------------------------\r\n\r\n");
    } else {
      uart_printf("ERROR: Could not acquire mutex for dashboard read\r\n");
    }

    // Wait 1000ms for 1Hz refresh rate
    osDelay(1000);
  }
  /* USER CODE END bufferRenderingTaskHook */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
