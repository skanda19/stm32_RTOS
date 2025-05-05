///**
//  ******************************************************************************
//  * @file    Examples_LL/USART/USART_Communication_Rx_IT/Src/main.c
//  * @author  MCD Application Team
//  * @brief   This example describes how to send bytes over USART IP using
//  *          the STM32F7xx USART LL API.
//  *          Peripheral initialization done using LL unitary services functions.
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2016 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
//
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///** @addtogroup STM32F7xx_LL_Examples
//  * @{
//  */
//
///** @addtogroup USART_Communication_Rx_IT
//  * @{
//  */
//
///* Private typedef -----------------------------------------------------------*/
//
///* Private define ------------------------------------------------------------*/
//
///* Private macro -------------------------------------------------------------*/
//
///* Private variables ---------------------------------------------------------*/
//
///* Private function prototypes -----------------------------------------------*/
//void     SystemClock_Config(void);
//void     Configure_USART(void);
//void     LED_Init(void);
//void     LED_On(uint8_t port, uint8_t pin);
//void     LED_Off(uint8_t port, uint8_t pin);
//void     LED_Blinking(uint32_t Period);
//void     UserButton_Init(void);
//static void CPU_CACHE_Enable(void);
//
///* Private functions ---------------------------------------------------------*/
//
///**
//  * @brief  Main program
//  * @param  None
//  * @retval None
//  */
//int main(void)
//{
//  /* Enable the CPU Cache */
//  CPU_CACHE_Enable();
//
//  /* Configure the system clock to 216 MHz */
//  SystemClock_Config();
//
//  /* Initialize LED1 */
//  LED_Init();
//
//  /* Set LED1 Off */
//  LED_Off();
//
//  /* Initialize button in EXTI mode */
//  UserButton_Init();
//
//  /* Configure USARTx (USART IP configuration and related GPIO initialization) */
//  Configure_USART();
//
//  printf("eg cmd: LED:01,STATE:ON... TO TURN ON GREEN LED, 02 FOR BLUE AND 03 FOR RED\r\n");
//
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
//
///**
//  * @brief  This function configures USARTx Instance.
//  * @note   This function is used to :
//  *         -1- Enable GPIO clock and configures the USART pins.
//  *         -2- NVIC Configuration for USART interrupts.
//  *         -3- Enable the USART peripheral clock and clock source.
//  *         -4- Configure USART functional parameters.
//  *         -5- Enable USART.
//  * @note   Peripheral configuration is minimal configuration from reset values.
//  *         Thus, some useless LL unitary functions calls below are provided as
//  *         commented examples - setting is default configuration from reset.
//  * @param  None
//  * @retval None
//  */
//void Configure_USART(void)
//{
//
//  /* (1) Enable GPIO clock and configures the USART pins *********************/
//
//  /* Enable the peripheral clock of GPIO Port */
//  USARTx_GPIO_CLK_ENABLE();
//
//  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
//  LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
//  USARTx_SET_TX_GPIO_AF();
//  LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
//  LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
//  LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);
//
//  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
//  LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_ALTERNATE);
//  USARTx_SET_RX_GPIO_AF();
//  LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
//  LL_GPIO_SetPinOutputType(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
//  LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);
//
//  /* (2) NVIC Configuration for USART interrupts */
//  /*  - Set priority for USARTx_IRQn */
//  /*  - Enable USARTx_IRQn */
//  NVIC_SetPriority(USARTx_IRQn, 0);
//  NVIC_EnableIRQ(USARTx_IRQn);
//
//  /* (3) Enable USART peripheral clock and clock source ***********************/
//  USARTx_CLK_ENABLE();
//
//  /* Set clock source */
//  USARTx_CLK_SOURCE();
//
//  /* (4) Configure USART functional parameters ********************************/
//
//  /* Disable USART prior modifying configuration registers */
//  /* Note: Commented as corresponding to Reset value */
//  // LL_USART_Disable(USARTx_INSTANCE);
//
//  /* TX/RX direction */
//  LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);
//
//  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
//  LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
//
//  /* No Hardware Flow control */
//  /* Reset value is LL_USART_HWCONTROL_NONE */
//  // LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);
//
//  /* Oversampling by 16 */
//  /* Reset value is LL_USART_OVERSAMPLING_16 */
//  //LL_USART_SetOverSampling(USARTx_INSTANCE, LL_USART_OVERSAMPLING_16);
//
//  /* Set Baudrate to 115200 using APB frequency set to 216000000/APB_Div Hz */
//  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
//  /* Ex :
//      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
//
//      In this example, Peripheral Clock is expected to be equal to 216000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
//  */
//  LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, LL_USART_OVERSAMPLING_16, 115200);
//
//  /* (5) Enable USART *********************************************************/
//  LL_USART_Enable(USARTx_INSTANCE);
//
//  /* Enable RXNE and Error interrupts */
//  LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
//  LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
//}
//
///**
//  * @brief  Initialize LED1.
//  * @param  None
//  * @retval None
//  */
//void LED_Init(void)
//{
//  /* Enable the LED1 Clock */
//  LED1_GPIO_CLK_ENABLE();
//
//  /* Configure IO in output push-pull mode to drive external LED1 */
//  LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
//  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
//  LL_GPIO_SetPinMode(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_MODE_OUTPUT);
//  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
//  //LL_GPIO_SetPinOutputType(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
//  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
//  //LL_GPIO_SetPinSpeed(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_SPEED_FREQ_LOW);
//  /* Reset value is LL_GPIO_PULL_NO */
//  //LL_GPIO_SetPinPull(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_PULL_NO);
//}
//
///**
//  * @brief  Turn-on LED1.
//  * @param  None
//  * @retval None
//  */
//void LED_On(uint8_t port, uint8_t pin)
//{
//  /* Turn LED1 on */
//  LL_GPIO_SetOutputPin(port, pin);
//}
//
///**
//  * @brief  Turn-off LED1.
//  * @param  None
//  * @retval None
//  */
//void LED_Off(uint8_t port, uint8_t pin)
//{
//  /* Turn LED1 off */
//  LL_GPIO_ResetOutputPin(port, pin);
//}
//
//
///**
//  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
//  * @param  None
//  * @retval None
//  */
//void UserButton_Init(void)
//{
//  /* Enable the BUTTON Clock */
//  USER_BUTTON_GPIO_CLK_ENABLE();
//
//  /* Configure GPIO for BUTTON */
//  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
//  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
//
//  /* Connect External Line to the GPIO*/
//  USER_BUTTON_SYSCFG_SET_EXTI();
//
//  /* Enable a rising trigger EXTI_Line15_10 Interrupt */
//  USER_BUTTON_EXTI_LINE_ENABLE();
//  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();
//
//  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
//  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 3);
//  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
//}
//
///**
//  * @brief  System Clock Configuration
//  *         The system Clock is configured as follow :
//  *            System Clock source            = PLL (HSE)
//  *            SYSCLK(Hz)                     = 216000000
//  *            HCLK(Hz)                       = 216000000
//  *            AHB Prescaler                  = 1
//  *            APB1 Prescaler                 = 4
//  *            APB2 Prescaler                 = 2
//  *            HSI Frequency(Hz)              = 8000000
//  *            PLL_M                          = 8
//  *            PLL_N                          = 432
//  *            PLL_P                          = 2
//  *            VDD(V)                         = 3.3
//  *            Main regulator output voltage  = Scale1 mode
//  *            Flash Latency(WS)              = 7
//  * @param  None
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  /* Enable HSE clock */
//  LL_RCC_HSE_EnableBypass();
//  LL_RCC_HSE_Enable();
//  while(LL_RCC_HSE_IsReady() != 1)
//  {
//  };
//
//  /* Set FLASH latency */
//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);
//
//  /* Enable PWR clock */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
//
//  /* Activation OverDrive Mode */
//  LL_PWR_EnableOverDriveMode();
//  while(LL_PWR_IsActiveFlag_OD() != 1)
//  {
//  };
//
//  /* Activation OverDrive Switching */
//  LL_PWR_EnableOverDriveSwitching();
//  while(LL_PWR_IsActiveFlag_ODSW() != 1)
//  {
//  };
//
//  /* Main PLL configuration and activation */
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 432, LL_RCC_PLLP_DIV_2);
//  LL_RCC_PLL_Enable();
//  while(LL_RCC_PLL_IsReady() != 1)
//  {
//  };
//
//  /* Sysclk activation on the main PLL */
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {
//  };
//
//  /* Set APB1 & APB2 prescaler */
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
//
//  /* Set systick to 1ms */
//  SysTick_Config(216000000 / 1000);
//
//  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
//  SystemCoreClock = 216000000;
//}
//
//
///******************************************************************************/
///*   IRQ HANDLER TREATMENT Functions                                          */
///******************************************************************************/
///**
//  * @brief  Function to manage Button push
//  * @param  None
//  * @retval None
//  */
//void UserButton_Callback(void)
//{
//  /* Turn LED1 Off on User button press (allow to restart sequence) */
//  LED_Off();
//}
//
///**
//  * @brief  Function called from USART IRQ Handler when RXNE flag is set
//  *         Function is in charge of reading character received on USART RX line.
//  * @param  None
//  * @retval None
//  */
//void UART_RxCpltCallback(void)
//{
//__IO uint32_t received_char;
//
//  /* Read Received character. RXNE flag is cleared by reading of RDR register */
//  received_char = LL_USART_ReceiveData8(USARTx_INSTANCE);
//
//  /* Check if received value is corresponding to specific one : S or s */
//  if ((received_char == 'S') || (received_char == 's'))
//  {
//    /* Turn LED1 On : Expected character has been received */
//    LED_On();
//  }
//
//  /* Echo received character on TX */
//  LL_USART_TransmitData8(USARTx_INSTANCE, received_char);
//}
//
///**
//  * @brief  Function called in case of error detected in USART IT Handler
//  * @param  None
//  * @retval None
//  */
//void Error_Callback(void)
//{
//  __IO uint32_t isr_reg;
//
//  /* Disable USARTx_IRQn */
//  NVIC_DisableIRQ(USARTx_IRQn);
//
//  /* Error handling example :
//    - Read USART ISR register to identify flag that leads to IT raising
//    - Perform corresponding error handling treatment according to flag
//  */
//  isr_reg = LL_USART_ReadReg(USARTx_INSTANCE, ISR);
//  if (isr_reg & LL_USART_ISR_NE)
//  {
//    /* case Noise Error flag is raised : ... */
//    LED_Blinking(LED_BLINK_FAST);
//  }
//  else
//  {
//    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
//    LED_Blinking(LED_BLINK_ERROR);
//  }
//}
//
//#ifdef  USE_FULL_ASSERT
//
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d", file, line) */
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
//#endif
///**
//  * @brief  CPU L1-Cache enable.
//  * @param  None
//  * @retval None
//  */
//static void CPU_CACHE_Enable(void)
//{
//  /* Enable I-Cache */
//  SCB_EnableICache();
//
//  /* Enable D-Cache */
//  SCB_EnableDCache();
//}
//
///**
//  * @}
//  */
//
///**
//  * @}
//  */
//



/**
  ******************************************************************************
  * @file    USART_Command_Parser/Src/main.c
  * @author  Embedded Programmer
  * @brief   Text-based command interpreter for toggling LEDs via USART
  ******************************************************************************
  * Command format: LED:XX,STATE:YYY
  * XX: LED number (01=green, 02=blue, 03=red)
  * YYY: LED state (ON or OFF)
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** @addtogroup STM32F7xx_Examples
  * @{
  */

/** @addtogroup USART_Command_Parser
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RX_BUFFER_SIZE 64

#define LED2_PIN						   LL_GPIO_PIN_7
#define LED2_GPIO_PORT					   GPIOB
#define LED2_GPIO_CLK_ENABLE()			   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

#define LED3_PIN						   LL_GPIO_PIN_14
#define LED3_GPIO_PORT					   GPIOB
#define LED3_GPIO_CLK_ENABLE()			   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint32_t RxBufferWriteIndex = 0;
uint32_t RxBufferReadIndex = 0;
uint8_t CommandComplete = 0;

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     Configure_USART(void);
void     LED_Init(void);
void     LED_On(uint8_t pin);
void     LED_Off(uint8_t pin);
void     LED_Blinking(uint32_t Period);
void     UserButton_Init(void);
static void CPU_CACHE_Enable(void);
uint8_t  IsCommandComplete(void);
void     ParseAndExecuteCommand(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Initialize LED1 */
  LED_Init();

  /* Set LEDs Off */
  LED_On(LED1_PIN);
  LED_On(LED2_PIN);
  LED_On(LED3_PIN);

  /* Initialize button in EXTI mode */
  UserButton_Init();

  /* Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();

  /* Send welcome message */
  /* Send welcome message */
    char welcome[] = "Command format: LED:XX,STATE:YYY\r\n";
    char instruc[] = "XX: 01=green, 02=blue, 03=red\r\n";
    char example[] = "Example: LED:01,STATE:ON\r\n\r\n";

    for(uint32_t i = 0; welcome[i] != '\0'; i++) {
      while(!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));
      LL_USART_TransmitData8(USARTx_INSTANCE, welcome[i]);
    }

    for(uint32_t i = 0; instruc[i] != '\0'; i++) {
      while(!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));
      LL_USART_TransmitData8(USARTx_INSTANCE, instruc[i]);
    }

    for(uint32_t i = 0; example[i] != '\0'; i++) {
      while(!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));
      LL_USART_TransmitData8(USARTx_INSTANCE, example[i]);
    }



  /* Infinite loop */
  while (1)
  {
    /* Check if a complete command is received */
    if(CommandComplete || IsCommandComplete())
    {
      ParseAndExecuteCommand();
      CommandComplete = 0;
    }
  }
}

/**
  * @brief  Check if buffer contains a complete command
  * @param  None
  * @retval 1 if command is complete, 0 otherwise
  */
uint8_t IsCommandComplete(void)
{
  /* Search for end of command marker (e.g., '\r' or '\n') */
  uint32_t readIndex = RxBufferReadIndex;

  while(readIndex != RxBufferWriteIndex)
  {
    if(RxBuffer[readIndex] == '\r' || RxBuffer[readIndex] == '\n')
    {
      return 1;
    }
    readIndex = (readIndex + 1) % RX_BUFFER_SIZE;
  }

  return 0;
}

/**
  * @brief  Parse and execute the received command
  * @param  None
  * @retval None
  */
void ParseAndExecuteCommand(void)
{
  char cmdBuffer[RX_BUFFER_SIZE];
  uint32_t cmdIndex = 0;
  uint8_t ledNum = 0;
  uint8_t ledState = 0; /* 0 for OFF, 1 for ON */

  /* Copy command from RxBuffer to cmdBuffer until end marker */
  while(RxBufferReadIndex != RxBufferWriteIndex)
  {
    uint8_t c = RxBuffer[RxBufferReadIndex];
    RxBufferReadIndex = (RxBufferReadIndex + 1) % RX_BUFFER_SIZE;

    if(c == '\r' || c == '\n')
    {
      break;
    }

    cmdBuffer[cmdIndex++] = c;
    if(cmdIndex >= RX_BUFFER_SIZE - 1)
    {
      break;
    }
  }
  cmdBuffer[cmdIndex] = '\0'; /* Null terminate the string */

  /* Parse the command - Simple version using strstr */
  char *ledPart = strstr(cmdBuffer, "LED:");
  char *statePart = strstr(cmdBuffer, "STATE:");

  if(ledPart && statePart)
  {
    /* Extract LED number */
    if(isdigit(ledPart[4]) && isdigit(ledPart[5])) {
      ledNum = (ledPart[4] - '0') * 10 + (ledPart[5] - '0');
    }

    /* Extract STATE (ON/OFF) */
    if(statePart[6] == 'O' && statePart[7] == 'N')
    {
      ledState = 1; /* ON */
    }
    else if(statePart[6] == 'O' && statePart[7] == 'F' && statePart[8] == 'F')
    {
      ledState = 0; /* OFF */
    }

    /* Control the LED based on parsed values */
    switch(ledNum)
    {
      case 1: /* Green LED */
        if(ledState)
          LED_On(LED1_PIN);
        else
          LED_Off(LED1_PIN);
        break;

      case 2: /* Blue LED */
        if(ledState)
          LED_On(LED2_PIN);
        else
          LED_Off(LED2_PIN);
        break;

      case 3: /* Red LED */
        if(ledState)
          LED_On(LED3_PIN);
        else
          LED_Off(LED3_PIN);
        break;

      default:
        /* Invalid LED number */
        break;
    }

    /* Send confirmation message */
    char msg[50];
    sprintf(msg, "Command executed: LED:%02d,STATE:%s\r\n",
            ledNum, ledState ? "ON" : "OFF");

    /* Send message character by character */
    for(uint32_t i = 0; msg[i] != '\0'; i++)
    {
      /* Wait until TXE flag is set */
      while(!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));

      /* Send character */
      LL_USART_TransmitData8(USARTx_INSTANCE, msg[i]);
    }
  }
  else {
    /* Invalid command format */
    char error[] = "Invalid command. Format: LED:XX,STATE:YYY\r\n";

    for(uint32_t i = 0; error[i] != '\0'; i++) {
      while(!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE));
      LL_USART_TransmitData8(USARTx_INSTANCE, error[i]);
    }
  }
}

/**
  * @brief  This function configures USARTx Instance.
  * @note   This function is used to :
  *         -1- Enable GPIO clock and configures the USART pins.
  *         -2- NVIC Configuration for USART interrupts.
  *         -3- Enable the USART peripheral clock and clock source.
  *         -4- Configure USART functional parameters.
  *         -5- Enable USART.
  * @param  None
  * @retval None
  */
void Configure_USART(void)
{
  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  USARTx_GPIO_CLK_ENABLE();

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  USARTx_SET_TX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  USARTx_SET_RX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USARTx_IRQn, 0);  
  NVIC_EnableIRQ(USARTx_IRQn);

  /* (3) Enable USART peripheral clock and clock source ***********************/
  USARTx_CLK_ENABLE();

  /* Set clock source */
  USARTx_CLK_SOURCE();

  /* (4) Configure USART functional parameters ********************************/
  /* Disable USART prior modifying configuration registers */
  LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  LL_USART_SetOverSampling(USARTx_INSTANCE, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 216000000/APB_Div Hz */
  LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, LL_USART_OVERSAMPLING_16, 115200); 

  /* (5) Enable USART *********************************************************/
  LL_USART_Enable(USARTx_INSTANCE);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
  LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
}

/**
  * @brief  Initialize LED1.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED1 Clock */
  LED1_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED1 */
  LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinPull(LED3_GPIO_PORT, LED3_PIN, LL_GPIO_PULL_NO);
}

/**
  * @brief  Turn-on LED.
  * @param  port: GPIO port
  * @param  pin: GPIO pin
  * @retval None
  */
void LED_On(uint8_t pin)
{
  /* Turn LED on */
  LL_GPIO_SetOutputPin(GPIOB, pin);
}

/**
  * @brief  Turn-off LED.
  * @param  port: GPIO port
  * @param  pin: GPIO pin
  * @retval None
  */
void LED_Off(uint8_t pin)
{
  /* Turn LED off */
  LL_GPIO_ResetOutputPin(GPIOB, pin);
}

/**
  * @brief  Set LED to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED1 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
    LL_mDelay(Period);
  }
}

/**
  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
  * @param  None 
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();

  /* Enable a rising trigger EXTI_Line15_10 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 3);  
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn); 
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Enable HSE clock */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);

  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Activation OverDrive Mode */
  LL_PWR_EnableOverDriveMode();
  while(LL_PWR_IsActiveFlag_OD() != 1)
  {
  };

  /* Activation OverDrive Switching */
  LL_PWR_EnableOverDriveSwitching();
  while(LL_PWR_IsActiveFlag_ODSW() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 432, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  /* Set systick to 1ms */
  SysTick_Config(216000000 / 1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 216000000;
}

/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/
/**
  * @brief  Function to manage Button push
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Reset all LEDs */
  LED_Off(LED1_PIN);
  LED_Off(LED2_PIN);
  LED_Off(LED3_PIN);
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void UART_RxCpltCallback(void)
{
  __IO uint32_t received_char;

  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  received_char = LL_USART_ReceiveData8(USARTx_INSTANCE);

  /* Store the character in the buffer */
  RxBuffer[RxBufferWriteIndex] = received_char;
  RxBufferWriteIndex = (RxBufferWriteIndex + 1) % RX_BUFFER_SIZE;

  /* Check if command is complete */
  if(received_char == '\r' || received_char == '\n')
  {
    CommandComplete = 1;
  }

  /* Echo received character on TX */
  LL_USART_TransmitData8(USARTx_INSTANCE, received_char);
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USARTx_INSTANCE, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */
