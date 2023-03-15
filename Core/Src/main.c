/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEDSPERLINE 300
#define NUMLEDS (8*LEDSPERLINE)

// 3 bytes per bit, 24 bits per led plus padding on each end
#define DMAHEAD 4
#define DMATRAIL 140
#define DMALEN (DMAHEAD + LEDSPERLINE * 3 * 24 + DMATRAIL)

#define USBLINESIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned char leds[NUMLEDS*3];
struct {
  unsigned char header[DMAHEAD];
  unsigned char data[LEDSPERLINE * 3 * 24];
  unsigned char trailer[DMATRAIL];
} dmabuffer[2]; // 2 copies for double buffering

int active_buffer=0;
int ledsupdated=0;

const uint8_t gamma8[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
    2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
    10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
    37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
    51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
    69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
    90, 92, 93, 95, 96, 98, 99,101, 102,104,105,107,109,110,112,114,
    115,117,119,120,122,124,126,127, 129,131,133,135,137,138,140,142,
    144,146,148,150,152,154,156,158, 160,162,164,167,169,171,173,175,
    177,180,182,184,186,189,191,193, 196,198,200,203,205,208,210,213,
    215,218,220,223,225,228,231,233, 236,239,241,244,247,249,252,255 };

const uint8_t gamma5[32] = {
    0,1,2,3,4,5,6,7,8,10,13,16,20,25,30,36,
    43, 50, 59, 68, 78,89, 101, 114, 127, 142, 158, 175, 193, 213, 233, 236 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *data, int len) {
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
    errno = EBADF;
    return -1;
  }
#if 0
  int left = len;
  while (left--) {
    while (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE));
    huart3.Instance->DR = *(data++);
  }

  while (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE));
  while (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC));
#else
  CDC_Transmit_FS((uint8_t*)data, len);
#endif
  return len;
}

void blank_leds() {
  bzero(&dmabuffer, sizeof(dmabuffer));
  bzero(leds,sizeof(*leds));
  for (int b = 0; b < 2; b++)
    for (int i = 0; i < LEDSPERLINE; i++)
      for (int j = 0; j < 24; j++)
        dmabuffer[b].data[(i * 24 + j) * 3 + 0]=0xff;
  // Notably no need to set the zero bytes
}

// This routine updates single led into dmabuffer
// notably utilizing this does not properly work with double buffering
void update_led1(int n){
  const unsigned char bitset = 1 << (n / LEDSPERLINE);
  const int i = n % LEDSPERLINE;
  for (int j=0; j<3; j++) // RGB
    for (int k=0; k < 8; k++) {
      if (leds[n * 3 +j] & (128>>k))
        dmabuffer[active_buffer].data[(i * 24 + j * 8 + k) * 3 +1] |= bitset;
      else
        dmabuffer[active_buffer].data[(i * 24 + j * 8 + k) * 3 +1] &= ~bitset;
    }
}

// This routine updates the whole dmabuffer
void update_leds() {
  for (int i=0; i<LEDSPERLINE; i++)
    for (int j=0; j<3; j++) // RGB
      for (int k=0; k < 8; k++) {
        unsigned char bit = 128>>k;
        int offset = i * 3 +j;
        dmabuffer[active_buffer].data[(i*24 + j * 8 + k) * 3 +1] =
              ((leds[LEDSPERLINE*0*3 + offset] & bit) ? 1 : 0 )+
              ((leds[LEDSPERLINE*1*3 + offset] & bit) ? 2 : 0 )+
              ((leds[LEDSPERLINE*2*3 + offset] & bit) ? 4 : 0 )+
              ((leds[LEDSPERLINE*3*3 + offset] & bit) ? 8 : 0 )+
              ((leds[LEDSPERLINE*4*3 + offset] & bit) ? 16 : 0 )+
              ((leds[LEDSPERLINE*5*3 + offset] & bit) ? 32 : 0 )+
              ((leds[LEDSPERLINE*6*3 + offset] & bit) ? 64 : 0 )+
              ((leds[LEDSPERLINE*7*3 + offset] & bit) ? 128 : 0);
      }
}

void ledsetrgb(int led, unsigned char r, unsigned char g, unsigned char b) {
  if (led < NUMLEDS) {
    leds[led*3+0]=r;
    leds[led*3+1]=g;
    leds[led*3+2]=b;
  }
}

void usbline(char *line){
  int a,b,c,d;
  if (4 == sscanf(line,"%x %x %x %x",&a,&b,&c,&d)) {
    ledsetrgb(a,b,c,d);
    ledsupdated=1;
  }
}

void usbbyte(unsigned char c) {
#if 0
  static char usblinebuf[USBLINESIZE+1];
  static int  usblinelen = 0;

  switch (c) {
    case '\n':
    case '\r':
    case 0:
      if (usblinelen) {
        usblinebuf[usblinelen]=0;
        usbline(usblinebuf);
        usblinelen=0;
      }
      break;
    default:
      if (usblinelen < USBLINESIZE )
        usblinebuf[usblinelen++] = c;
      break;
  }
#else
  static char prevbyte = 0;
  static int hibyte = 0;
  static int ledno = -1;
  if ((prevbyte == 0xff) && (c == 0xff)) {
    // start of frame
    hibyte = 1;
    ledno = 0;
  } else if (hibyte) {
    // hibyte on packet
    if (c & 0x80) {
      // packet termination
      if (c != 0xff)
        ledsupdated = c - 0x7f;
      ledno=0;
    } else {
      hibyte = 0;
    }
  } else {
    // lobyte on packet
    hibyte = 1;
    unsigned short rgb15 = (prevbyte << 8) | (c);
    ledsetrgb(ledno++, gamma5[(rgb15>>10)&31], gamma5[(rgb15>>5)&31], gamma5[(rgb15)&31]);
  }
  prevbyte = c;
#endif
}

void usbdatain(unsigned char *buf, int len) {
  for (int i=0;i<len;i++)
    usbbyte(buf[i]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  blank_leds();
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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&dmabuffer[active_buffer], (uint32_t)&GPIOD->ODR, DMALEN);
    active_buffer=!active_buffer; // switch buffer to write to
    while (!ledsupdated) HAL_Delay(1);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    if (ledsupdated == 2) {
      blank_leds();
      ledsupdated = 0;
    } else if (ledsupdated == 3){
      // FADEOUT
      for (int i=0; i<NUMLEDS*3; i++)
        leds[i]>>=1;
    }
    if (ledsupdated) {
      ledsupdated = 0;
      update_leds();
    }
    // Ensure previous transfer has completed
    HAL_DMA_PollForTransfer(&hdma_tim1_up, HAL_DMA_FULL_TRANSFER, 50000);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 38;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  /* USER CODE END TIM1_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LEDS1_Pin|LEDS2_Pin|LEDS3_Pin|LEDS4_Pin
                          |LEDS5_Pin|LEDS6_Pin|LEDS7_Pin|LEDS8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDS1_Pin LEDS2_Pin LEDS3_Pin LEDS4_Pin
                           LEDS5_Pin LEDS6_Pin LEDS7_Pin LEDS8_Pin */
  GPIO_InitStruct.Pin = LEDS1_Pin|LEDS2_Pin|LEDS3_Pin|LEDS4_Pin
                          |LEDS5_Pin|LEDS6_Pin|LEDS7_Pin|LEDS8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
