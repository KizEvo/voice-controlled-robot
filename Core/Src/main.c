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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct INT_FLAGS{
	uint8_t button;
	uint8_t halfCompleteCB;
	uint8_t completeCB;
	uint8_t dataReceived;
};

struct INT_FLAGS flags;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 128
#define FFT_LENGTH 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
void toggleAudioStream();
void transformHalfCompletedReceivedAudioStream();

void transformCompletedReceivedAudioStream();
void fftTransform();
void processData();

float complexABS(float real, float compl);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Initialize array of 4 uint16 bit buffer
uint16_t receiveBuffer[BUFFER_SIZE];
static volatile uint16_t *rxBufferPtr;

// Initialize FFT IN array
float fftBufferIn[BUFFER_SIZE / 2];

// Initialize FFT OUT array
float fftBufferOut[BUFFER_SIZE / 2];
//static volatile float *fftBufferOutPtr;

// Initialize index for array
int bufferIndex;

// RFFT fast instance
arm_rfft_fast_instance_f32 fft_handler;

// Initialize Freq (Hz) array

int check;

int state;

int freqs[FFT_LENGTH / 2];

int fftIndex;

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
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  // Initialize rfft
  arm_rfft_fast_init_f32(&fft_handler, FFT_LENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Clear buffer index after each loop
	  bufferIndex = 0;

	  if(flags.button == 1) toggleAudioStream();
//	  if(flags.halfCompleteCB == 1) transformHalfCompletedReceivedAudioStream();
//	  if(flags.completeCB == 1) transformCompletedReceivedAudioStream();
	  if(flags.dataReceived == 1) processData();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INT_STATE_GPIO_Port, INT_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : INT_STATE_Pin */
  GPIO_InitStruct.Pin = INT_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INT_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_MIC_STATE_Pin */
  GPIO_InitStruct.Pin = LED_MIC_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_MIC_STATE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Check if GPIO_Pin is not PIN 0
	if(GPIO_Pin != GPIO_PIN_0) return;
	// Set button pressed flags
	flags.button = 1;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	// Set half complete callback of I2S_Receive
//	flags.halfCompleteCB = 1;
	rxBufferPtr = &receiveBuffer[0];
	fftIndex = 0;

	flags.dataReceived = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	// Set full complete callback of I2S_Receive
//	flags.completeCB = 1;
	rxBufferPtr = &receiveBuffer[BUFFER_SIZE / 2];
	fftIndex = BUFFER_SIZE / 4;

	flags.dataReceived = 1;
}

void toggleAudioStream()
{
	// Check for I2S state to turn on/off accordingly
	state = HAL_I2S_GetState(&hi2s2);
	if(state > HAL_I2S_STATE_READY){
		// Stops the audio DMA Stream/Channel playing from the Media
		HAL_I2S_DMAStop(&hi2s2);
		// Clear LED state
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		// Clear callback flags
//		flags.halfCompleteCB = 0;
//		flags.completeCB = 0;
		flags.dataReceived = 0;
	} else {
		// Enable I2S Receive DMA
		HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *) receiveBuffer, BUFFER_SIZE);
		// Set LED state
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
	// Clear button flag
	flags.button = 0;
}

void processData(){

	for(uint8_t i = 0; i < BUFFER_SIZE; i += 2){
		fftBufferIn[bufferIndex] = (1.0 / 65536.0) * rxBufferPtr[i];

		bufferIndex++;
	}

	fftTransform();

	flags.dataReceived = 0;
}

void transformHalfCompletedReceivedAudioStream()
{
	// Prepare data for FFT buffer input from 0 to 511
	uint16_t i = 0;
	while(bufferIndex < BUFFER_SIZE / 2){
		fftBufferIn[i++] = (float) ((int) receiveBuffer[bufferIndex]);
		// I2S DMA only works in Stereo mode => receiveBuffer receives left channel
		// first then right channel.
		// We only need left channel so need to increment index by 2
		bufferIndex += 2;
	}

	// Call FFT transform function
	fftTransform();
	// Clear half complete callback flag
	flags.halfCompleteCB = 0;
}

void transformCompletedReceivedAudioStream()
{
	// Prepare data for FFT buffer input from 512 to 1023
	uint16_t i = 0;
	while(bufferIndex < BUFFER_SIZE){
		fftBufferIn[i++] = (float) ((int) receiveBuffer[bufferIndex]);
		// I2S DMA only works in Stereo mode => receiveBuffer receives left channel
		// first then right channel.
		// We only need left channel so need to increment index by 2
		bufferIndex += 2;
	}

	// Call FFT transform function
	fftTransform();
	// Clear complete callback flag
	flags.completeCB = 0;
}

float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}

void fftTransform(){

	arm_rfft_fast_f32(&fft_handler,(float32_t *) fftBufferIn,(float32_t *) fftBufferOut, 0); // 0 means Time domain -> Freq Domain

	uint16_t freqIndex = 0;

	for(uint16_t i = 0; i < BUFFER_SIZE / 2; i += 2){
		freqs[freqIndex] = (int)(20*log10f(complexABS(fftBufferOut[i], fftBufferOut[i+1])));
		if(freqs[freqIndex] < 0) freqs[freqIndex] = 0;
		freqIndex++;
	}

	check = freqs[1]; // 1 = 1488(Hz) * 32 / (47619 / 2)
}

// Test config for printf debug
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
