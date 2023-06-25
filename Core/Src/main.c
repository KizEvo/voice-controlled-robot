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
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct INT_FLAGS{
	uint8_t audioButton;
	uint8_t recordButton;
	uint8_t recordTimer;
	uint8_t dataReceived;
	uint8_t voiceControl;
	uint8_t dataReceivedUART;
};

struct INT_FLAGS flags;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMIT_TOTAL_RECORDS_COUNT 54
#define RESET_RECORDS_COUNT 55
#define START_RECORD 56
#define START_VOICE_RECOGNITION 57

#define BUFFER_SIZE 1024 // 128
#define FFT_LENGTH 512 // 64
#define DEBOUNCE_TIME_MS 20
#define MAX_RECORD_COUNT 2
#define MAX_FREQ_AND_GAIN 20 // If the number of dB stored is 10 then it must be doubled
							 // => 20 to keep track of which dB belongs to a certain FREQS
const uint8_t EEPROM_NEXT_RECORD_ADDRESS = 20;
const uint16_t EEPROM_RECORDS_COUNT_ADDRESS = 32767;
const uint16_t EEPROM_DEFAULT_RECORDS_AUDIO_ADDRESS = 0;
const uint16_t EEPROM_ADDRESS = 0xA0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// RFFT fast instance
arm_rfft_fast_instance_f32 fft_handler;

// Initialize array of 4 uint16 bit buffer
uint16_t receiveBuffer[BUFFER_SIZE];
static volatile uint16_t *rxBufferPtr;

// Initialize FFT IN array
float32_t fftBufferIn[BUFFER_SIZE / 2];
uint16_t fftBufferIndex;

// Initialize FFT OUT array
//float32_t fftBufferOut[BUFFER_SIZE / 2];

// Initialize FREQS (Hz) array
int freqs[FFT_LENGTH / 4];
uint8_t storedFreqs[MAX_RECORD_COUNT][FFT_LENGTH / 4];
uint8_t highestFreqs[MAX_RECORD_COUNT][MAX_FREQ_AND_GAIN];
uint8_t averageHighestFreqs[MAX_FREQ_AND_GAIN];

// UART Tx data
uint8_t rxData[1];

// Debounce button time
uint32_t currentTimeInMillis = 0;
uint32_t previousTimeInMillis = 0;

// Numb of time audio got recorded while STM32 is turned on
// this will be reset when STM32 is turned off
uint8_t recordsCount;

// Numb of records that were saved in EEPROM
uint8_t recordsAudioCount[1];

// FFT complex struct
const float pi = 3.14159265358979323846264338327950288419716939937510;
struct complex {
	float re;
	float im;
};
struct complex b[FFT_LENGTH],c[FFT_LENGTH];

// State of I2S
int stateI2S;

// State of audio record sequence
uint8_t stateRecordAudio;

// ** TESTING **
uint8_t resultAvg = 2;
uint8_t readAfter;
uint8_t readBefore;
uint8_t arrayF[MAX_FREQ_AND_GAIN];
uint8_t toWrite[3] = {1, 4, 5};
uint8_t testTotal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void toggleAudioStream();
void fftTransform();
void processData();
void enableRecordAudio();
void setLedSuccess();
uint8_t store10HighestFreqsdB();
void startVoiceRecognition();
void startRecordSequence();
void processDataUART();
uint8_t writeEEPROM(uint8_t *dataArray, uint16_t addressToWrite, uint16_t size);
uint8_t readEEPROM(uint8_t *dataArray, uint16_t addressToRead, uint16_t size);
uint8_t getValidKeys();
uint8_t startRecordAudio();
float complexABS(float real, float compl);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct complex w(float kn, float N)
{
	struct complex A;
	A.re = cos(2*pi*(kn/N));
	A.im = -sin(2*pi*(kn/N));
	return A;
}

int reverseBit(int a)
{
	int b = 0;
	for(int i = 0; i < log2f(FFT_LENGTH); i++)
	{
		b = b << 1;
		if ((a & 1) == 1)
		{
			b = b | 1;
		}
		a = a >> 1;
	}
	return b;
}

void fft(float* input,int length)
{
	struct complex pro;
	for(int i = 0; i < length; i++)
	{
		b[i].re = input[reverseBit(i)];
		b[i].im = 0;
	}
	for (int step = 0 ; (1 << step) < length; step ++)
	{
		for(int i = 0; i < length ; i ++)
		{
			if((i & (1 << step )) == 0)
			{
				pro.re = b[i+ (1 << step)].re * w((i+ (1 << step)) % (1 << step),pow(2,step+1)).re -  b[i+ (1 << step)].im * w((i+ (1 << step)) % (1 << step),pow(2,step+1)).im;
				pro.im = b[i+ (1 << step)].im * w((i+ (1 << step)) % (1 << step),pow(2,step+1)).re +  b[i+ (1 << step)].re * w((i+ (1 << step)) % (1 << step),pow(2,step+1)).im;
				b[i + (1 << step)].re = b[i].re - pro.re;
				b[i + (1 << step)].im = b[i].im - pro.im;
				b[i].re += pro.re;
				b[i].im += pro.im;
			}
		}
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  flags.recordTimer = 1;
  flags.recordButton = 0;
  flags.audioButton = 0;

  HAL_UART_Receive_IT(&huart1, (uint8_t *) rxData, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Clear FFT buffer index after each iteration
	  fftBufferIndex = 0;

	  if(flags.dataReceivedUART == 1) processDataUART();
	  if(flags.audioButton == 1) toggleAudioStream();
	  if(flags.recordButton == 1) startRecordSequence();
	  if(flags.dataReceived == 1) processData();
	  if(flags.voiceControl == 1) startVoiceRecognition();
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 95;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, INT_STATE_Pin|RECORD_STATE_Pin|LED_MIC_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A_STATE_Pin|B_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_RECORD_STATE_Pin|WRITE_RECORD_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : INT_STATE_Pin */
  GPIO_InitStruct.Pin = INT_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INT_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RECORD_STATE_Pin LED_MIC_STATE_Pin */
  GPIO_InitStruct.Pin = RECORD_STATE_Pin|LED_MIC_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A_STATE_Pin B_STATE_Pin */
  GPIO_InitStruct.Pin = A_STATE_Pin|B_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_RECORD_STATE_Pin WRITE_RECORD_STATE_Pin */
  GPIO_InitStruct.Pin = RESET_RECORD_STATE_Pin|WRITE_RECORD_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	// Set pointer points to the memory of the first index of receiveBuffer
	// Set dataReceived flag
	rxBufferPtr = &receiveBuffer[0];
	flags.dataReceived = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	// Set pointer points to the memory of half the length of receiveBuffer
	// Set dataReceived flag
	rxBufferPtr = &receiveBuffer[BUFFER_SIZE / 2];
	flags.dataReceived = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5) {
		if(flags.recordTimer == 0) flags.recordTimer = 1;
		else flags.recordTimer = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) flags.dataReceivedUART = 1;
}

void toggleAudioStream()
{
	// Check for I2S state to turn on/off
	stateI2S = HAL_I2S_GetState(&hi2s2);
	if(stateI2S > HAL_I2S_STATE_READY){
		// Stops the audio DMA Stream/Channel playing from the Media
		HAL_I2S_DMAStop(&hi2s2);
		// Clear LED state
		HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_RESET);
		// Clear callback flag
		flags.dataReceived = 0;
	} else {
//		Enable I2S Receive DMA
//		HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *) receiveBuffer, BUFFER_SIZE);
		HAL_GPIO_WritePin(INT_STATE_GPIO_Port, INT_STATE_Pin, GPIO_PIN_RESET);

		recordsCount = 0;
		flags.recordButton = 1;

		enableRecordAudio();
		startRecordAudio();

		flags.recordButton = 0;
		flags.voiceControl = 1;
		// Set LED state
		HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_SET);
	}
	// Clear audioButton flag
	flags.audioButton = 0;
}

void processData()
{
	// Loop through receiveBuffer, only get the left channel in buffer array
	// which is EVEN index and place the result in fftBufferIn
	for(uint16_t i = 0; i < BUFFER_SIZE; i += 2){
		// Transform uint16_t to float by dividing 65536.0
		fftBufferIn[fftBufferIndex] = (1.0 / 65536.0) * rxBufferPtr[i];
		// Increment fftBufferIndex
		fftBufferIndex++;
	}
	// FFT processing
	fftTransform();
	// Clear dataReceived flag
	flags.dataReceived = 0;
}

void fftTransform()
{
	// Transform real FFT, 0 means time domain to frequency domain
	// Return array contains real ([i]) and complex ([i + 1]) number
	//	arm_rfft_fast_f32(&fft_handler, (float32_t *) fftBufferIn, (float32_t *) fftBufferOut, 0);
	fft(fftBufferIn, FFT_LENGTH);

	// Calculate ABS value of complex vectors and translate into log dB range
	// Equation: 20 * log10( sqrt(real * real + complex * complex) );
	// Store the result in freqs array
	// Formula to check which index (index of freqs[]) correspond to a certain frequency:
	// INDEX = FREQ * (FFT_LENGTH / 4) / (F_SAMPLE / 2)
	// EX: 12 = 3000 * (64 / 2) / (16000 / 2);

	for(uint16_t i = 0; i < FFT_LENGTH / 4; i++){
//		freqs[i] = (int)(20.0 * log10f(sqrtf(b[i].re * b[i].re + b[i].im * b[i].im)));
		freqs[i] = (int)(b[i].re * b[i].re + b[i].im * b[i].im);
		freqs[i] = floor(freqs[i] / FFT_LENGTH);
		if(flags.recordButton == 1 && flags.recordTimer == 1) storedFreqs[recordsCount][i] = (uint8_t) freqs[i];
	}
}

void enableRecordAudio()
{
	// Start TIM5 2 seconds timer
	HAL_TIM_Base_Start_IT(&htim5);
}

uint8_t startRecordAudio()
{
	uint8_t isNoErrorWhileRecording = 1;

	// Start record audio sequence, wait 2 second
	// Set RECORD_STATE_Pin
	HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_SET);
	while(flags.recordTimer == 0);

	// Enable I2S Receive DMA
	HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *) receiveBuffer, BUFFER_SIZE);
	// Clear RECORD_STATE_Pin
	HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_RESET);
	while(flags.recordTimer == 1) {
		fftBufferIndex = 0;
		if(flags.dataReceived == 1) processData();
	}

	// Stops the audio DMA Stream/Channel playing from the Media
	stateI2S = HAL_I2S_GetState(&hi2s2);
	if(stateI2S > HAL_I2S_STATE_READY) HAL_I2S_DMAStop(&hi2s2);

	// Clear the audio record LED state

	HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_SET);
	while(flags.recordTimer == 0);

	// Stop TIM5
	HAL_TIM_Base_Stop_IT(&htim5);

	// Clear RECORD_STATE Led
	HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_RESET);
	// Clear record timer flags
	flags.recordTimer = 0;
	// Clear dataReceived flag
	flags.dataReceived = 0;
	// Return status of record audio sequence
	return isNoErrorWhileRecording;
}

uint8_t store10HighestFreqsdB(){
	uint8_t totalFreqs = 0, tempFreqsIndex = 0, correctFreqsIndex = 0;
	uint8_t tempFreqs[MAX_FREQ_AND_GAIN];

	for(uint16_t freqSampledIndex = 0; freqSampledIndex < MAX_RECORD_COUNT; freqSampledIndex++){
		// First loop to get MAX_RECORD_COUNT storedFreqs[freqSampledIndex][x]; x means don't care
		for(uint16_t magniIndex = 0; magniIndex < FFT_LENGTH / 4; magniIndex++){
			// Second loop to check dB gain for a storedFreqs[x][dBIndex] sample; x means dont't care
			if(storedFreqs[freqSampledIndex][magniIndex] > 0 && freqSampledIndex <= 0) {
				tempFreqs[tempFreqsIndex] = magniIndex;
				tempFreqs[tempFreqsIndex + 1] = storedFreqs[freqSampledIndex][magniIndex];
				totalFreqs++;
				tempFreqsIndex += 2;
			} else if(storedFreqs[freqSampledIndex][magniIndex] > 0 && freqSampledIndex > 0){
				uint8_t isMatch = 0;

				for(uint8_t i = 0; i < tempFreqsIndex; i += 2){
					isMatch = tempFreqs[i] == magniIndex ? 1 : 0;

					if(isMatch >= 1) {
						averageHighestFreqs[correctFreqsIndex] = magniIndex;
						averageHighestFreqs[correctFreqsIndex + 1] = (storedFreqs[freqSampledIndex][magniIndex] + tempFreqs[i + 1] ) / 2;
						correctFreqsIndex += 2;
						break;
					}

				}
			}

			if(correctFreqsIndex >= 20 && freqSampledIndex > 0) break;
			if(totalFreqs >= 10 && freqSampledIndex <= 0) break;
		}
	}

	readAfter = correctFreqsIndex;
	readBefore = tempFreqsIndex;

	return 100 * readAfter / readBefore;
}

void startRecordSequence(){
	// Clear previous records counts
	HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	recordsCount = 0;
	// Clear previous recorded/processed arrays
	for(uint16_t i = 0; i < MAX_FREQ_AND_GAIN; i++){
	  averageHighestFreqs[i] = 0;
	}
//	for(uint16_t col = 0; col < MAX_RECORD_COUNT; col++){
//	  for(uint16_t row = 0; row < MAX_FREQ_AND_GAIN; row++){
//		  highestFreqs[col][row] = 0;
//	  }
//	}

	// Record audio MAX_RECORD_COUNT time
	while(recordsCount < MAX_RECORD_COUNT){
	  enableRecordAudio();
	  stateRecordAudio = startRecordAudio();

	  if(stateRecordAudio == 1) setLedSuccess();
	  else break;

	  recordsCount++;
	}

	resultAvg = store10HighestFreqsdB();

	// If successful then proceed to store highest FREQS got from records
	if(resultAvg >= 70 && stateRecordAudio == 1) {
	  writeEEPROM(averageHighestFreqs, EEPROM_DEFAULT_RECORDS_AUDIO_ADDRESS, sizeof(averageHighestFreqs));
	  HAL_Delay(500);
	  // Increment when write to EEPROM return successfully
	  readEEPROM(recordsAudioCount, EEPROM_RECORDS_COUNT_ADDRESS, sizeof(recordsAudioCount));
	  HAL_Delay(500);
	  recordsAudioCount[0]++;
	  writeEEPROM(recordsAudioCount, EEPROM_RECORDS_COUNT_ADDRESS, sizeof(recordsAudioCount));
	  flags.recordButton = 0;
	}
}

void setLedSuccess(){
	uint16_t delayTime = 500;
	for(uint8_t i = 0; i < 10; i++){
		HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_SET);
		HAL_Delay(delayTime);
		HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		delayTime -= 50;
	}
	HAL_Delay(1000);

}

void startVoiceRecognition(){
	uint8_t keyMatches = 0, totalKey = 0, freqIndex = 0,
			previousMatchPercent = 1, currentMatchPercent = 0, prevHighestMagni = 0,
			indexHighestMagni = 0, indexHighestEEPROMMagni = 0, prevHighestEEPROMMagni = 0,
			totalStored = 0, status;

	flags.voiceControl = 0;

	// Clear previous recorded/processed arrays
	for(uint16_t i = 0; i < MAX_FREQ_AND_GAIN; i++){
	  averageHighestFreqs[i] = 0;
	}
	HAL_GPIO_WritePin(A_STATE_GPIO_Port, A_STATE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(B_STATE_GPIO_Port, B_STATE_Pin, GPIO_PIN_RESET);

	// store record counts to recordsAudioCount[]
	status = readEEPROM(recordsAudioCount, EEPROM_RECORDS_COUNT_ADDRESS, sizeof(recordsAudioCount));
	HAL_Delay(100);
	if(status != HAL_OK) return;

	totalStored = recordsAudioCount[0];

	if(totalStored <= 0) return;


	while(totalStored > 0){
//		if(isCorrectAudio == 1) break;

		// decrement recordsAudioCount so that readEEPROM can calc the
		// correct address
		totalStored--;
		recordsAudioCount[0] = totalStored;

		// store keys to averageHighestFreqs[]
		readEEPROM(averageHighestFreqs, EEPROM_DEFAULT_RECORDS_AUDIO_ADDRESS, sizeof(averageHighestFreqs));
		HAL_Delay(100);

		keyMatches = 0;
		freqIndex = 0;
		totalKey = 0;

		for(uint8_t avgIndex = 0; avgIndex < MAX_FREQ_AND_GAIN; avgIndex += 2){
			freqIndex = averageHighestFreqs[avgIndex];

			if(freqIndex == 0 && (freqIndex + 1) == 0) break;
			if(storedFreqs[0][freqIndex] > 0) {

				if(storedFreqs[0][freqIndex] > prevHighestMagni && freqIndex != 0) {
					prevHighestMagni = storedFreqs[0][freqIndex];
					indexHighestMagni = freqIndex;
				}

				keyMatches++;
			}

			if(averageHighestFreqs[avgIndex + 1] > prevHighestEEPROMMagni && avgIndex != 0) {
				prevHighestEEPROMMagni = averageHighestFreqs[avgIndex + 1];
				indexHighestEEPROMMagni = freqIndex;
			}

			totalKey++;
		}

		readBefore = indexHighestEEPROMMagni;
		readAfter = indexHighestMagni;

		currentMatchPercent = 100 * keyMatches / totalKey;
		if(currentMatchPercent > previousMatchPercent
				&& currentMatchPercent >= 80
				&& indexHighestEEPROMMagni == indexHighestMagni) {
			previousMatchPercent = currentMatchPercent;
			if(totalStored == 0) HAL_GPIO_WritePin(A_STATE_GPIO_Port, A_STATE_Pin, GPIO_PIN_SET);
			else if(totalStored == 1) HAL_GPIO_WritePin(B_STATE_GPIO_Port, B_STATE_Pin, GPIO_PIN_SET);
		}

		prevHighestEEPROMMagni = 0;
		prevHighestMagni = 0;

//		uint8_t totalKey = getValidKeys();
//		if(totalKey <= 0) return;

//		for(uint8_t i = 0; i < totalKey * 2; i += 2){
//		  for(uint16_t freqIndex = 0; freqIndex < FFT_LENGTH / 4; freqIndex++){
//			  if(storedFreqs[0][freqIndex] > temp_dB) {
//				  temp_dB = storedFreqs[0][freqIndex];
//				  tempIndex = freqIndex;
//			  }
//		  }
//
//		  if(averageHighestFreqs[i] == tempIndex){
//			  uint8_t diff = temp_dB > averageHighestFreqs[i + 1] ?
//					  (temp_dB - averageHighestFreqs[i + 1]) : (averageHighestFreqs[i + 1] - temp_dB);
//			  if(diff > 5) break;
//
//			  correctMatches++;
//		  }
//
//		  // if totalKey > 2 then the only need to match totalKey - correctMatches <= 1
//		  // else to found match audio correctMatches have to equal totalKey
//		  if(totalKey - correctMatches <= 1 && totalKey > 2) {
//			  isCorrectAudio = 1;
//			  break;
//		  } else if (totalKey - correctMatches <= 0 && totalKey <= 2){
//			  isCorrectAudio = 1;
//			  break;
//		  }
//
//		  storedFreqs[0][tempIndex] = 0;
//		  temp_dB = 0;
//		  tempIndex = 0;
//		}
	}

	if(previousMatchPercent >= 80) setLedSuccess();
}

uint8_t getValidKeys(){
	uint8_t totalValidKeys = 0;
	for(uint8_t i = 0; i < MAX_FREQ_AND_GAIN; i += 2){
	  if(averageHighestFreqs[i] == 0 && averageHighestFreqs[i + 1] == 0) break;
	  totalValidKeys++;
	}

	return totalValidKeys;
}

void processDataUART(){
	uint8_t rxDataUART;
	flags.dataReceivedUART = 0;

	rxDataUART = rxData[0];

	if(rxDataUART == RESET_RECORDS_COUNT) {
		recordsAudioCount[0] = 0;
		writeEEPROM(recordsAudioCount, EEPROM_RECORDS_COUNT_ADDRESS, sizeof(recordsAudioCount));
	} else if(rxDataUART == START_RECORD) {
		flags.recordButton = 1;
	} else if(rxDataUART == START_VOICE_RECOGNITION){
		flags.audioButton = 1;
	} else if(rxDataUART == TRANSMIT_TOTAL_RECORDS_COUNT){
		readEEPROM(recordsAudioCount, EEPROM_RECORDS_COUNT_ADDRESS, sizeof(recordsAudioCount));
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1, recordsAudioCount, 1, 1000);
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t *) rxData, 1);
}

uint8_t writeEEPROM(uint8_t *dataArray, uint16_t addressToWrite, uint16_t size){
	uint8_t status;

	if(addressToWrite != EEPROM_RECORDS_COUNT_ADDRESS){
		status = readEEPROM(recordsAudioCount, EEPROM_RECORDS_COUNT_ADDRESS, sizeof(recordsAudioCount));

		if(status == HAL_OK) addressToWrite = addressToWrite + EEPROM_NEXT_RECORD_ADDRESS * recordsAudioCount[0] + floor(recordsAudioCount[0] / 3 ) * 4;
		else return HAL_ERROR;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addressToWrite, 2, dataArray, size, 1000);
	return status;
}

uint8_t readEEPROM(uint8_t *dataArray, uint16_t addressToRead, uint16_t size){
	uint8_t status;

	if(addressToRead != EEPROM_RECORDS_COUNT_ADDRESS){
		addressToRead = addressToRead + EEPROM_NEXT_RECORD_ADDRESS * recordsAudioCount[0] + floor(recordsAudioCount[0] / 3 ) * 4;
	}

	status = HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, addressToRead, 2, dataArray, size, 1000);
	return status;
}

// TESTING PURPOSE ONLY
// Config for printf debug
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
