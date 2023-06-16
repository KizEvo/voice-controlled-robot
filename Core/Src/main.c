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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct INT_FLAGS{
	uint8_t audioButton;
	uint8_t recordButton;
	uint8_t recordTimer;
	uint8_t dataReceived;
	uint8_t testRecord;
};

struct INT_FLAGS flags;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 128
#define FFT_LENGTH 64
#define DEBOUNCE_TIME_MS 20
#define MAX_RECORD_COUNT 3
#define MAX_FREQ_AND_GAIN 20 // If the number of dB stored is 10 then it must be doubled
							 // => 20 to keep track of which dB belongs to a certain FREQS
#define FLASH_SECTOR_4 4U    // U means unsigned
const uint32_t FLASH_ADDRESS_SECTOR_4 = 0x08010000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

// RFFT fast instance
arm_rfft_fast_instance_f32 fft_handler;

// Initialize array of 4 uint16 bit buffer
uint16_t receiveBuffer[BUFFER_SIZE];
static volatile uint16_t *rxBufferPtr;

// Initialize FFT IN array
float32_t fftBufferIn[BUFFER_SIZE / 2];
uint16_t fftBufferIndex;
float32_t sum;
// Initialize FFT OUT array
float32_t fftBufferOut[BUFFER_SIZE / 2];

// Initialize FREQS (Hz) array
int freqs[FFT_LENGTH / 2];
uint8_t storedFreqs[MAX_RECORD_COUNT][FFT_LENGTH / 2];
uint8_t highestFreqs[MAX_RECORD_COUNT][MAX_FREQ_AND_GAIN];
uint8_t averageHighestFreqs[MAX_FREQ_AND_GAIN];
// Numb of time audio got recorded
uint8_t recordsCount;

//
uint32_t previousProgrammedAddress = 0;

// Debounce button time
uint32_t currentTimeInMillis = 0;
uint32_t previousTimeInMillis = 0;

// State of I2S
int stateI2S;

// State of audio record sequence
uint8_t stateRecordAudio;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void toggleAudioStream();
void fftTransform();
void processData();
void enableRecordAudio();
void setLedSuccess();
void store10HighestFreqsdB();
void writeFlash();
void readFlash(uint32_t address);
void deleteFlashSector(uint32_t sector);
uint8_t startRecordAudio();
float complexABS(float real, float compl);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  // Initialize RFFT
  arm_rfft_fast_init_f32(&fft_handler, FFT_LENGTH);

  // Clear INT_State_Pin status
  HAL_GPIO_WritePin(INT_STATE_GPIO_Port, INT_STATE_Pin, GPIO_PIN_RESET);

  flags.recordTimer = 0;
  flags.recordButton = 0;
  flags.audioButton = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Clear FFT buffer index after each iteration
	  fftBufferIndex = 0;

	  if(flags.audioButton == 1) toggleAudioStream();
	  if(flags.recordButton == 1) {
		  // Clear previous records counts
		  recordsCount = 0;
		  // Clear previous recorded/processed arrays
		  for(uint16_t i = 0; i < MAX_FREQ_AND_GAIN; i++){
			  averageHighestFreqs[i] = 0;
		  }
		  for(uint16_t col = 0; col < MAX_RECORD_COUNT; col++){
			  for(uint16_t row = 0; row < MAX_FREQ_AND_GAIN; row++){
				  highestFreqs[col][row] = 0;
			  }
		  }

		  // Record audio MAX_RECORD_COUNT time
		  while(recordsCount < MAX_RECORD_COUNT){
			  enableRecordAudio();
			  stateRecordAudio = startRecordAudio();

			  if(stateRecordAudio == 1) setLedSuccess();
			  else break;

			  recordsCount++;
		  }

		  // If successful then proceed to store highest FREQS got from records
		  if(recordsCount >= 3 && stateRecordAudio == 1) {
			  store10HighestFreqsdB();
			  HAL_GPIO_WritePin(INT_STATE_GPIO_Port, INT_STATE_Pin, GPIO_PIN_RESET);
			  HAL_Delay(2000);
			  HAL_GPIO_WritePin(INT_STATE_GPIO_Port, INT_STATE_Pin, GPIO_PIN_SET);
		  }
		  flags.recordButton = 0;
	  }
	  if(flags.dataReceived == 1) processData();
	  if(flags.testRecord == 1){
		  uint8_t temp_dB = 0;
		  uint8_t tempIndex = 0;
		  uint8_t correctMatches = 0;
		  uint8_t totalKey = 0;

		  flags.testRecord = 0;

		  for(uint8_t i = 0; i < MAX_FREQ_AND_GAIN; i += 2){
			  if(averageHighestFreqs[i] == 0 && averageHighestFreqs[i + 1] == 0) break;
			  totalKey++;
		  }

		  for(uint8_t i = 0; i < totalKey * 2; i += 2){
			  for(uint8_t freqIndex = 0; freqIndex < FFT_LENGTH / 2; freqIndex++){
				  if(storedFreqs[0][freqIndex] > temp_dB) {
					  temp_dB = storedFreqs[0][freqIndex];
					  tempIndex = freqIndex;
				  }
			  }

			  if(averageHighestFreqs[i] == tempIndex){
				  uint8_t diff = temp_dB > averageHighestFreqs[i + 1] ?
						  (temp_dB - averageHighestFreqs[i + 1]) : (averageHighestFreqs[i + 1] - temp_dB);
				  if(diff > 5) break;

				  correctMatches++;
			  }

			  if(totalKey - correctMatches <= 1) {
				  for(uint8_t i = 0; i < 10; i++){
					  HAL_GPIO_TogglePin(INT_STATE_GPIO_Port, INT_STATE_Pin);
					  HAL_Delay(500);
				  }
				  break;
			  }
			  storedFreqs[0][tempIndex] = 0;
			  temp_dB = 0;
			  tempIndex = 0;
		  }
	  }

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

  /*Configure GPIO pins : INT_STATE_Pin RECORD_STATE_Pin LED_MIC_STATE_Pin */
  GPIO_InitStruct.Pin = INT_STATE_Pin|RECORD_STATE_Pin|LED_MIC_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_Pin RECORD_BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin|RECORD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	uint8_t isDoubleClick;
//	// Check for GPIO_Pin to set flag correctly
//	currentTimeInMillis = HAL_GetTick();
//	isDoubleClick = (currentTimeInMillis - previousTimeInMillis) < DEBOUNCE_TIME_MS ? 1 : 0;
//
//	if(isDoubleClick >= 1) return;

	if(GPIO_Pin == BUTTON_Pin && flags.recordButton <= 0) flags.audioButton = 1;
	else if(GPIO_Pin == RECORD_BUTTON_Pin && flags.audioButton <= 0) flags.recordButton = 1;

	for(uint32_t delay = 0; delay < 3000000; delay++);
//	previousTimeInMillis = currentTimeInMillis;
}

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
		// Enable I2S Receive DMA
		// HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *) receiveBuffer, BUFFER_SIZE);
		HAL_GPIO_WritePin(INT_STATE_GPIO_Port, INT_STATE_Pin, GPIO_PIN_RESET);

		recordsCount = 0;
		flags.recordButton = 1;

		enableRecordAudio();
		startRecordAudio();

		flags.recordButton = 0;
		flags.testRecord = 1;
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

float complexABS(float real, float compl)
{
	return sqrtf(real*real + compl*compl);
}

void fftTransform()
{
	float32_t temp = 0;
	for (int i = 0; i < 64; i++)
	{
		temp = temp + fftBufferIn[i];
	}
	sum = temp;
	// Transform real FFT, 0 means time domain to frequency domain
	// Return array contains real ([i]) and complex ([i + 1]) number
	arm_rfft_fast_f32(&fft_handler, &fftBufferIn, &fftBufferOut, 0);

	// Clear freqIndex
	uint16_t freqIndex = 0;

	// Calculate ABS value of complex vectors and translate into log dB range
	// Equation: 20 * log10( sqrt(real * real + complex * complex) );
	// Store the result in freqs array
	// Formula to check which index (index of freqs[]) correspond to a certain frequency:
	// INDEX = FREQ * (FFT_LENGTH / 2) / (F_SAMPLE / 2)
	// EX: 12 = 3000 * (64 / 2) / (16000 / 2);
	for(uint16_t i = 0; i < BUFFER_SIZE / 2; i += 2){
		freqs[freqIndex] = (int)(20*log10f(complexABS(fftBufferOut[i], fftBufferOut[i+1])));
		if(freqs[freqIndex] < 0) freqs[freqIndex] = 0;
		if(flags.recordButton == 1 && flags.recordTimer == 1) storedFreqs[recordsCount][freqIndex] = (uint8_t) freqs[freqIndex];
		freqIndex++;
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
	HAL_I2S_DMAStop(&hi2s2);

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

void setLedSuccess(){
	uint16_t delayTime = 500;
	for(uint8_t i = 0; i < 10; i++){
		HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_SET);
		HAL_Delay(delayTime);
		HAL_GPIO_WritePin(RECORD_STATE_GPIO_Port, RECORD_STATE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_MIC_STATE_GPIO_Port, LED_MIC_STATE_Pin, GPIO_PIN_RESET);
		HAL_Delay(150);
		delayTime -= 50;
	}
	HAL_Delay(1000);

}

void store10HighestFreqsdB(){
	uint8_t freqs_dB = 0, tempHighestdB = 0, indexOfHighestdB = 0, repeat = 0,
			highestFreqsIndex = 0, isCorrectFreqIndex = 0, averageFreqsIndex = 0,
			tempTotalHighestFreq = 0;
	const uint8_t INCORRECT_FREQ = FFT_LENGTH / 2;

	while(repeat < (MAX_FREQ_AND_GAIN / 2)){

		for(uint16_t freqSampledIndex = 0; freqSampledIndex < MAX_RECORD_COUNT; freqSampledIndex++){
			// First loop to get MAX_RECORD_COUNT storedFreqs[freqSampledIndex][x]; x means don't care
			for(uint16_t dBIndex = 0; dBIndex < FFT_LENGTH / 2; dBIndex++){
				// Second loop to check dB gain for a storedFreqs[x][dBIndex] sample; x means dont't care
				freqs_dB = storedFreqs[freqSampledIndex][dBIndex];
				if(freqs_dB > tempHighestdB) {
					tempHighestdB = freqs_dB;
					indexOfHighestdB = dBIndex;
				}
			}
			// Store first [x][i] as FREQ, [x][i + 1] as dB of that FREQ
			highestFreqs[freqSampledIndex][highestFreqsIndex] = indexOfHighestdB;
			highestFreqs[freqSampledIndex][highestFreqsIndex + 1] = tempHighestdB;
			// Clear current highest dB in array to get the
			// next highest dB in the next iteration
			storedFreqs[freqSampledIndex][indexOfHighestdB] = 0;
			// Check if 3 recorded highest index FREQS match each other
			if(freqSampledIndex > 0){
				isCorrectFreqIndex =
						indexOfHighestdB == highestFreqs[freqSampledIndex - 1][highestFreqsIndex] ?
						indexOfHighestdB : INCORRECT_FREQ;
			}
			// Store total 3 recorded highest dB
			tempTotalHighestFreq += tempHighestdB;
			// Clear current value of highest dB and index
			indexOfHighestdB = 0;
			tempHighestdB = 0;
		}

		if(isCorrectFreqIndex != INCORRECT_FREQ){
			averageHighestFreqs[averageFreqsIndex] = isCorrectFreqIndex;
			averageHighestFreqs[averageFreqsIndex + 1] = tempTotalHighestFreq / 3;
		}
		// Increment average FREQS array index
		averageFreqsIndex += 2;
		// Increment index by 2, prepare for next iteration
		highestFreqsIndex += 2;
		// Clear total highest FREQS from the previous iteration
		tempTotalHighestFreq = 0;
		// Increment loop
		repeat++;
	}
}


void writeFlash(){
	HAL_FLASH_Unlock();
	// NOT FINISHED
	// Store the address that got incremented to for the next loop
	for(uint32_t i = 0; i <= MAX_FREQ_AND_GAIN; i++){
		if(i != MAX_FREQ_AND_GAIN) HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_SECTOR_4 + i, averageHighestFreqs[i]);
		else HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_SECTOR_4, i);
	}
	HAL_FLASH_Lock();
}

void deleteFlashSector(uint32_t sector){
	HAL_FLASH_Unlock();

	FLASH_Erase_Sector(sector, FLASH_VOLTAGE_RANGE_3);

	HAL_FLASH_Lock();
}

void readFlash(uint32_t address){
	HAL_FLASH_Unlock();

	// NOT implemented yet
	//

	HAL_FLASH_Lock();
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
