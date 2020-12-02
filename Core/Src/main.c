/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "constants.h"
#include "acoustic_sl.h"
#include "sound_localizer.h"
#include "oled.h"
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
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter2;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter3;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel3;
DMA_HandleTypeDef hdma_dfsdm1_flt1;
DMA_HandleTypeDef hdma_dfsdm1_flt3;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
DMA_HandleTypeDef hdma_dfsdm1_flt2;

I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t PlayBuf0[AUDIO_REC_SIZE];
int16_t PlayBuf1[AUDIO_REC_SIZE];
int16_t PlayBuf2[AUDIO_REC_SIZE];
int16_t PlayBuf3[AUDIO_REC_SIZE];
int32_t RecBuf0[AUDIO_REC_SIZE];
int32_t RecBuf1[AUDIO_REC_SIZE];
int32_t RecBuf2[AUDIO_REC_SIZE];
int32_t RecBuf3[AUDIO_REC_SIZE];

// Offset fixing
// TODO: Use this for offset-fixing, as needed
int32_t PlayBufSums[MICS];

volatile int32_t AngleExists0;
volatile int32_t AngleExists1;
volatile int32_t AngleExists2;
volatile int32_t AngleRaw0;
volatile int32_t AngleRaw1;
volatile int32_t AngleRaw2;

volatile int32_t AngleEstimation00;
volatile int32_t AngleEstimation01;
volatile int32_t AngleEstimation10;
volatile int32_t AngleEstimation11;
volatile int32_t AngleEstimation20;
volatile int32_t AngleEstimation21;
volatile int32_t RaiseIRQ;

volatile int32_t IsConvergence;
volatile int32_t ConvergenceAngle;

// Smoothing stuff
volatile int32_t pastEstimatesForSmoothing[3][SMOOTHING_SAMPLES];
volatile int32_t currentSmoothingIndex[3];
volatile int32_t currentSmoothingSum[3];
volatile int32_t currentSmoothingSumOfSquares[3];

volatile int32_t pastEstimatesForSmoothingConvergence[SMOOTHING_SAMPLES];
volatile int32_t currentSmoothingIndexConvergence;
volatile int32_t currentSmoothingSumConvergence;
volatile int32_t currentSmoothingSumOfSquaresConvergence;

volatile int32_t SmoothedAngleExists0;
volatile int32_t SmoothedAngleExists1;
volatile int32_t SmoothedAngleExists2;
volatile int32_t SmoothedAngleEstimation0;
volatile int32_t SmoothedAngleEstimation1;
volatile int32_t SmoothedAngleEstimation2;

volatile int32_t SmoothedConvergenceExists;
volatile int32_t SmoothedConvergenceAngle;

volatile int alreadyBinned = 1;
int globalLastAngle = 8;

// Whether first half of PlayBuf[i] is ready for acousticSL
uint8_t PlayHalfReady[MICS] = {0, 0, 0, 0};

// Whether second half of PlayBuf[i] is ready for acousticSL
uint8_t PlaySecondHalfReady[MICS] = {0, 0, 0, 0};

uint8_t DmaRecHalfBuffComplete[MICS] = {0, 0, 0, 0};
uint8_t DmaRecBuffComplete[MICS] = {0, 0, 0, 0};

int DFSDMFilterToIndex(DFSDM_Filter_HandleTypeDef *hfdfsdm_filter) {
	if (&hdfsdm1_filter0 == hfdfsdm_filter) return 0;
	else if (&hdfsdm1_filter1 == hfdfsdm_filter) return 1;
	else if (&hdfsdm1_filter2 == hfdfsdm_filter) return 2;
	else if (&hdfsdm1_filter3 == hfdfsdm_filter) return 3;
	else Error_Handler();

	return -1;
}

DFSDM_Filter_HandleTypeDef* IndexToDFSDMFilter(int index) {
	switch (index) {
		case 0: return &hdfsdm1_filter0;
		case 1: return &hdfsdm1_filter1;
		case 2: return &hdfsdm1_filter2;
		case 3: return &hdfsdm1_filter3;
	}

	Error_Handler();
	return NULL;
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	int filter = DFSDMFilterToIndex(hdfsdm_filter);
	DmaRecHalfBuffComplete[filter] = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	int filter = DFSDMFilterToIndex(hdfsdm_filter);
	DmaRecBuffComplete[filter] = 1;
}

void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	Error_Handler();
}

int IsAllSet(uint8_t *buffer, int n) {
	for (int i = 0; i < n; i++) {
		if (buffer[i] == 0) {
			return 0;
		}
	}

	return 1;
}

void GetBuffers(int index, int16_t **playbuf, int32_t **recbuf) {
	int16_t *buffer = 0;
	switch (index) {
		case 0: buffer = PlayBuf0; break;
		case 1: buffer = PlayBuf1; break;
		case 2: buffer = PlayBuf2; break;
		case 3: buffer = PlayBuf3; break;
		default: Error_Handler(); break;
	}

	int32_t *recbuffer = 0;
	switch (index) {
		case 0: recbuffer = RecBuf0; break;
		case 1: recbuffer = RecBuf1; break;
		case 2: recbuffer = RecBuf2; break;
		case 3: recbuffer = RecBuf3; break;
		default: Error_Handler(); break;
	}

	if (playbuf) *playbuf = buffer;
	if (recbuf) *recbuf = recbuffer;
}

void ClearUI8Buffers(uint8_t *buffer, int n) {
	for (int i = 0; i < n; i++) {
		buffer[i] = 0;
	}
}

void ClearI32Buffers(volatile int32_t *buffer, int n) {
	for (int i = 0; i < n; i++) {
		buffer[i] = 0;
	}
}

void ClearBuffers(int offset) {
	int16_t *playbuf;
	int32_t *recbuf;
	GetBuffers(offset, &playbuf, &recbuf);
	for (int i = 0; i < AUDIO_REC_SIZE; i++) {
		playbuf[i] = 0;
		recbuf[i] = 0;
	}
}

int GetOffsets(int offset) {
	// If you think you know what you want to do here,
	// you probably don't. Never change these numbers
	// unless you want to fail.
	//
	// - :eyes:
	int evil_offset = 300;
	switch (offset) {
	case 0:
		return (465) - evil_offset;
	case 1:
		return (349) - evil_offset;
	case 2:
		return (423) - evil_offset;
	case 3:
		return (600) - evil_offset;
	default:
		return 0;
	}
}

void TransferBuffers(int offset, int low, int high) {
	int16_t *buffer;
	int32_t *recbuffer;
	GetBuffers(offset, &buffer, &recbuffer);

	// acousticSL only wants 16 bits
	for (uint16_t i = low; i < high; i++) {
		buffer[i] = (recbuffer[i] >> 16) - GetOffsets(offset);
	}
}

// TODO: Check if angle is measured anticlockwise from x-axis
// TODO: Update this when covering 360 degree range
uint8_t angleToDirectionNumber(int32_t exists, int32_t estimation) {
	if (!exists) return 8;
	if (estimation >= 90 && estimation <= 270) return 8;
	return (((estimation + 23) / 45) % 8);
}

void EXTI1_Callback(void) {
	if (RaiseIRQ & 1) {
		Audio_ProcessAngle(0);
		int32_t angle = Audio_GetAngle(0);
		if (angle == ACOUSTIC_SL_NO_AUDIO_DETECTED) {
			AngleExists0 = 0;
		} else {
			AngleExists0 = 1;
			AngleRaw0 = angle;

		}
	}

	if (RaiseIRQ & 2) {
		Audio_ProcessAngle(1);
		int32_t angle = Audio_GetAngle(1);
		if (angle == ACOUSTIC_SL_NO_AUDIO_DETECTED) {
			AngleExists1 = 0;
		} else {
			AngleExists1 = 1;
			AngleRaw1 = angle;
		}
	}

	/*
	if (RaiseIRQ & 4) {
		Audio_ProcessAngle(2);
		int32_t angle = Audio_GetAngle(2);
		if (angle == ACOUSTIC_SL_NO_AUDIO_DETECTED) {
			AngleExists2 = 0;
		} else {
			AngleExists2 = 1;
			AngleRaw2 = angle;
		}
	}
	*/

	int32_t input[3] = {AngleRaw0, AngleRaw1, AngleRaw2};
	int32_t exists[3] = {AngleExists0, AngleExists1, AngleExists2};
	volatile int32_t *smoothExists[3] = {&SmoothedAngleExists0, &SmoothedAngleExists1, &SmoothedAngleExists2};
	volatile int32_t *smoothAngle[3] = {&SmoothedAngleEstimation0, &SmoothedAngleEstimation1, &SmoothedAngleEstimation2};
	for (int i = 0; i < 2; i++) {
		if (!exists[i]) {
			*(smoothExists[i]) = 0;
			continue;
		}

		// Smoothing stuff
		int32_t AngleEstimation = input[i];
		currentSmoothingSum[i] -= pastEstimatesForSmoothing[i][currentSmoothingIndex[i]];
		currentSmoothingSumOfSquares[i] -= pastEstimatesForSmoothing[i][currentSmoothingIndex[i]] * pastEstimatesForSmoothing[i][currentSmoothingIndex[i]];
		currentSmoothingSum[i] += AngleEstimation;
		currentSmoothingSumOfSquares[i] += AngleEstimation * AngleEstimation;
		pastEstimatesForSmoothing[i][currentSmoothingIndex[i]] = AngleEstimation;
		currentSmoothingIndex[i] = (currentSmoothingIndex[i] + 1) % SMOOTHING_SAMPLES;
		int mean = currentSmoothingSum[i] / SMOOTHING_SAMPLES;
		int variance = currentSmoothingSumOfSquares[i] / SMOOTHING_SAMPLES - mean * mean;
		if (variance < SMOOTHING_THRESHOLD * SMOOTHING_THRESHOLD) {
			*(smoothExists[i]) = 1;
			*(smoothAngle[i]) = mean;
		}
		else {
			*(smoothExists[i]) = 0;
		}
	}

	if (SmoothedAngleExists0 && SmoothedAngleExists1 /*&& SmoothedAngleExists2 */) {
		AngleEstimation00 = (((-90 + MIC_OFFSET - SmoothedAngleEstimation0) + 720) % 360);
		AngleEstimation01 = (((90 + MIC_OFFSET + SmoothedAngleEstimation0) + 720) % 360);
		AngleEstimation10 = (((90 - MIC_OFFSET - SmoothedAngleEstimation1) + 720) % 360);
		AngleEstimation11 = (((270 - MIC_OFFSET + SmoothedAngleEstimation1) + 720) % 360);
		AngleEstimation20 = ((SmoothedAngleEstimation2 + 720) % 360);
		AngleEstimation21 = ((180 - SmoothedAngleEstimation2 + 720) % 360);

		IsConvergence = 1;
		int32_t numbers[6] = {AngleEstimation00, AngleEstimation01, AngleEstimation10, AngleEstimation11, AngleEstimation20, AngleEstimation21};
		int32_t minDiff = 360;
		int32_t avg = 0;
		int32_t pair0_num = -1;
		int32_t pair1_num = -1;
		for (int i = 0; i < 2; i++) {
			for (int j = 2; j < 4; j++) {
				int32_t diff = numbers[j] - numbers[i];
				if (diff < 0) diff = -diff;

				int32_t diff1 = diff % 360;
				int32_t diff2 = (360 - diff) % 360;
				int32_t trueDiff = (diff1 < diff2) ? diff1 : diff2;
				if (trueDiff < minDiff) {
					minDiff = trueDiff;
					pair0_num = i;
					pair1_num = j;

					if (diff > 180) {
						avg = (numbers[j] < numbers[i]) ? numbers[i] : numbers[j];
						avg = (avg + minDiff/2) % 360;
					} else {
						avg = (numbers[j] < numbers[i]) ? numbers[j] : numbers[i];
						avg += (minDiff / 2);
					}
				}
			}
		}

		ConvergenceAngle = avg;
		if (ConvergenceAngle < 180) {
			ConvergenceAngle = numbers[pair1_num];
		} else {
			ConvergenceAngle = numbers[pair0_num];
		}
	} else {
		IsConvergence = 0;
	}

	if (IsConvergence) {
		int32_t AngleEstimation = ConvergenceAngle;
		currentSmoothingSumConvergence -= pastEstimatesForSmoothingConvergence[currentSmoothingIndexConvergence];
		currentSmoothingSumOfSquaresConvergence -= pastEstimatesForSmoothingConvergence[currentSmoothingIndexConvergence] * pastEstimatesForSmoothingConvergence[currentSmoothingIndexConvergence];
		currentSmoothingSumConvergence += AngleEstimation;
		currentSmoothingSumOfSquaresConvergence += AngleEstimation * AngleEstimation;
		pastEstimatesForSmoothingConvergence[currentSmoothingIndexConvergence] = AngleEstimation;
		currentSmoothingIndexConvergence = (currentSmoothingIndexConvergence + 1) % SMOOTHING_SAMPLES;
		int mean = currentSmoothingSumConvergence / SMOOTHING_SAMPLES;
		int variance = currentSmoothingSumOfSquaresConvergence / SMOOTHING_SAMPLES - mean * mean;
		if (variance < SMOOTHING_THRESHOLD * SMOOTHING_THRESHOLD) {
			SmoothedConvergenceExists = 1;
			SmoothedConvergenceAngle = mean;
		}
		else {
			SmoothedConvergenceExists = 0;
		}
	} else {
		SmoothedConvergenceExists = 0;
	}
	alreadyBinned = 0;
	RaiseIRQ = 0;
}

/* USER CODE END 0 */

/*
 * --------------------
 * Interface for Bins!
 * --------------------
 */
void clearBins(int *bins) {
	for (int i = 0; i < NUMBER_OF_BINS; i++) {
		bins[i] = 0;
	}
}

void binAngle(int *bins, int delta) {
	if (!(delta < NUMBER_OF_BINS)) Error_Handler();
	bins[delta]++;
}

int getBinMaxIfOk(int * bins, int binnedSamples) {
	for (int i = 0; i < NUMBER_OF_BINS; i++) {
		int percentOk = (bins[i] * 100) / binnedSamples;
		if (percentOk >= BIN_PERCENT_THRESHOLD) {
			return i;
		}
	}
	return 8;
}
/*
 * --------------------
 * End Interface for Bins!
 * --------------------
 */

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
  RaiseIRQ = 0;
  AngleExists0 = 0;
  AngleEstimation00 = 0;
  AngleEstimation01 = 0;
  AngleExists1 = 0;
  AngleEstimation10 = 0;
  AngleEstimation11 = 0;
  AngleExists2 = 0;
  AngleEstimation20 = 0;
  AngleEstimation21 = 0;

  // Initialize smoothing stuff
  ClearI32Buffers(pastEstimatesForSmoothing[0], SMOOTHING_SAMPLES);
  ClearI32Buffers(pastEstimatesForSmoothing[1], SMOOTHING_SAMPLES);
  ClearI32Buffers(pastEstimatesForSmoothing[2], SMOOTHING_SAMPLES);
  ClearI32Buffers(PlayBufSums, MICS);
  for (int i = 0; i < 3; i++) {
	currentSmoothingIndex[i] = 0;
	currentSmoothingSum[i] = 0;
	currentSmoothingSumOfSquares[i] = 0;
  }

  ClearI32Buffers(pastEstimatesForSmoothingConvergence, SMOOTHING_SAMPLES);
  currentSmoothingIndexConvergence = 0;
  currentSmoothingSumConvergence = 0;
  currentSmoothingSumOfSquaresConvergence = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  oled_Init(&hi2c3);

  // TODO: Don't preempt computing angles -- dubious at best, evil at worst?
  HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn);

  int status = Audio_Libraries_Init(M12_DISTANCE, SAMPLING_FREQUENCY);

  for (int i = 0; i < MICS; i++) {
	  ClearBuffers(i);
	  DFSDM_Filter_HandleTypeDef *filter = IndexToDFSDMFilter(i);
	  HAL_DFSDM_Filter_RegisterCallback(filter, HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, HAL_DFSDM_FilterRegConvHalfCpltCallback);
	  HAL_DFSDM_Filter_RegisterCallback(filter, HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, HAL_DFSDM_FilterRegConvCpltCallback);
	  HAL_DFSDM_Filter_RegisterCallback(filter, HAL_DFSDM_FILTER_ERROR_CB_ID, HAL_DFSDM_FilterErrorCallback);
  }

  for (int i = 0; i < MICS; i++) {
	  DFSDM_Filter_HandleTypeDef *filter = IndexToDFSDMFilter(i);
	  int32_t *recbuf;
	  GetBuffers(i, NULL, &recbuf);
	  status += HAL_DFSDM_FilterRegularStart_DMA(filter, recbuf, AUDIO_REC_SIZE);
  }

  if (status != 0) {
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int lastAngle = 8;
  globalLastAngle = lastAngle;
  int binnedSamples = 0;
  int bins[NUMBER_OF_BINS];
  while (1)
  {
	  if (!alreadyBinned) {
		  alreadyBinned = 1;
		  int delta = angleToDirectionNumber(SmoothedConvergenceExists, SmoothedConvergenceAngle);
		  binAngle(bins, delta);
		  binnedSamples++;
		  if (BIN_RATE <= binnedSamples) {
			  int newBinAngle = getBinMaxIfOk(bins, binnedSamples);
			  if (newBinAngle != lastAngle) {
				  lastAngle = newBinAngle;
				  globalLastAngle = lastAngle;
				  oled_Display(lastAngle);
			  }
			  binnedSamples = 0;
			  clearBins(bins);
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  RaiseIRQ = 0;
	  if (IsAllSet(PlayHalfReady, MICS)) {
		  int16_t *rec0;
		  int16_t *rec1;
		  int16_t *rec2;
		  int16_t *rec3;
		  GetBuffers(0, &rec0, NULL);
		  GetBuffers(1, &rec1, NULL);
		  GetBuffers(2, &rec2, NULL);
		  GetBuffers(3, &rec3, NULL);
		  ClearUI8Buffers(PlayHalfReady, MICS);

		  RaiseIRQ = 0;
		  RaiseIRQ |= Audio_Process_Data_Input(0, rec3, rec2);
		  RaiseIRQ |= (Audio_Process_Data_Input(1, rec1, rec0) << 1);
		  // RaiseIRQ |= (Audio_Process_Data_Input(2, rec0, rec3) << 2);
	  } else if (IsAllSet(PlaySecondHalfReady, MICS)) {
		  int16_t *rec0;
		  int16_t *rec1;
		  int16_t *rec2;
		  int16_t *rec3;
		  GetBuffers(0, &rec0, NULL);
		  GetBuffers(1, &rec1, NULL);
		  GetBuffers(2, &rec2, NULL);
		  GetBuffers(3, &rec3, NULL);
		  ClearUI8Buffers(PlaySecondHalfReady, MICS);

		  RaiseIRQ = 0;
		  RaiseIRQ |= Audio_Process_Data_Input(0, rec3 + (AUDIO_REC_SIZE/2), rec2 + (AUDIO_REC_SIZE/2));
		  RaiseIRQ |= (Audio_Process_Data_Input(1, rec1 + (AUDIO_REC_SIZE/2), rec0 + (AUDIO_REC_SIZE/2)) << 1);
		  // RaiseIRQ |= (Audio_Process_Data_Input(2, rec0 + (AUDIO_REC_SIZE/2), rec3 + (AUDIO_REC_SIZE/2)) << 2);
	  }

	  if (RaiseIRQ) {
		  HAL_NVIC_SetPendingIRQ(EXTI1_IRQn);
	  }

	  for (int j = 0; j < MICS; j++) {
		  if (DmaRecHalfBuffComplete[j]) {
			  TransferBuffers(j, 0, AUDIO_REC_SIZE/2);
			  DmaRecHalfBuffComplete[j] = 0;
			  PlayHalfReady[j] = 1;
		  }
		  if (DmaRecBuffComplete[j]) {
			  TransferBuffers(j, AUDIO_REC_SIZE / 2, AUDIO_REC_SIZE);
			  DmaRecBuffComplete[j] = 0;
			  PlaySecondHalfReady[j] = 1;
		  }
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 63;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 63;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter2.Instance = DFSDM1_Filter2;
  hdfsdm1_filter2.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter2.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter2.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter2.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter2.Init.FilterParam.Oversampling = 63;
  hdfsdm1_filter2.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter2) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter3.Instance = DFSDM1_Filter3;
  hdfsdm1_filter3.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter3.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter3.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter3.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter3.Init.FilterParam.Oversampling = 63;
  hdfsdm1_filter3.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter3) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 40;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x7;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 40;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x07;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x7;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel3.Instance = DFSDM1_Channel3;
  hdfsdm1_channel3.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel3.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel3.Init.OutputClock.Divider = 40;
  hdfsdm1_channel3.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel3.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel3.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel3.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel3.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel3.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel3.Init.Awd.Oversampling = 1;
  hdfsdm1_channel3.Init.Offset = 0;
  hdfsdm1_channel3.Init.RightBitShift = 0x7;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_3, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter2, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter3, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00300F33;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C3);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_Delay(1000);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
