/*
 * sound_localizer.c
 *
 *  Created on: Nov 2, 2020
 *      Author: William
 */

#include "constants.h"
#include "sound_localizer.h"
#include "acoustic_sl.h"
#include <stdlib.h>

/*Handler and Config structure for Source Localization*/
AcousticSL_Handler_t libSoundSourceLoc_Handler_Instance0;
AcousticSL_Handler_t libSoundSourceLoc_Handler_Instance1;
AcousticSL_Config_t  libSoundSourceLoc_Config_Instance0;
AcousticSL_Config_t  libSoundSourceLoc_Config_Instance1;
volatile int32_t angle0;
volatile int32_t angle1;

/**
* @brief  AudioSL Data Input
* @param  buf1 Mic 1 (1ms buffer)
* @param  buf2 Mic 2 (1ms buffer)
* @retval 1 if should process or 0 if not
*/
int Audio_Process_Data_Input(int instance, int16_t *buf1, int16_t *buf2) {
	if (instance == 0)
		return AcousticSL_Data_Input(buf1, buf2, NULL, NULL, &libSoundSourceLoc_Handler_Instance0);
	else
		return AcousticSL_Data_Input(buf1, buf2, NULL, NULL, &libSoundSourceLoc_Handler_Instance1);
}

/**
* @brief  Initialize the audio libraries adopted
* @param  m12_distance Distance between 2 mics in decimals of millimeters
* @param  freq Sampling Frequency
* @retval Error code
*/
uint32_t Audio_Libraries_Init(uint16_t m12_distance, uint32_t freq)
{
  volatile uint32_t error_value = 0;
  /* Enable CRC peripheral to unlock the library */
  __CRC_CLK_ENABLE();

  AcousticSL_Handler_t* ins[2] = {&libSoundSourceLoc_Handler_Instance0, &libSoundSourceLoc_Handler_Instance1};
  for (int i = 0; i < 2; i++) {
	  AcousticSL_Handler_t* libSoundSourceLoc_Handler_Instance = ins[i];
	  /*Setup Source Localization static parameters*/
	  libSoundSourceLoc_Handler_Instance->channel_number = 2;
	  libSoundSourceLoc_Handler_Instance->M12_distance = m12_distance;
	  libSoundSourceLoc_Handler_Instance->sampling_frequency = freq;
	  libSoundSourceLoc_Handler_Instance->algorithm = ACOUSTIC_SL_ALGORITHM_GCCP;
	  libSoundSourceLoc_Handler_Instance->ptr_M1_channels = 1;
	  libSoundSourceLoc_Handler_Instance->ptr_M2_channels = 1;
	  libSoundSourceLoc_Handler_Instance->ptr_M3_channels = 1;
	  libSoundSourceLoc_Handler_Instance->ptr_M4_channels = 1;
	  libSoundSourceLoc_Handler_Instance->samples_to_process = 4 * libSoundSourceLoc_Handler_Instance->channel_number * AUDIO_REC_SIZE;
	  AcousticSL_getMemorySize(libSoundSourceLoc_Handler_Instance);
	  libSoundSourceLoc_Handler_Instance->pInternalMemory=(uint32_t *)malloc(libSoundSourceLoc_Handler_Instance->internal_memory_size);
	  error_value += (libSoundSourceLoc_Handler_Instance->pInternalMemory == NULL);
	  error_value += AcousticSL_Init(libSoundSourceLoc_Handler_Instance);
  }

  AcousticSL_Config_t* config[2] = {&libSoundSourceLoc_Config_Instance0, &libSoundSourceLoc_Config_Instance1};
  for (int i = 0; i < 2; i++) {
	  AcousticSL_Config_t* libSoundSourceLoc_Config_Instance = config[i];
	  libSoundSourceLoc_Config_Instance->resolution = RESOLUTION;
	  libSoundSourceLoc_Config_Instance->threshold = NOISE_THRESHOLD;
	  error_value += AcousticSL_setConfig(ins[i], libSoundSourceLoc_Config_Instance);
  }


  /*Setup Source Localization dynamic parameters*/
  return error_value;
}

void Audio_ProcessAngle(int instance) {
  if (instance == 0)
	  (void)AcousticSL_Process((int32_t *)&angle0, &libSoundSourceLoc_Handler_Instance0);
  else
	  (void)AcousticSL_Process((int32_t *)&angle1, &libSoundSourceLoc_Handler_Instance1);
}

int32_t Audio_GetAngle(int instance) {
	if (instance == 0)
		return angle0;
	else
		return angle1;
}
