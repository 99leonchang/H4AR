/*
 * sound_localizer.c
 *
 *  Created on: Nov 2, 2020
 *      Author: William
 */

#include "sound_localizer.h"
#include "acoustic_sl.h"
#include <stdlib.h>

/*Handler and Config structure for Source Localization*/
AcousticSL_Handler_t libSoundSourceLoc_Handler_Instance;
AcousticSL_Config_t  libSoundSourceLoc_Config_Instance;
volatile int32_t angle;

/**
* @brief  AudioSL Data Input
* @param  buf1 Mic 1 (1ms buffer)
* @param  buf2 Mic 2 (1ms buffer)
* @retval 1 if should process or 0 if not
*/
int Audio_Process_Data_Input(int16_t *buf1, int16_t *buf2) {
	return AcousticSL_Data_Input(buf1, buf2, NULL, NULL, &libSoundSourceLoc_Handler_Instance);
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

  /*Setup Source Localization static parameters*/
  libSoundSourceLoc_Handler_Instance.channel_number = 2;
  libSoundSourceLoc_Handler_Instance.M12_distance = m12_distance;
  libSoundSourceLoc_Handler_Instance.sampling_frequency = freq;
  libSoundSourceLoc_Handler_Instance.algorithm = ACOUSTIC_SL_ALGORITHM_GCCP;
  libSoundSourceLoc_Handler_Instance.ptr_M1_channels = 1;
  libSoundSourceLoc_Handler_Instance.ptr_M2_channels = 1;
  libSoundSourceLoc_Handler_Instance.samples_to_process = 512;
  AcousticSL_getMemorySize(&libSoundSourceLoc_Handler_Instance);
  libSoundSourceLoc_Handler_Instance.pInternalMemory = (uint32_t *)malloc(libSoundSourceLoc_Handler_Instance.internal_memory_size);
  error_value += AcousticSL_Init( &libSoundSourceLoc_Handler_Instance);

  /*Setup Source Localization dynamic parameters*/
  libSoundSourceLoc_Config_Instance.resolution=5;
  libSoundSourceLoc_Config_Instance.threshold=15;
  error_value += AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance, &libSoundSourceLoc_Config_Instance);
  error_value += (libSoundSourceLoc_Handler_Instance.pInternalMemory == NULL);
  return error_value;
}

void Audio_ProcessAngle(void) {
  (void)AcousticSL_Process((int32_t *)&angle, &libSoundSourceLoc_Handler_Instance);
}

int32_t Audio_GetAngle(void) {
	return angle;
}
