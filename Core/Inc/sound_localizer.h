/*
 * sound_localizer.h
 *
 *  Created on: Nov 2, 2020
 *      Author: William
 */

#ifndef INC_SOUND_LOCALIZER_H_
#define INC_SOUND_LOCALIZER_H_

#include "stm32l4xx_hal.h"

int Audio_Process_Data_Input(int16_t *buf1, int16_t *buf2);
uint32_t Audio_Libraries_Init(uint16_t m12_distance, uint32_t freq);
void Audio_ProcessAngle(void);
int32_t Audio_GetAngle(void);

#endif /* INC_SOUND_LOCALIZER_H_ */
