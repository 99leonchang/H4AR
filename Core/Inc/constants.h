/*
 * constants.h
 *
 *  Created on: Nov 3, 2020
 *      Author: William
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define SAMPLING_FREQUENCY (8000)

/* 2 1ms-sample buffers since DFSDM has half/full */
#define AUDIO_REC_SIZE ((SAMPLING_FREQUENCY / 1000) * 2)

#define MICS 4

#define M12_DISTANCE (150)

#define RESOLUTION (5)
#define NOISE_THRESHOLD (15)

#endif /* INC_CONSTANTS_H_ */
