/*
 * constants.h
 *
 *  Created on: Nov 3, 2020
 *      Author: William
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define SAMPLING_FREQUENCY (32000)

/* 2 1ms-sample buffers since DFSDM has half/full */
#define AUDIO_REC_SIZE ((SAMPLING_FREQUENCY / 1000) * 2)

#define MICS 4

#define M12_DISTANCE (1150)

#define RESOLUTION (4)
#define NOISE_THRESHOLD (1)

#define SMOOTHING_SAMPLES (1)
#define SMOOTHING_THRESHOLD (1000)

#define MIC_OFFSET (24)

# define NUMBER_OF_BINS (10)
# define BIN_RATE (20)
# define BIN_PERCENT_THRESHOLD (50)

#endif /* INC_CONSTANTS_H_ */
