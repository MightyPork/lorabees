/*
 * audio.h
 *
 *  Created on: Dec 31, 2017
 *      Author: ondra
 */

#ifndef SRC_AUDIO_H_
#define SRC_AUDIO_H_

#include <stdint.h>
#include <stdbool.h>


/** A detected peak struct */
struct peak {
    float position;  // precise position, unit is 1 bin
    float magnitude; // precise magnitude
    float weight;    // sorting weight (internal use)
};

void audio_capture(struct peak *peaks, uint32_t pcount, float *noise, float *totalpower) ;


#endif /* SRC_AUDIO_H_ */
