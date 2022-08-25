/*
 * AllStarPayload.h
 *
 *  Created on: Aug 23, 2022
 *      Author: Owna
 */

#ifndef ALLSTARPAYLOAD_H_
#define ALLSTARPAYLOAD_H_

#include <stdint.h>

typedef struct {
  uint8_t stage;
  uint8_t ckind;
} AllstarLineup;

typedef struct
{
  uint32_t rng_seed;                  // 0x0, 4 bytes
  AllstarLineup lineup_data[24]; // 0x4, 48 bytes
} AllstarPayload;

void InitISP();

extern AllstarPayload asp;

#endif /* ALLSTARPAYLOAD_H_ */
