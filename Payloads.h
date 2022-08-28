/*
 * AllStarPayload.h
 *
 *  Created on: Aug 23, 2022
 *      Author: Owna
 */

#ifndef PAYLOADS_H_
#define PAYLOADS_H_

#include <stdint.h>

void InitPayloadData();

// first 4 bytes are RNG seed
// remainder are the all star character and stage list
extern uint8_t allstar_data[1024];
extern uint8_t adventure_data[1024];

#endif /* PAYLOADS_H_ */
