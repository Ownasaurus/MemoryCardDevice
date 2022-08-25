/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2022 by Ownasaurus <Ownasaurus@gmail.com>
 * SSI3DMASlave Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SSI3DMASlave_H_INCLUDED
#define _SSI3DMASlave_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdint.h>


void SSI3_Begin(); // Default
void SSI3_End();
bool SSI3_IsMessageAvailable(void);
uint32_t SSI3_GetMessageSize(void);
uint8_t* SSI3_PopMessage(void);
int SSI3_QueueResponse(uint8_t* data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
