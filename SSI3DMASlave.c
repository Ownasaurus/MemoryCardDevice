/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2022 by Christopher Dellman <christopher@dellman.net>
 * Copyright (c) 2022 by Ownasaurus <Ownasaurus@gmail.com>
 * SSI3DMASlave Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "DataPackage.h"
#include "SSI3DMASlave.h"
#include "main.h"

extern QueueHandle_t incomingEXIData;
DataPackage_t datapackage;

//SPI Clock Speed
#define SPI_CLOCK 8000000

//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define SSI_BUFFER_SIZE       1024
#define SSI_RX_BUFFER_COUNT   5
#define SSI_TX_BUFFER_COUNT   5

//*****************************************************************************
//
// The SSI3 Buffers
//
//*****************************************************************************
uint8_t g_ui8SSITxBuf[SSI_TX_BUFFER_COUNT][SSI_BUFFER_SIZE];
uint8_t g_ui8SSIRxBuf[SSI_RX_BUFFER_COUNT][SSI_BUFFER_SIZE];

//*****************************************************************************
//
// The Memory Control Variables
//
//*****************************************************************************
uint32_t g_ui32MessageSizes[SSI_RX_BUFFER_COUNT];
uint_fast8_t g_ui8RxWriteIndex = 0;
uint_fast8_t g_ui8RxReadIndex = 0;

volatile uint_fast8_t g_ui8TxWriteIndex = 0;
volatile uint_fast8_t g_ui8TxReadIndex = 0;
volatile bool g_ui8TXEmpty = true;

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
//
// The count of SSI3 buffers filled/read
//
//*****************************************************************************
volatile uint32_t g_ui32SSIRxWriteCount = 0;
volatile uint32_t g_ui32SSIRxReadCount = 0;
// uint32_t g_ui32SSITxCount = 0;

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

void uDMAErrorHandler(void)
{
  uint32_t ui32Status;

  //
  // Check for uDMA error bit
  //
  ui32Status = ROM_uDMAErrorStatusGet();

  //
  // If there is a uDMA error, then clear the error and increment
  // the error counter.
  //
  if (ui32Status)
  {
    ROM_uDMAErrorStatusClear();
    g_ui32uDMAErrCount++;
  }
}

//*****************************************************************************
//
// CS Rising Edge Interrupt Handler
//
//*****************************************************************************
void gpioQ1IntHandler(void) {
  uint32_t ui32Status;

  ui32Status = ROM_GPIOIntStatus(GPIO_PORTQ_BASE, 1);
  ROM_GPIOIntClear(GPIO_PORTQ_BASE, ui32Status);

  bool cs_n = GPIOPinRead(GPIO_PORTQ_BASE, GPIO_PIN_1);

  if (cs_n) { //Rising edge
    uint32_t xferSize = ROM_uDMAChannelSizeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);

    //Calculate next RxWriteIndex (buffer in which to write the next set of data)
    uint8_t nextIndex = g_ui8RxWriteIndex + 1;
    if (nextIndex >= SSI_RX_BUFFER_COUNT) nextIndex = 0;

    //Get the size of the completed transfer
    uint32_t msgSize = SSI_BUFFER_SIZE - xferSize;
    g_ui32MessageSizes[g_ui8RxWriteIndex] = msgSize;

    // ------------------------------
    //TODO: intercede here and queue up fast responses to 0x74 and 0x75 with int SSI3_QueueResponse(uint8_t* data, size_t length) if necessary
    // OR
    //pass it onto
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    datapackage.addr = &g_ui8SSIRxBuf[g_ui8RxWriteIndex];
    datapackage.numBytes = msgSize;
    xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);

    //Move current rx write index to the next buffer
    g_ui8RxWriteIndex = nextIndex;

    //At this point, an index collision should not be possible (due to the CSn
    //falling edge increment), so index equality means empty
    if (g_ui8TxReadIndex == g_ui8TxWriteIndex)
      g_ui8TXEmpty = true;

    ROM_uDMAChannelDisable(UDMA_CH14_SSI3RX);
    ROM_uDMAChannelDisable(UDMA_CH15_SSI3TX);

    //Enable DMA channel to write in the next buffer position
    ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(SSI3_BASE + SSI_O_DR),
                               g_ui8SSIRxBuf[g_ui8RxWriteIndex], sizeof(g_ui8SSIRxBuf[g_ui8RxWriteIndex]));

    ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, g_ui8SSITxBuf[g_ui8TxReadIndex],
                               (void *)(SSI3_BASE + SSI_O_DR),
                               sizeof(g_ui8SSITxBuf[g_ui8TxReadIndex]));

    ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX);
    ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);

    //Increment receive count
    g_ui32SSIRxWriteCount++;

  }
  else { //Falling edge
    //It is necessary to increment the read pointer here to avoid a race with the data queue logic
    //as the placement of the index accounting at the CSn rising edge could cause new data to drop

    //Update TX head
    g_ui8TxReadIndex = (g_ui8TxReadIndex + 1) % SSI_TX_BUFFER_COUNT;
  }
}

void ConfigureSSI3() {
  //
  // Enable the SSI3 Peripheral.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
  ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_SSI3);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

  // Configure GPIO Pins for SSI3 mode.
  //
  ROM_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
  ROM_GPIOPinConfigure(GPIO_PQ1_SSI3FSS);
  ROM_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
  ROM_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
  ROM_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

  ROM_SSIConfigSetExpClk(SSI3_BASE, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_SLAVE, SPI_CLOCK / 12, 8);

  ROM_SSIEnable(SSI3_BASE);
  ROM_SSIDMAEnable(SSI3_BASE, SSI_DMA_RX | SSI_DMA_TX);
}

void ConfigureDMA() {
  //Enable uDMA
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

  //Register DMA interrupt to handler
  IntRegister(INT_UDMAERR, uDMAErrorHandler);

  //Enable interrupt
  ROM_IntEnable(INT_UDMAERR);

  // Enable the uDMA controller.
  ROM_uDMAEnable();

  // Point at the control table to use for channel control structures.
  ROM_uDMAControlBaseSet(pui8ControlTable);

  //Assign DMA channels to SSI3
  ROM_uDMAChannelAssign(UDMA_CH14_SSI3RX);
  ROM_uDMAChannelAssign(UDMA_CH15_SSI3TX);

  // Put the attributes in a known state for the uDMA SSI0RX channel.  These
  // should already be disabled by default.
  ROM_uDMAChannelAttributeDisable(UDMA_CH14_SSI3RX, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                  (UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK));

  // Configure the control parameters for the primary control structure for
  // the SSIORX channel.
  ROM_uDMAChannelControlSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                            UDMA_ARB_4);


  //Enable DMA channel to write in the next buffer position
  ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT,
                             UDMA_MODE_BASIC,
                             (void *)(SSI3_BASE + SSI_O_DR),
                             g_ui8SSIRxBuf[g_ui8RxWriteIndex], sizeof(g_ui8SSIRxBuf[g_ui8RxWriteIndex]));

  // Configure TX

  //
  // Put the attributes in a known state for the uDMA SSI0TX channel.  These
  // should already be disabled by default.
  //
  ROM_uDMAChannelAttributeDisable(UDMA_CH15_SSI3TX,
                                  UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK);

  //
  // Configure the control parameters for the primary control structure for
  // the SSIORX channel.
  //
  ROM_uDMAChannelControlSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  //Prefill buffers with 0x00 and mem card identity response
  size_t i, j;
  for (i = 0; i < SSI_TX_BUFFER_COUNT; i++)
  {
    for (j = 0; j < sizeof(g_ui8SSITxBuf[i]); j++)
    {
      g_ui8SSITxBuf[i][j] = 0x00;
    }

    // set mem card response
    // first bytes [0] and [1] the command being sent
    // response bytes [2] [3] [4] [5] are the reply
    // Some sort of USB adapter is 0x04060000
    // therefore set [2] to 0x04 and [3] to 06
    g_ui8SSITxBuf[i][2] = 0x04;
    g_ui8SSITxBuf[i][3] = 0x06;
  }

  //Enable DMA channel to write in the next buffer position
  ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                             UDMA_MODE_BASIC, g_ui8SSITxBuf[g_ui8TxReadIndex],
                             (void *)(SSI3_BASE + SSI_O_DR),
                             sizeof(g_ui8SSITxBuf[g_ui8TxReadIndex]));

  ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX);
  ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);
}

void ConfigureCSInterrupt() {
  IntRegister(INT_GPIOQ1, gpioQ1IntHandler);
  ROM_GPIOIntTypeSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_BOTH_EDGES | GPIO_DISCRETE_INT);
  ROM_IntPrioritySet(INT_GPIOQ1, 0xA0); // priority 5
  ROM_GPIOIntEnable(GPIO_PORTQ_BASE, GPIO_INT_PIN_1);
  ROM_IntEnable(INT_GPIOQ1);
}

void SSI3_Begin() {
  ConfigureSSI3();
  ConfigureDMA();
  ConfigureCSInterrupt();
}

void SSI3_End() {
  ROM_SSIDisable(SSI3_BASE);
}

bool SSI3_IsMessageAvailable() {
  return g_ui32SSIRxWriteCount > g_ui32SSIRxReadCount;
}

uint32_t SSI3_GetMessageSize() {
  return g_ui32MessageSizes[g_ui8RxReadIndex];
}

uint8_t* SSI3_PopMessage() {
  uint8_t readIndex = g_ui8RxReadIndex;

  //Increment read index, keep between 0 and buffer count
  g_ui8RxReadIndex++;
  if (g_ui8RxReadIndex >= SSI_RX_BUFFER_COUNT) g_ui8RxReadIndex = 0;

  g_ui32SSIRxReadCount++; //Increment read count

  //Return pointer to array containing the message to read
  return g_ui8SSIRxBuf[readIndex];
}

//queueResponse prepares from 1 to SSI_TX_BUFFER_COUNT messages for the
//EXI channel to read. These are done in fixed size 1024 byte transfers
//so a requested length of >1K will result in messages which need multiple
//EXI transfers to fully retrieve

//FIXME: Initiating an EXI transaction on an empty queue while starting to
//queue data is possibly UNDEFINED. The user CSn handler was changed to
//improve the index accounting, but it needs testing
int SSI3_QueueResponse(uint8_t* data, size_t length) {
  size_t neededBuffers = (length / SSI_BUFFER_SIZE) + 1;

  if (neededBuffers > SSI_TX_BUFFER_COUNT)
    return -1;

  //BEGIN CRITICAL SECTION
  //IntMasterDisable();
  taskENTER_CRITICAL();

  //TODO: review logic here
  if(!g_ui8TXEmpty && g_ui8TxWriteIndex == g_ui8TxReadIndex)
  {
    taskEXIT_CRITICAL();
    return -2; //Write will overflow
  }

  size_t i;
  for (i = 0; i < neededBuffers; i++) {
    memcpy(g_ui8SSITxBuf[g_ui8TxWriteIndex], data + (i * SSI_BUFFER_SIZE), length % SSI_BUFFER_SIZE);
    //Update RX head
    g_ui8TxWriteIndex = (g_ui8TxWriteIndex + 1) % SSI_TX_BUFFER_COUNT;
    g_ui8TXEmpty = false;
    if (length > SSI_BUFFER_SIZE)
      length -= SSI_BUFFER_SIZE;
  }

  //END CRITICAL SECTION
  //IntMasterEnable();
  taskEXIT_CRITICAL();
  //You might be tempted to move the critical section end before the memcpy, but that's a bad idea as
  //the buffer empty accounting is interrupt mediated with the user CSn logic instead of having the
  //DMA engine itself do buffer management. Moving the critical section will introduce a race.

  return 0;
}
