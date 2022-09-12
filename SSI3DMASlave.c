// Ownasaurus

#include <Payloads.h>
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

#define SPI_CLOCK_SPEED 8000000 // 8Mhz. GCN supports up to 32MHz, but this board can only do up to 10MHz as slave reliably
#define DMA_SIZE 1024 // Max size of DMA transfers

// buffers
uint8_t TX_Buffer[DMA_SIZE];
uint8_t RX_Buffer_A[DMA_SIZE];
uint8_t RX_Buffer_B[DMA_SIZE];

// primary and alternate control tables
#pragma DATA_ALIGN(uDMAControlTable, 1024)
uint8_t uDMAControlTable[1024];

// Initialization function
void SSI3_Init_SPI_0_0()
{
    // ----- Enable SSI3 -----

    // Enable the relevant peripherals for SSI3 & uDMA functionality
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Properly configure all four SSI3 pins
    ROM_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    ROM_GPIOPinConfigure(GPIO_PQ1_SSI3FSS);
    ROM_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    ROM_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    ROM_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Set and enable SPI slave mode operating at 8MHz, with uDMA
    ROM_SSIConfigSetExpClk(SSI3_BASE, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_0, SSI_MODE_SLAVE, SPI_CLOCK_SPEED / 12, 8);
    ROM_SSIEnable(SSI3_BASE);
    ROM_SSIDMAEnable(SSI3_BASE, SSI_DMA_RX | SSI_DMA_TX);

    // ----- Enable DMA -----

    // Enable the uDMA controller.
    ROM_uDMAEnable();

    //Assign DMA channels to SSI3
    ROM_uDMAChannelAssign(UDMA_CH14_SSI3RX);
    ROM_uDMAChannelAssign(UDMA_CH15_SSI3TX);

    // Set control table
    ROM_uDMAControlBaseSet(uDMAControlTable);

    // Configure Tx for basic mode
    ROM_uDMAChannelAttributeDisable(UDMA_CH15_SSI3TX, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    ROM_uDMAChannelControlSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);
    ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_Buffer,
                               (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer));

    // Prefill default transmit buffer with 0x00 and mem card identity response
    // first bytes [0] and [1] the command being sent, and the bytes are not being read by the GCN
    // response bytes [2] [3] [4] [5] are the reply
    // Some sort of USB adapter is 0x04060000
    TX_Buffer[2] = 4;
    TX_Buffer[3] = 6;
    TX_Buffer[4] = 0;
    TX_Buffer[5] = 0;
    ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);

    // Configure Rx for ping-pong mode
    ROM_uDMAChannelAttributeDisable(UDMA_CH14_SSI3RX, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK));
    ROM_uDMAChannelControlSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);
    ROM_uDMAChannelControlSet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);
    ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                               RX_Buffer_A, sizeof(RX_Buffer_A));
    ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX);

    // ----- Enable Interrupts -----

    ROM_IntEnable(INT_UDMAERR); // enable dma error interrupts
    ROM_SSIIntEnable(SSI3_BASE, SSI_DMATX | SSI_DMARX); // enable DMA transfer complete and Rx timeout interrupts

    ROM_GPIOIntTypeSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_RISING_EDGE | GPIO_DISCRETE_INT); // signifies SPI transfer complete
    ROM_IntPrioritySet(INT_GPIOQ1, 0xA0); // priority 5
    ROM_GPIOIntEnable(GPIO_PORTQ_BASE, GPIO_INT_PIN_1); // enable interrupts on this pin
    ROM_IntEnable(INT_GPIOQ1); // enable the actual interrupt

    // ----- Initialize -----

    InitPayloadData();
}

void ResetSSI3(void)
{
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_SSI3);

    ROM_SSIConfigSetExpClk(SSI3_BASE, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_SLAVE, SPI_CLOCK_SPEED / 12, 8);

    ROM_SSIEnable(SSI3_BASE);
    ROM_SSIDMAEnable(SSI3_BASE, SSI_DMA_RX | SSI_DMA_TX);
}

void SSI3IntHandler(void)
{
    // Clear interrupt flag
    uint32_t ui32Status;
    ui32Status = ROM_GPIOIntStatus(SSI3_BASE, 1);
    ROM_GPIOIntClear(SSI3_BASE, ui32Status);

    // DMA is complete, so get it ready to go again

    // Ready the next Rx buffer
    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);

    if(ui32Status == UDMA_MODE_STOP) // buffer A is done with a transfer
    {
        // So switch to B
        ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_B, sizeof(RX_Buffer_B));
        // Send contents of A
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        datapackage.addr = RX_Buffer_A;
        datapackage.numBytes = 1024;
        xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);
    }

    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT);

    if(ui32Status == UDMA_MODE_STOP) // buffer B is done with a transfer
    {
        // so switch back to A
        ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_A, sizeof(RX_Buffer_A));
        // Send contents of B
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        datapackage.addr = RX_Buffer_B;
        datapackage.numBytes = 1024;
        xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);
    }

    //ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX); // this should never get disabled? so no need to re-enable?

    // Ready the next Tx buffer
    if(!ROM_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_Buffer,
                                       (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer));
        ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    }
}

//*****************************************************************************
//
// CS Rising Edge Interrupt Handler
//
//*****************************************************************************
void Q1IntHandler(void)
{
    // Clear interrupt flag
    uint32_t ui32Status;
    ui32Status = ROM_GPIOIntStatus(GPIO_PORTQ_BASE, 1);
    ROM_GPIOIntClear(GPIO_PORTQ_BASE, ui32Status);

    // Check if the primary is active, and if so, flush it
    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);
    if(ui32Status == UDMA_MODE_PINGPONG)
    {
        uint32_t xferSize = ROM_uDMAChannelSizeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);
        //Get the size of the remainder of the transfer
        uint32_t msgSize = DMA_SIZE - xferSize;

        ROM_uDMAChannelDisable(UDMA_CH14_SSI3RX);

        // Send contents of A
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        datapackage.addr = RX_Buffer_A;
        datapackage.numBytes = msgSize;
        xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);

        // reset back to A
        ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_A, sizeof(RX_Buffer_A));

        ResetSSI3();
        ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX);
    }

    // Check if the alternate is active, and if so, flush it
    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT);
    if(ui32Status == UDMA_MODE_PINGPONG)
    {
        uint32_t xferSize = ROM_uDMAChannelSizeGet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT);
        //Get the size of the remainder of the transfer
        uint32_t msgSize = DMA_SIZE - xferSize;

        ROM_uDMAChannelDisable(UDMA_CH14_SSI3RX);

        // Send contents of B
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        datapackage.addr = RX_Buffer_B;
        datapackage.numBytes = msgSize;
        xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);

        // reset back to A
        ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_A, sizeof(RX_Buffer_A));

        ResetSSI3();
        ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX);
    }
}

// callback if an uDMA error occurs
void uDMAErrorHandler(void)
{
    static uint32_t uDMAErrCount = 0;

    uint32_t ui32Status;

    ui32Status = ROM_uDMAErrorStatusGet();

    if (ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        uDMAErrCount++;
    }
}
