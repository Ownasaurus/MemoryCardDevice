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
#include "message_buffer.h"

#include "SSI3DMASlave.h"
#include "main.h"

extern MessageBufferHandle_t outgoingUDPData;

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
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Properly configure all four SSI3 pins
    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PQ1_SSI3FSS);
    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Set and enable SPI slave mode operating at 8MHz, with uDMA
    MAP_SSIConfigSetExpClk(SSI3_BASE, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_0, SSI_MODE_SLAVE, SPI_CLOCK_SPEED / 12, 8);
    MAP_SSIEnable(SSI3_BASE);
    MAP_SSIDMAEnable(SSI3_BASE, SSI_DMA_RX | SSI_DMA_TX);

    // ----- Enable DMA -----

    // Enable the uDMA controller.
    MAP_uDMAEnable();

    //Assign DMA channels to SSI3
    MAP_uDMAChannelAssign(UDMA_CH14_SSI3RX);
    MAP_uDMAChannelAssign(UDMA_CH15_SSI3TX);

    // Set control table
    MAP_uDMAControlBaseSet(uDMAControlTable);

    // Configure Tx for basic mode
    MAP_uDMAChannelAttributeDisable(UDMA_CH15_SSI3TX, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    MAP_uDMAChannelControlSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);
    MAP_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_Buffer,
                               (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer));

    // Prefill default transmit buffer with 0x00 and mem card identity response
    // first bytes [0] and [1] the command being sent, and the bytes are not being read by the GCN
    // response bytes [2] [3] [4] [5] are the reply
    // Some sort of USB adapter is 0x04060000
    TX_Buffer[2] = 4;
    TX_Buffer[3] = 6;
    TX_Buffer[4] = 0;
    TX_Buffer[5] = 0;
    MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);

    // Configure Rx for ping-pong mode
    MAP_uDMAChannelAttributeDisable(UDMA_CH14_SSI3RX, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK));
    MAP_uDMAChannelControlSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);
    MAP_uDMAChannelControlSet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);
    MAP_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                               RX_Buffer_A, sizeof(RX_Buffer_A));
    MAP_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                               RX_Buffer_B, sizeof(RX_Buffer_B));
    MAP_uDMAChannelEnable(UDMA_CH14_SSI3RX);

    // ----- Enable Interrupts -----

    MAP_IntEnable(INT_UDMAERR); // enable dma error interrupts
    MAP_SSIIntDisable(SSI3_BASE, SSI_DMATX | SSI_DMARX | SSI_TXEOT | SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR); // start with a clean slate
    MAP_SSIIntEnable(SSI3_BASE, SSI_DMATX | SSI_DMARX); // enable DMA transfer complete and Rx timeout interrupts
    MAP_IntPrioritySet(INT_SSI3, 0xA0); // priority 5
    MAP_IntEnable(INT_SSI3); // enable SSI3 interrupts in general

    MAP_GPIOIntTypeSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_RISING_EDGE | GPIO_DISCRETE_INT); // signifies SPI transfer complete
    MAP_IntPrioritySet(INT_GPIOQ1, 0xA0); // priority 5
    MAP_GPIOIntEnable(GPIO_PORTQ_BASE, GPIO_INT_PIN_1); // enable interrupts on this pin
    MAP_IntEnable(INT_GPIOQ1); // enable the actual interrupt
}

void ResetSSI3(void)
{
    // https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/319424/ssi-buffer-reset
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_SSI3);

    MAP_SSIConfigSetExpClk(SSI3_BASE, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_SLAVE, SPI_CLOCK_SPEED / 12, 8);

    MAP_SSIEnable(SSI3_BASE);
    MAP_SSIIntEnable(SSI3_BASE, SSI_DMATX | SSI_DMARX);
    MAP_IntPrioritySet(INT_SSI3, 0xA0);
    MAP_IntEnable(INT_SSI3);
    MAP_SSIDMAEnable(SSI3_BASE, SSI_DMA_RX | SSI_DMA_TX);
}

void SSI3IntHandler(void)
{
    // Clear interrupt flag
    uint32_t ui32Status;
    ui32Status = MAP_SSIIntStatus(SSI3_BASE, 1);
    if(ui32Status & SSI_DMARX) // DMA Rx done
    {
        MAP_SSIIntClear(SSI3_BASE, SSI_DMARX);
    }
    if(ui32Status & SSI_DMATX) // DMA Tx done
    {
        MAP_SSIIntClear(SSI3_BASE, SSI_DMATX);
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // DMA is complete, so get it ready to go again

    // Ready the next Rx buffer
    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);

    if(ui32Status == UDMA_MODE_STOP) // buffer A is done with a transfer
    {
        // Copy/send contents of A
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_A, sizeof(RX_Buffer_A), &xHigherPriorityTaskWoken);

        // So set up A for next time!
        MAP_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_A, sizeof(RX_Buffer_A));
    }

    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT);

    if(ui32Status == UDMA_MODE_STOP) // buffer B is done with a transfer
    {
        // Copy/send contents of B
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_B, sizeof(RX_Buffer_B), &xHigherPriorityTaskWoken);

        // So set up B for next time!
        MAP_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_B, sizeof(RX_Buffer_B));
    }

    // Ready the next Tx buffer
    if(!MAP_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX))
    {
        MAP_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_Buffer,
                                       (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer));
        MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    }
}

//*****************************************************************************
//
// CS Rising Edge Interrupt Handler
//
//*****************************************************************************
void Q1IntHandler(void)
{
    // Clear CS_n interrupt flag
    uint32_t ui32Status;
    ui32Status = MAP_GPIOIntStatus(GPIO_PORTQ_BASE, 1);
    MAP_GPIOIntClear(GPIO_PORTQ_BASE, ui32Status);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t TX_needs_resetting = 1;

    // inactive one should be size 1024
    uint32_t xferSizePrimary = MAP_uDMAChannelSizeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);
    uint32_t xferSizeAlternate = MAP_uDMAChannelSizeGet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT);

    if(xferSizePrimary == 1024 && xferSizeAlternate == 1024) // edge case where neither has data left
    {
        // notify ethernet function that our frame is done
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_A, 1, &xHigherPriorityTaskWoken);
    }
    else if(xferSizePrimary == 1024) // primary has nothing, alternate has data
    {
        MAP_uDMAChannelDisable(UDMA_CH14_SSI3RX);
        ResetSSI3(); // flush
        // send the last bit of the ALTERNATE
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_B, (1024-xferSizeAlternate), &xHigherPriorityTaskWoken);

        // notify ethernet function that our frame is done
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_A, 1, &xHigherPriorityTaskWoken);

        if(RX_Buffer_B[0] == 0x74) // all star?
        {
            TX_needs_resetting = 2;
        }
        else if(RX_Buffer_B[0] == 0x75) // adventure mode?
        {
            TX_needs_resetting = 3;
        }
    }
    else if(xferSizeAlternate == 1024) // primary has data, alternate has nothing
    {
        MAP_uDMAChannelDisable(UDMA_CH14_SSI3RX);
        ResetSSI3(); // flush
        // send the last bit of the PRIMARY
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_A, (1024-xferSizePrimary), &xHigherPriorityTaskWoken);

        // notify ethernet function that our frame is done
        xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(outgoingUDPData, &RX_Buffer_A, 1, &xHigherPriorityTaskWoken);

        if(RX_Buffer_A[0] == 0x74) // all star?
        {
            TX_needs_resetting = 2;
        }
        else if(RX_Buffer_A[0] == 0x75) // adventure mode?
        {
            TX_needs_resetting = 3;
        }
    }
    else // both transfers are in-process? should be impossible!
    {
        // should never get here
    }

    // reset the Rx DMA transfers
    MAP_uDMAChannelDisable(UDMA_CH14_SSI3RX);
    MAP_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                           RX_Buffer_A, sizeof(RX_Buffer_A));
    MAP_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                           RX_Buffer_B, sizeof(RX_Buffer_B));
    MAP_uDMAChannelEnable(UDMA_CH14_SSI3RX);

    // reset the Tx DMA transfers
    if(TX_needs_resetting == 1) // default
    {
        MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);
        MAP_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_Buffer,
                                       (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer));
        //ResetSSI3(); // resetting SSI3 forces the Tx and Rx FIFOs to flush/clear
        MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    }
    else if(TX_needs_resetting == 2) // all star?
    {
        ROM_uDMAChannelDisable(UDMA_CH15_SSI3TX);
        ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                                               UDMA_MODE_BASIC, allstar_data,
                                               (void *)(SSI3_BASE + SSI_O_DR),
                                               sizeof(allstar_data));
        //ResetSSI3(); // resetting SSI3 forces the Tx and Rx FIFOs to flush/clear
        ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    }
    else if(TX_needs_resetting == 3) // adventure?
    {
        ROM_uDMAChannelDisable(UDMA_CH15_SSI3TX);
        ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                                               UDMA_MODE_BASIC, adventure_data,
                                               (void *)(SSI3_BASE + SSI_O_DR),
                                               sizeof(adventure_data));
        //ResetSSI3(); // resetting SSI3 forces the Tx and Rx FIFOs to flush/clear
        ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    }
}

// callback if an uDMA error occurs
void uDMAErrorHandler(void)
{
    static uint32_t uDMAErrCount = 0;

    uint32_t ui32Status;

    ui32Status = MAP_uDMAErrorStatusGet();

    if (ui32Status)
    {
        MAP_uDMAErrorStatusClear();
        uDMAErrCount++;
    }
}
