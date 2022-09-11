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
                               (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer);

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
                               RX_Buffer_A, sizeof(RX_Buffer_A);
    ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX);

    // ----- Enable Interrupts -----

    ROM_IntEnable(INT_UDMAERR); // enable dma error interrupts
    ROM_SSIIntEnable(SSI3_BASE, SSI_DMATX | SSI_DMARX); // enable DMA transfer complete interrupts

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
                                       RX_Buffer_B, sizeof(RX_Buffer_B);
        // Send contents of A
        if(msgSize > 0)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            datapackage.addr = RX_Buffer_A;
            datapackage.numBytes = msgSize;
            xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);
        }
    }

    ui32Status = MAP_uDMAChannelModeGet(UDMA_CH14_SSI3RX | UDMA_ALT_SELECT);
    uint32_t msgSize = 1024; // I think this is always true? If this particular interrupt is hitting? Need to confirm.

    if(ui32Status == UDMA_MODE_STOP) // buffer B is done with a transfer
    {
        // so switch back to A
        ROM_uDMAChannelTransferSet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(SSI3_BASE + SSI_O_DR),
                                       RX_Buffer_A, sizeof(RX_Buffer_A);
        // Send contents of B
        if(msgSize > 0)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            datapackage.addr = RX_Buffer_B;
            datapackage.numBytes = msgSize;
            xQueueSendFromISR(incomingEXIData, &datapackage, &xHigherPriorityTaskWoken);
        }
    }

    // Ready the next Tx buffer
    if(!ROM_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX))
    {
        ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_Buffer,
                                       (void *)(SSI3_BASE + SSI_O_DR), sizeof(TX_Buffer);
    }

    //ROM_uDMAChannelEnable(UDMA_CH14_SSI3RX); // this should never get disabled?
    ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);

    bool cs_n = GPIOPinRead(GPIO_PORTQ_BASE, GPIO_PIN_1);

    if (cs_n) { //Rising edge
      // handle special cases that are time-sensitive
      if(g_ui8SSIRxBuf[g_ui8RxWriteIndex][0] == 0x74) // All star request
      {
          ROM_uDMAChannelDisable(UDMA_CH15_SSI3TX);
          /* When a uDMA transfer is completed, the channel is **automatically disabled** by the uDMA controller.
           * Therefore, this function should be called prior to starting up any new transfer.
           * Therefore we probably didn't need to manually call the disable function.*/

          ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                                                 UDMA_MODE_BASIC, allstar_data,
                                                 (void *)(SSI3_BASE + SSI_O_DR),
                                                 sizeof(allstar_data));
          ResetSSI3(); // resetting SSI3 forces the Tx and Rx FIFOs to flush/clear
          ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);
          g_ui32SSIRxWriteCount++;
          return;
      }
      else if(g_ui8SSIRxBuf[g_ui8RxWriteIndex][0] == 0x75) // Adventure mode
      {
          ROM_uDMAChannelDisable(UDMA_CH15_SSI3TX);
          ROM_uDMAChannelTransferSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
                                                 UDMA_MODE_BASIC, adventure_data,
                                                 (void *)(SSI3_BASE + SSI_O_DR),
                                                 sizeof(adventure_data));
          ResetSSI3(); // resetting SSI3 forces the Tx and Rx FIFOs to flush/clear
          ROM_uDMAChannelEnable(UDMA_CH15_SSI3TX);
          g_ui32SSIRxWriteCount++;
          return;
      }

      uint32_t xferSize = ROM_uDMAChannelSizeGet(UDMA_CH14_SSI3RX | UDMA_PRI_SELECT);

      //Get the size of the completed transfer
      uint32_t msgSize = SSI_BUFFER_SIZE - xferSize;
      g_ui32MessageSizes[g_ui8RxWriteIndex] = msgSize;



      //Move current rx write index to the next buffer
      g_ui8RxWriteIndex = (g_ui8RxWriteIndex + 1) % SSI_RX_BUFFER_COUNT;

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

      ResetSSI3(); // resetting SSI3 forces the Tx and Rx FIFOs to flush/clear

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

    // TODO: send message with IPC to flush received data out as UDP packet
}
