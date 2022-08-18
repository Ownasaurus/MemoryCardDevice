// main.c
// Ownasaurus

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "main.h"

// Texas Instruments
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"

// FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "message_buffer.h"

// Other
//#include "SSI3DMASlave.h"
#include "DataPackage.h"

// Task function prototypes
//void ethernetTask(void *pvParameters);
void heartbeatTask(void *pvParameters);
void uart0Task(void *pvParameters);
//void EXISendTask(void *pvParameters);

// Other function prototypes
void UART0_Begin();
//void Ethernet_Begin();
void task_print(char* fmt, ...);

// FreeRTOS data structures
QueueHandle_t incomingEXIData;
QueueHandle_t outgoingEXIData;
QueueHandle_t printableData;

int main(void)
{
    // Initialize system clock to 120 MHz
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    // Initialize the virtual serial port
    UART0_Begin();
    UARTprintf("UART0 Initialized.\r\n");

    // Create IPC structures
    incomingEXIData = xQueueCreate(8, sizeof(DataPackage_t)); // length of 8 should be more than enough. size of 8 bytes = 4 for address + 4 for data length
    ASSERT(incomingEXIData != NULL);
    outgoingEXIData = xQueueCreate(8, sizeof(struct pbuf *)); // length of 8 should be more than enough. size of 8 bytes = 4 for address + 4 for data length
    ASSERT(outgoingEXIData != NULL);
    printableData = xQueueCreate(8, 1024);
    ASSERT(printableData != NULL);

    // Create tasks
    xTaskCreate(heartbeatTask, (const portCHAR *)"HB", 1024, NULL, 1, NULL);
    xTaskCreate(uart0Task, (const portCHAR *)"UART0", 1024, NULL, 2, NULL);
    //xTaskCreate(ethernetTask, (const portCHAR *)"ENET", 4096, NULL, 4, NULL);
    //xTaskCreate(EXISendTask, (const portCHAR *)"EXDataPackage.hI", 4096, NULL, 5, NULL);

    // This should start up all of our tasks and never progress past this line of code
    vTaskStartScheduler();

    // Code should never reach this point
    return 0;
}

// task-safe print function that supports formatted strings and defers it to a low priority printing task
// supported max string length: 1024 characters
void task_print(char* fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 1024, fmt, args);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(NVIC_ACTIVE0_R == 0 && NVIC_ACTIVE1_R == 0 && NVIC_ACTIVE2_R == 0 && NVIC_ACTIVE3_R == 0)
    {
        xQueueSend(printableData, buffer, 0);
    }
    else
    {
        xQueueSendFromISR(printableData, buffer, &xHigherPriorityTaskWoken);
    }
    va_end(args);
}

// Initialize UART0
void UART0_Begin()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, SYSTEM_CLOCK);
}

// Low priority task which blocks until text is sent to it to print over UART0
void uart0Task(void *pvParameters)
{
    char textToPrint[1024];

    while(true)
    {
        size_t messageSize = xQueueReceive(printableData, textToPrint, portMAX_DELAY);
        if(messageSize == 0) // nothing was actually received
        {
            continue;
        }

        UARTprintf(textToPrint);
    }
}

// Low priority task which sends periodic heartbeat messages over UART0
void heartbeatTask(void *pvParameters)
{
    uint32_t counter = 1;

    while(true)
    {
        task_print("Heartbeat #%u\r\n", counter);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        counter++;
        //task_print("Free heap: %u\r\n", xPortGetFreeHeapSize());
    }
}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
