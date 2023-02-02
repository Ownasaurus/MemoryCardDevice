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
#define FAULT_SYSTICK           15          // System Tick, instead of including all of "inc/hw_ints.h"
#define GPIO_PF0_EN0LED0        0x00050005
#define GPIO_PF4_EN0LED1        0x00051005

// FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"

// lwIP
#include "utils/lwiplib.h"

// Other
#include "SSI3DMASlave.h"
#include <Payloads.h>

// Task function prototypes
void ethernetTask(void *pvParameters);
void heartbeatTask(void *pvParameters);
void uart0Task(void *pvParameters);
void EXISendTask(void *pvParameters);
void EXIReceiveTask(void *pvParameters);

// Other function prototypes
void UART0_Begin();
void Ethernet_Begin();
void task_print(char* fmt, ...);
void DisplayIPAddress(uint32_t ui32Addr);

// FreeRTOS data structures
StreamBufferHandle_t incomingEXIData;
StreamBufferHandle_t outgoingUDPData;
QueueHandle_t outgoingEXIData;
QueueHandle_t printableData;

TaskHandle_t EXIReceiveTaskHandle;

#define SYSTICK_INT_PRIORITY    0xE0 // priority 7
#define ETHERNET_INT_PRIORITY   0xC0 // priority 6

#define MESSAGE_BUFFER_SIZE 2048

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

    outgoingUDPData = xStreamBufferCreate(2048, 1); // length of 2048 should be more than enough. Size rarely breaks 1024.
    ASSERT(outgoingUDPData != NULL);
    incomingEXIData = xStreamBufferCreate(2048, 1); // length of 2048 should be more than enough. Size rarely breaks 1024.
    ASSERT(incomingEXIData != NULL);
    outgoingEXIData = xQueueCreate(8, sizeof(struct pbuf *)); // length of 8 should be more than enough. size of 8 bytes = 4 for address + 4 for data length
    ASSERT(outgoingEXIData != NULL);
    printableData = xQueueCreate(8, 1024);
    ASSERT(printableData != NULL);

    // Create tasks
    BaseType_t creationResult;
    creationResult = xTaskCreate(heartbeatTask, (const portCHAR *)"HB", 1024, NULL, 1, NULL);
    ASSERT(creationResult == pdPASS);
    creationResult = xTaskCreate(uart0Task, (const portCHAR *)"UART0", 8192, NULL, 2, NULL);
    ASSERT(creationResult == pdPASS);
    creationResult = xTaskCreate(ethernetTask, (const portCHAR *)"ENET", 10240, NULL, 3, NULL);
    ASSERT(creationResult == pdPASS);
    creationResult = xTaskCreate(EXISendTask, (const portCHAR *)"EXISend", 8192, NULL, 4, NULL);
    ASSERT(creationResult == pdPASS);
    creationResult = xTaskCreate(EXIReceiveTask, (const portCHAR *)"EXIReceive", 8192, NULL, 4, NULL);
    ASSERT(creationResult == pdPASS);

    // This should start up all of our tasks and never progress past this line of code
    vTaskStartScheduler();

    // Code should never reach this point
    return 0;
}

// Initialize Ethernet
// Example initialization code provided by Texas Instruments
void Ethernet_Begin()
{
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACArray[8];

    //
    // this app wants to configure for ethernet LED function.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_GPIOPinConfigure(GPIO_PF0_EN0LED0); // Causes a Fault Interrupt
    MAP_GPIOPinConfigure(GPIO_PF4_EN0LED1);

    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    //
    // Configure Port N1 for as an output for the animation LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize LED to OFF (0)
    //
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

    //
    // Configure SysTick for a periodic interrupt.
    //
    // NOTE: This is already handled by the RTOS

    //
    // Configure the hardware MAC address for Ethernet Controller filtering of
    // incoming packets.  The MAC address will be stored in the non-volatile
    // USER0 and USER1 registers.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
        //
        // We should never get here.  This is an error if the MAC address has
        // not been programmed into the device.  Exit the program.
        // Let the user know there is no MAC address
        //
        UARTprintf("No MAC programmed!\n");
        while(1)
        {
        }
    }

    //
    // Tell the user what we are doing just now.
    //
    //UARTprintf("Waiting for IP.\n");

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

    uint32_t localip = 0xC0A800CD; // 192.168.0.205
    uint32_t netmask = 0xFFFFFF00;
    uint32_t gateway = 0;

    //
    // Initialize the lwIP library, using DHCP.
    //
    //lwIPInit(SYSTEM_CLOCK, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);

    // Static lwIP Initialization
    lwIPInit(SYSTEM_CLOCK, pui8MACArray, localip, netmask, gateway, IPADDR_USE_STATIC);

    //
    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
}

void EXIReceiveTask(void *pvParameters)
{
    EXIReceiveTaskHandle = xTaskGetHandle("EXIReceive");
    ASSERT(EXIReceiveTaskHandle != NULL);

    uint8_t receivedBytes[4096]; // extra room in case of overflow, which shouldn't happen
    uint8_t *currPtr = receivedBytes;
    uint16_t currIndex = 0;

    while(true)
    {
        size_t sizeReceived = xStreamBufferReceive(incomingEXIData, currPtr, 2048, portMAX_DELAY); // blocks until data is available
        taskENTER_CRITICAL();
        if(sizeReceived != 0) // something was actually received
        {
            // update pointers
            currIndex += sizeReceived;
            currPtr += sizeReceived;
        }

        // this hopefully eliminates the possibility of a race condition. double check for remaining data
        if(!xStreamBufferIsEmpty(incomingEXIData))
        {
            taskEXIT_CRITICAL();
            continue;
        }

        // check if semaphore was set, meaning we're ready to flush
        uint32_t ulNotifiedValue = ulTaskNotifyTake(pdTRUE, 0);
        if(ulNotifiedValue != 0) // we actually got a notification
        {
            //if so, dump to ethernetTask
            xStreamBufferSend(outgoingUDPData, &receivedBytes, currIndex, 0);

            //reset buffers and buffer pointers
            currPtr = receivedBytes;
            currIndex = 0;
        }
        taskEXIT_CRITICAL();
    }
}

// Callback when UDP data is received
// NOTE: The callback function is responsible for deallocating the pbuf, but we pass this along to the EXISendTask function
void udp_data_received(void * args, struct udp_pcb * upcb, struct pbuf * p, const ip_addr_t * addr, u16_t port)
{
    // pass the byte pbuf to EXISendTask
    xQueueSend(outgoingEXIData, &p, 0); //send to our exi send task.
}

// Task which handles all outgoing UDP communication
#pragma diag_suppress=112 // suppress the warning that the last line cannot be reached.
void ethernetTask(void *pvParameters)
{
    Ethernet_Begin();
    task_print("Ethernet Initialized.\r\n");

    //--------Wait until we have our IP-----------
    uint32_t currIp = lwIPLocalIPAddrGet();
    while(currIp == 0xFFFFFFFF || currIp == 0)
    {
        task_print("Waiting for IP....\r\n");
        currIp = lwIPLocalIPAddrGet();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    task_print("Obtained IP: ");
    DisplayIPAddress(currIp);
    task_print("\r\n");
    //--------------------------------------------

    struct udp_pcb *pcb_send, *pcb_receive;
    struct pbuf *p;

    pcb_send = udp_new();
    configASSERT(pcb_send != NULL);

    ip_set_option(pcb_send, SOF_BROADCAST); // enable broadcast ability on this PCB

    if(udp_connect(pcb_send, IP_ADDR_BROADCAST, 55559) != ERR_OK)
    {
        task_print("ERROR: Failed to CONNECT!\r\n");
        while(1);
    }

    pcb_receive = udp_new();
    configASSERT(pcb_receive != NULL);

    if(udp_bind(pcb_receive, IP_ADDR_ANY, 55558) != ERR_OK)
    {
        task_print("ERROR: Failed to BIND!\r\n");
        while(1);
    }
    udp_recv(pcb_receive, udp_data_received, NULL);

    uint8_t fullMessageBuffer[2048];
    while(true)
    {
        size_t sizeReceived = xStreamBufferReceive(outgoingUDPData, &fullMessageBuffer, 2048, portMAX_DELAY); // blocks until data is available
        if(sizeReceived == 0) // nothing was actually received
        {
            continue;
        }

        p = pbuf_alloc(PBUF_TRANSPORT, sizeReceived, PBUF_REF);
        p->payload = fullMessageBuffer;
        p->len = sizeReceived;
        err_t tempVal = udp_send(pcb_send, p);
        if(tempVal != ERR_OK)
        {
            // getting ERR_MEM for large packets
            task_print("ERROR: Failed to SEND!\r\n");
        }
        pbuf_free(p);
    }

    udp_remove(pcb_send);
    udp_remove(pcb_receive);
}

// Task which queues up an EXI Response
void EXISendTask(void *pvParameters)
{
    // Initialize the SSI3 peripheral for SPI(0,0)
    SSI3_Init_SPI_0_0();
    task_print("SSI3 Initialized.\r\n");

    struct pbuf *p;

    while(true)
    {
        size_t sizeReceived = xQueueReceive(outgoingEXIData, &p, portMAX_DELAY); // blocks until data is available
        if(sizeReceived == 0) // nothing was actually received
        {
            continue;
        }

        //size_t length = p->len > 1024 ? 1024 : p->len; // limit len to 1024

        // debug prints to UART0
        task_print("UDP Rx: %s\r\n", p->payload);

        // normal EXI behavior
        //int retval = SSI3_QueueResponse((uint8_t*)p->payload, length); // queue up response for next EXI transfer
        pbuf_free(p); // previously-deferred free

        /*if(retval != 0)
        {
            task_print("ERROR: Could not queue EXI reply. Write will overflow!\r\n");
        }*/
    }
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
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

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

//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void
DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];

    //
    // Convert the IP Address into a string.
    //
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
            (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);

    //
    // Display the string.
    //
    xQueueSend(printableData, pcBuf, 0);
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

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
    taskDISABLE_INTERRUPTS(); for(;;);
}
