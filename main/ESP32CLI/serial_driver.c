/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
    BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER.

    Note1:  This driver is used specifically to provide an interface to the
    FreeRTOS+CLI command interpreter.  It is *not* intended to be a generic
    serial port driver.  Nor is it intended to be used as an example of an
    efficient implementation.  In particular, a queue is used to buffer
    received characters, which is fine in this case as key presses arrive
    slowly, but a DMA and/or RAM buffer should be used in place of the queue in
    applications that expect higher throughput.

    Note2:  This driver does not attempt to handle UART errors.
*/

/* Scheduler includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* Demo application includes. */
#include "serial_driver.h"

/* ESP32 Includes */
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_log.h"

/*-----------------------------------------------------------*/

/* The queue into which received key presses are placed.  NOTE THE COMMENTS AT
THE TOP OF THIS FILE REGARDING THE USE OF QUEUES FOR THIS PURPOSE. */
static QueueHandle_t xRxQueue = NULL;

/* The semaphore used to indicate the end of a transmission. */
static SemaphoreHandle_t xTxCompleteSemaphore = NULL;

static const char *TAG = "uart_events";
/*-----------------------------------------------------------*/

/*
 * The UART interrupt handler is defined in this file to provide more control,
 * but still uses parts of the Xilinx provided driver.
 */
void prvUART_Handler( void *pvNotUsed );

/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( uint32_t ulWantedBaud, UBaseType_t uxQueueLength )
{

const uart_config_t xUartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    /* Create the queue used to hold received characters.  NOTE THE COMMENTS AT
    THE TOP OF THIS FILE REGARDING THE USE OF QUEUES FOR THIS PURPSOE. */
    xRxQueue = xQueueCreate( uxQueueLength, sizeof( char ) );
    configASSERT( xRxQueue );

    /* Create the semaphore used to signal the end of a transmission, then take
    the semaphore so it is in the correct state the first time
    xSerialSendString() is called.  A block time of zero is used when taking
    the semaphore as it is guaranteed to be available (it was just created). */
    xTxCompleteSemaphore = xSemaphoreCreateBinary();
    configASSERT( xTxCompleteSemaphore );
    xSemaphoreTake( xTxCompleteSemaphore, 0 );

    /* Initialise the driver. */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, RX_BUF_SIZE, TX_BUF_SIZE, uxQueueLength, &xRxQueue, 0));
    uart_param_config(EX_UART_NUM, &xUartConfig);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_ERROR);
    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


    xSemaphoreGive( xTxCompleteSemaphore );

    return ( xComPortHandle ) 0;
}
/*-----------------------------------------------------------*/

BaseType_t xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
BaseType_t xReturn;

    /* Only a single port is supported. */
    ( void ) pxPort;

    uart_event_t xEvent;

    /* Obtain a received character from the queue - entering the Blocked state
    (so not consuming any processing time) to wait for a character if one is not
    already available. */
    xReturn = xQueueReceive( xRxQueue, &xEvent, xBlockTime );

    //bzero(pcRxedChar, RD_BUF_SIZE);
    ESP_LOGI(TAG, "uart[%d] xEvent:", EX_UART_NUM);
    switch(xEvent.type) {
        //Event of UART receving data
        /*We'd better handler data event fast, there would be much more data events than
        other types of events. If we take too much time on data event, the queue might
        be full.*/
        case UART_DATA:
            ESP_LOGI(TAG, "[UART DATA]: %d", xEvent.size);
            uart_read_bytes(EX_UART_NUM, pcRxedChar, xEvent.size, portMAX_DELAY);
            ESP_LOGI(TAG, "[DATA EVT]:");
            //uart_write_bytes(EX_UART_NUM, (const char*) pcRxedChar, xEvent.size);
            break;
        //Event of HW FIFO overflow detected
        case UART_FIFO_OVF:
            ESP_LOGI(TAG, "hw fifo overflow");
            // If fifo overflow happened, you should consider adding flow control for your application.
            // The ISR has already reset the rx FIFO,
            // As an example, we directly flush the rx buffer here in order to read more data.
            uart_flush_input(EX_UART_NUM);
            xQueueReset(xRxQueue);
            break;
        //Event of UART ring buffer full
        case UART_BUFFER_FULL:
            ESP_LOGI(TAG, "ring buffer full");
            // If buffer full happened, you should consider encreasing your buffer size
            // As an example, we directly flush the rx buffer here in order to read more data.
            uart_flush_input(EX_UART_NUM);
            xQueueReset(xRxQueue);
            break;
        //Event of UART RX break detected
        case UART_BREAK:
            ESP_LOGI(TAG, "uart rx break");
            break;
        //Event of UART parity check error
        case UART_PARITY_ERR:
            ESP_LOGI(TAG, "uart parity error");
            break;
        //Event of UART frame error
        case UART_FRAME_ERR:
            ESP_LOGI(TAG, "uart frame error");
            break;
        //UART_PATTERN_DET
        case UART_PATTERN_DET:
            ESP_LOGI(TAG, "Pattern detected");
            // uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
            // int pos = uart_pattern_pop_pos(EX_UART_NUM);
            // ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
            // if (pos == -1) {
            //     // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
            //     // record the position. We should set a larger queue size.
            //     // As an example, we directly flush the rx buffer here.
            //     uart_flush_input(EX_UART_NUM);
            // } else {
            //     uart_read_bytes(EX_UART_NUM, pcRxedChar, pos, 100 / portTICK_PERIOD_MS);
            //     uint8_t pat[PATTERN_CHR_NUM + 1];
            //     memset(pat, 0, sizeof(pat));
            //     uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
            //     ESP_LOGI(TAG, "read data: %s", pcRxedChar);
            //     ESP_LOGI(TAG, "read pat : %s", pat);
            // }
            break;
        //Others
        default:
            ESP_LOGI(TAG, "uart event type: %d", xEvent.type);
            break;
    }
    return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
    const TickType_t xMaxWait = 500UL / portTICK_PERIOD_MS;

    /* Only a single port is supported. */
    ( void ) pxPort;

    /* Wait until the string has been transmitted before exiting this function,
    otherwise there is a risk the calling function will overwrite the string
    pointed to by the pcString parameter while it is still being transmitted.
    The calling task will wait in the Blocked state (so not consuming any
    processing time) until the semaphore is available. */
    while(xSemaphoreTake( xTxCompleteSemaphore, xMaxWait ) == pdFALSE);

    /* Start the transmission.  The interrupt service routine will complete the
    transmission if necessary. */
    uart_write_bytes(EX_UART_NUM, pcString, usStringLength);

    // Take to avoid exiting this until the buffer has been written
    //xSemaphoreTake( xTxCompleteSemaphore, xMaxWait );
    xSemaphoreGive( xTxCompleteSemaphore );
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
    const TickType_t xMaxWait = 500UL / portTICK_PERIOD_MS;

    /* Only a single port is supported. */
    ( void ) pxPort;

    while(xSemaphoreTake( xTxCompleteSemaphore, xMaxWait ) == pdFALSE);

    /* Send the character. */
    uart_write_bytes(EX_UART_NUM, &cOutChar, sizeof(cOutChar));

    /* Wait for the transmission to be complete so the semaphore is left in the
    correct state for the next time vSerialPutString() is called. */
//    xSemaphoreTake( xTxCompleteSemaphore, xBlockTime );
    xSemaphoreGive( xTxCompleteSemaphore );

    return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose(xComPortHandle xPort)
{
    /* Not supported as not required by the demo application. */
    ( void ) xPort;
}
/*-----------------------------------------------------------*/

