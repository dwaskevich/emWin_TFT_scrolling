/******************************************************************************
* File Name: uart_receive_task.c
*
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_receive_task.h"
#include "queue.h"

/* Queue handle for UART event */
static QueueHandle_t uartQueue;

/* Queue handle for string message */
extern QueueHandle_t stringQueue;

void uart_event_callback(void *callback_arg, cyhal_uart_event_t event)
{
	char c;
	size_t size = 1;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    if((event & CYHAL_UART_IRQ_RX_NOT_EMPTY) != 0u)
    {
    	while(cyhal_uart_readable(&cy_retarget_io_uart_obj))
		{
    		cyhal_uart_read(&cy_retarget_io_uart_obj, (void *)&c, &size);
		}

    	xQueueSendFromISR(uartQueue, &c, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
    }
}

void task_uart_receive(void *param)
{
	char rxChar;
	char stringBuffer[LINE_LENGTH];
	BaseType_t charCount = 0;

	/* create UART queue */
	uartQueue = xQueueCreate(1, sizeof(char));

	/* create string queue */
	stringQueue = xQueueCreate(20, sizeof(stringBuffer));

	cyhal_gpio_init(P11_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	/* UART interface is already enabled using ReTarget - IO. Here let us enable the Event CYHAL_UART_IRQ_RX_NOT_EMPTY  */
	cyhal_uart_register_callback(&cy_retarget_io_uart_obj, uart_event_callback, NULL);
	cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY, 3u, true);

	for(;;)
	{
	    if(xQueueReceive(uartQueue, &rxChar, portMAX_DELAY) == pdTRUE)
	    {
	    	/* echo character */
	    	cyhal_uart_putc(&cy_retarget_io_uart_obj, rxChar);
	    	cyhal_gpio_toggle_internal(P11_1);

	    	/* build input string buffer */
	    	if('\r' == rxChar || '\n' == rxChar)
	    	{
	    		stringBuffer[charCount] = '\0';
	    		printf("\r\nString length = %d\tString complete - %s\r\n", (int) charCount, stringBuffer);
	    		xQueueSend(stringQueue, stringBuffer, 100);
	    		charCount = 0;
	    	}
	    	else
	    	{
	    		stringBuffer[charCount] = rxChar;
	    		if(charCount < sizeof(stringBuffer) - 1)
	    			charCount++;
	    	}
	    }
	}
}
