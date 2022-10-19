/******************************************************************************
*
* File Name: tft_task.c
*
* Description: This file contains task and functions related to the tft-task
* that demonstrates controlling a tft display using the EmWin Graphics Library.
* The project displays a start up screen with Cypress logo and
* text "CYPRESS EMWIN GRAPHICS DEMO tft DISPLAY".
*
* The project then displays strings received from the UART task.
*
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "stdbool.h"
#include <string.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "GUI.h"
#include "mtb_st7789v.h"
#include "bitmaps.h"
#include "tft_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define STARTUP_DELAY               (2000/*ms*/) /* Amount of time to show the startup logo */
#define Y_STEP_SIZE					(15u)
#define LINE_LENGTH					(40u)
#define NUMBER_OF_LINES				(16u)

/* The pins are defined by the st7789v library. If the display is being used on different hardware the mappings will be different. */
const mtb_st7789v_pins_t tft_pins =
{
    .db08 = CYBSP_J2_2,
    .db09 = CYBSP_J2_4,
    .db10 = CYBSP_J2_6,
    .db11 = CYBSP_J2_10,
    .db12 = CYBSP_J2_12,
    .db13 = CYBSP_D7,
    .db14 = CYBSP_D8,
    .db15 = CYBSP_D9,
    .nrd  = CYBSP_D10,
    .nwr  = CYBSP_D11,
    .dc   = CYBSP_D12,
    .rst  = CYBSP_D13
};


/*******************************************************************************
* Forward Function Prototypes
*******************************************************************************/
void show_startup_screen(void);


/*******************************************************************************
* Function Name: void tft_task(void *arg)
********************************************************************************
*
* Summary: The Following functions are performed in this task
*           1. Initializes the EmWin display engine
*           2. Displays startup screen for 3 seconds
*           3. In an infinite loop, displays strings from UART task
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void tft_task(void *arg)
{
    cy_rslt_t result;
    char rxStringBuffer[LINE_LENGTH];
    extern QueueHandle_t stringQueue;
    BaseType_t lineNumber = 0;
    char screenBuffer[NUMBER_OF_LINES][LINE_LENGTH];
    bool scrollFlag = false;
    BaseType_t screenBufferOffset = 0;
    char (*ptrLineBuffer)[LINE_LENGTH];

    /* Initialize the display controller */
    result = mtb_st7789v_init8(&tft_pins);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    
    /* To avoid compiler warning */
    (void)result;
    
    GUI_Init();

    /* Show the startup screen */
    show_startup_screen();
    vTaskDelay(STARTUP_DELAY);

    /* Clear the display */
    GUI_Clear();

    ptrLineBuffer = screenBuffer;

    for(;;)
    {
        xQueueReceive(stringQueue, rxStringBuffer, portMAX_DELAY);
        strcpy((char *) ptrLineBuffer, rxStringBuffer);
        GUI_DispStringAtCEOL((char *) ptrLineBuffer++, 0, lineNumber * Y_STEP_SIZE);
        if(++lineNumber >= NUMBER_OF_LINES)
        {
        	scrollFlag = true;
        	lineNumber = 0;
        }
        if(true == scrollFlag)
        {
        	ptrLineBuffer = screenBuffer;
        	for(uint8_t i = 0; i < NUMBER_OF_LINES; i++)
        	{
        		printf("%s\r\n", (char *) ptrLineBuffer++);
        	}
        }
    }
}

/*******************************************************************************
* Function Name: void show_startup_screen(void)
********************************************************************************
*
* Summary: This function displays the startup screen.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void show_startup_screen(void)
{
    /* Set font size, foreground color, and background color */
    GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetColor(GUI_WHITE);
    GUI_SetBkColor(GUI_BLACK);

    /* Clear the screen */
    GUI_Clear();

    /* Draw the Cypress Logo */
    GUI_DrawBitmap(&bmCypressLogoFullColor_PNG_316pixels, 4, 4);

    /* Print the text Cypress EMWIN GRAPHICS DEMO TFT DISPLAY */
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("CYPRESS", 160, 120);
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("EMWIN GRAPHICS DEMO", 160, 140);
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("TFT DISPLAY", 160, 160);
}


/* END OF FILE */
