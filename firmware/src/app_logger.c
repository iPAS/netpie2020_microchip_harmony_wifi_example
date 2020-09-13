/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_logger.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_logger.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

static APP_LOGGER_DATA app_Data;
static enum 
{
    USART_BM_INIT,
    USART_BM_WORKING,
    USART_BM_DONE,
} usartBMState;

typedef struct
{
    uint8_t buffer[LOGGER_QUEUE_ITEM_SIZE];
    uint8_t length;
} logger_queue_item_t;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/**
 * Enqueue a message to UART Tx
 */
BaseType_t logger_send_tx_queue(const char *fmt, ... )
{
    va_list args;
    logger_queue_item_t q_item;
    uint16_t len;

    va_start(args, fmt);
    len = vsnprintf(q_item.buffer, LOGGER_QUEUE_ITEM_SIZE, fmt, args);
    va_end(args);

    q_item.length = strlen(q_item.buffer);
    return xQueueSendToBack(app_Data.q_tx, &q_item, 0);
}


/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter by reading characters from a specified pipe.  The pipeRead function is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.  Typically, the pipe is connected to the application's
    USART receive function, but could be any other Harmony module which supports the pipe interface. 
*/
static void USART_Task (void)
{
    switch (usartBMState)
    {
        default:
        case USART_BM_INIT:
        {            
            usartBMState = USART_BM_WORKING;
            break;
        }

        case USART_BM_WORKING:
        {
            // ******
            // * TX *
// --           while (!DRV_USART_TransmitBufferIsFull(app_Data.handleUSART))
// --           {
                static uint8_t index = 0;
                static logger_queue_item_t q_item;
                bool do_send = true;

                if (index == 0)
                {
                    //uxQueueMessagesWaiting();
                    do_send = xQueueReceive(app_Data.q_tx, &q_item, 0);
                }

                if (do_send)
                {
/* -- Remove for SYS_CONSOLE --
                    DRV_USART_WriteByte(app_Data.handleUSART, q_item.buffer[index]);
                    index = (index == q_item.length-1)? 0 : index+1;
*/
                    
                    SYS_CONSOLE_PRINT("%.*s", q_item.length, q_item.buffer);
                }
// --           }

            // ******
            // * RX *
/* -- Remove for SYS_CONSOLE --
            while (!DRV_USART_ReceiverBufferIsEmpty(app_Data.handleUSART))
            {
                if (uxQueueSpacesAvailable(app_Data.q_rx) > 0)
                {
                    uint8_t c_rx = DRV_USART_ReadByte(app_Data.handleUSART);
                }
            }
*/
            usartBMState = USART_BM_DONE;
            break;
        }

        case USART_BM_DONE:
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            usartBMState = USART_BM_WORKING;
            break;
        }
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_LOGGER_Initialize ( void )

  Remarks:
    See prototype in app_logger.h.
 */
void APP_LOGGER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_Data.state = APP_LOGGER_STATE_INIT;

    app_Data.handleUSART = DRV_HANDLE_INVALID;
    
    /* Message queue */
    app_Data.q_tx = xQueueCreate(LOGGER_QUEUE_SIZE, sizeof(logger_queue_item_t));
    app_Data.q_rx = xQueueCreate(LOGGER_QUEUE_SIZE, sizeof(logger_queue_item_t));
    if (app_Data.q_tx == NULL || app_Data.q_rx == NULL)
    {
        // Some error
    }
    xQueueReset(app_Data.q_tx);
    xQueueReset(app_Data.q_rx);
}


/******************************************************************************
  Function:
    void APP_LOGGER_Tasks ( void )

  Remarks:
    See prototype in app_logger.h.
 */
void APP_LOGGER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_Data.state )
    {
        /* Application's initial state. */
        case APP_LOGGER_STATE_INIT:
        {
            bool appInitialized = true;
/* -- Remove for SYS_CONSOLE --
            if (app_Data.handleUSART == DRV_HANDLE_INVALID)
            {
                app_Data.handleUSART = DRV_USART_Open(
                        APP_LOGGER_DRV_USART, 
                        DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= ( DRV_HANDLE_INVALID != app_Data.handleUSART );
            }
*/
            if (appInitialized)
            {
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
            
                app_Data.state = APP_LOGGER_STATE_SERVICE_TASKS;
            
// --                DRV_USART_WriteByte(app_Data.handleUSART, '.');  // DEBUG: iPAS
                logger_send_tx_queue("Logger ready..\r\n");
            }
            break;
        }

        case APP_LOGGER_STATE_SERVICE_TASKS:
        {
			USART_Task();
        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
