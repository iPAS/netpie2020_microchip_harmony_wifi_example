/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_pubsub.c

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

#include "app_pubsub.h"
#include "app_netpie.h"
#include "register_mapping.h"

#if defined(DO_TRACE)
#include "app_uart_term.h"
#define TRACE_LOG(...) uart_send_tx_queue(__VA_ARGS__)
#elif defined(DO_LOG)
#include "app_logger.h"
#define TRACE_LOG(...) logger_send_tx_queue(__VA_ARGS__)
#else
#define TRACE_LOG(...)
#endif


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

APP_PUBSUB_DATA app_pubsubData;


#define REGISTER_PUBLISH_INTERVAL_MS 500 

st_register_t *st_prev_registers;  // Allocated for keeping previous values of registers
float *register_prev_values;
uint16_t register_count = 0;


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

#define LEN_OF_ARRAY(arr) (sizeof(arr)/sizeof(arr[0]))

static void mqttclient_callback(const char *sub_topic, const char *message)
{
    TRACE_LOG("[PubSub] --- calling back for updating reg:'%s' with '%s'\n\r", sub_topic, message);  // DEBUG: iPAS

    st_register_t *p_reg = st_registers;
    char *endptr = NULL;
    float value;
    
    while (p_reg->sub_topic != NULL)  // Find the matched reference
    {
        if (strcmp(sub_topic, p_reg->sub_topic) == 0)
        {
            if (strlen(message) == 0)  // Just refresh
            {
                
            }
            else  // Update if valid
            {
                value = strtof(message, &endptr);
            }
            break;
        }
        p_reg++;
    }
    
    
    char msg[50];
    
    if (p_reg->sub_topic == NULL)  // Reference error
    {
        snprintf(msg, sizeof(msg), "Unknown sub_topic:'%s'", sub_topic);
        netpie_publish_log(msg);
    } else
    if (endptr == message)  // Value conversion error
    {
        snprintf(msg, sizeof(msg), "Conversion error on message:'%s'", message);
        netpie_publish_log(msg);
    } 
    else
    {
        if (endptr != NULL)  // Valid
            *p_reg->p_value = value;            
        snprintf(msg, sizeof(msg), "%f", *p_reg->p_value);
        netpie_publish_register(sub_topic, msg);
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_PUBSUB_Initialize ( void )

  Remarks:
    See prototype in app_pubsub.h.
 */
void APP_PUBSUB_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_pubsubData.state = APP_PUBSUB_STATE_INIT;

    // Allocate and initial 'st_prev_registers' for memorizing the latest
    st_register_t *p_reg = st_registers;
    for (; p_reg->sub_topic != NULL; p_reg++)
    {
        register_count++;  // Using counting method because of unknown-size extern array st_registers
    }
    st_prev_registers = (void *)malloc(register_count * sizeof(st_register_t));
    memcpy(st_prev_registers, st_registers, register_count * sizeof(st_register_t));

    // Allocate Buffer for the previous values
    register_prev_values = (void *)malloc(register_count * sizeof(float));
    uint16_t i;
    for (i = 0; i < register_count; i++)
    {
        st_prev_registers[i].p_value = &register_prev_values[i];
        *st_prev_registers[i].p_value = *st_registers[i].p_value;
    }
            
    // Callback function for coping with incoming MQTT message
    netpie_set_callback(mqttclient_callback);
}


/******************************************************************************
  Function:
    void APP_PUBSUB_Tasks ( void )

  Remarks:
    See prototype in app_pubsub.h.
 */
void APP_PUBSUB_Tasks ( void )
{
    static bool first_time = true;

    /* Check the application's current state. */
    switch ( app_pubsubData.state )
    {
        /* Application's initial state. */
        case APP_PUBSUB_STATE_INIT:
        {
            if (netpie_ready())
            {
                app_pubsubData.state = APP_PUBSUB_STATE_REGISTER_UPDATE;
            }
            else
            {
                TRACE_LOG("[PubSub] Wait MQTT ready ...\n\r");  // DEBUG: iPAS
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            break;
        }
        
        /* Loop periodically updating all changed registers */
        case APP_PUBSUB_STATE_REGISTER_UPDATE:
        {
            static st_register_t *p_reg = st_registers;
            
            if (netpie_ready())
            {
                st_register_t *p_prev = st_prev_registers;
                uint32_t i = ((uint32_t)p_reg - (uint32_t)st_registers) / sizeof(st_register_t);
                p_prev += i;
                                
                if ((*p_prev->p_value != *p_reg->p_value) || first_time)  // The value has been changed.
                {   
                    *p_prev->p_value = *p_reg->p_value;  // Update
                    
                    const char *sub_topic = p_reg->sub_topic;
                    char message[20];

                    snprintf(message, sizeof(message), "%f", *p_reg->p_value);
                    netpie_publish_register(sub_topic, message);

                    TRACE_LOG("[PubSub] update reg#%d %s > '%s'\n\r", i, sub_topic, message);  // DEBUG: iPAS
                
                    vTaskDelay(REGISTER_PUBLISH_INTERVAL_MS / portTICK_PERIOD_MS);
                }
                else
                {
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
                
                p_reg++;  // Next register
                if (p_reg->sub_topic == NULL)
                {
                    first_time = false;
                    p_reg = st_registers;
                    
                    
                    
                    // -------------------------------
                    // --- For testing only ----------
                    // --- Randomly changing value ---
                    uint16_t i = rand() % (register_count-1);
                    float val = rand() % 100;
                    *st_registers[i].p_value = val;  // Minus one for skipping the null terminator
                    TRACE_LOG("[PubSub] randomly change on '%s' with '%.2f'\n\r", st_registers[i].sub_topic, val);  // DEBUG: iPAS

                    
                    
                }
            }
            else
            {
                first_time = true;
                p_reg = st_registers;
                app_pubsubData.state = APP_PUBSUB_STATE_INIT;
            }
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
