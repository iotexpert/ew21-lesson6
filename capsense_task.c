/*******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that handles touch sensing.
*
********************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
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
*****************************************​**************************************/


/******************************************************************************
* Header files includes
******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include <stdio.h>

#include "global.h"
#include "capsense_task.h"

/*******************************************************************************
* Global constants
*******************************************************************************/
#define CAPSENSE_INTERRUPT_PRIORITY     (7)

#define CAPSENSE_SCAN_INTERVAL_MS       (20)   /* in milliseconds*/

#define NUM_ELEMENTS                    (1)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t capsense_init(void);
static void process_touch(void);
static void capsense_isr(void);
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr);
static void capsense_timer_callback(TimerHandle_t xTimer);

/******************************************************************************
* Global variables
******************************************************************************/
QueueHandle_t capsense_command_q;
TimerHandle_t scan_timer_handle;
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;

typedef enum
{
    CAPSENSE_SCAN,
    CAPSENSE_PROCESS
} capsense_command_t;

/* SysPm callback params */
cy_stc_syspm_callback_params_t callback_params =
{
    .base       = CYBSP_CSD_HW,
    .context    = &cy_capsense_context
};

cy_stc_syspm_callback_t capsense_deep_sleep_cb =
{
    Cy_CapSense_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    (CY_SYSPM_SKIP_CHECK_FAIL | CY_SYSPM_SKIP_BEFORE_TRANSITION | CY_SYSPM_SKIP_AFTER_TRANSITION),
    &callback_params,
    NULL,
    NULL
};


/*******************************************************************************
* Function Name: task_capsense
********************************************************************************
* Summary:
*  Task that initializes the CapSense block and processes the touch input.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_capsense(void* param)
{
    /* See the Cypress CapSense Middleware Library link in the Quick Panel
     * Documentation for details on the CapSense API */

	BaseType_t rtos_api_result;
    cy_status status;
    capsense_command_t capsense_cmd;

    /* Remove warning for unused parameter */
    (void)param;

    /* Create queue to handle commands to alternate between scanning and processing of results. */
    capsense_command_q  = xQueueCreate( NUM_ELEMENTS, sizeof(capsense_command_t));

    /* Initialize timer for periodic CapSense scan */
    scan_timer_handle = xTimerCreate ("Scan Timer", CAPSENSE_SCAN_INTERVAL_MS,
                                      pdTRUE, NULL, capsense_timer_callback);

    /* Initialize CapSense block */
    status = capsense_init();
    if(CY_RET_SUCCESS != status)
    {
    	printf("CapSense Initialization Failed!\n");
    	CY_ASSERT(0u);
    }
    else
    {
    	printf("CapSense Initialized\n");
    }

    /* Start the timer */
    xTimerStart(scan_timer_handle, 0u);

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a CapSense command has been received over queue */
        rtos_api_result = xQueueReceive(capsense_command_q, &capsense_cmd, portMAX_DELAY);

        /* Command has been received from capsense_cmd */
        if(rtos_api_result == pdTRUE)
        {
            /* Check if CapSense is busy with a previous scan */
            if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
            {
                switch(capsense_cmd)
                {
                    case CAPSENSE_SCAN:
                    { 
                        /* Start scan */
                        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
                        break;
                    }
                    case CAPSENSE_PROCESS:
                    {
                        /* Process all widgets */
                        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
                        process_touch();
                        break;
                    }
                    /* Invalid command */
                    default:
                    {
                        break;
                    }
                }
            }
        }
        /* Task has timed out and received no data during an interval of
         * portMAXDELAY ticks.
         */
        else
        {
            /* Handle timeout here */
        }
    }
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  This function processes the touch input and sends command to LED task.
*
*******************************************************************************/
static void process_touch(void)
{
    /* Variables used to store touch information */
    uint32_t button0_status = 0;
    uint32_t button1_status = 0;
    uint16_t slider_pos = 0;
    uint8_t slider_touched = 0;
    cy_stc_capsense_touch_t *slider_touch;

    /* Variables used to store previous touch information */
    static uint32_t button0_status_prev = 0;
    static uint32_t button1_status_prev = 0;
    static uint16_t slider_pos_perv = 0;

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive( CY_CAPSENSE_BUTTON0_WDGT_ID, CY_CAPSENSE_BUTTON0_SNS0_ID, &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive( CY_CAPSENSE_BUTTON1_WDGT_ID, CY_CAPSENSE_BUTTON1_SNS0_ID, &cy_capsense_context);

    /* Get slider status */
    slider_touch = Cy_CapSense_GetTouchInfo( CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    /* Slider range is scaled from 0 - 300 to 0 - 100 */
    /* This could instead have been changed using the CapSense configurator */
    slider_pos = (slider_touch->ptrPosition->x / 3);
    slider_touched = slider_touch->numPosition;

    /* Detect new touch on Button0 */
    if((0u != button0_status) && (0u == button0_status_prev))
    {
        printf("Button 0 pressed\n");
    }

    /* Detect new touch on Button1 */
    if((0u != button1_status) && (0u == button1_status_prev))
    {
        printf("Button 1 pressed\n");
    }

    /* Detect new touch on slider */
    if((0u != slider_touched) && (slider_pos_perv != slider_pos ))
    {
        printf("Slider position %d\n",slider_pos);
        xQueueOverwrite(motor_value_q, (uint8_t*) &slider_pos);

    }

    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_perv = slider_pos;
}


/*******************************************************************************
* Function Name: capsense_init
********************************************************************************
* Summary:
*  This function initializes the CSD HW block, and configures the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t capsense_init(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTERRUPT_PRIORITY,
    };

    /*Initialize CapSense Data structures */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&capSense_intr_config, &capsense_isr);
    NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
    NVIC_EnableIRQ(capSense_intr_config.intrSrc);

    /* Initialize the CapSense deep sleep callback functions. */
    /* See the "MTB CAT1 Peripheral driver library documentation > PDL API Reference > SysPM"
     * link in the Quick Panel Documentation for information on setting up the SysPm callbacks */
    Cy_SysPm_RegisterCallback(&capsense_deep_sleep_cb);
    /* Register end of scan callback */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
                                              capsense_end_of_scan_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }
    
    return status;
}


/*******************************************************************************
* Function Name: capsense_end_of_scan_callback
********************************************************************************
* Summary:
*  CapSense end of scan callback function. This function sends a command to
*  CapSense task to process scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t * active_scan_sns_ptr (unused)
*
*******************************************************************************/
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr)
{
    BaseType_t xYieldRequired;

    (void)active_scan_sns_ptr;

    /* Send command to process CapSense data */
    capsense_command_t commmand = CAPSENSE_PROCESS;
    xYieldRequired = xQueueSendToBackFromISR(capsense_command_q, &commmand, 0u);
    portYIELD_FROM_ISR(xYieldRequired);
}


/*******************************************************************************
* Function Name: capsense_timer_callback
********************************************************************************
* Summary:
*  CapSense timer callback. This function sends a command to start CapSense
*  scan.
*
* Parameters:
*  TimerHandle_t xTimer (unused)
*
*******************************************************************************/
static void capsense_timer_callback(TimerHandle_t xTimer)
{
	Cy_CapSense_Wakeup(&cy_capsense_context);
    capsense_command_t command = CAPSENSE_SCAN;
    BaseType_t xYieldRequired;

    (void)xTimer;

    /* Send command to start CapSense scan */
    xYieldRequired = xQueueSendToBackFromISR(capsense_command_q, &command, 0u);
    portYIELD_FROM_ISR(xYieldRequired);
}


/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CSD block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/* END OF FILE [] */
