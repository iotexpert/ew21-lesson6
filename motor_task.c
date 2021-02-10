/*******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that handles the 3D magnetic joystick.
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
#include "motor_task.h"

#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "capsense_task.h"
#include "global.h"

#include "ew_tle9879_system.h"

/*******************************************************************************
* Global constants
*******************************************************************************/
#define		RPM_CHANGE_INTERVAL		(100)
#define		RPM_CHANGE_MAX			(100)

#define 	MAX_PCT 				100.0
#define 	MIN_PCT 				10.0
#define  	MAX_RPM 				3000.0
#define  	MIN_RPM 				1000.0

#define 	SLOPE 					((MAX_RPM-MIN_RPM)/(MAX_PCT-MIN_PCT))
#define 	INTERCEPT 				(MAX_RPM - (SLOPE * MAX_PCT))

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
ew_tle9879_sys_t tle9879_sys;

/*******************************************************************************
* Function Name: task_joystick
********************************************************************************
* Summary:
*  Task that initializes the CapSense block and processes the touch input.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_motor(void* param)
{
    bool motorRunning = false;
    bool motorSpeedChange = false;
    uint8_t motorPercent;
    float motorSpeed = 0;
    float motorSpeedDesired = 0;
    uint8_t numberOfBoards = 1;
	BaseType_t rtos_api_result;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize and configure the motor driver */
    ew_tle9879sys_init(&tle9879_sys,
    					CYBSP_D11,
						CYBSP_D12,
						CYBSP_D13,
						NULL,
						CYBSP_D4,
						CYBSP_D5,
						CYBSP_D6,
						CYBSP_D7,
						&numberOfBoards);
    ew_tle9879sys_setMode(&tle9879_sys, FOC, 1, false);

    /* Repeatedly running part of the task */
    for(;;)
    {
    	/*Look for a new motor speed percentage in the queue but don't wait if there isn't one */
    	rtos_api_result = xQueueReceive(motor_value_q, &motorPercent, 0);

       	/* Value has been received from the queue (i.e. not a timeout) */
		if(rtos_api_result == pdTRUE)
		{
			/* Scale the percentage to a desired motor speed */
			if(motorPercent < 10)
			{
				motorSpeedDesired = 0;
			}
			else
			{
				motorSpeedDesired = SLOPE * (float) motorPercent + INTERCEPT;
			}

			printf("New Desired Motor Speed: %0.0f\n", motorSpeedDesired);
		}

		/* Adjust motor speed depending on difference between actual and desired.
		 * The rate of change is limited to keep the motor operation smooth */
		if (motorSpeedDesired > motorSpeed + RPM_CHANGE_MAX) /* Speed needs to go up */
		{
			if (motorSpeed < MIN_RPM) /* Jump to minimum speed right away */
			{
				motorSpeed = MIN_RPM;
			}
			else /* Increase by the maximum allowed amount */
			{
				motorSpeed += RPM_CHANGE_MAX;
			}
			motorSpeedChange = true;
		}
		else if (motorSpeedDesired + RPM_CHANGE_MAX < motorSpeed) /* Speed needs to go down */
		{
			if (motorSpeed < MIN_RPM) /* Drop to 0 once below min speed */
			{
				motorSpeed = 0;
			}
			else
			{
				motorSpeed -= RPM_CHANGE_MAX; /* Decrease by the maximum allowed amount */
			}
			motorSpeedChange = true;
		}
		else if (motorSpeedDesired != motorSpeed) /* Within RPM_CHANGE_MAX so jump to final value */
		{
			motorSpeed = motorSpeedDesired;
			motorSpeedChange = true;
		}

		if(motorSpeedChange == true)	/* Not at desired speed yet */
		{
			motorSpeedChange = false;

			printf("New Motor Speed: %0.0f\n", motorSpeed);

			if(motorSpeed < MIN_RPM)	/* Too slow for motor to run */
			{
				if(motorRunning == true)
					{
						ew_tle9879sys_setMotorMode(&tle9879_sys, STOP_MOTOR, 1);
						printf("Motor Stopped\n");
						motorRunning = false;
					}
			}
			else	/* Set new speed and start motor if it isn't already running */
			{
				ew_tle9879sys_setMotorSpeed(&tle9879_sys, motorSpeed, 1);
				if(motorRunning == false)
				{
					ew_tle9879sys_setMotorMode(&tle9879_sys, START_MOTOR, 1);
					printf("Motor Started\n");
					motorRunning = true;
				}
			}
		}

		vTaskDelay(RPM_CHANGE_INTERVAL); /* Max rate to change motor speed */

    }
}

/* END OF FILE [] */

