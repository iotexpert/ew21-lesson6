
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

#include "motor_task.h"
#include "cloud_task.h"

#include "tle9879_system.h"



static QueueHandle_t motor_value_q;

/*******************************************************************************
* Global constants
*******************************************************************************/
#define		RPM_CHANGE_INTERVAL		(100)
#define		RPM_CHANGE_RATE			(10)
#define		RPM_PERCENT_MIN			(10)

#define  	RPM_MAX 				5500.0
#define  	RPM_MIN 				1000.0


tle9879_sys_t tle9879_sys;

static int currentPercentage=0;
static int desiredPercentage=0;

void motor_task(void* param)
{
    (void)param;


    uint8_t numberOfBoards = 1;
	BaseType_t rtos_api_result;

	motor_value_q = xQueueCreate(1,sizeof(int));

    /* Initialize and configure the motor driver */
    tle9879sys_init(&tle9879_sys,
    					CYBSP_D11,
						CYBSP_D12,
						CYBSP_D13,
						NULL,
						CYBSP_D4,
						CYBSP_D5,
						CYBSP_D6,
						CYBSP_D7,
						&numberOfBoards);
    tle9879sys_setMode(&tle9879_sys, FOC, 1, false);

    /* Repeatedly running part of the task */
    for(;;)
    {
    	/*Look for a new motor speed percentage in the queue but don't wait if there isn't one */
    	rtos_api_result = xQueueReceive(motor_value_q, &desiredPercentage, RPM_CHANGE_INTERVAL);

       	/* Value has been received from the queue (i.e. not a timeout) */
		if(rtos_api_result == pdTRUE)
		{
			if(desiredPercentage < RPM_PERCENT_MIN) /* Any value less than 10% will result in stopping the motor */
				desiredPercentage = 0;
		
			if(desiredPercentage>100)
				desiredPercentage = 100;

			printf("Desired Motor Percentage: %d\n", desiredPercentage);
		}

		if(currentPercentage != desiredPercentage)
		{
			if (currentPercentage < desiredPercentage)
				currentPercentage = currentPercentage + RPM_CHANGE_RATE;
			if (currentPercentage > desiredPercentage)
				currentPercentage = currentPercentage + RPM_CHANGE_RATE;
			
			if(abs(currentPercentage-desiredPercentage) < RPM_CHANGE_RATE)
				currentPercentage = desiredPercentage;


			float speed = ((float)(currentPercentage-RPM_PERCENT_MIN))/100.0 * (RPM_MAX - RPM_MIN) + RPM_MIN;

			printf("Current %d%% Desired=%d%% Speed=%f\n",currentPercentage,desiredPercentage,speed);



		}

		#if 0

		/* Adjust motor speed depending on difference between actual and desired.
		 * The rate of change is limited to keep the motor operation smooth */
		if (motorSpeedDesired > motorSpeed + RPM_CHANGE_MAX) /* Speed needs to go up */
		{
			if (motorSpeed < MIN_RPM) /* Jump up to the minimum speed right away */
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
						tle9879sys_setMotorMode(&tle9879_sys, STOP_MOTOR, 1);
						printf("Motor Stopped\n");
						motorRunning = false;
					}
			}
			else	/* Set new speed and start motor if it isn't already running */
			{
				tle9879sys_setMotorSpeed(&tle9879_sys, motorSpeed, 1);
				if(motorRunning == false)
				{
					tle9879sys_setMotorMode(&tle9879_sys, START_MOTOR, 1);
					printf("Motor Started\n");
					motorRunning = true;
				}
			}
		}
#endif
    }
}

void motor_update(int speed)
{
	xQueueSend(motor_value_q,&speed,0);

}