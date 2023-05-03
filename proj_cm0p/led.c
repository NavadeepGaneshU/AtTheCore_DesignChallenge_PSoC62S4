/******************************************************************************
* File Name: led.c
*
* Description: This file contains source code that controls LED.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header files includes
*******************************************************************************/
#include "cybsp.h"
#include "led.h"

/*******************************************************************************
* Global constants
*******************************************************************************/
extern led_state_t led1_state_cur = LED_ON;
extern led_state_t led2_state_cur = LED_ON;

/*******************************************************************************
* Function Name: update_led_state
********************************************************************************
* Summary:
*  This function updates the LED state, based on the touch input.
*
* Parameter:
*  led_data: the pointer to the LED data structure
*
* Return
*  none
*
*******************************************************************************/
void update_led1_state(led_data_t *led1_data)
{
    if ((led1_state_cur == LED_OFF) && (led1_data->state == LED_ON))
    {
        /* Enable PWM */
        Cy_TCPWM_PWM_Enable(PWM1_HW, PWM1_NUM);

        #if(CY_IP_MXTCPWM_VERSION > 1)
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart_Single(PWM1_HW, PWM1_NUM);
        }
        #else
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart(PWM1_HW, PWM1_MASK);
        }
        #endif

        led1_state_cur = LED_ON;
        led1_data->brightness = LED_MAX_BRIGHTNESS;
    }
    else if ((led1_state_cur == LED_ON) && (led1_data->state == LED_OFF))
    {
        /* Disable PWM to turn off the LED */
        Cy_TCPWM_PWM_Disable(PWM1_HW, PWM1_NUM);
        led1_state_cur = LED_OFF;
        led1_data->brightness = 0;
    }
    else
    {
    }

    if ((LED_ON == led1_state_cur) || ((LED_OFF == led1_state_cur) && (led1_data->brightness > 0)))
    {
        /* Enable PWM */
        Cy_TCPWM_PWM_Enable(PWM1_HW, PWM1_NUM);

        #if(CY_IP_MXTCPWM_VERSION > 1)
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart_Single(PWM1_HW, PWM1_NUM);

        }
        #else
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart(PWM1_HW, PWM1_MASK);

        }
        #endif

        uint32_t brightness = (led1_data->brightness < LED_MIN_BRIGHTNESS) ? LED_MIN_BRIGHTNESS : led1_data->brightness;

        /* Drive the LED with brightness */
        Cy_TCPWM_PWM_SetCompare0(PWM1_HW, PWM1_NUM, brightness);

        led1_state_cur = LED_ON;
    }
}

void update_led2_state(led_data_t *led2_data)
{
    if ((led2_state_cur == LED_OFF) && (led2_data->state == LED_ON))
    {
        /* Enable PWM */
        Cy_TCPWM_PWM_Enable(PWM2_HW, PWM2_NUM);

        #if(CY_IP_MXTCPWM_VERSION > 1)
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart_Single(PWM2_HW, PWM2_NUM);
        }
        #else
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart(PWM2_HW, PWM2_MASK);
        }
        #endif

        led2_state_cur = LED_ON;
        led2_data->brightness = LED_MAX_BRIGHTNESS;
    }
    else if ((led2_state_cur == LED_ON) && (led2_data->state == LED_OFF))
    {
        /* Disable PWM to turn off the LED */
        Cy_TCPWM_PWM_Disable(PWM2_HW, PWM2_NUM);
        led2_state_cur = LED_OFF;
        led2_data->brightness = 0;
    }
    else
    {
    }

    if ((LED_ON == led2_state_cur) || ((LED_OFF == led2_state_cur) && (led2_data->brightness > 0)))
    {
        /* Enable PWM */
        Cy_TCPWM_PWM_Enable(PWM2_HW, PWM2_NUM);

        #if(CY_IP_MXTCPWM_VERSION > 1)
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart_Single(PWM2_HW, PWM2_NUM);

        }
        #else
        {
            /* Start PWM */
            Cy_TCPWM_TriggerStart(PWM2_HW, PWM2_MASK);

        }
        #endif

        uint32_t brightness = (led2_data->brightness < LED_MIN_BRIGHTNESS) ? LED_MIN_BRIGHTNESS : led2_data->brightness;

        /* Drive the LED with brightness */
        Cy_TCPWM_PWM_SetCompare0(PWM2_HW, PWM2_NUM, brightness);

        led2_state_cur = LED_ON;
    }
}

/*******************************************************************************
* Function Name: initialize_led
********************************************************************************
* Summary:
*  Initializes a PWM resource for driving an LED.
*
* Parameters:
*  none
*
* Return
*  cy_rslt_t rslt - status of operation
*
*******************************************************************************/
void initialize_led(void)
{
    /* Configure the TCPWM for PWM operation */
    Cy_TCPWM_PWM_Init(PWM1_HW, PWM1_NUM, &PWM1_config);
    Cy_TCPWM_PWM_Init(PWM2_HW, PWM2_NUM, &PWM2_config);

    /* Enable PWM */
    Cy_TCPWM_PWM_Enable(PWM1_HW, PWM1_NUM);
    Cy_TCPWM_PWM_Enable(PWM2_HW, PWM2_NUM);

    /* Drive the LED with brightness */
    Cy_TCPWM_PWM_SetCompare0(PWM1_HW, PWM1_NUM, LED_MAX_BRIGHTNESS);
    Cy_TCPWM_PWM_SetCompare0(PWM2_HW, PWM2_NUM, LED_MAX_BRIGHTNESS);

    #if(CY_IP_MXTCPWM_VERSION > 1)
    {
        /* Start PWM */
        Cy_TCPWM_TriggerStart_Single(PWM1_HW, PWM1_NUM);
        Cy_TCPWM_TriggerStart_Single(PWM2_HW, PWM2_NUM);
    }
    #else
    {
        /* Start PWM */
        Cy_TCPWM_TriggerStart(PWM1_HW, PWM1_MASK);
        Cy_TCPWM_TriggerStart(PWM2_HW, PWM2_MASK);

    }
    #endif
}

/* [] END OF FILE */
