/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM0+ in the the Dual CPU Motor Control Application Project
*
* Related Document: See README.md
*
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "led.h"
#include "ipc_def.h"
#include <stdlib.h>
#include "cyhal.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY  (3u)
#define EZI2C_INTR_PRIORITY     (2u)
#define HIGH					(1u)
#define LOW						(0u)

#define MY_IPC_CHAN_INDEX       (8UL) /* Example of IPC channel index */
#define MY_IPC_INTR_INDEX       (8UL) /* Example of IPC interrupt channel index */
#define MY_IPC_INTR_MASK        (1UL << MY_IPC_INTR_INDEX) /* IPC release interrupt mask */

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t initialize_capsense(void);
static void process_touch(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
static void capsense_callback(cy_stc_active_scan_sns_t *);
static void ezi2c_isr(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile bool capsense_scan_complete = false;
cy_stc_scb_ezi2c_context_t ezi2c_context;

uint32_t myMsg; 	/* reception message variable*/

//int *pLED1_State, *pLED2_State, *pLED1_PWM, *pLED2_PWM = 0;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize tuner communication
*  - scan touch input continuously and update the LED accordingly.
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Lock the sempahore to wait for CM4 to be init */
    Cy_IPC_Sema_Set(SEMA_NUM, false);

    /* Enable CM4.*/
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

	#if defined (CY_DEVICE_SECURE)
	   cyhal_wdt_t wdt_obj;

	   /* Clear watchdog timer so that it doesn't trigger a reset */
	   result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
	   CY_ASSERT(CY_RSLT_SUCCESS == result);
	   cyhal_wdt_free(&wdt_obj);
	#endif

	do
	{
	   __WFE();
	}
	while (Cy_IPC_Sema_Status(SEMA_NUM) == CY_IPC_SEMA_STATUS_LOCKED);

   /* Update clock settings */
   SystemCoreClockUpdate();

    initialize_led();
    initialize_capsense_tuner();

    /* Initialize CapSense */
    result = initialize_capsense();

    if (CYRET_SUCCESS != result)
    {
        /* Halt the CPU if CapSense initialization failed */
        CY_ASSERT(0);
    }

    /* Initiate first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;)
    {
        if (capsense_scan_complete)
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Initiate next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

            capsense_scan_complete = false;
         }

        /* Check if the button is pressed */
        if (Cy_GPIO_Read(CYBSP_SW2_PORT, CYBSP_SW2_PIN) == 0)
        {
        #if ENABLE_SEMA
            if (Cy_IPC_Sema_Set(SEMA_NUM, false) == CY_IPC_SEMA_SUCCESS)
        #endif
            {
                /*LED/MOTOR control state receive*/
                IPC_STRUCT_Type * myIpc = Cy_IPC_Drv_GetIpcBaseAddress(MY_IPC_CHAN_INDEX); /* Get IPC base register address */
                if ((CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_ReadMsgWord(myIpc, &myMsg)) && (myMsg ==0x01))
                {
                    /* Now myMsg contains the received message word */
                    /* The IPC data is received and processed.
                     * Free up the channel for the next transaction.
                     */
                    	/*Allow MOTOR1 ON*/
                	    cyhal_gpio_write(CYBSP_A8, HIGH);
                	    cyhal_gpio_write(CYBSP_A9, HIGH);

                    Cy_IPC_Drv_ReleaseNotify(myIpc, MY_IPC_INTR_MASK);
                }
                else
                {
                    /* Insert error handling */
                    Cy_SCB_UART_PutString(CYBSP_UART_HW, "Failed to receive MOTOR signal in IPC pipe\r\n");
                    /*Inhibit motor action*/
            	    cyhal_gpio_write(CYBSP_A8, LOW);
            	    cyhal_gpio_write(CYBSP_A9, LOW);
                    cyhal_system_delay_ms(500);
                }

                /* Print a message to the console */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "Hello from CM0+\r\n");
                //printf("Message sent from CM0+\r\n");
                cyhal_system_delay_ms(500);
            #if ENABLE_SEMA
                while (CY_IPC_SEMA_SUCCESS != Cy_IPC_Sema_Clear(SEMA_NUM, false));
            #endif
            }
        }
    }
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
 * Parameters:
 *  none
 *
 * Return
 *  uint32_t status - status of operation
 *
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = NvicMux3_IRQn,
        .cm0pSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&capSense_intr_config, capsense_isr);
    NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
    NVIC_EnableIRQ(capSense_intr_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
            capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
* Parameters:
*  none
*
* Return
*  none
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from EZI2C block.
*
*******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
* Return
*  none
*
*******************************************************************************/
static void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
* Parameters:
*  none
*
* Return
*  none
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t *slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status;
    bool led1_update_req = false;
    bool led2_update_req = false;

    static uint32_t button0_status_prev;
    static uint32_t button1_status_prev;
    static uint16_t slider_pos_prev;
    static led_data_t led1_data = {LED_ON, LED_MAX_BRIGHTNESS};
    static led_data_t led2_data = {LED_ON, LED_MAX_BRIGHTNESS};

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
                                CY_CAPSENSE_BUTTON0_WDGT_ID,
                                CY_CAPSENSE_BUTTON0_SNS0_ID,
                                &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
                                CY_CAPSENSE_BUTTON1_WDGT_ID,
                                CY_CAPSENSE_BUTTON0_SNS0_ID,
                                &cy_capsense_context);

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect new touch on Button0 */
    if((0u != button0_status) &&
       (0u == button0_status_prev))
    {
        /* Turn USER LED1 ON/OFF */
    	if(led1_data.state){
        led1_data.state = LED_OFF;
        led1_update_req = true;
    	}
    	else{
            led1_data.state = LED_ON;
            led1_update_req = true;
    	}
    }

    /* Detect new touch on Button1 */
    if((0u != button1_status) &&
       (0u == button1_status_prev))
    {
        /* Turn the USER LED2 ON/OFF */
    	if(led2_data.state){
        led2_data.state = LED_OFF;
        led2_update_req = true;
    	}
    	else{
            led2_data.state = LED_ON;
            led2_update_req = true;
    	}
    }

    /* Detect the new touch on slider */
    if ((0 != slider_touch_status) &&
        (slider_pos != slider_pos_prev))
    {
    	if(led1_data.state)
    	{
        led1_data.brightness = (slider_pos * 100)
                / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;

        led1_update_req = true;
    	}

    	if(led2_data.state)
    	{
        led2_data.brightness = (slider_pos * 100)
                / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;

        led2_update_req = true;
    	}
    }

    /* Update the LED1 state if requested */
    if (led1_update_req)
    {
        update_led1_state(&led1_data);
    }

    /* Update the LED2 state if requested */
    if (led2_update_req)
    {
        update_led2_state(&led2_data);
    }

    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_prev = slider_pos;

}

/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
* Parameters:
*  none
*
* Return
*  none
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    /* EZI2C interrupt configuration structure */
    static const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = NvicMux2_IRQn,
        .cm0pSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize EZI2C */
    Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    /* Initialize and enable EZI2C interrupts */
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set up communication data buffer to CapSense data structure to be exposed
     * to I2C master at primary slave address request.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8 *)&cy_capsense_tuner,
        sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
        &ezi2c_context);

    /* Enable EZI2C block */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

}
/* [] END OF FILE */
