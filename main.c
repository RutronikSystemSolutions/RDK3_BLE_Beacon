/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3 Beacon Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-12-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
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
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_syslib.h"
#include "cy_sysint.h"
#include "cycfg.h"
#include "cycfg_ble.h"

/*******************************************************************************
* Global variables
*******************************************************************************/
/* BLESS interrupt configuration.
 * It is used when BLE middleware operates in BLE Single CM4 Core mode. */
const cy_stc_sysint_t blessIsrCfg =
{
    /* The BLESS interrupt */
    .intrSrc      = bless_interrupt_IRQn,

    /* The interrupt priority number */
    .intrPriority = 1u
};
/*******************************************************************************
* Private Function Prototypes
*******************************************************************************/
void AppMainLoop(void);
void AppCallBack(uint32_t event, void *eventParam);
void IasEventHandler(uint32_t event, void *eventParam);
void LowPowerImplementation(void);
void BlessInterrupt(void);

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LEDs */
    result = cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);

    /*Charger control*/
    result = cyhal_gpio_init(CHR_DIS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    cyhal_gpio_write((cyhal_gpio_t)CHR_DIS, true); /*Charger OFF*/

    /* Configure USER_BTN */
    result = cyhal_gpio_init(USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}


    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("RDK3 Beacon\r\n\n");

    /* Initialize the BLESS interrupt */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;
    Cy_SysInt_Init(cy_ble_config.hw->blessIsrConfig, BlessInterrupt);

    /* Register the generic event handler */
    Cy_BLE_RegisterEventCallback(AppCallBack);

    /* Initialize the BLE */
    Cy_BLE_Init(&cy_ble_config);

    /* Enable BLE Low-power Mode (LPM) */
    Cy_BLE_EnableLowPowerMode();

    /* Enable BLE */
    Cy_BLE_Enable();

    /* Register the IAS CallBack */
    Cy_BLE_IAS_RegisterAttrCallback(IasEventHandler);

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        /* Main loop implementation */
        AppMainLoop();
    }
}

void AppMainLoop(void)
{
    /* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
    Cy_BLE_ProcessEvents();

    /* Enter deep-sleep to achieve low power in the device */
    LowPowerImplementation();
}

/*******************************************************************************
* Function Name: AppCallBack
********************************************************************************
* Summary: This is an event callback function to receive events from the PSoC 6 BLE Middleware.
*
* Parameters:
*   event      - The event code.
*   eventParam - The event parameters.
*
*******************************************************************************/
void AppCallBack(uint32_t event, void *eventParam)
{
	cy_en_ble_api_result_t ble_rslt;
    switch(event)
    {

    /* This event is received when the BLE stack is initialized and turned ON by invoking the Cy_BLE_StackInit() function */
    case CY_BLE_EVT_STACK_ON:
        /* Enter into discoverable mode so that remote can find it */
    	ble_rslt = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX);
    	if(ble_rslt == CY_BLE_SUCCESS)
    	{
    		printf("Advertisement has started.\r\n");
    		cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_ON);
    	}
        break;

    /* This event indicates the peripheral device has started/stopped advertising */
    case  CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        if (Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_STOPPED)
        {
        	printf("Advertisement has stopped, restarting...\r\n");
        	Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX);
        }
        break;
    }
}

void IasEventHandler(uint32_t event, void *eventParam)
{
    (void) eventParam;
}

/*******************************************************************************
* Function Name: LowPowerImplementation
********************************************************************************
* Summary: Implements low power in the project.
*
* Theory:
*  The function tries to enter CPU deep sleep as much as possible - whenever the
*  BLE is idle.
*
*******************************************************************************/
void LowPowerImplementation(void)
{
    /* Entering Deep Sleep */
    Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
}

/*******************************************************************************
* Function Name: BlessInterrupt
********************************************************************************
* BLESS ISR
* It is used used when BLE middleware operates in BLE single CM4
*
*******************************************************************************/
/* BLESS ISR */
void BlessInterrupt(void)
{
    /* Call interrupt processing */
    Cy_BLE_BlessIsrHandler();
}

/* [] END OF FILE */
