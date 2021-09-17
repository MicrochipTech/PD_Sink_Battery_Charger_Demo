/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "SEPIC_CTRL.h"
#include "PSF_Config.h"
#include <stdio.h>
#include <string.h>
#include "config/default/peripheral/eic/plib_eic.h"
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

APP_DATA appData;

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


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

uint8_t screenNum = 0;

void APP_Initialize(void) {
    ADC_Enable();
    SEPIC_init();
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
        //Ext ints for OLED1 buttons
    EIC_CallbackRegister(EIC_PIN_2, (EIC_CALLBACK) BTN1_EventHandler, (uintptr_t) NULL);
    EIC_CallbackRegister(EIC_PIN_4, (EIC_CALLBACK) BTN2_EventHandler, (uintptr_t) NULL);
    EIC_CallbackRegister(5, (EIC_CALLBACK) BTN3_EventHandler, (uintptr_t) NULL);
    EIC_CallbackRegister(0, (EIC_CALLBACK) PSF_EventHandler, (uintptr_t) NULL);
    
    printf("app initialized\r\n");
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}



/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

#define LINES 8
#define LINE_Y(_line)   ((_line % 8) * 8)

uint8_t start_line_address = 0;
uint8_t scroll = 0;
uint32_t line = 0;
SYS_TIME_HANDLE test_timer;
uint8_t display_index = 0;

//battery condition default to 0 (FAULT)
uint8_t condition = 0;

void APP_Tasks(void) {

    //printf("entered APP_Tasks()\r\n");
    char display_char = '-';
    char display_string[32];
    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;

            gfx_mono_init();

            gfx_mono_draw_string("PD Demo Board", 0, LINE_Y(0), &sysfont);

            set_timer_ms(&test_timer, 500);
            if (appInitialized) {
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            if (timer_ms_done(&test_timer)) {

                if (screenNum == 0) {
                    //draw screen 1
                    gfx_mono_draw_string("Charger Status  ", 0, LINE_Y(0), &sysfont);
                    condition = getStatus();
                    switch (condition) {
                        case 0:
                            gfx_mono_draw_string("Status: Fault       ", 0, LINE_Y(1), &sysfont);
                            break;
                        case 1:
                            gfx_mono_draw_string("Status: Pre-condition", 0, LINE_Y(1), &sysfont);
                            break;
                        case 2:
                            gfx_mono_draw_string("Status: CC Charge    ", 0, LINE_Y(1), &sysfont);
                            break;
                        case 3:
                            gfx_mono_draw_string("Status: CV Charge    ", 0, LINE_Y(1), &sysfont);
                            break;
                        case 4:
                            gfx_mono_draw_string("Status: Fully Chrged ", 0, LINE_Y(1), &sysfont);
                            break;
                        case 5:
                            gfx_mono_draw_string("Status: Trickle Chrg ", 0, LINE_Y(1), &sysfont);
                            break;
                        default:
                            gfx_mono_draw_string("Not reporting       ", 0, LINE_Y(1), &sysfont);
                            break;
                    }


                    if (condition != 0) {
                        //display the estimated SOC
                        char str[10];
                        sprintf(str, "SOC: %d%%          ", getSOC());
                        gfx_mono_draw_string(str, 0, LINE_Y(2), &sysfont);
                    } else {
                        uint8_t faultcode = getFaultCode();
                        switch (faultcode) {
                            case 0:
                                gfx_mono_draw_string("Fault: GENERIC       ", 0, LINE_Y(2), &sysfont);
                                break;
                            case 1:
                                gfx_mono_draw_string("Fault: NOSOURCE      ", 0, LINE_Y(2), &sysfont);
                                break;
                            case 2:
                                gfx_mono_draw_string("Fault: UVLO          ", 0, LINE_Y(2), &sysfont);
                                break;
                            case 3:
                                gfx_mono_draw_string("Fault: OVERTEMP       ", 0, LINE_Y(2), &sysfont);
                                break;
                            default:
                                gfx_mono_draw_string("Not Reporting      ", 0, LINE_Y(2), &sysfont);
                                break;
                        }
                    }
                } else {
                    //draw screen 2
                    gfx_mono_draw_string("PD Contract         ", 0, LINE_Y(0), &sysfont);
                    char str[17];
                    uint16_t current = gasCfgStatusData.sPerPortData[0].u16NegoCurrentInmA;
                    sprintf(str, "Current: %d mA ", current);
                    gfx_mono_draw_string(str, 0, LINE_Y(1), &sysfont);

                    char str2[17];
                    uint16_t voltage = gasCfgStatusData.sPerPortData[0].u16NegoVoltageInmV;
                    sprintf(str2, "Voltage: %d mV  ", voltage);
                    gfx_mono_draw_string(str2, 0, LINE_Y(2), &sysfont);
                    gfx_mono_draw_string("                ", 0, LINE_Y(3), &sysfont);
                }
                set_timer_ms(&test_timer, 500);

                display_index++;
                if (display_index > 6) {
                    display_index = 0;
                }

                switch (display_index) {
                    case 0:
                        display_char = '/';
                        break;
                    case 1:
                        display_char = '-';
                        break;
                    case 2:
                        display_char = '\\';
                        break;
                    case 3:
                        display_char = '|';
                        break;
                    case 4:
                        display_char = '/';
                        break;
                    case 5:
                        display_char = '-';
                        break;
                    case 6:
                        display_char = '\\';
                        break;
                }

                sprintf(display_string, "%c", display_char);
                gfx_mono_draw_string(display_string, 123, LINE_Y(3), &sysfont);
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

void BTN1_EventHandler(uintptr_t context) {
    //OLED_LED1_Toggle();
}

void BTN2_EventHandler(uintptr_t context) {
    stopSEPIC();
}

void BTN3_EventHandler(uintptr_t context) {
    // OLED_LED1_Toggle();
    if (screenNum)
        screenNum = 0;
    else
        screenNum = 1;
}

void PSF_EventHandler(uintptr_t context) {
    MchpPSF_UPDIrqHandler(PORT0);
}

/*******************************************************************************
 End of File
 */
