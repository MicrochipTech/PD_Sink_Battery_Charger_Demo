/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/
/*******************************************************************************
Copyright ©  [2019] Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and
any derivatives exclusively with Microchip products. It is your responsibility
to comply with third party license terms applicable to your use of third party
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS,
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN
NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN
ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST
EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU
HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "psf_stdinc.h"                 // PSF include file
#include <stdio.h>
#include <xc.h>
#include "SEPIC_CTRL.h"
#include "config/default/peripheral/eic/plib_eic.h"
#include "time_delay.h"
#include <inttypes.h>
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************



int main(void) {

    char debug_string[32];
    uint8_t u8ReadData[4];

    /* Initialize all modules */
    SYS_Initialize(NULL);

    /*PSF init called*/
    (void) MchpPSF_Init();
    MCHP_PSF_HOOK_PRINT_TRACE("PSF Init Complete\r\n");

    /* Print out the VID and PID for debug */
    UPD_RegisterRead(0, (UINT16) UPD_VID, u8ReadData, BYTE_LEN_4);
    sprintf(debug_string, "VID: %04X PID: %04X\r\n", ((uint16_t) u8ReadData[1] << 8) + u8ReadData[0],
            ((uint16_t) u8ReadData[3] << 8) + u8ReadData[2]);
    MCHP_PSF_HOOK_PRINT_TRACE(debug_string);

    while (true) {

        /* Maintain state machines of all polled MPLAB Harmony modules. */
        /*PSF stack Run*/
        SYS_Tasks();

        MchpPSF_RUN();
        if (GPIO_PB16_Get() == 0) {
            MchpPSF_UPDIrqHandler(PORT0);
        }

    }
}

/*******************************************************************************
 End of File
 */