/*******************************************************************************
  SAMD20 HAL Drivers porting Header File

  Company:
    Microchip Technology Inc.

  File Name:
    Drivers.h

  Description:
    This file contains provides the HAL drivers required for PSF Porting and integration. 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) [2019-2020] Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _DRIVERS_H    /* Guard against multiple inclusion */
#define _DRIVERS_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <stdbool.h>
#include <stddef.h>

#include <generic_defs.h>
        
#include "PSF_Config.h"

#include "../../firmware/src/config/default/peripheral/tc/plib_tc3.h"
//#include "../../firmware/src/config/default/peripheral/sercom/spim/plib_sercom5_spi.h"
#include "../../firmware/src/config/default/peripheral/sercom/spi_master/plib_sercom5_spi_master.h"
#if (TRUE == CONFIG_HOOK_DEBUG_MSG)
#include "../../firmware/src/config/default/peripheral/sercom/usart/plib_sercom3_usart.h"
#include <string.h>
#endif 
#include "../../firmware/src/config/default/peripheral/port/plib_port.h"
#include "../../firmware/src/config/default/peripheral/eic/plib_eic.h"
#if (TRUE == INCLUDE_PD_SINK)
#include "../../firmware/src/config/default/peripheral/dac/plib_dac.h"
#endif
// *****************************************************************************
// *****************************************************************************
// Section: SAMD20 Configuration
// *****************************************************************************
// *****************************************************************************

/* SAMD20 Hardware configuration:
 * 
 * All the SAMD20 Hardware drivers are generated via Harmony configurator 3
 * 
 * The harmony configuration file is available at path
 * ../../firmware/src/config/default/SAMD20_PSFHarmonyConfiguration.xml
 *
 * Hardware Interface configuration 
 * -------------------------------
 *  - SERCOM0 is configured as SPI Master of frequency 8MHz
 *  - DO/MOSI on PAD[3] (PA11), SCK/CLK on PAD[1] (PA09), DI/MISO on PAD[0] (PA08)
 *  - PORT_PIN_PA10 & PORT_PIN_PA01 are configured as GPIO in Output mode and driven high by default.
 *  - PORT_PIN_PA10 is renamed as SPI_SS_0_PIN  in Harmony GUI and used as SPI Chip Select for Port 0
 *  - PORT_PIN_PA01 is renamed as SPI_SS_1_PIN in Harmony GUI and used as SPI Chip Select for Port 1
 *  - SPI drivers are generated in polling mode as SPI read and write API are called in critical section
 *  - SPI drivers for different SERCOM can be configured by changing SAMD20_SPI_INSTANCE in the file if 
 *      and only all the required SPI drivers are generated using Harmony for that instance.
 *  - SERCOM0_SPI_Initialize() and SERCOM0_SPI_WriteRead() drivers are used for this example
 *
 * 
 * Timer Configuration  
 * -------------------
 * - TC0 timer instance configured for 1ms for timer interrupt frequency
 * - Different timer instance can be used  by changing SAMD20_TIMER_INSTANCE in the file if and only
 *      all the required timer drivers are generated using Harmony for that instance.
 * -TC0_TimerInitialize(), TC0_TimerStart(), and TC0_TimerCallbackRegister() driver APIs are used.
 * 
 * 
 * UPD350 Alert interrupt configuration
 * ------------------------------------
 *  - PA14 and PA15 are configured for UPD350 Alert lines for Port0 and Port1 respectively
 *  - They are configured as active low detection pins with wake up capable.
 *  - The PIO can be changed for this port by changing SAMD20_PORT0_EIC_PIN, SAMD20_PORT1_EIC_PIN d
 *      define in this file with required PIO's EIC enum definition provided pin is configured 
 *      for EIC functionality as active low and wake up capable signal in Harmony GUI.
 * 
 * UPD350 Reset Configuration
 * --------------------------
 *  - PA00 is configured as UPD350_Reset pin. By default, it is configured as input pin with internal
 *     pull up. 
 *  - PORT_PIN_PA00 renamed as UPD350_RESET_PIN in GUI is used.
 *  - If the user like to configure a different pin as UPD350_Reset pin, Rename it as UPD350_RESET_PIN
 *      and input with internal pullup by default.
 */

#define SAMD21_TIMER_INSTANCE   3
#define SAMD21_SPI_INSTANCE     5

#define SAMD21_UART_INSTANCE  3

// *****************************************************************************
// *****************************************************************************
// Section: Interface Functions
// *****************************************************************************
// *****************************************************************************
/****************************************************************************
    Function:
        UINT8 SAMD20_HWTimerInit(void)
    Summary:
        Wrapper function to initialize SAMD20 Hardware Timer  
    Description:
        This API serves as a wrapper between PSF stack defined Timer initialization function 
        MCHP_PSF_HOOK_HW_PDTIMER_INIT and Harmony generated timer drivers. It shall initialize and 
        start the timer.
    Conditions:
        None
    Input:
        None
    Return:
        UINT8 - Returns TRUE - If the Hardware timer initialization is successful.
            Returns FALSE - If the Hardware timer initialization is not successful.
    Remarks:
        None
**************************************************************************************************/
UINT8 SAMD21_HWTimerInit(void);

/****************************************************************************
    Function:
        UINT8 SAMD20_SPIInitialisation(void)
    Summary:
        Wrapper function to initialize SAMD20 Hardware Interface for UPD350 communication  
    Description:
        This API serves as a wrapper between PSF stack defined Hardware interface initialization 
        function MCHP_PSF_HOOK_UPDHW_INTF_INIT and Harmony generated SPI drivers. It shall initialize 
        the Hardware interface for communication. In this case, it is SPI.
    Conditions:
        None
    Input:
        None
    Return:
        UINT8 - Returns TRUE - If the SPI initialization is successful.
                Returns FALSE - If the SPI initialization is not successful.
    Remarks:
        None
**************************************************************************************************/
UINT8 SAMD21_SPIInitialisation(void);

/****************************************************************************
    Function:
        UINT8 SAMD20_SPIReaddriver (UINT8 u8PortNum, UINT8 *pu8WriteBuffer, UINT8 u8Writelength,\
                UINT8 *pu8ReadBuffer, UINT8 u8Readlength)
    Summary:
        Wrapper function for SPI read drivers.  
    Description:
        This API serves as a wrapper between PSF stack defined UPD350 read driver 
        MCHP_PSF_HOOK_UPD_READ and Harmony generated SAMD20 SPI read driver.
    Conditions:
        None
    Input:
        u8PortNum - Port number. Value passed will be less than CONFIG_PD_PORT_COUNT
        pu8WriteBuffer- Pointer to Write Buffer to be written first before read take place 
        u8Writelength - Write Buffer length to written on SPI bus
        pu8ReadBuffer - Pointer to Read buffer
        u8Readlength - Length of bytes to read
    Return:
        UINT8 - Returns TRUE - If the SPI read is successful.
                Returns FALSE - If the SPI read is not successful
    Remarks:
        None
**************************************************************************************************/

UINT8 SAMD21_SPIReaddriver (UINT8 u8PortNum, UINT8 *pu8WriteBuffer, UINT8 u8Writelength,\
                UINT8 *pu8ReadBuffer, UINT8 u8Readlength);
/****************************************************************************
    Function:
        UINT8 SAMD20_SPIWritedriver (UINT8 u8PortNum, UINT8 *pu8WriteBuffer, UINT8 u8Writelength)
    Summary:
        Wrapper function for SPI write drivers.  
    Description:
        This API serves as a wrapper between PSF stack defined UPD350 write driver 
        MCHP_PSF_HOOK_UPD_WRITEand Harmony generated SAMD20 SPI write driver.
    Conditions:
        None
    Input:
        u8PortNum - Port number. Value passed will be less than CONFIG_PD_PORT_COUNT
        pu8WriteBuffer- Pointer to Write Buffer to be written 
        u8Writelength - Write Buffer length to written on SPI bus
    Return:
        UINT8 - Returns TRUE - If the SPI write is successful.
                Returns FALSE - If the SPI write is not successful
    Remarks:
        None
**************************************************************************************************/
UINT8 SAMD21_SPIWritedriver (UINT8 u8PortNum, UINT8 *pu8WriteBuffer, UINT8 u8Writelength);

/****************************************************************************
    Function:
        SAMD20_UPD350AlertCallback(uintptr_t u8PortNum)
    Summary:
        UPD350 Alert Callback wrapper function.  
    Description:
        This API serves as a wrapper for PSF's function MchpPSF_UPDIrqHandler to 
        register as callback for SAMD20's function EIC_CallbackRegister.
    Conditions:
        None
    Input:
        u8PortNum - Port number. Value passed will be less than CONFIG_PD_PORT_COUNT
    Return:
        None.
    Remarks:
        None
**************************************************************************************************/
void SAMD21_UPD350AlertCallback(uintptr_t u8PortNum);

/****************************************************************************
    Function:
        void SAMD20_EnterCriticalSection(void)
    Summary:
        Wrapper function to SAMD20 disable interrupts globally to provide critical section
    Description:
        This API serves as a wrapper between PSF stack defined SOC interrupt disable function
        MCHP_PSF_HOOK_DISABLE_GLOBAL_INTERRUPT and SAMD20 global interrupt disable option.
    Conditions:
        None
    Input:
        None
    Return:
        None
    Remarks:
        None
**************************************************************************************************/
void SAMD21_EnterCriticalSection(void);

/****************************************************************************
    Function:
        void SAMD20_EnterCriticalSection(void)
    Summary:
        Wrapper function to SAMD20 enable interrupts globally to provide critical section
    Description:
        This API serves as a wrapper between PSF stack defined SOC interrupt enable function
        MCHP_PSF_HOOK_ENABLE_GLOBAL_INTERRUPT and SAMD20 global interrupt enable option.
    Conditions:
        None
    Input:
        None
    Return:
        None
    Remarks:
        None
**************************************************************************************************/
void SAMD21_ExitCriticalSection(void);

/**************************************************************************
    Function:
        void* SAMD20_MemCpy(void *dest, const void *src, int n)
    Summary:
        Wrapper to Copy one memory area to another memory area
    Description:
        It is a wrapper for PSF stack's MCHP_PSF_HOOK_MEMCPY
    Conditions:
        None.
    Input:
        dest -  pointer to block of destination memory region
        src -   pointer to block of source memory region
        n -   number of bytes to be copied.
    Return:
        None.
    Remarks:
        None                    
 **************************************************************************/
void* SAMD21_MemCpy(void *pdest, const void *psrc, int ilen);

/**************************************************************************
    Function:
        int SAMD20_MemCmp(const void *pau8Data1, const void *pau8Data2, int ilen)
    Summary:
        Wrapper to compare two memory location
    Description:
        It is a wrapper for PSF stack's MCHP_PSF_HOOK_MEMCMP
    Conditions:
        None.
    Input:
        pau8Data1 -  pointer to block of Memory region 1
        pau8Data2 -  pointer to block of Memory region 2
        ilen -   number of bytes to be compared.
    Return:
        Returns 0 if two memory regions are same.
    Remarks:
        None                    
**************************************************************************/
int SAMD21_MemCmp(const void *pau8Data1, const void *pau8Data2, int ilen);

/*Debug UART APIs*/
#if (TRUE == CONFIG_HOOK_DEBUG_MSG)
void SAMD21_UART_Initialisation(void);

void SAMD21_UART_Write_Char(char);

void SAMD21_UART_Write_Int(UINT32, UINT8);

void SAMD21_UART_Write_String(char*);

#endif //CONFIG_HOOK_DEBUG_MSG


#endif /*_DRIVERS_H */

/* *****************************************************************************
 End of File
 */
