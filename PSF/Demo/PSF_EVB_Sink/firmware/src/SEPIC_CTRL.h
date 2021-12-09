/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    SEPIC_CTRL.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */
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
#ifndef _SEPIC_CTRL_H    /* Guard against multiple inclusion */
#define _SEPIC_CTRL_H

enum battery_status {
    FAULT = 0,
    PRECONDITIONING = 1,
    CCMODE = 2,
    CVMODE = 3,
    CHARGED = 4,
    RECHARGE = 5
};

enum fault_code {
    GENERIC = 0,
    NOSOURCE = 1,
    UVLO = 2,
    OVLO = 3,
    OVERTEMP = 4,
    UNDERTEMP = 5

};

void SEPIC_init();
void SEPIC_calibrate();
void updateFuelGauge(int battSOC);
void clearFuelGauge();
void blinkLED(uint16_t addr);
void SEPIC_run(void);
uint8_t getStatus();
uint8_t getFaultCode();
void setBattStatus();
void disableSEPIC();
void stopSEPIC();
//update the fuel gauge and run the SEPIC state machine
void adjustTC4(uint16_t period);
void TIMER4_EventHandler(uintptr_t context);
void TIMER7_EventHandler(uintptr_t context);

uint16_t readBattV();
void setBattV(uint16_t value);
uint16_t getBattV();

uint16_t readChargeI();
uint16_t getChgI();
void setChgI(uint16_t current);

void setIbatRef();
void setInputILIMRef();


uint16_t readMikroAN();

void estimateSOC(uint16_t voltage);
uint16_t getSOC();
/* Provide C++ Compatibility */
#ifdef __cplusplus

#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
