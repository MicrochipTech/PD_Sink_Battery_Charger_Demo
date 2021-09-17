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
