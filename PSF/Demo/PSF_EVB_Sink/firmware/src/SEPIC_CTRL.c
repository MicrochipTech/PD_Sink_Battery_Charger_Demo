/* ************************************************************************** */
/** SEPIC Control

  @Company
    Microchip Technology Inc.

  @File Name
    SEPIC_CTRL.c

  @Summary
    This file contains the state machine to run and control the SEPIC on the USB-PD demo board.

  @Description
    This file sets up and runs the SEPIC on the USB PD demo board through a state machine  */
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
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "psf_stdinc.h"                 // PSF include file
#include <stdio.h>
#include <xc.h>
#include "SEPIC_CTRL.h"
#include "time_delay.h"
#include <inttypes.h>

//set battery parameters
#define CELLVMIN 2600 //inidivual cell min voltage in mV
#define CELLVMAX 4200 //inidivual cell max voltage in mV
#define BATTIMAX 3000 //max charge current in mA
#define NUMCELLS 3 //number of series cells
#define UCLO 150 //charge cutoff current in mA
#define BATTVMAX (NUMCELLS * CELLVMAX) //total battery maximum voltage
#define BATTVMIN (NUMCELLS * CELLVMIN) //total battery minimum voltage (UVLO value)
#define RECHARGETHRESH 4050*NUMCELLS //threshold for trickle charge engage
#define MINCCCHARGETHRESH 3000*NUMCELLS //threshold for full speed cc charging
#define CVTHRESH 4100*NUMCELLS //threshold to swtich from CC to CV charge

#define VSENSEGAIN 499000/(3300000+499000) //divider calcs for VOUT
#define VREF 1.664 //half of VDDA

#define PERVAL 2399 //this is the period value from TCC2 harmony config
#define CLKREQ 48000000

#define CALEN 0   //calibration mode enable, 0 = off, 1 = on

enum battery_status status = FAULT;
enum fault_code faultcode = NOSOURCE;
//UPD350 register values
UINT8 blinkstat = 0x0F;
uint8_t LEDON = 0x0F;
UINT8 LEDOFF = 0x07;

//this variable controls the ADC channel switching for each timer interrupt
uint8_t timerop = 1;

uint8_t pwmrunning = FALSE;
uint16_t IbatRefPWM = 160; //this is value where current actually starts to increase
uint16_t ILIMRefPWM = 0;

uint16_t BattSOC = 0, BattV = 0, chgI = 0;
uint16_t interval = 0;

//calibration factors
float calIoffset = 0;
uint16_t calIslope = 1000;
uint32_t readval = 0;

uint16_t mikroCurrent = 0;

//used to adjust voltage reading for manual test setup
float leadresistance = .13;

//initialize sepic by setting control signal initial values to a safe state

void SEPIC_init() {
    CHG_EN_Clear(); //disable charger
    BATT_EN_Clear(); //disconnect the battery
    ADC_Enable();
    //set the duty cycle to 0 to begin
    TCC2_REGS->TCC_CC[1] = 0U;
    TCC0_REGS->TCC_CC[0] = 0U;

    TC4_TimerCallbackRegister((TC_TIMER_CALLBACK) TIMER4_EventHandler, (uintptr_t) NULL);
    TC7_TimerCallbackRegister((TC_TIMER_CALLBACK) TIMER7_EventHandler, (uintptr_t) NULL);


    NVMCTRL_Read(&readval, sizeof (uint32_t), 0x00806020);
    printf("\r\nBOOTPROT Size %" PRIu32 "\r\n", readval & 0b0111);
    NVMCTRL_Read(&readval, sizeof (uint32_t), 0x00806020);
    printf("EEPROM Size %" PRIu32 "\r\n", readval & 0x70);

    uint32_t bufferarray[64];
    if (CALEN) {
        NVMCTRL_Read(bufferarray, 64 * sizeof (uint32_t), 0x3C000);
        printf("Calibration complete? %" PRIu32 "\r\n", bufferarray[0]);
        if (bufferarray[0] == 1) {
            //read calibration values from EEPROM

            NVMCTRL_Read(bufferarray, 64 * sizeof (uint32_t), 0x3C000);

            calIslope = bufferarray[1];
            calIoffset = bufferarray[2];

            printf("slope from memory: %" PRIu32 "\r\n", bufferarray[1]);
            printf("offset from memory: %" PRIu32 "\r\n", bufferarray[2]);
        }
    }

    //set TC4 to 250ms loop
    adjustTC4(500);
    TC7_TimerStart();

    readBattV();
    clearFuelGauge();
    int voltage = getBattV();
    estimateSOC(voltage);
    updateFuelGauge(getSOC());
}

void SEPIC_calibrate() {
    int enteredCurrent1 = 0;
    int enteredCurrent2 = 0;

    uint16_t adcI1 = 0;
    uint16_t adcI2 = 0;

    BATT_EN_Set();
    CHG_EN_Set();

    printf("\r\nCalibration start\r\n");
    TCC0_REGS->TCC_CC[0] = 0.75 * PERVAL;
    TCC2_REGS->TCC_CC[1] = 198; //command 200mA

    //    ADC_ChannelSelect(ADC_POSINPUT_PIN6, ADC_NEGINPUT_GND);
    printf("Data point 1: \r\n");
    //    printf("Enter measured voltage in mV: \r\n");
    //    scanf("%d", &enteredVoltage1);
    printf("Enter measured current in mA: \r\n");
    scanf("%d", &enteredCurrent1);

    readChargeI();
    adcI1 = getChgI();
    adcI1 = 198;

    TCC2_REGS->TCC_CC[1] = 255; //command 400 mA

    readChargeI();
    adcI2 = getChgI();
    adcI2 = 400;
    //    ADC_ChannelSelect(ADC_POSINPUT_PIN7, ADC_NEGINPUT_GND);

    printf("Data point 2: \r\n");
    //    printf("Enter measured voltage in mV: \r\n");
    //    scanf("%d", &enteredVoltage2);
    printf("Enter measured current in mA: \r\n");
    scanf("%d", &enteredCurrent2);
    //
    //    readBattV();
    //    adcV2 = getBattV();

    TCC0_REGS->TCC_CC[0] = 0;
    TCC2_REGS->TCC_CC[1] = 0;

    //    BATT_EN_Clear();
    //    CHG_EN_Clear();

    printf("Data point 1 results: \r\n");
    printf("Entered I1: %d \r\n", enteredCurrent1);
    printf("ADC I: %d \r\n", adcI1);

    printf("Data point 2 results: \r\n");
    printf("Entered I2: %d \r\n", enteredCurrent2);
    printf("ADC I: %d \r\n \r\n", adcI2);

    float actualslope = (float) (enteredCurrent2 - enteredCurrent1) / (adcI2 - adcI1);

    float offset = enteredCurrent1 - actualslope * 200; //200 is for 200mA commanded during test
    //    printf("%d\r\n", (uint16_t)(actualslope*1000));
    if (offset < 0) {
        offset = offset*-1;
    }

    uint32_t slopeval = (uint32_t) (actualslope * 1000); //multiply by 1000 for storage as int
    uint32_t offsetval = (uint32_t) (offset); //multiply by -1 to get a positive value to store in uint

    printf("Calibration slope %" PRIu32 "\r\n \r\n", slopeval);
    printf("Calibration offset %" PRIu32 "\r\n \r\n", offsetval);

    uint32_t calfinished = 0;

    //make sure values are within an expected range before writing them
    if (slopeval > 0 && slopeval <= 2000 && offsetval >= 0 && offsetval < 500) {
        calfinished = 1;

        uint32_t bufferarray[64];
        NVMCTRL_Read(bufferarray, 64 * sizeof (uint32_t), 0x3C000);
        NVMCTRL_RowErase(0x3C000);
        while (NVMCTRL_IsBusy());
        bufferarray[0] = calfinished;
        bufferarray[1] = slopeval;
        bufferarray[2] = offsetval;
        NVMCTRL_PageWrite(bufferarray, 0x3C000);
        while (NVMCTRL_IsBusy());
        NVMCTRL_Read(bufferarray, sizeof (uint32_t), 0x3C000);
        printf("Calibration complete? %" PRIu32 "\r\n", bufferarray[0]);


        //        NVMCTRL_Read(&readval, sizeof (uint32_t), 0x3C0A0);
        //        printf("Calibration complete? %" PRIu32 "\r\n", readval);
        calIoffset = offset;
        calIslope = (uint16_t) slopeval;
    } else {
        slopeval = 1;
        calIslope = 1000;
        calIoffset = 0;
        offsetval = 0;
        calfinished = 0;

        uint32_t bufferarray[64];
        NVMCTRL_Read(bufferarray, 64 * sizeof (uint32_t), 0x3C000);
        NVMCTRL_RowErase(0x3C000);
        while (NVMCTRL_IsBusy());
        bufferarray[0] = calfinished;
        bufferarray[1] = slopeval;
        bufferarray[2] = offsetval;
        NVMCTRL_PageWrite(bufferarray, 0x3C000);
    }
}
uint8_t lastSOC = 0;

void estimateSOC(uint16_t voltage) {
    float SOC = 0;

    if (voltage <= BATTVMIN) {
        SOC = 0;
    } else {
        SOC = (float) ((voltage - BATTVMIN)*100 / (BATTVMAX - BATTVMIN));
    }
    if (SOC > 100 || status == CHARGED) {
        SOC = 100;
    }

    if (SOC >= lastSOC) {
        BattSOC = SOC;
    }
    lastSOC = BattSOC;
}

uint16_t getSOC() {
    return BattSOC;
}

void setBattV(uint16_t v) {
    BattV = v;
}

uint16_t getBattV() {
    return BattV;
}

void setChgI(uint16_t current) {
    chgI = current;
}

uint16_t getChgI() {
    return chgI;
}

//this function will run when a charge source is connected and a contract has been established

int deltaCurrent = 0;
int lastCurrent = 0;

void setIbatRef() {

    int pwmincrement = 3;

    uint16_t maxcurrent = gasCfgStatusData.sPerPortData[0].u16NegoCurrentInmA;
    //set this value for a manual max charge current limit,
    //otherwise comment this line to use the PD negotiated current
    //maxcurrent = 750;

    if (maxcurrent > 1500) {
        maxcurrent = 1500;
    }

    if (status == PRECONDITIONING) {
        maxcurrent = 200; //limit current in pre-conditioning mode
    }

    deltaCurrent = chgI - lastCurrent;
    //printf("delta current: %d\r\n", deltaCurrent);
    int currentTolerance = 50; //add 50mA current tolerance for stability
    if (status == CCMODE || status == PRECONDITIONING || (status == CVMODE && chgI < UCLO)) {
        if (chgI >= (maxcurrent - currentTolerance) && chgI <= (maxcurrent + currentTolerance)) {
            //do nothing because you are within 5% of set current level
            //        } else if (chgI < (maxcurrent - currentTolerance) && (deltaCurrent >= 1 || chgI <= 0)) {
        } else if (chgI < (maxcurrent - currentTolerance)) {
            //charge current is below max allowed current
            IbatRefPWM += pwmincrement;
        } else if (chgI > (maxcurrent + currentTolerance)) {
            //charge current is above max allowed current
            IbatRefPWM -= pwmincrement;
        }
    } else if (status == CVMODE && chgI > UCLO) {
        //manual 25 mV adjustment added
        if (getBattV() >= BATTVMAX - 25) {

            IbatRefPWM -= 2 * pwmincrement;
        }
    } else if (status == RECHARGE) {
        maxcurrent = 500;
        if (chgI >= maxcurrent - currentTolerance && chgI <= maxcurrent + currentTolerance) {
            status = CVMODE; //change to CV charge mode to slowly decrement current on trickle mode
        } else if (chgI < (maxcurrent - currentTolerance)) {
            //charge current is below max allowed current
            IbatRefPWM += pwmincrement;
        } else if (chgI > (maxcurrent + currentTolerance)) {
            //charge current is above max allowed current
            IbatRefPWM -= 2 * pwmincrement;
        }
    } else {
        IbatRefPWM = 0;
    }

    if (IbatRefPWM < 0) {
        IbatRefPWM = 0;
    } else if (IbatRefPWM > PERVAL) {
        IbatRefPWM = PERVAL;
    }
    lastCurrent = chgI;

    //IbatRefPWM = PERVAL*.5;
    printf("pwm value: %d\r\n", IbatRefPWM);
    TCC2_REGS->TCC_CC[1] = IbatRefPWM;
}

void setInputILIMRef() {
    //set input limit to 500mA above the max charge limit
    //    uint16_t inputILIM = gasCfgStatusData.sPerPortData[0].u16NegoCurrentInmA + 500;

    //calculate the corresponding PWM duty cycle based on control loop calcs
    //    double multiplier = (double) inputILIM / 3300.0;
    //    if (inputILIM > 3300)
    //        multiplier = 1;

    //printf("duty val: %d - multiplier: %d - inputILIM: %d \r\n", (uint16_t) (multiplier * PERVAL), (int) (multiplier * 100), inputILIM);
    //set the duty cycle
    TCC0_REGS->TCC_CC[0] = (uint16_t) (1 * PERVAL);
    //    TCC0_REGS->TCC_CC[0] = .75 * PERVAL;
}

void disableSEPIC() {

    TCC2_PWMStop();
    TCC0_PWMStop();
    //TC7_TimerStop();
    pwmrunning = FALSE;
    IbatRefPWM = 0;
    CHG_EN_Clear();
    MCP1632_EN_Set();
}

void stopSEPIC() {
    TCC2_PWMStop();
    TCC0_PWMStop();
    //TC7_TimerStop();
    pwmrunning = FALSE;
    IbatRefPWM = 0;
    CHG_EN_Clear();
    MCP1632_EN_Set();
}

//run the SEPIC state machine to control charge states

void SEPIC_run() {

    uint8_t u8Data;
    UPD_RegisterRead(0, TYPEC_VBUS_MATCH, &u8Data, 1);

    if (CALEN && u8Data > 0x0D) {
        //        uint32_t calvalue;
        //        NVMCTRL_PageWrite(&readval, 0x3C000);
        uint32_t bufferarray[64];
        NVMCTRL_Read(bufferarray, 64 * sizeof (uint32_t), 0x3C000);
        //        printf("Calibration complete? %" PRIu32 "\r\n", bufferarray[0]);

        if (bufferarray[0] != 1) {
            SEPIC_calibrate();
        }
    }

    //    printf("max negotiated current: %d \r\n", gasCfgStatusData.sPerPortData[0].u16NegoCurrentInmA);
    //    printf("max negotiated voltage: %d \r\n", gasCfgStatusData.sPerPortData[0].u16NegoVoltageInmV);
    setBattStatus();

    if (status == FAULT) {
        //printf("Battery Status: Fault\r\n");
        disableSEPIC();
    } else if (status > FAULT && status != CHARGED) {
        if (!pwmrunning) {
            TCC2_PWMStart();
            TCC0_PWMStart();
            TC7_TimerStart();
            pwmrunning = TRUE;
            BATT_EN_Set();
            CHG_EN_Set();
            MCP1632_EN_Clear();
            IbatRefPWM = 850;
            //printf("charge mode enabled\r\n");
        }

    } else if (status == CHARGED) {
        stopSEPIC();
    } else {
        //default
        disableSEPIC();
    }
}

//return status of the state machine

uint8_t getStatus() {
    return status;
}

//return the fault code

uint8_t getFaultCode() {
    return faultcode;
}

//readjusts the period and clock pre-scaler for TC4 used in SEPIC control loop

void adjustTC4(uint16_t period) {
    TC4_TimerStop();
    uint16_t periodval = 0;
    periodval = period * (CLKREQ / 256) / 1000;
    TC4_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_WAVEGEN_MPWM;
    if (periodval > sizeof (uint16_t)) {
        periodval = period * (CLKREQ / 1024) / 1000;
        TC4_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_WAVEGEN_MPWM;
    }
    TC4_Timer16bitPeriodSet(periodval);
    TC4_TimerStart();
}

//do ADC readings to determine what state the battery is in based on battery voltage and charge current

void setBattStatus() {
    uint16_t voltage = getBattV();
    estimateSOC(voltage);


    uint8_t u8Data;
    //    char debug_string[32];
    UPD_RegisterRead(0, TYPEC_VBUS_MATCH, &u8Data, 1);
    //    sprintf(debug_string, "VBUS_MATCH = %02X\r\n", u8Data);
    //    MCHP_PSF_HOOK_PRINT_TRACE(debug_string);

    //if u8Data > 0 then it recognizes a source is connected
    //Note: manual hysteresis values have been added/subtracted to some threshold values
    if (u8Data > 0x00) {
        //         printf("battV: %d \r\n", battV);
        if (voltage > BATTVMIN && voltage < MINCCCHARGETHRESH) {
            status = PRECONDITIONING;
        } else if ((voltage >= MINCCCHARGETHRESH + 50) && (voltage < CVTHRESH - 50) && status != CHARGED) {
            if (status != CCMODE) {
                adjustTC4(500);
            }
            status = CCMODE;
        } else if (status != CHARGED && voltage >= CVTHRESH && chgI > 200) {
            if (status != CVMODE) {
                adjustTC4(1000);
            }
            status = CVMODE;
            //manually subtract 50mV offset
        } else if (voltage >= BATTVMAX - 25 && chgI <= 200) {
            if (status != CHARGED) {
                adjustTC4(1000);
            }
            status = CHARGED;

        } else if ((status == CHARGED) && voltage <= RECHARGETHRESH - 75) {
            if (status == CHARGED) {
                adjustTC4(1000);
            }
            status = RECHARGE;
        } else {
            //status unchanged or fault
            //printf("battV is: %d and Minimum is: %d\r\n", battV, BATTVMIN);
            if (voltage < BATTVMIN) {
                status = FAULT;
                faultcode = UVLO;
            } else if (voltage > BATTVMAX) {
                status = FAULT;
                faultcode = OVLO;
            }
        }
    } else {
        status = FAULT;
        faultcode = NOSOURCE;
        //printf("Fault: No charge source connected\r\n");
    }
    //printf("Status: %d\r\n", status);
}

//Read the battery voltage level

uint16_t readBattV() {
    //set the ADC channel for the voltage sense input and get the result

    ADC_ChannelSelect(ADC_POSINPUT_PIN7, ADC_NEGINPUT_GND);
    ADC_REGS->ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC1; 
    ADC_ConversionStart(); //start conversion
    while (!ADC_ConversionStatusGet());
    //average over 50 samples to improve stability
    uint16_t adcresult = ADC_ConversionResultGet();
    ;

    //math to convert result into battery voltage reading
    //    uint16_t voltage = ((adcresult * VREF / (1023 * VSENSEGAIN)*1000+711.54)/983.2)*1005-1045;
    //    uint16_t voltage = (uint16_t) (adcresult * VREF / (1023 * VSENSEGAIN)*1000) - leadresistance * getChgI();
    uint16_t voltage = (uint16_t) (adcresult * VREF / (1023 * VSENSEGAIN)*1000) - 75;
    //printf("Battery voltage: %d mV \r\n", voltage);
    
    if(voltage > NUMCELLS*CELLVMAX*1.1){
        voltage = 0;
    }
    
    return voltage;
}

//Read the battery charge current

uint16_t readChargeI() {
    //set the adc channel for the current sense input and get the result

    ADC_ChannelSelect(ADC_POSINPUT_PIN6, ADC_NEGINPUT_GND);
    ADC_REGS->ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC0;  
    ADC_ConversionStart(); //start conversion
    while (!ADC_ConversionStatusGet());
    uint16_t adcresult = ADC_ConversionResultGet();
    
    //math to convert result into charge current reading
    float measuredCurrent = (adcresult * 2.2 / (1023 * 0.01 * 34.05)*1000);
    //3600 offset is due to 1.2V offset on error amplifier output
    if (measuredCurrent >= 3600) {
        measuredCurrent -= 3600;
    } else {
        return 0;
    }
    uint16_t current = (uint16_t) measuredCurrent;
    //float current = (uint16_t) (((measuredCurrent + 14.564) / 100.88)*88.664 + 19.2);
    //printf("charge current: %d mA before calibrate\r\n", (uint16_t) current);
    //this is the calibrated current based on measured curves
    //current = ((calIslope * current / 1000) - calIoffset);
    //printf("calIslope: %d offset: %d\r\n", (uint16_t) (calIslope), (uint16_t) calIoffset);

    //printf("charge current: %d mA after calibrate\r\n", (uint16_t) current);

    //control for integer overflows
    if ((uint16_t) current > 65000) {
        return 0;
    }

    return (uint16_t) current;
}

//TC4 runs the SEPIC state machine

void TIMER4_EventHandler(uintptr_t context) {
    setChgI(readChargeI());
    SEPIC_run();
    setBattV(readBattV());

    setIbatRef();
    setInputILIMRef();

}

//slower loop to handle LED blink frequency

void TIMER7_EventHandler(uintptr_t context) {
    updateFuelGauge(getSOC());
    printf("battV: %d mV - chgI: %d mA - status: %d\r\n", getBattV(), getChgI(), status);
}

void blinkLED(uint16_t addr) {
    //blink the highest LED to indicate the battery is in charge mode
    if (status == CCMODE || status == CVMODE || status == PRECONDITIONING) {
        UPD_RegisterWrite(0, addr, (UINT8 *) & blinkstat, 1);
        if (blinkstat == 0x0F) {
            blinkstat = 0x07;
        } else {
            blinkstat = 0x0F;
        }
    } else {
        UPD_RegisterWrite(0, addr, (UINT8 *) & LEDOFF, 1);
    }
}

//turn off all the LEDs on the demo board fuel gauge

void clearFuelGauge() {
    //turn off all the leds on the fuel gauge

    uint16_t regadr = 0x0036;
    for (regadr = 0x0036; regadr < 0x0040; regadr++) {
        UPD_RegisterWrite(0, regadr, (UINT8 *) & LEDOFF, 1);
    }
}

//Control the LED fuel gauge indicator
//the code blinks the highest LED based on SOC during charging to indicate that the device is charging

void updateFuelGauge(int battSOC) {

    if (status == FAULT) {
        clearFuelGauge();
    }

    if (battSOC > 75 && status != FAULT) {
        UPD_RegisterWrite(0, 0x0036, (UINT8 *) & LEDON, 1);
        UPD_RegisterWrite(0, 0x0037, (UINT8 *) & LEDON, 1);
        UPD_RegisterWrite(0, 0x0038, (UINT8 *) & LEDON, 1);
        if (status != CHARGED) {
            blinkLED(0x0039);
        } else {
            UPD_RegisterWrite(0, 0x0039, (UINT8 *) & LEDON, 1);
        }

    } else if (battSOC > 50 && status != FAULT) {
        UPD_RegisterWrite(0, 0x0036, (UINT8 *) & LEDON, 1);
        UPD_RegisterWrite(0, 0x0037, (UINT8 *) & LEDON, 1);
        blinkLED(0x0038);
        UPD_RegisterWrite(0, 0x0039, (UINT8 *) & LEDOFF, 1);
    } else if (battSOC > 25 && status != FAULT) {
        UPD_RegisterWrite(0, 0x0036, (UINT8 *) & LEDON, 1);
        blinkLED(0x0037);
        UPD_RegisterWrite(0, 0x0038, (UINT8 *) & LEDOFF, 1);
        UPD_RegisterWrite(0, 0x0039, (UINT8 *) & LEDOFF, 1);

    } else if (battSOC > 0 && status != FAULT) {
        blinkLED(0x0036);
        UPD_RegisterWrite(0, 0x0037, (UINT8 *) & LEDOFF, 1);
        UPD_RegisterWrite(0, 0x0038, (UINT8 *) & LEDOFF, 1);
        UPD_RegisterWrite(0, 0x0039, (UINT8 *) & LEDOFF, 1);
    } else {
        clearFuelGauge();
    }
}