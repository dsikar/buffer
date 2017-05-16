/***************************************************************************
* $Workfile < peripheral.h >
* @brief  System Non-Volatile Memory Store Objects - Definitions
* @ingroup
* $Used On:     FV400
* $Revision:    <>$
*
* Copyright © ThornSecurity.
 ***************************************************************************/
#ifndef INC_PERIPHERAL
#define INC_PERIPHERAL

#define INDICATION_LIMIT 30U

#define SET_TO_HIGH_GAIN true
#define SET_TO_LOW_GAIN false
#define SIGNAL_ALARM true
#define CLEAR_ALARM false

#define SPI_INIT_DELAY 50U
#define CONTINIOUS_MODE_COMP 10U
#define FAULT_LOOP_LVL 0U
#define _7_PERCENT (7 / 100)

#define ALM_LED_PIN 0x02U

enum{
   LED_OFF,
   LED_ON,
   LED_EIGHTHHZ,
   LED_HALFHZ,
   LED_1HZ,
   LED_3HZ,
   LED_5HZ,
   // While the LED is in any of the states above, it can
   // be pulsed (inverted) for a period of 
   LED_ONESHOT_INACTIVE,//7
   LED_ONESHOT_200MS,
   LED_ONESHOT_300MS,
   LED_ONESHOT_500MS
};

/* HOW TO USE THE PRODUCT ID
Any changes to the PCB or BOM requires the
'Software compatability revision' to be incremented.
This should be done even if the change does not require software
support.
H(R31)          : NoFit=PAL, Fit=NTSC
G(R32)          : Reserved. Must be no fit.
F(R33) - A(R38) : Software compatability revision
*/
enum{             // Add to this list, DO NOT modify it
   ISSUE00_REVB,  // 0000 (prototype)
   ISSUE00_REVC,  // 0001 (prototype)
   ISSUE00_REVD,  // 0010 (prototype)
   ISSUE00_REVE,  // 0011 (prototype)
   ISSUE00_REVF,  // 0100 (Development batch)
   ISSUE01_BOM_01,// 0101 (pre production)
   // Add new issue names above this line
   ISSUE_NOTSUPORTED

//----------------ADD NEW REVISIONS ABOVE THIS COMMENT-------------------//
};

// enum for window heater state
enum{
   WINDOW_OFF,
   WINDOW_ON,
   WINDOW_AUTO,
   WINDOW_ENDMARKER
};

enum{
   ALL_LAMPS_OFF,
   WINDOW_LAMP_OFF,
   WINDOW_LAMP_ON,
   ALARM_LAMP_OFF,
   ALARM_LAMP_ON
};

BYTEu HardwareGetProductId(void);
void InitProductId(void);
BYTEu GetProductId(void);
BYTEu GetSoftwareCompatibilityId(void);
BYTEu VideoIsNtsc(void);
//following for Fv400
void AlarmLED(BYTEu AlarmLED_OnOff);
void AlarmLEDToggle(void);
void AlarmRelay(BYTEu AlarmRelay_OnOff);
void ConvEOLTest(BYTEu EOLOnOff);
void ConvAlarmControl(BYTEu AlarmOnOff);
void FaultRelay(BYTEu FaultRelay_OnOff);
void FaultLED(BYTEu FaultLED_OnOff);
BYTEu GetAlarmRelayState(void);
void LampControl(BYTEu lampnstate);
void MxAiscSwitch(BYTEu mxswitch);
void IrRcvCktSwitch(BYTEu swstate);
void Timer3MXPWMEnable(void);//Feb 2011
void OutputMX_4_20mALevel(BYTEu level);
void SetHartModem(BYTEu swstate);
//till here
void InitJtagInterface(void);

void KickWdt(void);
void KickWdtMain(void);
void ControlledKickWdt(BYTEu hilo);
void PowerUpIndication(void);
void SetPyroElectricalPin(BYTEu swstate);
void GainControl(BYTEu setpinvl);

void Enable_4_20mA(void);
void Disable_4_20mA(void);
void Output_4_20mA_Level(BYTEu level);
void StopTimer0_IRQs (void);
BYTEu ReadAlarmLED(void);

void FaultLedStateMachine(void);
void AlarmSignallingHandlerAlgo(BYTEu almstatus,BYTEu latchstatus,
                            BYTEu prealarmstatus);
// Test point routines
void InitTestPoint(void);   

#endif   //#ifndef 
