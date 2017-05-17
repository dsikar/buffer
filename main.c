/***********************************************************************//**
* @file  main.c
* @brief
*  This file contains the main loop, before jumping into the main loop system
*  initilization functions were called in here as well. This file is the top level
*  view of FV400 code.
*  Used On:     FV400
***************************************************************************/
#define  ENABLE_BIT_DEFINITIONS
#include <iom1281.h>
#include <inavr.h>
#include <pgmspace.h>

#include "defs.h"
#include "library.h"
#include "uart.h"
#include "spi.h"
#include "video.h"
#include "peripheral.h"
#include "fde.h"
#include "interruptcaptureandhwmonitor.h"
#include "panelcomms.h"
#include "configuration.h"
#include "debug.h"
#include "opmfaultsentinel.h"
#include "adc.h"
#include "serviceprotocol.h"
#include "servicecommands.h"
#include "hardwarefaultsentinel.h"
#include "ir.h"
#include "hardwaretest.h"
#include "modbus.h"
#include "termdebug.h"
#include "heatedoptics.h"
#include "s231interface.h"
#include "AFDPL15_IAR.H"

#define  ENABLE_BIT_DEFINITIONS

void MainApplication(void);

/*******************************************************//**
*  MainApplication();
*
*  This is the top level function of FV400 code, which contains
*  embedded super loop. Major elements of the loop are as follow:
*
*  AlarmSentinel()-S250 algorithm, processes pyro data and reports
*  alarm if found.
*
*  ModbusOrSmpHandler()-As per configuration looks for modbus or
*  SMP packet. This will collect data from buffer within UART
*  interrupt.
*
*  HardwarefaultMonitor/sentinel()- Monitors hardware fault conditions.
*  Puts detector into hardware fault if any fault detected. Monitored
*  analogue channels are read within flame Timer 2, 10ms interrupt.
*
*  CheckExternalRemoteTest()- Looks for wired walktest input test
*  request.
*
*  UpdateTimerCounter()- Updates internal time registers like
*  "secondssincepowerup". This functionality is also driven by
*  same Timer 2 10ms interrupt.
*
*  KickWDT()-Watchdog
*
*  VideoDisplayUpdate()- Updates if any new text is
*  there in buffer, doesn't works in conventional mode.
***********************************************************/
void MainApplication(void)
{
  /* Initalise timers */
  SetTimeUntilNextOpm(TagToConfigReadInt(OPMTIMER));
  /* Set time of last walktest to the current timer value */
  WalkTestReminder();
  /* Look for any DIP Faults */
  DipSwitchFaultMon();
  /* set and start the pyro timer here. */
  StartInterruptTimer2();
  ServiceTimer2_10msMode(true);
  /* Init Test Byte */
  InitTestByte();
  /* Embedded loop:- */

  while (true)
  {
    /* Entry to alarm algo */
    AlarmSentinel();   /* timings 40us to 1ms aug 2011 */
    /* Call Modbus/SMP interface handler */
    ModbusOrSmpHandler();
    /* max time 820 us */
    OpmSentinel();
    /* Periodic selftest routines for faults, temperature etc */
    HardwareFaultSentinel();
    /* Update state of the heated optics based on configuration */
    /* UpdateHeatedOpticsState(); */
    /* Check external Walktest input */
    CheckExternalRemoteTest();
    /*
     * Update the internal timers from interrupt
     * driven 8-bit, 10mS counter
     * This allows reading of timer values without
     * having to disable interrupts
     */
    UpdateTimerCounter();
    /* Update video display if not in conv mode */
    /* VideoUpdateDisplay(); */
    /* Watchdog and Conventional Fault Monitoring*/
    KickWdtMain();
  }   
  /* end of embedded loop */
} /* void */

/*******************************************************//**
*  main()
*
*  All the initilization functions are called in here at the
*  end "MainApplication" function is called.
***********************************************************/
void main(void)
{
  StartUpPowerManagement();
  /* Init Timer and Nonvolatile timestamp.
  *Initlize internal one second counter */
  InitTimer();
  /* Before we can start the logging, we must initalise the
   * detector configuration EEPROM. This is because the logging
   * levels are themselves a configurable item! eg LOGENABLE.
   * InitBlankDce() checks for a blank EEPROM and initalises the
   * configuration to factory defaults - including the factory
   * protected data area
   * At this point, the configuration can be in 3 states:
   * 1) New build - will have fully initalised DCE to factory
   *                defaults. This includes the factory data
   * 2) Upgrade --  Data in DCE will be marked as having a
   *                different revision. In this case the version
   *                currently running must save important data,
   *                and initalise the new DCE fields
   * 3) CRC error - The revisons may match, but a CRC error
   *                may exist
   * 4) OK --       Revision and checksum are as expected. No action
   */
  InitDceAndFde();
  /*
   * Verify if the configuration EEPROM revision is valid. If
   * not, a firmware upgrade has just occured that requires
   * reinitalisation of the DCE. Log event if this is the case
   */
  CheckDceRevisionValid();
  /*
   * Now the ADC DAC has been setup, we can write a power on log
   * to mark the fact the detector has been rebooted.
   * We have waited this long because a voltage measurment forms
   * part of the log.
   */
  LogPowerUp();
  /* time for inetial HW test Moisture, Video Type & CRC. */
  InitHWTests();
  /* Update the local copy of my serial number from configuration */
  UpdateLocalCopyOfMySerial();
  /*
   * Perform a power on controller board selftest suit
   * Now the configuration and FDE initalisation is complete,
   * check results of the full program code CRC test performed
   * previously.
   */
  ApplicationCodeCrcTest(CRC_BOOT_UPDATE_FAULT, true, true);
  /* Program data test (SRAM). Update fault state on fail */
  RamSelfTest(true);
  /* Verify the DCE checksum */
  DceChecksumSelftest(true);
  /* Basic check on the controller board ProductId */
  //CheckControlProductId(true);
  /* set DIP switch global variable and Interface type. */
 
  // InterfaceTypeSet();
 
  /* check for soak test */
  SoakTestConditionCheck(ACTUAL_POWERUP);
  /* IR remote setup */
  InitIr();
  InitIrRx();
  /* PrintRs485Header(); */
  /* prepare algo engine for start */
  InitProcess250Sensors();
  SetToNormalAtPowerUpPlusDisplay();
  /* check calibration */
  CalibrationCountCheck();
  /* Run the main application */
  // Temporarily monitoring interrupts
  // Turn Alarm LED on - function AlarmLED globally disabled
  PORTA|=(0x01U<<PA1);
  
  InterfaceTypeSet();

  // Ten second delay to accomodate sensors starting up
  delayms(10000U);
  
  MainApplication();
}


/* ////////// EOF ///////////// */
