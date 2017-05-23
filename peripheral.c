/***********************************************************************//**
* $Workfile < peripheral.c >
* @brief
*  This file contains some of very basic driver level functions for controlling
*  Alarm/fault leds, alarm/fault relays, EOL load, heater and so on.
* @ingroup
* $Used On:     FV400
* $Revision:    <>$
*
* Copyright © ThornSecurity.
***************************************************************************/
#define ENABLE_BIT_DEFINITIONS;
#include <iom1281.h>
#include <inavr.h>
#include <pgmspace.h>

#include "defs.h"
#include "spi.h"
#include "peripheral.h"
#include "configuration.h"
#include "hardwarefaultsentinel.h"
#include "hardwaretest.h"
#include "serviceprotocol.h"
#include "servicecommands.h"
#include "adc.h"
#include "library.h"
#include "watchdog.h"
#include "interruptcaptureandhwmonitor.h"
#include "debug.h"
#include "video.h"
#include "AFDPL15_IAR.h"
#include "s231interface.h"

/* Local function prototypes */
extern volatile BYTEu g_HWFaultState,g_timer4_20val;
extern BYTEu g_enablechecksum;
extern volatile INT16s g_TemperatureThmstr;
extern volatile BYTEu g_flag15v_on,g_flag15v_off;
static BYTEu g_almrelaystate;
static BYTEu g_ProductId;

/**********************************************************//**
*  @name InitProductId()
*  @brief
*  This function reads the ADC and identifies PCB id.  
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified g_ProductId
**************************************************************/
void InitProductId(void)
{
/*
 * int  temp =  __CPU__;
 * char  temp =  __CPU__;
 * "char const __farflash *" C:\Projects\S250\source\peripheral.c 69
 * Awaiting Dave R's update
 */
#if __CPU__  == 1281
  return;
#endif
  /* First read loads the shift register */
  HardwareGetProductId();
  /*
   * Since the product ID will never change and we may wish to
   * read the product ID during an SPI exchange and disabling the
   * level translator may have some unpredictable side effects,
   * the Product ID is copied and read into this static
   */
  g_ProductId = HardwareGetProductId();
}

/**********************************************************//**
*  @name HardwareGetProductId()
*  @brief
*  This is a supporting function for InitProductId.  
*  @param none
*  @return data
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
BYTEu HardwareGetProductId(void)
{
  BYTEu data;
  __delay_cycles(SPI_INIT_DELAY);
  /* Select the Product ID shift register and get byte value */
  SpiSelectDevice(SPI_PRODUCTID);
  data = SpiExchangeByte(0U);
  SpiSelectDevice(SPI_NONE);
  __delay_cycles(SPI_INIT_DELAY);
  return(data);
}

/**********************************************************//**
*  @name GetProductId()
*  @brief
*  Returns product ID to requesting function.  
*  @param none
*  @return none
*  globalvariablesused ProductId
*  globalvariablesmodified none
**************************************************************/
BYTEu GetProductId(void)
{
  return(g_ProductId);
}

/**********************************************************//**
*  @name GetSoftwareCompatibilityId()
*  @brief
*  This function read id bits for software compability.
*  @param none
*  @return none
*  globalvariablesused ProductId
*  globalvariablesmodified none
**************************************************************/
BYTEu GetSoftwareCompatibilityId(void)
{
  return(g_ProductId & 0x0FU);
}

/**********************************************************//**
*  @name VideoIsNtsc()
*  @brief
*  This function uses g_ProductId to identify NTSC camera. 
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
BYTEu VideoIsNtsc(void)
{
  return((GetProductId() >> 7U) & 0x01U);
}

/************************************************************//**
*  AlarmLED();
*  @brief
*  This function is switch for Alarm LED. True=ON,False=OFF.
*  @param AlarmLED_OnOff
*  @return none
*  globalvariablesused ALARM_LED
*  globalvariablesmodified none
****************************************************************/
void AlarmLED(BYTEu AlarmLED_OnOff)
{
  // Enable AlarmLED in defs.h
  if(ALARM_LED) 
  {
   if (AlarmLED_OnOff == true)
   {
     PORTA |= (0x01U << PA1); /* Alarm LED on */
     /* __no_operation(); */
   }
   else if (AlarmLED_OnOff == false)
   {
     PORTA &= ~(0x01U << PA1); /* Alarm LED off */
     /* __no_operation(); */
   }
   else{}
  }
 }

/************************************************************//**
*  AlarmLEDToggle(void);
*  @brief
*  This function toggles Alarm LED. 
*  @param none
*  @return none
*  globalvariablesused ALARM_LED
*  globalvariablesmodified none
****************************************************************/
void AlarmLEDToggle(void)
{
  // Enable AlarmLED in defs.h
  if(ALARM_LED)
  {
   PORTA ^= (0x01U << PA1);
  }
}

/************************************************************//**
*  ReadAlarmLED();
*  @brief
*  This function is switch for Alarm LED. True=ON,False=OFF.
*  @param AlarmLED_OnOff
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
BYTEu ReadAlarmLED(void)
{
  BYTEu retx = false; 
    
  /*PORTA&= ~(0x01U << PA1);
  DDRA &=~ (0x01U << DDA1);
  
  retx = PINA & ALM_LED_PIN;
  DDRA |= (0x01U << DDA1);*/
  
  retx = (0x01 & MxDigitalOutputRead());
  
  
  return(retx);
}

/************************************************************//**
*  AlarmRelay();
*  @brief
*  This function is switch for Alarm Relay. True=ON,False=OFF.
*  @param Relay_OnOff
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void AlarmRelay(BYTEu Relay_OnOff)
{
  PORTD &= ~(0x01U << PD0);  
  if(Relay_OnOff == true)
  {
   __no_operation();
  }
}

/************************************************************//**
*  ConvEOLTest();
*  @brief
*  This function is switch for EOL. True=ON,False=OFF.
*  @param EOLOnOff
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void ConvEOLTest(BYTEu EOLOnOff)
{
  DDRA |= (0x01U << DDA2);
  if (EOLOnOff == true)
  {
    PORTA |= (0x01U << PA2); /* EOL on */
  }
  else if (EOLOnOff == false)
  {
    PORTA &= ~(0x01U << PA2);
  }
  else{}
/* The EOL resistor is controlled by PA2; high = connected. */
}

/************************************************************//**
*  ConvAlarmControl();
*  @brief
*  This function is switch for conventional alarm. True=ON,False=OFF.
*  @param AlarmOnOff
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void ConvAlarmControl(BYTEu AlarmOnOff)
{
  DDRA |= (0x01U << DDA3);
  if (AlarmOnOff == true)
  {
    PORTA |= (0x01U << PA3); /* alarm load on */
    /* __no_operation(); */
  }
  else if (AlarmOnOff == false)
  {
    PORTA &= ~(0x01U << PA3);
  }
  else
  {}
/* The alarm load is applied via PA3; PA3 'high' applies the alarm load. */
}

/************************************************************//**
*  FaultRelay();
*  @brief
*  This function is switch for fault relay. True=ON,False=OFF.
*  @param FaultRelay_OnOff
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void FaultRelay(BYTEu FaultRelay_OnOff)
{
  PORTB &= ~(0x01U << PB4); /* Fault relay driver off */
   if(FaultRelay_OnOff == true)
  {
   __no_operation();
  }
}

/************************************************************//**
*  FaultLED();
*  @brief
*  This function is switch for fault LED. True=ON,False=OFF.
*  @param FaultLED_OnOff
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void FaultLED(BYTEu FaultLED_OnOff)
{
  DDRA |= (0x01U << DDA0);
  if (FaultLED_OnOff == true)
  {
    PORTA |= (0x01U << PA0); /* Fault LED on */
    /* __no_operation(); */
  }
  else if (FaultLED_OnOff == false)
  {
    PORTA &= ~(0x01U << PA0); /* Fault LED off */
  }
  else
  {}
  /* __no_operation(); */
}

/************************************************************//**
*  GainControl(BYTEu);
*  @brief
*  This function is switch for Gain control. True=ON,False=OFF.
*  @param setpinvl
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void GainControl(BYTEu setpinvl)
{
  DDRG |= (0x01U << PG0);
  if (setpinvl)
  {
    PORTG |= (0x01U << PG0);
  }
  else
  {
    PORTG &= ~(0x01U << PG0);
  }
}
/************************************************************//**
*  LampControl();
*  @brief
*  This function is switch for Lamps either for window or alarm. 
*  True=ON,False=OFF.
*  @param lampnstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void LampControl(BYTEu lampnstate)
{
  /* set pins as o/p */
  DDRC |= (0x01U << PC7);
  DDRB |= (0x01U << PB0);
  switch (lampnstate)
  {
  case ALL_LAMPS_OFF:
    PORTB &= ~(0x01U << PB0);
    PORTC &= ~(0x01U << PC7);
    break;
  case WINDOW_LAMP_OFF:
    PORTB &= ~(0x01U << PB0);
    break;
  case WINDOW_LAMP_ON:
    PORTB |= (0x01U << PB0);
    break;
  case ALARM_LAMP_OFF:
    PORTC &= ~(0x01U << PC7);
    break;
  case ALARM_LAMP_ON:
    PORTC |= (0x01U << PC7);
    break;
  default:
    /* all off */
    break;
  }
}

/************************************************************//**
*  GetAlarmRelayState();
*  @brief
*  This function reads alarm relay state. True=ON,False=OFF.
*  @param none
*  @return g_almrelaystate
*  globalvariablesused g_almrelaystate
*  globalvariablesmodified none
****************************************************************/
BYTEu GetAlarmRelayState(void)
{
  return(g_almrelaystate);
}

/************************************************************//**
*  MxAsicSwitch();
*  @brief
*  This function is switch for MX ASIC. True=ON,False=OFF.
*  @param mxswitch
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void MxAiscSwitch(BYTEu mxswitch)
{
  DDRB |= (0x01U << PB5); /* configure as o/p */
  if (mxswitch)
  {
    PORTB &= ~(0x01U << PB5);
  }
  else
  {
    PORTB |= (0x01U << PB5);
  }
}

/************************************************************//**
*  IrRcvCktSwitch();
*  @brief
*  This function is switch for IR RCV. True=ON,False=OFF.
*  @param swstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void IrRcvCktSwitch(BYTEu swstate)
{
  DDRG |= (0x01U << PG4);
  if (swstate)
  {
    PORTG &= ~(0x01U << PG4);
  }
  else
  {
    PORTG |= (0x01U << PG4);
  }
}

/************************************************************//**
*  SetPyroElectricalPin();
*  @brief
*  This function is switch for Electrical test pin. True=ON,False=OFF.
*  @param swstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void SetPyroElectricalPin(BYTEu swstate)
{
  DDRA |= (0x01U << PA4);
  if (swstate)
  {
    PORTA |= (0x01U << PA4);
  }
  else
  {
    PORTA &= ~(0x01U << PA4);
  }
}

/************************************************************//**
*  SetHartModem();
*  @brief
*  This function is switch for HART Modem. True=ON,False=OFF.
*  @param swstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void SetHartModem(BYTEu swstate)
{
  DDRG |= (0x01U << DDG3);
  if (swstate)
  {
    PORTG &= ~(0x01U << PG3);
  }
  else
  {
    PORTG |= (0x01U << PG3);
  }
}


/************************************************************//**
*  PowerUpIndication()
*  @brief
*  This function does powerup LED flashing. Boath Alarm and Fault
*  LEDs are flashed except for conventional or MX mode.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void PowerUpIndication(void)
{
  static BYTEu indicationdone = 0U;
  /* return if already done */
  if (indicationdone >= INDICATION_LIMIT)
  {
    return;
  }
  if ((GetServiceModeFlag() == true) || (PersonalityType() == CONVENT_MODE)
      || (PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE) 
          )
  {
    indicationdone++;
    return;
  }
  if (indicationdone == false)
  {
    // PORTA |= (0x01U << PA1);
    AlarmLED(true);
    PORTA &= ~(0x01U << PA0);
  }
  else if (indicationdone % 2U)
  {
    PORTA ^= (0x01U << PA0);
    AlarmLEDToggle();
    // PORTA ^= (0x01U << PA1);
  }
  else
  {}
  indicationdone++;
  if (indicationdone >= INDICATION_LIMIT)
  {
    // PORTA &= ~(0x01U << PA1);
    AlarmLED(false);
    PORTA &= ~(0x01U << PA0);
  }
}

/********************************************************************//**
 * Enable_4_20mA()
 * @brief
 * 10.12.1.3	Drivers Overview
 * The software shall have three functions
 * "	a function to enable and disable the 4-20mA interface
 * "	a function to set the 4-20mA current according to a passed 8-bit value
 * "	a function that is called periodically to check the 4-20mA monitor input.
 *
 * 10.12.1.4	Enabling and Disabling 4-20mA
 * The PWM signal is created from timer/counter 2 running in fast-PWM mode.
 * The signal appears on processor port PB4 from where it is changed to a DC level using a hardware-based filter
 * and then fed to the 4-20mA driver.
 *
 * (The PWM duty cycle is then set by writing an 8-bit value to OCR2A.
 * For information the PWM frequency is as follows:
 * (XTAL=3.6864x106 ) / [ (CLKPR = 1) x (TCCR20=1) x (256 counts) ] = 14400Hz  )
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
void Enable_4_20mA(void)
{
  printf_P("Calling Enable_4_20mA()");
  if (PersonalityType() != CONVENT_MODE) /* may 2011 */
  {   /* Config PB4 for output. */
    DDRB |= (0x01U << DDB7);
    /* Output-compare  interrupts need to be disabled,  so ensure that OCIE2A in TIMSK2 is zero. - TBC in next spec. */
    StopTimer0_IRQs();
    TCCR0B = 0U;
    TCCR0A = 0U;
    /* no prescaling, setup new clock speed @ 14Khz */
    TCCR0B |= (0x01U << CS00);
    TCCR0B &= ~(0x01U << CS01);
    TCCR0B &= ~(0x01U << CS02);
    /* set associated OC pin to toggle. */
    TCCR0A |= (0x01U << COM0A1);
    TCCR0A &= ~(0x01U << COM0A0);
    /* For fast PWM */
    TCCR0B &= ~(0x01U << WGM02);
    TCCR0A |= (0x01U << WGM01);
    TCCR0A |= (0x01U << WGM00);
    /* When enabling 4-20mA initialise OCR2A to zero. */
    OCR0A = 0U;
  }
  else
  {
    __no_operation();
  }
}

/************************************************************//**
*  Disable_4_20mA()
*  @brief
*  This function disables Timer 0 used for 4to20mA loopp.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
****************************************************************/
void Disable_4_20mA(void)
{
  /* disable timer 0 for 4to20 loop */
  TCCR0B = 0x00U;
  TCCR0A = 0x00U;
  /* force loop pin to 0 */
  DDRB |= (0x01U << DDB7);
  PORTB &= ~(0x01U << PB7);
}

/********************************************************************//**
 * Output_4_20mA_Level(level);
 * @brief
 * This function sets the output compair value for the timer 0
 * generated PWM signal for 4to20mA loop. Global variable g_timer4_20val
 * is used in health monitor function. One might ask why OCR0A is not
 * used directly in health monitor function global variable could be
 * avoided but OCR0A is used in some other places so.
 *
 * Any alarm signal will override the fault signal value.
 *
 * Future modification For hybrid band add feature for the 2ma window
 * fault/// Dec 2011
*  @param level
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
void Output_4_20mA_Level(BYTEu level)
{
  /*
   * update health monitor variable aswell
   * g_timer4_20val will tell HM function about the value change.
   */
  if (((g_HWFaultState == HARDWARE_FAULT) || (g_HWFaultState == WINDOWSCLEANLINESS)) && 
      (level < (SET_17mA - CONTINIOUS_MODE_COMP)) && (ReturnDebugMenuState() == false) && 
        (level != SET_4mAHART) && (GetServiceModeFlag() == false))
  {
    /* we are in some sort of fault and detector is not in alarm */
    if (((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == ONLY_4_20_DIS)) &&
        (GetEnhancedBandBit() == false))
    {
      g_timer4_20val = SET_FAULT_DEFAULT;
      OCR0A = SET_FAULT_DEFAULT;

    }
    else if ((((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == ONLY_4_20_DIS)) && (GetEnhancedBandBit())) ||
             ((PersonalityType() == ONLY_4_20_VAR) || (PersonalityType() == DEFVAR_4_20MA_MODE)))
    {
      /* use zero for fault and 2mA for window dirty for variable and enhanced discrete bands */
      if (g_HWFaultState == HARDWARE_FAULT)
      {
        g_timer4_20val = FAULT_LOOP_LVL;
        OCR0A = FAULT_LOOP_LVL;
      }
      else if (g_HWFaultState == WINDOWSCLEANLINESS)
      {
        g_timer4_20val = SET_WINDOWFAULT;
        OCR0A = SET_WINDOWFAULT;
      }
      else
      {}
    }
    else
    {}
  }
  else
  {
    g_timer4_20val = level;
    OCR0A = level;
  }
/*  printf_P("\rlvl=%d",level); */
}

/**********************************************************//**
*  StopTimer0_IRQs() 
*  @brief
*  Stops timer 0 interrupt.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void StopTimer0_IRQs(void)
{
  __disable_interrupt();
  /* Set Output compare interrupt disable on compare register A */
  TIMSK0 &= ~(0x01U << OCIE0A);
  __enable_interrupt();
}

/**********************************************************//**
*  Timer3MXPWMEnable();
*  @brief
*  1- Configure TIMER3 in normal mode.
*  2- Calculate OCR3x=fclk_I/O /(2 . N . foutx)
*  3- Save these 3 calculated values in 3 words.
*  4- Configure all OC3x outputs to "Toggle OCnA/OCnB/OCnC on Compare Match" (Non-PWM mode).
*  5- Activate all 3 compare match interrupts.
*  6- In each interrupt routine, add the related saved value to current OCR3x.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void Timer3MXPWMEnable(void)
{
  /* set pins as o/p */
  DDRE |= (0x01U << DDE4);
  DDRE |= (0x01U << DDE5); /* set as o/p */
  __disable_interrupt();
  TCCR3A = 0U; /* clear first */
  TCCR3A |= (0x01U << WGM30) | (0X01U << COM3C1) | (0X01U << COM3B1);
  TCCR3B |= (0X01U << WGM32) | (0x01U << CS30);
  /* TIMSK3 |=(0x01<<OCIE3C); */
  TCNT3 = 0U;
/* TCCR3B|=;//same prescl factor is used as in timer 2 for 4_20loop. */
  OCR3B = 0U; /* PE4 comp reg */
  OCR3C = 0U; /* PE5 comp reg */
  /* TOP=OCR3C; */
  __enable_interrupt();
}


/********************************************************************//**
 * void OutputMX_4_20mALevel()
 * @brief
 * This function will set the PWM value for timer 3 controlling the
 * MX ASIC. Levels are different than the default 4to20mA loop.
 * Alarm signal will override any fault signal but pre alarm won't.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
void OutputMX_4_20mALevel(BYTEu level)
{

  if ((g_HWFaultState == HARDWARE_FAULT) && (level < (MX_17mA - MX_LVL_OFFSET))
      && (ReturnDebugMenuState() == false))
  {
    OCR3C = MX_FAULT; /* PE5 comp reg */
  }
  else if ((g_HWFaultState == WINDOWSCLEANLINESS) && (level < (MX_17mA - MX_LVL_OFFSET))
           && (ReturnDebugMenuState() == false))
  {
    OCR3C = MX_WINFLT; /* PE5 comp reg */
  }
  else
  {
    OCR3C = level; /* PE5 comp reg */
  }
}

/******************************************************************//**
 * AlarmSignallingHandlerAlgo()
 * @brief
 * Depending on the type of personality this function is responsible
 * for alarm signalling. This function is called in under the "process_totals"
 * within the algorithm engine. If alarm is in non latching then there is
 * a 3 sec hangover delay.
 * As this function is responsible of reporting an alarm event so
 * must be changed carefully.
 * Sep 14 2011
*  @param almstatus,latchstatus,prealarmstatus
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***********************************************************************/
void AlarmSignallingHandlerAlgo(BYTEu almstatus,BYTEu latchstatus,
                                BYTEu prealarmstatus)
{
  static BYTEu fdealarmlog = false,alarm_hangover = 0U,vidindication = false,
                       powerupstatus = false;

  if (almstatus == alarm)
  {
    alarm_hangover = TagToConfigReadChar(ALARM_HANGOVER); /* 3 seconds atleast; */
    /* Hold alarm for alarm_hangover seconds if no latching */
    SetInternalAlarmState(STATUS_ALARM,REQ_LOOP_TEST);
    /* set alarm led ON if not in MX */
    if ((PersonalityType() != MX_IFACE_MODE) && (PersonalityType() != MX_IFACE_DIPSW_ACTIVE))
    {
      AlarmLED(true);
    }
    /* followinf if else ladder handles alarm reporting depending on the interface type */
    if ((PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE))
    {
      Timer3MXPWMEnable();
      OutputMX_4_20mALevel(MX_17mA);      /* high state = alarm on. */
      //printfv(8U,9U,vidindication,"!!ALARM!!");      /* NEW JAN 2011 */
    }
    else if (PersonalityType() == CONVENT_MODE)
    {
      ConvAlarmControl(true);
    }
    else if (PersonalityType() == DEFAULT_4_20MA_MODE)
    {
      /* this is default mode */
      AlarmRelay(true);
      Enable_4_20mA();
      Output_4_20mA_Level(SET_17mA);      /* high state = alarm on. */
      //printfv(8U,9U,vidindication,"!!ALARM!!");      /* NEW JAN 2011 */
    }
    else if (PersonalityType() == DEFVAR_4_20MA_MODE)
    {
      AlarmRelay(true);
      //printfv(8U,9U,vidindication,"!!ALARM!!");
    }
    else if (PersonalityType() == ONLY_4_20_DIS)
    {
      Enable_4_20mA();
      Output_4_20mA_Level(SET_17mA);      /* high state = alarm on. */
      //printfv(8U,9U,vidindication,"!!ALARM!!");      /* NEW JAN 2011 */
    }
    else
    {}

    /* log to FDE */
    if (fdealarmlog == false)
    {
      fdealarmlog = true;
      FdeAlarmLog(STATUS_NORMAL,STATUS_ALARM,g_TemperatureThmstr / CONVERT_TO_DEG);
    }
    /* flash video */
    if (vidindication == false)
    {
      vidindication = true;
    }
    else
    {
      vidindication = false;
    }
    /*
     * vidindication = !vidindication;
     * /OUT OF ALARM HERE
     */
  }
  else if ((latchstatus == false) || (almstatus == reset) || (powerupstatus == false)
           || (PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE)
           || (PersonalityType() == DEFVAR_4_20MA_MODE) || (PersonalityType() == ONLY_4_20_VAR))
  {
    powerupstatus = true;
    if ((alarm_hangover != false) && (almstatus != reset) && (GetOverRideStatus() == false))  /* ignore hangover if reset requested */
    {
      alarm_hangover--;
    }
    else
    {
      /* don't touch LED in MX mode */
      if ((PersonalityType() != MX_IFACE_MODE) && (PersonalityType() != MX_IFACE_DIPSW_ACTIVE))
      {
        AlarmLED(false);
      }

      /* safe to set following regardless */
      AlarmRelay(false);
      SetInternalAlarmState(STATUS_NORMAL,REQ_LOOP_TEST);
      /* for s231 */
      ConvAlarmControl(false);
      /* return here if conv more */
      if (PersonalityType() == CONVENT_MODE)
      {
        /* log to FDE */
        if (fdealarmlog)
        {
          fdealarmlog = false;
          FdeAlarmLog(STATUS_ALARM,STATUS_NORMAL,g_TemperatureThmstr / CONVERT_TO_DEG);
        }
        return;
      }

      if ((PersonalityType() != MX_IFACE_MODE) && (PersonalityType() != MX_IFACE_DIPSW_ACTIVE))
      {
        /* clear video */
        //printfv(8U,9U,false,"         ");
      }
      else if(ReadAlarmLED() == false)
      {
       /* only clear if led is clear for MX mode */
       // printfv(8U,9U,false,"         ");
      }
      else{}
      
      /* for 4_20 mode */
      if ((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == ONLY_4_20_DIS))
      {

        if (prealarmstatus == false)
        {
          Enable_4_20mA();
          Output_4_20mA_Level(SET_NORMAL);         /* low state = alarm off. */
        }
        /* for MX */
      }
      else if ((PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE))
      {
        if (GetPreAlarmState() == false)
        {
          Timer3MXPWMEnable();
          OutputMX_4_20mALevel(MX_4mA);
        }
      }
      else
      {}
      /* log to FDE */
      if (fdealarmlog != false)
      {
        fdealarmlog = false;
        FdeAlarmLog(STATUS_ALARM,STATUS_NORMAL,g_TemperatureThmstr / CONVERT_TO_DEG);
      }
    }
  }
  else if ((latchstatus != false) &&
           (GetDetectorStatus()->AlarmState != STATUS_ALARM))
  {
    /*if detector is set for alarm latching then output4_20func
     * will not be called and this function is responsible for setting the
     * fault current so call it here.*/
    if ((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == ONLY_4_20_DIS))
    {
      /* set normal but this will set fault it self within the rutine. */
      Output_4_20mA_Level(SET_NORMAL);
    }
  }
  else
  {}
}

/******************************************************************//**
* FaultLedStateMachine();
* @brief
*  Fault Led flashing pattern is generated depending on current 
*  detector status. 
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified g_flag15v_on,g_flag15v_off
 ***********************************************************************/
void FaultLedStateMachine(void)
{
  static BYTEu prevstate = false;
  static INT16u delaycount = 0U;

  if (GetOverRideStatus() == FAULT_LED_FLASH)
  {
    delaycount++;
    if (prevstate == false)
    {
      prevstate = true;
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0);  /* led ON */
    }
    else if (delaycount == CONVMSDEL)
    {
      PORTA &= ~(0x01U << PA0);    /* off */
      g_flag15v_off = true;
    }
    else if (delaycount >= ONESECONDONLY)
    {
      prevstate = false;
      delaycount = 0U;
    }
    else
    {}
    return;
  }
  /* this flag is meant for calib indications for SMP */
  if (GetOverRideStatus() == true)
  {
    return;
  }

  DDRA |= (0x01U << DDA0); /* set fault led pin as O/P */
  /* chk for SMP */
  if ((GetServiceModeFlag() == true) || (GetConfigFaultState() != false) ||
      ((PersonalityType() == MX_IFACE_MODE) && (GetEnhancedBandBit() == true)))
  {
    delaycount++;
    if (prevstate == false)
    {
      prevstate = true;
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0); /* led ON */
      if (GetServiceModeFlag())
      {
        // PORTA |= (0x01U << PA1);
        AlarmLED(true);
      }
    }
    else if (delaycount == CONVMSDEL)
    {
      PORTA &= ~(0x01U << PA0); /* off */
      if (GetServiceModeFlag())
      {
        // PORTA &= ~(0x01U << PA1);
        AlarmLED(false);
      }
      g_flag15v_off = true;
    }
    else if (delaycount == HUNDREDMS + CONVMSDEL) /* 13 */
    {
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0); /* led ON */
      if (GetServiceModeFlag())
      {
        // PORTA |= (0x01U << PA1);
        AlarmLED(true);
      }
    }
    else if (delaycount == HUNDREDMS + CONVMSDEL + 2U)
    {
      PORTA &= ~(0x01U << PA0); /* off */
      g_flag15v_off = true;
      if (GetServiceModeFlag())
      {
        // PORTA &= ~(0x01U << PA1);
        AlarmLED(false);
      }
    }
    else if (delaycount >= FIVESECONDS)    /* 5sec */
    {
      prevstate = false;
      delaycount = 0U;
    }
    else
    {}
    return;
  }
  /* no SMP */
  switch (g_HWFaultState)
  {
  case NO_FAULT:
    PORTA &= ~(0x01U << PA0);   /* led OFF */
    break;

  case WINDOWSCLEANLINESS:
    delaycount++;
    if (prevstate == false)
    {
      prevstate = true;
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0); /* led ON */
      /* FaultRelay(true); */
    }
    else if (delaycount == CONVMSDEL)
    {
      PORTA &= ~(0x01U << PA0); /* off */
      g_flag15v_off = true;
      /* FaultRelay(false); */
    }
    else if (delaycount == TWOHUNDREDMS)
    {
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0); /* led ON */
    }
    else if (delaycount == TWOHUNDREDMS + CONVMSDEL)
    {
      PORTA &= ~(0x01U << PA0); /* off */
      g_flag15v_off = true;
    }
    else if (delaycount == FIVESECONDS)  /* 5sec */
    {
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0); /* led ON */
    }
    else if (delaycount == FIVESECONDS + CONVMSDEL)
    {
      PORTA &= ~(0x01U << PA0); /* off */
      g_flag15v_off = true;
    }
    else if (delaycount >= 1000U)
    {
      prevstate = false;
      delaycount = 0U;
    }
    else
    {}
    break;

  case HARDWARE_FAULT:
    delaycount++;
    if (prevstate == false)
    {
      prevstate = true;
      g_flag15v_on = true;
      PORTA |= (0x01U << PA0);  /* led ON */
    }
    else if (delaycount == CONVMSDEL)
    {
      PORTA &= ~(0x01U << PA0);    /* off */
      g_flag15v_off = true;
    }
    else if (delaycount >= FIVESECONDS)
    {
      prevstate = false;
      delaycount = 0U;
    }
    else
    {}
    break;

  default:
    break;
  }
}

/**********************************************************//**
*  InitJtagInterface() 
*  @brief
*  Not part of main loop, only used during development for JTAG
*  debugging.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
#ifdef JTAGS_ON
void InitJtagInterface(void){}
#else
void InitJtagInterface(void)
{

  /*
   * Check configuration. Disable JTAG if not used.
   * ALWAYS disable JTAG in release versions of the code
   */
  // if (g_enablechecksum == true)
// {
    /* We dont use JTAG. Disable it. */

    __disable_interrupt();
/*
 * #ifdef __ATmega128__
 *    MCUCSR |= (0x01<<JTD);
 *    MCUCSR |= (0x01<<JTD); // must be written to twice within 4 clocks
 *
 * #elif __ATmega1281__
 * //Build for 1281:
 */
    MCUCR |= (0x01U << JTD);
    MCUCR |= (0x01U << JTD); /* must be written to twice within 4 clocks */
    __enable_interrupt();

 // }
}
#endif

/**********************************************************//**
*  KickWdtMain() 
*  @brief
*  There is an external watchdog connected to port PA5 (from PD4) which if not pulsed at least every second will reset the processor.
*  It is suggested that the watchdog routine is part of the top level poll loop and operates according to a counter that
*  is incrimented by the 10msec interrupt routine. Therefore if either the poll loop gest stuck or the 10msec interrupts
*  don't happen then the watchdog will trip.
*  Note that during the ADC loop-back test (see section 7.6.2.5) the 10msec interrupts are disabled and so this test function
*  shall need to pulse the watchdog locally to stop the processor from being reset.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void KickWdtMain(void)
{
  if ((PersonalityType() != CONVENT_MODE) && (PersonalityType() != MX_IFACE_MODE) && 
      (PersonalityType() != MX_IFACE_DIPSW_ACTIVE))
  {
    WATCHDOG_MACRO   /* Toggles PORTA / PA5 */
  }
  else
  {
    if(PersonalityType() == CONVENT_MODE)
    {
     StatusOfFaultPulsesOnPowerLine();
    }
    
    /* Watchdog controlled */
    if (GetMxSmpStatus() == false)
    {
    ControlledKickWdt(false);
    }
    
    SleepModeActive(MAIN_LOOP);
   
    if (GetMxSmpStatus() == false)
    {
    ControlledKickWdt(true);
    }
   
  }
}

/**********************************************************//**
*  KickWdt() 
*  @brief
*  Toggles the watchdog pin.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void KickWdt(void)
{
  WATCHDOG_MACRO     /* Toggles PORTA / PA5 */
}


/**********************************************************//**
*  ControlledKickWdt() 
*  @brief
*  Controlled watchdogging for testing.
*  @param hilo
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void ControlledKickWdt(BYTEu hilo)
{
  if (hilo)
  {
    PORTA |= (0x01U << PA5);
  }
  else
  {
    PORTA &= ~(0x01U << PA5);
  }
}

/**********************************************************//**
*  InitTestPoint() 
*  @brief
*  Only for testing not part of main loop.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void InitTestPoint(void)
{
  /* Setup PB4 as output */
  DDRB |= (0x01U << DDB4);
  /* Set low */
  PORTB &= ~(0x01U << PB4);
}

/* /////////////////////// EOF ///////////////////////// */
