/***********************************************************************//**
* @file  calibration.c
* @brief Functions associated with calibration of flame pyros are defined
* within tis file. 
*  Used On:     FV400
***************************************************************************/

#define ENABLE_BIT_DEFINITIONS
#include <iom1281.h>
#include <inavr.h>
#include <inttypes.h>

#include "library.h"
#include "defs.h"
#include "peripheral.h"
#include "adc.h"
#include "AFDPL15_IAR.h"
#include "interruptcaptureandhwmonitor.h"
#include "debug_250.h"
#include "hardwarefaultsentinel.h"
#include "hardwaretest.h"
#include "sprogserial.h"
#include "configuration.h"
#include "video.h"
#include "fde.h"
#include "spi.h"
#include "uart.h"
#include "panelcomms.h"
#include "s231interface.h"
#include "serviceprotocol.h"
#include "servicecommands.h"
#include "scememory.h"

#define cal43 0x0248000000
#define cal38  0x0522000000

extern volatile INT16u g_SamplesReady;
extern volatile unsigned char g_TimerFlbias;
extern volatile unsigned char g_TimerGubias;
extern volatile unsigned char g_TimerFlame0;
extern volatile unsigned char g_TimerFlame1;
extern volatile unsigned char g_TimerGuard;

static unsigned char calibrationinprogress = false,GainCapture;
static int flamaxi = 0,flamini = 255,flbmaxi = 0,flbmini = 255;
static int gumaxi = 0,gumini = 255;

signed int CAL_FL_A_avg = 17792; /* Flame channel A */
signed int CAL_FL_B_avg = 17792; /* Flame channel B */
signed int CAL_GU_avg = 17792; /* Guard channel */
signed int CalibBiaslta = 17792; /* ??? new addtion */

static __INT64_T_TYPE__ capturefla;
static __INT64_T_TYPE__ captureflb;
static __INT64_T_TYPE__ capturegu;

unsigned char CAL_primhga; /* Holder for calibration value */
unsigned char CAL_primlga; /* Holder for calibration value */
unsigned char CAL_primlgb; /* Holder for calibration value */
unsigned char CAL_primhgb = 10U; /* Holder for calibration value */
unsigned char CAL_sechg = 10U; /* Holder for calibration value */
unsigned char CAL_seclg; /* Holder for calibration value */

static unsigned char calibrationinprogress,tempcalflah,tempcalflal,
                     tempcalflbh,tempcalflbl,tempcalflgh,tempcalflgl,
                     GainCapture;

INT16s CAL_FL_A_x4p3a;  /* RMB     2          Primary_Signal_A */
INT16s CAL_FL_B_x4p3b;  /* RMB     2          Primary_Signal_B */
INT16s CAL_GU_x3p8;   /* RMB */ 

__INT64_T_TYPE__ CAL_eauto = 0;

/**********************************************************//**
*  @name F()
*  @brief
*  Description: This function manages the following:
*  
*  @param none
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void DoAllCalibrate250Sensors_2ch(unsigned char reqtyp)
{
  calibrationinprogress = true;
  GainCapture = false;
  INT16u calib_loop;
  INT16u flame_low, flame_high, guard_low, guard_high;
  /* reset maxi minima */
  flamaxi = 0;
  flbmaxi = 0;
  gumaxi = 0;
  flamini = 255;
  flbmini = 255;
  gumini = 255;
  CAL_GU_avg = 17792;
  /*
   * CAL_FL_A_avg=17792;
   * CAL_FL_B_avg=17792;
   * Set 10ms timer
   */
  StartTimer2_10msMode();
  /* set HW flag indicator for SMP calib request */
  if ((reqtyp != REQ_DEBUG) && (reqtyp != REQ_DEBUG_VERIFY))
  {
    // PORTA |= (0x01U << PA1);
    AlarmLED(true);
  }
  /* clear static 64 bit variables */
  capturefla = 0;
  captureflb = 0;
  capturegu = 0;

  KickWdt();
  if (reqtyp == REQ_DEBUG_VERIFY)
  {
    printf_P("\r*Verification only memory write disabled Wait*\r");
  }
  /*
   * Calibration start collect samples, it will take 10sec approx
   * accumulate 255*4=1020 samples
   */
  for (calib_loop = 1U; calib_loop <= 255U * 4U; ++calib_loop)
  {
    while (g_SamplesReady == 0U)
    {}                          /* wait.. */
    Calibrate250Sensors(ALL_IN_ONE_GO);
    /* wait for next valid sample:- */
    g_SamplesReady = 0U;
    KickWdt();
    if (calib_loop % 127U == 0U) /* indicate we're working - takes some seconds */
    {     /* print an asterix ~ once every sec. */
      if (reqtyp == REQ_DEBUG)
      {
        printf_P("*");
      }
      if (reqtyp == REQ_DEBUG_VERIFY)
      {
        printf_P("-");
      }
    }
  }
  /* Samples accumulated calculation now print stuff */

  /* Print for FL A */
  KickWdt();
  if ((reqtyp == REQ_DEBUG) || (reqtyp == REQ_DEBUG_VERIFY))
  {
    printf_P("\r\n");   /* CR after asterixes */
    printf_P("Flame A Sum = ");
    My64bitHexPrint(capturefla);
    printf_P("  ");
    My64bitDecimalPrint(capturefla);
    printf_P("\r");
    printf_P("MAX=%d,MIN=%d\r\n",flamaxi,flamini);
  }
  KickWdt();
  /* sqrt max vals:- */
  flame_low = (INT16u)MySqrt(cal43, capturefla);
  /* perform check */
  if ((reqtyp != REQ_SMP_VERIFY) && (reqtyp != REQ_CAPTURE)
      && (reqtyp != REQ_DEBUG_VERIFY))
  {
    if ((flame_low >= 200U) || (flame_low <= 100U))
    {
      /* flag it */
      if (flame_low >= 200U)
      {
        TagToDceWriteChar(FLMACALFLG,TOO_HIGH);
      }
      else
      {
        TagToDceWriteChar(FLMACALFLG,TOO_LOW);
      }
    }
    else
    {
      TagToDceWriteChar(FLMACALFLG,CALIBRATED);
    }
    /* increment calib count only once */
    if (TagToConfigReadChar(CALIBCOUNT) == 255U)
    {
      TagToDceWriteChar(CALIBCOUNT,1U);
    }
    else
    {
      TagToDceWriteChar(CALIBCOUNT,(TagToConfigReadChar(CALIBCOUNT) + 1U));
    }
    /* copy 1 wire crc to controller board */
    KickWdt();
    __disable_interrupt();
    if (ReadROMOfSCE(REQ_SMP_TEST))
    {
      StoreOneWireCrc();
    }
    else
    {
      /* add 16 to flA flag */
      TagToDceWriteChar(FLMACALFLG,TagToConfigReadChar(FLMACALFLG) + 16U);
    }
    __enable_interrupt();
  }
  /* Add 1/2 to round up if needed. */
  flame_high = flame_low / 16U;
  KickWdt();
  if ((flame_low % 16U) > 7U)
  {
    flame_high = flame_high + 1U;
  }
  if ((reqtyp == REQ_DEBUG) || (reqtyp == REQ_DEBUG_VERIFY))
  {
    printf_P("Channel 0 low = 0x%x  (%d)\r\n", flame_low, flame_low);
    printf_P("Channel 0 high = 0x%x  (%d)\r\n", flame_high, flame_high);
  }
  if ((reqtyp != REQ_SMP_VERIFY) && (reqtyp != REQ_CAPTURE)
      && (reqtyp != REQ_DEBUG_VERIFY))
  {
    TagToDceWriteChar(DFLT_PRIMLGA,(unsigned char)flame_low);
    TagToDceWriteChar(DFLT_PRIMHGA,(unsigned char)flame_high);
    CAL_primhga = (unsigned char)flame_high;
    CAL_primlga = (unsigned char)flame_low;
  }
  else if (reqtyp == REQ_SMP_VERIFY)
  {
    tempcalflal = (unsigned char)flame_low;
    tempcalflah = (unsigned char)flame_high;
  }
  else
  {}

  /* Print for FL B */
  KickWdt();
  if ((reqtyp == REQ_DEBUG) || (reqtyp == REQ_DEBUG_VERIFY))
  {
    printf_P("\r\n");   /* CR after asterixes */
    printf_P("Flame B Sum = ");
    My64bitHexPrint(captureflb);
    printf_P("  ");
    My64bitDecimalPrint(captureflb);
    printf_P("\r");
    printf_P("MAX=%d,MIN=%d\r\n",flbmaxi,flbmini);
  }
  KickWdt();
  /* sqrt max vals:- */
  flame_low = (INT16u)MySqrt(cal43, captureflb);
  /* perform check */
  if ((reqtyp != REQ_SMP_VERIFY) && (reqtyp != REQ_CAPTURE)
      && (reqtyp != REQ_DEBUG_VERIFY))
  {
    if ((flame_low >= 200U) || (flame_low <= 100U))
    {
      /* flag it */
      if (flame_low >= 200U)
      {
        TagToDceWriteChar(FLMBCALFLG,TOO_HIGH);
      }
      else
      {
        TagToDceWriteChar(FLMBCALFLG,TOO_LOW);
      }
      /*
       * set default
       * flame_low=134;
       */
    }
    else
    {
      TagToDceWriteChar(FLMBCALFLG,CALIBRATED);
    }
  }
  /* Add 1/2 to round up if needed. */
  flame_high = flame_low / 16U;
  KickWdt();
  if ((flame_low % 16U) > 7U)
  {
    flame_high = flame_high + 1U;
  }
  if ((reqtyp == REQ_DEBUG) || (reqtyp == REQ_DEBUG_VERIFY))
  {
    printf_P("Channel 1 low = 0x%x  (%d)\r\n", flame_low, flame_low);
    printf_P("Channel 1 high = 0x%x  (%d)\r\n", flame_high, flame_high);
  }
  if ((reqtyp != REQ_SMP_VERIFY) && (reqtyp != REQ_CAPTURE)
      && (reqtyp != REQ_DEBUG_VERIFY))
  {
    TagToDceWriteChar(DFLT_PRIMLGB,(unsigned char)flame_low);
    TagToDceWriteChar(DFLT_PRIMHGB,(unsigned char)flame_high);
    CAL_primhgb = (unsigned char)flame_high;
    CAL_primlgb = (unsigned char)flame_low;
  }
  else if (reqtyp == REQ_SMP_VERIFY)
  {
    tempcalflbl = (unsigned char)flame_low;
    tempcalflbh = (unsigned char)flame_high;
  }
  else
  {}

  /* Print for GU */
  KickWdt();
  if ((reqtyp == REQ_DEBUG) || (reqtyp == REQ_DEBUG_VERIFY))
  {
    printf_P("\r\n");   /* CR after asterixes */
    printf_P("Guard Sum = ");
    My64bitHexPrint(capturegu);
    printf_P("  ");
    My64bitDecimalPrint(capturegu);
    printf_P("\r");
    printf_P("MAX=%d,MIN=%d\r\n",gumaxi,gumini);
  }

  KickWdt();
  guard_low = (INT16u)MySqrt(cal38, capturegu);
  /* perform check */
  if ((reqtyp != REQ_SMP_VERIFY) && (reqtyp != REQ_CAPTURE)
      && (reqtyp != REQ_DEBUG_VERIFY))
  {
    if ((guard_low >= 200U) || (guard_low <= 100U))
    {
      /* flag it */
      if (guard_low >= 200U)
      {
        TagToDceWriteChar(GDCALFLG,TOO_HIGH);
      }
      else
      {
        TagToDceWriteChar(GDCALFLG,TOO_LOW);
      }
      /*
       * set default
       * guard_low=140;
       */
    }
    else
    {
      TagToDceWriteChar(GDCALFLG,CALIBRATED);
    }
  }
  guard_high = guard_low / 16U;
  KickWdt();
  if ((guard_low % 16U) > 7U)
  {
    guard_high = guard_high + 1U;
  }

  if ((reqtyp == REQ_DEBUG) || (reqtyp == REQ_DEBUG_VERIFY))
  {
    printf_P("Channel 2  low = 0x%x  (%d)\r\n", guard_low, guard_low);
    printf_P("Channel 2  high = 0x%x  (%d)\r\n", guard_high, guard_high);
  }
  if ((reqtyp != REQ_SMP_VERIFY) && (reqtyp != REQ_CAPTURE)
      && (reqtyp != REQ_DEBUG_VERIFY))
  {
    TagToDceWriteChar(DFLT_SECLG,(unsigned char)guard_low);
    TagToDceWriteChar(DFLT_SECHG,(unsigned char)guard_high);
    UpdateChecksum();
    CAL_sechg = (unsigned char)guard_high; /* TAJ was flame_high; */
    CAL_seclg = (unsigned char)guard_low; /* TAJ was flame_low; */
  }
  else if (reqtyp == REQ_SMP_VERIFY)
  {
    tempcalflgl = (unsigned char)guard_low; /* TAJ was flame_low; */
    tempcalflgh = (unsigned char)guard_high; /* TAJ was flame_high; */
  }
  else
  {}

  KickWdt();
  /* record gain during capture chech gain switch pin PG0 */
  if (PORTG & GAIN_PING0)
  {
    GainCapture = true;
  }
  else
  {
    GainCapture = false;
  }
  /* load calib values from eeprom regardless */
  KickWdt();
  CAL_primlga = TagToConfigReadChar(DFLT_PRIMLGA);
  CAL_primlgb = TagToConfigReadChar(DFLT_PRIMLGB);
  CAL_seclg = TagToConfigReadChar(DFLT_SECLG);
  CAL_primhga = TagToConfigReadChar(DFLT_PRIMHGA);
  CAL_primhgb = TagToConfigReadChar(DFLT_PRIMHGB);
  CAL_sechg = TagToConfigReadChar(DFLT_SECHG);
  /* set high gain */
  SetHigh();
  /* relese the indicator for SMP */
  if ((reqtyp != REQ_DEBUG) && (reqtyp != REQ_DEBUG_VERIFY))
  {
    //PORTA &= ~(0x01U << PA1);
    AlarmLED(false);
  }
  /* end of calibration */
  calibrationinprogress = false;
}

/**********************************************************//**
* MOVE TO CALIBRATION FILE??
* NEED TO WORK OUT WHICH CALIBRATION FUNCTIONS ARE USED AND ADD COMMENTS??
*  @name Calibrate250Sensors
*  @brief
*  Calibrate flame and guard(?) channels. Called from SMP or menu system. 
*  Data captured from pyros by 10mS interrupt handler.
*  
*  @param calib_type
*  @return none
*
*  globalvariablesused none?
*  globalvariablesmodified none?
**************************************************************/
void Calibrate250Sensors(BYTEu calib_type)
{
  signed char lcldrift = 0;
  int tempbiasltacalc = 0;
  int tempsignalval = 0;
  //TotalSamples++;
  g_SamplesReady = 1U;        /* New method of access & calcs. for ProcessSensorReadings(); */

  if (g_SamplesReady)
  {
    /* set variables for avg */
    CAL_FL_A_x4p3a = (INT16s)g_TimerFlame0;
    CAL_FL_B_x4p3b = (INT16s)g_TimerFlame1;
    CAL_GU_x3p8 = (INT16s)g_TimerGuard;

    if (calib_type == ALL_IN_ONE_GO)
    {
      /*
       * switch (calib_type){
       *  case ALL_IN_ONE_GO:
       * do avg first
       */
      CalCalculateAvg();
      /*
       * printf_P("adcgu=%d,Gavg=%d\r",TimerGuard,((CAL_GU_avg+64)/128));
       * return;
       */

      /* #### ch 0 #### */
      lcldrift = 0;
      lcldrift = (signed char)(((CAL_FL_A_avg + 64) / 128) - (signed int)g_TimerFlbias);
      CalibrationInitProcs();
      CAL_eauto = capturefla;

      tempsignalval = (int)((int)g_TimerFlame0 - (int)g_TimerFlbias - lcldrift);

      CAL_FL_A_x4p3a = tempsignalval;
      CAL_FL_B_x4p3b = CAL_FL_A_x4p3a;
      if (flamaxi <= tempsignalval)
      {
        flamaxi = tempsignalval;
      }
      else if (flamini >= tempsignalval)
      {
        flamini = tempsignalval;
      }
      else
      {}

      CalibrationReadSamples();            /* from c_read */
      CAL_eauto += (__INT64_T_TYPE__)((long)CAL_FL_A_x4p3a * (long)CAL_FL_B_x4p3b);
      /* now copy e auto */
      capturefla = CAL_eauto;
      /* reset drift */
      lcldrift = 0;

      /* ##### ch 1 ###### */
      lcldrift = 0;
      lcldrift = (signed char)(((CAL_FL_B_avg + 64) / 128) - (signed int)g_TimerFlbias);
      CalibrationInitProcs();
      CAL_eauto = captureflb;

      tempsignalval = (int)((int)g_TimerFlame1 - (int)g_TimerFlbias - lcldrift);

      CAL_FL_A_x4p3a = tempsignalval;
      CAL_FL_B_x4p3b = CAL_FL_A_x4p3a;
      if (flbmaxi <= tempsignalval)
      {
        flbmaxi = tempsignalval;
      }
      else if (flbmini >= tempsignalval)
      {
        flbmini = tempsignalval;
      }
      else
      {}

      CalibrationReadSamples();           /* from c_read */
      CAL_eauto += (__INT64_T_TYPE__)((long)CAL_FL_A_x4p3a * (long)CAL_FL_B_x4p3b);
      /* now copy e auto */
      captureflb = CAL_eauto;

      /* #### ch guard #### */
      lcldrift = 0;
      lcldrift = (signed char)(((CAL_GU_avg + 64) / 128) - (signed int)g_TimerFlbias);
      CalibrationInitProcs();
      CAL_eauto = capturegu;

      tempsignalval = (int)((int)g_TimerGuard - (int)g_TimerFlbias - lcldrift);

      CAL_FL_A_x4p3a = tempsignalval;
      CAL_FL_B_x4p3b = CAL_FL_A_x4p3a;

      /* calculate maxi minima */
      if (gumaxi <= tempsignalval)
      {
        gumaxi = tempsignalval;
      }
      else if (gumini >= tempsignalval)
      {
        gumini = tempsignalval;
      }
      else
      {}

      /*
       * take LTA for BIAS
       * LTA*128 = LTA*128 + High_Byte((Sample * 128 - LTA*128) + 128)
       */
      tempbiasltacalc = ((int)g_TimerFlbias * 128 - CalibBiaslta) + LTAroundingfactor;
      CalibBiaslta = CalibBiaslta + (tempbiasltacalc >> 8);

      CalibrationReadSamples();     /* from c_read */
      CAL_eauto += (__INT64_T_TYPE__)((long)CAL_FL_A_x4p3a * (long)CAL_FL_B_x4p3b);
      /* now copy e auto */
      capturegu = CAL_eauto;
      CAL_eauto = 0;

      KickWdt();
      return;
    }
  }
}

/**********************************************************//**
*  @name CalibrationReadSamples
*  @brief
*  REVIEW: What is this for? Samples are multiplied together so this 
*  is not required?!
*  
*  @param none
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void CalibrationReadSamples(void)
{
  if (CAL_FL_A_x4p3a < 0)
  {
    CAL_FL_A_x4p3a *= -1;
  }                     /* NEGA */
/*
 * if (x4p3b >128)
 *      x4p3b = 256 - x4p3b;	//NEGA
 */
  if (CAL_FL_B_x4p3b < 0)
  {
    CAL_FL_B_x4p3b *= -1;
  }
}
/**********************************************************//**
*  @name CalCalculateAvg
*  @brief
*  Use raw pyro ADC values to calculate long term average 
*  to track signal drift.
*  This function stores avg to global variables returns nothing.
*  Average_128 = Average_128+ High_Byte(Sample * 128 - Average_128)
*  
*  @param none
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified CAL_FL_A_avg, CAL_FL_B_avg, CAL_GU_avg, BIAS_avg
*  DO we need to prefix with g_???
**************************************************************/
void CalCalculateAvg(void)
{
  signed int tempcalc = 0;
  /* flame A */
  tempcalc = (CAL_FL_A_x4p3a * 128 - CAL_FL_A_avg) + LTAroundingfactor;
  CAL_FL_A_avg += tempcalc >> 8;
  /* flame B */
  tempcalc = (CAL_FL_B_x4p3b * 128 - CAL_FL_B_avg) + LTAroundingfactor;
  CAL_FL_B_avg = CAL_FL_B_avg + (tempcalc >> 8);
  /* guard */
  tempcalc = (CAL_GU_x3p8 * 128 - CAL_GU_avg) + LTAroundingfactor;
  CAL_GU_avg = CAL_GU_avg + (tempcalc >> 8);
  /*
   * sun sensor not currently used
   * tempcalc=(SUN_sens *128 - SUN_avg)+LTAroundingfactor;
   * SUN_avg=SUN_avg+(tempcalc>>8);
   */
}

/**********************************************************//**
*  @name GetCapturedSigOrBiasLta
*  @brief
*  Description: This function manages the following:???
*  
*  @param none
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
signed char GetCapturedSignal(unsigned char reqtypval)
{
  switch (reqtypval)
  {

  case REQ_FLAMAXI:
    return(flamaxi);

  case REQ_FLAMINI:
    if (flamini < -127)
    {
      return(-127);
    }
    return((signed char)flamini);

  case REQ_FLBMAXI:
    return(flbmaxi);

  case REQ_FLBMINI:
    if (flbmini < -127)
    {
      return(-127);
    }
    return((signed char)flbmini);

  case REQ_GUARDMAXI:
    return(gumaxi);

  case REQ_GUARDMINI:
    if (gumini < -127)
    {
      return(-127);
    }
    return((signed char)gumini);

  case REQ_SUNMAXI:
    return(0);

  case REQ_SUNMINI:
    return(0);

  default:
    return(0);
  }
}

/**********************************************************//**
*  @name GetAccumulatedValue
*  @brief
*  Read algorithm accumulations. Used by SMP. 
*  
*  @param none
*  @return Requested 64 bit parameter value
*  
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
__INT64_T_TYPE__ CalGetAccumulatedValue(unsigned char valreqtyp)
{

  switch (valreqtyp)
  {
  case REQ_CAPEAUTOA:
    return(capturefla);
  case REQ_CAPEAUTOB:
    return(captureflb);
  case REQ_CAPEAUTOG:
    return(capturegu);
  case REQ_CAPEAUTOS:
    return(0);   /* capturesun */
  default:
    break;
  }
  return(0);
}

/**********************************************************//**
*  @name F()
*  @brief
*  Read requested analogue channel.
*  
*  @param Requested channel
*
*  @return Value (8 bit) of requested channel
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
unsigned char CalGetRequestedAnalogCh(unsigned char requestedch)
{
  switch (requestedch)
  {
  case REQ_BIASLTACAP:
    return((unsigned char)((CalibBiaslta + 64) / 128));
  /* temp calib values */
  case REQ_TEMPCALFLAH:
    return(tempcalflah);
  /* break; */
  case REQ_TEMPCALFLAL:
    return(tempcalflal);
  case REQ_TEMPCALFLBH:
    return(tempcalflbh);
  /* break; */
  case REQ_TEMPCALFLBL:
    return(tempcalflbl);
  case REQ_TEMPCALGUH:
    return(tempcalflgh);
  /* break; */
  case REQ_TEMPCALGUL:
    return(tempcalflgl);
  case REQ_TEMPCALSUNH:
    return(0U);
  /* break; */
  case REQ_TEMPCALSUNL:
    return(0U);

  default:
    return(0U);
    /* break; */
  }
}

/**********************************************************//**
*  @name CalibrationInitProcs()
*  @brief
*  Description: This function manages the following:
*  
*  @param none
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void CalibrationInitProcs(void)
{
  CAL_eauto = 0; 
  SetRange();
  CAL_primhga = 1U;
  CAL_primhgb = 1U;
  CAL_sechg = 1U;         /* guard gain */
  SetHigh();
}

/********************************************************************//**
*  @name GetCalibrationInProgressStatus
*  @brief
*  This function will return true if calibration is in progress.
*  
*  @param none
*  @return calibrationinprogress
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
unsigned char GetCalibrationInProgressStatus(void)
{
  return(calibrationinprogress);
}

/**********************************************************//**
*  @name GainDuringCapture
*  @brief
*  Read gain state during calibration or capture.
*  
*  @param none
*  @return GainCapture
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
unsigned char GainDuringCapture(void)
{
  return(GainCapture);
} /* func end */
