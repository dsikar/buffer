/***********************************************************************//**
* @file  adc.c
* @brief
*  This file contains the functions associated with ADC of AVR 1281 microcontroller.
*  Functions which are used for reading temperature and various analogue channels
*  could be found in this file.
*  Used On:     FV400
***************************************************************************/
#define  ENABLE_BIT_DEFINITIONS
#include <iom1281.h>
#include "my_iom1281.h"
#include <inavr.h>
#include <pgmspace.h>

#include "defs.h"
#include "adc.h"
#include "configuration.h"
#include "peripheral.h"
#include "library.h"
#include "panelcomms.h"
#include "hardwarefaultsentinel.h"
#include "serviceprotocol.h"
#include "temperaturesentinel.h"
#include "video.h"
#include "interruptcaptureandhwmonitor.h"
#include "servicecommands.h"

/*
 * Thermister ADC value to temperature conversion table.
 * The controller board thermister does not have a linear response.
 * This table provides a lookup table from ADC value (0-255) to
 * (temperature * 100).
 * Due to sharp drop off, the lower temperature has been clipped
 * to -50 deg
 * The accuracy is required when calculating the temperature
 * ramp between single ADC transitions. Example: if the temperature
 * changed from 2803 to 2836 in 600 seconds, calcultate the ramp.
 */
const __farflash int TemeratureAdcLookup[] = {
  -5561,-5408,-5256,-5104,-4951,-4799,-4646,-4494,
  -4342,-4189,-4037,-3884,-3732,-3580,-3442,-3320,
  -3198,-3077,-2963,-2864,-2766,-2667,-2568,-2474,
  -2393,-2311,-2229,-2147,-2066,-1986,-1917,-1849,
  -1780,-1711,-1642,-1573,-1504,-1444,-1385,-1326,
  -1266,-1207,-1148,-1089,-1029,-974,  -922, -870,
  -818,  -766, -714, -662, -609, -557, -505, -458,
  -411,  -365, -318, -271, -224, -177, -131,  -84,
  -37,   9,     52,   95,   138,  181,  224,  267,
  310,  353,   396,  439,  482,  523,  564,  604,
  645,  685,   726,  766,  807,  847,  888,  928,
  969, 1009,  1048, 1087, 1126, 1165, 1204, 1243,
  1282,1321,  1360, 1399, 1438, 1477, 1516, 1554,
  1593,1631,  1669, 1708, 1746, 1785, 1823, 1862,
  1900,1939,  1977, 2016, 2054, 2093, 2132, 2171,
  2209,2248,  2287, 2326, 2364, 2403, 2442, 2481,
  2520,2560,  2600, 2639, 2679, 2719, 2759, 2799,
  2839,2879,  2918, 2958, 2998, 3040, 3082, 3123,
  3165,3207,  3249, 3290, 3332, 3374, 3416, 3457,
  3499,3544,  3588, 3633, 3677, 3722, 3766, 3811,
  3855,3900,  3944, 3989, 4036, 4084, 4132, 4181,
  4229,4277,  4325, 4373, 4421, 4470, 4519, 4572,
  4625,4678,  4731, 4783, 4836, 4889, 4942, 4995,
  5052,5111,  5169, 5228, 5286, 5345, 5403, 5462,
  5523,5588,  5654, 5719, 5785, 5850, 5915, 5981,
  6052,6126,  6200, 6273, 6347, 6421, 6494, 6577,
  6661,6744,  6828, 6911, 6995, 7089, 7184, 7279,
  7375,7470,  7574, 7683, 7791, 7900, 8010, 8134,
  8259,8383,  8509, 8651, 8794, 8937, 9091, 9255,
  9419,9595,  9783, 9972,10184,10400,10633,10881,
  11129,11377,11625,11873,12121,12369,12617,12865,
  13113,13361,13609,13857,14105,14353,14601,14849
};


unsigned int CalcCalibration(unsigned char normalised, unsigned char raw);
/*
 * Flag set when ATMega128 CODE checksuming is enabled.
 * This flag is set to false only during development to allow usage of JTAG
 */
//extern unsigned char g_enablechecksum;
extern volatile unsigned char g_mux3[8],g_mux1[8];
unsigned char mxbitset[12] = {0x11U,0x01U,0x10U,0x22U,0x02U,0x20U,0x44U,0x04U,0x40U,0x88U,0x08U,0x80U};

/********************************************************************//**
 * NAME
 * GetAdcRawValue()
 * DESCRIPTION:
 * Perform a single ADC on the selected analogue input.
 * The result is adjusted according to the ADC calibration value.
 * If the JTAG is used, the ADC inputs that are shared with the JTAG are
 *  set to a default value to prevent unpredictable results
 * @param channel Selects an input to perform ADC conversion. Range ADC0-ADC7
 *
 * RETURN VALUE
 * Calibration adjusted result. 0-255
 *
 * LOCAL DATA
 * adcval, calval: Temp holders for pre-calibrated ADC result and the
 * calibration value
 *
 * RANGE CHECK RISK ANALYSIS
 * channel - input parameter. Masked at point of usage. Using enums
 *  prevents invalid value is entered
 * calval - Should the calibration value be out of the normal range,
 *  the value is set to 0x8000 (1:1 or unity)
 *
 * DEADLOCK RISK ANALYSIS
 * while() loop - Relies on a hardware flag that is normally false.
 *  If stuck in while loop, WDT reset signals a hardware fault.
 ***************************************************************************/
unsigned char GetAdcRawValue(unsigned char channel)
{
  unsigned char adcval;

#ifdef JTAGS_ON
  static BOOL jtag_print = FALSE;   /* remove after dev? */
  if (jtag_print == FALSE)
  {
    printf_P("\r\n\r\n**  JTAG MODE, ENSURE H/W SWITCH IN 'TEST' POSITION  **\r\n\r\n");
    delayms(1000U);
    jtag_print = true;
  }
#endif

  ADMUX = (0x01U << REFS0) |   /*  External 3.3v applied to AREF, internal Vref turned off */
          (0x01U << ADLAR) | /* Left adjust, since we are reading byte values */
          (channel & 0x07U); /* Select channels with no gain */

  ADCSRA = (0x01U << ADEN) |   /* Enable ADC */
          (0x01U << ADSC) | /* Set to single convertion mode */
/*      (0x06<<ADPS0); // set ADC prescalar */
          (0x04U << ADPS0); /* set ADC prescalar */
  /* Wait for convertion to complete */
  while (ADCSRA & (0x01U << ADSC) )
  {}

  adcval = ADCH;

  return(adcval);
}

/********************************************************************//**
 * GetAdcMuxValue();
 *
 * @param portc_bits configure MUX.
 * @param adc_channel selects requested adc channel.
 *
 * Discription: This function sets the requested MUX channel then waits for
 * 2 ms and then reads the takes the ADC reading. The thig to notice here is
 * if the request is for reading the 4_20mA monitor channel then this
 * function is required to be called twice with a gap of atleast 6ms,
 * for this reason you will notice under the 10ms ISR this function is called
 * twice. During the first call this will set the MUX ports and in the second
 * call this willtake the reading, as the ISR is 10ms so there is sufficient time
 * to settle.
 ***************************************************************************/
BYTEu GetAdcMuxValue(unsigned char portc_bits, BYTEu adc_channel){
  unsigned char ADC_result=0U;
  //make sure we DO NOT address PC0, 1 , 6, 7 directly via portc_bits...
  portc_bits&=~(ADC_MUX_MASK);
  PORTC|=portc_bits;    // set the channel bits we want!!
  PORTC&=(portc_bits|ADC_MUX_MASK);      //Reset other bits
  //now wait for settle then read
  if(GetMxSmpStatus() == false)
  {
   __sleep();
  }
  else
  {
   delayms(10U);
  }
  ADC_result=GetAdcRawValue250(adc_channel);
  return(ADC_result);
} //void

/********************************************************************//**
 * SetAdcMuxChannel(unsigned char portcbits);
 * @param portcbits configure port c of 1281 for required MUX.
 * following two functions must be called one after another after a
 * settling delay, these functin were made to replace the existing
 * adc read mechanism which adds a polling delay of 2ms for each read,
 * by opting this new apporach those 2 ms would be saved within the 10ms
 * ISR, hence we can extend the sleep time in order to save current consumption
 * for S231 mode. AR, APR 28 2011.
 ***************************************************************************/
void SetAdcMuxChannel(unsigned char portc_bits)
{

  portc_bits &= ~(ADC_MUX_MASK);
  PORTC |= portc_bits;    /* set the channel bits we want!! */
  PORTC &= (portc_bits | ADC_MUX_MASK);      /* Reset other bits */
}

/********************************************************************//**
 * ReadAdcSettledCh(channel);
 * @param channel requested adc channel on 1281 chip.
 *************************************************************************/
unsigned char ReadAdcSettledCh(unsigned char adc_channel)
{
  return GetAdcRawValue250(adc_channel);
}

/********************************************************************//**
 * BYTEu VideoGetAdcMuxValue(BYTEu portc_bits, BYTEu adc_channel);
 * @param portc_bits pins to configure MUX
 * @param adc_channel adc channel on AVR 1281
 *************************************************************************/

unsigned char VideoGetAdcMuxValue(unsigned char portc_bits,unsigned char adc_channel)
{
/*
 * used by video init. & main loop
 * Do a belt & braces check here - make sure we DO NOT address PC0, 1 or 7 directly via portc_bits...
 */
  portc_bits &= ~(ADC_MUX_MASK);

  PORTC |= portc_bits;    /* set the channel bits we want!! */
  PORTC &= (portc_bits | ADC_MUX_MASK);      /* Reset other bits (but Dont clobber the bits set by mux mask). */
  delayms(WAITFOR_ADC_SETTLE); /* Re-entrant code substitued...June 2012 */
  BYTEu ADC_result = GetAdcRawValue250(adc_channel);
  /* printf_P("\n\r adc video val=%d",ADC_result); */
/*   printf_P("GetAdcMuxValue  portc_bits=%d  adc_channel=%d  result=%d \n\r", portc_bits, adc_channel, ADC_result); */
  return ADC_result;
}

/********************************************************************//**
 * unsigned char GetAdcRawValue250(unsigned char channel)
 * @param channel requested ADC channel.
 *************************************************************************/
unsigned char GetAdcRawValue250(unsigned char channel)
{

  unsigned char adcval = 0U;
/* why someone will write in such an uncommon way I dont know AR 2011 */
  switch (channel)
  {
  case CH0_FLAME:    /* ADC0 */
  case CH1_FLAME:    /* ... */
  case CH2_GUARD:
  case CH3_SUN_SENSOR:
  case CH4_WIN_TEST:
  case CH5_FLAME_BIAS:
#ifdef JTAGS_OFF
  case ADC6:
  case ADC7:
#endif

/*
 * case CH3_MULX:
 * Error if specified directly (rule may change thou)
 */
    adcval = GetAdcRawValue(channel);
    break;

/* case CH3_MULX://This shouldn't be specified directly (rule may change thou) */
  default:
/* FLAG ERROR MESSAGE HERE?!? invalid channel has been specified */
    printf_P("ERROR - invalid CH3_MULX or unknown channel specified  (GetAdcRawValue250  ,  adc.c)\r\n");
    adcval = 0U;
    break;
  }

  return(adcval);
}

/********************************************************************//**
 * GetAdcTenBitCalValue(unsigned char channel, unsigned char faultreadreqst);
 * @param channel rwquested adc channel
 * @param faultreadreqst request to read the bias error
 *************************************************************************/
unsigned int calibtenbitval[] = {264U,264U,263U,263U,262U,262U,261U,261U,260U,260U,
                                 259U,259U,258U,258U,257U,257U,256U,256U,256U,255U,
                                 255U,254U,254U,253U,253U,252U,252U,251U,251U,251U,
                                 250U,250U,249U,249U,248U,248U,248U};
unsigned int GetAdcTenBitCalValue(unsigned char channel,unsigned char faultreadreqst)
{
  static unsigned int recordfaultid = 0U;
  unsigned int adctenbitval = 0U,calibratedvalue = 0U;

  if (!faultreadreqst)
  {
    ADMUX = (0x01U << REFS0) | /*  External 3.3v applied to AREF, internal Vref turned off */
            (0x01U << ADLAR) | /* Left adjust, since we are reading byte values */
            (channel & 0x07U); /* Select channels with no gain */

    ADCSR = (0x01U << ADEN) | /* Enable ADC */
            (0x01U << ADSC) | /* Set to single convertion mode */
            (ADC_PRESCALAR << ADPS0); /* set ADC prescalar */
    /* Wait for convertion to complete */
    while (ADCSR & (0x01U << ADSC) )
    {}

    adctenbitval = ADC;
    /* right adjust (well quite opposite form traditional way, but works fine A.R.) */
    adctenbitval = (adctenbitval >> RIGHT_ADJUST) & 0x03FFU;

    /* check for faults */
    if (adctenbitval > BIAS_MAX_VAL)
    {
      recordfaultid = 2U;
      adctenbitval = BIAS_DEF_VAL;
    }
    else if (adctenbitval < BIAS_MIN_VAL)
    {
      recordfaultid = 1U;
      adctenbitval = BIAS_DEF_VAL;
    }
    else if ((recordfaultid == false) && (adctenbitval <= BIAS_MAX_VAL) && (adctenbitval >= BIAS_MIN_VAL))
    {
      recordfaultid = 0U;
    }
    else
    {
      __no_operation();
    }

    calibratedvalue = calibtenbitval[adctenbitval - BIAS_MIN_VAL];
  }
  else
  {
    /* just fault read */
    calibratedvalue = recordfaultid;
  }
  return(calibratedvalue);
}

/********************************************************************//**
 * GetExternalWalkTestState()
 *
 * DESCRIPTION
 * Using ADC, Get walk test state
 * The following comment is by Dave R.
 * The walk-test input connects to processor pin 57 (PF4: ADC4/TCK).
 * The following thresholds are expressed as an 8-bit A/D output
 * The three switch resistors are
 *  SW0: 1k8 (1.67v to 1.96v at screw terminal)
 *  SW1: 4k7 (3.48v to 4.01v at screw terminal)
 *  SW2: 15k (6.51v to 7.24v at screw terminal)
 * 5% can be used, though 1% will most likely be used in practice.
 * Comments by A.Rana:
 * An eight bit ADC is used: This gives the following ranges
 * Short circuit:  below 20
 * SW0 (1K8)       up to 65
 * SW1 (4K7)       up to 122
 * SW2 (15K)       up to 190
 * Open Circuit:   up to 244
 * @param reqtyp raw adc val or test request
 *
 * RETURN Walk test state enum
 *************************************************************************/
unsigned char GetExternalWalkTestState(unsigned char reqtyp)
{
  static unsigned char adcval,confirmcount = 0U;
  unsigned char retval;

  if (reqtyp != RAW_VAL)
  {
    /* Read settled ADC input */
    adcval = ReadAdcSettledCh(ADC_CH6_MULX_2_3);
    /*
     * adcval = GetAdcMuxValue(WTEST, ADC_CH6_MULX_2_3);
     * printf_P ("WALK TEST VAL = %d\r\n",adcval);
     */
    if (adcval <= WT_LIMIT_SHORTCIRCUIT) /* 20 */
    {     /* We have a short circuit */
      retval = WT_SHORTCIRCUIT;
    }
    else if (adcval <= WT_LIMIT_BUILTINTEST) /* 100 */
    {
      confirmcount++;
      if (confirmcount >= MAX_WIRED_CNT)
      {
        confirmcount = MAX_WIRED_CNT;
        /* Switch with 1K8 load is present */
        retval = WT_BUILTINTEST;
      }
      else
      {
        retval = WT_OPENCIRCUIT;
      }

    }
    else if (adcval <= WT_LIMIT_OPM) 
    {
      confirmcount++;
      if (confirmcount >= MAX_WIRED_CNT)
      {
        confirmcount = MAX_WIRED_CNT;
        /* Switch with 15K load is present */
        retval = WT_OPM;
      }
      else
      {
        retval = WT_OPENCIRCUIT;
      }

    }
    else if (adcval <= WT_LIMIT_RESET)  
    {
      confirmcount++;
      if (confirmcount >= MAX_WIRED_CNT)
      {
        confirmcount = MAX_WIRED_CNT;
        // Switch with 4K7 load is present 
        retval = WT_RESET;
      }
      else
      {
        retval = WT_OPENCIRCUIT;
      }

    }
    else
    {
      retval = WT_OPENCIRCUIT;
      /* reset internal count */
      confirmcount = 0U;
    }
  }
  else
  {
    retval = adcval;
  }
  return(retval);
}

/*
 For Flame Proof
 FTE Modification Only
*/
unsigned char FpGetExternalWalkTestState(unsigned char reqtyp)
{
  static unsigned char adcval,confirmcount = 0U;
  unsigned char retval;

  if (reqtyp != RAW_VAL)
  {
    /* Read settled ADC input */
    adcval = ReadAdcSettledCh(ADC_CH6_MULX_2_3);
    /*
     * adcval = GetAdcMuxValue(WTEST, ADC_CH6_MULX_2_3);
     * printf_P ("WALK TEST VAL = %d\r\n",adcval);
     */
    if (adcval <= FP_LIMIT_SHORTCIRCUIT) /* 20 */
    {     /* We have a short circuit */
      retval = WT_SHORTCIRCUIT;

    }
    else if (adcval <= FP_LIMIT_BUILTINTEST) /* 100 */
    {
      confirmcount++;
      if (confirmcount >= MAX_WIRED_CNT)
      {
        confirmcount = MAX_WIRED_CNT;
        /* Switch with 1K8 load is present */
        retval = WT_BUILTINTEST;
      }
      else
      {
        retval = WT_OPENCIRCUIT;
      }

    }
    else if (adcval <= FP_LIMIT_RESET) /* 160 */
    {
      confirmcount++;
      if (confirmcount >= MAX_WIRED_CNT)
      {
        confirmcount = MAX_WIRED_CNT;
        /* Switch with 4K7 load is present */
        retval = WT_OPM;
      }
      else
      {
        retval = WT_OPENCIRCUIT;
      }

    }
    else if (adcval <= FP_LIMIT_OPM) /* 227 */
    {
      confirmcount++;
      if (confirmcount >= MAX_WIRED_CNT)
      {
        confirmcount = MAX_WIRED_CNT;
        /* Switch with 15K load is present */
        retval = WT_RESET;
      }
      else
      {
        retval = WT_OPENCIRCUIT;
      }

    }
    else
    {
      retval = WT_OPENCIRCUIT;
      /* reset internal count */
      confirmcount = 0U;
    }
  }
  else
  {
    retval = adcval;
  }
  return(retval);
}

/********************************************************************//**
 * GetHeatedOpticsState()
 *
 * DESCRIPTION
 * Get the state of the heated optics driver circuit.
 * If the window heater is not active, the return state will indicate
 * open circuit.
 * The following comment is by Dave R.
 * I have put in the window heater resistor drive. I have put in
 * analogue feedback to processor PF5 (pin 56).
 * Heater resistor ON if PG4 high, OFF if low.
 *  PF5< 5      Heater off or resistor open-circuit
 *  5 <= PF5 < 72     Resistor OK
 *  72<= PF5 < 220    Resistor short-circuit
 *  220 <= PF5     A general driver fault.
 * The driver is continuously rated for a shorted load, but
 * ideally it should be switched off.
 * @param adcresult
 *
 *************************************************************************/
unsigned char GetHeatedOpticsState(unsigned char *adcresult)
{
  unsigned char adcvalue,retval;
  /*
   * Read ADC input
   * MRP
   */

/*   adcvalue=GetAdcRawValue(HEATERMONITOR,true); */

  /* PORTA |= (0X01<<PA6);// select window-heater monitor */
  PORTG |= (0X01U << PG2);   /* enable monitoring via isolation barrier */
  PORTD &= ~(0X01U << PD6);   /* The heater is on */

  adcvalue = GetAdcMuxValue(VIDEO_SYNC_AND_WHEAT_MON, ADC_CH7_MULX_0_1);   /* Read MUX1, Y5 */

/*
 *   PORTA &= ~(0X01<<PA6);// select window-heater monitor OFF
 * This is selected with PD6=low.
 */
  PORTG &= ~(0X01U << PG2);   /* disable isolation barrier */
  PORTD |= (0X01U << PD6);   /* video on */
  delayms(WAITFOR_SETTLE);
  CameraPowerControl(true,true);



  if (adcvalue >= WINDOW_HEATER_LIMIT)
  {
    /* Headed optics is off OR resistor is opencircuit */
    retval = HO_OPENCIRCUIT;
  }
  else
  {
    /* Resistor OK */
    retval = HO_RESISTORONOK;
  }

/*
 * N/A ? :-
 * Heater is not supported in RevB and below, set to OK
 *   if(GetSoftwareCompatibilityId()<=ISSUE00_REVB)...
 * Pass back the ADC value & resistor state to caller.
 */
  *adcresult = adcvalue;
  return(retval);
}

/********************************************************************//**
 * MxDigitalOutputRead()
 *
 * Returns the one byte value read on MX ASIC digital output.
 *************************************************************************/
unsigned char MxDigitalOutputRead(void)
{
  static unsigned char retmxval = 0x00U;
  unsigned char mxval[4],loopcntx = 0U,arrayrowcount = 0U;
  /* global flag set true by ISR */
  if (GetMxUpdateFlag())
  {
    __disable_interrupt();
    /* get values from 4 MUX1 channels updated in 10ms ISR */
    mxval[0] = g_mux1[MXVAL_D0_D4];
    mxval[1] = g_mux1[MXVAL_D1_D5];
    mxval[2] = g_mux1[MXVAL_D2_D6];
    mxval[3] = g_mux1[MXVAL_D3_D7];
    __enable_interrupt();
    retmxval = 0U; /* reset retval */
    /*
     * }else return retmxval;
     * now decode the values with the help of lookup and defined range.
     */
    for (loopcntx = 0U; loopcntx < MAX_ADC_LINES; loopcntx++)
    {
      if (mxval[loopcntx] <= BOTHZERO)
      {
        retmxval |= 0x00U;     /* mxbitset[arrayrowcount]; */
      }
      else if (mxval[loopcntx] <= MSBSIDEONEOTHERZERO)
      {
        retmxval |= mxbitset[arrayrowcount + 2U];
      }
      else if (mxval[loopcntx] <= LSBSIDEONEOTHERZERO)
      {
        retmxval |= mxbitset[arrayrowcount + 1U];
      }
      else     /* BOTHONE */
      {
        retmxval |= mxbitset[arrayrowcount];
      }
      arrayrowcount = arrayrowcount + 3U;
    }     /* for */
  }
  /*
   * printf_P("\r MX VAL is=%X",retmxval);
   * printf_P("\r 0,1,2,3 =%d,%d,%d,%d",mxval[0],mxval[1],mxval[2],mxval[3]);
   * printf_P("\r MX VAL dec is=%d",retmxval);
   */
  return(retmxval);
}
/********************************************************************//**
 * GetTemperature()
 *
 * DESCRIPTION
 * Measure temperature in 1/100 Deg C by using thermistor mounted
 * on controller board.
 * Uses lookup table to convert from ADC value to temperature.
 *************************************************************************/
int GetTemperature(void)
{
  int temperature;
  /*
   * Read ADC input (calibrated) and convert to 1/100th of a degree using
   * lookup table.
   * g_mux3[TEMPERATURE_RD]=GetAdcMuxValue(THERMISTOR, ADC_CH6_MULX_2_3);
   */
  g_mux3[TEMPERATURE_RD] = ReadAdcSettledCh(ADC_CH6_MULX_2_3);
  temperature = TemeratureAdcLookup[g_mux3[TEMPERATURE_RD]];
  return(temperature);
}

/********************************************************************//**
 * GetLampState()
 *
 * DESCRIPTION
 * Measures the current through the OPM lamps. Used to detect bulb fault
 * conditions. Current monitoring is located on the sensor board and
 * routed to the controller board.
 *
 * PRECONDITIONS
 * Current must be allowed to settle before a measurment can occur.
 * Immediately after bulb turn on, the resistance of the fillement
 * indicates a short circuit. This settles to a steady current after a few
 * milliseconds.
 * Only one lamp can be tested at a time.
 * When the bulb is off, a reading will indicate bulb 'open circuit'
 *
 * INFO
 * The following is an example of a quick sucession of readings:
 * @ 25 deg C
 * Lamp1
 * 27|52|67|75|81|84|86|87|88|89|89|90|90|91|91|91|92|92|92|93|93|93|94|94|94|
 *  95|95|95|96|96|96|97|97|97|98|98|99|99|99|100|100|101|101|101|102|102|
 * Lamp2
 * 27|53|68|77|83|86|88|90|91|91|92|93|93|93|94|94|94|95|95|96|96|96|97|97|97|
 *  98|98|99|99|99|100|100|101|101|102|102|103|103|103|104|104|105|105|106|
 * Lamp1 shorted
 * 25|39|47|51|54|55|56|57|57|57|57|57|58|58|58|58|58|58|58|58|58|58|58|58|58|
 *  58|58|58|58|58|57|58|58|57|58|57|58|58|58|57|57|58|57|57|57|57|57|58|57|
 *************************************************************************/
unsigned char GetLampState(unsigned char *adcresult)
{
  unsigned char adcvalue,retval;
  /* Read ADC input */
/*   adcvalue=GetAdcRawValue(LAMPCURRENT,true); */

#ifndef PRELIM_BOARD
/* #error this switch TBD */
#endif
/* MRP */
  adcvalue = LP_LIMIT_FILEMENTOK - 1U;
/* dummy value */

  if (adcvalue < LP_LIMIT_SHORTCIRCUIT)
  {
    /* Shorted */
    retval = LP_SHORTCIRCUIT;
  }
  else if (adcvalue < LP_LIMIT_FILEMENTOK)
  {
    /* Filement OK */
    retval = LP_FILEMENTOK;
  }
  else
  {
    /* Open circuit */
    retval = LP_OPENCIRCUIT;
  }
  /* Pass back the ADC value & lampstate to caller. */
  *adcresult = adcvalue;
  return(retval);
}

/********************************************************************//**
 * GetRelayState()
 *
 * DESCRIPTION
 * Relay coil integrety check.
 * The Relay coil can only be tested when the coil is
 * not energised. It relies on the resistance of the coil pulling
 * analogue input to ground. Should coil fail, a small voltage
 * supplied by a pullup should be seen.
 * PARAMETERS
 * NONE
 * RETURN VALUE
 * adcresult: The raw value of the ADC conversion.
 * return(): Relay coil pass / fail state
 * LOCAL DATA
 * retval: destination register for coil state
 * adcresult: Calibrated ADC result from relay coil check
 * PRECONDITIONS
 * The relay MUST be in the off state before this test can occur. The
 * relay must have also been off for sufficiently long enough for the
 * residual magnetic field in the coil to decay to nothing.
 * POSTCONDITIONS
 * NONE
 * RANGE CHECK RISK ANALYSIS
 * NO RISK
 * DEADLOCK RISK ANALYSIS
 * NO RISK
 * INFO
 * Example of Coil current @ 25degC after turnoff (no delay)
 *  93|71|53|40|30|22|16|11|7|4|2|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|
 ***************************************************************************/
unsigned char GetRelayState(unsigned char *adcresult)
{
  unsigned char adcvalue,retval;
  /* Read ADC input */
/*   adcvalue=GetAdcRawValue(ALARMRELAYCOIL,true); */
#ifndef PRELIM_BOARD
/*
 * #error this switch TBD
 * adcvalue=GetAdcRawValueCHECKME(ALARMRELAYCOIL,true);
 */
#endif
  adcvalue = RY_LIMIT_COILOK - 1U;
/* dummy ok? */

  if (adcvalue < RY_LIMIT_COILOK)
  {
    /* No voltage detected. Coil OK */
    retval = PASS;
  }
  else
  {
    retval = FAIL;
  }
  /* Pass back the ADC value & relaystate to caller. */
  *adcresult = adcvalue;
  return(retval);
}

void SelectADC0Volts(void)
{
/*  External 3.3v applied to AREF, internal Vref turned off */
  ADMUX = (0x01U << REFS0) |
/* Left adjust, since we are reading byte values */
          (0x01U << ADLAR) |
/* Select internal 0 volts (select away from analogue inputs), D.R, 13/7/10):- */
          (0x1fU); /* setting all channels to GN */
}
/* ///////////////////////////// EOF ///////////////////////////// */
