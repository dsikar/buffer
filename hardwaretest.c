/***********************************************************************//**
* @file  hardwaretest.c
* @brief
*  This file contains individual fault monitoring and test functions.
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

#include "library.h"
#include "defs.h"
#include "hardwaretest.h"
#include "adc.h"
#include "peripheral.h"
#include "fde.h"
#include "spi.h"
#include "panelcomms.h"
#include "hardwarefaultsentinel.h"
#include "configuration.h"
#include "interruptcaptureandhwmonitor.h"
#include "serviceprotocol.h"
#include "opmfaultsentinel.h"
#include "ir.h"
#include "termdebug.h"
#include "heatedoptics.h"
#include "video.h"
#include "fde.h"
#include "servicecommands.h"
#include "AFDPL15_IAR.h"
#include "s231interface.h"
#include "scememory.h"
#include "debug.h"

/* NEW ALARM TEST ARRAY */
const __farflash INT16u g_AlarmLampLookUp[] = {
361U,391U,421U,451U,482U,514U,546U,579U,612U,646U,680U,715U,750U,786U,823U,861U,
899U,937U,977U,1017U,1058U,1100U,1142U,1186U,1230U,1275U,1321U,1367U,1415U,1464U,1514U,1564U,
1616U,1669U,1723U,1778U,1835U,1892U,1951U,2012U,2073U,2136U,2201U,2267U,2335U,2404U,2475U,2548U,
2622U,2699U,2777U,2858U,2940U,3025U,3112U,3202U,3294U,3389U,3486U,3586U,3690U,3796U,3905U,4018U,
4135U,4255U,4379U,4507U,4640U,4777U,4919U,5066U,5218U,5375U,5539U,5709U,5885U,6068U,6259U,6458U,
6665U,6880U,7106U,7341U,7587U,7845U,8115U,8399U,8696U,9010U,9339U,9687U,10055U,10444U,10855U,11293U,
11757U,12253U,12781U,13347U,13954U,14606U,15310U,16070U,16895U
};
/*
 * Flag set when ATMega128 CODE checksuming is enabled.
 * This flag is set to false only during development to allow usage of JTAG
 * ALSO, when false, every log to the FDE is also printed to the external
 * configuration port  (RS485)
 */

/* make true for FINAL */
BYTEu g_enablechecksum = true; 

BYTEu g_RunAwayFlg = false;
volatile BYTEu g_ResetButtonTrigger = 0x00U,g_timer4_20val = 0U,g_flag15v_on = false,g_flag15v_off = false;
volatile BYTEu g_faultflag[LAST_FLTFLAG];

extern volatile BYTEu g_WinHeatAdcValue,g_mux1[8],g_mux3[8]; /* jun 2011 */
extern volatile INT16s g_TemperatureThmstr;
extern volatile BYTEu g_interruptmonitor;
/* ISR based flash checksum calculation Dec 2015 */
extern volatile INT16u g_isrfullchksm;
extern volatile BYTEu g_isrchksumready;
extern volatile BYTEu g_WtestAdcVal;
/* ATMega128 Checksum */
#pragma location=0x1DFFE /* DO NOT MODIFY THIS LINE! */
/* Update this checksum for every release */
__root const __hugeflash INT16u g_ApplicationChecksum =  0x00D7U;

/* Use the checksum table held in boot sector */
extern __farflash const INT16u g_sprog_crc16_table[256];

static BYTEu g_VideoType = PAL_SYS,Buildid = 0U;
static BYTEu g_fifteenvoltfltstate = NO_15V_FAULT;
static INT32u timelasttest = 0U;

/* Following variables are only used by electrical alarm test functions they may be made local in future */
static BYTEu elefla_A[LAMP_TEST_CYCLES / 2U],eleflb_A[LAMP_TEST_CYCLES / 2U],elegu_A[LAMP_TEST_CYCLES / 2U],
                     elefla_B[LAMP_TEST_CYCLES / 2U],eleflb_B[LAMP_TEST_CYCLES / 2U],elegu_B[LAMP_TEST_CYCLES / 2U];

BYTEu firstentrytesttag = false;

extern BYTEu dbgtestbyte;
extern INT16u g_opmtimerx;
/**
 * Dear Jon,
 *
 * The CRC in XLINK have the same effect on all targets (at least right now): The CRC-calculation will be performed over all memory, so there is no way of excluding.
 *
 * NOTE: Please do not delete the CALL_ID in the subject line, for future reference!
 *
 * Best Regards,
 *
 * Hassan A H
 *
 * IAR Systems Ltd
 * Technical Support Engineer
 * Web site: http://www.iar.com
 * Email: support@iarsys.co.uk
 * Tel: +44 (0) 20 7554 8582
 * Fax: +44 (0) 20 7554 8586
 *
 * -----Original Message-----
 * From: Jon Newcomb [Sunbury - Safety Products] [mailto:jnewcomb@tycoint.com]
 * Sent: 18 June 2004 11:48
 * To: support@iarsys.co.uk
 * Subject: Request for technical support
 *
 * NAME
 * Jon Newcomb
 *  
 * LICENCE NUMBER
 * 3000-082-422-9649 (EW Version 3.10A)
 *  
 * HARDWARE PLATFORM
 *  Atmel ATmega128
 *  Using JTAG ICE debugger under AvrStudio 4.09
 *  
 * WHAT I AM TRYING TO DO:
 * Prevent checksum (-J linker option) from covering my bootsector.
 *  
 * DETAILED DESCRIPTION
 * The linker automatically calculates a checksum. How do I restrict the address range checksumed so it does not include the boot sector?
 *    (I have managed to locate '__checksum' at the end of my application
 * segment)
 *  
 * My application code is in the range 00000 - 1DFFF (including checksum) My bootsector is in the range 1E000 - 1FFFF
 *  
 *  
 * Extract from the .map file gives:
 *  
 *                 ****************************************
 *                 *                                      *
 *                 *     CHECKSUMMED AREAS, IN ORDER      *
 *                 *                                      *
 *                 ****************************************
 *  
 * 00000000 - 0001DFFD in CODE memory
 * 0001E000 - 0001FFFF in CODE memory   <- How do I prevent it from covering this address range?
 *  
 * Checksum = 0x791a
 *  
 *  
 * Setup of checksum;
 *
 *  
 * Thanks;
 * Jon.
 */

/********************************************************************//**
* @name RamSelfTest();
* @brief
* checks the user RAM (not the stack) by :
* Read contents of address under test into temp.
* Invert contents of address and & with temp. Result should equel zero.
* Re-invert contents of address.
* To prevent corruption of data during test, interrupts are disabled.
* Test from the beginning the end of the IO registers (0x000200)
* upto the start of the CSTACK
* Transmit data register empty
* Sep08,2011
* @param updatefaultstate
* @return none
* globalvariablesused none
* globalvariablesmodified none
 ***************************************************************************/
BYTEu RamSelfTest(BYTEu updatefaultstate)
{
  BYTEu *address, *endaddress;
  BYTEu tempdata, retval = PASS;
  INT16u intaddress = 0U;
  /*
   * Disable interrupts while this test occurs to prevent
   * modification of static variables and the return stack
   */
  __disable_interrupt();
  /*
   * Get the end address of the test
   * Return pointer to data segment
   */
   #pragma segment="CSTACK"
  endaddress = (BYTEu *) __segment_begin("CSTACK");
  /*
   * Loop round testing ram, until we get to the end or
   * the test fails
   * start address for AVR1281 is 200hex, it was 100hex for 128
   */
  for (address = (BYTEu *)START_ADDRESS_AVR1281;
       ( (address < endaddress) && (retval == PASS) );
       address++)
  {
    /* Read into temp */
    tempdata = *address;
    
    if((ReturnTestByte() & 0xFFU) == 0x08U)
    {
     /* force fail */ 
     *address = tempdata;
    }
    else
    {
     /* invert contents of current address under test */
     *address = ~tempdata;
    }
    /*
     * <RAMTST> Software acceptance test: Force ramtest fail
     * if((INT16u)address==0x0222)tempdata^=0x01;
     * Check for memory error
     */
    if ( (tempdata & (*address) ) != 0x00U)
    {
      /* Return state back to caller. Also exits at current position */
      retval = FAIL;
    }
    /* Restore the old value */
    *address = tempdata;
    /* Prevent WDT reset */
    KickWdt();
  }
  /* Test finished. Enable interrupts */
  __enable_interrupt();
  /*
   * If requested by caller, update fault status depending on
   * result of test
   */
  if (updatefaultstate == true)
  {
    if (retval == FAIL)
    {
      /*
       * SRAM test failed.
       * Do not log if already in CNT_PROGRAMDATATEST fault. This prevents
       * multiple log entries.
       */
      if (ReadHardwareFaultFlag(CNT_PROGRAMDATATEST) == false)
      {
        /* Copy address into int */
        intaddress = (INT16u)address;
        FdeSelftest(PROGRAMDATA_RW,(BYTEu)((intaddress >> SHIFT_BY_BYTE) & 0x00FFU),
                    (BYTEu)(intaddress & 0x00FFU));
      }
      /* There is a read write error */
      UpdateHardwareFaultSentinel(CNT_PROGRAMDATATEST,true,0U);     /* active AR 2011 */
    }
    else
    {                                
      /* Data test passed. Clear fault. */
      UpdateHardwareFaultSentinel(CNT_PROGRAMDATATEST,false,0U);
    }
  }
  return(retval);
}

/********************************************************************//**
* @name fast_crc16()
* @brief
* The compiler is unable to support automatic checksum calculation when
* boot sector is enabled. As a result the checksum must be hand coded
* following a recompile for a release. The checksum is stored at the
* address 0x1DFFE
* @param p,len
* @return none
* globalvariablesused none
* globalvariablesmodified none
***************************************************************************/
unsigned short fast_crc16(unsigned short sum, BYTEu __hugeflash *p,
                          INT32u len, BYTEu sleepmden)
{
  if((ReturnTestByte() != 0x02U) && (ReturnTestByte() != 0x04U))
  { 
     while (len--)
     {
       if (((long)p < TEXTLOCATION + 80L) ||
           ((long)p >= TEXTLOCATION + 80L + 20L))
       {
         sum = g_sprog_crc16_table[(sum >> SHIFT_BY_BYTE_S) ^ *p++] ^ (sum << SHIFT_BY_BYTE_S);
       }
       else
       {
         p++;
       }
       KickWdt();
       if(sleepmden == true)
       {
         __sleep();
       }
     }
  }
  
  if((ReturnTestByte() & 0xFFU) == 0x04U)
  { 
    //ignore last byte
     while (len != 1U)
     {
       len--;
       
       if (((long)p < TEXTLOCATION + 80L) ||
           ((long)p >= TEXTLOCATION + 80L + 20L))
       {
         sum = g_sprog_crc16_table[(sum >> SHIFT_BY_BYTE_S) ^ *p++] ^ (sum << SHIFT_BY_BYTE_S);
       }
       else
       {
         p++;
       }
       KickWdt();
       if(sleepmden == true)
       {
         __sleep();
       }
     }
  }
  
  if ((ReturnTestByte() & 0xFFU) == 0x02U)
  {  
    //ignore the first byte
    *p++;
     while (len--)
     {
       if (((long)p < TEXTLOCATION + 80L) ||
           ((long)p >= TEXTLOCATION + 80L + 20L))
       {
         sum = g_sprog_crc16_table[(sum >> SHIFT_BY_BYTE_S) ^ *p++] ^ (sum << SHIFT_BY_BYTE_S);
       }
       else
       {
         p++;
       }
       KickWdt();
       if(sleepmden == true)
       {
         __sleep();
       }
     }
  }
  
  return sum;
}

/********************************************************************//**
* @name ApplicationCodeCrcTest()
* @brief
* This function calculates the code CRC the function works fine and is
* active. It logs the CRC in to the FDE as selftest event with the actual
* and expected values.
* NOTE: This function must be run at startup with fulltest=true
*  @param crctesttype,udpatefaultstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
BYTEu ApplicationCodeCrcTest(BYTEu crctesttype,
                                     BYTEu udpatefaultstate, BYTEu sleepenable)
{
  /* Variables required for CRC testing in blocks */
  static INT16u partialchksm = 0U;
  static BYTEu __hugeflash *partialaddress = 0U;
  BYTEu partialcrccomplete = false;
  /* Variable required for CRC testing in one go */
  INT16u fullchksm = 0U;
  BYTEu retval = PASS;
  BYTEu putinsleep = sleepenable;
  /* Has a full checksum test been requested? */
  if ((crctesttype == CRC_BOOT_TEST_ONLY) || (crctesttype == CRC_FULL))
  {
    /* Complete CRC test in one go */
    fullchksm = fast_crc16(0U,(BYTEu __hugeflash *)0U,
                           (INT32u)&g_ApplicationChecksum, putinsleep);
    /*
     * If this is a poweron test, store the checksum in partialchksm
     * for processing later on.
     */
    if (crctesttype == CRC_BOOT_TEST_ONLY)
    {
      partialchksm = fullchksm;
    }
  }
  else if (crctesttype == CRC_RUNTIME_BLOCK)
  {
    /*
     * Prevent the checksum covering the build date. This is because
     * the build date is comiled into the code and would result in a
     * checksum error every time a rebuild all performed.
     * This also allows users to checkout a version from sourcesafe
     * and build all without having to recompile in a new checksum
     */
    if (((long)partialaddress < TEXTLOCATION + 80L) ||
        ((long)partialaddress >= TEXTLOCATION + 80L + 20L))
    {
      /*
       * A checksum of part of the memory has been requested
       * Recalculate checksum for a single byte
       */
      partialchksm = g_sprog_crc16_table[(partialchksm >> 8) ^
                                       *partialaddress++] ^ (partialchksm << 8);
    }
    else
    {
      /* Move to the next address */
      partialaddress++;
    }
    /* Has the last byte been tested. */
    if (partialaddress >= (BYTEu __hugeflash *)&g_ApplicationChecksum)
    {
      /*
       * Yes, mark the test as complete. Now compare the CRC we have just
       * calculated with the actual CRC
       */
      partialcrccomplete = true;
      /* Copy the partialchksm into fullchksum */
      fullchksm = partialchksm;
      /* Reset the partial checksum address so we can start all over again */
      partialaddress = (BYTEu __hugeflash *)0;
      /* Reset the checksum ready for the next */
      partialchksm = 0U;
    }
  }
  else
  {
    /*
     * we have been asked not to test but process the result of the #
     * CRC_BOOT_TEST_ONLY test. The results of the preious checksum
     * test is stored in partialchksm. Copy across into checkum for
     * determining fault status
     */
    fullchksm = partialchksm;
    /* Reset the partial address checksum ready for the runtime CRC check */
    partialchksm = 0U;
  }
  /*
   * If we have just completed a CRC_BOOT_TEST_ONLY check, retain the
   * checksum and print the relevant messages
   */
  if (crctesttype == CRC_BOOT_TEST_ONLY)
  {
    /* if(g_enablechecksum){ */
    if (fullchksm != g_ApplicationChecksum)
    {
       if((PersonalityType() != CONVENT_MODE) && (PersonalityType() != MX_IFACE_MODE) &&
      (PersonalityType() != MX_IFACE_DIPSW_ACTIVE))
      {
         printf_P("APPLICATION CRC: EXPECTED= 0x%04X, ACTUAL= 0x%04X\r\n",
                  g_ApplicationChecksum,fullchksm);
         printf_P("NB Change CRC to 0x%04X once debugged: hardwaretest.c, line 74 \r\n", fullchksm);
      }
    }
    /* } */

  }
  else if (((partialcrccomplete == true) && (crctesttype == CRC_RUNTIME_BLOCK)) ||
           (crctesttype == CRC_BOOT_UPDATE_FAULT) || (crctesttype == CRC_FULL) )
  {
    /*
     * Partial CRC has completed OR we have just been
     * asked to do a full CRC check. Did the test pass or fail?
     */
    if (fullchksm != g_ApplicationChecksum)
    {
      /* The test failed. Do we need to log and update the fault state? */
      if (udpatefaultstate)
      {
        /*
         * The checksum has failed - Do not log if already in
         * CNT_PROGRAMCODETESTfault. This prevents multiple log entries.
         */
        if (g_enablechecksum == true)
        {
          /* Production release. Enable fault of checksum fail */
          if (ReadHardwareFaultFlag(CNT_PROGRAMCODETEST) == false)
          {
            FdeSelftest(PROGRAMCODE_CRC,
                        (BYTEu)((fullchksm >> SHIFT_BY_BYTE) & 0x00FFU),
                        (BYTEu)(fullchksm & 0x00FFU));
          }
          /* Update fault */
          UpdateHardwareFaultSentinel(CNT_PROGRAMCODETEST,true,0U);         /* /active AR 2011 */
        }
        else
        {
          /* Development release, disable fault on checksum */
          __no_operation();
        }
      }
      /* Return the results of the test */
      retval = FAIL;
    }
  }
  else
  {}
  /* Pass back the return state if we have completed a full CRC check */
  return(retval);
}

/********************************************************************//**
* @name IsrApplicationCodeCrcTest()
* @brief
* This function calculates the code CRC during runtime. 
* NOTE: Only for periodic CRC tests 
*  @param crctesttype,udpatefaultstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
void IsrApplicationCodeCrcTest(void)
{
 static BYTEu crcerrorcnt = 0U;
  
 if(g_isrchksumready == true)
 {
  if (g_isrfullchksm != g_ApplicationChecksum)
  {
      if(crcerrorcnt >= CRC_ERROR_LIM)
      {
        crcerrorcnt = 0U;
        if (g_enablechecksum == true)
        {
          UpdateHardwareFaultSentinel(CNT_PROGRAMCODETEST,true,0U); 
          //printf_P("\r isr=%X,actl=%X",g_isrfullchksm,g_ApplicationChecksum); 
          //delayms(20U);
        }
      }
      else
      {
        crcerrorcnt++;
      }
   }
   else if(crcerrorcnt != 0U)  
   {
     crcerrorcnt = 0U;
   }
   else{}

   g_isrchksumready = false;
 }
}

/********************************************************************//**
* @name IsrCalculateAppCodeCrc()
* @brief
* This function calculates the code CRC within 10ms interrupt. 
* NOTE: Only used by periodic CRC tests 
*  @param none
*  @return none
*  globalvariablesused g_isrchksumready,g_isrfullchksm
*  globalvariablesmodified g_isrfullchksm,g_isrchksumready
 *************************************************************************/
void IsrCalculateAppCodeCrc(void)
{
  static BYTEu __hugeflash *px = (BYTEu __hugeflash *)0U;
  static INT32u lenx = (INT32u)&g_ApplicationChecksum;
  static unsigned short sumx = 0U;
  
 if(g_isrchksumready == false)
 {
   
     if (lenx != false)
      {
         lenx--;
         if (((long)px < TEXTLOCATION + 80L) ||
             ((long)px >= TEXTLOCATION + 80L + 20L))
         {
           sumx = g_sprog_crc16_table[(sumx >> SHIFT_BY_BYTE_S) ^ *px++] ^ (sumx << SHIFT_BY_BYTE_S);
         }
         else
         {
           px++;
         }
       
     }
    else
     {
       g_isrchksumready = true;
       g_isrfullchksm = sumx;
       lenx = (INT32u)&g_ApplicationChecksum;
       px = (BYTEu __hugeflash *)0U;
       sumx = 0U;
     }
 }
}

/********************************************************************//**
* @name TestFdeFlashEpromRead()
*  @brief
* Read test only. Verifies the 'Device Code' is an AT25F1024 or
* pin compatable device. If not, you have some work to do!
* NOTE: all FDE function have been written to work with AT25F1024
* or pin compatible replacements.
* This routine is for FTE support only
* Status: Might need modification
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
BYTEu TestFdeFlashEpromRead(void)
{
  BYTEu retval = 0U;
  /* Get and compare the internal 'Device Code' */
  if (GetFlashDeviceCode(SPI_FDE_EEPROM) == MC25LC1024_DEVICECODE)
  {
    retval = PASS;
  }
  else if (GetFlashDeviceCodeFS() == AT25FS010_DEVICECODE)
  {
    retval = PASS;
  }
  else
  {
    retval = FAIL;
  }
  return(retval);
}

/****************************************************************//**
* @name TestFdeFlashWrite()
* @brief
* Write 0x00 to 4 sectors from address 0 to 255
* Read back those addresses if all zeros then pass else fail.
* Erase and then read back if all OxFF then pass else fail.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 **********************************************************************/
BYTEu TestFdeFlashWrite(void)
{
  BYTEu retval = PASS, sector;
  INT32u sectoraddress = 0U;
  INT16u testbyte = 0U;

  for (sector = SECTOR_ONE; sector <= TOTAL_SECTORS; sector++)
  {
    /*
     * Calculate the sector address we are doing read
     * write tests from
     */
    sectoraddress = (INT32u)(sector - 1U) * SECTORSIZE;

    /* Write test data 0x00 to the sector */
    for (testbyte = 0U; testbyte < BYTES_TO_TEST; testbyte++)
    {
      /* Write to all bits from start of sector */
      WriteFlashByteNew(sectoraddress +
                        ((INT32u)testbyte),0x00U);
    }
    WriteCompleteNew();
    /*
     * Check all the test data 0x00 has been written
     * take device out of sleep
     */
    GetFlashMultiByteStart(sectoraddress,SPI_FDE_EEPROM);
    for (testbyte = 0U; testbyte < BYTES_TO_TEST; testbyte++)
    {
      /* Read all bits from start of sector */
      if (GetFlashMultiByte() != 0x00U)
      {
        /* Bit(s) stuck at '1' */
        retval = FAIL;
      }
    }
    GetFlashMultiByteEnd(SPI_FDE_EEPROM);
    /* Clear test data contained default 0xFF */
    BulkEraseMemory(SPI_FDE_EEPROM);
    /* Check test data had been erased and all 0xFF */
    GetFlashMultiByteStart(sectoraddress,SPI_FDE_EEPROM);
    for (testbyte = 0U; testbyte < BYTES_TO_TEST; testbyte++)
    {
      /* READ all bits from start of sector */
      if (GetFlashMultiByte() != 0xFFU)
      {
        /* Bit(s) stuck at '0' */
        retval = FAIL;
      }
    }
    /* activate sleep again */
    GetFlashMultiByteEnd(SPI_FDE_EEPROM);
  }
  return(retval);
}

/********************************************************************//**
* @name CheckExternalRemoteTest();
* @brief
* The value for the walktest input is read within the ISR after every
* 100ms, global variable WtestAdcVal will get updated after every 100ms.
* If in case the value matches the associated test would be carried out.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void CheckExternalRemoteTest(void)
{
  static BYTEu previouswalktestate = WT_OPENCIRCUIT,wtrecordflag = false;
  static INT16u countduration = 0U, sccnt =0U; /* ###june 2010 */
  BYTEu currentwalkteststate = 0U;
  /*
   * check for MZX
   * Interface periodic update if in MX mode
   */
  UpdateMxParameters();
  /* main loop will start 3 sec early in conv mode so take precaution */
  if ((PersonalityType() == CONVENT_MODE) && (!GetSettlingState()))
  {
    return;
  }
  if (SoakTestConditionCheck(READ_RESULT))
  {
    return;
  }
  /* take the test value */
  currentwalkteststate = g_mux3[WALK_TEST];
  /* If both agree, carry out the test */
  if (currentwalkteststate == previouswalktestate)
  {
    /* They are the same */
    if ((currentwalkteststate == WT_SHORTCIRCUIT) &&  (g_faultflag[WT_FLTFLAG] == false))
    {
      /* __no_operation(); */
      if(sccnt < SC_CNT_DURATION)
      {
         sccnt = 0U;
         g_faultflag[WT_FLTFLAG] = true;
         UpdateHardwareFaultSentinel(CNT_WALKTEST_INPUT,true,g_WtestAdcVal);
      }
      /* check weather test has been carried out or not */
    }
    else if((g_faultflag[WT_FLTFLAG] == true) && (currentwalkteststate == WT_OPENCIRCUIT))
    {
      g_faultflag[WT_FLTFLAG] = false;
      UpdateHardwareFaultSentinel(CNT_WALKTEST_INPUT,false,0U);
    }
    else if ((wtrecordflag == false) && (currentwalkteststate != WT_OPENCIRCUIT)
             && (currentwalkteststate != WT_SHORTCIRCUIT))
    {
      /* go for test */
      wtrecordflag = true;   /* set flag */
      /* A valid test value is rcvd clear any fault if set */
      if(g_faultflag[WT_FLTFLAG] == true)
      {
        g_faultflag[WT_FLTFLAG] == false;
        UpdateHardwareFaultSentinel(CNT_WALKTEST_INPUT,false,0U);
      }
      
      RemoteTestTrigger(WT_SOURCE_EXTERNAL,currentwalkteststate);

      if ((PersonalityType() != CONVENT_MODE) && (g_enablechecksum == false))
      {
        switch (currentwalkteststate)
        {
        case WT_BUILTINTEST:
          printf_P("\r%d Wired ALARM",currentwalkteststate);
          break;
        case WT_RESET:
          printf_P("\r%d Wired RESET",currentwalkteststate);
          break;
        case WT_OPM:
          printf_P("\r%d Wired OPM",currentwalkteststate);
          break;
        default:
          break;
        }
      }
    }
    else
    {}
  }
  /* Save the walk test state */
  previouswalktestate = currentwalkteststate;
  /* reset flag if set and ckt is open now */
  if ((wtrecordflag == true) && (currentwalkteststate == WT_OPENCIRCUIT))
  {
    wtrecordflag = false; /* reset record flag flag */
    /* remove fault if set save machine cycles as well */
    if (g_faultflag[WT_FLTFLAG] == true)
    {
      g_faultflag[WT_FLTFLAG] = false;   /* reset flag */
      UpdateHardwareFaultSentinel(CNT_WALKTEST_INPUT,false,0U);
    }
  }
  else if (wtrecordflag == true)
  {
    /* /not open ckt after test, monitor the duration */
    countduration++;
    if (countduration == WAIT_ONE_MINUTE) /* (30000) 1 minute if loop time is 2 ms had passed ckt still not open raise fault */
    {
      countduration = RESET_WIRED_CNT;   /* reset countduration */
      /* avoid multiple entries if fault already set by using fault flag. */
      if (g_faultflag[WT_FLTFLAG] == false)
      {
        g_faultflag[WT_FLTFLAG] = true;
        /* RAISE FAULT HERE FOR FV400 */
        UpdateHardwareFaultSentinel(CNT_WALKTEST_INPUT,true,1U);
        /* printf_P("RELEASE SWITCH PLZ"); */
      }
    }
  }
  else
  {}
} /* void */

/********************************************************************//**
* @name RemoteTestTrigger()
* @brief 
* Number of seconds the operator must wait before retriggering a flame or
* OPM test. This is like a 'debounce'. In the case of dual lamp failure,
* it is possible for the operator to retrigger tests once a second. This
* causes the logs to fill.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void RemoteTestTrigger(BYTEu triggersource, BYTEu testtype)
{
  static BYTEu lasttest = WT_OPENCIRCUIT;
  /* Exit if walk test is not a valid test */
  if ((testtype == WT_SHORTCIRCUIT) || (testtype == WT_OPENCIRCUIT))
  {
    return;
  }
  /* Have we selected to fire off the same test within a short space of time? */
  if (testtype == lasttest)
  {
    /* This is the same test. Wait a suitable time to re arm */
    if ((timelasttest + REARM_SECONDS) >= GetSecondsSincePowerUp())
    {
      return;
    }
  }
  /* Save the type of test about to be carried out */
  lasttest = testtype;
  /* Blink the yellow LED to register an external walk test */
  if ((triggersource == WT_SOURCE_EXTERNAL) || (triggersource == WT_SOURCE_IR)
      || (triggersource == WT_SOURCE_MODBUS))
  {
    /* led flash ACK */
    __disable_interrupt();
    g_flag15v_on = true;
    PORTA |= (0x01U << FAULT_LED_PPIN);
    delayms(BLINK_DURATION_A);
    PORTA &= ~(0x01U << FAULT_LED_PPIN);   /* off */
    g_flag15v_off = true;
    __enable_interrupt();
  }
  /* No other tests are being run. Carry out requested function */
  switch (testtype)
  {
  case WT_BUILTINTEST:
    if (GetConvPcbMode() == true)
    {
      SignalAlarmForAlarmTest();
    }
    else
    {
      /* Log to FDE an alarm test has been triggered */
      FdeSelftest(WALKTEST_OCCURRED,triggersource,WT_BUILTINTEST);
      /* Force selftest */
      RamSelfTest(true);
      DceChecksumSelftest(true);
      /* Trigger a flame test */ 
      ElectricalAlarmTest(MANUAL_TEST_TRIGGER);
    }
    break;
  case WT_RESET:
    g_ResetButtonTrigger = RESET_BUTTON_LOG_VAL;   /* set for pyro log */
    /* Log to FDE an OPM test has been triggered */
    FdeSelftest(WALKTEST_OCCURRED,triggersource,WT_RESET);
    /* Clear latched fault or alarms */
    ClearLatchedStates();
    break;
  case WT_OPM:
    /* Log to FDE an OPM test has been triggered */
    FdeSelftest(WALKTEST_OCCURRED,triggersource,WT_OPM);
    /* Force a retrigger of the OPM test. */
    SetManualOpmTest(OPMTEST_MANUALACTIVE);
    RequestManualOpmTest(OPMTEST_OPMREQUESTED);
    ElectricalAlarmTest(NEW_MAN_SELF_TEST);
    SetManualOpmTest(OPMTEST_IDLE);
    break;
  case WT_RESERVED:
    break;
  default:
    break;
  }
  
 /* Do not flash End of test LED if detector is in Fault state */
  if ((GetDetectorStatus()->HardwareFaultState == NO_FAULT) && 
        (GetDetectorStatus()->OpmFaultState == WINDOW_CLEAN))
    {
        /* LED END of Test Indication double flash 20ms on - 200ms off - 20ms on */
         if ((triggersource == WT_SOURCE_EXTERNAL) || (triggersource == WT_SOURCE_IR)
            || (triggersource == WT_SOURCE_MODBUS))
        {
          /* led flash ACK */
           __disable_interrupt();
          g_flag15v_on = true;
          PORTA |= (0x01U << FAULT_LED_PPIN);
          delayms(BLINK_DURATION_A);
          __enable_interrupt();
          PORTA &= ~(0x01U << FAULT_LED_PPIN);   /* off */
          g_flag15v_off = true;
          ConvModeDelayMs(LED_OFF_TIME);
          /* led flash ACK */
          __disable_interrupt();
          g_flag15v_on = true;
          PORTA |= (0x01U << FAULT_LED_PPIN);
          delayms(BLINK_DURATION_A);
          PORTA &= ~(0x01U << FAULT_LED_PPIN);   /* off */
          g_flag15v_off = true;
          __enable_interrupt();
        }
    }
  
  /* Record the time of the selftest */
  timelasttest = GetSecondsSincePowerUp();
} /* void */

/********************************************************************//**
* @name WalkTestReminder()
* @brief
* This function is a timer for the walk test. If walk test is not
* carried out within the sepcified time then this will indicate the face
* on Video and possibly on yellow LED.
* Status : Under devlopment
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
/* Determine if a walk test reminder is required */
BYTEu WalkTestReminder(void)
{
  BYTEu retval = false;
  INT32u remindtime;
  if (timelasttest == 0x00000000UL)
  {
    /* Fist time run. Save timestamp */
    timelasttest = GetSecondsSincePowerUp();
  }
  else
  {
    /* Check if the number of requred seconds have elapsed */
    if (TagToConfigReadChar(VIDEOWTALERT) != 0x00U)
    {
      /* An alert is active. Has it expired? */
      remindtime = timelasttest + (60UL * 60UL * 24UL *
                                   (INT32u)TagToConfigReadChar(VIDEOWTALERT));
      /* Check if the reminder timer has expired */
      if (GetSecondsSincePowerUp() > remindtime)
      {
        /*
         * Expired. Flag the fact a walk test needs to be
         * carried out
         */
        retval = true;
      }
    }
  }
  return(retval);
}

/********************************************************************//**
* @name CheckControlProductId()
* @brief
* This function will read the controller board issue id and then the
* firmware version number. If they are not compatible then a fault would
* be raised.
* Status: Under development
*  @param updatefaultstate
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
BYTEu CheckControlProductId(BYTEu updatefaultstate)
{
  BYTEu retval = PASS, prodid;
  /* Get product resistor Identification */
  prodid = GetProductId();
  /*
   * Determine if this is not an old version of software running
   * on a newer controller board
   */
  if (GetSoftwareCompatibilityId() >= ISSUE_NOTSUPORTED)
  {
    /* Software does not support the current hardware platform */
    if (updatefaultstate)
    {
      UpdateHardwareFaultSentinel(CNT_EXCEPTION,true,
                                  EXP_SOFWAREHARDWARE_INCOMPATABLE);
      retval = FAIL;
    }
  }
  /* Check to see if the reserved bit is not fitted */
  if (prodid & RESERVED_BIT_CONST)
  {
    if (updatefaultstate != false)
    {
      UpdateHardwareFaultSentinel(CNT_EXCEPTION,true,EXP_INVALIDPRODUCTID);
    }
    retval = FAIL;
  }
  return(retval);
}

/********************************************************************//**
* @name MoistureLevelTest();
* @brief
* Read MUX1, Y6. If moisture reading is in between the 169 to 226
* then its under normal limit else fault with different ids.
* Recover the fault with a fixed ID=0. Make sure EOL resistor
* is connected.
*  @param reqst
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
BYTEu MoistureLevelTest(BYTEu reqst)
{
  static BYTEu moisthresh = false;
  BYTEu outcome = true;

  if (moisthresh == false)
  {
    moisthresh = TagToConfigReadChar(MOISTURETHRESH);
  }

  if ((g_mux1[MOISTURE_SENS] < moisthresh) && (g_faultflag[MOIST_FLTFLAG] == false))
  {
    g_faultflag[MOIST_FLTFLAG] = true;
    if (reqst == REQ_LOOP_TEST)
    {
      /* raise fault only if configured to do so */
      if (TagToConfigReadChar(MOISTURESW) != false)
      {
        UpdateHardwareFaultSentinel(CNT_MOISTUREFAULT,true,1U);
      }
      FdeSelftest(MOISTUREFAULT, g_mux1[MOISTURE_SENS],0U);
    }
    else
    {
      outcome = false;
    }
  }
  else if ((g_faultflag[MOIST_FLTFLAG] == false) && (g_mux1[MOISTURE_SENS] > MOISTURE_THRESHX))
  {
    g_faultflag[MOIST_FLTFLAG] = true;
    if (reqst == REQ_LOOP_TEST)
    {
      if (TagToConfigReadChar(MOISTURESW) != false)
      {
        UpdateHardwareFaultSentinel(CNT_MOISTUREFAULT,true,2U);
      }
      FdeSelftest(MOISTUREFAULT, g_mux1[MOISTURE_SENS],0U);
    }
    else
    {
      outcome = false;
    }
  }
  /* remove fault is set previously but now OK... */
  else if ((g_faultflag[MOIST_FLTFLAG] != false) && ( (g_mux1[MOISTURE_SENS] >= moisthresh) &&  
                                        (g_mux1[MOISTURE_SENS] <= MOISTURE_THRESHX)))
  {
    g_faultflag[MOIST_FLTFLAG] = false;
    UpdateHardwareFaultSentinel(CNT_MOISTUREFAULT,false,0U);
    FdeSelftest(MOISTUREFAULT, g_mux1[MOISTURE_SENS],0U);
  }
  else
  {}
  return(outcome);
}

/********************************************************************//**
* @name AlarmRelayTest();
* @brief
* Only run the test when the detector is configured for relay operation.
* Note that the relay monitor line is biased from the fault relay drive
* and will only return valid values when the fault relay is energised,
* ie the 'no fault' state. It follows that if the detector is already in
* fault then there is no point in carrying out the alarm relay test.
*
* Once alarm relay fault is set it won't come out by itself even in
* non latching because fault relay is off. Only a reset can take it
* out.
*
* To run the test, check that the fault relay is energised then read
* Read MUX3, Y7
*
* Commanded alarm relay state	MUX3, Y7	Required action
* ON	>250	None - response is correct
* ON	<=250	Alarm relay fault type 1
* OFF	either	None - response is correct
*
* Relay 'on':
* 'OK' band was 71-116. Change to two bands, 42-118 and 200-240.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void AlarmRelayTest(void)
{
  static BYTEu relayprevstate = false,relaycurrentstate = false,
                       localwaitcnt = 0U,faultrelstate = true;
  /* don't carry out the test if any of following i/f are not selected. */
  if ((PersonalityType() == DEFVAR_4_20MA_MODE) ||
      (PersonalityType() == DEFAULT_4_20MA_MODE))
  {

    /* don't carry out test if fault relay in OFF */
    if ((PORTB & FAULTRELAY_PIN) == false)
    {
      faultrelstate = false;
      return;
    }

    /* find out alarm relay state from alarm relay pin */
    if (PORTD & ALARMRELAY_PIN)
    {
      relaycurrentstate = true;
    }
    else
    {
      relaycurrentstate = false;
    }

    if ((relaycurrentstate != relayprevstate) || (faultrelstate == false))
    {
      localwaitcnt = WAITDURATION;
      relayprevstate = relaycurrentstate;
      faultrelstate = true;
      return;
    }

    relayprevstate = relaycurrentstate;
    if (localwaitcnt)
    {
      localwaitcnt--;
      return;
    }

    if (g_faultflag[ALMREL_FLTFLAG] == false)
    {
      /* carry on with HM */
      if (relaycurrentstate != false)
      {
        /* alarm relay is ON */
        if ((g_mux3[ALARM_RELAY_MON] < ALM_REL_ON_LOW_LIM) || (g_mux3[ALARM_RELAY_MON] >= ALM_REL_ON_UPR_LIM))
        {
          g_faultflag[ALMREL_FLTFLAG] = true;
          UpdateHardwareFaultSentinel(CNT_ALARMRELAYFAULT,true,1U);
          FdeSelftest(ALARMRELAYFLT, g_mux3[ALARM_RELAY_MON],0U);
        }
        else if ((g_mux3[ALARM_RELAY_MON] >= ALM_REL2_ON_LIM_X) && (g_mux3[ALARM_RELAY_MON] < ALM_REL2_ON_LIM_Y))
        {
          g_faultflag[ALMREL_FLTFLAG] = true;
          UpdateHardwareFaultSentinel(CNT_ALARMRELAYFAULT,true,2U);
          FdeSelftest(ALARMRELAYFLT, g_mux3[ALARM_RELAY_MON],0U);
        }
        else
        {}
      }
      else
      {
        /* alm relay is off */
        if (g_mux3[ALARM_RELAY_MON] > ALM_REL2_ON_LIM_Y)     /* fault fault */
        {
          g_faultflag[ALMREL_FLTFLAG] = true;
          UpdateHardwareFaultSentinel(CNT_ALARMRELAYFAULT,true,3U);
          FdeSelftest(ALARMRELAYFLT, g_mux3[ALARM_RELAY_MON],0U);
        }
        else if (g_mux3[ALARM_RELAY_MON] < ALM_REL3_LIM)
        {
          g_faultflag[ALMREL_FLTFLAG] = true;
          UpdateHardwareFaultSentinel(CNT_ALARMRELAYFAULT,true,4U);
          FdeSelftest(ALARMRELAYFLT, g_mux3[ALARM_RELAY_MON],0U);
        }
        else
        {}
      }

    }
    else
    {
      /* remove faults here, but detector is in fault so fltrelay would be off?? */
      if ((relaycurrentstate == false) && (g_mux3[ALARM_RELAY_MON] <= ALM_REL2_ON_LIM_Y) 
          && (g_mux3[ALARM_RELAY_MON] >= ALM_REL3_LIM))
      {
        /* relay off, I am getting 139 for this situation */
        g_faultflag[ALMREL_FLTFLAG] = false;
        UpdateHardwareFaultSentinel(CNT_ALARMRELAYFAULT,false,0U);
        FdeSelftest(ALARMRELAYFLT, g_mux3[ALARM_RELAY_MON],0U);
      }
      else if (((g_mux3[ALARM_RELAY_MON] < ALM_REL4_LIM) &&  (g_mux3[ALARM_RELAY_MON] >= ALM_REL_ON_LOW_LIM)) ||
               ((g_mux3[ALARM_RELAY_MON] < ALM_REL_ON_UPR_LIM) && (g_mux3[ALARM_RELAY_MON] >= ALM_REL2_ON_LIM_Y)))
      {
        /* relay on, */
        g_faultflag[ALMREL_FLTFLAG] = false;
        UpdateHardwareFaultSentinel(CNT_ALARMRELAYFAULT,false,0U);
        FdeSelftest(ALARMRELAYFLT, g_mux3[ALARM_RELAY_MON],0U);
        /* fault relay off */
      }
      else
      {}
    }
  }
} /* void */

/********************************************************************//**
* @name FourPFourLevelMonitorTest();
* @brief
* If greater than 237d then enter a 4v4 error ID=1. If
* below 222d then enter a 4v4 error with ID=2.
*
* These values are calculated by DR after quite a struggle.
*
* I am increasing the upper limit to 242, as the normal value is
* read as 235 on minitor channel which is quite close to upper limit.
*
* original upper limit=237, new =242
*
* Modification may be required for some monitoring time.
*
* Sep 2011
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void FourPFourLevelMonitorTest(void)
{
  BYTEu lcltempstore = g_mux3[_4V4_MONITOR];
  /* printf_P("four=%d\r",calibratedfourpfourlevel); */
  if (g_faultflag[FOUTPFOUR_FLTFLAG] == false)
  {
    if (lcltempstore <= FOUR_P_FOUR_MIN) /* was 222 then 216//lower 219 in 2012 */
    {
      g_faultflag[FOUTPFOUR_FLTFLAG] = true;
      UpdateHardwareFaultSentinel(CNT_4V4SUPPLY_FAULT,true,1U);
      FdeSelftest(_4V4SUPPLY_FAULT,lcltempstore,0U);
    }
    else if (lcltempstore > FOUR_P_FOUR_MAX) /* was 237 then 251//upper 247 2012 */
    {
      g_faultflag[FOUTPFOUR_FLTFLAG] = true;
      UpdateHardwareFaultSentinel(CNT_4V4SUPPLY_FAULT,true,2U);
      FdeSelftest(_4V4SUPPLY_FAULT,lcltempstore,0U);
    }
    else
    {}
    return;
  }
  else
  {
    /* remove fault if all OK .... */
    if ((lcltempstore > FOUR_P_FOUR_MIN) && (lcltempstore <= FOUR_P_FOUR_MAX))
    {
      g_faultflag[FOUTPFOUR_FLTFLAG] = false;
      UpdateHardwareFaultSentinel(CNT_4V4SUPPLY_FAULT,false,0U);
      FdeSelftest(_4V4SUPPLY_FAULT,lcltempstore,0U);
    }
  }
}

/********************************************************************//**
*  @name FourToTwentyLoopMonitor()
*  @brief
*  Following function is carefully designed as 4TO20 loop can
*  flactuate during emc or surge. So a suitable monitoring time
*  is there before detector is put in to fault. If current value
*  is changed recently (OCR0A PWM comp val) then it will miss
*  next 20 attempts in order to give some stablization time for
*  monitor ckt.
*
*  //new spec
*  For the current build standard:
*  ADC value (8-bit) needs to be = (8-bit value fed to the PWM)±2
*
*  Where:
*  N is the number of bits for the drive level (assumed to be 8)
*  M is the number of bits for reading MUX1, Y7 (assumed to be 8)
*
*  Thus if M=N then the returned value should be the same as the written value.
*
*  For the alternative build standard:
*
*  ADC value needs to be within ± 7% of 2 x (8-bit value fed to the PWM)
*  @param reqst
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
BYTEu FourToTwentyLoopMonitor(BYTEu reqst)
{
  BYTEu currentreqlevel;
  BYTEu offset4_20 = DEFAULT_OFFSET_X,outcome = true;
  static BYTEu ignorecnt = 0U,
                       prevtimerval = SET_NORMAL,localsettle = 0U;
  /* Disable monitor all togeather as suggested by Dave */
  if (reqst == REQ_LOOP_TEST)
  {
   return(true);
  }
  
  /* don't carry out the test if any of following i/f is selected. */
  if ((PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE)
      || (PersonalityType() == CONVENT_MODE))
  {
    return(true);
  }

  currentreqlevel = g_timer4_20val;
  /* give some time to settle if timer value is recently changed */
  if (currentreqlevel != prevtimerval)
  {
    prevtimerval = currentreqlevel;
    localsettle = STABLIZATION_DURATION;
    return(true);
  }
  /* store previous value */
  prevtimerval = currentreqlevel;

  if (localsettle)
  {
    localsettle--;
    return(true);
  }
  /*
   * printf_P("\r actl=%d,req=%d ",g_mux1[_4_20MA_MON],currentreqlevel);
   * set offset first
   */
  if (g_timer4_20val > SET_11mA)
  {
    offset4_20 = OFFSET_GREATER_11MA; /* was 8 */
  }
  else if (g_timer4_20val > SET_NORMAL)
  {
    offset4_20 = OFFSET_BELOW_11MA; /* was 5 */
  }
  else
  {
    offset4_20 = OFFSET_AROUND_4MA; /* was 2 */
  }
  /* chk fault state */
  if ((g_mux1[_4_20MA_MON] < FAULT_SIGNAL_THRESHX) /* && (GetDetectorStatus()->HardwareFaultState) */
      && (g_faultflag[FOURTOTWENTY_FLTFLAG]))
  {
    return(true);
  }
  /* check levels report if fault... */
  if (!g_faultflag[FOURTOTWENTY_FLTFLAG])
  {
    if (g_mux1[_4_20MA_MON] > (currentreqlevel + offset4_20))
    {
      ignorecnt++;
      if (ignorecnt > HYSTERESIS_LOOP)
      {
        g_faultflag[FOURTOTWENTY_FLTFLAG] = true;
        if (reqst == REQ_LOOP_TEST)
        {
          UpdateHardwareFaultSentinel(CNT_4_20ERROR,true,2U);
          FdeSelftest(_4_20ERROR,g_mux1[_4_20MA_MON],currentreqlevel);
        }
        else
        {
          outcome = false;
        }
      }
    }
    else if (g_mux1[_4_20MA_MON] < (currentreqlevel - offset4_20))
    {
      ignorecnt++;
      if (ignorecnt > HYSTERESIS_LOOP)
      {
        g_faultflag[FOURTOTWENTY_FLTFLAG] = true;
        if (reqst == REQ_LOOP_TEST)
        {
          UpdateHardwareFaultSentinel(CNT_4_20ERROR,true,1U);
          FdeSelftest(_4_20ERROR,g_mux1[_4_20MA_MON],currentreqlevel);
        }
        else
        {
          outcome = false;
        }
      }
    }
    else
    {
      ignorecnt = RESET_CNT_HM;
    }
  }
  else if ((g_mux1[_4_20MA_MON] <= (currentreqlevel + offset4_20))
           && (g_mux1[_4_20MA_MON] >= (currentreqlevel - offset4_20)))
  {
    UpdateHardwareFaultSentinel(CNT_4_20ERROR,false,0U);
    FdeSelftest(_4_20ERROR,g_mux1[_4_20MA_MON],0U);
    g_faultflag[FOURTOTWENTY_FLTFLAG] = false;
    ignorecnt = RESET_CNT_HM;
  }
  else
  {}
  return(outcome);
} /* void */

/********************************************************************//**
* @name _24V_LineMon();
*  @brief
* Well decision depends on interface type, if detector is
* configured to be in MX interface then limits are different. First
* just set the limits then calibrate and then check... thats it.
*
* step size 1v=4.65 adc, oct 2012
*
* some monitoring time required AR Sep 2011
* limits changed 26 sep 2011
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void _24V_LineMon(void)
{
  BYTEu upprlim = DEF_SUPPLY_UPPER_LIM,lowrlim = DEF_SUPPLY_LOWER_LIM, linefltid = 3U;
 //set limits
  if(PersonalityType() == CONVENT_MODE)
  {
   upprlim = CONVORREL_UPPER_LIM;
   lowrlim = CONV_LOWER_LIM;
   linefltid = 5U;
  }
  else if((PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE))
  {
   upprlim = MX_UPPER_SUPPLY_LIM;
   lowrlim = MX_LOWER_SUPPLY_LIM;
   linefltid = 1U;
  }
  else
  {}
  // carry out test
  if (g_mux3[_24VRAIL_MON] > upprlim)
  {
    if (!g_faultflag[TWENTYFOUR_FLTFLAG])
    {
      g_faultflag[TWENTYFOUR_FLTFLAG] = true;
      UpdateHardwareFaultSentinel(CNT_SUPPLYERROR,true,linefltid);
      FdeSelftest(SUPPLYERROR,g_mux3[_24VRAIL_MON],linefltid);
    }
  }
  else if (g_mux3[_24VRAIL_MON] < lowrlim)
  {
    if (!g_faultflag[TWENTYFOUR_FLTFLAG])
    {
      g_faultflag[TWENTYFOUR_FLTFLAG] = true;
      //UpdateHardwareFaultSentinel(CNT_SUPPLYERROR,true,2U);
      FdeSelftest(SUPPLYERROR,g_mux3[_24VRAIL_MON],linefltid);
    }
  }
  else if ((g_faultflag[TWENTYFOUR_FLTFLAG] == true) && (g_mux3[_24VRAIL_MON] > (lowrlim + SUPPLY_HYS)) 
           && (g_mux3[_24VRAIL_MON] < (upprlim - SUPPLY_HYS)))
  {
    /* normal remove fault */
    g_faultflag[TWENTYFOUR_FLTFLAG] = false;
    UpdateHardwareFaultSentinel(CNT_SUPPLYERROR,false,0U);
    FdeSelftest(SUPPLYERROR,g_mux3[_24VRAIL_MON],0U);
  }
  else
  {}
}

/********************************************************************//**
* @name _15V_HealthMon();
* @brief
* MinDifference = 1960 counts/sec, so for 1sec check interval, MinDifference = 1960d
* MaxDifference = 5840 counts/sec, so for 1sec check interval MinDifference = 5840d
* MinFinalLevel = 22000d
* MaxFinalLevel = 31500d
*
* Scale the raw ADC value by 256.
*
* NOTE: Fault must not be set or cleared in this rutine as this routine
* is a part of ISR, this routine will flag faults to a global variable which is
* accessed by another function _15V_HealthMonCheck() and is a part of
* main loop.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void _15V_HealthMonUpdate(void)
{
  static INT16u fastcharge,slowcharge;
  static BYTEu firstentryflag = true;
  /* printf_P("fs=%d\r", g_mux3[_24VRAIL_MON]); */
  if ((PersonalityType() == CONVENT_MODE) &&
      (g_mux3[_24VRAIL_MON] < SUPPLY_THRESH_FOR_CONVMODE))
  {
    return;
  }

  if (firstentryflag)
  {
    firstentryflag = false;
    fastcharge = g_mux3[_15V_RAIL_MON];
    slowcharge = g_mux3[_15V_RAIL_MON];
    return;
  }

  if (g_flag15v_on == true)
  {
    fastcharge = (INT16u)(g_mux3[_15V_RAIL_MON] * SCALING_FACT_15V);           /* {[1] - see above} */
    slowcharge = fastcharge;
  }
  else
  {
    if (fastcharge + MaxDifference  <  MaxFinalLevel)
    {
      fastcharge += MaxDifference;
    }
    else
    {
      fastcharge = MaxFinalLevel;
    }
    if (slowcharge + MinDifference  <  MinFinalLevel)
    {
      slowcharge += MinDifference;
    }
    else
    {
      slowcharge = MinFinalLevel;
    }
  }

  if (g_flag15v_off == true)
  {
    g_flag15v_on = false;
    g_flag15v_off = false;

    /*
     * check for fault do not update or clear fault in this function
     * as this is part of ISR just put fault state in a statioc variable
     */
    if ((INT16u)(g_mux3[_15V_RAIL_MON] * SCALING_FACT_15V) > fastcharge)
    {
      /* g_fifteenvoltfltstate = FAULT_TYPE_TWO; */
    }
    else if ((INT16u)(g_mux3[_15V_RAIL_MON] * SCALING_FACT_15V) < slowcharge)
    {
      /* g_fifteenvoltfltstate = FAULT_TYPE_ONE; */
    }
    else
    {
      g_fifteenvoltfltstate = NO_15V_FAULT;
    }

  }
  /*
   * slchg=slowcharge;
   * fschg=fastcharge;
   */
} /* void */

/********************************************************************//**
* @name _15V_HealthMonCheck();
* @brief
* This function is just fault update function as rest of the calculations
* are done within the ISR but it's not a good idea to write the fault to
* FDE from inside the ISR.
*  @param reqst
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/

BYTEu _15V_HealthMonCheck(BYTEu reqst)
{
  static BYTEu resvfltflg = false;
  BYTEu outcome = true;

  if ((PersonalityType() == CONVENT_MODE) &&
      (g_mux3[_24VRAIL_MON] < SUPPLY_THRESH_FOR_CONVMODE))
  {
    return(true);
  }

  if ((g_fifteenvoltfltstate == FAULT_TYPE_ONE) && (resvfltflg == false))
  {
    /* printf_P("fault 1\r"); */
    resvfltflg = true;
    if (reqst == REQ_LOOP_TEST)
    {
      UpdateHardwareFaultSentinel(CNT_RESERVOIRFLT,true,1U);
      FdeSelftest(RESERVOIRFLT,g_mux3[_15V_RAIL_MON],0U);
    }
    else
    {
      outcome = false;
    }
  }
  else if ((g_fifteenvoltfltstate == FAULT_TYPE_TWO) && (resvfltflg == false))
  {
    /* printf_P("fault 2\r"); */
    resvfltflg = true;
    if (reqst == REQ_LOOP_TEST)
    {
      UpdateHardwareFaultSentinel(CNT_RESERVOIRFLT,true,2U);
      FdeSelftest(RESERVOIRFLT,g_mux3[_15V_RAIL_MON],0U);
    }
    else
    {
      outcome = false;
    }
  }
  else if ((resvfltflg != false) && (g_fifteenvoltfltstate == NO_15V_FAULT))
  {
    /* printf_P("No fault\r"); */
    resvfltflg = false;
    UpdateHardwareFaultSentinel(CNT_RESERVOIRFLT,false,0U);
    FdeSelftest(RESERVOIRFLT,g_mux3[_15V_RAIL_MON],0U);
  }
  else
  {}
  return(outcome);
} /* void */

/********************************************************************//**
* @name void CheckBiasError();
* @brief
*  Sets Bias error is flag is set within GetAdctenBitVal().
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
void CheckBiasError(void)
{
  /* check for any faults as well, dont worry this func just reads flag */
  /*vreffaultstate = (BYTEu)GetAdcTenBitCalValue(CH5_FLAME_BIAS,true);
  if ((vreffaultstate != false) && (g_faultflag[VREF_FLTFLAG] == false))
  {
    //set fault 
    UpdateHardwareFaultSentinel(CNT_VREF_ERROR,true,vreffaultstate);
    g_faultflag[VREF_FLTFLAG] = true;
  }
  else if (g_faultflag[VREF_FLTFLAG] && !vreffaultstate)
  {
    // remove fault 
    g_faultflag[VREF_FLTFLAG] = false;
    UpdateHardwareFaultSentinel(CNT_VREF_ERROR,false,0U);
  }
  else
  {}*/
}

/********************************************************************//**
* @name FiveVoltMonitor()
* @brief
* Monitors 5.25V monitor line and if goes outside defined limits then 
* put detector into fault mode.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void FiveVoltMonitor(void)
{
  BYTEu lcltmpstr = g_mux3[FIVEVOLT_MONITOR];
  /* printf_P("four=%d\r",calibratedfourpfourlevel); */
  if (!g_faultflag[FIVEV_FLTFLAG])
  {
    if (lcltmpstr <= FIVE_VOL_MIN)
    {
      g_faultflag[FIVEV_FLTFLAG] = true;
      UpdateHardwareFaultSentinel(CNT_FIVE_VOLT,true,1U);
      FdeSelftest(FIVEVOLTRAILFLT,lcltmpstr,0U);
    }
    else if (lcltmpstr > FIVE_VOL_MAX)
    {
      g_faultflag[FIVEV_FLTFLAG] = true;
      UpdateHardwareFaultSentinel(CNT_FIVE_VOLT,true,2U);
      FdeSelftest(FIVEVOLTRAILFLT,lcltmpstr,0U);
    }
    else
    {}
    return;
  }
  else
  {
    /* remove fault if all OK .... */
    if ((lcltmpstr > FIVE_VOL_MIN) && (lcltmpstr <= FIVE_VOL_MAX))
    {
      g_faultflag[FIVEV_FLTFLAG] = false;
      UpdateHardwareFaultSentinel(CNT_FIVE_VOLT,false,0U);
      FdeSelftest(FIVEVOLTRAILFLT,lcltmpstr,0U);
    }
  }
}

/********************************************************************//**
* @name TemperatureFaultMon();
* @brief
* This function is called in temperaturesentinal.c.
*
* Status: Might need improvement.
*
* normal limits are -43 to 84, If I use -40 to 80 then
* detector ll enter to an equlibrium state during oven test..
* I don't want to fill up logs..
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
void TemperatureFaultMon(void)
{
  if (g_faultflag[TEMP_FLTFLAG] == false)
  {
    if ((g_TemperatureThmstr > TEMP_FLT_MAX) && (g_TemperatureThmstr < TEMP_SC_MAX)) /* greater than 80C but below 100C### PUT 7999 in final code AR APRIL 2011 */
    {
      UpdateHardwareFaultSentinel(CNT_TEMPERATURE,true,2U);
      FdeSelftest(TEMPERATURE,(BYTEu)(g_TemperatureThmstr >> SHIFT_BY_BYTE_S),0U);
      g_faultflag[TEMP_FLTFLAG] = true;
    }
    else if ((g_TemperatureThmstr < TEMP_FLT_MIN) && (g_TemperatureThmstr > TEMP_SC_MIN)) /* below -43C but above -50C */
    {
      UpdateHardwareFaultSentinel(CNT_TEMPERATURE,true,3U);
      FdeSelftest(TEMPERATURE,(BYTEu)(g_TemperatureThmstr >> SHIFT_BY_BYTE_S),0U);
      g_faultflag[TEMP_FLTFLAG] = true;
    }
    else if (g_TemperatureThmstr > TEMP_SC_MAX) /* above 100C which implies SC or OC */
    {
      UpdateHardwareFaultSentinel(CNT_TEMPERATURE,true,1U);
      FdeSelftest(TEMPERATURE,(BYTEu)(g_TemperatureThmstr >> SHIFT_BY_BYTE_S),0U);
      g_faultflag[TEMP_FLTFLAG] = true;
    }
    else if (g_TemperatureThmstr < TEMP_SC_MIN) /* below -50C which implies SC or OC */
    {
      UpdateHardwareFaultSentinel(CNT_TEMPERATURE,true,4U);
      FdeSelftest(TEMPERATURE,(BYTEu)(g_TemperatureThmstr >> SHIFT_BY_BYTE_S),0U);
      g_faultflag[TEMP_FLTFLAG] = true;
    }
    else
    {}
  }
  else if (((g_TemperatureThmstr < (TEMP_FLT_MAX - TEMP_HYSTRESYS)) 
            && (g_TemperatureThmstr > (TEMP_FLT_MIN + TEMP_HYSTRESYS))) &&
           (g_faultflag[TEMP_FLTFLAG]))
  {
    /* all ok temp within limits */
    g_faultflag[TEMP_FLTFLAG] = false;
    UpdateHardwareFaultSentinel(CNT_TEMPERATURE,false,0U);
    FdeSelftest(TEMPERATURE,(BYTEu)(g_TemperatureThmstr >> SHIFT_BY_BYTE_S),0U);
  }
  else
  {}
}

/********************************************************************//**
* @name DipSwitchFaultMon()
* @brief
* Called only at powerup just before Main Loop.
* This fault cannot be cleared by main loop, only Wt300/Wired/powercycle 
* Can clear it.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void DipSwitchFaultMon(void)
{
  if(DipSwitchFaultState() != DIP_ALL_OK)
  {
   UpdateHardwareFaultSentinel(CNT_DIPSW,true,DipSwitchFaultState());
  }
  else
  {
    __no_operation();
  }
}
/********************************************************************//**
* @name AlarmTestReq()
* @brief 
*  Top level alarm test function for main application.
*  Calls the bulb flashing function followed by decesion function.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
BYTEu AlarmTestReq(BYTEu typofreq)
{
  BYTEu resalm = false;
  //UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,false,0U);
  SetFlameTest(FLAMETEST_WAITALARMON);
  WindowLampPulseFp(LAMP_2);
  resalm = MakeAlarmTestDecesion(typofreq);
  SetFlameTest(FLAMETEST_IDLE);
  return(resalm);
}

/********************************************************************//**
* @name MakeAlarmTestDecesion()
* @brief
*  Makes alarm test decesion and reports accordngly.
* Score = [Cal * {(0.75 * AvgPkPk) + 17}]/256
* Nov 2014 Modified.AR
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
BYTEu MakeAlarmTestDecesion(BYTEu typofsrc)
{
  static BYTEu bperiodicelecounter = ELE_CNT_RESET;
  INT16u alarmcalcgu,alarmcalcfl,flgudiff,tempavgpkgu,tempavgpkfl;
  INT16u daveC4gu,daveC2gu,daveC4fl,daveC2fl,alarmarrayindex,calctempstore;

  /* for gu */
  /* tempavgpk = (INT16u)GetCalculatedLampAvg(LAMP_2_GU);
  tempavgpkgu = tempavgpk;
  alarmcalcgu = TagToConfigReadInt(TESTLAMPCALGU) / ALMTEST_CALCDIVCONST2;

   [(0.75 * AvgPkPk) + 17] 
  tempavgpk = ALMTEST_MULCONST * tempavgpk;
  tempavgpk = (tempavgpk / ALMTEST_DIVCONST) + ALMTEST_ADDCONST;
   Cal * [(0.75 * AvgPkPk) + 17] 
  freestore = alarmcalcgu * tempavgpk;
  alarmcalcgu = freestore / ALMTEST_CALCDIVCONST1; */
  //GET AVG GU
  tempavgpkgu = (INT16u)GetCalculatedLampAvg(LAMP_2_GU);
  daveC4gu = TagToConfigReadInt(TESTLAMPCALGU);
  if (daveC4gu < ALARM_TEST_OFFSET){
     daveC4gu = 0U;
  }
  else
  {
     daveC4gu = daveC4gu - ALARM_TEST_OFFSET;
     if(daveC4gu>((ALARM_TEST_TABLE_SIZE - 1U)*ALARM_TEST_DIVISOR)){
        daveC4gu = (ALARM_TEST_TABLE_SIZE - 1U)*ALARM_TEST_DIVISOR;
     }
  }
  
  alarmarrayindex = daveC4gu / ALARM_TEST_DIVISOR;
  daveC2gu = g_AlarmLampLookUp[alarmarrayindex];
  if (ReturnDebugMenuState() == true)
  {
   printf_P("GU INDEX = %d, C2Gu = %d \r",alarmarrayindex,daveC2gu);
  }
  daveC2gu = daveC2gu + daveC4gu;
  calctempstore= alarmarrayindex*ALARM_TEST_DIVISOR; 
  calctempstore= calctempstore + (ALARM_TEST_DIVISOR/ALMTEST_CALCDIVCONST2);
  daveC2gu = daveC2gu - calctempstore;
  //score
  alarmcalcgu = (daveC2gu / SCALING_ALARM_BULB_2);
  alarmcalcgu = alarmcalcgu * tempavgpkgu;
  alarmcalcgu = alarmcalcgu / SCALING_ALARM_BULB_1;
  /* for fla & flb */
  /* tempavgpk = (INT16u)(GetCalculatedLampAvg(LAMP_2_FLA) + GetCalculatedLampAvg(LAMP_2_FLB));
  tempavgpk = tempavgpk / ALMTEST_CALCDIVCONST2;
  tempavgpkfl = tempavgpk;
  alarmcalcfl = TagToConfigReadInt(TESTLAMPCALFL) / ALMTEST_CALCDIVCONST2;

  tempavgpk = ALMTEST_MULCONST * tempavgpk;
  tempavgpk = (tempavgpk / ALMTEST_DIVCONST) + ALMTEST_ADDCONST;

  freestore = alarmcalcfl * tempavgpk;
  alarmcalcfl = freestore / ALMTEST_CALCDIVCONST1; */

  //GET AVG FL
  tempavgpkfl = (INT16u)(GetCalculatedLampAvg(LAMP_2_FLA) + GetCalculatedLampAvg(LAMP_2_FLB));
  tempavgpkfl = tempavgpkfl / ALMTEST_CALCDIVCONST2;
  
  daveC4fl = TagToConfigReadInt(TESTLAMPCALFL);
  if (daveC4fl < ALARM_TEST_OFFSET){
     daveC4fl = 0U;
  }
  else
  {
     daveC4fl = daveC4fl - ALARM_TEST_OFFSET;
     if(daveC4fl>((ALARM_TEST_TABLE_SIZE - 1U)*ALARM_TEST_DIVISOR)){
        daveC4fl = (ALARM_TEST_TABLE_SIZE - 1U)*ALARM_TEST_DIVISOR;
     }
  }
  
  alarmarrayindex = daveC4fl / ALARM_TEST_DIVISOR;
  daveC2fl = g_AlarmLampLookUp[alarmarrayindex];
  if (ReturnDebugMenuState() == true)
  {
   printf_P("FL INDEX = %d, C2Fl = %d \r",alarmarrayindex,daveC2fl);
  }
  daveC2fl = daveC2fl + daveC4fl; 
  calctempstore = alarmarrayindex*ALARM_TEST_DIVISOR; 
  calctempstore= calctempstore + (ALARM_TEST_DIVISOR/ALMTEST_CALCDIVCONST2);
  daveC2fl = daveC2fl - calctempstore; 
  //score
  alarmcalcfl = (daveC2fl / SCALING_ALARM_BULB_2); 
  alarmcalcfl = alarmcalcfl * tempavgpkfl;
  alarmcalcfl = alarmcalcfl / SCALING_ALARM_BULB_1;
  
  /* difference */
  if (alarmcalcfl >= alarmcalcgu)
  {
    flgudiff = alarmcalcfl - alarmcalcgu;
  }
  else
  {
    flgudiff = alarmcalcgu - alarmcalcfl;
  }
  /* Exit if its a debug request */
  if (ReturnDebugMenuState() == true)
  {
    
    printf_P("FL VAL=%d FLAVG=%d, GU VAL=%d GUAVG=%d, Diff=%d \r", 
             alarmcalcfl,tempavgpkfl,alarmcalcgu,tempavgpkgu,flgudiff);
    printf_P("Confugured Thresholds: FL/GU MAX=%d FL/GU MIN=%d, DIFF LIM=%d \r", 
             TagToConfigReadInt(ALARMTESTFLGUMAX),TagToConfigReadInt(ALARMTESTFLGUMIN),TagToConfigReadInt(ALARMTESTLEVEL));
    
    if((TagToConfigReadInt(TESTLAMPCALGU) == WIN_CAL_DEFAULT) && (TagToConfigReadInt(TESTLAMPCALFL) == WIN_CAL_DEFAULT)){
       //printf_P("Calculated FL CAL=%d and GU Cal=%d \r", CAL_CALCULATE_FACTX/alarmcalcfl,CAL_CALCULATE_FACTX/alarmcalcgu);
    }
   return((BYTEu)(true));
  }
  
  /* log event */
  FdeAlarmTestResult(alarmcalcfl,tempavgpkfl,alarmcalcgu,
                     tempavgpkgu,LAMP_2,GetDetectorTemperature() / CONVERT_TO_DEG,(PORTG & GAIN_PING0),
                     GetAccumulatedValue(REQ_EAUTO),GetAccumulatedValue(REQ_ECROSS),
                     g_mux1[MOISTURE_SENS]);
  /* compair thresholds */
 // tempavgpk = false;
  if ((alarmcalcfl >= TagToConfigReadInt(ALARMTESTFLGUMIN)) &&
      (alarmcalcfl < TagToConfigReadInt(ALARMTESTFLGUMAX)) && (alarmcalcgu >= TagToConfigReadInt(ALARMTESTFLGUMIN))
      && (alarmcalcgu < TagToConfigReadInt(ALARMTESTFLGUMAX)) && (flgudiff < TagToConfigReadInt(ALARMTESTLEVEL)))
  {
    /* alarm condition met */
    UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,false,0U);
    /*  tempavgpk = true; */
    if (typofsrc == MZX_PANEL_TRIGGER)
    {
      return(true);
    }
    else if ((typofsrc != TIMED_SELF_TEST) && (typofsrc != NEW_MAN_SELF_TEST))
    {
      SignalAlarmForAlarmTest();
    }
    else{}
  }
  else if (typofsrc == MZX_PANEL_TRIGGER)
  {
    /* set fault */
    UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,true,1U);
    return(false);
  }
  else
  {
    if (typofsrc != TIMED_SELF_TEST)
    {
      UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,true,0U);
    }
    else
    {
       /* periodic test failure */
      bperiodicelecounter++;
      if(bperiodicelecounter>=ELE_CNT_LIMIT)
      {
        bperiodicelecounter=ELE_CNT_RESET;  
        UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,true,2U);
      }
    }
  }
  /* let alarm relay settle */
  ModbusDelayms(RELAY_SETTLE_DEL);

  return((BYTEu)(false));
}

/********************************************************************//**
* @name ElectricalAlarmTest()
* @brief
* This is only ment for conventional mode so no video indication
* required.
*
* The test signal is a square wave so (in low gain) the amplifier
* outputs will briefly go into saturation after each edge then come
* out of saturation just before the next edge. We can use these FV400
* key behaviours to advantage especially if we drop the test frequency
* to 1Hz. This makes sure that the amplifiers are out of saturation
* prior to each switching edge.
*
* Therefore the first job is to change from 2Hz to 1Hz. Ideally the
* frequency is the same between the two operating modes though this
* won't be critical.
*
* Then, with reference to the attached diagram:
*      Three samples are taken per cycle from flame A:
*              A: 100msec after the test signal falling edge
*              B: 100msec after the test signal rising edge
*              C: 150msec before the test signal rising edge
*
* Ideally ignore the samples from the first cycle then take in the
* samples from the next 'n' cycles, where 'n' should be at least two.
*
* Thence:
*      Aav = average(A1..An)
*      Bav = average(B1..Bn)
*      Cav = average(C1..Cn)
*
* Pass criteria:
*      Aav: >3v (232)
*      Bav: <0.5v (38.63)
*      Cav: between 0.5v and 1.8v (38 to 139)
*
* The same test is carried out for flame B and the guard channel.
* The thresholds are the same for all three channels.
*
* The 3v threshold can be hard-coded.
* The 0.5v and 1.8v thresholds can be put in the DCE.
*
* The above is carried out in low gain. We can also do a
* high-gain test. The test process is the same; the only
* difference is the pass criteria:
*
*      Aav: >3v
*      Bav: <0.5v
*      Cav: >3v
*
* I suggest then we do the test in low gain then in high gain.
*
* All told, this should trap at least half the component
* failures in the flame and guard amplifiers.
* LAMP_TEST_CYCLES=8.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 *************************************************************************/
BYTEu ElectricalAlarmTest(BYTEu typofreq)
{
  BYTEu lclcn,storegainval;
  /*
   * reset alarm fault
   * UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,false,0U);
   */
  /* Halt Timer 2 ISR ADC readings */
  g_RunAwayFlg = true;
  /* store then set high gain */
  storegainval = PORTG & GAIN_PING0;
  /* set high gain */
  GainControl(HIGH_G);
  /* settle */
  ConvModeDelayMs(AMP_SETTLE);
 
  /* actual test */
  //set test pin high
  SetPyroElectricalPin(true);
  ConvModeDelayMs(WAIT_STEP_1);//10
  SetPyroElectricalPin(false);
  ConvModeDelayMs(WAIT_STEP_2);//110
  //read gu
  __disable_interrupt();
  elegu_A[0U]=GetAdcRawValue250(CH2_GUARD);
  __enable_interrupt();
  ConvModeDelayMs(WAIT_STEP_3);//140
  //read fl
  __disable_interrupt();
  elefla_A[0U]=GetAdcRawValue250(CH0_FLAME);
  eleflb_A[0U]=GetAdcRawValue250(CH1_FLAME);
  __enable_interrupt();
  ConvModeDelayMs(WAIT_STEP_4);//270
  //read gu
  __disable_interrupt();
  elegu_A[1U]=GetAdcRawValue250(CH2_GUARD);
  __enable_interrupt();
  ConvModeDelayMs(WAIT_STEP_5);//390
  //read fl
  __disable_interrupt();
  elefla_A[1U]=GetAdcRawValue250(CH0_FLAME);
  eleflb_A[1U]=GetAdcRawValue250(CH1_FLAME);
  __enable_interrupt();
  ConvModeDelayMs(WAIT_STEP_6);//820
  //read both
  __disable_interrupt();
  elefla_A[2U]=GetAdcRawValue250(CH0_FLAME);
  eleflb_A[2U]=GetAdcRawValue250(CH1_FLAME);
  elegu_A[2U]=GetAdcRawValue250(CH2_GUARD);
  __enable_interrupt();
  //bring test pin down
  SetPyroElectricalPin(false);
 
  /* repeat for low gain */
  GainControl(LOW_G);
  /* settle */
  ConvModeDelayMs(AMP_SETTLE_LG);
  /* actual test */
   //set test pin high
  SetPyroElectricalPin(true);
  ConvModeDelayMs(WAIT_STEP_7);//10
  SetPyroElectricalPin(false);
  ConvModeDelayMs(WAIT_STEP_8);//80
  //read all
  __disable_interrupt();
  elefla_B[0U]=GetAdcRawValue250(CH0_FLAME);
  eleflb_B[0U]=GetAdcRawValue250(CH1_FLAME);
  elegu_B[0U] =GetAdcRawValue250(CH2_GUARD);
  __enable_interrupt();
  ConvModeDelayMs(WAIT_STEP_9);//230
  //read all
  __disable_interrupt();
  elefla_B[1U]=GetAdcRawValue250(CH0_FLAME);
  eleflb_B[1U]=GetAdcRawValue250(CH1_FLAME);
  elegu_B[1U] =GetAdcRawValue250(CH2_GUARD);
  __enable_interrupt();
   /* restore gain */
  GainControl(storegainval);
  /* print */
  if (typofreq == DEBUG_SMP_TRIGGER)
  {
    printf_P("\r HG: (%d,%d,%d), (%d,%d,%d), (%d,%d,%d), LG: (%d,%d), (%d,%d), (%d,%d)",
               elefla_A[0U],elefla_A[1U],elefla_A[2U],
               eleflb_A[0U],eleflb_A[1U],eleflb_A[2U],
               elegu_A[0U],elegu_A[1U],elegu_A[2U],
               //LG
               elefla_B[0U],elefla_B[1U],
               eleflb_B[0U],eleflb_B[1U],
               elegu_B[0U],elegu_B[1U]
               );  
    delayms(PRINT_WAIT);
  }
  
  /* decsion */
  lclcn = MakeEleAlarmTestDecesion(typofreq);
  ConvModeDelayMs(AMP_SETTLE_WAIT);
  /* restore timer 2 interrupt */
  g_RunAwayFlg = false;
  
  return(lclcn);
} /* void */

/********************************************************************//**
* @name MakeEleAlarmTestDecesion()
* @brief 
*  Electrical alarm test decesion function. 
* AvgPkPk=[(sum of the samples from lamp=on) - (sum of the samples from lamp=off)] / NS
*  @param typofsrc
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none 
***************************************************************************/
BYTEu MakeEleAlarmTestDecesion(BYTEu typofsrc)
{
  static BYTEu electricalfltcnt = ELE_CNT_RESET, elefaultx = ELEFLT_NOT_SET;
  BYTEu loopx = 0U;
  
  loopx = false;
  /* comp and make decsion */
  /* High gain */
  if ((elefla_A[0U] > HG_FL_LIM_1) && (elefla_A[1U] < HG_FL_LIM_2) && (elefla_A[2U] > HG_FL_LIM_3) &&
      (eleflb_A[0U] > HG_FL_LIM_1) && (eleflb_A[1U] < HG_FL_LIM_2) && (eleflb_A[2U] > HG_FL_LIM_3) &&
      (elegu_A[0U] > HG_GU_LIM_1) && (elegu_A[1U] < HG_GU_LIM_2) && (elegu_A[2U] > HG_GU_LIM_3) &&
      /* Low gain */
      (elefla_B[0U] > LG_FL_LIM_X) && (elefla_B[1U] < LG_FL_LIM_X) &&
      (eleflb_B[0U] > LG_FL_LIM_X) && (eleflb_B[1U] < LG_FL_LIM_X) &&
      (elegu_B[0U] > LG_GU_LIM_1) && (elegu_B[1U] < LG_GU_LIM_2) 
      )
  {
    loopx = true;
    /* clear periodic counter and restore configured time interval */
    if(electricalfltcnt != ELE_CNT_RESET)
    {
      RestoreOrginalOpmInterval();
      electricalfltcnt = ELE_CNT_RESET;
    }
    /* log event pass */
    FdeAlarmTestResult((INT16u)loopx,0U,0U,0U,0U,GetDetectorTemperature() / CONVERT_TO_DEG,(PORTG & GAIN_PING0),
                       GetAccumulatedValue(REQ_EAUTO),GetAccumulatedValue(REQ_ECROSS),
                       g_mux1[MOISTURE_SENS]);
    UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,false,0U);
    elefaultx = ELEFLT_NOT_SET;
    if (typofsrc == MZX_PANEL_TRIGGER)
    {
      return(loopx);
    }
    if (typofsrc == DEBUG_SMP_TRIGGER)
    {
      printf_P("\r!!ELE TEST SUCCESS!!\r");
      delayms(WAIT_1_SEC);
      return(loopx);
    }
    /* all conditions met set alarm if its not periodic test */
    if ((typofsrc != TIMED_SELF_TEST) && (typofsrc != NEW_MAN_SELF_TEST))
    {
      SignalAlarmForAlarmTest();
    }
    /* give some settling and adc update time for alm rel monitor. */
    if ((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == DEFVAR_4_20MA_MODE))
    {
      delayms(MS_200);
    }

  }
  else
  {
    /* log event fail */
    FdeAlarmTestResult((INT16u)loopx,0U,0U,0U,0U,GetDetectorTemperature() / CONVERT_TO_DEG,(PORTG & GAIN_PING0),
                       GetAccumulatedValue(REQ_EAUTO),GetAccumulatedValue(REQ_ECROSS),
                       g_mux1[MOISTURE_SENS]);
    /* set fault */
    if (typofsrc != TIMED_SELF_TEST)
    {
      UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,true,3U);
    }
    else
    {   
      /* periodic test failure */
      if((electricalfltcnt == ELE_CNT_RESET) && (elefaultx == ELEFLT_NOT_SET))
      {
        /* change self test time as well */
        SetTimeUntilNextOpm(OPM_TIME_BETWEEN_TEST_IF_OBSCURED);
      }
      
      electricalfltcnt++;
      if(electricalfltcnt >= ELE_CNT_LIMIT)
      {
        electricalfltcnt = ELE_CNT_RESET;  
        UpdateHardwareFaultSentinel(CNT_ALARMTESTFAIL,true,2U);
        elefaultx = ELEFLT_SET;
        RestoreOrginalOpmInterval();
      }
    }
  }
  
  return(loopx);
}
/********************************************************************//**
* @name InitHWTests();
* @brief
* NOTE: further addtion ADD no camera as well for future board.
* Tests to be carried out at startup:
* a. Moisture detector
* b. PAL/NTSC check
* c. software CRC check
* Disable interrupts as ADC are read in this rutine at start up.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/
void InitHWTests(void)
{
  BYTEu flagnumb = 0U;
  BYTEu sysfltvalA = 0U;

  __disable_interrupt();
  OCR2A = 143U;
  __enable_interrupt();
  
  /* set all the health monitors flags to defaults */
  for (flagnumb = 0U; flagnumb <= LAST_FLTFLAG; flagnumb++)
  {
    g_faultflag[flagnumb] = false;
  }
  
  PORTC &= ~(0x01U << PC1);
  DDRC &= ~(0x01U << DDC1);
  SetAdcMuxChannel(MOISTURE_DETECT);
  //delayms(ADC_SETTLE_WAIT);
  KickWdt();
  __sleep();
  g_mux1[MOISTURE_SENS] = ReadAdcSettledCh(ADC_CH7_MULX_0_1);
  MoistureLevelTest(REQ_LOOP_TEST);

  /*
   * PAL/NTSC TEST
   * read sys flt line
   */
  ////////////////////
  
  SetAdcMuxChannel(SYSFLT);
  //set open collector
  DDRC &= ~(0x01 << PC5);
  PORTC &= ~(0x01U << PC5);  /* set low */
  KickWdt(); 
   __sleep();
   __sleep();
   __sleep();
  sysfltvalA = ReadAdcSettledCh(ADC_CH6_MULX_2_3);

  /* now read BULID ID */
  DDRC  |= (0x01 << PC5);
  PORTC |= (0x01U << PC0);  /* set high */
  SetAdcMuxChannel(BUILD_ID);
   __sleep();
  Buildid = ReadAdcSettledCh(ADC_CH7_MULX_0_1);
  PORTC &= ~(0x01U << PC0);  /* set low */
 
  if (true == true)
  {
   //printf_P("SYS:A=%d\r",sysfltvalA);   /* comment out !!! */
  }
  if (sysfltvalA < SYS_FLT_LIMX)
  {
    UpdateHardwareFaultSentinel(CNT_SYSTEMFAULT,true,0U);
  }
  
 KickWdt();
 ////////////////////////////
  /*
   * SENSOR BOARD match test
   * by default the CRC field is set to 0 in AVR eeprom
   */  
  /*
  Build ID test
  for IS value should be around then 209U.
  */
  if((Buildid > Build_Id_MAX) || (Buildid < Build_Id_MIN))
   {
     UpdateHardwareFaultSentinel(CNT_BUILD_ID,true,Buildid);
   }
  
 __sleep(); 
 KickWdt();
} /* void */

/********************************************************************//**
*  @name SoakTestConditionCheck()
*  @brief
*  Assumptions
*  All dip switches off, Interface mode default 4_20 from DCE
*  If above condition not met then test would be ignored
*  Checks for soak condition if present then puts detector into soak test mode.
*  @param reqtyp
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
BYTEu SoakTestConditionCheck(BYTEu reqtyp)
{
  static BYTEu outcome = false,relaysetflg = false;

  KickWdt();
  /* read master switch */
  if ((!(GetDipSwitchVal() & MASTER_SW)) && (reqtyp == ACTUAL_POWERUP))
  {
    /* check DCE for default mode */
    if (TagToConfigReadChar(INTERFACES) == DCE_DEFAULT_4_20MA_MODE)
    {
      /*
       * now check for soak test condition
       * pull up the monitor line
       */
      PORTG |= (0x01U << PG2);
      SetAdcMuxChannel(WTEST);
      /* send pulses on alarm led then read wtest i/p */
      AlarmLED(true);
      delayms(LED_FLASH_WAIT);
      if (ReadAdcSettledCh(ADC_CH6_MULX_2_3) < SOAK_TEST_LW_LIM)
      {
        /* NPN transistor is active now invert */
        AlarmLED(false);
        delayms(LED_FLASH_WAIT);
        if (ReadAdcSettledCh(ADC_CH6_MULX_2_3) > SOAK_TEST_UP_LIM)
        {
          /* transistor active confirm soak test */
          outcome = true;
          ChangePersonalityType(MX_IFACE_MODE);
          FaultRelay(false);
          MxAiscSwitch(true);
          printf_P("!!Soak Test!!\r");
        }
      }
      AlarmLED(false);
      /* set PG2 to Gnd */
      PORTG &= ~(0x01U << PG2);
    }
  }
  /* safe to turn on fault relays */
  if ((outcome == false) && (relaysetflg == false))
  {
    relaysetflg = true;
    if ((PersonalityType() == DEFVAR_4_20MA_MODE) || (PersonalityType() == DEFAULT_4_20MA_MODE))
    {
      FaultRelay(true);
    }
  }

  return(outcome);
}

/********************************************************************//**
*  @name StoreOneWireCrc()
*  @brief
*  Stores sensor board CRC on AVR eeprom.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
void StoreOneWireCrc(void)
{
  BYTEu crctmp;
  if (g_enablechecksum == true)
  {
    crctmp = ReadROMOfSCE(REQ_LOOP_TEST);
    TagToDceWriteChar(ONEWIRECRC,crctmp);
  }
}
/********************************************************************//**
*  @name GetVideoType()
*  @brief   
*  returns PAL,NTSC or NO Camera
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
BYTEu GetVideoType(void)
{
  return(g_VideoType);
}

/********************************************************************//**
*  @name RunAwayFromInterrupt()
*  @brief   
*  returns g_RunAwayFlg for Electrical test
*  @param none
*  @return g_RunAwayFlg
*  globalvariablesused g_RunAwayFlg
*  globalvariablesmodified none
************************************************************************/
BYTEu RunAwayFromInterrupt(void)
{
  return(g_RunAwayFlg);
}

/********************************************************************//**
*  @name SignalAlarmForAlarmTest()
*  @brief
*  Alarm handler for Manual Alarm tests,
*  Signals alarm depending on personality type.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
void SignalAlarmForAlarmTest(void)
{
  BYTEu delim,delx;
  /* set internal state to alarm */
  SetInternalAlarmState(STATUS_ALARM,REQ_MANUAL);
  /* if in service mode use 3 sec hangover */
  if (!GetServiceModeFlag())
  {
    delim = TagToConfigReadChar(ALARM_HANGOVER);
  }
  else
  {
    delim = HANGOVER_OVERRIDE_LIM;
  }
  /* video */
  if (PersonalityType() != CONVENT_MODE)
  {
    //printfv(8U,9U,true,"! ALARM !");
  }

  if ((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == DEFVAR_4_20MA_MODE)
      || (PersonalityType() == ONLY_4_20_DIS) || (PersonalityType() == ONLY_4_20_VAR))
  {
    /* either default or 4_20 modes */
    AlarmLED(true);
    if ((PersonalityType() == DEFAULT_4_20MA_MODE) || (PersonalityType() == DEFVAR_4_20MA_MODE))
    {
      AlarmRelay(true);
    }

    Enable_4_20mA();
    Output_4_20mA_Level(SET_17mA);

    for (delx = 0U; delx < delim; delx++)
    {
      ModbusDelayms(HOLD_FOR_1SEC);      /* 1 sec delay */
    }
    AlarmLED(false);
    AlarmRelay(false);
    Output_4_20mA_Level(SET_NORMAL);

  }
  else if (PersonalityType() == CONVENT_MODE)
  {
    AlarmLED(true);
    ConvAlarmControl(true);
    for (delx = 0U; delx < delim; delx++)
    {
      delayms(HOLD_FOR_1SEC);      /* 1 sec delay */
    }
    AlarmLED(false);
    ConvAlarmControl(false);

  }
  else if ((PersonalityType() == MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE))
  {
    Timer3MXPWMEnable();
    OutputMX_4_20mALevel(MX_17mA);    /* high state = alarm on. */
    for (delx = 0U; delx < delim; delx++)
    {
      ModbusDelayms(HOLD_FOR_1SEC);       /* 1 sec delay */
    }
    OutputMX_4_20mALevel(MX_4mA);
  }
  else
  {}
  /* set internal state to normal */
  SetInternalAlarmState(STATUS_NORMAL,REQ_MANUAL);
  /* clear video */
  if (PersonalityType() != CONVENT_MODE)
  {
    //printfv(8U,9U,false,"         ");
  }

}

/*******************************************************************//*
* @name CalibrationCountCheck()
* @brief 
* Calibration Check if calibration count is zero
* raise a hardware fault.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**********************************************************************/ 
void CalibrationCountCheck(void)
{
 static BYTEu calibcntx = 0U; 
 
 if(TagToConfigReadChar(ONEWIRECRC) != false)
 {
   if (calibcntx == 0U)
   {
    calibcntx = TagToConfigReadChar(CALIBCOUNT);
   }
   
    if(g_enablechecksum == true)
    {
      if(calibcntx == 0U)
      {
        UpdateHardwareFaultSentinel(CNT_CALIB_NOT_DONE,true,0U);
      }
    }
 }
}

/*******************************************************************//*
* @name InterruptMonitor()
* @brief 
* Monitors weather interrupt is active or not
* If not then stop kicking watchdog
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
**********************************************************************/ 
void InterruptMonitor(void)
{
 static INT16u intmoncounter = 0U; 
  
 if (g_interruptmonitor == true)
 {
   g_interruptmonitor = false;
   intmoncounter = 0U;
 }
 else
 {
   intmoncounter++;
 }

 if (intmoncounter > INT_MON_COUNTER_MAX)
 {
   intmoncounter = 0U;
   if(GetServiceModeFlag() == false)
   {
    /* future modification force reset by external watchdog as main 10ms interrupt has stopped */
     while(true)
     {}
   }
 }
}

/********************************************************************//**
*  @name ConvModeDelayMs()
*  @brief
*  delay with sleep mode, timer 2 running.
*
*  Note: Turning on conv alarm load right before this rutine is trigrring
*  pulse detect mechanism, whenever a pulse is detected it disables the
*  sleep mode for a duration (64ms) and if main loop is not active then
*  sleep will not be enabled again.
*
*  If main loop is running normally then after a single event of -ve
*  pulse it will reset detection mechanism after 64ms and restore sleep
*  mode settings.
*
*  AR Jan 2013
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
void ConvModeDelayMs(INT16u delx)
{
  INT16u lcldelcnt = 0U;

  if ((PersonalityType() == CONVENT_MODE) || (PersonalityType() == MX_IFACE_MODE) ||
      (PersonalityType() == MX_IFACE_DIPSW_ACTIVE))
  {
    while (lcldelcnt < delx / CONVERT_TO_MS)
    {
      lcldelcnt++;
      ControlledKickWdt(false);
      SleepModeActive(MAIN_LOOP);
      ControlledKickWdt(true);
    }
  }
  else
  {
    ModbusDelayms(delx);
  }
}

/********************************************************************//**
*  @name GetBuildId()
*  @brief
*  Returns bulid id of board.
*  @param none
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
************************************************************************/
BYTEu GetBuildId(void)
{
  return(Buildid);
}

/********************************************************************//**
*  InitTestByte()
************************************************************************/
void InitTestByte(void)
{ 
  dbgtestbyte = TagToConfigReadChar(TESTBYTEENABLE);
  firstentrytesttag =true;
}

/********************************************************************//**
*  ReturnTestByte()
************************************************************************/
unsigned char ReturnTestByte(void)
{
 return(dbgtestbyte);
}

/********************************************************************//**
* @name WindowHeaterTest();
* @brief
* NOTE: After meeting following procedure is not valid. Carry
* out a simple on of monitoring only.
* Algorithm:(implemented in ISR)
* Set the window heater control either HIGH or LOW as required
* Wait 10msec
* Set video chip-select LOW (if not already)
* Enable opto-isolator PG5
* Wait for settle 2ms
* Read window heater status (MUX1, Y5)
* Disable opto-isolator
* Return video chip-select to original state
* Enable SPI
*  @param reqst
*  @return none
*  globalvariablesused none
*  globalvariablesmodified none
 ***************************************************************************/

/*BYTEu WindowHeaterTest(BYTEu reqst)
{
  BYTEu outcome = true;
  
  if (reqst == REQ_LOOP_TEST)
  {
    return(true);
  }
  if (!g_faultflag[WINHEAT_FLTFLAG])
  {
    if (g_WinHeatAdcValue <= HEATER_ON_LIM && !GetHeatedOpticsPoweredState())
    {
       //heater is read as ON while the control pin if at OFF 
      g_faultflag[WINHEAT_FLTFLAG] = true;
      if (reqst == REQ_LOOP_TEST)
      {
        UpdateHardwareFaultSentinel(CNT_WINDOWHEATERFAULT,true,1U);
      }
      else
      {
        outcome = false;
      }
    }
    else if (g_WinHeatAdcValue >= HEATER_OFF_LIM && GetHeatedOpticsPoweredState())
    {
      //heater is read as off when it should be ON 
      //g_faultflag[WINHEAT_FLTFLAG] = true;
      if (reqst == REQ_LOOP_TEST)
      {
        UpdateHardwareFaultSentinel(CNT_WINDOWHEATERFAULT,true,2U);
      }
      else
      {
        outcome = false;
      }
    }
    else
    {}
  }
  else
  {
    //remove fault if set and all OK 
    if (g_WinHeatAdcValue < HEATER_ON_LIM && GetHeatedOpticsPoweredState())
    {
      g_faultflag[WINHEAT_FLTFLAG] = false;
      UpdateHardwareFaultSentinel(CNT_WINDOWHEATERFAULT,false,1U);
    }
    else if (g_WinHeatAdcValue > HEATER_OFF_LIM && !GetHeatedOpticsPoweredState())
    {
      g_faultflag[WINHEAT_FLTFLAG] = false;
      UpdateHardwareFaultSentinel(CNT_WINDOWHEATERFAULT,false,2U);
    }
    else
    {}
  }

  return(outcome);
}*/

/* // ######### EOF ######### //// */
