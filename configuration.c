/***********************************************************************//**
* @file  configuration.c
* @brief
*  This file contains the functions associated with powerup status and
*  configuration parameters. This file also contains fuctions which writes/reads
*  data to/from the DCE.
*  Used On:     FV400
***************************************************************************/
#define  ENABLE_BIT_DEFINITIONS
#include <iom1281.h>
#include <inavr.h>
#include <pgmspace.h>
#include <string.h>
#include <stdlib.h>

#include "defs.h"
#include "configuration.h"
#include "library.h"
#include "peripheral.h"
#include "hardwarefaultsentinel.h"
#include "fde.h"
#include "panelcomms.h"
#include "serviceprotocol.h"
#include "modbus.h"
#include "uart.h"
#include "termdebug.h"
#include "video.h"
#include "adc.h"
#include "interruptcaptureandhwmonitor.h"
#include "scememory.h"
#include "AFDPL15_IAR.h"
#include "s231interface.h"
#include "servicecommands.h"
#include "heatedoptics.h"
#include "debug.h"
#include "hardwaretest.h"
#include "spi.h"
#include "opmfaultsentinel.h"
#include "ir.h"
#include "watchdog.h"

/*
 * #######################################################################
 * global data
 * #######################################################################
 * Use the checksum table held in boot sector
 */
extern __farflash const unsigned short g_sprog_crc16_table[256];
/**
 * Do not use 0x0000 as the start adderss because pointer will rest
 * at this location in order to avoid the data corruption.
 * the EEPROM start address is - address 0x0001. AR May 2011
 */
#pragma location=DCE_STARTADDRESS /* 0x0001 */
__eeprom __no_init unsigned char Dce[DCE_SIZE]; /* 0x01ff; */

/*
 * when the EEPROM is not being used, the address pointer
 * should be set to zero. This
 */
#pragma location=0x0000
__eeprom __no_init unsigned char addresszero;
/*
 * Flag set when ATMega128 CODE checksuming is enabled.
 * This flag is set to false only during development to allow usage of JTAG
 */
#define END_TABLE_MARKER 0xffU

extern unsigned char g_enablechecksum;
extern volatile unsigned char dispinfoarry[4],g_HWFaultState;
volatile unsigned char spikemx = 3U,historymx = 5U,rangemx = RANGE_50M;
static unsigned int dipswitchvalue;
static unsigned char diphwfault = DIP_ALL_OK;
static unsigned char interfacemode = DEFAULT_4_20MA_MODE,mbset = NONE_SET,HeaderPrintState = false;
static unsigned char enhancedbandbitval = 0U, hartprotocolreq = false,auxsupplysw = false;
extern __root const __farflash char ControllerDate[12];
extern __root const __farflash char ControllerTime[8];

extern BYTEu firstentrytesttag;

BOOL ProcessADCTasks(BOOL init_stat);

void ReadADCsFromTaskTable(ADC_DATA_STRUCT table_pt[]);
/**
 * NAME
 * dce_map[]
 *
 * Address map of the data stored in the detector configuration EEPROM (DCE)
 * A configuration setting is referred to by its 'tag'. A tag is
 * looked up in this table to find the address of the configuration setting
 * in the EEPROM
 *
 * INFROMATION
 * When altering this struct, ensure your changes will still work
 * with products in the field that run an earlier version of the
 * firmware.
 * Take care whem moving the address of used fields to a new location as
 * that location could point to rubbish!
 * If there is no choice but to move address, use the DCEREVISION tag
 * to set the DCE to factory defaults values.
 * Production test and identity data is retained, event after DCE
 * re-initalisation. The only way to modify it is using service mode
 * commands to read and write data
 */
const __farflash DCE_MEMORYMAP dce_map[] = {

/*
 * ##### Address 0-142 DATA IS PROTECTED FROM DCE REINITALISATION ###########
 * My serial number stamped on the guard plate                        //
 */
  {MYSERIALNUM,     "SerialNumber",        0U,SIZEOFMYSERIAL,0U,0U,0U},      /*  */
  /* Topcase Manufacture data (Front assembly)                          // */
  {TOPCSERIALNUM,   "TopCSerial",          8U,SIZEOFUNITSERIAL,0U,1U,1U},    /*  */
  /* Controller PCB Identification Data                                 // */
  {CONTSERIALNUM,   "ContSerial",         27U,SIZEOFUNITSERIAL,0U,1U,1U},    /*  */
  /* Topcase environmental test test data                               // */
  {ENVTESTDATE,     "EnvTestDate",        46U,SIZEOFDATE,0U,0U,0U},          /*  */
  {ENVTESTRESULT,   "EnvTestResult",      52U,SIZEOFCHAR,0U,1U,0U},          /*  */
  {ENVTESTITERAT,   "EnvTestIterat",      53U,SIZEOFCHAR,0U,0U,0U},          /*  */
  {ENVTESTRESERVED, "EnvTestReserved",    54U,TEST_RESERVED,0U,0U,0U},       /*  */
  /* Topcase functional test test data                                  // */
  {FNCTESTDATE,     "FncTestDate",        62U,SIZEOFDATE,0U,0U,0U},          /*  */
  {FNCTESTRESULT,   "FncTestResult",      68U,SIZEOFCHAR,0U,1U,0U},          /*  */
  {FNCTESTITERAT,   "FncTestIterat",      69U,SIZEOFCHAR,0U,0U,0U},          /*  */
  {FNCTESTRESERVED, "FncTestReserved",    70U,TEST_RESERVED,0U,0U,0U},       /*  */
  /* Controller PCB test data                                           // */
  {CONTTESTDATE,    "ContTestDate",       78U,SIZEOFDATE,0U,0U,0U},          /*  */
  {CONTTESTRESULT,  "ContTestResult",     84U,SIZEOFCHAR,0U,1U,0U},          /*  */
  {CONTTESTITERAT,  "ContTestIterat",     85U,SIZEOFCHAR,0U,0U,0U},          /*  */
  {CONTTESTRESERVED,"ContTestReseserved", 86U,TEST_RESERVED,0U,0U,0U},       /*  */
  /* potting serials */
  {POTTINGTESTDATE,    "PottinTestDate",  94U,SIZEOFDATE,0U,0U,0U},          /*  */
  {POTTINGTESTRESULT,  "PottingTestResult",100U,SIZEOFCHAR,0U,1U,0U},          /*  */
  {POTTINGTESTITERAT,  "PottingTestIterat",101U,SIZEOFCHAR,0U,0U,0U},          /*  */
  {POTTINGTESTRESERVED,"PottingTestResrvd",102U,TEST_RESERVED,0U,0U,0U},
  /* Controller board Reference voltage calibration                     // */
  {ADCCALIBRATION,  "AdcCalibration",   110U,SIZEOFINT, 0U,0U,0U},          /*  */
  /* Conteroller service address. Changing it would lose communications // */
  {SERVICEADDRESS,  "ServiceAddress",   112U,SIZEOFINT, 0U,255U,1U},          /*  */
  /* Camera fitted during factory production                            // */
  {CAMERAFITTED,    "CameraFitted",     114U,SIZEOFCHAR,0U,0U,true},      /*  */
  /* sensor board ID */
  {ONEWIRECRC,   "1 Wire CRC for Calib",115U,SIZEOFCHAR, 0U,255U,0U},
  /* MX ASIC serial number */
  {MXSERIALNUM,   "MXASIC Serial",      116U,SIZEOFMYSERIAL,  0U,  0U,0U},
  /* pyro calib */
  {DFLT_PRIMLGA,    "Default prim lga", 124U,SIZEOFCHAR,   0U,  255U, 133U},
  {DFLT_PRIMLGB,    "Default prim lgb", 125U,SIZEOFCHAR,   0U,  255U, 134U},
  {DFLT_SECLG,     "Default sec lg",    126U,SIZEOFCHAR,   0U,  255U, 140U},
  {DFLT_PRIMHGA,   "Default prim hga",  127U,SIZEOFCHAR,   0U,  255U, 8U},
  {DFLT_PRIMHGB,   "Default prim hgb",  128U,SIZEOFCHAR,   0U,  255U, 8U},
  {DFLT_SECHG,     "Default sec hg",    129U,SIZEOFCHAR,   0U,  255U, 8U},
  {CALIBCOUNT,     "RecCalibCnt",       130U,SIZEOFCHAR,   0U,  255U, 0U},
  /* lamp tests */
  {OPMLAMPCAL,       "OpmLampCalib",    131U,SIZEOFINT,   0U,  0U,WIN_CAL_DEFAULT},
  {OPMLEDCAL,        "OpmLedCalib",     133U,SIZEOFINT,   0U,  0U,450U},
  {TESTLAMPCALGU,   "Test GU cal val",  135U,SIZEOFINT,   0U,  0U,WIN_CAL_DEFAULT}, /* alm test GU */
  {TESTLAMPCALFL,   "Test Fl cal val",  137U,SIZEOFINT,   0U,  0U,WIN_CAL_DEFAULT}, /* fl alm test */
  /* flags */
  {FLMACALFLG,      "Fl A Calib State", 139U,SIZEOFCHAR, 0U,3U,0U},
  {FLMBCALFLG,      "Fl B Calib State", 140U,SIZEOFCHAR, 0U,3U,0U},
  {GDCALFLG,        "Gu Calib State",   141U,SIZEOFCHAR, 0U,3U,0U},
  {SUNSCALFLG,      "Sun Calib State",  142U,SIZEOFCHAR, 0U,3U,0U},

  {OPMDRIVE,        "DL for Lamp",      143U,SIZEOFCHAR, 0U, 4U, 4U},
  {OPMLEDDRIVE,     "Drive Level IR",   144U,SIZEOFCHAR, 0U, 7U, 2U},
  {ELELIMMAX,      "Max Ele Test Lim",  145U,SIZEOFCHAR, 0U,255U,232U},
  {ELELIMMIN,      "Min Ele Test Lim",  146U,SIZEOFCHAR, 0U,255U,38U},
  /*
   * #########################################################################//123
   * Operation settings - Note, Dipswitch can overide these settings
   */
  {OPMLEDTIMER,     "OPMLEDTimer",        147U,SIZEOFINT,  0U, 0U, 1200U}, /* 1200sec=20min */
  /* {Spare,  "xxxxx",   149,SIZEOFCHAR, 0,255, 0}, */

  {LOGPRINTLEVEL,   "LogPrintLevel",     150U,SIZEOFCHAR,  0U,TERM_ENDMARKER - 1U,TERM_OFF},
  /* interface stuff is here!!! */
  {INTERFACES,    "Interface Selection",     151U,SIZEOFCHAR,   0U,  5U,DCE_DEFAULT_4_20MA_MODE},
  {HARTSELECT,    "Hart Interface on/off",   152U,SIZEOFCHAR,   0U,  1U, 0U},
  {_4_20MAMODE1,  "4_20mA Band/Analogue",    153U,SIZEOFCHAR,   0U,  1U, 0U},
  {_4_20MAMODE2,  "4_20 Enhanced",           154U,SIZEOFCHAR,   0U,  1U, 0U},
  {MXMODE1,       "MX Mode options",         155U,SIZEOFCHAR,   0U,  1U, 0U},
  {AUXSUPPLY,     "Aux Supply Switch",       156U,SIZEOFCHAR,   0U,  1U, 0U},
  {DEVELOPMENT,   "Development Mode Select", 157U,SIZEOFCHAR,   0U,  12U, 0U},
  /* NOSEx setup (Total of 10 bytes) */
  {LOOPADDRESS,     "LoopAddress",       160U,SIZEOFCHAR,  0U,255U,1U}, /* entry nu41 */
  /* FIELDBUS select (Total of 1 byte) */
  {FIELDBUSSELECT,  "FieldBusSelect",    170U,SIZEOFCHAR,  0U,2U,0U},    /* 1=Modbus, 2=Profibus */
  /* MODBUS settings (Total of 10 bytes) */
  {MODBUSSLAVEADDR, "ModbusSlaveAddr",   171U,SIZEOFCHAR,  1U,247U,2U},  /* As define in protocol doc */
  {MODBUSADDROFFSET,"ModbusAddrOffset",  172U,SIZEOFINT,   0U,  0U,0U},  /* All values valid */
  {MODBUSBAUDRATE,  "ModbusBaudRate",    174U,SIZEOFCHAR,BAUD19200,BAUD9600,BAUD19200},
  {MODBUSPARITY,    "ModbusParity",      175U,SIZEOFCHAR,NOPARITY,ODDPARITY,EVENPARITY},
  /* Video and Camera Configuration */
  {VIDEODISPLAYSET,"VideoDisplaySetting",190U,SIZEOFCHAR,   DISP_OFF,DISP_ENDMARKER - 1U,1U},
  {VIDEOSERIAL,    "VideoSerial",        191U,SIZEOFVIDSERL,0U,  0U,0U},
  {VIDEODESCRIPTION,"VideoDescription",  203U,SIZEOFVIDDESC,0U,  0U,0U},
  {VIDEOWTALERT,   "VideoWalkTestAlert", 227U,SIZEOFCHAR,   0U,  255U,0U},  /* All values valid */

  {TESTBYTEENABLE,  "Enable Test Byte", 228U,SIZEOFCHAR,   0U,  15U,0U},
  {DETECT_RANGE,    "Detector range",   229U,SIZEOFCHAR,   0U,  RANGE_6M, RANGE_50M},
  {LOGENABLE,       "LogEnable",        230U,SIZEOFCHAR,  0U,  1U,true},
  /* latching */
  {ALARMLATCHING,   "AlarmLatching",    231U,SIZEOFCHAR,  0U,  1U,true},
  {FAULTLATCHING,   "FaultLatching",    232U,SIZEOFCHAR,  0U,  1U,true}, /* was false till Oct 2014 */
  /* OPM settings */
  {OPMINHIBIT,      "OpmInhibit",       233U,SIZEOFCHAR,  0U,  1U,false}, /* false */
  {OPMTIMER,        "OpmTimer",         234U,SIZEOFINT,   0U,  0U,OITIMER},
  {ALARM_HANGOVER,  "Alarm Hold Del",   236U,SIZEOFCHAR,  1U,  60U, 10U},
  {WINDOWHEATER,    "WindowHeater",     237U,SIZEOFCHAR,  0U,WINDOW_ENDMARKER - 1U,WINDOW_OFF},
  {WHEATERONTEMP,   "Heater On Limit",  238U,SIZEOFCHAR,  0U,  255U, 65U}, /* 30=-10C */
  {WHEATEROFFTEMP,  "Heater Off Limit", 239U,SIZEOFCHAR,  0U,  255U, 80U}, /* 120=80C */
  /* lamps calibration data */
  {LOGFAULT,        "LogFault",         240U,SIZEOFCHAR,  0U,  1U,true},
  {LOGOPMDATA,      "LogOpmData",       241U,SIZEOFCHAR,  0U,  1U,true},
  {LOGALARM,        "LogAlarm",         242U,SIZEOFCHAR,  0U,  1U,true},
  {LOGALARMDATA,    "LogAlarmData",     243U,SIZEOFCHAR,  0U,  1U,true},
  /* new calib states */
  {LOGRESERVED1,    "LogReserved1",     244U,SIZEOFCHAR,  0U,  1U,false},
  {MOISTURESW,    "SwitchMoisture ",    245U,SIZEOFCHAR,  0U,  1U,true},
  {MOISTURETHRESH, "MoistThresh",       246U,SIZEOFCHAR,  0U,  255U,188U},
  {LAMPTESTENABLE, "Enable Win Lamp",   247U,SIZEOFCHAR,  0U,  true,false}, /* LAMP TEST */
  {TESTDRIVE,       "Test Drive  ",     248U,SIZEOFCHAR,  0U, 255U,0U},
  {LOGTEMP24HOUR,   "LogTemp24Hour",    249U,SIZEOFCHAR,  0U,  1U,true},
  {ALARMONSATUR,   "AlarmOnSaturation", 250U,SIZEOFCHAR,  0U,  1U,false},

  {OPMDIRTY,        "Lmp Dirty Limit",  251U,SIZEOFCHAR,  0U,  255U,LAMPONDIRTY},
  {OPMBLOCKED,      "Lmp Blocked Limit",252U,SIZEOFCHAR,  0U,  255U,LAMPONBLOCKED},
  {OPMLEDDIRTY,     "LED Dirty Limit",  253U,SIZEOFCHAR,  0U,  255U,WINDIRTYLIM},
  {OPMLEDBLOCKED,   "LED Blocked Limit",254U,SIZEOFCHAR,  0U,  255U,FULLYBLOCKEDWIN},
  /* alarm test */
  {ALARMTESTLEVEL,"Alm Test Diff FL-GU",255U,SIZEOFINT,   0U,  0U,70U},
  {ALARMTESTFLGUMAX,"Alm Test Max Lim", 257U,SIZEOFINT,   0U,  0U,375U},
  {ALARMTESTFLGUMIN,"Alm Test Min Lim", 259U,SIZEOFINT,   0U,  0U,200U},

  {ALARMDELAY,      "Delay/Samples",    261U,SIZEOFCHAR,  3U, 12U, 3U},
  {OPMLEDINHIBIT,   "OPMLEDEnable",     262U,SIZEOFCHAR,  0U, 255U, true},
  /*
   *  System - (Total of 2 bytes) Position fixed from
   * the end of the reserved memory space. Do not edit these two lines directly
   */
  {DCEREVISION,     "DecRevision",       (DCE_SIZE - SIZEOFINT - SIZEOFCHAR),
   SIZEOFCHAR,0U,255U,CURRENT_FORMAT_REVISION},
  {DCECHECKSUM,     "DecChecksum",       (DCE_SIZE - SIZEOFINT),
   SIZEOFINT,0U,0U,0U},
  /* End marker - NOTE: ENDMARKER must be last entry in this table */
  {ENDMARKER,'\0',0U,0U,0U,0U,0U}
};

/********************************************************************//**
 * NAME
 * ConvertServiceDefToTag()
 * DESCRIPTION
 * Converts the fixed #define label use in the service command to
 * a tag value
 *
 * @param CNT_xxx value received in the service command
 *
 * RETURN VALUE
 * tag value as define in configuration.h enum
 * Flag to indicate if the SRV_xxx value exists
 * LOCAL DATA
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 * For the stuff associated with calibration use calibration
 * functions within the SMP.
 *************************************************************************/
unsigned char ConvertServiceDefToTag(unsigned char *tag,
                                     unsigned char servicedefine)
{
  unsigned char retval = PASS;

  switch (servicedefine)
  {
/* ########### DETECTION SETTINGS ################ */
  case CNF_ALARMONSATUR:
    *tag = ALARMONSATUR;
    break;
/*      case CNF_RESERVED1: */
/*         *tag=CONRESERVED1; */
/*         break; */
/*      case CNF_RESERVED2: */
/*         *tag=CONRESERVED2; */
/*         break; */
  case CNF_ALARMDELAY:
    *tag = ALARMDELAY;
    break;
  case CNF_ALARMLATCHING:
    *tag = ALARMLATCHING;
    break;
  case CNF_FAULTLATCHING:
    *tag = FAULTLATCHING;
    break;
  case CNF_ALARM_HANGOVER:
    *tag = ALARM_HANGOVER;
    break;
  case CNF_DETECT_RANGE:
    *tag = DETECT_RANGE;
    break;
/* ########## Interfaces ############ */
  case CNF_INTERFACE:
    *tag = INTERFACES;
    break;
  case CNF_HARTSELECT:
    *tag = HARTSELECT;
    break;
  case CNF_4_20MAMODE1:
    *tag = _4_20MAMODE1;
    break;
  case CNF_4_20MAMODE2:
    *tag = _4_20MAMODE2;
    break;
  case CNF_MXMODE1:
    *tag = MXMODE1;
    break;
  case CNF_AUXSUPPLY:
    *tag = AUXSUPPLY;
    break;
  case CNF_DEVELOPMENT:
    *tag = DEVELOPMENT;
    break;
/* ########### OPM SETTINGS ################ */
  case CNF_OPMSPARE:
    /* *tag=OPMDRIVE; */
    break;
  case CNF_OPMINHIBIT:
    *tag = OPMINHIBIT;
    break;
  case CNF_OPMTIMER:
    *tag = OPMTIMER;
    break;
  case CNF_OPMDIRTY:
    *tag = OPMDIRTY;
    break;
  case CNF_OPMBLOCKED:
    *tag = OPMBLOCKED;
    break;
  case CNF_OPMLEDDIRTY:
    *tag = OPMLEDDIRTY;
    break;
  case CNF_OPMLEDBLOCKED:
    *tag = OPMLEDBLOCKED;
    break;
  case CNF_OPMLEDINHIBIT:
    *tag = OPMLEDINHIBIT;
    break;
  case CNF_OPMLEDTIMER:
    *tag = OPMLEDTIMER;
    break;
  case CNF_PYRODIFFTHRESH:
    *tag = ALARMTESTLEVEL;
    break;
  case CNF_ALARMTESTFLGUMAX:
    *tag = ALARMTESTFLGUMAX;
    break;
  case CNF_ALARMTESTFLGUMIN:
    *tag = ALARMTESTFLGUMIN;
    break;
  case CNF_ELEALARMTHRESHMAX:
    *tag = ELELIMMAX;
    break;
  case CNF_ELEALARMTHRESHMIN:
    *tag = ELELIMMIN;
    break;
/* ########### CAMERA ################ */
  case CNF_VIDEODISPLAYSET:
    *tag = VIDEODISPLAYSET;
    break;
  case CNF_VIDEOSERIAL:
    *tag = VIDEOSERIAL;
    break;
  case CNF_VIDEODESCRIPTION:
    *tag = VIDEODESCRIPTION;
    break;
  case CNF_CAMERAFITTED:
    *tag = CAMERAFITTED;
    break;
  case CNF_VIDEOWTALERT:
    *tag = VIDEOWTALERT;
    break;
/* ########### HEATED OPTICS / TEMPERATURE */
  case CNF_WINDOWHEATER:
    *tag = WINDOWHEATER;
    break;
  case CNF_WHEATERONTEMP:
    *tag = WHEATERONTEMP;
    break;
  case CNF_WHEATEROFFTEMP:
    *tag = WHEATEROFFTEMP;
    break;
/* ######## Moisture */
  case CNF_MOISTURESW:
    *tag = MOISTURESW;
    break;
  case CNF_MOISTURETHRESH:
    *tag = MOISTURETHRESH;
    break;

/* ########### MASK - (Not implemented) ############## */
  case CNF_MASKSETTING:
    /* *tag=MASKSETTING; */
    break;
  case CNF_MASKXCENTRE:
    /* *tag=MASKXCENTRE; */
    break;
  case CNF_MASKYCENTRE:
    /* *tag=MASKYCENTRE; */
    break;
  case CNF_MASKXSIZE:
    /* *tag=MASKXSIZE; */
    break;
  case CNF_MASKYSIZE:
    /* *tag=MASKYSIZE; */
    break;
/* ########### FDE Logging settings ################ */
  case CNF_LOGENABLE:
    *tag = LOGENABLE;
    break;
  case CNF_LOGFAULT:
    *tag = LOGFAULT;
    break;
  case CNF_LOGOPMDATA:
    *tag = LOGOPMDATA;
    break;
  case CNF_LOGALARM:
    *tag = LOGALARM;
    break;
  case CNF_LOGALARMDATA:
    *tag = LOGALARMDATA;
    break;
  /*
   * case CNF_LOGDSPPROTO:
   *  *tag=LOGDSPPROTO;
   * break;
   */
  case CNF_LOGTEMP24HOUR:
    *tag = LOGTEMP24HOUR;
    break;
  case CNF_LOGRESERVED1:
    *tag = LOGRESERVED1;
    break;
/* ########### NOSEX - (Not implemented) ################ */
  case CNF_LOOPADDRESS:
    *tag = LOOPADDRESS;
    break;
/* ########### RS485 interface select ################ */
  case CNF_FIELDBUSSELECT:
    *tag = FIELDBUSSELECT;
    break;
/* ########### MODBUS ################ */
  case CNF_MODBUSSLAVEADDR:
    *tag = MODBUSSLAVEADDR;
    break;
  case CNF_MODBUSADDROFFSET:
    *tag = MODBUSADDROFFSET;
    break;
  case CNF_MODBUSBAUDRATE:
    *tag = MODBUSBAUDRATE;
    break;
  case CNF_MODBUSPARITY:
    *tag = MODBUSPARITY;
    break;
/* ########### FOR DEVELOPMENT USE ONLY ################ */
  case CNF_JTAGENABLE:
    *tag = TESTBYTEENABLE;
    break;
  case CNF_SERVICEADDRESS:
    *tag = SERVICEADDRESS;
    break;
  case CNF_LOGPRINTLEVEL:
    *tag = LOGPRINTLEVEL;
    break;

  default:
    retval = FAIL;
    break;
  }
  return(retval);
}


/********************************************************************//**
 * NAME
 * InitDceAndFde()
 * DESCRIPTION
 * Checks DCE for 'new build' PCBs. If EEPROM is blank, initalise it with
 * factory default values.
 * Before we initalise or change the configuration, initalise
 * the FDE so changes to the configuration can be logged.
 * PARAMETERS
 * none
 * RETURN VALUE
 * none
 * LOCAL DATA
 *
 * PRECONDITIONS
 * Run once on power up
 * POSTCONDITIONS
 * DCE is checked and ready to use
 * OTHER INFO
 *************************************************************************/
void InitDceAndFde(void)
{
  /*
   * Check for first time powerup
   * if(1){
   */
  if (IsMicroEepromBlank())
  {
    /*
     * First time the controller board has been powered up.
     * Initalise the EEPROM with factory defaults.
     * (As defined in Dce_map[].factorydefaults)
     * This means setting the protected area to factory defaults
     * too, such as test pass / fail flags etc...
     */
    InitDceToFactoryDefault(false);
    /*
     * Put a marker on the 'DCE REVISION' to indicate the DCE has been
     * reinitalised from blank. Done because at this point the
     * FDE is not initalised.
     */
    TagToDceWriteChar(DCEREVISION,0xFEU);
    UpdateChecksum();
    /* reset nonvolatile time stamp to zero Sep 2011, */
    ResetTimestampNonVolatile();
    /* reset FDE */
    InitFde(true);
    /* return; */
  }
  else
  {
    /* init FDE as well */
    InitFde(false);
  }
}

/********************************************************************//**
 * NAME
 * InitDceToFactoryDefault()
 * DESCRIPTION
 * Clear and rewrite all the factory default values to DCE.
 * Factory default values are stored in dce_map[]
 * If the controller board is configured to have a camera
 * fitted, the video defaults to 'ON' and my serial number is copied
 * into the video serial
 * PARAMETERS
 * none
 * RETURN VALUE
 * none
 * LOCAL DATA
 * tag: Current configuration setting being re-initialised
 * index: Index of where to find details in dce_map[]
 * tempstr: used to initialise string data to '/0'
 * PRECONDITIONS
 * POSTCONDITIONS
 * DCE is checked and ready to use
 * OTHER INFO
 *************************************************************************/
void InitDceToFactoryDefault(unsigned char saveprotected)
{
  unsigned char tag,index,starttag;
  char tempstr;
  unsigned char protecteddatastore[DCE_SIZEOFPROTECTED];
  unsigned char addr;
  char serial[SIZEOFMYSERIAL];

  /* Save protected data if required */
  if (saveprotected == true)
  {
    for (addr = 0U; addr < DCE_SIZEOFPROTECTED; addr++)
    {
      protecteddatastore[addr] = Dce[addr];
      KickWdt();
    }
  }
  /* Blank EEPROM before initalisation */
  __no_operation();
  EraseDce();
  __no_operation();
  /* Restore protected data if required */
  if (saveprotected == true)
  {
    __disable_interrupt();
    for (addr = 0U; addr < DCE_SIZEOFPROTECTED; addr++)
    {
      Dce[addr] = protecteddatastore[addr];
      KickWdt();
    }
    __enable_interrupt();
    /*
     * Since the protected area of has been saved and restored,
     * only initalise starting from the beggining of the unprotected
     * DCE address
     */
    starttag = ELELIMMIN + 1U; /* last protected tag */
  }
  else
  {
    /*
     * All data has been erased, including the protected area.
     * Everything will need reinitalising
     */
    starttag = MYSERIALNUM; /* first protected tag */
  }
  /* Reset back to factory defaults */
  for (tag = starttag; tag < ENDMARKER; tag++)
  {
    /*
     * When setting factory default for the following fields:
     * VIDEODISPLAYSET and VIDEOSERIAL are dependant on the CAMERAFITTED
     * configuration field held in protected EEPROM.
     */
    if (tag == VIDEODISPLAYSET)
    {
      /* If camera is fitted, default to video on */
      if (TagToConfigReadChar(CAMERAFITTED))
      {
        TagToDceWriteChar(VIDEODISPLAYSET,DISP_STANDARD);
      }
      else
      {
        TagToDceWriteChar(VIDEODISPLAYSET,DISP_OFF);
      }
      /*
       * Set the default value for the serial number used in the video
       * to the same number stamped onto the guard plate (SER.NO.)
       */
    }
    else if (tag == VIDEOSERIAL)
    {
      /* Firstly, retreive my SerialNumber from DCE */
      TagToConfigReadString(MYSERIALNUM,serial,SIZEOFMYSERIAL);
      /*
       * If the first letter of my S/N is non zero, copy it
       * into the video serial number
       */
      if (serial[0])
      {
        TagToDceWriteString(VIDEOSERIAL,serial,SIZEOFMYSERIAL);
      }
      else
      {
        /* There is no string contained in my S/N, blank the video serial */
        tempstr = '\0';
        TagToDceWriteString(tag,&tempstr,1U);
      }
      /*
       * Copy factory default values from dec_map[] table into EEPROM
       * Traslate the tag to an index number in dec_map[]
       */
    }
    else if (TagToIndex(tag,&index) != ERROR)
    {
      /* Found... What data type is it? */
      if (dce_map[index].size == SIZEOFCHAR)
      {
        /* Write default char data */
        TagToDceWriteChar(tag,(unsigned char)dce_map[index].factorydefault);
      }
      else if (dce_map[index].size == SIZEOFINT)
      {
        /* Write default char data */
        TagToDceWriteInt(tag,dce_map[index].factorydefault);
      }
      else if (dce_map[index].size > SIZEOFINT)
      {
        /* Blank string data. No default value supported */
        tempstr = '\0';
        TagToDceWriteString(tag,&tempstr,1U);
      }
      else
      {
        __no_operation();
      }
    }
    else
    {
      __no_operation();
    }
    KickWdt();
  }
  /* Update revision. */
  TagToDceWriteChar(DCEREVISION,CURRENT_FORMAT_REVISION);
  /* Update checksum */
  UpdateChecksum();
  /* Log the fact the configuration has been reset to factory defaults */
  FdeConfigurationLog(CNFIGLOG_FACTORYDEFAULT,CURRENT_FORMAT_REVISION,
                      (unsigned int)saveprotected);
}


/********************************************************************//**
 * NAME
 * EraseDce()
 * DESCRIPTION
 * Clear whole EEPROM contents
 * To speed things up, only erase addresses that have been used.
 * PARAMETERS
 * none
 * RETURN VALUE
 * none
 * LOCAL DATA
 * address: current address to be erased
 * PRECONDITIONS
 * POSTCONDITIONS
 * All contents of EEPROM set to 0xFF
 * OTHER INFO
 *************************************************************************/
void EraseDce(void)
{
  unsigned int address;
  for (address = 0U; address < DCE_SIZE; address++)
  {
    if (Dce[address] != 0xFFU)
    {
      /* Contains data. Erase it. */
      __disable_interrupt();
      Dce[address] = 0xFFU;
      __enable_interrupt();
      KickWdt();
    }
  }
  /* Move pointer away from this location to prevent corruption */
  SetEepromPointerToDefault();
}


/********************************************************************//**
 * NAME
 * IsMicroEepromBlank()
 * DESCRIPTION
 * Check if all contents of microcontroller EEPROM are blank
 * PARAMETERS
 * none
 * RETURN VALUE
 * true: EEPROM is blank; false: EEprom is not blank
 * LOCAL DATA
 * address: current address to be erased
 * PRECONDITIONS
 * POSTCONDITIONS
 * All contents of EEPROM set to 0xFF
 * OTHER INFO
 *************************************************************************/
unsigned char IsMicroEepromBlank(void)
{
  unsigned int address;
  unsigned char retval = true;
  /* test for non blank data */
  for (address = 0U; address < DCE_SIZE; address++)
  {
    if (Dce[address] != 0xFFU)
    {
      retval = false;
    }
    KickWdt();
    __sleep();
  }
  /* Move pointer away from this location to prevent corruption */
  SetEepromPointerToDefault();
  /* Send back result */
  return(retval);
}

/********************************************************************//**
 * NAME
 * CheckDceRevisionValid()
 * DESCRIPTION
 * Compare the revision of the data written in EEPROM to
 * the revision expected in this version of firmware.
 * If they are different, the formatting of the data in the DCE
 * is incompatible and should be reinitialised back to factory
 * defaults. This situation can only occur after a firmware upgrade
 * PARAMETERS
 * none
 * RETURN VALUE
 * none
 * LOCAL DATA
 * eepromrevision: Revision of data written in DCE
 * PRECONDITIONS
 * POSTCONDITIONS
 * All contents of EEPROM set factory defaults if it is found to be
 * incompatable with this version of code.
 * OTHER INFO
 *************************************************************************/
void CheckDceRevisionValid(void)
{
  unsigned char eepromrevision;
  /* Save copy of value held in DCE */
  eepromrevision = TagToConfigReadChar(DCEREVISION);
  /*
   * Is this a new build detector? If so, the revision should
   * be set to 0xFF. If this is the case, write a log indicating
   * the DCE was blank and has been completley reinitalise
   */
  if (eepromrevision == 0xFEU)
  {
    /*
     * This is a marker for boards that have powered up with a
     * compleatly blank DCE. Log to the FDE now it has been
     * initalised.
     */
    FdeConfigurationLog(CNFIGLOG_FACTORYDEFAULT,CURRENT_FORMAT_REVISION,
                        false);
    /* Update with correct revision. */
    TagToDceWriteChar(DCEREVISION,CURRENT_FORMAT_REVISION);
    /* Update checksum */
    UpdateChecksum();
  }
  else
  /* Does the revision match what the code expects. */
  if (eepromrevision != CURRENT_FORMAT_REVISION)
  {
    /*
     * WARNING! The revision is different. If we need to take steps
     * to rectify the problem, the first part is done here by
     * copying out the required data already held in the DSE
     *  -- Add your code here
     *
     * Reset to factory defaults, Write protect production /
     * test data
     */
    InitDceToFactoryDefault(true);
    /*
     * Copy back required saved configuration data following
     * a reinitalisation of the non protected area of the DCE
     *  -- Add your code here
     *
     * Following such a change, clear any faults to do with
     * configuration faults. This is due to uninitalise data
     * within the DCE
     */
    UpdateHardwareFaultSentinel(CNT_CONFIGINVALID,false,0U);
    /*
     * If a change to the code has been made that requires the
     * FDE to be reinitalised, do so here
     *  -- Add your code here
     *
     * Log a change in revision has occurred
     */
    FdeConfigurationLog(CNFIGLOG_REVISION_CHANGES,(unsigned int)eepromrevision,
                        CURRENT_FORMAT_REVISION);
  }
  else
  {
    __no_operation();
  }
}

/********************************************************************//**
 * NAME
 * TagToConfigReadChar()
 * DESCRIPTION
 * Read byte data from DCE. If EEPROM OVERRIDE switch is false, some
 * configuration settings determined by the DIP switches 1-8
 * PARAMETERS
 * tag: Configuration setting required
 * RETURN VALUE
 * Configuration setting value
 * LOCAL DATA
 * index: Index of where to find details in dce_map[]
 * dipswitch: holder of the current DIP switch settings
 * configdata: Configuration setting
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
unsigned char TagToConfigReadChar(unsigned char const tag)
{
  unsigned char index,configdata = 0U,tmpx,tmpy;
  /* Get the index number of dce_map[] containing tag information */
  if ((TagToIndex(tag,&index) == ERROR) && (firstentrytesttag == true))
  {
    /* Invalid tag entry in dce_map[]. FAULT. */
    configdata = 0U;
    /* prevent multiple log entries */
    if (ReadHardwareFaultFlag(CNT_CONFIGINVALID) == false)
    {
      FdeConfigurationLog(CNFIGLOG_READCHARTAGINVALID,
                          (unsigned int)tag,0U);
    }
    /* Put detecor into fault */
    UpdateHardwareFaultSentinel(CNT_CONFIGINVALID,true,0U);
  }
  else
  {
    /* for MX modes Alarm and Fault state cannot be latched */
    if (((interfacemode == MX_IFACE_MODE) || (interfacemode == MX_IFACE_DIPSW_ACTIVE))
        && ((tag == FAULTLATCHING) || (tag == ALARMLATCHING)))
    {
      KickWdt();
      /* return(false); */
      configdata = false;
    }
    /*
     * Valid tag entry in dce_map[], continue
     * Some fields can be overridden by the DIP switch. Check the mode.
     */
    else if ((dipswitchvalue & MASTER_SW) && ((tag == FAULTLATCHING) || (tag == ALARMLATCHING)
                                              || (tag == ALARMDELAY) || (tag == OPMINHIBIT) ||
                                              (tag == INTERFACES) || (tag == WINDOWHEATER) ||
                                              (tag == DETECT_RANGE) || (tag == HARTSELECT) ||
                                              (tag == MXMODE1) || (tag == _4_20MAMODE1) ||
                                              (tag == _4_20MAMODE2) || (tag == AUXSUPPLY)))
    {

      if (tag == FAULTLATCHING)
      {
        if (dipswitchvalue & FAULT_LATCHSW_NU)
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == ALARMLATCHING)
      {
        if (dipswitchvalue & ALARM_LATCHSW_NU)
        {
          configdata = false;
        }
        else
        {
          configdata = true;
        }
      }
      else if (tag == ALARMDELAY)
      {
        AlarmDILSwitchBitRead(dipswitchvalue, &tmpx, &tmpy);
        configdata = tmpx;
      }
      else if (tag == OPMINHIBIT)
      {
        if ((dipswitchvalue >> SHIFT_BY_BYTE) & OPM_SW_NU)
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == INTERFACES)
      {
        /* convert to DCE tags */
        if ((interfacemode == ONLY_4_20_DIS) || (interfacemode == ONLY_4_20_VAR))
        {
          configdata = DCE_4_20;
        }
        else if ((interfacemode == DEFVAR_4_20MA_MODE) || (interfacemode == DEFAULT_4_20MA_MODE))
        {
          configdata = DCE_DEFAULT_4_20MA_MODE;
        }
        else if (interfacemode == CONVENT_MODE)
        {
          configdata = DCE_CONVENTIONAL;
        }
        else if ((interfacemode == MX_IFACE_MODE) || (interfacemode == MX_IFACE_DIPSW_ACTIVE))
        {
          configdata = DCE_MX;
        }
        else
        {
          __no_operation();
        }
      }
      else if (tag == WINDOWHEATER)
      {
        if ((dipswitchvalue >> SHIFT_BY_BYTE) & WINDOW_HEATER_SW_NU)
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == DETECT_RANGE)
      {
        configdata = RangeDILSwitchBitRead(dipswitchvalue);
      }
      else if (tag == HARTSELECT)
      {
        if ((dipswitchvalue >> SHIFT_BY_BYTE) & HARTPROTICOLSW)
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == MXMODE1)
      {
        if (interfacemode == MX_IFACE_DIPSW_ACTIVE)
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == _4_20MAMODE1)
      {
        if ((interfacemode == ONLY_4_20_VAR) || (interfacemode == DEFVAR_4_20MA_MODE))
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == _4_20MAMODE2)
      {
        if ((interfacemode == ONLY_4_20_DIS) || (interfacemode == ONLY_4_20_VAR) ||
            (interfacemode == DEFVAR_4_20MA_MODE) || (interfacemode == DEFAULT_4_20MA_MODE))
        {
          if (enhancedbandbitval)
          {
            configdata = true;
          }
          else
          {
            configdata = false;
          }
        }
        else
        {
          configdata = false;
        }
      }
      else if (tag == AUXSUPPLY)
      {
        if (dipswitchvalue & AUXSUPPLYSWITCH)
        {
          configdata = true;
        }
        else
        {
          configdata = false;
        }
      }
      else
      {
        __no_operation();
      }

    }
    else
    {
      /*
       * Otherwise we are not in DIP switch override
       * Get value from DCE
      */ //if (ReturnDebugMenuState() == true){printf_P("index=%d\r",index);}
      configdata = Dce[dce_map[index].address];
      /*
       * Verify the configuration paramerter is within the range
       * limits specified in dce_map[].minval and dce_map[].maxval
       */
      if ((dce_map[index].maxval == 0U) && (dce_map[index].minval == 0U))
      {
        /* All ranges valid on this parameter. No checking performed */
        __no_operation();
      }
      else if ((configdata > dce_map[index].maxval) ||
               (configdata < dce_map[index].minval))
      {
        /*
         * We have an out of range config value. Use
         * factory default to prevent software exception
         * Are we already in fault? If so prevent multiple log entries
         */
        if ((ReadHardwareFaultFlag(CNT_CONFIGINVALID) == false) &&
            ((tag>LOGRESERVED1)||(tag<LOGENABLE)))//was tag < Logenable
        {
          /*
           * Don't write a log if the configuration item is
           * to do with logging. This prevents infinate recurion.
           */
          FdeConfigurationLog(CNFIGLOG_READCHARRANGE,
                              (unsigned int)tag,(unsigned int)configdata);
          /*
           * Set to hardware fault. Again, updating the fault
           * state also writes a log. For logging, just use defaults
           * printf_P("\n\r index=%d,config=%d,arr=%d",index,configdata,dce_map[index].address);
           */
          UpdateHardwareFaultSentinel(CNT_CONFIGINVALID,true,1U);         /* active */
        }
        /* Use factory default instead and continue. */
        configdata = (unsigned char)dce_map[index].factorydefault;
      }
      else
      {
        __no_operation();
      }
      /* Move pointer away from this location to prevent corruption */
      SetEepromPointerToDefault();
    }
  }
  KickWdt();
  return(configdata);
}

/********************************************************************//**
 * NAME
 * TagToConfigReadInt()
 * DESCRIPTION
 * Read int data from DCE.
 * PARAMETERS
 * tag: Configuration setting required
 * RETURN VALUE
 * Configuration setting value
 * LOCAL DATA
 * highbyte,lowbyte: temp holder for converting char to int
 * index: Index of where to find details in dce_map[]
 * configdata: Configuration setting
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
unsigned int TagToConfigReadInt(unsigned char const tag)
{
  unsigned int lowbyte, highbyte, configdata;
  unsigned char index;

  /* Get the index number of dce_map[] containing tag information */
  if (TagToIndex(tag,&index) == ERROR)
  {
    /* Invalid tag entry in dce_map[]. FAULT. */
    configdata = 0U;
    /* prevent multiple log entries */
    if (ReadHardwareFaultFlag(CNT_CONFIGINVALID) == false)
    {
      FdeConfigurationLog(CNFIGLOG_READINTTAGINVALID,
                          (unsigned int)tag,0U);
    }
    /* Put detecor into fault */
    UpdateHardwareFaultSentinel(CNT_CONFIGINVALID,true,2U);
  }
  else
  {
    /* Valid tag entry in dce_map[], continue */
    highbyte = Dce[dce_map[index].address];
    lowbyte = Dce[(dce_map[index].address) + 1U];
    /* Move pointer away from this location to prevent corruption */
    SetEepromPointerToDefault();
    configdata = (lowbyte | (highbyte << SHIFT_BY_BYTE) & 0xFF00U);
  }
  KickWdt();
  return(configdata);
}

/********************************************************************//**
 * NAME
 * TagToConfigReadString()
 * DESCRIPTION
 * Read string data from DCE.
 * PARAMETERS
 * tag: Configuration setting required
 * size: number of bytes to read from configuration
 * RETURN VALUE
 * stringdata: Address of read string data
 * LOCAL DATA
 * index: Index of where to find details in dce_map[]
 * indexoffset: position of string read pointer
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
void TagToConfigReadString(unsigned char const tag,
                           char *stringdata,
                           unsigned char size)
{
  unsigned char index, indexoffset,data;
  /* Get the index number of dce_map[] containing tag information */
  if (TagToIndex(tag,&index) == ERROR)
  {
    /* Invalid tag entry in dce_map[]. FAULT. */
    *stringdata = '\0';
    /* prevent multiple log entries */
    if (ReadHardwareFaultFlag(CNT_CONFIGINVALID) == false)
    {
      FdeConfigurationLog(CNFIGLOG_READSTRINGTAGINVALID,
                          (unsigned int)tag,0U);
    }
  }
  else
  {
    /*
     * Valid tag entry in dce_map[], continue:
     * Loop until the destination string has been filled
     */
    for (indexoffset = 0U; (indexoffset < size); indexoffset++)
    {
      /* Still data to copy from the configuration EEPROM */
      if (indexoffset < dce_map[index].size)
      {
        data = Dce[dce_map[index].address + indexoffset];
      }
      else
      {
        /*
         * We have reached the end of the source string
         * pad the rest of the destination with zeros
         */
        data = 0U;
      }
      /* copy to destination */
      *stringdata++ = data;
    }
    SetEepromPointerToDefault();
  }
  KickWdt();
}

/********************************************************************//**
 * NAME
 * TagToDceWriteChar()
 * DESCRIPTION
 * Write byte data to DCE. Range check the data to be written. If out
 * of range, the DCE is not updated
 * PARAMETERS
 * tag: Configuration setting to be updated
 * data: byte to write to DCE
 * RETURN VALUE
 * retval: PASS FAIL error code
 * LOCAL DATA
 * index: Index of where to find details in dce_map[]
 * retval: Returns true if DCE was updated
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
unsigned char TagToDceWriteChar(unsigned char const tag, unsigned char const data)
{
  unsigned char index,retval = FAIL;

  /*
   * SetEepromPointerToDefault();//placed here on june 2011
   * Get the index number of dce_map[] containing tag information
   */
  if (TagToIndex(tag,&index) != ERROR)
  {
    /*
     * Valid tag entry in dce_map[]. Is the value we are trying to
     * write to this field within range?
     */
    if ((tag == TESTBYTEENABLE) || (IndexToValueWithinLimits(index,(unsigned int)data) == PASS))
    {
      /* Yes, it is within limits. */
      __disable_interrupt();
      Dce[dce_map[index].address] = data;
      __enable_interrupt();
      /* Move pointer away from this location to prevent corruption */
      SetEepromPointerToDefault();
      /* Log the configuration change to EEPROM */
      FdeConfigurationLog(CNFIGLOG_USER_CONFIG_UPDATE,(unsigned int)tag,(unsigned int)data);
      /* return success */
      retval = PASS;
    }
  }
  KickWdt();
  return(retval);
}

/********************************************************************//**
 * NAME
 * TagToDceWriteInt()
 * DESCRIPTION
 * Write int data to DCE. Range check the data to be written. If out
 * of range, the DCE is not updated
 * PARAMETERS
 * tag: Configuration setting to be updated
 * data: byte to write to DCE
 * RETURN VALUE
 * retval: PASS FAIL error code
 * LOCAL DATA
 * highbyte,lowbyte: temp holder for converting char to int
 * index: Index of where to find details in dce_map[]
 * retval: Returns true if DCE was updated
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
unsigned char TagToDceWriteInt(unsigned char const tag, unsigned int const data)
{
  unsigned char lowbyte, highbyte;
  unsigned char index, retval = FAIL;
  /* Get the index number of dce_map[] containing tag information */
  if (TagToIndex(tag,&index) != ERROR)
  {
    /*
     * Valid tag entry in dce_map[]. Is the value we are trying to
     * write to this field within range?
     */
    if (IndexToValueWithinLimits(index,data) == PASS)
    {
      /* Yes, it is within limits, continue. */
      highbyte = (unsigned char)((data >> SHIFT_BY_BYTE) & 0x00FFU);
      lowbyte = (unsigned char)(data & 0x00FFU);
      /* Write bytes to DCE */
      __disable_interrupt();
      Dce[dce_map[index].address] = highbyte;
      Dce[dce_map[index].address + 1U] = lowbyte;
      __enable_interrupt();
      /* Move pointer away from this location to prevent corruption */
      SetEepromPointerToDefault();
      /*
       * Log the configuration change to EEPROM: Don't log
       * updates to the checksum
       */
      if (tag != DCECHECKSUM)
      {
        FdeConfigurationLog(CNFIGLOG_USER_CONFIG_UPDATE,(unsigned int)tag,data);
      }
      /* return success */
      retval = PASS;
    }
  }
  KickWdt();
  return(retval);
}

/********************************************************************//**
 * NAME
 * TagToDceWriteString()
 * DESCRIPTION
 * Write string data to DCE.
 * PARAMETERS
 * tag: Configuration setting to be updated
 * stringdata: Address of where to find string data
 * RETURN VALUE
 * none
 * LOCAL DATA
 * index: Index of where to find details in dce_map[]
 * indexoffset: position of string read pointer
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
void TagToDceWriteString(unsigned char const tag, const char *stringdata,
                         unsigned char size)
{
  unsigned char index, indexoffset = 0U;
  /* Get the index number of dce_map[] containing tag information */
  if (TagToIndex(tag,&index) != ERROR)
  {
    /* Valid tag entry in dce_map[],continue: */
    for (indexoffset = 0U; ((indexoffset < size) && (indexoffset < dce_map[index].size)); indexoffset++)
    {
      __disable_interrupt();
      Dce[dce_map[index].address + indexoffset] = *stringdata++;
      __enable_interrupt();
      KickWdt();
    }
    /* Any space left over? fill with '\0' */
    while (indexoffset < dce_map[index].size)
    {
      __disable_interrupt();
      Dce[dce_map[index].address + indexoffset] = '\0';
      __enable_interrupt();
      indexoffset++;
      /* Long string writes need to kick the WDT */
      KickWdt();
    }
    /* Move pointer away from this location to prevent corruption */
    SetEepromPointerToDefault();
    /* Log the configuration change to EEPROM */
    FdeConfigurationLog(CNFIGLOG_USER_CONFIG_UPDATE,(unsigned int)tag,(unsigned int)size);
  }
  KickWdt();
}

/********************************************************************//**
 * NAME
 * TagToIndex()
 * DESCRIPTION
 * Translate a tag (from enum) to an index number of dce_map[] that
 * contains the start address and size. Error checking included.
 * ERROR CHECK: This function can be used to check if a tag has a
 * valid entry in the dce_map[] table.
 * PARAMETERS
 * tagtofind: Configuration setting to find in dce_map[]
 * index: address for stringdata: Address of where to find string data
 * RETURN VALUE
 * return ERROR if tag is not found in dce_map[]
 * LOCAL DATA
 * index: Index of dce_map[] where tag details can be found
 * retval: Tag entry not found flag
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
int TagToIndex(unsigned char const tagtofind, unsigned char *index)
{
  unsigned char indexsearch;
  int retval = 0;
  /* Search through and drop out if enum is found */
  for (indexsearch = 0U; ((dce_map[indexsearch].tag != ENDMARKER) && (dce_map[indexsearch].tag != tagtofind)); indexsearch++)
  {}
  /* Check if tag was found */
  if (indexsearch == ENDMARKER)
  {
    /* Reached end without finding an entry in dce_map[] for the tag */
    retval = ERROR; /* ERROR=-1; */
  }
  *index = indexsearch;
  return(retval);
}

/********************************************************************//**
 * NAME
 * TagToSize()
 * DESCRIPTION
 * Translate a tag (from enum) to the size of the configuration data
 * PARAMETERS
 * tagtofind: Configuration setting to find in dce_map[]
 * RETURN VALUE
 * return ERROR if tag is not found in dce_map[]
 * LOCAL DATA
 * retval: Tag entry not found flag
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
int TagToSize(unsigned char *sizeofdata, unsigned char const tagtofind)
{
  unsigned char index;
  int retval;
  retval = TagToIndex(tagtofind,&index);
  if (retval != ERROR)
  {
    *sizeofdata = dce_map[index].size;
  }
  else
  {
    *sizeofdata = 0U;
  }
  return(retval);
}

/********************************************************************//**
 * NAME
 * TagToDefault()
 * DESCRIPTION
 * Translate a tag (from enum) to the factory default value
 * PARAMETERS
 * tagtofind: Configuration setting to find in dce_map[]
 * RETURN VALUE
 * return ERROR if tag is not found in dce_map[]
 * LOCAL DATA
 * retval: Tag entry not found flag
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
int TagToDefault(unsigned int *defaultvalue, unsigned char const tagtofind)
{
  unsigned char index;
  int retval;

  retval = TagToIndex(tagtofind,&index);
  if (retval != ERROR)
  {
    *defaultvalue = (unsigned int)dce_map[index].factorydefault;
  }
  else
  {
    *defaultvalue = 0U;
  }
  return(retval);
}

/********************************************************************//**
 * NAME
 * IndexToValueWithinLimits()
 * DESCRIPTION
 * Translate a tag (from enum) and a value to a flag indicating if
 * the value is within the limits specified in the EEPROM address map.
 * PARAMETERS
 * index: Tag to DCE index lookup
 * value: value to be range checked
 * RETURN VALUE
 * return FAIL if value is out of range. Otherwise, return PASS.
 * LOCAL DATA
 * retval: Tag entry not found flag
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 *************************************************************************/
unsigned char IndexToValueWithinLimits(unsigned char const index,
                                       unsigned int const value)
{
  /* Default to error */
  unsigned char retval = FAIL;
  /* Is range checking performed on this field? */
  if ((dce_map[index].maxval == 0U) && (dce_map[index].minval == 0U))
  {
    /* All ranges valid on this parameter. No checking performed */
    retval = PASS;
  }
  else
  {
    /* Range checking is to be performed */
    if ( (value >= (unsigned int)((dce_map[index].minval) & 0x00FFU)) &&
         (value <= (unsigned int)((dce_map[index].maxval) & 0x00FFU)) )
    {
      /* Within limits specified. */
      retval = PASS;
    }
  }
  return(retval);
}

/********************************************************************//**
 * NAME
 * CalculateChecksum()
 *
 * DESCRIPTION
 * Read the contents of the DCE and calculate the checksum value
 *
 * RETURN VALUE
 * chksum: 16 bit checksum value of DCE data
 * LOCAL DATA
 * index: Current byte of DCE being read
 * chksum: current checksum value
 * tempchksum: holder for previous byte checksum value
 * bitindex: increment through bit of current byte
 * databyte: current data byte being checksumed
 *
 * OTHER INFO
 * Use CCITT checksum
 *************************************************************************/
unsigned int CalculateChecksum(void)
{
  unsigned short index = 0U, checksum = 0U;
  /* Start the test with a WDT kick */
  KickWdt();
  
  if((ReturnTestByte() & 0xFF) != 0x10U)
  {
    /* Start at the beginning of the DCE and calculate the checksum */
    for (index = 0U; index < (DCE_SIZE - SIZEOFINT); index++)
    {
      checksum = g_sprog_crc16_table[(checksum >> SHIFT_BY_BYTE) 
        ^ (INT16u)Dce[index]] ^ (checksum << SHIFT_BY_BYTE);
    }
  }
  else if((ReturnTestByte() & 0xFFU) == 0x10U)
  {
    for (index = 1U; index < (DCE_SIZE - SIZEOFINT); index++)
    {
      checksum = g_sprog_crc16_table[(checksum >> SHIFT_BY_BYTE) 
        ^ (INT16u)Dce[index]] ^ (checksum << SHIFT_BY_BYTE);
    }
  }
  else{}
  
  /* tail test with WDT kick */
  KickWdt();
  /* Move pointer away from this location to prevent corruption */
  SetEepromPointerToDefault();
  return(checksum);
}

/********************************************************************//**
 * NAME
 * UpdateChecksum()
 * DESCRIPTION
 * Read the contents of the DCE and calculate the checksum value
 * update the checksum value in EEPROM
 * PARAMETERS
 * RETURN VALUE
 * chksum: 16 bit checksum value of DCE data
 * LOCAL DATA
 * chksum: current checksum value
 * PRECONDITIONS
 * POSTCONDITIONS
 * DCECHECKSUM tag in the configuration is updated
 * OTHER INFO
 * Use CCITT checksum
 *************************************************************************/
unsigned int UpdateChecksum(void)
{
  unsigned int chksum = 0U;
  
  if(ReturnTestByte() == 0x01U)
  {
   return(TagToConfigReadInt(DCECHECKSUM));
  }
  /* Get checksum */
  chksum = CalculateChecksum();
  /* Write new checksum */
  TagToDceWriteInt(DCECHECKSUM,chksum);
  /* Return new checksum */
  return(chksum);
}

/********************************************************************//**
 * NAME
 * VerifyChecksumMatch()
 * DESCRIPTION
 * Read the contents of the DCE and calculate the checksum value
 * Check to see if the calculated matches the value in EEPROM
 * PARAMETERS:
 * calcchksum: pointrt to returned calculated checksum
 * eepromchksum: ptr to returned checksum value read directly from DCE
 * RETURN VALUE
 * PASS: checksum matches, FAIL: checksum mismatch
 * LOCAL DATA
 * retval: PASS fail flag
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 * Use CCITT checksum
 *************************************************************************/
unsigned char VerifyChecksumMatch(unsigned int *calcchksum,
                                  unsigned int *eepromchksum)
{
  unsigned char retval;
  /* Read the checksum values */
  *calcchksum = CalculateChecksum();
  *eepromchksum = TagToConfigReadInt(DCECHECKSUM);
  /* Compare */
  if (*calcchksum == *eepromchksum)
  {
    retval = PASS;
  }
  else
  {
    retval = FAIL;
  }
  return(retval);
}

/********************************************************************//**
 * NAME
 * DceChecksumSelftest()
 * DESCRIPTION
 * Verify DCE checksum. If there is a checksum missmatch, put detector
 * into fault
 * PARAMETERS:
 * updatefaultsentinel: TRUE: Update fault state on checksum test
 *                      FALSE: do not update fault state
 * RETURN VALUE
 * PASS: checksum matches, FAIL: checksum mismatch
 * LOCAL DATA
 * calcchksum: pointrt to returned calculated checksum
 * eepromchksum: ptr to returned checksum value read directly from DCE
 * retval: PASS fail flag
 * PRECONDITIONS
 * POSTCONDITIONS
 * OTHER INFO
 * Use CCITT checksum
 *************************************************************************/
unsigned char DceChecksumSelftest(unsigned char udpatefaultstate)
{
  unsigned int calcchksum, eepromchksum;
  unsigned char retval;
  /* Carry out selftest */
  retval = VerifyChecksumMatch(&calcchksum, &eepromchksum);
  /* Update hardware fault sentinel if requested */
  if (udpatefaultstate == true)
  {
    if (retval == FAIL)
    {
      /*
       * The DCE checksum has failed, Data may be corrupted. Log to FDE
       * Do not log if already in CNT_DCECHECKSUM fault.
       * This prevents multiple log entries.
       */
      if (ReadHardwareFaultFlag(CNT_DCECHECKSUM) == false)
      {
        FdeConfigurationLog(CNFIGLOG_CHECKSUM,calcchksum,eepromchksum);
      }
      /*
       * We are unable to rely on the data contained within the
       * configuration. Signal hardware fault.
       */
     // delayms(5U);
      UpdateHardwareFaultSentinel(CNT_DCECHECKSUM,true,0U);
    }
    else
    {
      /* Checksum now matches. Clear fault */
      UpdateHardwareFaultSentinel(CNT_DCECHECKSUM,false,0U);
    }
  }
  /* Return result of test */
  KickWdt();
  return(retval);
}

/********************************************************************//**
 * NAME
 * SetEepromPointerToDefault()
 * DESCRIPTION
 * Sets the AVR EEPROM read/write address pointer to an unused bit
 * of memory (Address 0). This was done after seeing checksum fails
 * following erronious writes to other addresses.
 * The following is a reponse from atmel:
 * Some people may argue that address 0 is more vulnerable than other
 * addresses since the EEAR register will most often have address 0,
 * and if a brown-out should occur and the EEPROM write command is
 * erroneously issues, this address is the most likely to be corrupted.
 * PARAMETERS:
 * RETURN VALUE:
 * LOCAL DATA
 * dummy: used as a destination register when reading EEPROM address 0
 * PRECONDITIONS
 * POSTCONDITIONS
 * After running the EEAR register will be set to zero.
 * OTHER INFO
 *************************************************************************/
void SetEepromPointerToDefault(void)
{
  /*
   * Reset the EEPROM address to pointer to an unused location.
   * Set registers to known setting
   * eeprom control regsiter
   * __disable_interrupt();
   */
  EECR = 0U;
  /* try move pointer to last address May 2011 */
  EEARH = 0U; /* 0x0F */
  EEARL = 0U; /* 0xFF */
  /* eeprom data register */
  EEDR = 0U;
  /* __enable_interrupt(); */
}

/********************************************************************//**
 * NAME
 * GetTestData()
 * DESCRIPTION
 * Caller specifies TestType. (see TestType enum for CMD_GETTESTDATA)
 * Function gets the test data from the controller or sensor board
 * PARAMETERS:
 * testtype: Enum of test type to Get data
 * RETURN VALUE:
 * testdate,testresult,testiteration,reserved: Pointers to write test
 *  data to
 * LOCAL DATA:
 * retval: Return state. SPIRET_MESSAGERECEIVED means test data correctly
 *  read from EEPROM, SPIRET_UNKNOWN means TestType is unknown, otherwise
 *  the DSP failt state is returned
 * address: Address (or tag) of where the data is held in the controller
 *  board EEPROM
 * PRECONDITIONS
 * When requesting sensor board test data, the sensor board must be attached
 * POSTCONDITIONS
 * none
 *************************************************************************/
unsigned char GetTestData(unsigned char testtype, signed char testdate[],
                          unsigned char *testresult, unsigned char *testiteration,
                          unsigned char reserved[])
{
  unsigned char retval = SPIRET_MESSAGERECEIVED,loopcntr,intcnt = 0U;
  unsigned char onewirerecords[16];
  /* Determine where this data is held, on the Controller or sensor board? */
  if (testtype <= TEST_POTTING)  /* was >TEST_CONTROLLER */
  {   /*
       * It is held on the controller board
       * Lookup where in the configuration this data is held
       */
    switch (testtype)
    {
    case TEST_CONTROLLER:
      TagToConfigReadString(CONTTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      *testresult = TagToConfigReadChar(CONTTESTRESULT);
      *testiteration = TagToConfigReadChar(CONTTESTITERAT);
      TagToConfigReadString(CONTTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;
    case TEST_TOPCASEFUNC:
      TagToConfigReadString(FNCTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      *testresult = TagToConfigReadChar(FNCTESTRESULT);
      *testiteration = TagToConfigReadChar(FNCTESTITERAT);
      TagToConfigReadString(FNCTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;
    case TEST_ENVIRONMENTAL:
      TagToConfigReadString(ENVTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      *testresult = TagToConfigReadChar(ENVTESTRESULT);
      *testiteration = TagToConfigReadChar(ENVTESTITERAT);
      TagToConfigReadString(ENVTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;
    /* POtted controller results */
    case TEST_POTTING:
      TagToConfigReadString(POTTINGTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      *testresult = TagToConfigReadChar(POTTINGTESTRESULT);
      *testiteration = TagToConfigReadChar(POTTINGTESTITERAT);
      TagToConfigReadString(POTTINGTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;

    /* sensor board data */
    case TEST_SENSORINITIAL:
      __disable_interrupt();
      ReadPageOfSCE(TESTANDDATERECORDSINETIAL,&onewirerecords[0],SIZEOFTESTDATA);
      __enable_interrupt();
      for (loopcntr = 0U; loopcntr < SIZEOFDATE; loopcntr++)
      {
        testdate[loopcntr] = (signed char)onewirerecords[loopcntr];
      }
      *testresult = onewirerecords[6];
      *testiteration = onewirerecords[7];
      for (loopcntr = 8U; loopcntr < TEST_RESERVED + 8U; loopcntr++)
      {
        reserved[intcnt] = onewirerecords[loopcntr];
        intcnt++;
      }
      break;
    case TEST_SENSORSECOND:
      __disable_interrupt();
      ReadPageOfSCE(TESTANDDATERECORDSSECOND,&onewirerecords[0],SIZEOFTESTDATA);
      __enable_interrupt();
      for (loopcntr = 0U; loopcntr < SIZEOFDATE; loopcntr++)
      {
        testdate[loopcntr] = (signed char)onewirerecords[loopcntr];
      }
      *testresult = onewirerecords[6];
      *testiteration = onewirerecords[7];
      for (loopcntr = 8U; loopcntr < TEST_RESERVED + 8U; loopcntr++)
      {
        reserved[intcnt] = onewirerecords[loopcntr];
        intcnt++;
      }
      break;
    default:
      retval = SPIRET_UNKNOWN;
      break;
    }
  }
  else
  {
    /* Data is held on the sensor board. Go get it. */
    retval = SPIRET_MESSAGERECEIVED;
  }
  KickWdt();
  return(retval);
}

/********************************************************************//**
 * NAME
 * SetTestData()
 * DESCRIPTION
 * Caller specifies TestType. (see TestType enum for CMD_GETTESTDATA)
 * and supplies the test data for the controller or sensor board
 * PARAMETERS:
 * testtype: Enum of test type to Get data
 * testdate,testresult,testiteration,reserved: Pointers to read test
 *  data from and store in EEPROM
 * RETURN VALUE:
 * retval:
 * errorvalue:
 * LOCAL DATA:
 * retcal: Return state. SPIRET_MESSAGERECEIVED means test data correctly
 *  read from EEPROM, SPIRET_UNKNOWN means TestType is unknown, otherwise
 *  the DSP fault state is returned
 * address: Address (or tag) of where the data is held in the controller
 *  board EEPROM
 * PRECONDITIONS
 * When requesting sensor board test data, the sensor board must be attached
 * POSTCONDITIONS
 * none
 *************************************************************************/
unsigned char SetTestData(unsigned char *errorvalue,
                          unsigned char testtype, signed char testdate[],
                          unsigned char testresult, unsigned char testiteration,
                          unsigned char reserved[])
{
  unsigned char retval = SPIRET_MESSAGERECEIVED;
  unsigned char localerrorvalue = ERROR_NONE;
  /* Determine where this data is to be written, Controller or sensor? */
  if ((testtype <= TEST_POTTING) && ((testtype != TEST_SENSORINITIAL) ||
                                     (testtype != TEST_SENSORSECOND)))
  {
    /*
     * Destined for the controller board
     * Lookup where in the configuration this data is held
     */
    switch (testtype)
    {
    case TEST_CONTROLLER:
      TagToDceWriteString(CONTTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      TagToDceWriteChar(CONTTESTRESULT,testresult);
      TagToDceWriteChar(CONTTESTITERAT,testiteration);
      TagToDceWriteString(CONTTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;
    case TEST_TOPCASEFUNC:
      TagToDceWriteString(FNCTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      TagToDceWriteChar(FNCTESTRESULT,testresult);
      TagToDceWriteChar(FNCTESTITERAT,testiteration);
      TagToDceWriteString(FNCTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;
    case TEST_ENVIRONMENTAL:
      TagToDceWriteString(ENVTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      TagToDceWriteChar(ENVTESTRESULT,testresult);
      TagToDceWriteChar(ENVTESTITERAT,testiteration);
      TagToDceWriteString(ENVTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;
    case TEST_POTTING:
      TagToDceWriteString(POTTINGTESTDATE,(char *)&testdate[0],SIZEOFDATE);
      TagToDceWriteChar(POTTINGTESTRESULT,testresult);
      TagToDceWriteChar(POTTINGTESTITERAT,testiteration);
      TagToDceWriteString(POTTINGTESTRESERVED,(char *)&reserved[0],TEST_RESERVED);
      break;

    default:
      retval = SPIRET_UNKNOWN;
      break;
    }
    /* Update DCE checksum */
    UpdateChecksum();
  }
  else
  {
    /* Destined for the sensor board. Pass it on. */
    localerrorvalue = ERROR_NONE;
    retval = SPIRET_MESSAGERECEIVED;
  }
  /* Set error value to true if value was sucessfully committed to EEPROM */
  *errorvalue = (unsigned char)(localerrorvalue == ERROR_NONE);
  KickWdt();
  return retval;
}

/* ######### DEBUG SUPPORT FUNCTION ############ */
__farflash char dcestr41[] = "<ESCAPE>. Press Escape again to return to menu\r\n";
void PrintEepromData(unsigned int startaddress, unsigned int size)
{
  unsigned int address;
  unsigned int loop;
  unsigned char disable_linebreak = true;

  printf_P("Address=%u) ",startaddress);

  for (loop = 0U; loop < size; loop++)
  {
    address = startaddress + loop;
    /* Every 10 bytes prined out, <CR> and print address */
    if ( ((address % 10U) == 0x00U) && (disable_linebreak == false) )
    {
      printf_P("\r\n");
      printf_P("Address=%u) ",address);
    }
    printf_P("%03d, ",Dce[address]);
    disable_linebreak = false;
  }
  printf_P("\r\n");
  /* Move pointer away from this location to prevent corruption */
  SetEepromPointerToDefault();
}

void GetAndWriteDceData(unsigned char data)
{
  char keybuf[5];
  unsigned int address;

  printf_P("Now enter ADDRESS (dec)\r\n");

  if (getstring_small(keybuf, sizeof(keybuf)) != ERROR)
  {
    address = atoi_small(keybuf);
    /* Write data to EEPROM */
    __disable_interrupt();
    Dce[address] = data;
    __enable_interrupt();
    /* Message to help user to find the menu */
    printf_P("___! NOW PRESS <ESC> TO RETURN TO MENU !___\r\n");
  }
  else
  {
    printf_P(dcestr41);
  }
  /* Move pointer away from this location to prevent corruption */
  SetEepromPointerToDefault();
}
/********************************************************************//**
*  PrintAlConfigData();
*  // These function are used for debug prints only. They are not called
*  // as part of normal operation
************************************************************************/
void PrintAllConfigData(void)
{
  unsigned char index,tag;
  unsigned char chardata, count;
  unsigned int intdata;
  char stringdata[25];

  for (tag = 0U; tag < ENDMARKER; tag++)
  {
    /* Traslate the tag to an index number in dec_map[] */
    if (TagToIndex(tag,&index) == ERROR)
    {
      /* Entry not found. Print error message */
      printf_P(" does not exist in dce_map[]\r\n",tag);
    }
    else
    {
      printf_P("TAG=%03d",tag);
      /* Found... If it is a string, print it */
      if (dce_map[index].size == SIZEOFCHAR)
      {
        /* Get char data and print it */
        chardata = TagToConfigReadChar(tag);
        printf_P(", Addr=%04ud, Val=%03d (0x%04X), ",dce_map[index].address,chardata,chardata);
      }
      else if (dce_map[index].size == SIZEOFINT)
      {
        /* Get char data and print it */
        intdata = TagToConfigReadInt(tag);
        printf_P(", Addr=%04ud, Val=%05u (0x%04X), ",dce_map[index].address,intdata,intdata);
      }
      else if (dce_map[index].size > SIZEOFINT)
      {
        /* Clear string */
        for (count = 0U; count < sizeof(stringdata); count++)
        {
          stringdata[count] = '\0';
        }
        /* get data from DCE */
        TagToConfigReadString(tag,stringdata,sizeof(stringdata));
        /* Replace non-printable chars with '?' */
        for (count = 0U; count < dce_map[index].size; count++)
        {
          if (ischarprintable(stringdata[count]) == false)
          {
            stringdata[count] = '?';
          }
        }
        /* It must be a string (doubles not supported). Print it. */
        printf_P(", Addr=%04ud, Str=[%s], ",dce_map[index].address,stringdata);
      }
      else
      {}
      /* Print the name of the field */
      printf_P(dce_map[index].text);
      printf_P("\r\n");
    }
    KickWdt();
  }
}

/********************************************************************//**
*  SetAllConfigData();
************************************************************************/
void SetAllConfigData(void)
{
  unsigned char index,tag;
  unsigned char chardata;
  unsigned int intdata;
  char keybuf[4];
  char stringdata[25];
  unsigned char clrbuf;

  /* Show us what we are about to modify */
  PrintAllConfigData();
  /* Enter prompt */
  printf_P("Enter tag number to modify. <ESC> to quit\r\n");
  /* Get selection, */
  if (getstring_small(keybuf, sizeof(keybuf)) != ERROR)
  {
    /* Got a string. Process it. */
    tag = (unsigned char)atoi_small(keybuf);
    /* Tell the user what they have just selected. */
    if (TagToIndex(tag,&index) == ERROR)
    {
      printf_P("Fool, Tag=%d does not exist! <RETURN> to try again, <ESC> quit\r\n",tag);
    }
    else
    {
      /* Valid tag, continue */
      printf_P("SELECTED TAG=%d, or ",tag);
      printf_P(dce_map[index].text);
      printf_P(". ENTER DATA: type ");

      /* Get the expected data */
      if (dce_map[index].size > SIZEOFINT)
      {
        printf_P("STRING ");
      }
      else if (dce_map[index].size == SIZEOFINT)
      {
        printf_P("INT    ");
      }
      else
      {
        printf_P("CHAR   ");
      }
      /* CR */
      printf_P("\r\n");
      /* Clear string data buffer */
      for (clrbuf = 0U; clrbuf < sizeof(stringdata); clrbuf++)
      {
        stringdata[clrbuf] = '\0';
      }
      /* Get data */
      if (getstring_small(stringdata, sizeof(stringdata)) != ERROR)
      {
        /* Process and write data according to its type */
        if (dce_map[index].size > SIZEOFINT)
        {
          /* We are expecting a string. Process it. */
          TagToDceWriteString(tag,stringdata,24U);
        }
        else if (dce_map[index].size == SIZEOFINT)
        {
          /* We are expecting an int. Process it. */
          intdata = atoi_small(stringdata);
          if (TagToDceWriteInt(tag,intdata) == FAIL)
          {
            printf_P("Range check error\r\n");
          }
        }
        else
        {
          /* We are expecting an int. Process it. */
          chardata = (unsigned char)atoi_small(stringdata);
          if (TagToDceWriteChar(tag,chardata) == FAIL)
          {
            printf_P("Range check error\r\n");
          }
        }
        /* Havig modified the EEPROM, we need to recalculate the Chksum */
        UpdateChecksum();
        /* restart modbus handler */
        RestartModbusHandler();
        printf_P("\r\nPress <RETURN> to continue. <ESC> to quit\r\n");
      }
      else
      {
        /*
         * Pressed escape on data entry.
         * Must press escape again to return to menu
         */
        printf_P(dcestr41);
      }
    }
  }
  else
  {
    /*
     * Pressed escape on tag entry.
     * Must press escape again to return to menu
     */
    printf_P(dcestr41);
  }
}

/*
 * MUX 2 - ctl card
 *                         PC7-PC5  PC4-PC2  PC1 & PC0
 */
ADC_DATA_STRUCT MX_Task_Table[] = {
  {ADC_CH6_MULX_2_3, PTEMP_SENSCFG, 0U}, /* 010    000      00, 0}, */
  {ADC_CH6_MULX_2_3, BULB_PWR_MON, 0U}, /* 010    100      00, 0}, */
  {ADC_CH6_MULX_2_3, MX_ASIC_D00, 0U}, /* 010    010      00, 0}, */
  {ADC_CH6_MULX_2_3, MX_ASIC_D01, 0U}, /* 010    110      00, 0}, */
  {ADC_CH6_MULX_2_3, MX_ASIC_D02, 0U}, /* 010    001      00, 0}, */
  {ADC_CH6_MULX_2_3, MX_ASIC_D03, 0U}, /* 010    101      00, 0}, */
  {ADC_CH6_MULX_2_3, MX_ASIC_D04, 0U}, /* 010    011      00, 0}, */
  {ADC_CH6_MULX_2_3, MX_ASIC_D05, 0U}, /* 010    111      00, 0}, */
  {END_TABLE_MARKER, END_TABLE_MARKER, END_TABLE_MARKER} /* Essential end-of-table markers */
};

/*
 * DUPE ENTRIES:-
 * TBD / D.R. H/W investigation - resolved?? I believe this was an H/W, (mux wiring) issue.
 */
ADC_DATA_STRUCT ADC_Task_Table [] = {
  {ADC_CH6_MULX_2_3, _24V_MON, 0U}, /* 001    000      00},//Y0 */
  {ADC_CH6_MULX_2_3, _24V_MON, 0U}, /* 001    000      00},//Y0 */

  {ADC_CH6_MULX_2_3, _4V4_MON, 0U}, /* 001    110      00},//Y3 */
  {ADC_CH6_MULX_2_3, _4V4_MON, 0U}, /* 001    110      00},//Y3 */

  {END_TABLE_MARKER, END_TABLE_MARKER, END_TABLE_MARKER} /* Essential end-of-table markers */
};

/* if (Analogue_Threshold_0 >= V > Analogue_Threshold_1) then B.0 = 0, B.1 = 0 */
ADC_DATA_STRUCT DIL_SW_Task_Table[] = {
  {ADC_CH7_MULX_0_1, DIL_SW_B11_B21, 0U}, /* Start at Mux 0, sw0 */
  {ADC_CH7_MULX_0_1, DIL_SW_B12_B22, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B13_B23, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B14_B24, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B15_B25, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B16_B26, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B17_B27, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B18_B28, 0U}, /* End at Mux 0, sw 7 */

/* only PC3 & PC4 get set; */
  {ADC_CH7_MULX_0_1, COMPATIBILTY_BIT_01, 0U}, /* Start at Mux 1, sw0 */
  {ADC_CH7_MULX_0_1, COMPATIBILTY_BIT_23, 0U},
  {ADC_CH7_MULX_0_1, COMPATIBILTY_BIT_45, 0U},
  {ADC_CH7_MULX_0_1, COMPATIBILTY_BIT_67, 0U}, /* End at Mux 1, sw 3 */

  {END_TABLE_MARKER, END_TABLE_MARKER, END_TABLE_MARKER} /* Essential end-of-table markers */
};


ADC_DATA_STRUCT Range_DIL_SW_Task_Table[] = {
  {ADC_CH7_MULX_0_1, DIL_SW_B14_B24, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B15_B25, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B16_B26, 0U},
  {END_TABLE_MARKER, END_TABLE_MARKER, END_TABLE_MARKER} /* Essential end-of-table markers */
};


ADC_DATA_STRUCT Alarm_DIL_SW_Task_Table[] = {
  {ADC_CH7_MULX_0_1, DIL_SW_B12_B22, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B13_B23, 0U},
  {END_TABLE_MARKER, END_TABLE_MARKER, END_TABLE_MARKER} /* Essential end-of-table markers */
};


ADC_DATA_STRUCT Interface_DIL_SW_Task_Table[] = {
  {ADC_CH7_MULX_0_1, DIL_SW_B13_B23, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B14_B24, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B15_B25, 0U},
  {ADC_CH7_MULX_0_1, DIL_SW_B16_B26, 0U},
  {END_TABLE_MARKER, END_TABLE_MARKER, END_TABLE_MARKER} /* Essential end-of-table markers */
};


RANGE_INDEX_STRUCT Range_DIL_to_index_table[] = {
  { RANGE_25M, '2','5'},
  { RANGE_6M, '6', '.'}, /* sw4 = 1. */
/* 1/4 not readily available from the char map. */
  { RANGE_12M, '1', '2'}, /* sw 5 = 1, */
  { RANGE_50M, '5', '0'}, /* sw 4 = sw5 =1 */
};


ALARM_INDEX_STRUCT Alarm_DIL_to_index_table[] = {
  { 3U, 5U}, /* sw2 = sw3 = 0, alarm = 3 of 5 */
  { 12U, 14U}, /* sw2 = 1, sw3 = 0, alarm = 12 of 14 */
  { 6U, 8U}, /* sw2 = 0, sw3 = 1, alarm = 6 of 8 */
  { 3U, 5U}, /* sw2 = sw3 = 0, alarm = 3 of 5 */
};


void MenuDILSwitchandCompatibilityBitRead(void)
{
  unsigned int sw_bt;

  DILSwitchandCompatibilityBitRead(&sw_bt);
  printf_P(" Hi There, DIP switcc value = %X",sw_bt);
  RangeDILSwitchBitRead(sw_bt);
}


unsigned char DILSwitchandCompatibilityBitRead(unsigned int *sw_bits)
{
/*
 * These are the 2 DIL switches connected to muxs 0 & 1, connected to Y0- Y7, Y0-Y3 & PC2- PC4 respectively.
 * See Dave R's document 'FV400 Test Menu and DRivers spec', section DIL-SWITCH AND COMPATIBILTY-BIT READ for more.
 */
  INT16u Switch_1_bits = 0U; /* Compat bits are read into high byte */
  INT16u Switch_2_bits = 0U; /*             " */
  INT16u Switch_1_Reverse_bits = 0U; /*     " */
  INT16u Switch_2_Reverse_bits = 0U; /*     " */

  BYTEu SWxNorm_bit_count = 0U;
  BYTEu Compat_bit_count = 0U;
  INT16u sw1_norm, sw2_norm, sw_compat;

/* DIL_Switches MUX0_1_DIL_Switches; */
  InitPortCForDILRead();
  ReadADCsFromTaskTable(DIL_SW_Task_Table);
  unsigned char table_index = 0U;
/*   while ((DIL_SW_Task_Table[table_index][0]!= '\0')&&(hardware_fault == FALSE)){ */
  while (DIL_SW_Task_Table[table_index].adc_channel_mux != END_TABLE_MARKER)
  {
    if (DIL_SW_Task_Table[table_index].adc_result < Analogue_Threshold_1)
    {
/*         from DIL_SW_Task_Table[table_index][0] = 2;, etc... */
      Switch_1_bits &= ~(INT16u)(0x01U << table_index);   /* clear both bits (i.e sw1 & 2 = off) */
      Switch_2_bits &= ~(INT16u)(0x01U << table_index);
    }
    else if (DIL_SW_Task_Table[table_index].adc_result < Analogue_Threshold_2)   /* 104 */
    {
      Switch_1_bits &= ~(INT16u)(0x01U << table_index);   /* sw1 = off */
      Switch_2_bits |= (INT16u)(0x01U << table_index);
      if (table_index <= 7U)  /* Are we reading the first 8 bits? */
      {
        ++SWxNorm_bit_count;
      }                        /* if so increment that bit count - Switch x, Normal bit count */
      else
      {
        ++Compat_bit_count;
      }                       /* Otherwise inc. the count for the compat bits. */
    }
    else if (DIL_SW_Task_Table[table_index].adc_result < Analogue_Threshold_3)  /* 140 */
    {
      Switch_1_bits |= (INT16u)(0x01U << table_index);
      Switch_2_bits &= ~(INT16u)(0x01U << table_index);   /* sw2 = on */
      if (table_index <= 7U)  /* Are we reading the first 8 bits? */
      {
        ++SWxNorm_bit_count;
      }                        /* if so increment that bit count */
      else
      {
        ++Compat_bit_count;
      }                       /* Otherwise inc. the count for the compat bits. */
    }
    else if (DIL_SW_Task_Table[table_index].adc_result < Analogue_Threshold_4)   /* <203 */
    {
      Switch_1_bits |= (INT16u)(0x01U << table_index);   /* sw1 = on */
      Switch_2_bits |= (INT16u)(0x01U << table_index);   /* sw2 = on */
      if (table_index <= 7U)  /* Are we reading the first 8 bits? */
      {
        SWxNorm_bit_count += 2U;
      }                          /* if so increment that bit count */
      else
      {
        Compat_bit_count += 2U;
      }                         /* Otherwise inc. the count for the compat bits. */
    }
    else    /* V > T4n */
    {
      Switch_1_bits = 0U;   /* unreadable/ reliable bits when error */
      Switch_2_bits = 0U;   /* unreadable/ reliable bits when error */
      diphwfault |= DIP_FAULT_ONE;//true;
      printf_P("\r H/W error 1: unreadable");
    }
    table_index++;
  }

  InitPortCForReverseDILRead();
  ReadADCsFromTaskTable(DIL_SW_Task_Table);
  table_index = 0U;
/*      while ((DIL_SW_Task_Table[table_index][0]!= '\0')&&(hardware_fault == FALSE)){ */
  while (DIL_SW_Task_Table[table_index].adc_channel_mux != END_TABLE_MARKER)
  {
    if (DIL_SW_Task_Table[table_index].adc_result > Analogue_Threshold_Reverse_4)      /* 215 */
    {
      Switch_1_Reverse_bits &= ~(INT16u)(0x01U << table_index);      /* sw1 = off */
      Switch_2_Reverse_bits &= ~(INT16u)(0x01U << table_index);      /* sw2 = off */
    }
    else if (DIL_SW_Task_Table[table_index].adc_result > Analogue_Threshold_Reverse_3)      /* 151 */
    {
      Switch_1_Reverse_bits &= ~(INT16u)(0x01U << table_index);      /* sw1 = off */
      Switch_2_Reverse_bits |= (INT16u)(0x01U << table_index);
    }
    else if (DIL_SW_Task_Table[table_index].adc_result > Analogue_Threshold_Reverse_2)      /* 116 */
    {
      Switch_1_Reverse_bits |= (INT16u)(0x01U << table_index);
      Switch_2_Reverse_bits &= ~(INT16u)(0x01U << table_index);      /* sw2 = off */
    }
    else if (DIL_SW_Task_Table[table_index].adc_result > Analogue_Threshold_Reverse_1)      /* 52 */
    {
      Switch_1_Reverse_bits |= (INT16u)(0x01U << table_index);      /* sw1 = on */
      Switch_2_Reverse_bits |= (INT16u)(0x01U << table_index);      /* sw2 = on */
    }
    else       /* V > T5n */
    {
      Switch_1_Reverse_bits = 0U;     /* sw1 = off - always off when error */
      Switch_2_Reverse_bits = 0U;     /* sw2 = off - always off when error */
      diphwfault |= DIP_FAULT_TWO;//true;
      printf_P("H/W error #2, table index = %d\r\n",table_index);
    }
    table_index++;
  }


  if ((Switch_1_bits & 0xffU) != (Switch_1_Reverse_bits & 0xffU))
  {
    diphwfault |= DIP_FAULT_THREE;//true;
    //printf_P("H/W error #3, Switch_1_bits=0x%04X   Switch_1_Reverse_bits=0x%04X\r\n", Switch_1_bits & 0xffU, Switch_1_Reverse_bits & 0xffU);
  }

  if ((Switch_2_bits & 0xffU) != (Switch_2_Reverse_bits & 0xffU))
  {
    diphwfault |= DIP_FAULT_FOUR;//true;
    //printf_P("H/W error #4, Switch_2_bits=0x%04X   Switch_2_Reverse_bits=0x%04X\r\n", Switch_2_bits & 0xffU, Switch_2_Reverse_bits & 0xffU);
  }

  /*if ((Switch_1_bits & 0x0f00U) != (Switch_1_Reverse_bits & 0x0f00U))
  {
    diphwfault |= 0x10;//true;
  }

  if ((Switch_2_bits & 0x0f00U) != (Switch_2_Reverse_bits & 0x0f00U))
  {
    diphwfault |= 0x20;//true;
  }

  if (SWxNorm_bit_count % 2U == 0U)
  {
    //diphwfault |= 0x40;//true;
  }

  if (Compat_bit_count % 2U == 0U)
  {
    //diphwfault |= 0x80;//true;
  }*/

  SetPortCToIdleState();

  if (!g_enablechecksum)
  {
    printf_P("Switch_1_bits=0x%04X   Switch_2_bits=0x%04X\r\n", Switch_1_bits, Switch_2_bits);
  }
  sw1_norm = Switch_1_bits & 0x00ffU;
  sw2_norm = Switch_2_bits & 0x00ffU;

  sw_compat = (Switch_1_bits >> 8U);    /* make msb lsb for readout */
  sw_compat |= ((Switch_2_bits >> 4U) & 0x00f0U);   /* and bring in the high nibble from the compatability switch */


  *sw_bits = (sw2_norm << 8U);
  *sw_bits |= (sw1_norm);

  return(diphwfault);
}

unsigned char DipSwitchFaultState(void)
{
  return(diphwfault);
}


void InitPortCForDILRead(void)
{
  DDRC = 0xFFU;
  PORTC |= (0x01U << PC0);         /* only required for DIP switch readings... */
  PORTC &= ~(0x01U << PC1);
}

void InitPortCForReverseDILRead(void)
{
  PORTC &= ~(0x01U << PC0);               /* only required for DIP switch readings... */
  PORTC |= (0x01U << PC1);
}


void SetPortCToIdleState(void)
{
  DDRC &= ~(0x01U << PC1);           /* PC1 to input */
  PORTC &= ~(0x01U << PC1);           /* and disable PC1 */
  PORTC &= ~(0x01U << PC0);           /* PC0 just disabled */
}

/********************************************************************//**
 * ReadADCsFromTaskTable(ADC Struct)
 *************************************************************************/
void ReadADCsFromTaskTable(ADC_DATA_STRUCT table_ptr[])
{
  BYTEu table_index=0U;
  BYTEu test=0U;
  
  while((table_ptr[table_index].adc_channel_mux!= END_TABLE_MARKER)&&(table_index<MAX_TASK_INDEX))
  {  
    test=GetAdcMuxValue(table_ptr[table_index].port_c_bits, table_ptr[table_index].adc_channel_mux);
    table_ptr[table_index].adc_result=test;
    table_index++;
  }
}

/********************************************************************//**
 StartUpPowerManagement()
*************************************************************************/
void StartUpPowerManagement(void)
{
 InitAllPorts();
 ZeroAllPorts();
 // init timer 2 for 10 ms Interrupt
 SetupTimerTwo10msMode();
 SleepModeActive(INITIALIZATION);
 __sleep();
 // Initalise sections that have no dependancies
 InitTestPoint();
 SpiInitPort();
 //ADC AUg 2015
 // Before we can read some of the analogue inputs, we must
 // check if a JTAG debugger is fitted. If not, enable the
 // shared JTAG / analogue input pins to analogue inputs
 InitJtagInterface();
 __sleep();
 GetAdcRawValue(0U);
 LoadDipSwitchValues();
 __sleep();
 // Init UART0
  InitUART(UART0,BAUD19200,NOPARITY); // set the baudrate RS485
  //InitUART(UART1, BAUD1200,NOPARITY);// set the baudrate HART
  SetUartCommMode(UART0,UART_RS485_OFF);
 __sleep();
   // At this point the above peripherals are initalised.
  // Perform an application CRC test without accessing the configuration
  // or the FDE. Putting the CRC test here also adds a delay
  // before the field data EERPOM power up log is written.
  // This reduces the possibility of power on glitches causing
  // invalid data being written to the FDE.
  OCR2A=1U; 
  InitTestByte();
  ApplicationCodeCrcTest(CRC_BOOT_TEST_ONLY, false,true);
  __sleep();
}

/********************************************************************//**
 *      Monitor 4-20mA ()
 * Enable the 4-20mA interface if not already enabled.
 * In this situation, leave the 4-20mA interface enabled
 * until after exit from the test menus.
 *************************************************************************/
BOOL Monitor_4_20mA_Level(void)
{
  printf_P("Ask Anshuman for help..\r\n");
  return(0);
}
/********************************************************************//**
 * EnableTheHARTModem()
 *
 * Hart modem needs a clock of 460.8KHz with 1% tolerance. PG1 is INRTS
 * it must be high in order to rcv data and low for transmit. All the
 * HART communication is on USART 1.
 *************************************************************************/
void EnableTheHARTModem(void)
{
/* Configure PD2 as an input then write '0' to this pin to disable the pull-up resistor. */
  DDRD &= ~(0x01U << DDD2);      /* Config PD2 for input. */
  PORTD &= ~(0x01U << PD2);     /* disable the pull-up resistor. */
/*  Configure the following ports as output and initialise as stated: */
  DDRD |= (0x01U << DDD3);     /* Config PD3 for output. */
  PORTD &= ~(0x01U << PD3);     /* PD3=0, */

  DDRG |= (0x01U << DDG1);        /* Config PG1 for output. */
  PORTG |= (0x01U << PG1);      /* ,INRTS, PG1=0 (needs to be low for transmission) */
  SetHartModem(true);
  Setup460_8kHzTimer3();
}

/********************************************************************//**
 * Setup460_8kHzTimer3(void)
 *
 * This function generates 460.8KHz clock signal
 * on PE3 (OC3A). It means is this clock thing is active
 * then timer 3 is going to be quite busy, and it will never
 * attain value greater than OCR3A, hence useless for greater
 * delays.
 *
 * One more thing there is no ISR #pragma thing for this
 * toggling whenever timer value is equal to OCR3A is there it
 * will toggle the pin and clear the timer.
 *
 * July 28, 2010
 *************************************************************************/
void Setup460_8kHzTimer3(void)
{
  __disable_interrupt();
/* OC3A over rides any other state for timers */
  TCCR3A |= (0x01U << COM3A0);
  TCCR3A &= ~(0x01U << COM3A1);

/* For 460.8kHz the timer prescaler needs to be set to ÷1, ie have TCCR3B bits CS30=1, CS31=0, CS32=0. */
  TCCR3B |= (0x01U << CS30);       /* have TCCR3B bits CS30=1, CS31=0, CS32=0. */
  TCCR3B &= ~(0x01U << CS31);
  TCCR3B &= ~(0x01U << CS32);
/*
 * Then configure processor port PB5PE4 to toggle on each counter reset by having TCCR3B bits 3WGM33,
 * and WGM32 set to 1 and TCCR3A bits WGM31 and WGM30 set to 1 (see [2] Table 17-2 p148).
 */
  TCCR3B |= (0x01U << WGM32);  /* CTC with OCR3A */
  /*
   * DO not use interrupt it crashes the speed
   * TIMSK3=0x00; //Interrupt disable.
   */

  OCR3A = COMPAIR_TIMER_VAL_HART;  /* write set 0x0003 to OCR3A. This sets the counter to count from 0x000x0000 to OCR3A */
  TCNT3 = 0U;               /* then resets to 0x0000. */
/* Port PE3 toggles at this rate, creating the required 460.8kHz. */
  __enable_interrupt();

/* set associated pin to o/p, no pullup, inetial state low */
  DDRE |= (0x01U << DDE3);
  PORTE &= ~(0x01U << DDE3);    /* disable pulllups */
  PORTE &= ~(0x01U << PE3);
}

/********************************************************************//**
*  DisableTheHARTModem();
*
*  Turn off the supply to modem, stop timer 3 clock.
************************************************************************/
void DisableTheHARTModem(void)
{
  SetHartModem(false);
  DDRG |= (0x01U << DDG1);        /* Config PG1 for output. */
  PORTG &= ~(0x01U << PG1);     /* rts low */
  /* stop timer 3 clock */
  __disable_interrupt();
  TCCR3B = 0U;
  TCCR3A = 0U;
  __enable_interrupt();
}

/* BYTEu PB4_saved; */
BYTEu DDRA_backup;
BYTEu DDRB_backup;
BYTEu DDRC_backup;
BYTEu DDRD_backup;
BYTEu DDRE_backup;
BYTEu DDRF_backup;

BYTEu PORTA_backup;
BYTEu PORTB_backup;
BYTEu PORTC_backup;
BYTEu PORTD_backup;
BYTEu PORTE_backup;
BYTEu PORTF_backup;

void SavePortValues(void)
{
  DDRA_backup = DDRA;
  DDRB_backup = DDRB;
  DDRC_backup = DDRC;
  DDRD_backup = DDRD;
  DDRE_backup = DDRE;
  DDRF_backup = DDRF;

  PORTA_backup = PORTA;
  PORTB_backup = PORTB;
  PORTC_backup = PORTC;
  PORTD_backup = PORTD;
  PORTE_backup = PORTE;
  PORTF_backup = PORTF;
}


void LoadPortValues(void)
{
  DDRA = DDRA_backup;
  DDRB = DDRB_backup;
  DDRC = DDRC_backup;
  DDRD = DDRD_backup;
  DDRE = DDRE_backup;
  DDRF = DDRF_backup;

  PORTA = PORTA_backup;
  PORTB = PORTB_backup;

  PORTC = PORTC_backup;
  PORTD = PORTD_backup;
  PORTE = PORTE_backup;
  PORTF = PORTF_backup;
/* LoadGlobalOrLocalExpander(&global_expander_register); */
}

/*
 * 10.11.2	Test Functions
 * 10.11.2.1	Top Level Test Menu
 * The following test functions shall be supported:
 *
 * Option	MX Interface Test Functions
 * 1	 Set output Ai0 *
 * 2	Set outputs Ai1 and Ai2
 * 3	Read Inputs
 */

/*
 *  * done by 4-20ma test/ write
 * TBD - test H/W :-
 */
void SetMXOutputAi1andAi2(unsigned char level)
{
/*
 * 10.11.2.3	Option 2: Set Outputs Ai1 and Ai2
 * Prompt the user to enter a value between 0 and 3 inclusive (equivalent to 00, 01, 10 or 11), or <esc> to cancel.
 * Output the value entered to PB6 and PE7 then re-prompt the user. If a number outside 0 to 3 is entered,
 * or a non-numeric value then just re-prompt the user. After <esc> or two-minute timeout go to the top level test menu
 * in section 10.11.2.1. On exiting from this test option leave the alarm output in its current state.
 * However, on exit from test mode return this output to its former state.
 */

  if (level <= 3U)
  {
    DDRE |= (0x01U << DDE6);     /* Config PE6 & 7 for output */
    DDRE |= (0x01U << DDE7);

    if ((level & 0x01U) == 1U)
    {
      PORTE |= (0x01U << PE6);
    }
    else
    {
      PORTE &= ~(0x01U << PE6);
    }

    if (level & 0x02U)
    {
      PORTE |= (0x01U << PE7);
    }
    else
    {
      PORTE &= ~(0x01U << PE7);
    }
  }
}

void PrintCalibrationData(void)
{
  BYTEu dflt_data = TagToConfigReadChar(DFLT_PRIMLGA);
  printf_P("Flame0 Low = %d\r\n", dflt_data);
  dflt_data = TagToConfigReadChar(DFLT_PRIMLGB);
  printf_P("Flame1 Low = %d\r\n", dflt_data);
  dflt_data = TagToConfigReadChar(DFLT_SECLG);
  printf_P("Guard Low = %d\r\n", dflt_data);

  dflt_data = TagToConfigReadChar(DFLT_PRIMHGA);
  printf_P("Flame0 High = %d\r\n", dflt_data);
  dflt_data = TagToConfigReadChar(DFLT_PRIMHGB);
  printf_P("Flame1 High = %d\r\n", dflt_data);
  dflt_data = TagToConfigReadChar(DFLT_SECHG);
  printf_P("Guard High = %d\r\n", dflt_data);
}

void WriteCalibrationData(unsigned char tag,unsigned char data)
{
  TagToDceWriteChar(tag,data);
}

/********************************************************************//**
 * RangeDILSwitchBitRead()
 * I am keeping this style of reading range as in rest of the file
 * same apporach is followed. Switch 4 and 5 are for the range. SW6 is
 * discounted from this function as this would be used for configuring
 * Aux power supply.
 * AR, May 10,2011
 *************************************************************************/
unsigned char RangeDILSwitchBitRead(unsigned int Switch_1_bits)
{
  unsigned char range_index;
  /* i am keeping this instruction to avoid possible program flow conflict */
  Switch_1_bits = ((Switch_1_bits >> 3U) & (SWITCH_1 | SWITCH_2));
  /* Move switches 4-5 to positions 1-2, mask them & use them as an index into the Range_DIL_to_index_table */
  range_index = Range_DIL_to_index_table[Switch_1_bits].associated_c2indx;
  /* printf_P("Range set for %c%cm\r\n", Range_DIL_to_index_table[Switch_1_bits].range_distance1, Range_DIL_to_index_table[Switch_1_bits].range_distance2); */

  return range_index;
}


unsigned char DCERangeConfigRead(void)
{
  unsigned char range_index = TagToConfigReadChar(DETECT_RANGE);
  return range_index;
}

/********************************************************************//**
 * WriteSerialToSCE(unsigned char*);
 * This function will write the 19 bytes sensor board serial number
 * to the SCE, the number is supplied by debug menu. Initial 6 digits
 * of number were fixed so they are not required to be entered by
 * user.
 *************************************************************************/
void WriteSerialToSCE(unsigned char*serialstring)
{
  unsigned char serialarr[19];
  unsigned char serialcount = 6U;
  /* add inetial 6 digits */
  serialarr[0] = '6';
  serialarr[1] = '8';
  serialarr[2] = '5';
  serialarr[3] = '0';
  serialarr[4] = '7';
  serialarr[5] = '2';
  /*
   * set serial arr element count to 6
   * extract digits rcvd from terminal, DL number till end
   */
  for (serialcount = 6U; serialcount < SIZEOFSENSORBOARDSERIAL; serialcount++)
  {
    serialarr[serialcount] = *serialstring;
    serialstring++;
  }
  /* all set pass address to memory write function. */
  WriteEPROMOfSCE(SERIALNUMBER,&serialarr[0]);
} /* void */


void AlarmDILSwitchBitRead(unsigned int Switch_1_bits,unsigned char *spike,unsigned char *history)
{
  Switch_1_bits = ((Switch_1_bits >> 1) & (SWITCH_1 | SWITCH_2));
/* Move switch bits 2 & 3 to positions 1&2, , mask them, and generate an index ; */
  *spike = Alarm_DIL_to_index_table[Switch_1_bits].spike_count;
  *history = Alarm_DIL_to_index_table[Switch_1_bits].history_size;
}


void DCEAlarmConfigRead(unsigned char *spike,unsigned char *history)
{
  *spike = (TagToConfigReadChar(ALARMDELAY) & 0x0FU);
/* Alarm data is stored in the lower nibble only. */
  *history = *spike + 2U;
/* history is always spike + 2; 3of5, 6of8 or 12of14 */
}

/********************************************************************//**
 * InterfaceTypeSet()
 *
 * This function sets the type of interface detector is in and stores
 * the state to a global variable "interfacemode". This function stores the 16
 * bit dip switch values to a variable called "dipswitchvalue", this variable
 * is further used in algo engine file.
 *************************************************************************/
void InterfaceTypeSet(void)
{
  //unsigned char inerfacetempval=0U;
  static unsigned char loopflagdipsw = false,interfacelocalflag = false;

  if (!loopflagdipsw)
  {
    loopflagdipsw = true;
    /* for all interfaces except MX, pin for MX ASIC power (PB5) should remain High */
    MxAiscSwitch(false);
    /* IR WTEST Rcv ON */
    IrRcvCktSwitch(true);
    /* turn off EOL & fault relay */
    ConvEOLTest(false);
    FaultRelay(false);
   // inerfacetempval = (unsigned char)((dipswitchvalue >> 12U) & INTERFACEBITSONLY);
    /* set interface type */
    if (dipswitchvalue & MASTER_SW)
    {
      interfacemode = (unsigned char)((dipswitchvalue >> 12U) & INTERFACEBITSONLY);
      enhancedbandbitval = (unsigned char)((dipswitchvalue >> 12U) & ENHANCEDBANDBIT);
      //hartprotocolreq = (unsigned char)((switch_val >> SHIFT_BY_BYTE) & HARTPROTICOLSW);
      auxsupplysw = true;//(unsigned char)(((switch_val & AUXSUPPLYSWITCH) == AUXSUPPLYSWITCH));
      /* resolve hart switch confilict */
      if (hartprotocolreq)
      {
        if (interfacemode == ONLY_4_20_DIS_H)
        {
          interfacemode = ONLY_4_20_DIS;
        }
        else if (interfacemode == ONLY_4_20_VAR_H)
        {
          interfacemode = ONLY_4_20_VAR;
        }
        else if (interfacemode == DEFVAR_4_20MA_MODE_H)
        {
          interfacemode = DEFVAR_4_20MA_MODE;
        }
        else if (interfacemode == DEFAULT_4_20MA_MODE_H)
        {
          interfacemode = DEFAULT_4_20MA_MODE;
        }
        else
        {}
      }
      /* resolve conv mode various sw issue */
      if ((interfacemode & CHECK_CONTENTIONAL_IF_BITS) == CONVENT_MODE)
      {
        interfacemode = CONVENT_MODE;
      }
    }
    else
    {
      /*
       * read from configuration memory then
       * set interfacemode and enhanced band bit values here.
       */
      enhancedbandbitval = TagToConfigReadChar(_4_20MAMODE2);
      hartprotocolreq = TagToConfigReadChar(HARTSELECT);
      interfacemode = TagToConfigReadChar(INTERFACES);
      auxsupplysw = TagToConfigReadChar(AUXSUPPLY);
      if (interfacemode == DCE_DEFAULT_4_20MA_MODE)
      {
        /* identify var or band */
        if (TagToConfigReadChar(_4_20MAMODE1))
        {
          interfacemode = DEFVAR_4_20MA_MODE;
        }
        else
        {
          interfacemode = DEFAULT_4_20MA_MODE;
        }
      }
      else if (interfacemode == DCE_CONVENTIONAL)
      {
        interfacemode = CONVENT_MODE;
      }
      else if (interfacemode == DCE_VDS_MULTI)
      {
         interfacemode=DEFAULT_4_20MA_MODE; 
      }
      else if (interfacemode == DCE_4_20)
      {
        /* check weather variable or discrete */
        if (TagToConfigReadChar(_4_20MAMODE1))
        {
          interfacemode = ONLY_4_20_VAR;
        }
        else
        {
          interfacemode = ONLY_4_20_DIS;
        }
      }
      else if (interfacemode == DCE_MX)
      {
        /* check weather dce-dipswitch OR Consys */
        if (!TagToConfigReadChar(MXMODE1))
        {
          /* its consys */
          interfacemode = MX_IFACE_MODE;
        }
        else
        {
          /* its DCE in this case but named as DIP switch */
          interfacemode = MX_IFACE_DIPSW_ACTIVE;
        }
      }
      else
      {
        __no_operation();
      }
    }
    
    if((interfacemode != CONVENT_MODE) && (interfacemode != MX_IFACE_MODE) &&
      (interfacemode != MX_IFACE_DIPSW_ACTIVE))
    {
        printf_P("DIP = %X\r",dipswitchvalue); 
    }
    
  }

  if (interfacemode == MX_IFACE_MODE_ETC)
  {
    //interfacemode = MX_IFACE_MODE;
  }

  switch (interfacemode)
  {
  case ONLY_4_20_DIS:
    if (!interfacelocalflag)
    {
      interfacelocalflag = true;
      Output_4_20mA_Level(SET_NORMAL);
      /* FaultRelay(false); */
    }
    break;
  case ONLY_4_20_VAR:
    interfacelocalflag = true;
    Output_4_20mA_Level(SET_NORMAL);
    break;
  case DEFVAR_4_20MA_MODE:
    if (!interfacelocalflag)
    {
      interfacelocalflag = true;
      Output_4_20mA_Level(SET_NORMAL);
      /* FaultRelay(true); */
    }
    break;
  /*
   * case VARIABLE_4_20MA_HART_MODE:
   *  printf_P("\n\r Variable 420 hart");
   *   break;
   */
  case CONVENT_MODE:
    Disable_4_20mA();
    /*
     * FaultRelay(false);
     * turn off IR Rcv
     */
    IrRcvCktSwitch(false);
    if (!interfacelocalflag)
    {
      PORTG &= ~(0x01U << PG2);   /* set PG2 low */
      ConvEOLTest(true);       /* EOL set */
      /* deactivate alarm load for conv mode at start up */
      ConvAlarmControl(false);
      /* printf_P("\n\r Configured as S231 \n\r"); */
      interfacelocalflag = true;
      InitCaptureFaultPulsesOnPowerLine();
    }
    StatusOfFaultPulsesOnPowerLine();
    break;

  /*
   * case DISCRETE_4_20MA_HART_MODE:
   * printf_P("HART_IF i/face called here\n\r");
   * fallthru
   */
  case DEFAULT_4_20MA_MODE:
    if (!interfacelocalflag)
    {
      interfacelocalflag = true;
      Output_4_20mA_Level(SET_NORMAL);
      /* FaultRelay(true); */
    }
    break;

  case MX_IFACE_MODE:
    MxAiscSwitch(true);
    if (!interfacelocalflag)
    {
      interfacelocalflag = true;
      /* new Aug 2012 MX programmer issue */
      if (enhancedbandbitval || (dipswitchvalue & MX_PROG_SWITCH))
      {
        /*
         * MX Address programmer reduce current
         * turn on ASIC and do nothing )
         */
        StuckForMxAddressProgrmmer();
      }
    }
    UpdateMxParameters();
    /* FaultRelay(false); */
    break;

  case MX_IFACE_DIPSW_ACTIVE:
    MxAiscSwitch(true);
    if (!interfacelocalflag)
    {
      interfacelocalflag = true;
      if (enhancedbandbitval || (dipswitchvalue & MX_PROG_SWITCH))
      {
        /*
         * MX Address programmer reduce current
         * turn on ASIC and do nothing )
         */
        StuckForMxAddressProgrmmer();
      }
      /* printf_P("\n\r Configured as S271 with DIP SW ACTIVE \n\r"); */
    }
    UpdateMxParameters();
    /* FaultRelay(false); */
    break;

  default:
    /*
     * invalid DCE or Switch settings
     * set default FV400 mode relay plus 4_20
     */
    if (!interfacelocalflag)
    {
      interfacelocalflag = true;
      /* FaultRelay(true); */
    }
    break;
  }
}
/********************************************************************//**
 * unsigned char PersonalityType();
 * Returns the Interface type.
 *************************************************************************/
unsigned char PersonalityType(void)
{
  return(interfacemode);
}

/********************************************************************//**
 * unsigned int GetDipSwitchVal();
 * Returns the 16 bit value.
 *************************************************************************/
unsigned int GetDipSwitchVal(void)
{
  return(dipswitchvalue);
}

/********************************************************************//**
 * void ChangePersonalityType(unsigned char);
 *************************************************************************/
void ChangePersonalityType(unsigned char reqpersonality)
{
  interfacemode = reqpersonality;
}

/********************************************************************//**
 * unsigned char GetEnhancedBandBit();
 * Returns state of enhanded bit switch.
 *************************************************************************/
unsigned char GetEnhancedBandBit(void)
{
  return(enhancedbandbitval);
}

/********************************************************************//**
 * unsigned char GetAuxSupplySw();
 * Returns state of enhanded bit switch.
 *************************************************************************/
unsigned char GetAuxSupplySw(void)
{
  return(auxsupplysw);
}

/********************************************************************//**
 * unsigned char GetModbusSw();
 * Returns state of enhanded bit switch.
 *************************************************************************/
unsigned char GetModbusSw(void)
{
  if (mbset == MODBUS_SET)
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

/********************************************************************//**
 * unsigned char GetHartProtocolBit();
 * Returns state of hart protocol confuguration true/false.
 * If hart bit is set and any of 4_20 interface is active then
 * return true else false.
 *************************************************************************/
unsigned char GetHartProtocolBit(void)
{
  if ((hartprotocolreq) && ((interfacemode == ONLY_4_20_DIS) ||
                            (interfacemode == ONLY_4_20_VAR) || (interfacemode == DEFVAR_4_20MA_MODE) ||
                            (interfacemode == DEFAULT_4_20MA_MODE)))
  {
    return(true);
  }
  else
  {
    return(false);
  }
}
/********************************************************************//**
 * UpdateMxParameters()
 *
 * updates range and alarm delay from the MX asic if configured to do so.
 *************************************************************************/
void UpdateMxParameters(void)
{
  static unsigned long secondscountreg = 0U;
  static unsigned char selftestflag = 0U,mxval = DEFAULTALARMDELAYANDRANGE,
                       pastmxval = 0U,samplemxval = 0U,soaktestwrite = 0U; /* default val 50m range, 3 of 5, no self test */
  unsigned char temp;
  char testdays[SIZEOFDATE];
  testdays[0] = '0';
  testdays[1] = '0';
  testdays[2] = '0';
  testdays[3] = '0';

  if ((PersonalityType() != MX_IFACE_MODE) && (PersonalityType() != MX_IFACE_DIPSW_ACTIVE))
  {
    return;
  }
  if (!secondscountreg)
  {
    samplemxval = MAX_MX_SAMP_LIM;
    secondscountreg = GetSecondsSincePowerUp();

  }
  else if (secondscountreg + 2U < GetSecondsSincePowerUp())
  {
    secondscountreg = GetSecondsSincePowerUp();
    /* 5sec have passed read MX status */
    mxval = MxDigitalOutputRead();
    /* printf_P("\r Mx Val %d",mxval); */
    if (mxval == 0U)
    {
      mxval = DEFAULTALARMDELAYANDRANGE;
    }                                                 /* default val 50m, delay 3of5.. */

    /* count samples if 5 consitive values are same then update global variables */
    if (pastmxval == mxval)
    {
      samplemxval++;
    }
    else
    {
      samplemxval = 0U;
    }
    /* store prev mx val */
    pastmxval = mxval;
    /* self test request check follow */
    if ((mxval & MXD5) && (!(mxval & MXD6)) && (selftestflag > 1U))
    {
      selftestflag = 0U;
      /*
       * if(selftestflag>2){
       * if so carry out self test here
       */
      //printf_P("\n\r Self Test Request Received");
      /* do window test first */
      OpmTestReq(LED,TagToConfigReadChar(OPMLEDDRIVE));
      temp = MakeOpmTestDecesion(LED,MANUAL_TEST_TRIGGER);
      /*
       * now do alarm test
       * results collected analyze them
       */
      if ((ElectricalAlarmTest(MZX_PANEL_TRIGGER) == true) && (temp == WINDOW_CLEAN))
      {
        Timer3MXPWMEnable();
        OutputMX_4_20mALevel(MX_17mA);
        /* signal alm level as long as D5 is set */
        while (MxDigitalOutputRead() & MXD5)
        {
          /* its safe dead lock as it signals alarm */
          KickWdt();
        }
      }
      /* restore normal, this will signal any fault as well */
      OutputMX_4_20mALevel(MX_4mA);
      /* reset flag */
      selftestflag = 0U;
      /* let electrical thing settle down */
      delayms(MX_TEST_AMP_SETTLE);
      /* } */
    }
    else
    {
      selftestflag = 0U;
    }
    /* soak test checks, check DO6 is set? */
    if ((SoakTestConditionCheck(READ_RESULT)) && (mxval & MXD6) && (soaktestwrite > 1U))
    {
      soaktestwrite = 0U;
      /* test result DO7 */
      if (mxval & MXD7)
      {
        TagToDceWriteChar(ENVTESTRESULT,true);
      }
      else
      {
        TagToDceWriteChar(ENVTESTRESULT,false);
      }
      /*
       * read only D0 to D5
       * tens place 0 to 3 only
       */
      testdays[4] = char_itoa((unsigned char)((mxval & 0x30U) >> 4U));
      /* unit place 0 to 9 */
      testdays[5] = char_itoa((unsigned char)(mxval & 0x0FU));
      /* days only format is YYMMDD */
      TagToDceWriteString(ENVTESTDATE,(char*)&testdays,SIZEOFDATE);
      /* Update DCE checksum, May28,2013 */
      UpdateChecksum();
    }
  }
  else
  {
    /* update MX digital line anyway */
    temp = MxDigitalOutputRead();
    if ((temp & MXD5) && (!(temp & MXD6)))
    {
      /* self test condition is set cofirm stability */
      if (selftestflag < 2U)
      {
        selftestflag++;
      }

    }
    else if (selftestflag)
    {
      selftestflag = 0U;
    }
    else
    {}
    /* collect buffer for soak test result write */
    if (temp & MXD6)
    {
      /* self test condition is set cofirm stability */
      if (soaktestwrite < SOAK_TEST_CONFIRM_CNT)
      {
        soaktestwrite++;
      }

    }
    else if (soaktestwrite)
    {
      soaktestwrite = 0U;
    }
    else
    {}
  }
  /*
   * set alarmdelay, range and monitor self test request
   * only if stable for 3 samples alarm delay follow
   */
  if (samplemxval == MAX_MX_SAMP_LIM)
  {   /* val has changed update following */
    samplemxval = 0U;
    temp = mxval & DEFAULT_DELAY_MX;
    if(PersonalityType() == MX_IFACE_MODE)
    {
       switch (temp)
       {
       case DEFAULT_DELAY_MX:
         /* set 3 of 5 */
         spikemx = 3U;
         historymx = 5U;
         dispinfoarry[DISP_DELAY] = 'S';
         break;
       case _12OUT14_DELAY_MX:
         /* set 12 of 14 */
         spikemx = 12U;
         historymx = 14U;
         dispinfoarry[DISP_DELAY] = 'L';
         break;
       case _6OUT8_DELAY_MX:
         /* set 6 0f 8 */
         spikemx = 6U;
         historymx = 8U;
         dispinfoarry[DISP_DELAY] = 'M';
         break;
       case DEFAULT_DELAY_MXD:
         /* set 3 0f 5 again */
         spikemx = 3U;
         historymx = 5U;
         dispinfoarry[DISP_DELAY] = 'S';
         break;
       default:
         break;
       }
    }
    
    /* range follow */
    temp = mxval & DEFAULT_RANGE_MX;
    if(PersonalityType() == MX_IFACE_MODE)
    {
       switch (temp)
       {
       case DEFAULT_RANGE_MX:
         /* set 50m */
         rangemx = RANGE_50M;
         dispinfoarry[DISP_RANGE] = RangeConverter(rangemx);
         break;
       case RANGE_MX_12:
         /* set 12.5m */
         rangemx = RANGE_12M;
         dispinfoarry[DISP_RANGE] = RangeConverter(rangemx);
         break;
       case RANGE_MX_25:
         /* set 25m normal */
         rangemx = RANGE_25M;
         dispinfoarry[DISP_RANGE] = RangeConverter(rangemx);
         break;
       case RANGE_MX_50:
         /* set 50m again */
         rangemx = RANGE_50M;
         dispinfoarry[DISP_RANGE] = RangeConverter(rangemx);
         break;
       default:
         break;
       }
    }
  }
}

/********************************************************************//**
 * SetToNormalAtPowerUpPlusDisplay()
 *
 * This function sets all the values to normal like 4_20mA loop to 4mA
 * at powerup before the settling of Pyros. This is done for a stable boot
 * up and to avoid any false hardware fault log.
 *
 * Along with that this will also display some vital parameters on the
 * RS485 channel.
 * June 2011
 *************************************************************************/
void SetToNormalAtPowerUpPlusDisplay(void)
{
  char lclserialnum[19 + 1]; /* SIZEOFUNITSERIAL+1 */
  unsigned char tempreg1 = 0U,tempreg2 = 0U;
  lclserialnum[SIZEOFUNITSERIAL] = '\0';
  if ((interfacemode == DEFAULT_4_20MA_MODE) || (interfacemode == DEFVAR_4_20MA_MODE)
      || (interfacemode == ONLY_4_20_DIS) || (interfacemode == ONLY_4_20_VAR))
  {
    Enable_4_20mA();
    Output_4_20mA_Level(SET_ZERO_NON_4_20_MODES);   /* SET_ZERO_NON_4_20_MODES. */
  }
  /* train lamps */
  LampControl(ALL_LAMPS_OFF);
  if((interfacemode != CONVENT_MODE) && (interfacemode != MX_IFACE_MODE) &&
      (interfacemode != MX_IFACE_DIPSW_ACTIVE))
  {
    delayms(EXERCISE_MUX_LINE_DELAY);
  }
  else
  {
    __sleep();
  }
  LampControl(WINDOW_LAMP_ON);
  LampControl(ALARM_LAMP_ON);
  __delay_cycles(50U);
  /* OPM LAMP off */
  LampControl(ALL_LAMPS_OFF);
  /*
   * set for Alarm LAMP not for IR here
   * set PC0 low
   */
  DDRC |= (0x01U << PC0);
  PORTC &= ~(0x01U << PC0);
  
  if((interfacemode != CONVENT_MODE) && (interfacemode != MX_IFACE_MODE) &&
      (interfacemode != MX_IFACE_DIPSW_ACTIVE))
  //if(true)
  {
    /* Now it's time to throw some information on RS485 */
    printf_P(PRODUCT_NAME);
    printf_P("\r");
  
    TagToConfigReadString(MYSERIALNUM,lclserialnum,
                          SIZEOFMYSERIAL);
    lclserialnum[SIZEOFMYSERIAL] = '\0';
    printf_P("FP:%s\r", lclserialnum);
  
    TagToConfigReadString(CONTSERIALNUM,lclserialnum,
                          SIZEOFUNITSERIAL);
    printf_P("CO:%s\r", lclserialnum);
    delayms(PRINT_WAIT_XX);
    /* one wire read sensor serial */
    __disable_interrupt();
    ReadPageOfSCE(SERIALNUMBER,(unsigned char*)lclserialnum,
                  SIZEOFUNITSERIAL);
    SDAPin(INPUT,ONE_WIRE_PIN_LOW);
    __enable_interrupt();
    printf_P("SE:%s\r", lclserialnum);
  
    /* sw ver number */
    printf_P("S/W:");
    printf_P(CONT_VERSION);
    printf_P("\r");
  
    /* configuration either on dip or dce */
    if (dipswitchvalue & MASTER_SW)
    {
      printf_P("D\r");
    }
    else
    {
      printf_P("E\r");
    }
  
    /* calibration values */
    printf_P("Cal: %d,%d, %d,%d, %d,%d\r",TagToConfigReadChar(DFLT_PRIMHGA),TagToConfigReadChar(DFLT_PRIMLGA),
             TagToConfigReadChar(DFLT_PRIMHGB),TagToConfigReadChar(DFLT_PRIMLGB),
             TagToConfigReadChar(DFLT_SECHG),TagToConfigReadChar(DFLT_SECLG));
  
    /* Aux power */
    if ((dipswitchvalue & AUX_POWER_SW_NU) && (dipswitchvalue & MASTER_SW))
    {
      printf_P("AX: Y\r");
    }
    else
    {
      printf_P("AX: N\r");
    }
  
    /* Interface type */
    switch (interfacemode)
    {
    case ONLY_4_20_DIS:
      printf_P("I/F: 4O-D\r");
      break;
    case ONLY_4_20_VAR:
      printf_P("I/F: 4O-V\r");
      break;
    case DEFVAR_4_20MA_MODE:
      printf_P("I/F: 4R-V(H)\r");
      break;
    case CONVENT_MODE:
      printf_P("I/F: CO\r");
      break;
    case DEFAULT_4_20MA_MODE:
      printf_P("I/F: 4R-D(H)\r");
      break;
    case MX_IFACE_MODE:
      printf_P("I/F: MX-C\r");
      break;
    case MX_IFACE_DIPSW_ACTIVE:
      printf_P("I/F: MX-D\r");
      break;
    default:
      printf_P("I/F: Invalid switch setting\r");
      break;
    }
    /* Alarm & Fault latching status */
    if (TagToConfigReadChar(ALARMLATCHING))
    {
      printf_P("L: AL,");
    }
    else
    {
      printf_P("L: AN,");
    }
    /* flt */
    if (TagToConfigReadChar(FAULTLATCHING))
    {
      printf_P("FL\r");
    }
    else
    {
      printf_P("FN\r");
    }
  
    /* Range .. */
    if (dipswitchvalue & MASTER_SW)
    {
      tempreg2 = RangeDILSwitchBitRead(dipswitchvalue);
      printf_P("R: %c\r",RangeConverter(tempreg2));
      /* GLOBAL ARRAY USED BY VIDEO DISPLAY */
      dispinfoarry[DISP_RANGE] = RangeConverter(tempreg2);
    }
    else
    {
      tempreg2 = TagToConfigReadChar(DETECT_RANGE);
      printf_P("R: %c\r",RangeConverter(tempreg2));
      dispinfoarry[DISP_RANGE] = RangeConverter(tempreg2);
    }
  
    /* Alm delay display.. */
    if (dipswitchvalue & MASTER_SW)
    {
      AlarmDILSwitchBitRead(dipswitchvalue, &tempreg1, &tempreg2);
      printf_P("D: %d\r",tempreg1);
    }
    else
    {
      printf_P("D: %d\r",(unsigned int)(TagToConfigReadChar(ALARMDELAY) & 0x0FU));
      tempreg1 = TagToConfigReadChar(ALARMDELAY);
    }
    if (tempreg1 == ALM_DEL_3)
    {
      dispinfoarry[DISP_DELAY] = 'S';
    }
    else if (tempreg1 == ALM_DEL_6)
    {
      dispinfoarry[DISP_DELAY] = 'M';
    }
    else if (tempreg1 == ALM_DEL_12)
    {
      dispinfoarry[DISP_DELAY] = 'L';
    }
    else
    {
      dispinfoarry[DISP_DELAY] = 'X'; /* error */
    }
    /* Window Heater.. */
    if (ConfigWindowHeater(READ_STATE))
    {
      printf_P("W:A\r");
      dispinfoarry[DISP_WINDOWHEAT] = 'W';
    }
    else
    {
      printf_P("W:O\r");
      dispinfoarry[DISP_WINDOWHEAT] = ' ';
    }
  
    /* OPM.. */
    if (TagToConfigReadChar(OPMINHIBIT))
    {
      printf_P("O:M\r");
      dispinfoarry[DISP_OPM] = 'M';
    }
    else
    {
      printf_P("O:A\r");
      dispinfoarry[DISP_OPM] = 'A';
    }
  }
} /* void */

/********************************************************************//**
 * SleepModeActive()
 *
 * Only used for conventional mode in order to save current.
 * Only timer 2 could run in this mode, for this reason
 * timer 2 was selected as the main flame timer.
 *************************************************************************/
void SleepModeActive(unsigned char sleeptype){
  static unsigned char configsleepflag=false;
  //static unsigned char ckdvdflg=false;
  if(sleeptype == MAIN_LOOP)
  {
    if(!configsleepflag)
      {
        configsleepflag=true;
        //HART OFF
        PORTG|=(0x01U<<PG3);
        //digital input disable registers(on ADC channels)
        DIDR0|=0x80U;
        DIDR1|=0x03U;
        DIDR2|=0x40U;
        //sleep mode control register
        //000-idle,001-adc noise,010-pwrdowm,011-pwrsave,110-stby,111-extended stby.
        if(PersonalityType()==CONVENT_MODE){
        //if(true){  
          SMCR&=~(0X01U<<SM2);
          SMCR|=(0X01U<<SM1);
          SMCR|=(0X01U<<SM0);
          //all set now exicute sleep instruction
          SMCR|=(0X01U<<SE);  // enable sleep mode 
        }
        else if(GetMxSmpStatus() == false)
        {
          SMCR = 0x00U;//idle
          //all set now exicute sleep instruction
          SMCR|=(0X01U<<SE);  // enable sleep mode
          
          PRR0 = 0x82U;
          PRR1 = 0x37U;
        }
        else
        {}
      }
    //take care of fault pulses tx and rx.
     if(PersonalityType()==CONVENT_MODE){
       
      if(PulseDetectState()==NOPULSES)
        {
         __sleep();
         ActivateSpiModule();
        }
    }
    
    else if((PersonalityType()== MX_IFACE_MODE) || (PersonalityType() == MX_IFACE_DIPSW_ACTIVE)){
      if (GetMxSmpStatus() == false)
      {
         __sleep();
         //ActivateSpiModule();
      }
      else
      {
        WATCHDOG_MACRO
      }
    }
    else
    {}
  }
  else
  {
    /*SMCR&=~(0X01U<<SM2);
    SMCR|=(0X01U<<SM1);
    SMCR|=(0X01U<<SM0);*/
    //ADC NRM
   /* SMCR&=~(0X01U<<SM2);
    SMCR&=~(0X01U<<SM1);
    SMCR|=(0X01U<<SM0);*/
    SMCR = 0x00U;//idle
    //all set now exicute sleep instruction
    SMCR|=(0X01U<<SE);       
  }
} //void

/********************************************************************//**
 * ModbusOrSmpHandler()
 *
 * This functin decides weather detector is configured for MODBUS or not.
 * If MODBUS is active the baud rate would be 19.2k and to enter into SMP
 * there is a window of 30 sec at power up.
 *************************************************************************/

void ModbusOrSmpHandler(void)
{
  static unsigned long secnt = 0UL;
  unsigned char data;
  /*
   * static unsigned char mbset=NONE_SET;
   * Remain here during SERVICE MODE
   */
  EnterServiceModeOnRequest();

  if (mbset == MODBUS_SET)
  {
    /* update from dedicated timer 3 */
    UpdateTimerModbus();
    ModbusHandler();
    /* Check to see if there are characters to process on the Ir Link */
    while (IrDataInReceiveBuffer(PCM) == true)
    {
      /* Get and process data */
      data = ReceivePcmByte();
      ServiceSlaveReveiveState(PCM,data);
      KickWdt();
    }
    return;
  }
  else if (mbset == SMP_SET)
  {
    /*
     * Listen out for service commands in IR or UART while not in
     * service mode
     */
    CheckForAndProcessServicecommand();
    return;
  }
  else
  {}

  /* following only at start up... */
  if (!secnt)
  {
    secnt = GetSecondsSincePowerUp();
  }
  if ((TagToConfigReadChar(FIELDBUSSELECT)) && ((GetSecondsSincePowerUp() - secnt) > WAIT_AFTER_POWERUP)) /* 30 */
  {
    InitUART(UART0,BAUD19200,EVENPARITY);
    mbset = MODBUS_SET;
    if (interfacemode != CONVENT_MODE)
    {
      /* Init modbus timer.. */
      Timer3ForModbus();
    }
    return;
  }
  else if (!TagToConfigReadChar(FIELDBUSSELECT))
  {
    mbset = SMP_SET;
  }
  else
  {
    CheckForAndProcessServicecommand();
  }
}

/********************************************************************//**
 * ConfigWindowHeater();
 *
 * June 2011, set the heater to the requested state either from dip
 * switches or DCE  at power up.
 *************************************************************************/
unsigned char ConfigWindowHeater(unsigned char oprreq)
{
  static unsigned char winheatstate = false;
  unsigned char tempoff,tempon;

  if (oprreq == SET_STATE)
  {
    /* turn off for safety here but train the isolator */
    SetWindowHeater(WINDOW_OFF);
    delayms(WAIT_1_MS);
    SetWindowHeater(WINDOW_ON);
    delayms(WAIT_1_MS);
    SetWindowHeater(WINDOW_OFF);

    if (dipswitchvalue & MASTER_SW)
    {
      if ((dipswitchvalue & AUXSUPPLYSWITCH) && (dipswitchvalue & WINHEATERSW))
      {
        /* set heater on */
        winheatstate = true;
      }
      else if (dipswitchvalue & WINHEATERSW)
      {
        /* heater off */
        winheatstate = false;
        UpdateHardwareFaultSentinel(CNT_CONFIGINVALID,true,3U);
      }
      else
      {}
    }
    else
    {
      /* set from DCE */
      if ((TagToConfigReadChar(WINDOWHEATER)) && (TagToConfigReadChar(AUXSUPPLY)))
      {
        /* set on */
        winheatstate = true;
      }
      else if (TagToConfigReadChar(WINDOWHEATER))
      {
        /* set off */
        winheatstate = false;
        UpdateHardwareFaultSentinel(CNT_CONFIGINVALID,true,4U);
      }
      else
      {}
    }
    /* carry out test only if heater is configured */
    if (winheatstate)
    {
      /* set adc channel for win heater read */
      SetAdcMuxChannel(VIDEO_SYNC_AND_WHEAT_MON);
      /* following PG2 involving fast periodic readings */
      PORTG |= (0x01U << PG2);
      PORTD &= ~(0x01U << PD6);
      delayms(MS_10);
      tempoff = ReadAdcSettledCh(ADC_CH7_MULX_0_1);
      /* turn it on and set adc */
      SetWindowHeater(WINDOW_ON);
      delayms(MS_10);
      tempon = ReadAdcSettledCh(ADC_CH7_MULX_0_1);
      PORTG &= ~(0x01U << PG2);
      PORTD |= (0x01U << PD6);
      SetWindowHeater(WINDOW_OFF);
      if ((tempoff < HEATER_OFF_LIM) || (tempon > HEATER_ON_LIM))
      {
        __no_operation();       /* UpdateHardwareFaultSentinel(CNT_WINDOWHEATERFAULT,true,0);//disable for first release */
      }

    }
    /* printf_P ("\r ON =%d,OFF=%d \r",tempon,tempoff); */
  }
  return(winheatstate);
}

/********************************************************************//**
 * unsigned char RangeConverter(unsigned char)
 *
 * Convert range parameter from s200 to actual range in meters.
 *************************************************************************/
unsigned char RangeConverter(unsigned char range_indx)
{
  switch (range_indx)
  {
  case R_EXTENDED:
    return('E');
  /* break; */
  case R_NORMAL:
    return('N');
  /* break; */
  case R_HALF:
    return('H');
  /* break; */
  case R_QUES:
    return('Q');
  /* break; */
  default:
    break;
  }
  return(0U);
}

/********************************************************************//**
 * InitAllPorts();
 *
 * Configure port pins as i/p or o/p.
 *************************************************************************/
void InitAllPorts(void)
{
  DDRF = 0U; /* SET  to input */
  PORTF = 0U;  /* Disable pullups */

/* set the other ports to (nearly) all outputs (portf exception): */
  DDRA = 0xFFU;
  DDRB = 0xFFU;
  DDRC = 0xFFU; /* bit 7 is for input/ reset */
  DDRD = 0xFFU;
  DDRE = 0xFFU;

  DDRE &= ~(0x01U << DDE0);
  DDRE &= ~(0x01U << DDE7); /* INPUT cap pin for s231 */
  DDRD &= ~(0x01U << DDD4); /* input cap for Ir rcv */
  DDRB &= ~(0x01U << DDB3); /* set MISO as i/p May 2012 */
  DDRG = 0x3FU; /* PG6 & 7 dont exist on PORTG */
  if (interfacemode != CONVENT_MODE)
  {
    DDRE &= ~(0x01U << DDE7);
  }
}

/********************************************************************//**
 * ZeroAllPorts();
 *
 * Set all ports to zero, used at power up.
 *************************************************************************/

void ZeroAllPorts(void)
{
  PORTA = 0U;
  DDRB |= (0x01U << PB5);
  PORTB = 0x20U; /* PB5 must be high */
  PORTC = 0U;
  PORTD = 0U;
  PORTE = 0U;
  PORTF = 0U; /* Disable pullups */
  PORTG = 0U;
/* saves current in conv mode */
  PORTD |= (0x01U << PD5);
/* Hart modem is active low */
  DisableTheHARTModem();
/* disable MX asic march 2012 */
  MxAiscSwitch(false);
}

/********************************************************************//**
 * StuckForMxAddressProgrmmer()
 *************************************************************************/
void StuckForMxAddressProgrmmer(void)
{
  char myserialx[8 + 1]; /* SIZEOFMYSERIAL+1 */
  KickWdt();
  TagToConfigReadString(MYSERIALNUM,myserialx,SIZEOFMYSERIAL);
  myserialx[SIZEOFMYSERIAL] = '\0';
 // printfv(2U,1U,false,"Ser.No. ");
  DisplayString(10U,1U,false,true,(unsigned char *)&myserialx[0],false);
 // printfv(0U,2U,false," !! Deactivated !! ");
 // printfv(0U,8U,true,"  MX Address Prog Mode  ");
  printf_P("\n\r!! MX Address Programming Mode !!\n\r");
  delayms(MS_20);
  delayms(MS_10);
  InitAllPorts();
  ZeroAllPorts();
  DDRE |= (0x01U << DDE5);
  PORTE &= ~(0x01U << PE5);
  MxAiscSwitch(true); /* turn on asic */
  ClearLatchedStates();
  FaultRelay(false);
  SetupTimerTwo10msMode();
  SleepModeActive(MAIN_LOOP);
  PORTE &= ~(0x01U << PE5);
  delayms(MS_20);
  __disable_interrupt();
  TCCR3A = 0x00U;
  TCCR3B = 0x00U;
  OCR3B = 0U;  /* PE4 comp reg */
  OCR3C = 0U;  /* PE5 comp reg */
  __enable_interrupt();
  /*
   * yes u r right we are stuck
   * we are ment to stuck for this configuration
   */
  while (true)
  {
    PORTE &= ~(0x01U << PE5);
    KickWdt();
    __sleep();
  }
}

/********************************************************************//**
void LoadDipSwitchValues()
 *************************************************************************/
void LoadDipSwitchValues(void)
{
  unsigned int switch_val;
  DILSwitchandCompatibilityBitRead(&switch_val);
  dipswitchvalue=switch_val;
  interfacemode=(unsigned char)((dipswitchvalue>>12U)&INTERFACEBITSONLY);
}

/********************************************************************//**
void PrintHeaderEnd()
 *************************************************************************/
void PrintHeaderEnd(void)
{
  printf_P("*****");
  delayms(20U);
  HeaderPrintState = true;
}

/********************************************************************//**
   unsigned char HeaderPrintDone()
 *************************************************************************/
unsigned char HeaderPrintDone(void)
{
 return(HeaderPrintState);
}

/**********************************************************//**
*  @name MyIntToAsciiHex
*  @brief
*  Converts a short intger to ascii hex. Why do we need this??
*  
*  @param none
*  itoa_integer - integer to convert,  hex_string - final ascii string.
*
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void MyIntToAsciiHex(BYTEu itoa_integer, char *hex_string)
{
  BYTEu mod16;

  if (itoa_integer < 16U)
  {
    hex_string[0] = '0';
  }
  else if (itoa_integer >= 160U)
  {
    hex_string[0] = (itoa_integer - 160U) / 16U + 'A';
  }
  else  /* else (itoa_integer < 160) */
  {
    hex_string[0] = itoa_integer / 16U + '0';
  }

  mod16 = itoa_integer % 16U;
  if (mod16 >= 10U)
  {
    hex_string[1] = mod16 - 10U + 'A';
  }
  else
  {
    hex_string[1] = mod16 + '0';
  }

  hex_string[2] = '\0';
}

/**********************************************************//**
*  @name My64bitHexPrint
*  @brief
*  Print (serial o/p) in ascii/ hex format a 64bit integer.
*  positive values only?
*  
*  @param none
*  hex_to_print - 64 bit value to print.
*
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void My64bitHexPrint(__INT64_T_TYPE__ hex_to_print)
/* This routine is required as printf_P cannot deal with %ll/ 64 bit numbers */
{
  BYTEu byte_sample;
  BYTEu print_loop;
/* Print a leading header to indicate we're showing a hex value; */
  printf_P("0x");
  for (print_loop = 1U; print_loop <= 8U; ++print_loop)
  {
    /* shift the entire 64 bit integer right so that we're only looking at the highest byte:- */
    byte_sample = (BYTEu)(hex_to_print >> 56);
    /* If the byte is less than 16 we need to print a leading zero to pad it; */
    if (byte_sample < 0x10U)
    {
      printf_P("0%x", byte_sample);
    }
    else
    {
      printf_P("%x", byte_sample);
    }
    /* Now shift the original number left so that we lose/ dont consider the byte we just printed. */
    hex_to_print <<= 8;
  }
}

/**********************************************************//**
*  @name MyI64toA()
*  @brief
 * Convert a 64 bit integer to an ASCII string
*  
*  @param none
 * itoa_integer - integer to be converted
 * itoa_string - resultant string
*
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void MyI64toA(__INT64_T_TYPE__ itoa_integer, char itoa_string[])
{
  int temp, index, back_index;
  int sign = TRUE; /* POSITIVE */
  const BYTEs ten = 10; /* Bug resolved!! */
  const BYTEs zero = 0; /* Bug resolved!! */
/* 64 bit maths division requires use of constant/ SRAM (or flash) */

  if (itoa_integer < zero)     /* record sign */
  {
    itoa_integer = -itoa_integer;            /* make n positive */
    sign = FALSE;
  }
  index = 0;
  do           /* generate digits in reverse order */
  {
    itoa_string[index++] = (char)((itoa_integer % ten) + (BYTEs)'0');       /* get next digit */
  }
  while ((itoa_integer /= ten ) > zero);        /* delete it */

  if (sign == FALSE)
  {
    itoa_string[index++] = '-';
  }

  itoa_string[index] = '\0';

/* now reverse */
  for (index = 0, back_index = (int)(strlen(itoa_string) - 1U); index < back_index; index++, back_index--)
  {
    temp = (int)itoa_string[index];
    itoa_string[index] = itoa_string[back_index];
    itoa_string[back_index] = (char)temp;
  }
}

/**********************************************************//**
*  @name My64bitDecimalPrint
*  @brief
*  Print 64 bit valribales in decimal mode. 
*  
*  @param none
*  @return none
*
*  globalvariablesused none
*  globalvariablesmodified none
**************************************************************/
void My64bitDecimalPrint(__INT64_T_TYPE__ dec_to_print)
/* This routine is required as printf_P cannot deal with %ll/ 64 bit numbers */
{
  char byte_sample[20];
  BYTEu index;
  const BYTEs ten_const = 10;
  const BYTEs zero_const = 0;

  index = 18U;
  byte_sample[19] = '\0';
/* shift the entire 64 bit integer right so that we're only looking at the lowest digit:- */
  do
  {
/* convert the remainder digit to ASCII: */
    byte_sample[index--] = (char)((dec_to_print % ten_const) + (BYTEs)'0');
/* need the ten_const & zero_const for the 64 bit bugfix */
    dec_to_print /= ten_const;
  }
  while (dec_to_print > zero_const);
/* point to the correct start of the array/ print chars:- */
  ++index;
  printf_P("%s", &byte_sample[index]);

}

/********************************************************************//**
 * unsigned char char_itoa()
 *************************************************************************/
unsigned char char_itoa(unsigned char i)
{
  unsigned char a;
  a = i & 0x0fU;
  a += 0x30U;
  if (a > 0x39U)
  {
    a += 0x07U;
  }

  return(a);
}
/* //////// EOF ////////// */
