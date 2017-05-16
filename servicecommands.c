/***********************************************************************//**
* @file  servicecommands.c
* @brief This is the file for servicemode protocol and entry for debug menu.
*
*  Used On:     FV400
***************************************************************************/
#define ENABLE_BIT_DEFINITIONS;
#include <iom1281.h>
#include <inavr.h>
#include <pgmspace.h>

#include "library.h"
#include "defs.h"
#include "scememory.h"
#include "serviceprotocol.h"
#include "servicecommands.h"
#include "uart.h"
#include "ir.h"
#include "configuration.h"
#include "peripheral.h"
#include "debug.h"
#include "video.h"
#include "hardwaretest.h"
#include "adc.h"
#include "fde.h"
#include "hardwarefaultsentinel.h"
#include "interruptcaptureandhwmonitor.h"
#include "heatedoptics.h"
#include "AFDPL15_IAR.h"
#include "panelcomms.h"
#include "s231interface.h"
#include "opmfaultsentinel.h"
#include "spi.h"

#define WT_IR_BUILTINTEST 0x1F
#define WT_IR_RESET       0x2F
#define WT_IR_OPM         0x4F
#define WT_IR_RESERVED    0x8F
#define PREAMBLECOUNT     3U /* only for HART via UART1 */

/* Set up function pointer to hardware RESET vector. */
extern void __SPROG_RESTART(void);
/* Use the checksum table held in boot sector */
extern __farflash const unsigned short g_sprog_crc16_table[256];

/* hardware fault data storage. Used in SRV_FAULTDATA message */
extern __eeprom unsigned char g_hardwarefaultdata[NUM_FAULT_FLAGS];
extern __eeprom __no_init unsigned char startupstoredstate; /* new MAR 2011 */
extern const __farflash int TemeratureAdcLookup[256];
extern const __farflash char ControllerVersion[6];
extern const __farflash char ControllerComment[19];
extern const __farflash char ControllerDate[12];
extern const __farflash char ControllerTime[9];

/* This flag is set to false only during development for extra debug */
extern unsigned char g_enablechecksum;
extern volatile unsigned char g_HWFaultState,
                              g_mux3[8],g_mux1[8],ackpackrcvd;
extern unsigned char g_flag15v_on,g_flag15v_off;
extern volatile INT16s g_TemperatureThmstr;
extern volatile INT16u g_P;
/*
 * Number of seconds of inactivity before the detector exits service mode
 * (3600 seconds is 1 hour orignally now its 5 minutes)
 */
static unsigned int serviceinactivity = 300U; /* was 3600 */
/* Store for the time of the last received message */
static unsigned long lastservicemessage,secreglong = 0U;
static unsigned char smpalmtestflg = false,convpcbmode = false,flashupdate = false,
                     mxsmpstatus = false, convmodesmpstatus = false;
unsigned char versionreadflg = false;
/* Local function prototypes */
void ProcessIrWalkTestMessage(unsigned char testcommand);

/*
 * Service commands can be send to the detector any time after the 'Welcome..'
 * message appears; even during 'normal detector operation'.
 * Putting the detector into 'service mode' prevents 'normal detector operation'
 * interfearing with what you are trying to ask the detector to do.
 * Example; you shut the camera down; the detector thinks the camera has
 * failed and it starts setting a blue colour wash and printing a 'SIGNAL LOST'
 * message on the display!
 */
unsigned char ServiceModeFlag = SERVICE_OFF,ovrideflg = false;
unsigned char FieldDataExtract = false;

/*
 * Unit serial number. Used instead of configuration when trying to 'call'
 * the debug menu
 */
char myserialnum[SIZEOFMYSERIAL];

/**************************************************//**
 * GetServiceModeFlag();
 *******************************************************/
unsigned char GetServiceModeFlag(void)
{
  unsigned char retval;
  if ((ServiceModeFlag == SERVICE_OFF) ||
      (ServiceModeFlag == SERVICE_RESERVED))
  {
    retval = false;
  }
  else
  {
    retval = true;
  }
  return(retval);
}

/**************************************************//**
 * SetServiceMode();
 * #display status on video as well...
 *******************************************************/
unsigned char SetServiceMode(unsigned char mode)
{
  unsigned char rxstate;
  /*
   * Set the global flag to enable / disable normal detector
   * operation
   */
  ServiceModeFlag = mode;
  rxstate = SPIRET_MESSAGERECEIVED;
  /* set video */
  if (mode != SERVICE_OFF)
  {
    //printfv(6U,10U,true,"Service Mode");
    //printfv(3U,11U,true,"Detection Disabled");
  }
  else
  {
    //printfv(6U,10U,false,"            ");
    //printfv(3U,11U,false,"                  ");
  }

  return(rxstate);
}

/**************************************************//**
 * UpdateLocalCopyOfMySerial();
 *******************************************************/
/* Refresh the copy of my serial number for the configuration */
void UpdateLocalCopyOfMySerial(void)
{
  TagToConfigReadString(MYSERIALNUM,myserialnum,SIZEOFMYSERIAL);
}

/**************************************************//**
 * ProcessServiceMessage();
 *******************************************************/
unsigned char ProcessServiceMessage(unsigned char channel, SRVPROTOCOLMSG *ptrsrvmsg)
{
  static unsigned char servicetypvar = SERVICE_420MA;
  unsigned char rxstate = SPIRET_UNKNOWN;
  unsigned int varint1 = 0U,mulfl = 0U,calibfltmp = 0U,calibgutmp = 0U;
  unsigned char varchar1 = 0U, tempchar = 0U,lampch = 0U;
  unsigned char *ptrchar;
  char myserial[SIZEOFMYSERIAL];
  unsigned int currentswval;
  /*
   * Status read
   * Set the Reply bit in the address field to true
   */
  ptrsrvmsg->Generic.Header.Address |= 0x8000;
  /* Based upon message type, lookup the appropriate handler */
  switch (ptrsrvmsg->Generic.Header.Type)
  {
  case SRV_SERVICEMODE:
    /*
     * Set the global flag to enable / disable normal detector
     * operation in both sensor and controller
     */
    if ((ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_OFF) ||
        (ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_ON))
    {
      if (ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_ON)
      {
        servicetypvar = SERVICE_ON;
      }
      ovrideflg = false;
      rxstate = SetServiceMode(ptrsrvmsg->ServiceModeMsg.SrvModeReq);
      mxsmpstatus = false;
      convmodesmpstatus = false;
    }
    else if (ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_420MA)
    {
      servicetypvar = SERVICE_420MA;
      SetServiceMode(SERVICE_ON);
      ClearLatchedStates();
      FaultRelay(true);
      /* enable timers */
      Enable_4_20mA();
      Output_4_20mA_Level(SET_NORMAL);
      rxstate = SPIRET_MESSAGERECEIVED;
      mxsmpstatus = false;
      convmodesmpstatus = false;
    }
    else if (ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_CONV)
    {
      servicetypvar = SERVICE_CONV;
      SetServiceMode(SERVICE_ON);
      /* send ack first */
      SrvReplyAck(channel,ptrsrvmsg);
      delayms(6U);      /* for 19.2K */
      SetUartCommMode(UART0,UART_RS485_OFF);
      /* Reset Field Data Extracted flag */
      FieldDataExtract = false;
      InitAllPorts();
      ZeroAllPorts();
      Disable_4_20mA();
      /* clear latched states */
      MxAiscSwitch(false);      /* turn off for MX */
      ClearLatchedStates();
      ChangePersonalityType(CONVENT_MODE);
      //SetVideoCircuitPower(false);
     // FaultRelay(false);
      ConvEOLTest(true);      /* EOL set */
      /* printf_P("\n\r Configured as S231 \n\r"); */
      InitCaptureFaultPulsesOnPowerLine();
      /* InitFaultPulseOnPowerLine(true); */
      StatusOfFaultPulsesOnPowerLine();      /* sets EOL if fault captured */
      SleepModeActive(MAIN_LOOP);
      delayms(200U);
      /* exit from service mode bcoz of sleep mode */
      SetServiceMode(SERVICE_OFF);
      rxstate = SPIRET_MESSAGERECEIVED;
      mxsmpstatus = false;
      convmodesmpstatus = true;
    }
    else if (ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_MX)
    {
      servicetypvar = SERVICE_MX;
      SetServiceMode(SERVICE_ON);
      ClearLatchedStates();
      /* no fault relay */
      FaultRelay(false);
      /* turn on mx asic */
      MxAiscSwitch(true);
      /* enable timer */
      Timer3MXPWMEnable();
      /* set inetial value */
      OutputMX_4_20mALevel(MX_4mA);
      /* change personality forcefully */
      ChangePersonalityType(MX_IFACE_MODE);
      rxstate = SPIRET_MESSAGERECEIVED;
      /* set flag to disable sleep during SMP MX mode Sep 2016 AR */ 
      mxsmpstatus = true;
      convmodesmpstatus = false;
    }
    else if (ptrsrvmsg->ServiceModeMsg.SrvModeReq == SERVICE_CONV_PCB)
    {
      convpcbmode = true;
      servicetypvar = SERVICE_CONV_PCB;
      SetServiceMode(SERVICE_ON);
      /* send ack first */
      SrvReplyAck(channel,ptrsrvmsg);
      delayms(6U);      /* for 19.2K */
      SetUartCommMode(UART0,UART_RS485_OFF);
      /* Reset Field Data Extracted flag */
      FieldDataExtract = false;
      InitAllPorts();
      ZeroAllPorts();
      Disable_4_20mA();
      /* clear latched states */
      MxAiscSwitch(false);      /* turn off for MX */
      ClearLatchedStates();
      ChangePersonalityType(CONVENT_MODE);
      //SetVideoCircuitPower(false);
      //FaultRelay(false);
      ConvEOLTest(true);      /* EOL set */
      /* printf_P("\n\r Configured as S231 \n\r"); */
      InitCaptureFaultPulsesOnPowerLine();
      /* InitFaultPulseOnPowerLine(true); */
      StatusOfFaultPulsesOnPowerLine();      /* sets EOL if fault captured */
      SleepModeActive(MAIN_LOOP);
      delayms(200U);
      /* exit from service mode bcoz of sleep mode */
      SetServiceMode(SERVICE_OFF);
      rxstate = SPIRET_MESSAGERECEIVED;
      mxsmpstatus = false;
      convmodesmpstatus = true;
    }
    else
    {
      __no_operation();
    }
    /* Check return state */
    if ((ptrsrvmsg->ServiceModeMsg.SrvModeReq != SERVICE_CONV)
        && (ptrsrvmsg->ServiceModeMsg.SrvModeReq != SERVICE_CONV_PCB))
    {
      SrvReplyAck(channel,ptrsrvmsg);
      /* Reset Field Data Extracted flag */
      FieldDataExtract = false;
    }
    break;
  case SRV_CLUSTERREPORT:
    /* Response, Header:REQ_CAPEAUTOA */
    ptrsrvmsg->ClusterReportRsp.Header.Type = SRV_CLUSTERREPORT;
    ptrsrvmsg->ClusterReportRsp.Header.Size =
      (sizeof(SRV_CLUSTERREPORT_RSP) - sizeof(SRVHEADER));
    /* current state of status bits */
    ptrsrvmsg->ClusterReportRsp.StatusBitsStore = GetCurrentStatusBits();
    /* gain */
    ptrsrvmsg->ClusterReportRsp.GainDuringCap = GainDuringCapture();
    /* flame A */
    ptrsrvmsg->ClusterReportRsp.FlameAmin = GetCapturedSignal(REQ_FLAMINI);
    ptrsrvmsg->ClusterReportRsp.FlameAmax = GetCapturedSignal(REQ_FLAMAXI);
    ptrsrvmsg->ClusterReportRsp.EautososA1 = (signed char)((CalGetAccumulatedValue(REQ_CAPEAUTOA) >> 32) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososA2 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOA) >> 24) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososA3 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOA) >> 16) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososA4 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOA) >> 8) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososA5 = (unsigned char)(CalGetAccumulatedValue(REQ_CAPEAUTOA) & 0xFF);
    /* flame B */
    ptrsrvmsg->ClusterReportRsp.FlameBmin = GetCapturedSignal(REQ_FLBMINI);
    ptrsrvmsg->ClusterReportRsp.FlameBmax = GetCapturedSignal(REQ_FLBMAXI);
    ptrsrvmsg->ClusterReportRsp.EautososB1 = (signed char)((CalGetAccumulatedValue(REQ_CAPEAUTOB) >> 32) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososB2 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOB) >> 24) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososB3 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOB) >> 16) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososB4 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOB) >> 8) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososB5 = (unsigned char)(CalGetAccumulatedValue(REQ_CAPEAUTOB) & 0xFF);
    /* Guard */
    ptrsrvmsg->ClusterReportRsp.Guardmin = GetCapturedSignal(REQ_GUARDMINI);
    ptrsrvmsg->ClusterReportRsp.Guardmax = GetCapturedSignal(REQ_GUARDMAXI);
    ptrsrvmsg->ClusterReportRsp.EautososG1 = (signed char)((CalGetAccumulatedValue(REQ_CAPEAUTOG) >> 32) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososG2 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOG) >> 24) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososG3 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOG) >> 16) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososG4 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOG) >> 8) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososG5 = (unsigned char)(CalGetAccumulatedValue(REQ_CAPEAUTOG) & 0xFF);
    /* Sun */
    ptrsrvmsg->ClusterReportRsp.Sunmin = GetCapturedSignal(REQ_SUNMINI);
    ptrsrvmsg->ClusterReportRsp.Sunmax = GetCapturedSignal(REQ_SUNMAXI);
    ptrsrvmsg->ClusterReportRsp.EautososS1 = (signed char)((CalGetAccumulatedValue(REQ_CAPEAUTOS) >> 32) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososS2 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOS) >> 24) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososS3 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOS) >> 16) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososS4 = (unsigned char)((CalGetAccumulatedValue(REQ_CAPEAUTOS) >> 8) & 0xFF);
    ptrsrvmsg->ClusterReportRsp.EautososS5 = (unsigned char)(CalGetAccumulatedValue(REQ_CAPEAUTOS) & 0xFF);

    ptrsrvmsg->ClusterReportRsp.BiasLtaCapture = CalGetRequestedAnalogCh(REQ_BIASLTACAP);
    SrvReplyPacket(channel,ptrsrvmsg);
    break;

  case SRV_STATUS:
    ackpackrcvd = true;
    /* Response, Header: */
    ptrsrvmsg->StatusRsp.Header.Type = SRV_STATUS;
    ptrsrvmsg->StatusRsp.Header.Size =
      (sizeof(SRV_STATUS_RSP) - sizeof(SRVHEADER));

    ptrsrvmsg->StatusRsp.ConditionFlags = GetCurrentStatusBits();
    /* Send it out */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_CNTSELFTEST:
    /* Build response header */
    ptrsrvmsg->CntSelfTestRsp.Header.Type = SRV_CNTSELFTEST;
    ptrsrvmsg->CntSelfTestRsp.Header.Size =
      (sizeof(SRV_CNTSELFTEST_RSP) - sizeof(SRVHEADER));
    /*
     * Check to see if a lamp test has been requested. Delt with differently
     * because the return state of the DSP is used in
     * if there is a protocol failure
     */
    if ((ptrsrvmsg->CntSelfTestMsg.TestType == CNTTEST_LAMP1TEST) ||
        (ptrsrvmsg->CntSelfTestMsg.TestType == CNTTEST_LAMP2TEST) )
    {
      /* This is a lamp test. */
      ptrsrvmsg->CntSelfTestRsp.Result = PASS;
      ptrsrvmsg->CntSelfTestRsp.Data = LP_FILEMENTOK;
      /* Send back response */
      SrvReplyPacket(channel,ptrsrvmsg);
    }
    else
    {
      /* Standard selftest */
      switch (ptrsrvmsg->CntSelfTestMsg.TestType)
      {
      case CNTTEST_PROGRAMCODETEST:
        varchar1 = ApplicationCodeCrcTest(CRC_FULL,false,false);
        break;
      case CNTTEST_PROGRAMDATATEST:
        varchar1 = RamSelfTest(false);
        break;
      case CNTTEST_FDEFLASHREAD:
        varchar1 = TestFdeFlashEpromRead();
        break;
      case CNTTEST_FDEFLASHWRITE:
        varchar1 = TestFdeFlashWrite();
        break;
      case CNTTEST_DACLOOPBACK:
        varchar1 = PASS;
        break;
      case CNTTEST_ALARMRELAY:
        /*varchar1=AlarmRelayCoilTest()==PASS)*/
        varchar1 = PASS;
        break;

      case CNTTEST_LC74782LOOPBACK:
        /* stop timer 2 interrupt */
        StopTimer2();
        /* Power up VID+5V circuits */
        SetVideoCircuitPower(true);
        /* Reset LC74782 registers */
        NewInitCommands();
        varchar1 = LC74782PowerOnSelfTest();
        /* Power down VID+5V circuits */
        CameraPowerControl(false,true);
        SetVideoCircuitPower(false);
        /* start timer 2 again give 200 ms for clear up */
        SetupTimerTwo10msMode();
        delayms(200U);
        break;
      case CNTTEST_CAMERALOOPBACK:
        /* stop timer 2 interrupt */
        StopTimer2();
        /* Power up VID+5V circuits */
        SetVideoCircuitPower(true);
        /* Reset LC74782 registers */
        LC74782SoftwareReset();
        /*
         * ---- Apply the following logic to the test
         * If the detector is configured NOT to have a comera and
         * no camera is found -> PASS
         * If the detector is configured to have a comera and
         * no camera is found -> FAIL
         * If the detector is NOT configured to have a comera and
         * a camera is found -> FAIL
         * If the detector is NOT configured to have a comera and
         * a camera is NOT found -> PASS
         */
        if (CameraPowerOnSelftest())
        {
          /* Camera was found */
          if (TagToConfigReadChar(CAMERAFITTED))
          {
            /* Camera fitted and camera found. Pass */
            varchar1 = PASS;
          }
          else
          {
            /* Camera not fitted - but camera found. Fail */
            varchar1 = FAIL;
          }
        }
        else
        {
          /* Camera was not found */
          if (TagToConfigReadChar(CAMERAFITTED))
          {
            /* Camera fitted - but not found. Fail */
            varchar1 = FAIL;
          }
          else
          {
            /* Camera not fitted and camera not found. Pass */
            varchar1 = PASS;
          }
        }
        /* Power down VID+5V circuits */
        CameraPowerControl(false,true);
        SetVideoCircuitPower(false);
        /* start timer 2 again give 200 ms for clear up */
        SetupTimerTwo10msMode();
        delayms(200U);
        break;
      case CNTTEST_IRLOOPBACK:
        varchar1 = PASS;//IrLoopbackTest();
        break;
      case CTTTEST_DCE:
        varchar1 = DceChecksumSelftest(false);
        break;
      case CNTTEST_SOFTWARECOMP:
        /* carry on from here */
        varchar1 = PASS;
        break;
      case CNTTEST_HEATEDOPTICS:
        varchar1 = PASS;//WindowHeaterTest(REQ_SMP_TEST);
        break;
      case CNTTEST_ONEWIREMEMORY:
        /* read 64 bit unique number compair family code (2D) thats it */
        __disable_interrupt();
        varchar1 = ReadROMOfSCE(REQ_SMP_TEST);
        __enable_interrupt();
        break;
      case CNTTEST_ADCCALIBRATION:
        /* varchar1=(REQ_SMP_TEST); */
        varchar1 = PASS;
        break;
      case CNTTEST_MOISTURE:
        varchar1 = MoistureLevelTest(REQ_SMP_TEST);
        break;
      case CNTTEST_RESERVOIR:
        varchar1 = _15V_HealthMonCheck(REQ_SMP_TEST);
        break;
      case CNTTEST_4_20ERROR:
        varchar1 = FourToTwentyLoopMonitor(REQ_SMP_TEST);
        break;
      case CNTTEST_PYROTEST:
        varchar1 = PASS;
        /* tbd */
        break;
      default:
        /*
         * Selftest type unknown. Return error with the
         * received test type
         */
        SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,
                      ptrsrvmsg->CntSelfTestMsg.TestType);
        /* Exit after sending */
        return(0U);
      }
      /* Return the results of the selftest */
      ptrsrvmsg->CntSelfTestRsp.Result = varchar1;
      ptrsrvmsg->CntSelfTestRsp.Data = tempchar;
      /* Send back response */
      SrvReplyPacket(channel,ptrsrvmsg);
    }
    break;
  case SRV_WALKTEST:
    /* Process received command */
    RemoteTestTrigger(WT_SOURCE_SMP,ptrsrvmsg->WalkTestMsg.TestType);
    /* Send Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;


  case SRV_OPMTEST:
    /* Response, Header: */
    ptrsrvmsg->OpmTestRsp.Header.Type = SRV_OPMTEST;
    ptrsrvmsg->OpmTestRsp.Header.Size =
      (sizeof(SRV_OPMTEST_RSP) - sizeof(SRVHEADER));
    /* get lamp channel */
    lampch = ptrsrvmsg->OpmTestMsg.LampChannel;
    /* identify channel then load drive level */
    if (lampch == LED)
    {
      tempchar = TagToConfigReadChar(OPMLEDDRIVE);
    }
    else if (lampch == LAMP_2)
    {
      tempchar = TagToConfigReadChar(TESTDRIVE);
    }
    else
    {
      tempchar = TagToConfigReadChar(OPMDRIVE);
    }
    /* load cal value */
    if (lampch == LED)
    {
      calibgutmp = TagToConfigReadInt(OPMLEDCAL);
    }
    else if (lampch == LAMP_2)
    {
      calibgutmp = TagToConfigReadInt(TESTLAMPCALGU);
      calibfltmp = TagToConfigReadInt(TESTLAMPCALFL);
    }
    else
    {
      calibgutmp = TagToConfigReadInt(OPMLAMPCAL);
    }
    /* carry out test set fail code true.. */
    ptrsrvmsg->OpmTestRsp.FailCode = true;
    /* check weather to use internal drive level or requested one */
    if (ptrsrvmsg->OpmTestMsg.DlCycle == 0U)
    {
      OpmTestReq(lampch,tempchar);
    }
    else
    {
      OpmTestReq(lampch,ptrsrvmsg->OpmTestMsg.DlCycle);
    }
    /* get avrage PkPk */
    if (lampch == LED)
    {
      varint1 = g_P;
    }
    else
    {
      varint1 = GetCalculatedLampAvg(lampch);
    }

    if (lampch == LAMP_2)
    {
      /* calculate (FL) value */
      mulfl = (unsigned int)(GetCalculatedLampAvg(LAMP_2_FLA) + GetCalculatedLampAvg(LAMP_2_FLB));
      mulfl = mulfl / 2U;
      mulfl = ALMTEST_MULCONST * mulfl;
      mulfl = (mulfl / ALMTEST_DIVCONST) + ALMTEST_ADDCONST;
      calibfltmp = calibfltmp / 2U; /* cal val */
      mulfl = mulfl * calibfltmp;
      /*
       * else{}
       * calculate (GU) value
       */
      varint1 = ALMTEST_MULCONST * varint1;     /* avg */
      varint1 = (varint1 / ALMTEST_DIVCONST) + ALMTEST_ADDCONST;
      calibgutmp = calibgutmp / 2U;     /* cal */
      varint1 = varint1 * calibgutmp;
    }
    else
    {
      varint1 = varint1 / 2U;
      calibgutmp = calibgutmp / 2U;
      varint1 = varint1 * calibgutmp;
    }
    if (lampch != LAMP_2)
    {
      ptrsrvmsg->OpmTestRsp.Cleanliness = varint1 / 64U;
    }
    else if (lampch == LAMP_2)
    {
      /* Score to FTE=calval*[A*Measured Value+B] where A=0.75,B=17 */
      ptrsrvmsg->OpmTestRsp.Cleanliness = mulfl / 128U;
      ptrsrvmsg->OpmTestRsp.GuardTestResult = varint1 / 128U;
    }
    else
    {}
    /* set guard result only for Lamp */
    if (lampch != LAMP_2)
    {
      ptrsrvmsg->OpmTestRsp.GuardTestResult = 0U;
    }
    /* Send response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;


  case SRV_CONTANALOGUEOUT:
    /* Process received command */
    switch (ptrsrvmsg->ContAnalogueOutMsg.Interface)
    {
    case ADC_RAW:
      Output_4_20mA_Level((unsigned char)ptrsrvmsg->ContAnalogueOutMsg.Value);
    /* break; */
    case ADC_NOSEX:        /* MXC */
      /*
       * Set raw level on ADC
       * SetAnalogueOut((unsigned char)
       * (ptrsrvmsg->ContAnalogueOutMsg.Value&0x00FF));
       * 1mA=13.81(avg)
       */
      OutputMX_4_20mALevel((unsigned char)ptrsrvmsg->ContAnalogueOutMsg.Value);
      break;
    case ADC_4_20MA:
      /* 1mA=10.5 */
      tempchar = ((unsigned char)((ptrsrvmsg->ContAnalogueOutMsg.Value) / 1000U) * 11U);
      Output_4_20mA_Level(tempchar);
      break;
    default:
      break;
    }
    /* Send Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;
  case SRV_CONTANALOGUEIN:
    /*
     * MODIFY from here AR NOV 2011
     * Response, Header:
     */
    ptrsrvmsg->ContAnalogueInRsp.Header.Type = SRV_CONTANALOGUEIN;
    ptrsrvmsg->ContAnalogueInRsp.Header.Size =
      (sizeof(SRV_CONTANALOGUEIN_RSP) - sizeof(SRVHEADER));
    /* set the request */
    varchar1 = ptrsrvmsg->ContAnalogueInMsg.Requstchannel;
    ptrsrvmsg->ContAnalogueInRsp.RespTyp = varchar1;
    /* carry out the request */
    switch (varchar1)
    {
    case ANALOGUEIN_FLAMEA:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp =
        GetRequestedAnalogCh(REQ_FLA);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)
                                                   (GetRequestedAnalogCh(REQ_FLA) - GetRequestedAnalogCh(REQ_BIAS));
      break;
    case ANALOGUEIN_FLAMEB:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp =
        GetRequestedAnalogCh(REQ_FLB);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)
                                                   (GetRequestedAnalogCh(REQ_FLB) - GetRequestedAnalogCh(REQ_BIAS));
      break;
    case ANALOGUEIN_GUARD:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp =
        GetRequestedAnalogCh(REQ_GU);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)
                                                   (GetRequestedAnalogCh(REQ_GU) - GetRequestedAnalogCh(REQ_BIAS));
      break;
    case ANALOGUEIN_SUNSENS:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp =
        GetRequestedAnalogCh(REQ_SUN);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)
                                                   (GetRequestedAnalogCh(REQ_SUN) - GetRequestedAnalogCh(REQ_BIAS));
      break;
    case ANALOGUEIN_BIAS:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp =
        GetRequestedAnalogCh(REQ_BIAS);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_MX04:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[MXVAL_D0_D4];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_MX15:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[MXVAL_D1_D5];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_MX26:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[MXVAL_D2_D6];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_MX37:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[MXVAL_D3_D7];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_BULID:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = GetBuildId();
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_VIDEOMON:
      /*
       * video byte first
       * if(g_mux1[VID_WHEAT_MON]>127)
       */
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[VID_WHEAT_MON];
      /*
       * else
       * ptrsrvmsg->ContAnalogueInRsp.Analogueresp=true;
       * window heater byte
       * set adc channel for win heater read
       */
      delayms(5U);
      __disable_interrupt();
      SetAdcMuxChannel(VIDEO_SYNC_AND_WHEAT_MON);
      /* following PG2 involving fast periodic readings */
      PORTG |= (0x01U << PG2);
      PORTD &= ~(0x01U << PD6);
      SetWindowHeater(WINDOW_ON);
      delayms(10U);
      tempchar = ReadAdcSettledCh(ADC_CH7_MULX_0_1);
      PORTG &= ~(0x01U << PG2);
      PORTD |= (0x01U << PD6);
      __enable_interrupt();
      SetWindowHeater(WINDOW_OFF);
      /* make decesion */
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;           /* clear */
      if (tempchar < 50U)
      {
        ptrsrvmsg->ContAnalogueInRsp.Processedresp |= 0x0100;
      }
      if (g_mux1[VID_WHEAT_MON] < 127U)
      {
        ptrsrvmsg->ContAnalogueInRsp.Processedresp |= 0x0001;
      }
      break;
    case ANALOGUEIN_MOISTURE:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[MOISTURE_SENS];
      if (g_mux1[MOISTURE_SENS] < 188U)
      {
        ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;            /* damp */
      }
      else if (g_mux1[MOISTURE_SENS] > 234U)
      {
        ptrsrvmsg->ContAnalogueInRsp.Processedresp = 2;            /* o/c */
      }
      else
      {
        ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)true;           /* OK */
      }
      break;
    case ANALOGUEIN_4TO20MON:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux1[_4_20MA_MON];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)((g_mux1[_4_20MA_MON] * 2U) / 10U);
      break;
    case ANALOGUEIN_24VRAIL:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[_24VRAIL_MON];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)g_mux3[_24VRAIL_MON] * 203;
      break;
    case ANALOGUEIN_TEMP:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[TEMPERATURE_RD];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = g_TemperatureThmstr;
      break;
    case ANALOGUEIN_WALKTEST:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = GetExternalWalkTestState(RAW_VAL);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)g_mux3[WALK_TEST];
      break;
    case ANALOGUEIN_4P4RAIL:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[_4V4_MONITOR];
      /*
       * following should be =(scaling fac/1000)*adcval
       * ptrsrvmsg->ContAnalogueInRsp.Processedresp=(19/1000)*g_mux3[_4V4_MONITOR];
       */
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)g_mux3[_4V4_MONITOR] * 19;
      break;
    case ANALOGUEIN_LOOPBACK:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = 0U;
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_PALNTSC:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[SYS_PAL_NTSC];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_15VRAIL:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[_15V_RAIL_MON];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)g_mux3[_15V_RAIL_MON] * 142;
      break;
    case ANALOGUEIN_RELCOIL:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[ALARM_RELAY_MON];
      /* add monitoring here June 2012 */
      if (GetAlarmRelayState())
      {
        /* alarm relay is ON */
        if (g_mux3[ALARM_RELAY_MON] < 72U)
        {
          ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)RELAYCONTACT_FAULT;
        }
        else if (g_mux3[ALARM_RELAY_MON] >= 117U)
        {
          ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)RELAYCOIL_OC;
        }
        else if ((g_mux3[ALARM_RELAY_MON] < 117U) && (g_mux3[ALARM_RELAY_MON] >= 72U))
        {
          /* relay on, */
          ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)RELAYCOIL_CONTACT_OK;
        }
        else
        {}
      }
      else
      {
        /* alm relay is off */
        if (g_mux3[ALARM_RELAY_MON] >= 206U)               /* fault fault */
        {
          ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)RELAYCOIL_OC;
        }
        else if (g_mux3[ALARM_RELAY_MON] < 117U)
        {
          ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)RELAYCONTACT_FAULT;
        }
        else if ((g_mux3[ALARM_RELAY_MON] < 206U) && (g_mux3[ALARM_RELAY_MON] >= 117U))
        {
          /* relay off, getting 139 for this situation */
          ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)RELAYCOIL_CONTACT_OK;
        }
        else
        {}
      }
      break;
    case ANALOGUEIN_5VMON:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = g_mux3[FIVEVOLT_MONITOR];
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = (signed int)g_mux3[FIVEVOLT_MONITOR] * 26;
      break;
    case ANALOGUEIN_ISWINRCV:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = GetAdcRawValue250(CH4_WIN_TEST);
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    case ANALOGUEIN_SPARE2:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = 0U;
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    default:
      ptrsrvmsg->ContAnalogueInRsp.Analogueresp = 0U;
      ptrsrvmsg->ContAnalogueInRsp.Processedresp = 0;
      break;
    }            /* switch */
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_CONTDIGITALOUT:
    ovrideflg = true;
    /*
     * Write to all digital outputs
     * if(ptrsrvmsg->ContDigitalOutMsg.AlarmLed<0xFE){
     */
    AlarmLED(ptrsrvmsg->ContDigitalOutMsg.AlarmLed);
    /* flt led */
    if (ptrsrvmsg->ContDigitalOutMsg.FaultLed == FAULT_LED_FLASH)
    {
      ovrideflg = FAULT_LED_FLASH;
    }
    else if (ptrsrvmsg->ContDigitalOutMsg.FaultLed == true)
    {
      FaultLED(ptrsrvmsg->ContDigitalOutMsg.FaultLed);
    }
    else if (ptrsrvmsg->ContDigitalOutMsg.FaultLed == false)
    {
      ovrideflg = true;
      FaultLED(false);
    }
    else
    {}
    AlarmRelay(ptrsrvmsg->ContDigitalOutMsg.AlarmRelay);
    FaultRelay(ptrsrvmsg->ContDigitalOutMsg.FaultRelay);
    SetWindowHeater(ptrsrvmsg->ContDigitalOutMsg.WindowHeater);
    ConvEOLTest(ptrsrvmsg->ContDigitalOutMsg.EndOfLine);
    ConvAlarmControl(ptrsrvmsg->ContDigitalOutMsg.ConvAlarm);
    if (ptrsrvmsg->ContDigitalOutMsg.HartModem)
    {
      SetHartModem(true);
    }
    else
    {
      SetHartModem(false);
    }
    MxAiscSwitch(ptrsrvmsg->ContDigitalOutMsg.Mxasicsw);
    /* Send Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;
  case SRV_CONTDIGITALIN:
    /* Response, Header: */
    ptrsrvmsg->ContDigitalInRsp.Header.Type = SRV_CONTDIGITALIN;
    ptrsrvmsg->ContDigitalInRsp.Header.Size =
      (sizeof(SRV_CONTDIGITALIN_RSP) - sizeof(SRVHEADER));
    /* set pin PD5 as i/p */
    SDAPin(INPUT,0U);
    delayms(10U);
    if (servicetypvar != SERVICE_ON)
    {
      /* dynamic dip switch read */
      StopTimer2();
      DILSwitchandCompatibilityBitRead(&currentswval);
      SetupTimerTwo10msMode();
      ptrsrvmsg->ContDigitalInRsp.DipSwitch1 = (unsigned char)
                                               (currentswval & 0x00FFU);
      if (g_mux1[VID_WHEAT_MON] > 127U)
      {
        ptrsrvmsg->ContDigitalInRsp.VideoSyncStatus = false;
      }
      else
      {
        ptrsrvmsg->ContDigitalInRsp.VideoSyncStatus = true;
      }
      ptrsrvmsg->ContDigitalInRsp.DipSwitch2 = (unsigned char)
                                               ((currentswval >> 8U) & 0xFFU);
      ptrsrvmsg->ContDigitalInRsp.MxValues = MxDigitalOutputRead();
      ptrsrvmsg->ContDigitalInRsp.ValPortD = PIND;    /* was PORTD till Jan 16 */
    }
    else
    {
      /* passive dip switch read */
      ptrsrvmsg->ContDigitalInRsp.DipSwitch1 = (unsigned char)
                                               (GetDipSwitchVal() & 0x00FFU);
      if (g_mux1[VID_WHEAT_MON] > 127U)
      {
        ptrsrvmsg->ContDigitalInRsp.VideoSyncStatus = false;
      }
      else
      {
        ptrsrvmsg->ContDigitalInRsp.VideoSyncStatus = true;
      }
      ptrsrvmsg->ContDigitalInRsp.DipSwitch2 = (unsigned char)
                                               ((GetDipSwitchVal() >> 8U) & 0xFFU);
      ptrsrvmsg->ContDigitalInRsp.MxValues = MxDigitalOutputRead();
      ptrsrvmsg->ContDigitalInRsp.ValPortD = PIND;    /* PORTD; */
    }
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;

  case SRV_DSPDIGITALOUT:
    /*
     * Use varchar1 as a flag. If still true at end, all
     * messages have been sent across SPI without protocol fail
     * Set outputs on the sensor board
     * window lamp
     */
    if (ptrsrvmsg->DspDigitalOutMsg.Lamp & 0x01U)
    {
      /* turn on alarm lamps */
      LampControl(WINDOW_LAMP_ON);
    }
    else
    {
      LampControl(WINDOW_LAMP_OFF);
    }
    /* alm lamp */
    if (ptrsrvmsg->DspDigitalOutMsg.Lamp & 0x02U)
    {
      /* turn on alarm lamps */
      LampControl(ALARM_LAMP_ON);
    }
    else
    {
      LampControl(ALARM_LAMP_OFF);
    }
    if (ptrsrvmsg->DspDigitalOutMsg.Lamp & 0x04U)
    {
      /* turn on opm LED */
      PORTB &= ~(0x01U << PB6);
    }
    else
    {
      PORTB |= (0x01U << PB6);
    }
    if (ptrsrvmsg->DspDigitalOutMsg.Lamp & 0x08U)
    {
      StopTimer2();
    }
    else
    {
      SetupTimerTwo10msMode();
    }
    switch ((ptrsrvmsg->DspDigitalOutMsg.Lamp & 0x70U) >> 4U)
    {
    /* A=PC4,B=PC3 and C=PC2 */
    case 0U:        /* 000 */
      PORTC &= ~(0x01U << PC4);
      PORTC &= ~(0x01U << PC3);
      PORTC &= ~(0x01U << PC2);
      break;
    case 1U:        /* 001 */
      PORTC |= (0x01U << PC4);
      PORTC &= ~(0x01U << PC3);
      PORTC &= ~(0x01U << PC2);
      break;
    case 2U:        /* 010 */
      PORTC &= ~(0x01U << PC4);
      PORTC |= (0x01U << PC3);
      PORTC &= ~(0x01U << PC2);
      break;
    case 3U:        /* 011 */
      PORTC |= (0x01U << PC4) | (0x01U << PC3);
      PORTC &= ~(0x01U << PC2);
      break;
    case 4U:        /* 100 */
      PORTC &= ~(0x01U << PC4);
      PORTC &= ~(0x01U << PC3);
      PORTC |= (0x01U << PC2);
      break;
    case 5U:        /* 101 */
      PORTC |= (0x01U << PC4) | (0x01U << PC2);
      PORTC &= ~(0x01U << PC3);
      break;
    case 6U:        /* 110 */
      PORTC |= (0x01U << PC2) | (0x01U << PC3);
      PORTC &= ~(0x01U << PC4);
      break;
    case 7U:        /* 111 */
      PORTC |= (0x01U << PC2) | (0x01U << PC3) | (0x01U << PC4);
      break;
    default:
      break;
    }

    /* for automatic, gain would be set high */
    if (ptrsrvmsg->DspDigitalOutMsg.GainSelect)
    {
      PORTG |= (0x01U << PG0);
    }
    else
    {
      PORTG &= ~(0x01U << PG0);
    }
    /* GuardTestLine */
    if (ptrsrvmsg->DspDigitalOutMsg.GuardElectricalTest == true)
    {
      PORTA |= (0x01U << PA4);
    }
    else
    {
      PORTA &= ~(0x01U << PA4);
    }

    /* Set WalkTest IR */
    if (ptrsrvmsg->DspDigitalOutMsg.WalktestIR == true)
    {
      IrRcvCktSwitch(true);
    }
    else
    {
      IrRcvCktSwitch(false);
    }
    /* Set WindowTest IR led ckt */
    if (ptrsrvmsg->DspDigitalOutMsg.WindowTestIR == true)
    {
      PORTC |= (0x01U << PC0);
    }
    else
    {
      PORTC &= ~(0x01U << PC0);
    }
    /* Reply ACK. */
    SrvReplyAck(channel,ptrsrvmsg);

    break;
  case SRV_GETACTIVITY:
    /* Response, Header:FlameAStatus */
    ptrsrvmsg->GetActivityRsp.Header.Type = SRV_GETACTIVITY;
    ptrsrvmsg->GetActivityRsp.Header.Size =
      (sizeof(SRV_GETACTIVITY_RSP) - sizeof(SRVHEADER));
    /* status bits */
    ptrsrvmsg->GetActivityRsp.StatusBits = GetCurrentStatusBits();
    /* check for gain, robust way is monitor the PG0 gain amp switch pin */
    if (PORTG & GAIN_PING0)
    {
      /* its high gain */
      ptrsrvmsg->GetActivityRsp.GainStatus = true;
    }
    else
    {
      /* low gain */
      ptrsrvmsg->GetActivityRsp.GainStatus = false;
    }
    /*
     * eauto,ecross and eamulr all these 3 variables are 64 bits but the max goes upto
     * 40 bits only so each of then is divided into 5 bytes. Very first byte would be a signed
     * beacuse wauto can also negative, although ecross and eamulr cannot go negative
     * but they are same by defination so same apporach is used.
     */
    ptrsrvmsg->GetActivityRsp.EautoB1 = (signed char)((GetAccumulatedValue(REQ_EAUTO) >> 32) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EautoB2 = (unsigned char)((GetAccumulatedValue(REQ_EAUTO) >> 24) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EautoB3 = (unsigned char)((GetAccumulatedValue(REQ_EAUTO) >> 16) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EautoB4 = (unsigned char)((GetAccumulatedValue(REQ_EAUTO) >> 8) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EautoB5 = (unsigned char)(GetAccumulatedValue(REQ_EAUTO) & 0xFF);

    ptrsrvmsg->GetActivityRsp.EcrossB1 = (signed char)((GetAccumulatedValue(REQ_ECROSS) >> 32) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EcrossB2 = (unsigned char)((GetAccumulatedValue(REQ_ECROSS) >> 24) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EcrossB3 = (unsigned char)((GetAccumulatedValue(REQ_ECROSS) >> 16) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EcrossB4 = (unsigned char)((GetAccumulatedValue(REQ_ECROSS) >> 8) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EcrossB5 = (unsigned char)(GetAccumulatedValue(REQ_ECROSS) & 0xFF);

    ptrsrvmsg->GetActivityRsp.EamulrB1 = (signed char)((GetAccumulatedValue(REQ_EAMULR) >> 32) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EamulrB2 = (unsigned char)((GetAccumulatedValue(REQ_EAMULR) >> 24) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EamulrB3 = (unsigned char)((GetAccumulatedValue(REQ_EAMULR) >> 16) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EamulrB4 = (unsigned char)((GetAccumulatedValue(REQ_EAMULR) >> 8) & 0xFF);
    ptrsrvmsg->GetActivityRsp.EamulrB5 = (unsigned char)(GetAccumulatedValue(REQ_EAMULR) & 0xFF);

    ptrsrvmsg->GetActivityRsp.FlameAStatus = 0U;
    ptrsrvmsg->GetActivityRsp.FlameACurrentVal = GetRequestedAnalogCh(REQ_FLA);
    ptrsrvmsg->GetActivityRsp.FlameBStatus = 0U;
    ptrsrvmsg->GetActivityRsp.FlameBCurrentVal = GetRequestedAnalogCh(REQ_FLB);
    ptrsrvmsg->GetActivityRsp.GuardStatus = 0U;
    ptrsrvmsg->GetActivityRsp.GuardCurrentVal = GetRequestedAnalogCh(REQ_GU);
    ptrsrvmsg->GetActivityRsp.SunStatus = 0U;
    ptrsrvmsg->GetActivityRsp.SunCurrentVal = GetRequestedAnalogCh(REQ_SUN);
    ptrsrvmsg->GetActivityRsp.BiasCurrentVal = GetRequestedAnalogCh(REQ_BIAS);
    ptrsrvmsg->GetActivityRsp.BiasLta = GetRequestedAnalogCh(REQ_LTABIAS);

    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_VERSION:
    /* Response, Header: */
    versionreadflg = true;
    ptrsrvmsg->VersionRsp.Header.Type = SRV_VERSION;
    ptrsrvmsg->VersionRsp.Header.Size =
      (sizeof(SRV_VERSION_RSP) - sizeof(SRVHEADER));
    /* Version */
    flashcopy(&ptrsrvmsg->VersionRsp.ContVersion[0],
              ControllerVersion,SIZEOF_VERSION);
    /* Build date */
    flashcopy(&ptrsrvmsg->VersionRsp.ContBuildDate[0],
              ControllerDate,SIZEOF_BUILDDATE);
    ptrsrvmsg->VersionRsp.ContBuildDate[11] = (signed char)',';
    ptrsrvmsg->VersionRsp.ContBuildDate[12] = (signed char)' ';
    flashcopy(&ptrsrvmsg->VersionRsp.ContBuildDate[13],
              ControllerTime,SIZEOF_BUILDDATE - 13U);
    /* Build comment */
    flashcopy(&ptrsrvmsg->VersionRsp.ContBuildComment[0],
              ControllerComment,SIZEOF_BUILDCOMMENT);
    /* Bulid ID */
    ptrsrvmsg->VersionRsp.BulidId = GetBuildId();
    /* PAL or NTSC or No Camera */
    ptrsrvmsg->VersionRsp.PalNtscCamera = GetVideoType();
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    versionreadflg = false;
    break;
  case SRV_FDEEXTRACT:
    /* Keep local copy of Block address */
    varint1 = ptrsrvmsg->FdeExtractMsg.BlockAddress;
    /* Response, Header: */
    ptrsrvmsg->FdeExtractRsp.Header.Type = SRV_FDEEXTRACT;
    ptrsrvmsg->FdeExtractRsp.Header.Size =
      (sizeof(SRV_FDEEXTRACT_RSP) - sizeof(SRVHEADER));
    /* Send back the requested block as part of the response */
    ptrsrvmsg->FdeExtractRsp.BlockAddress = varint1;
    /* Read data out of FDE and into message body */
    GetRawFdeData(varint1,&ptrsrvmsg->FdeExtractRsp.BlockData[0]);
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    /* Set flag used to mark service mode log event that data was extracted */
    FieldDataExtract = true;
    break;
  case SRV_ACTION:
    ptrsrvmsg->ActionRsp.Header.Type = SRV_ACTION;
    ptrsrvmsg->ActionRsp.Header.Size =
      (sizeof(SRV_ACTION_RSP) - sizeof(SRVHEADER));
    /* Save action to carry out in varchar1 */
    varchar1 = ptrsrvmsg->ActionMsg.Action;
    /* Set return state OK */
    rxstate = SPIRET_MESSAGERECEIVED;
    /* clear flag if the action type is not recognised */
    tempchar = PASS;
    /* Sent response data to zero if unused. */
    ptrsrvmsg->ActionRsp.ActionResults = 0U;
    varint1 = 0U;
    /* Carry out command */
    switch (varchar1)
    {
    case ACT_FDERESET:
      /*
       * Reset the timestamp used to mark a log in the FDE
       * we also have to clear the FDE to prevent it getting
       * corrupted
       */
      ResetTimestampNonVolatile();
    /* Follow on with blanking the FDE (no break) */
    case ACT_FDEBLANK:
      /*
       * Clear all data
       * EraseFlashSector(1);
       * EraseFlashSector(2);
       * EraseFlashSector(3);
       * EraseFlashSector(4);
       */
      BulkEraseMemory(SPI_FDE_EEPROM);
      /*
       * Reinitalise the FDE ready for data to be written
       * from the beggining
       */
      InitFde(false);
      break;
    case ACT_DCETOFACTORYDEFAULTS:
      /*
       * Reset to factory defaults.
       * Save factory ident and test data
       */
      InitDceToFactoryDefault(true);
      break;
    case ACT_SOFTCONTROLLERRESET:
      /* Reply now because we will not get the chance later! */
      SrvReplyPacket(channel,ptrsrvmsg);
      /* Wait for message to be sent */
      delayms(100U);
      /* Reset controller */
      __SPROG_RESTART();
      break;
    case ACT_REDUCESERVICETIMOUT:
      /*
       * Modify the number of seconds the detector exits service mode
       * 2 minutes and 4 seconds. Why? Ask Graham, he told me to do it.
       */
      ModifyServiceModeTimeout(124U);
      break;
    case ACT_CALIBRATEDETECTOR:
      SrvReplyPacket(channel,ptrsrvmsg);
      ovrideflg = true;
      DoAllCalibrate250Sensors_2ch(REQ_SMP);
      ovrideflg = false;
      break;
    case ACT_CALIBRATEVERIFY:
      SrvReplyPacket(channel,ptrsrvmsg);
      ovrideflg = true;
      DoAllCalibrate250Sensors_2ch(REQ_SMP_VERIFY);
      ovrideflg = false;
      break;
    case ACT_ENABLEHARTCOMMS:
      /* StopTimer2(); */
      Output_4_20mA_Level(SET_4mAHART);
      InitUART(UART1,BAUD1200,NOPARITY);         /* set the baudrate HART */
      EnableTheHARTModem();
      delayms(10U);
      /* preamble */
      TransmitByte(UART1,0xffU);
      TransmitByte(UART1,0xffU);
      TransmitByte(UART1,0xffU);
      /* transmit over hart modem */
      TransmitByte(UART1,'H');
      TransmitByte(UART1,'A');
      TransmitByte(UART1,'R');
      TransmitByte(UART1,'T');
      TransmitByte(UART1,' ');
      TransmitByte(UART1,'A');
      TransmitByte(UART1,'C');
      TransmitByte(UART1,'T');
      TransmitByte(UART1,'I');
      TransmitByte(UART1,'V');
      TransmitByte(UART1,'E');
      TransmitByte(UART1,'\r');
      break;
    case ACT_DISABLEHARTCOMMS:
      /* StartTimer2_10msMode(); */
      DisableTheHARTModem();
      /* disable USART1 */
      UCSR1A = 0U; UCSR1B = 0U; UCSR1C = 0U;
      break;
    case ACT_SIGNALCAPTURE:
      /*
       * here values would not be written in to DCE instead
       * they will be stored into SRAM.
       */
      SrvReplyPacket(channel,ptrsrvmsg);
      ovrideflg = true;
      DoAllCalibrate250Sensors_2ch(REQ_CAPTURE);
      ovrideflg = false;
      break;
    case ACT_TEST_25RANGE:
      smpalmtestflg = true;
      ClearLatchedStates();
      KickWdt();
      ovrideflg = true;
      SrvReplyPacket(channel,ptrsrvmsg);
      if (PersonalityType() == MX_IFACE_MODE)
      {
        AlarmLED(true);
        //printfv(3U,11U,false,"                  ");
        //printfv(0U,9U,false,"25M");
        /* override range and delay settings */
        SetMxRangeDelay(1U,3U,5U);
        if (!secreglong)
        {
          secreglong = GetSecondsSincePowerUp();
        }
        /* run algo for 15 sec */
        while (secreglong + 15U > GetSecondsSincePowerUp())
        {
          /* service algo */
          AlarmSentinel();
          /* update status bits */
          GetCurrentStatusBits();
          /* kick wdt */
          KickWdt();
        }
        ResetTotals();
        secreglong = 0U;
        //printfv(0U,9U,false,"   ");
        //printfv(8U,9U,false,"         ");
        //printfv(3U,11U,true,"Detection Disabled");
        AlarmLED(false);
      }
      ResetSmpAlmTestFreezFlg();
      ovrideflg = false;
      smpalmtestflg = false;
      break;
    case ACT_TEST_50RANGE:
      smpalmtestflg = true;
      ClearLatchedStates();
      KickWdt();
      ovrideflg = true;
      SrvReplyPacket(channel,ptrsrvmsg);
      if (PersonalityType() == MX_IFACE_MODE)
      {
        AlarmLED(true);
        //printfv(3U,11U,false,"                  ");
        //printfv(0U,9U,false,"50M");
        /* override range and delay settings */
        SetMxRangeDelay(0U,3U,5U);
        if (!secreglong)
        {
          secreglong = GetSecondsSincePowerUp();
        }
        /* run algo for 15 sec */
        while (secreglong + 15U > GetSecondsSincePowerUp())
        {
          AlarmSentinel();
          /* update status bits */
          GetCurrentStatusBits();
          KickWdt();
        }
        ResetTotals();
        secreglong = 0U;
        //printfv(0U,9U,false,"   ");
        //printfv(8U,9U,false,"         ");
        //printfv(3U,11U,true,"Detection Disabled");
        AlarmLED(false);
      }
      ResetSmpAlmTestFreezFlg();
      ovrideflg = false;
      smpalmtestflg = false;
      break;
    case ACT_STOPWATCHDOG:
      SrvReplyPacket(channel,ptrsrvmsg);
      delayms(50U);
      while (1U)
      {}
    /* break; */
    default:
      tempchar = FAIL;
      break;
    }
    KickWdt();
    /* for calibration reply already sent */
    if ((varchar1 != ACT_CALIBRATEDETECTOR) && (varchar1 != ACT_SIGNALCAPTURE) &&
        (varchar1 != ACT_CALIBRATEVERIFY) && (varchar1 != ACT_TEST_50RANGE) &&
        (varchar1 != ACT_TEST_25RANGE))
    {
      /* Check end state */
      if (rxstate == SPIRET_MESSAGERECEIVED)
      {
        /*
         * Command did'nt use the DSP to carry out the command
         * or if it did, there were no comms problems
         */
        if (tempchar == PASS)
        {
          /* Action recognised and carried out */
          SrvReplyPacket(channel,ptrsrvmsg);
        }
        else
        {
          /* Action type not recognised. Data=requested action */
          SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID, varchar1);
        }
      }
      else
      {
        /* Error when communicating with DSP */
        SrvReplyError(channel,ptrsrvmsg, SRVERR_DSPCOMMS, rxstate);
      }
    }
    break;
  /* re addtion NOV 2011 AR */
  case SRV_VIDEOCONTROL:
    /* Process received command */
    if (ptrsrvmsg->VideoControlMsg.PowerLC74782 != 0xFFU)
    {
      /* Update camera power if we are about to shut down LC74782 */
      if (ptrsrvmsg->VideoControlMsg.PowerLC74782 == false)
      {
        CameraPowerControl(ptrsrvmsg->VideoControlMsg.PowerCamera,false);
      }
      /* Power on or off the 5V video subsystem */
      SetVideoCircuitPower(ptrsrvmsg->VideoControlMsg.PowerLC74782);
      /* Update the camera power if we have just powered up the LC74782 */
      if (ptrsrvmsg->VideoControlMsg.PowerLC74782)
      {
        CameraPowerControl(ptrsrvmsg->VideoControlMsg.PowerCamera,false);
      }
      /* Initalise display */
      NewInitCommands();
      /*
       * LC74782CommandSetup();
       * Set colour wash (if required)
       */
      if (ptrsrvmsg->VideoControlMsg.Colourwash != 0x00U)
      {
        VideoColourWash(ptrsrvmsg->VideoControlMsg.Colourwash - 1U);
      }
    }
    else
    {
      /*
       * When PowerLC74782 = 0xFF, the next two bytes are user
       * defined video commands. Send them.
       */
      VideoUserDefined(ptrsrvmsg->VideoControlMsg.PowerCamera,
                       ptrsrvmsg->VideoControlMsg.Colourwash);
    }
    /* Send to Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;
  /* addtion NOV 2011 */
  case SRV_VIDEOPRINTTEXT:
    /* Process received command */
    DisplayString(ptrsrvmsg->VideoPrintTextMsg.XStart,
                  ptrsrvmsg->VideoPrintTextMsg.YStart,
                  ptrsrvmsg->VideoPrintTextMsg.Flash,
                  ptrsrvmsg->VideoPrintTextMsg.AsciiText,
                  &ptrsrvmsg->VideoPrintTextMsg.PrintString[0],false);
    /* Send Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;

  case SRV_FLASHUPDATE:
    /*
     * erase memory
     * stop timer for smooth transfer
     */
    if (!flashupdate)
    {
      FaultRelay(false);
      StopTimer2();
      BulkEraseMemory(SPI_FDE_EEPROM);
      flashupdate = true;
      KickWdt();
    }
    /* Response header */
    ptrsrvmsg->FlashUpdateRsp.Header.Type = SRV_FLASHUPDATE;
    ptrsrvmsg->FlashUpdateRsp.Header.Size =
      (sizeof(SRV_FLASHUPDATE_RSP) - sizeof(SRVHEADER));
    /* Save the block address */
    varint1 = ptrsrvmsg->FlashUpdateMsg.BlockAddress;
    /* Assume the block has not been written */
    varchar1 = false;
    /* Process received command */
    if (ptrsrvmsg->FlashUpdateMsg.Destination == FLASH_CONTROLLER)    /* Check dest */
    {       /* This block is destined for the controller board */
      SetRawFdeData(ptrsrvmsg->FlashUpdateMsg.BlockAddress,
                    &ptrsrvmsg->FlashUpdateMsg.BlockData[0]);
      /* Indicate the data has been written */
      varchar1 = true;
    }
    else
    {
      /* Invalid destination. */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID, rxstate);
      break;
    }
    /* Copy back Block address to the response */
    ptrsrvmsg->FlashUpdateRsp.DataWritten = varchar1;
    ptrsrvmsg->FlashUpdateRsp.BlockAddress = varint1;
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_FLASHAUTHORISE:
    /* Response, Header: */
    flashupdate = false;
    ptrsrvmsg->FlashAuthoriseRsp.Header.Type = SRV_FLASHAUTHORISE;
    ptrsrvmsg->FlashAuthoriseRsp.Header.Size =
      (sizeof(SRV_FLASHAUTHORISE_RSP) - sizeof(SRVHEADER));
    /* Check the destination of the message. Cont or DSP? */
    if (ptrsrvmsg->FlashAuthoriseMsg.Destination == FLASH_CONTROLLER)
    {
      /*
       * Calculate the checksum of the FDE scrachpad that
       * contains the application code
       */
      varint1 = CalculateFdeChecksum(0x00000000UL,0x0001E000UL);
      /*
       * Does expected checksum calculated by the download application
       * match the actual checksum calculated by the detector. If
       * So, mark the FDE 'Ready for programming'
       */
      if (varint1 == ptrsrvmsg->FlashAuthoriseMsg.ExpectedChecksum)
      {
        /* Checksum match. Mark ready for programming. */
        MarkFdeReadyToProgramFlash(varint1);
        /* Construct response */
        ptrsrvmsg->FlashAuthoriseRsp.AuthorisationGranted = true;
      }
      else
      {
        ptrsrvmsg->FlashAuthoriseRsp.AuthorisationGranted = false;
      }
      ptrsrvmsg->FlashAuthoriseRsp.ActualChecksum = varint1;
    }
    else
    {
      /* Invalid destination. */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID, rxstate);
      break;
    }
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_GETFAULTDATA:
    /* Response, Header: */
    ptrsrvmsg->Generic.Header.Type = SRV_GETFAULTDATA;
    /* Initalise a pointer to the data section of the generic message */
    ptrchar = &ptrsrvmsg->Generic.Data;
    /* Default size to zero */
    ptrsrvmsg->Generic.Header.Size = 0U;
    /*
     * Loop round
     * varchar1=fault flag under test
     * tempchar=number of flags that are set;
     */
    tempchar = 0U;
    for (varchar1 = 0U;
         ((varchar1 < FAULTENDMARKER) && (tempchar < sizeof(SRVPROTOCOLMSG) - sizeof(SRVHEADER)) );
         varchar1++)
    {
      /* Test fault flag condition */
      if (ReadHardwareFaultFlag(varchar1) == true)
      {
        /* The flag is set, append the fault enum to message */
        *ptrchar = varchar1;
        /* Next data byte */
        ptrchar++;
        /* And fault data */
        *ptrchar = g_hardwarefaultdata[varchar1];
        /* Move pointer away from this location to prevent corruption */
        SetEepromPointerToDefault();
        /* Next data byte */
        ptrchar++;
        /* Increment size */
        ptrsrvmsg->Generic.Header.Size += 2U;
      }
    }
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_GETTEMPERATUREDATA:
    /* Response, Header: */
    ptrsrvmsg->GetTemperatureDataRsp.Header.Type = SRV_GETTEMPERATUREDATA;
    ptrsrvmsg->GetTemperatureDataRsp.Header.Size =
      (sizeof(SRV_GETTEMPERATUREDATA_RSP) - sizeof(SRVHEADER));
    ptrsrvmsg->GetTemperatureDataRsp.TemperatureNow =
      GetDetectorTemperature();
    ptrsrvmsg->GetTemperatureDataRsp.TemperatureMin = 0;
    /* GetDetectorTemperatureMaxMinData(SELECT_TEMP_MIN); */
    ptrsrvmsg->GetTemperatureDataRsp.TemperatureMax = 0;
    /* GetDetectorTemperatureMaxMinData(SELECT_TEMP_MAX); */
    ptrsrvmsg->GetTemperatureDataRsp.TempSlopeNow = 0;
    /* GetDetectorTemperatureSlope(); */
    ptrsrvmsg->GetTemperatureDataRsp.TempSlopeMin = 0;
    /* GetDetectorTemperatureMaxMinData(SELECT_TSLOPE_MIN); */
    ptrsrvmsg->GetTemperatureDataRsp.TempSlopeMax = 0;
    /* GetDetectorTemperatureMaxMinData(SELECT_TSLOPE_MAX); */
    ptrsrvmsg->GetTemperatureDataRsp.TempSampleTime = 0U;
    /*
     * (unsigned int)(GetSecondsSincePowerUp()%SECONDPERHOUR);
     * Send back response
     */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_SETCONTROLLERIDENTITY:
    /* Write Serial number to DCE */
    TagToDceWriteString(CONTSERIALNUM,
                        (char *)ptrsrvmsg->SetControllerIdentityMsg.SerialNumber,
                        SIZEOFUNITSERIAL);
    /* write mx Asic serial */
    TagToDceWriteString(MXSERIALNUM,
                        (char *)ptrsrvmsg->SetControllerIdentityMsg.Mxasicnumber,
                        SIZEOFMXASICSERIAL);
    /* Update DCE checksum */
    UpdateChecksum();
    /* Send Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;
  case SRV_SETTOPCASEIDENTITY:
    /* Write my Serial number to DCE */
    TagToDceWriteString(MYSERIALNUM,
                        (char *)ptrsrvmsg->SetTopCaseIdentityMsg.MySerialNumber,
                        SIZEOFMYSERIAL);
    /* Write Serial number to DCE */
    TagToDceWriteString(TOPCSERIALNUM,
                        (char *)ptrsrvmsg->SetTopCaseIdentityMsg.SerialNumber,
                        SIZEOFUNITSERIAL);
    /* Update DCE checksum */
    UpdateChecksum();
    /* Send Ack response */
    SrvReplyAck(channel,ptrsrvmsg);
    break;
  case SRV_GETIDENTITY:
    /* Response, Header: */
    ptrsrvmsg->GetIdentityRsp.Header.Type = SRV_GETIDENTITY;
    ptrsrvmsg->GetIdentityRsp.Header.Size =
      (sizeof(SRV_GETIDENTITY_RSP) - sizeof(SRVHEADER));
    /* Retreive my SerialNumber from DCE */
    TagToConfigReadString(MYSERIALNUM,
                          (char *)ptrsrvmsg->GetIdentityRsp.MySerialNumber,
                          SIZEOFMYSERIAL);
    /* Retreive the Top Case  SerialNumber from DCE */
    TagToConfigReadString(TOPCSERIALNUM,
                          (char *)ptrsrvmsg->GetIdentityRsp.TopCaseSerialNumber,
                          SIZEOFUNITSERIAL);
    /* Retreive the Controller SerialNumber from DCE */
    TagToConfigReadString(CONTSERIALNUM,
                          (char *)ptrsrvmsg->GetIdentityRsp.ControllerSerialNumber,
                          SIZEOFUNITSERIAL);
    delayms(2U);
    __disable_interrupt();
    /* Retreive the Sensor board 19 bytes */
    ReadPageOfSCE(SERIALNUMBER,(unsigned char *)ptrsrvmsg->GetIdentityRsp.SensorSerialNumber,
                  SIZEOFSENSORBOARDSERIAL);     /* just read 19 bytes */
    __enable_interrupt();
    /* Retreive one wire serial number 48 bits */
    __disable_interrupt();
    GetOneWireSerialNumber((unsigned char *)ptrsrvmsg->GetIdentityRsp.OneWireSerialNumber);
    __enable_interrupt();
    /* Retreive MX ASIC serial number */
    TagToConfigReadString(MXSERIALNUM,
                          (char *)ptrsrvmsg->GetIdentityRsp.MxAsicSerial,
                          SIZEOFMXASICSERIAL);
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;

  case SRV_SETTESTDATA:
    /* Response, Header: */
    ptrsrvmsg->SetTestDataRsp.Header.Type = SRV_SETTESTDATA;
    ptrsrvmsg->SetTestDataRsp.Header.Size =
      (sizeof(SRV_SETTESTDATA_RSP) - sizeof(SRVHEADER));
    /* Pass on to function handler */
    rxstate = SetTestData(
      &ptrsrvmsg->SetTestDataRsp.ErrorValue,
      ptrsrvmsg->SetTestDataMsg.TestType,
      &ptrsrvmsg->SetTestDataMsg.TestDate[0],
      ptrsrvmsg->SetTestDataMsg.TestResult,
      ptrsrvmsg->SetTestDataMsg.TestIteration,
      &ptrsrvmsg->SetTestDataMsg.Reserved[0]);
    /* Check return state */
    if (rxstate == SPIRET_MESSAGERECEIVED)
    {
      /* Send back response */
      SrvReplyPacket(channel,ptrsrvmsg);
    }
    else
    {
      /* Error when communicating with DSP */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DSPCOMMS, rxstate);
    }
    break;
  case SRV_GETTESTDATA:
    /* Response, Header: */
    ptrsrvmsg->GetTestDataRsp.Header.Type = SRV_GETTESTDATA;
    ptrsrvmsg->GetTestDataRsp.Header.Size =
      (sizeof(SRV_GETTESTDATA_RSP) - sizeof(SRVHEADER));
    /* Pass on to function handler */
    rxstate = GetTestData(
      ptrsrvmsg->GetTestDataMsg.TestType,
      &ptrsrvmsg->GetTestDataRsp.TestDate[0],
      &ptrsrvmsg->GetTestDataRsp.TestResult,
      &ptrsrvmsg->GetTestDataRsp.TestIteration,
      &ptrsrvmsg->GetTestDataRsp.Reserved[0]);
    /* Check return state */
    if (rxstate == SPIRET_MESSAGERECEIVED)
    {
      /* Everything OK, return response */
      SrvReplyPacket(channel,ptrsrvmsg);
    }
    else if (rxstate == SPIRET_UNKNOWN)
    {
      /* TestType not recognised */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID, rxstate);
    }
    else
    {
      /* Error when communicating with DSP */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DSPCOMMS, rxstate);
    }
    break;

  case SRV_SETCALIBRATION:
    /* Response, Header: */
    ptrsrvmsg->SetCalibrationMsg.Header.Type = SRV_SETCALIBRATION;
    ptrsrvmsg->SetCalibrationMsg.Header.Size =
      (sizeof(SRV_SETCALIBRATION_RSP) - sizeof(SRVHEADER));

    if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_FLAME_A_HIGH)
    {
      TagToDceWriteChar(DFLT_PRIMHGA,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_FLAME_A_LOW)
    {
      TagToDceWriteChar(DFLT_PRIMLGA,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_FLAME_B_HIGH)
    {
      TagToDceWriteChar(DFLT_PRIMHGB,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_FLAME_B_LOW)
    {
      TagToDceWriteChar(DFLT_PRIMLGB,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_GUARD_HIGH)
    {
      TagToDceWriteChar(DFLT_SECHG,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_GUARD_LOW)
    {
      TagToDceWriteChar(DFLT_SECLG,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CALIBRATION_COUNT)
    {
      TagToDceWriteChar(CALIBCOUNT,(BYTEu)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_OPMLAMPCAL)
    {
      TagToDceWriteInt(OPMLAMPCAL,ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_OPMLEDCAL)
    {
      TagToDceWriteInt(OPMLEDCAL,ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_TESTLAMPCALGU)
    {
      TagToDceWriteInt(TESTLAMPCALGU,ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_TESTLAMPCALFL)
    {
      TagToDceWriteInt(TESTLAMPCALFL,ptrsrvmsg->SetCalibrationMsg.Value);
    }
    /* further extension */
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_FLMACALFLG)
    {
      TagToDceWriteChar(FLMACALFLG,(unsigned char)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_FLMBCALFLG)
    {
      TagToDceWriteChar(FLMBCALFLG,(unsigned char)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_GDCALFLG)
    {
      TagToDceWriteChar(GDCALFLG,(unsigned char)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_SUNSCALFLG)
    {
      __no_operation();
    }
    /* TagToDceWriteInt(,ptrsrvmsg->SetCalibrationMsg.Value); */
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_TESTDRIVE)
    {
      TagToDceWriteChar(TESTDRIVE,(unsigned char)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_OPMDRIVE)
    {
      TagToDceWriteChar(OPMDRIVE,(unsigned char)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_LEDDRIVE)
    {
      TagToDceWriteChar(OPMLEDDRIVE,(unsigned char)ptrsrvmsg->SetCalibrationMsg.Value);
    }
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_SUN_HIGH)
    {
      __no_operation();
    }
    /* TagToDceWriteInt(,ptrsrvmsg->SetCalibrationMsg.Value); */
    else if (ptrsrvmsg->SetCalibrationMsg.Type == CAL_SUN_LOW)
    {
      __no_operation();
    }
    /* TagToDceWriteInt(,ptrsrvmsg->SetCalibrationMsg.Value); */

    else
    {
      ptrsrvmsg->SetCalibrationRsp.ErrorValue = false;
    }
    /* Update checksum */
    UpdateChecksum();
    /* Send back response */
    SrvReplyPacket(channel,ptrsrvmsg);
    break;

  case SRV_GETCALIBRATION:
    /* Response, Header: */
    ptrsrvmsg->GetCalibrationMsg.Header.Type = SRV_GETCALIBRATION;
    ptrsrvmsg->GetCalibrationMsg.Header.Size =
      (sizeof(SRV_GETCALIBRATION_RSP) - sizeof(SRVHEADER));

    if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_FLAME_A_HIGH)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(DFLT_PRIMHGA);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_FLAME_A_LOW)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(DFLT_PRIMLGA);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_FLAME_B_HIGH)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(DFLT_PRIMHGB);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_FLAME_B_LOW)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(DFLT_PRIMLGB);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_GUARD_HIGH)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(DFLT_SECHG);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_GUARD_LOW)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(DFLT_SECLG);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CALIBRATION_COUNT)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(CALIBCOUNT);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_SUN_HIGH)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = 0U;
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_SUN_LOW)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = 0U;
    }

    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_OPMLAMPCAL)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadInt(OPMLAMPCAL);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_OPMLEDCAL)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadInt(OPMLEDCAL);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_TESTLAMPCALGU)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadInt(TESTLAMPCALGU);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_TESTLAMPCALFL)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadInt(TESTLAMPCALFL);
    }

    /* further extension */
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_FLMACALFLG)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(FLMACALFLG);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_FLMBCALFLG)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(FLMBCALFLG);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_GDCALFLG)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(GDCALFLG);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_SUNSCALFLG)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = 0U;
    }                                             /* TagToConfigReadChar(); */
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_TESTDRIVE)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(TESTDRIVE);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_OPMDRIVE)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(OPMDRIVE);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == CAL_LEDDRIVE)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = TagToConfigReadChar(OPMLEDDRIVE);
    }

    else if (ptrsrvmsg->GetCalibrationMsg.Type == FLAMEAHIGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = CalGetRequestedAnalogCh(REQ_TEMPCALFLAH);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == FLAMELOGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = CalGetRequestedAnalogCh(REQ_TEMPCALFLAL);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == FLAMEBHIGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = CalGetRequestedAnalogCh(REQ_TEMPCALFLBH);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == FLAMEBLOGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = CalGetRequestedAnalogCh(REQ_TEMPCALFLBL);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == GUARDHIGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = CalGetRequestedAnalogCh(REQ_TEMPCALGUH);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == GUARDLOGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = CalGetRequestedAnalogCh(REQ_TEMPCALGUL);
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == SUNHIGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = 0U;
    }
    else if (ptrsrvmsg->GetCalibrationMsg.Type == SUNLOGAINVERIFY)
    {
      ptrsrvmsg->GetCalibrationRsp.Value = 0U;
    }
    else
    {}
    SrvReplyPacket(channel,ptrsrvmsg);
    break;
  case SRV_SETSERVICEADDR:
    /* Read my serial number currently programmed in config */
    TagToConfigReadString(MYSERIALNUM,myserial,SIZEOFMYSERIAL);
    /* compare it with the S/N in the received message */
    if (strncmp((const char *)ptrsrvmsg->SetServiceAddrMsg.MySerialNumber,
                (const char *)myserial, SIZEOFMYSERIAL) == (int)false)
    {
      /*
       * The serial number in the message matches the actual
       * serial number of the detector. Reconfigure the service
       * address
       */
      TagToDceWriteInt(SERVICEADDRESS,
                       ptrsrvmsg->SetServiceAddrMsg.NewAddress & 0x7FFFU);
      /* Update DCE checksum */
      UpdateChecksum();
      /* Send Ack response */
      SrvReplyAck(channel,ptrsrvmsg);
    }
    break;
  case SRV_SETREALTIMESTAMP:
    /* Add real time stamp to FDE log */
    FdeRealTimeStampLog(ptrsrvmsg->SetRealTimeStampMsg.TimeStamp);
    /* only reply if it is not a broadcast message */
    if ((ptrsrvmsg->SetConfigValueMsg.Header.Address & 0x7FFFU) != 0x0000U)
    {
      SrvReplyAck(channel,ptrsrvmsg);
    }
    break;
  case SRV_GETCONFIGVALUE:
    /* Response, Header: */
    ptrsrvmsg->GetConfigValueRsp.Header.Type = SRV_GETCONFIGVALUE;
    ptrsrvmsg->GetConfigValueRsp.Header.Size =
      (sizeof(SRV_GETCONFIGVALUE_RSP) - sizeof(SRVHEADER));
    /* Retreive Configuration value */
    if (ConvertServiceDefToTag(&varchar1,
                               ptrsrvmsg->GetConfigValueMsg.Parameter) == PASS)
    {
      /* Tag found. Is it an BYTE or INT value? */
      TagToSize(&tempchar,varchar1);
      if (tempchar == SIZEOFCHAR)
      {
        /* Byte data */
        ptrsrvmsg->GetConfigValueRsp.Value =
          ((unsigned int)TagToConfigReadChar(varchar1)) & 0x00FFU;

      }
      else if (tempchar == SIZEOFINT)
      {
        /* int data */
        ptrsrvmsg->GetConfigValueRsp.Value =
          TagToConfigReadInt(varchar1);
      }
      else
      {
        /* Type not supported for this message type */
        SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,1U);
        break;
      }
      /* Send the reply */
      SrvReplyPacket(channel,ptrsrvmsg);
    }
    else
    {
      /* Tag lookup failed. Return error */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,0U);
    }
    break;
  case SRV_SETCONFIGVALUE:
    /* Retreive Configuration enum as defined in configuration.h */
    tempchar = ConvertServiceDefToTag(&varchar1,
                                      ptrsrvmsg->SetConfigValueMsg.Parameter);
    /*
     * Check to see if this is to change the Service address.
     * If it is, only carry out the address change (if it is
     * not a broadcast message) OR (if it is a broadcase message
     * and the walk test input is in the 'ACTIVE' state)
     * (note: the reply bit is set so avoid it when checking address)
     */
    if ((ptrsrvmsg->SetConfigValueMsg.Parameter == CNF_SERVICEADDRESS) &&
        ((ptrsrvmsg->SetConfigValueMsg.Header.Address & 0x7FFFU) == 0x0000U) &&
        (g_mux3[WALK_TEST] != WT_RESET) )
    {
      /*
       * Rejected. Walktest is open circuit.
       * Dont even respond with a 'ACK' or 'ERROR' packet
       */
      break;       /* Break out of this case statment. */
    }
    /* Does the selected field exist? */
    if (tempchar == false)
    {
      /* No. Tag lookup failed. Return error */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,0U);
      break;       /* Break out of this case statment. */
    }
    /* Tag found. Is it an BYTE or INT value? */
    TagToSize(&tempchar,varchar1);
    if (tempchar == SIZEOFCHAR)
    {
      /* Byte data */
      tempchar = TagToDceWriteChar(varchar1,
                                   (unsigned char)(ptrsrvmsg->SetConfigValueMsg.Value & 0x00FFU) );
    }
    else if (tempchar == SIZEOFINT)
    {
      /* int data */
      tempchar = TagToDceWriteInt(varchar1,
                                  ptrsrvmsg->SetConfigValueMsg.Value);
    }
    else
    {
      /* Type not supported for this message type */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,1U);
      break;       /* Break out of this case statment. */
    }
    /*
     * We have a valid configuration field. Is the value we are
     * trying to write to that field outside its range limits?
     */
    if (tempchar == false)
    {
      /* Configuration range checking failed */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,2U);
      break;       /* Break out of this case statment. */
    }
    /*
     * Yes, the value is within limits
     * Update DCE checksum
     */
    UpdateChecksum();
    /* Send ACK */
    SrvReplyAck(channel,ptrsrvmsg);
    break;
  case SRV_GETCONFIGSTRING:
    /* Response, Header: */
    ptrsrvmsg->GetConfigStringRsp.Header.Type = SRV_GETCONFIGSTRING;
    ptrsrvmsg->GetConfigStringRsp.Header.Size =
      (sizeof(SRV_GETCONFIGSTRING_RSP) - sizeof(SRVHEADER));
    /* Retreive Configuration value */
    tempchar = ConvertServiceDefToTag(&varchar1,
                                      ptrsrvmsg->GetConfigStringMsg.Parameter);
    /* Check if it is a Known valid string config field */
    if (tempchar == true)
    {
      /* Tag found. Is it a STRING value? */
      TagToSize(&tempchar,varchar1);
      if (tempchar > SIZEOFINT)
      {
        /* Is string data. Get string */
        TagToConfigReadString(varchar1,
                              (char *)ptrsrvmsg->GetConfigStringRsp.Data,STRINGLENGTH);
        /* Send the reply */
        SrvReplyPacket(channel,ptrsrvmsg);
      }
      else
      {
        /* Tag is NOT a string! Fail. */
        SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,1U);
      }
    }
    else
    {
      /* Tag lookup failed. Return error */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,0U);
    }
    break;
  case SRV_SETCONFIGSTRING:
    /* Set string configuration value */
    tempchar = ConvertServiceDefToTag(&varchar1,
                                      ptrsrvmsg->SetConfigStringMsg.Parameter);
    /* Send back response */
    if (tempchar == true)
    {
      /* Tag found. Is it a STRING value? */
      TagToSize(&tempchar,varchar1);
      if (tempchar > SIZEOFINT)
      {
        /* Is string data. Set string */
        TagToDceWriteString(varchar1,
                            (char *)ptrsrvmsg->SetConfigStringMsg.Data,STRINGLENGTH);
        /* Update DCE checksum */
        UpdateChecksum();
        /* Send ACK */
        SrvReplyAck(channel,ptrsrvmsg);
      }
      else
      {
        /* Tag is NOT a string! Fail. */
        SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,1U);
      }
    }
    else
    {
      /* Tag lookup failed. Return error */
      SrvReplyError(channel,ptrsrvmsg, SRVERR_DATAINVALID,0U);
    }
    break;
  default:
    /*
     * Unknown command type. Reply with error passing back
     * the unknown command type
     */
    SrvReplyError(channel,ptrsrvmsg, SRVERR_TYPEUNKNOWN,
                  ptrsrvmsg->Generic.Header.Type);
    break;
  }
  return(0U);
}

/**************************************************//**
 * ProcessIrWalkTestMessage();
 *
 * This is where actual test functions are required to be called, note
 * the functions should be same as under the wired walk test input module under
 * hardwaretest.c file.
 * A.R Mar 2011.
 *******************************************************/
void ProcessIrWalkTestMessage(unsigned char testcommand)
{
  /* Given the command received on the IR, trigger the correct selftest */
  switch (testcommand)
  {
  case WT_IR_BUILTINTEST:
    /*
     * RemoteTestTrigger(WT_SOURCE_IR,WT_BUILTINTEST);
     * if in SMP turn on alarm for 3 sec
     */
    if (GetServiceModeFlag())
    {
      AlarmRelay(true);
      delayms(3000U);
      AlarmRelay(false);
    }
    else
    {
      RemoteTestTrigger(WT_SOURCE_IR,WT_BUILTINTEST);
    }
    break;
  case WT_IR_RESET:
    RemoteTestTrigger(WT_SOURCE_IR,WT_RESET);
    /* printf_P("\n\r IR RESET"); */
    break;
  case WT_IR_OPM:
    RemoteTestTrigger(WT_SOURCE_IR,WT_OPM);
    /* printf_P("\n\r IR WINDOW TEST"); */
    break;
  case WT_IR_RESERVED:
    /*
     * RemoteTestTrigger(WT_SOURCE_IR,WT_RESERVED);
     * printf_P("\n\r IR RESERVED");
     */
    break;
  default:
    break;
  }
}

/**************************************************//**
 * SrvMessageHeaderValidCheck();
 * WHEN ADDING A NEW SERVICE COMMAND, YOU MUST REGISTER
 * THE TYPE AND SIZE HERE. This allows Header Type and
 * Size checking on recipt of header.
 *******************************************************/
unsigned char SrvMessageHeaderValidCheck(unsigned char rxtype,
                                         unsigned char rxsize)
{
  unsigned char retval = false;

  switch (rxtype)
  {
  case SRV_SERVICEMODE:
    if ((sizeof(SRV_SERVICEMODE_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_CLUSTERREPORT:
    if ((sizeof(SRV_CLUSTERREPORT_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_STATUS:
    if ((sizeof(SRV_STATUS_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_CNTSELFTEST:
    if ((sizeof(SRV_CNTSELFTEST_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_VIDEOCONTROL:
    if ((sizeof(SRV_VIDEOCONTROL_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_VIDEOPRINTTEXT:
    if ((sizeof(SRV_VIDEOPRINTTEXT_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_WALKTEST:
    if ((sizeof(SRV_WALKTEST_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_OPMTEST:
    if ((sizeof(SRV_OPMTEST_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_CONTANALOGUEOUT:
    if ((sizeof(SRV_CONTANALOGUEOUT_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_CONTANALOGUEIN:
    if ((sizeof(SRV_CONTANALOGUEIN_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_CONTDIGITALOUT:
    if ((sizeof(SRV_CONTDIGITALOUT_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_CONTDIGITALIN:
    if ((sizeof(SRV_CONTDIGITALIN_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_DSPDIGITALOUT:
    if ((sizeof(SRV_DSPDIGITALOUT_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_VERSION:
    if ((sizeof(SRV_VERSION_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_FDEEXTRACT:
    if ((sizeof(SRV_FDEEXTRACT_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_ACTION:
    if ((sizeof(SRV_ACTION_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETFAULTDATA:
    if ((sizeof(SRV_GETFAULTDATA_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETTEMPERATUREDATA:
    if ((sizeof(SRV_GETTEMPERATUREDATA_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETCONFIGVALUE:
    if ((sizeof(SRV_GETCONFIGVALUE_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETCONFIGVALUE:
    if ((sizeof(SRV_SETCONFIGVALUE_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETCONFIGSTRING:
    if ((sizeof(SRV_GETCONFIGSTRING_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETCONFIGSTRING:
    if ((sizeof(SRV_SETCONFIGSTRING_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETTOPCASEIDENTITY:
    if ((sizeof(SRV_SETTOPCASEIDENTITY_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETCONTROLLERIDENTITY:
    if ((sizeof(SRV_SETCONTROLLERIDENTITY_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETSENSORIDENTITY:
    if ((sizeof(SRV_SETSENSORIDENTITY_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETIDENTITY:
    if ((sizeof(SRV_GETIDENTITY_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETTESTDATA:
    if ((sizeof(SRV_SETTESTDATA_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETTESTDATA:
    if ((sizeof(SRV_GETTESTDATA_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETCALIBRATION:
    if ((sizeof(SRV_SETCALIBRATION_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETCALIBRATION:
    if ((sizeof(SRV_GETCALIBRATION_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_GETACTIVITY:
    if ((sizeof(SRV_GETACTIVITY_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_FLASHUPDATE:
    if ((sizeof(SRV_FLASHUPDATE_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_FLASHAUTHORISE:
    if ((sizeof(SRV_FLASHAUTHORISE_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETSERVICEADDR:
    if ((sizeof(SRV_SETSERVICEADDR_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  case SRV_SETREALTIMESTAMP:
    if ((sizeof(SRV_SETREALTIMESTAMP_MSG) - sizeof(SRVHEADER)) == rxsize)
    {
      retval = true;
    }
    break;
  default:
    break;
  }
  return(retval);
}

/**************************************************//**
 * CheckForAndProcessServicecommand(void)
 *******************************************************/
/* Track which port the service mode message arrives on */
void CheckForAndProcessServicecommand(void)
{
  /* Number of times 'd' is pressed */
  static unsigned char debugcount;
  unsigned char data;


  /* Check to see if there are characters to process on the UART */
/*   while(DataInReceiveBuffer(UART1) == true){ */
  while ((DataInReceiveBuffer(UART0) == true) || (DataInReceiveBuffer(UART1) == true))
  {
    /* Get and process char */
    if (DataInReceiveBuffer(UART1))
    {
      data = ReceiveByte(UART1);
      ServiceSlaveReveiveState(UART1,data);
      /*
       * TransmitByte(UART0,data);
       * delayms(10);
       */
    }
    else
    {
      data = ReceiveByte(UART0);
      ServiceSlaveReveiveState(UART0,data);
    }
    /*
     * Enter the debug menu by typing in call<Ser.No.>
     * call00000000
     */
    switch (debugcount)
    {
    case 0U:
      if ((data == 'c') ||
          ((g_enablechecksum == false) && (data == 'd')))
      {
        debugcount++;
      }
      else
      {
        debugcount = 0U;
      }
      break;
    case 1U:
      if ((data == 'a') ||
          ((g_enablechecksum == false) && (data == 'd')))
      {
        debugcount++;
      }
      else
      {
        debugcount = 0U;
      }
      break;
    case 2U:
      /* If it is a development build, allow the 3 'd's */
      if ((g_enablechecksum == false) && (data == 'd'))
      {
        SetServiceMode(SERVICE_DEBUG_MENU);
        DebugMenu();
        SetServiceMode(SERVICE_OFF);
      }
      if (data == 'l')
      {
        debugcount++;
      }
      else
      {
        debugcount = 0U;
      }
      break;
    case 3U:
      if (data == 'l')
      {
        debugcount++;
      }
      else
      {
        debugcount = 0U;
      }
      break;
    default:
      /* Check the letters entered with the Ser.No. string */
      if ((debugcount < (SIZEOFMYSERIAL + 4U)) && (data == myserialnum[debugcount - 4U]))
      {
        /* The letter matches up. Shift to next byte */
        debugcount++;
        /*
         * if last Ser.No. character in configuration is a space or NULL then
         * it is enough that the first 7 characters are matched...
         * ...skip 8th go to end
         * FTE will make the 8th letter ' ' when there are 7 letter Ser.No.'s
         */
        if ((debugcount >= SIZEOFMYSERIAL)
            && ((myserialnum[4] == 0x20U) || (myserialnum[4] == 0x00U))
            && ((myserialnum[5] == 0x20U) || (myserialnum[5] == 0x00U))
            && ((myserialnum[6] == 0x20U) || (myserialnum[6] == 0x00U))
            && ((myserialnum[7] == 0x20U) || (myserialnum[7] == 0x00U)))
        {

          debugcount = SIZEOFMYSERIAL + 4U;
        }
        else if ((debugcount >= SIZEOFMYSERIAL + 1U)
                 && ((myserialnum[5] == 0x20U) || (myserialnum[5] == 0x00U))
                 && ((myserialnum[6] == 0x20U) || (myserialnum[6] == 0x00U))
                 && ((myserialnum[7] == 0x20U) || (myserialnum[7] == 0x00U)))
        {

          debugcount = SIZEOFMYSERIAL + 4U;
        }
        else if ((debugcount >= SIZEOFMYSERIAL + 2U)
                 && ((myserialnum[6] == 0x20U) || (myserialnum[6] == 0x00U))
                 && ((myserialnum[7] == 0x20U) || (myserialnum[7] == 0x00U)))
        {

          debugcount = SIZEOFMYSERIAL + 4U;
        }
        else if ((debugcount >= SIZEOFMYSERIAL + 3U)
                 && ((myserialnum[7] == 0x20U) || (myserialnum[7] == 0x00U)))
        {

          debugcount = SIZEOFMYSERIAL + 4U;
        }
        else
        {}

      }
      else
      {
        debugcount = 0U;
      }
      /* Have we checked every letter of the Ser.No. string? */
      if (debugcount == SIZEOFMYSERIAL + 4U)
      {
        {
          /* Yes!!! */
          SetServiceMode(SERVICE_DEBUG_MENU);
/*             FdeConfigurationLog(CNFLOG_SERVICEMODE,ServiceModeFlag,FieldDataExtract); */
          /* Enter the menu */
          DebugMenu();
          /* reset the state machine */
          debugcount = 0U;
          SetServiceMode(SERVICE_OFF);
/*
 * TBD
 *               FdeConfigurationLog(CNFLOG_SERVICEMODE,ServiceModeFlag,FieldDataExtract);
 */
        }
      }
      break;
    }
    KickWdt();
  }

  /* Check to see if there are characters to process on the Ir Link */
  while (IrDataInReceiveBuffer(PCM) == true)
  {
    /* Get and process data */
    data = ReceivePcmByte();
    ServiceSlaveReveiveState(PCM,data);
    KickWdt();
  }

}

/**************************************************//**
 * ResetServiceModeInactivityTimer();
 * Reset counters on recipt of message
 *******************************************************/
void ResetServiceModeInactivityTimer(void)
{
  lastservicemessage = GetSecondsSincePowerUp();
}

/**************************************************//**
 * ModifyServiceModeTimeout();
 * 'Setter' to update the service mode time out period
 *******************************************************/
void ModifyServiceModeTimeout(unsigned int timeout)
{
  serviceinactivity = timeout;
}

/**************************************************//**
 * EnterServiceModeOnRequest();
 * // Called in main loop. Detector will run this while loop for the duration of
 * // service mode
 *******************************************************/
void EnterServiceModeOnRequest(void)
{
  /* Record the time when the request to enter service mode was received */
  if (GetServiceModeFlag())
  {
    ResetServiceModeInactivityTimer();
  }
  /* Stay here while in service mode */
  while (GetServiceModeFlag())
  {
    /* Process any service mode messages if present */
    CheckForAndProcessServicecommand();
    /* Check the inactivity timeout */
    if ((GetSecondsSincePowerUp() - lastservicemessage) > serviceinactivity)
    {
      /* No message has been received after specifid timeout. Exit service mode */
      SetServiceMode(SERVICE_OFF);     /* forces an exit of this while() statment. */
      /*
       * Log the fact this detector was sitting on the roof in service mode
       * for longer than it should have been!
       */
      FdeConfigurationLog(CNFLOG_SERVICEINACTIVITY,0U,0U);
    }

    /* update MX digital read May20,2013 */
    if (PersonalityType() == MX_IFACE_MODE)
    {
      MxDigitalOutputRead();
    }
    /* Check for valid calibration */
    CalibrationCountCheck();
    
    /* Kick the old dog */
    KickWdt();
  }
}

/**************************************************//**
 * ServiceSlaveReveiveState()
 *
 * IR walk test input val processing.
 *******************************************************/
void ServiceSlaveReveiveState(unsigned char port, unsigned char rxchar)
{
  /* Incoming and outgoing message buffer */
  static SRVPROTOCOLMSG srvmsg;
  /* Byte access to message buffer */
  static unsigned char *dataptr;
  static unsigned char bytecount;
  static unsigned short calchecksum, rxchecksum;
  static unsigned char currentstate = SRV_RESET;
  unsigned char nextstate = 0U;
  /*
   * static unsigned int xy=0;
   * Based on state, process incoming chars
   */
  switch (currentstate)
  {
  case SRV_RESET:
    /*
     * Allow unsigned char byte access to SRVPROTOCOLMSG
     * data type. Also resets buffer pointer to beginning
     */
    dataptr = (unsigned char *) &srvmsg;
  case SRV_IDLE:
    /* Wait for an escape char to be received */
    if (rxchar == 0x1BU)
    {
      nextstate = SRV_SOF;
    }
    else if ((rxchar == 0x02U) && (port == PCM))
    {
      /* The T300 Walk test tool has sent first byte marker */
      nextstate = SRVWTT_SOF;
    }
    else
    {
      nextstate = SRV_IDLE;
      /* nextstate=SRVWTT_SOF;//remove */
    }
    break;

  case SRV_SOF:
    /* Next char must be a <SOF> */
    if (rxchar == 0x02U)
    {
      /* Yes, Now start receiving message */
      bytecount = 0U;
      calchecksum = 0U;
      rxchecksum = 0U;
      nextstate = SRV_HEADER;
    }
    else
    {
      /* Byte not recognised. Back to Idle. */
      nextstate = SRV_IDLE;
    }
    break;

  case SRVWTT_SOF:
    /* printf_P("hurry %X\r",rxchar);//remove */
    if ((rxchar == 0x2FU) && (port == PCM))
    {
      /* Valid SOF has been received from T300 WTT */
      nextstate = SRVWTT_CMD;
    }
    else
    {
      ProcessIrWalkTestMessage(WT_RESERVED);
      nextstate = SRV_IDLE;
    }
    break;

  case SRVWTT_CMD:

    /* next receive the command byte. Store it in */
    bytecount = rxchar;
    nextstate = SRVWTT_CMDINV;
    break;

  case SRVWTT_CMDINV:
    /*
     * Last byte. This should be the inverse of
     * the command byte.
     */
    if (rxchar == (bytecount ^ 0xFFU))
    {
      /* Pass command onto handler */
      ProcessIrWalkTestMessage(bytecount);
    }
    else
    {
      ProcessIrWalkTestMessage(WT_RESERVED);
    }
    nextstate = SRV_RESET;
    break;

  case SRV_HEADER:
    /* Receiving Header. Copy into message buf. */
    *dataptr = rxchar;
    /* Calculate checksum */
    calchecksum = g_sprog_crc16_table[(calchecksum >> 8U) ^ rxchar] ^
                  (calchecksum << 8U);
    /* Move pointer to next byte */
    *dataptr++;
    bytecount++;
    /* Is this the last byte of the header? */
    if (bytecount < sizeof(SRVHEADER))
    {
      /* No, More of the header to go */
      nextstate = SRV_HEADER;
    }
    else
    {
      /*
       * The number of bytes that makeup a header have been
       * read in. Is the header a valid Type and size.
       * (Note no checksum verification has taken place yet)
       */
      if (SrvMessageHeaderValidCheck(srvmsg.Generic.Header.Type,
                                     srvmsg.Generic.Header.Size) == true)
      {
        /*
         * Recognised Type and expected Size
         * Is there data attached to header?
         */
        if (srvmsg.Generic.Header.Size != 0U)
        {
          /* Yes, next byte is data */
          bytecount = 0U;
          nextstate = SRV_DATA;
        }
        else
        {
          /* No, next byte to follow is the checksum */
          nextstate = SRV_CHKSUM_LOW;
        }
      }
      else
      {
        /* Type and size are not recognised. Reject & reset */
        nextstate = SRV_RESET;
      }
    }
    break;

  case SRV_DATA:
    /* Receiving Data. Copy into message buf. */
    *dataptr = rxchar;
    /* Calculate checksum */
    calchecksum = g_sprog_crc16_table[(calchecksum >> 8U) ^ rxchar] ^
                  (calchecksum << 8U);
    /* Move pointer to next byte */
    *dataptr++;
    bytecount++;
    /* Have we have got the last of the data? */
    if (bytecount < srvmsg.Generic.Header.Size)
    {
      /* no, more to come. */
      nextstate = SRV_DATA;
    }
    else
    {
      /* Finished geting data. Next byte should be chksum */
      nextstate = SRV_CHKSUM_LOW;
    }
    break;

  case SRV_CHKSUM_LOW:
    /* First byte of the checksum (LSB) */
    rxchecksum = ((unsigned int)rxchar) & 0x00FFU;
    nextstate = SRV_CHKSUM_HI;
    break;

  case SRV_CHKSUM_HI:
    /* Second byte of checksum (MSB) */
    rxchecksum |= (((unsigned int)rxchar << 8U) & 0xFF00U);
    /*
     * Message received. Does checksum match?
     * delayms(100);
     * printf_P("%X,%X",rxchecksum,calchecksum);
     */
    if (rxchecksum == calchecksum)   /* enable it ATTENTION !!! */
    {     /*
           * if(true){
           * Checksum match
           */
      nextstate = SRV_ESC_EOF;
    }
    else
    {
      /*
       * Checksum mismatch!
       * Wait for RTS of transmit terminal to switch back to RX
       */
      delayms(100U);
/*
 * DISABLE ALL FDE PRINTOUTS TO THE EXTERNAL CONFIGURATION PORT
 * AS IT INTERFERES WITH THE SERVICE PROTOCOL COMMUNICATIONS
 * WHEN TRYING TO TALK TO A SINGLE DETECTOR ON A BUS
 * ONLY DO THIS FOR RELEASE VERSIONS
 */
      if (g_enablechecksum == false)
      {
        /*
         *  printf_P("Chksum mismatch: CalcCs=0x%04X, Rxchk=0x%04X\r\n",
         * calchecksum,rxchecksum);
         */
      }
      /* Reject current message */
      nextstate = SRV_RESET;
    }
    break;

  case SRV_ESC_EOF:
    /* This byte should be <ESC> */
    if (rxchar == 0x1BU)
    {
      /* Esc received */
      nextstate = SRV_EOF;
    }
    else
    {
      nextstate = SRV_RESET;
    }
    break;

  case SRV_EOF:
    /* This byte should be <EOF> */
    if (rxchar == 0x03U)
    {
      /*
       * EOF received.
       * Is the message address to us? Is it a broadcast?
       */
      if ((srvmsg.Generic.Header.Address == 0x0000U) ||
          /*
           * or directally addressed to us? (Note: this also
           * ensures the 'Reply bit' is not set)
           */
          (srvmsg.Generic.Header.Address ==
           (TagToConfigReadInt(SERVICEADDRESS) & 0x7FFFU)) )
      {
        /*
         * Addressed to me. Proccess and respond to message.
         * Wait for AGC to settle on the configuration tool
         * before a response is transmitted
         */
        if (port == PCM)
        {
          /*
           * Service command arrived over IR
           * Blink LED to indicate message has been received
           */
          // PORTA ^= (0x01U << PA1);       /* Toggle LED */
          AlarmLEDToggle();
          /* Wait for AGC of the transmiters reciever to settle */
          __delay_cycles(36840U);
          // PORTA ^= (0x01U << PA1);       /* Toggle LED */
          AlarmLEDToggle();
        }
        else
        {
          /*
           * RS485, Allow master a few ms to switch RTS to
           * to receive mode. Added to allow terminal programs
           * that dont release the RTS line a few bytes after
           * they should do!
           */
          // PORTA ^= (0x01U << PA1);       /* Toggle LED */
          AlarmLEDToggle();
          __delay_cycles(36840U);
          // PORTA ^= (0x01U << PA1);       /* Toggle LED */
          AlarmLEDToggle();
        }
        /* Proccess and respond to message. */
        ProcessServiceMessage(port,&srvmsg);
        /* Reset inactivity timeout counter */
        ResetServiceModeInactivityTimer();
      }
    }
    /* Reset, ready to receive next message. */
    nextstate = SRV_RESET;
    break;

  default:
    nextstate = SRV_RESET;
    break;
  }
  currentstate = nextstate;
}

/**************************************************//**
 * SrvReplyAck();
 *******************************************************/
void SrvReplyAck(unsigned char channel, SRVPROTOCOLMSG *ptrsrvmsg)
{
  ptrsrvmsg->AckRsp.Header.Type = SRV_ACK;
  ptrsrvmsg->AckRsp.Header.Size = 0U;
  SrvReplyPacket(channel,ptrsrvmsg);
}

/**************************************************//**
 * SrvReplyError();
 *******************************************************/
void SrvReplyError(unsigned char channel, SRVPROTOCOLMSG *ptrsrvmsg,
                   unsigned char errorcode, unsigned char errorvalue)
{
  ptrsrvmsg->ErrRsp.Header.Type = SRV_ERR;
  ptrsrvmsg->ErrRsp.Header.Size = (sizeof(SRV_ERR_RSP) - sizeof(SRVHEADER));
  ptrsrvmsg->ErrRsp.ErrorCode = errorcode;
  ptrsrvmsg->ErrRsp.ErrorValue = errorvalue;
  SrvReplyPacket(channel,ptrsrvmsg);
}

/**************************************************//**
 * SrvReplyPacket();
 *******************************************************/
void SrvReplyPacket(unsigned char channel, SRVPROTOCOLMSG *ptrsrvmsg)
{
  unsigned char *ptrdata;
  unsigned char bodybyte,bytecount,preamblecnt = 0U;
  unsigned short checksum = 0U;
  /* Init local pointer for access to SRVPROTOCOLMSG ptrsrvmsg */
  ptrdata = (unsigned char *) ptrsrvmsg;
  /* <SOF> */
  if (channel == UART1)
  {
    /* it means packet is for HART add 5 0xff as preamble */
    for (preamblecnt = 0U; preamblecnt < PREAMBLECOUNT; preamblecnt++)
    {
      TransmitByte(channel,0xFFU);
    }
  }
  TransmitByte(channel,0x1BU);
  TransmitByte(channel,0x02U);
  /* Message body (Header and data) */
  for (bytecount = 0U; bytecount < (ptrsrvmsg->Generic.Header.Size + sizeof(SRVHEADER)); bytecount++)
  {
    bodybyte = *ptrdata++;
    TransmitByte(channel,bodybyte);
    checksum = g_sprog_crc16_table[(checksum >> 8U) ^ bodybyte] ^ (checksum << 8U);
  }
  /* Send calculate checksum */
  TransmitByte(channel,(unsigned char)checksum);
  TransmitByte(channel,(unsigned char)(checksum >> 8U));
  /* <EOF> */
  TransmitByte(channel,0x1BU);
  TransmitByte(channel,0x03U);
  /*
   * following if statement looks useless but this is
   * ment for PC400 else you will struggle with so many
   * time out dialog boxes. AR Mar21,2013.
   */
  if ((channel == UART0) && (versionreadflg == false) && (flashupdate == false))
  {
    TransmitByte(channel,0xFFU);
    TransmitByte(channel,0xFFU);
  }
  /* set inrts of HART modem for rcv anyway */
  PORTG |= (0x01U << PG1);
}

/**************************************************//**
 * GetCurrentStatusBits();
 *******************************************************/
unsigned int GetCurrentStatusBits(void)
{
  unsigned int statusbitscurrentval = 0U;
  if (GetSettlingState())
  {
    statusbitscurrentval |= SRVST_FLAMEPYROSETTLE;
    statusbitscurrentval |= SRVST_GUARDSETTLE;
    statusbitscurrentval |= SRVST_FLGUREADYFORCALIB;
  }
  /* alarm &pre alarm */
  if (GetDetectorStatus()->AlarmState == STATUS_ALARM)
  {
    statusbitscurrentval |= SRVST_ALARM;
  }
  if (GetPreAlarmState())
  {
    statusbitscurrentval |= SRVST_PREALARM;
  }
  /* faults */
  if (g_HWFaultState == HARDWARE_FAULT)
  {
    statusbitscurrentval |= SRVST_HARDWAREFAULT;
  }
  if (g_HWFaultState == WINDOWSCLEANLINESS)
  {
    statusbitscurrentval |= SRVST_OPMFAULT;
  }
  /* add tracking maxi minima in algo later */
  statusbitscurrentval |= 0U;       /* SRVST_FLAME_SATURATION; */
  statusbitscurrentval |= 0U;       /* SRVST_GUARD_SATURATION; */
  /* calibration process indicator */
  if (GetCalibrationInProgressStatus())
  {
    statusbitscurrentval |= SRVST_GUARD_SATURATION;
  }

  return(statusbitscurrentval);
}

/**************************************************//**
 * GetSmpAlmReq();
 *******************************************************/
unsigned char GetSmpAlmReq(void)
{
  return(smpalmtestflg);
}

/**************************************************//**
 * GetConvPcbMode();
 *******************************************************/
unsigned char GetConvPcbMode(void)
{
  return(convpcbmode);
}

/**************************************************//**
 * GetOverRideStatus();
 *******************************************************/
unsigned char GetOverRideStatus(void)
{
  return(ovrideflg);
}

/**************************************************//**
 * GetMxSmpStatus();
 *****************************************************/
unsigned char GetMxSmpStatus(void)
{
  return(mxsmpstatus);
}

/**************************************************//**
 * GetConvModeSmpStatus();
 *****************************************************/
unsigned char GetConvModeSmpStatus(void)
{
 return(convmodesmpstatus);
}
/* ////////////EOF/////////////// */
