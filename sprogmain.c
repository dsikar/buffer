/*****************************************************************************
 *
 * Copyright (C) 1996-1998 Atmel Corporation
 *
 * File          : sprogmain.c
 * Compiler      : IAR EWAAVR 2.26c
 * Output size   :
 * Created       : 16-jun-99
 * Modified      : 5-jan-2000
 * Modified      : 17-April-2002 by AS
 * Modified      : 1-March-2002 by RAA
 * Modified      : 14-March-2013 by A Rana & T James
 *
 * Support mail  : avr@atmel.com
 *
 * Description   : This Program allows an AVR with bootloader capabilities to
 *                 Read/write its own Flash/EEprom. To enter Programming mode
 *                 PD4 is checked. If this pin is pulled low, programming mode
 *                 is entered. If not, normal execution is done from $0000
 *                 "reset" vector in Application area.
 *                 The PD4 pin should be pulled HIGH by an external Pull-up
 *                 resistor.
 *
 * Other info    : The defines.h file must be set up for individual devices
 *                 using the excel-sheet preprocessor
 *
 *                 In general the linker file should always be verifyed to match
 *                 the used part's boot address and size. Note that memory size
 *                 is specified in bytes in the linker file.
 *
 * FV400     info: This file is part of a 'sprog_' prefixed set of files that
 *                 are designed to link into the never read while write boot
 *                 sector (above address 1E000 for the ATMEGA1281).

 *                 These files are compiled with the application code below
 *                 address 1E000 and it is important that they do not share
 *                 code, therefore extra care has to be taken when using the
 *                 compiler's C library. If C library functions cannot be
 *                 avoided then choose the optimimise for speed rather than
 *                 size option for file properties: this should cause the
 *                 compiler to duplicate the library code within this module.
 *                 When recompiling any of the sprog_ prefixed .c files read
 *                 map file to make sure any of the sprog_**** functions do
 *                 not refer to code segments below the sprog boot address.
 *                 The 'REQUIRE' entries that do appear below this address are
 *                 a mystery that can be ignored for the time being as no code
 *                 space is being allocated.
 *                 Avoid depending on initialised variables, these would be
 *                 initialised from the INITTAB segment which is shared by
 *                 application code and will disappear after the flash memory
 *                 is erased.
                  For FV400, Do the full speed optimization for only sproglibrary.c,
                  sprogmain.c,sprogserial.c. For Sprogmcrconly.c just select
                  speed-none option, to avoid farflash truble.
                  Under preprocessor tab for defined symbols use
                  LOCATION="SPROG_CODE". MAR 14 2013 AR.
 ****************************************************************************/
#define ENABLE_BIT_DEFINITIONS
#include <inavr.h>
#include <yvals.h>

#define FDEBLOCKSIZE 128UL

#include "sproglibrary.h"
#include "sprogdefines.h"
#include "sprogserial.h"
#include "sprogassembly.h"
#include "sprogcrconly.h"

#ifndef LOCATION
#define SEGMENT_LOCATION
#else
#define SEGMENT_LOCATION @ LOCATION
#endif

//#pragma location=0x1FFFE                    // fixed location at end of Flash
//__farflash const unsigned int BootChecksum=0x9167;//0xC02F;

#pragma location=0x1FE00                    // fixed location after sprog_crc16_table
__farflash const char sprog_authorisestring[]="Authorise copying FDE to ATMEL FLASH";

#pragma location=0x1FC00                    // fixed location in code space
__farflash const unsigned short g_sprog_crc16_table[]={
  0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50a5U, 0x60c6U, 0x70e7U, 0x8108U,
  0x9129U, 0xa14aU, 0xb16bU, 0xc18cU, 0xd1adU, 0xe1ceU, 0xf1efU, 0x1231U, 0x0210U,
  0x3273U, 0x2252U, 0x52b5U, 0x4294U, 0x72f7U, 0x62d6U, 0x9339U, 0x8318U, 0xb37bU,
  0xa35aU, 0xd3bdU, 0xc39cU, 0xf3ffU, 0xe3deU, 0x2462U, 0x3443U, 0x0420U, 0x1401U,
  0x64e6U, 0x74c7U, 0x44a4U, 0x5485U, 0xa56aU, 0xb54bU, 0x8528U, 0x9509U, 0xe5eeU,
  0xf5cfU, 0xc5acU, 0xd58dU, 0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76d7U, 0x66f6U,
  0x5695U, 0x46b4U, 0xb75bU, 0xa77aU, 0x9719U, 0x8738U, 0xf7dfU, 0xe7feU, 0xd79dU,
  0xc7bcU, 0x48c4U, 0x58e5U, 0x6886U, 0x78a7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
  0xc9ccU, 0xd9edU, 0xe98eU, 0xf9afU, 0x8948U, 0x9969U, 0xa90aU, 0xb92bU, 0x5af5U,
  0x4ad4U, 0x7ab7U, 0x6a96U, 0x1a71U, 0x0a50U, 0x3a33U, 0x2a12U, 0xdbfdU, 0xcbdcU,
  0xfbbfU, 0xeb9eU, 0x9b79U, 0x8b58U, 0xbb3bU, 0xab1aU, 0x6ca6U, 0x7c87U, 0x4ce4U,
  0x5cc5U, 0x2c22U, 0x3c03U, 0x0c60U, 0x1c41U, 0xedaeU, 0xfd8fU, 0xcdecU, 0xddcdU,
  0xad2aU, 0xbd0bU, 0x8d68U, 0x9d49U, 0x7e97U, 0x6eb6U, 0x5ed5U, 0x4ef4U, 0x3e13U,
  0x2e32U, 0x1e51U, 0x0e70U, 0xff9fU, 0xefbeU, 0xdfddU, 0xcffcU, 0xbf1bU, 0xaf3aU,
  0x9f59U, 0x8f78U, 0x9188U, 0x81a9U, 0xb1caU, 0xa1ebU, 0xd10cU, 0xc12dU, 0xf14eU,
  0xe16fU, 0x1080U, 0x00a1U, 0x30c2U, 0x20e3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
  0x83b9U, 0x9398U, 0xa3fbU, 0xb3daU, 0xc33dU, 0xd31cU, 0xe37fU, 0xf35eU, 0x02b1U,
  0x1290U, 0x22f3U, 0x32d2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U, 0xb5eaU, 0xa5cbU,
  0x95a8U, 0x8589U, 0xf56eU, 0xe54fU, 0xd52cU, 0xc50dU, 0x34e2U, 0x24c3U, 0x14a0U,
  0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U, 0xa7dbU, 0xb7faU, 0x8799U, 0x97b8U,
  0xe75fU, 0xf77eU, 0xc71dU, 0xd73cU, 0x26d3U, 0x36f2U, 0x0691U, 0x16b0U, 0x6657U,
  0x7676U, 0x4615U, 0x5634U, 0xd94cU, 0xc96dU, 0xf90eU, 0xe92fU, 0x99c8U, 0x89e9U,
  0xb98aU, 0xa9abU, 0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18c0U, 0x08e1U, 0x3882U,
  0x28a3U, 0xcb7dU, 0xdb5cU, 0xeb3fU, 0xfb1eU, 0x8bf9U, 0x9bd8U, 0xabbbU, 0xbb9aU,
  0x4a75U, 0x5a54U, 0x6a37U, 0x7a16U, 0x0af1U, 0x1ad0U, 0x2ab3U, 0x3a92U, 0xfd2eU,
  0xed0fU, 0xdd6cU, 0xcd4dU, 0xbdaaU, 0xad8bU, 0x9de8U, 0x8dc9U, 0x7c26U, 0x6c07U,
  0x5c64U, 0x4c45U, 0x3ca2U, 0x2c83U, 0x1ce0U, 0x0cc1U, 0xef1fU, 0xff3eU, 0xcf5dU,
  0xdf7cU, 0xaf9bU, 0xbfbaU, 0x8fd9U, 0x9ff8U, 0x6e17U, 0x7e36U, 0x4e55U, 0x5e74U,
  0x2e93U, 0x3eb2U, 0x0ed1U, 0x1ef0U
};

unsigned int sprog_CalculateFlashChecksum(unsigned long start, unsigned long end);
void sprog_update_application(void);
unsigned char sprog_atoi(unsigned char);
unsigned char sprog_itoa(unsigned char);
void sprog_InitaliseOutputs(void);

// LED logic switch
#define bLED 0U

__C_task void sprog_main(void) SEGMENT_LOCATION
{
  unsigned int intval,address,i;
  unsigned char val, idletime;
  unsigned char buf[24];
  // Set up function pointer to RESET vector
  void (*funcptr)( void )=0x0000;
  //set fault relay off
  DDRB|=(0x01U<<PB4);
  PORTB&=~(0x01U<<PB4);
  // Reset the EEPROM address to pointer to an unused location.
  // Set registers to known setting
  EECR=0U;
  EEARH=0U;
  EEARL=0U;
  EEDR=0U;
  // initalise peripherals
  DDRA|=(0x01U<<PA5);   //watch dog pin as o/p
  sprog_KickWdt();                           // kick watchdog
  sprog_SpiInitPort();                       // initialize SPI port (all devices deselected)
  sprog_InitaliseOutputs();                  // Set outputs to a known state
  sprog_SpiInitPort();                       // initialize SPI port (all devices deselected)
  sprog_inituart();                          // initialize uart

  // Timer 2 -> Timer 0 settings close to main application except that interrupt not used and the
  // output compare register set to give full scale counting.
  // Set timer 2 mode and prescalar to divide by 1024
  // With a 4.6864MHz crystal, this gives 3600Hz
  TCCR2A=0U;
  TCCR2B=0U;
  //set prescaler
  TCCR2B|=(0x01U<<CS20);
  TCCR2B|=(0x01U<<CS21);
  TCCR2B|=(0x01U<<CS22);
  // Set the clear timer on compare mode (CTC)
  TCCR2A|=(0x01U<<WGM21);
  TCCR2A&=~(0x01U<<WGM20);
  // Clear the output compare register - let timer go to ff
  OCR2A=0xffU;
  TCNT2=0U;

  idletime=0U;
  sprog_KickWdt();
  val=sprog_recchar();
  if(!val)
    {
      val='b';
    }

  while(idletime < 2U)                       // 2 seconds maximum idle time
    {
      sprog_KickWdt();                      // kick watchdog
      if(val)
        {
          idletime=0U;
          if(val=='a')                      // set address
            {
              val=sprog_recchar();
              address=(unsigned int)((sprog_atoi(val)<<12U));
              sprog_KickWdt();              // kick watchdog
              val=sprog_recchar();
              address|=(unsigned int)((sprog_atoi(val)<<8U));
              sprog_KickWdt();              // kick watchdog
              val=sprog_recchar();
              address|=(unsigned int)((sprog_atoi(val)<<4U));
              sprog_KickWdt();              // kick watchdog
              val=sprog_recchar();
              address|=(sprog_atoi(val));
              sprog_KickWdt();              // kick watchdog
              if(address > 0x7FFFU)
                {
                  RAMPZ=(1U<<RAMPZ0);
                }
              else
                {
                  RAMPZ=0U;
                }
              address=address<<1U;           //convert from word address to byte address
              sprog_sendchar('\r');
            }

          else if(val=='b')                 // Return software identifier and checksum
            {
              intval=sprog_CalculateFlashChecksum(0x0001E000U,0x0001FFFEU);
              sprog_sendchar('\r');
              sprog_sendchar('\n');
              sprog_sendchar('\n');
              sprog_sendchar('\n');
              sprog_sendchar('F');
              sprog_sendchar('V');
              sprog_sendchar('4');
              sprog_sendchar('0');
              sprog_sendchar('0');
              sprog_sendchar('B');
              sprog_sendchar('O');
              sprog_sendchar('O');
              sprog_sendchar('T');
              sprog_sendchar(' ');
              sprog_sendchar('c');
              sprog_sendchar('h');
              sprog_sendchar('e');
              sprog_sendchar('c');
              sprog_sendchar('k');
              sprog_sendchar('s');
              sprog_sendchar('u');
              sprog_sendchar('m');
              sprog_sendchar(' ');
              val=sprog_itoa((unsigned char)(intval>>12));
              sprog_sendchar(val);
              val=sprog_itoa((unsigned char)(intval>>8));
              sprog_sendchar(val);
              val=sprog_itoa((unsigned char)(intval>>4));
              sprog_sendchar(val);
              val=sprog_itoa((unsigned char)intval);
              sprog_sendchar(val);
              sprog_sendchar(' ');
              // After the checksum test and message print, turn
              // red LED off
              if(bLED) 
              {
               PORTA&=~(0x01U<<PA1);
              }
              sprog_KickWdt();
              // Print out results of boot checksum test.
              if(intval!=GetBootFlashCrc())
                {
                  sprog_sendchar('N');
                  sprog_sendchar('O');
                  sprog_sendchar('K');
                  sprog_sendchar('\r');
                  sprog_sendchar('\n');
                  funcptr(); // Jump to Reset vector 0x0000 in Application Section
                }
              sprog_sendchar('O');
              sprog_sendchar('K');
              sprog_sendchar('\r');
              sprog_sendchar('\n');
            }

          else if(val=='c')                 // read lock bits
            {
              RAMPZ=0U;
              val=(unsigned char)sprog_read_program_memory(0x0001U,0x09U);
              sprog_sendchar(sprog_itoa(val>>4U));
              sprog_sendchar(sprog_itoa(val));
              sprog_sendchar('\r');
            }

          else if(val=='d')
            {
              val=sprog_recchar();
              if(val=='d')
                {
                  sprog_KickWdt();          // kick watchdog
                  val=sprog_recchar();
                  if(val=='d')              // Read all of program memory in pause mode - use any key to continue, esc to quit.
                    {
                      // Line up address for start of new line
                      address&=0xFFF0;
                      for(; val!=0x1bU; address+=2U)
                        {
                          sprog_KickWdt();  // kick watchdog
                          // format output '\r0x0001F800   FF FF FF FF  FF FF FF FF  FF FF FF FF  FF FF FF FF   ................'
                          if((address&0xFU)==0U)
                            {
                              sprog_sendchar('\r');
                              sprog_sendchar('\n');
                              sprog_sendchar('0');
                              sprog_sendchar('x');
                              sprog_sendchar('0');
                              sprog_sendchar('0');
                              sprog_sendchar('0');
                              sprog_sendchar(sprog_itoa((unsigned char)(RAMPZ&(1U<<RAMPZ0))));
                              sprog_sendchar(sprog_itoa((unsigned char)(address>>12)));
                              sprog_sendchar(sprog_itoa((unsigned char)(address>>8)));
                              sprog_sendchar(sprog_itoa((unsigned char)(address>>4)));
                              sprog_sendchar('0');
                              sprog_sendchar(0x20U);
                              sprog_sendchar(0x20U);
                              sprog_sendchar(0x20U);
                            }

                          intval=sprog_read_program_memory(address,0x00U);
                          buf[(address&0xFU)]=(char)intval;     // LSB;
                          buf[(address&0xFU)+1U]=(char)(intval>>8U); // MSB;

                          sprog_sendchar(sprog_itoa((unsigned char)(intval>>4)));
                          sprog_sendchar(sprog_itoa((unsigned char)intval));
                          sprog_sendchar(0x20U);
                          sprog_sendchar(sprog_itoa((unsigned char)(intval>>12)));
                          sprog_sendchar(sprog_itoa((unsigned char)(intval>>8)));
                          sprog_sendchar(0x20U);

                          if((address&0xFU)==0xEU)
                            {
                              sprog_sendchar(0x20U);
                              sprog_sendchar(0x20U);
                              for(i=0U; i<16U; i++)
                                {
                                  if((buf[i]<0x20U) || (buf[i]>0x80U))
                                    {
                                      sprog_sendchar('.');
                                    }
                                  else
                                    {
                                      sprog_sendchar(buf[i]);
                                    }
                                }
                              if((address&0x3FFU)==0x3FEU)
                                {
                                  val=0U;
                                  while((val==0U)&&(idletime<6U)) // maximum pause of 6 seconds
                                    {
                                      val=sprog_recchar();
                                      ++idletime;
                                    }
                                  idletime=0U;
                                }
                            }
                          // SPM uses Z pointer but the pointer is only 16bit
                          if(address == 0xFFFEU) // higher location it require the use of RAMPZ
                            {
                              if(RAMPZ)
                                {
                                  val=0x1bU;
                                } // (esc)
                              RAMPZ++;      // RAMPZ has to incremented into upper 64k of FLASH memory
                              RAMPZ&=(1U<<RAMPZ0); // Mask away any non used bits
                            }
                        }
                      sprog_sendchar('\r');
                      sprog_sendchar('\n');
                    }
                }
            }

          else if(val=='e')                 // read fuse bits
            {
              RAMPZ=0U;
              val=(unsigned char)sprog_read_program_memory(0x0000U,0x09U);
              sprog_sendchar(sprog_itoa(val>>4U));
              sprog_sendchar(sprog_itoa(val));
              sprog_sendchar('\r');
            }

          else if(val=='f')                 // read high fuse bits
            {
              RAMPZ=0U;
              val=(unsigned char)sprog_read_program_memory(0x0003U,0x09U);
              sprog_sendchar(sprog_itoa(val>>4U));
              sprog_sendchar(sprog_itoa(val));
              sprog_sendchar('\r');
            }
          else
            {}

        }
      else
        {
          ++idletime;
        }
      val=sprog_recchar();
      if((val<'a')||(val>'f'))
        {
          val=0U;
        }
    }
  sprog_KickWdt();                          // kick watchdog
  sprog_update_application();
  // Jump to Reset vector 0x0000 in Application Section
  funcptr();
}

/********************************************************************//**
   sprog_CalculateFlashChecksum()
   In this application, the xlink uses the crc16 algorithm to calculate the
   checksum and save it in the variable __checksum which is located in the
   CHECKSUM segment. By comparing the result of the fast_crc16() function
   with the checksum value calculated by xlink. Checks if
   the code on the hardware changes unexpectly during runtime.

   Calculate FLASH checksum betweem two addresses. To do a full
   scan of the FLASH, use start=0x00000000L, end=0x0001FFFFL
 *************************************************************************/
unsigned int sprog_CalculateFlashChecksum(unsigned long start, unsigned long end) SEGMENT_LOCATION {
  unsigned long address;
  unsigned int checksum, intval;
  unsigned char old_rampz, rampz;
  old_rampz=RAMPZ&(1U<<RAMPZ0);               // Mask away any non used bits
  rampz=0U;
  checksum=0U;
  //<SPROG_VERIFY> Software acceptance test: Force invalid checksum
  //start+=0x10;
  // Scan through Flash
  for(address=start; address<end; address++){
      if((address > 0xFFFFU)&&(rampz == 0U))
        {
          rampz=(1U<<RAMPZ0);     //RAMPZ has to incremented into upper 64k segment
        }
      // Stop RAMPZ going adrift!
      RAMPZ=rampz;
      intval=sprog_read_program_memory(((unsigned int)address&0xFFFEU),0x00U);
      // For even addresses checksum the LSB. For odd addresses put MSB in LSB and
      // checksum the LSB.
      if(address&1U)
        {
          intval>>=8U;
        }  // MSB to LSB;
      checksum=g_sprog_crc16_table[(checksum>>8U)^(char)(intval&0xFFU)]^(checksum<<8U);
      sprog_KickWdt();
    }
  RAMPZ=old_rampz;
  return(checksum);
}

/********************************************************************//**
   sprog_CalculateFdeChecksum()
   Calculate FDE checksum betweem two addresses. To do a full
   scan of the FDE, use start=0x00000000L, end=0x0001FFFFL
 *************************************************************************/
unsigned int sprog_CalculateFdeChecksum(unsigned long start, unsigned long end) SEGMENT_LOCATION {
  unsigned long address;
  unsigned int checksum;
  checksum=0U;
  // Set the start address pointer
  sprog_GetFlashByteAddressed(start);
  // Scan through FDE
  for(address=start; address<end; address++){
      checksum=g_sprog_crc16_table[(checksum>>8U)^
                                 sprog_SpiGetByte()]^(checksum<<8U);
      sprog_KickWdt();
    }
  sprog_GetFlashMultiByteEnd();
  return(checksum);
}

/********************************************************************//**
   sprog_update_application()
   Checks for FDE authorise string if ok then Gets the Checksum from
   FDE and compaires the with locally calulated one. If matches then
   ready for programming.

   Erases flash page by page then writes flash in same manner. After
   whole process re enable RWW and clear RAMPZ.
 *************************************************************************/
void sprog_update_application(void) SEGMENT_LOCATION
{
  unsigned long readaddress;
  unsigned int addressb,data,i;
  unsigned int checksum,currentchsum;
  unsigned char index, ldata,xlsb,ymsb;
  unsigned char status, retval;
  // Test the FDE is fitted and responding.
  sprog_SpiSelectDevice(SPI_FDE_EEPROM);
  sprog_SpiSendByte(RDSR);
  status=sprog_SpiGetByte();
  sprog_SpiSelectDevice(SPI_NONE);
  if(status!=0xFFU)
    {
      sprog_FlashWriteProtect(1U); //true      // protect EEPROM where new application may be
      sprog_KickWdt();
      // check the 'Ready to copy FDE to ATMEL flash' string
      // at the beginning of the last FDEBLOCK.
      // Set read address
      readaddress=(LASTSECTOR*SECTORSIZE)-FDEBLOCKSIZE;
      status=1U; //pass
      sprog_GetFlashByteAddressed(readaddress);
      for(index=0U; index<sizeof(sprog_authorisestring); index++){
          retval=sprog_SpiGetByte(); //sprog_GetFlashByteAddressed(readaddress++);
          if(sprog_authorisestring[index]!=retval)
            {
              status=0U; //fail
            }
          sprog_KickWdt();                    // kick watchdog
        }
      if(!status)
        {
          sprog_sendchar('\r');
          sprog_sendchar('S');
          sprog_sendchar('T');
          sprog_sendchar('R');
          sprog_sendchar('F');
          sprog_sendchar('L');
        }
      else //pass
        {
          // Check FDE Ready to Program Flash
          // Set read address
          readaddress=(LASTSECTOR*SECTORSIZE)-2UL;
          // Get the checksum - lsb
          sprog_GetFlashByteAddressed(readaddress);
          xlsb=sprog_SpiGetByte();
          //msb
          ymsb=sprog_SpiGetByte();
          checksum=(unsigned int)((ymsb<<8U)|(xlsb));

          currentchsum=sprog_CalculateFdeChecksum(0U,APP_END);

          if(checksum==currentchsum)
            {
              // Erase Flash
              RAMPZ=0x00U;             //SPM uses Z pointer but the pointer is only 16bit and
                                       //can only address up to 64Kbytes FLASH to address higher
                                       //location it require the use of RAMPZ
              for(addressb=0U; ((addressb<((unsigned int)(APP_END)))|(RAMPZ == 0x00U)); addressb+=PAGESIZE) //feb 07 2013
                {
                  sprog_KickWdt();    // kick watchdog
                  //Perform page erase
                  sprog_write_page(addressb,(1U<<PGERS)+(1U<<SPMEN));
                  //write done re enable and clear RAMPZ
                  //sprog_write_page(addressb, (1<<REENABLE_RWW_BIT) + (1<<SPMEN));
                  if(addressb >=(0xFFFFU-PAGESIZE)) //Last section on lower 64k segment is erased
                    {
                      if(!(RAMPZ&0x01U))
                        {
                          RAMPZ=(1U<<RAMPZ0); //RAMPZ has to incremented into upper 64k segment
                        }
                    }
                } //for
                  //clear RAMPZ
              sprog_write_page(addressb, (1U<<REENABLE_RWW_BIT)+(1U<<SPMEN));
              RAMPZ=0x00U;
              // Set the FDE to Update Flash
              readaddress=0U;
              sprog_GetFlashByteAddressed(readaddress++);
              // Copy from FDE to Flash page by page
              for(addressb=0U; ((addressb<((unsigned int)(APP_END)))|(RAMPZ == 0x00U)); addressb+=PAGESIZE) //feb 07 2013
                {
                  for(i=0U; i<PAGESIZE; i+=2U)
                    {
                      sprog_KickWdt(); // kick watchdog
                      // get FDE, low byte
                      ldata=sprog_SpiGetByte();
                      // get FDE, high byte
                      data=(unsigned int)(ldata|(sprog_SpiGetByte()<<8U));
                      if(data!=0xFFFFU)
                        {
                          sprog_fill_temp_buffer(data,(addressb+i)); //call asm routine.
                        }
                    }
                  //Perform page write
                  sprog_write_page(addressb, (1U<<PGWRT)+(1U<<SPMEN));
                  //write done re enable and clear RAMPZ
                  //sprog_write_page(addressb, (1<<REENABLE_RWW_BIT) + (1<<SPMEN));

                  if(addressb >=(0xFFFFU-PAGESIZE)) //Last section on lower 64k segment is written
                    {
                      if(!(RAMPZ&0x01U))
                        {
                          RAMPZ=(1U<<RAMPZ0);  //RAMPZ has to incremented into upper 64k segment
                        }
                    }
                  // Toggle fire LED to show program is running
                  if(!(addressb&PAGESIZE))
                    {
                      if(bLED)
                      {
                       PORTA^=(0x01U<<PA1); 
                      }
                    }
                } //for
                  //clear RAMP
              sprog_write_page(addressb, (1U<<REENABLE_RWW_BIT)+(1U<<SPMEN));
              RAMPZ=0x00U;
              //Turn off ALM LED
              if(bLED)
              {
               PORTA&=~(0x01U<<PA1); 
              }

              // Verify Flash with FDE
              if(checksum==sprog_CalculateFlashChecksum(0U,APP_END))
                {
                  //set fault relay on
                  PORTB|=(0x01U<<PB4);
                  sprog_KickWdt();
                  sprog_sendchar('\r');
                  sprog_sendchar('E');
                  sprog_sendchar('R');
                  sprog_sendchar('A');
                  sprog_sendchar('S');
                  sprog_sendchar('I');
                  sprog_sendchar('N');
                  sprog_sendchar('G');
                  sprog_sendchar(' ');
                  sprog_sendchar('F');
                  sprog_sendchar('D');
                  sprog_sendchar('E');
                  // Clear FDE
                  sprog_EraseFlash();
                  //Kick WDT
                  sprog_KickWdt();
                }
              else
                {
                  // set fault output relay...
                  PORTB&=~(0x01U<<PB4);
                  sprog_sendchar('\r');
                  sprog_sendchar('E');
                  sprog_sendchar('R');
                  sprog_sendchar('R');
                  sprog_sendchar('O');
                  sprog_sendchar('R');
                  sprog_sendchar(' ');
                  sprog_sendchar('F');
                  sprog_sendchar('L');
                  // ...and flash ALM LED waiting for detector to be turned off
                  while(1)
                    {
                      if(TCNT2>=240U) //COUNTSPERTICK
                        {
                          TCNT2=0U;
                          ++i;
                          if(i&0x6U) // on
                            {
                              // Used for Video, not applicable for FV421i
                              // PORTA|=(0x01U<<PA1);
                            }
                          else // off
                            {
                              // PORTA&=~(0x01U<<PA1);
                            }
                          sprog_KickWdt(); // kick watchdog
                        }
                    }
                }

            }
          else
            {
              sprog_sendchar('\r');
              sprog_sendchar('C');
              sprog_sendchar('H');
              sprog_sendchar('K');
              sprog_sendchar('F');
              sprog_sendchar('L');
            }
        }
    }
}

//Generally transmits letters in upper case and expects command letters
//in lower case, avoiding chance of RS485 echo leading to a continuous
//loop.

unsigned char sprog_atoi(unsigned char a) SEGMENT_LOCATION
{
  unsigned char i;
  i=0U;
  if(((a>='0')&&(a<='9'))||((a>='a')&&(a<='f')))
    {
      i=a-0x30U;
      if(i > 0x30U)
        {
          i-=0x27U;
        }
    }
  return(i);
}

unsigned char sprog_itoa(unsigned char i) SEGMENT_LOCATION
{
  unsigned char a;
  a=i&0x0fU;
  a+=0x30U;
  if(a > 0x39U)
    {
      a+=0x07U;
    }

  return(a);
}

void sprog_InitaliseOutputs(void) SEGMENT_LOCATION {
  // Initalise LED ports as outputs
  DDRA|=(0x01U<<DDA1);
  // Fault LED on
  //PORTA &= ~(0x01<<PA4);

  // Initalise relays as outputs
  DDRB|=(0x01U<<DDB4);
  // signal 'in fault' for the fault relay (open)
  PORTB&=~(0x01U<<PB4);
  // Signal 'not in alarm' for the alarm relay
  DDRD|=(0x01U<<DDD0);
  PORTD&=~(0x01U<<PD0);

  // Initalise window heater as output
  //DDRG |= (0x01<<DDG4);
  // Turn off heater
  //PORTG &= ~(0x01<<PG4);

  // UART0 to receive
  // Define UART0 RS485 control lines as outputs
  DDRB|=(0x01U<<DDE2)|(0x01U<<DDE6);
  // Switch UART->485 IC to receive mode
  PORTE&=~(0x01U<<PE2);
  PORTE&=~(0x01U<<PE6);

  // Initalise IR TX LED as output
  //DDRB |= (0x01<<DDB5);
  // LED off
  //PORTB &= ~(0x01<<PB5);

  // Set the 4-20 mA to 0 mA
  //sprog_SpiSelectDevice(SPI_DAC);
  //sprog_SpiSendByte(0x00);
  //sprog_SpiSendByte(0x00);
  sprog_SpiSelectDevice(SPI_NONE);

  // Put the video subsystem in a known state.
  // Firstly power up the 5V subsystem
  DDRG|=(0x01U<<DDG5);
  PORTG&=~(0x01U<<PG5);
  // Wait for 5 ms for powerup to complete
  // Alarm LED on
  // Used for Video, not applicable for FV421i
  // PORTA|=(0x01U<<PA1);
//tbd

  __delay_cycles(18432U);

  // Configure SPI port for communication with the video IC
  SPCR=0x00U;
  SPCR=(SPI_921KHZ&0x03U)|                 // Set clock rate
        ((MODE3&0x03U)<<CPHA)|         // Select clock phase & polarity
        (0x01U<<MSTR)|                // Master operation
        ((LSB_FIRST&0x01U)<<DORD)|    // Set data order
        (0x01U<<SPE);                 // Enable SPI port

  // Perform a software reset of the video IC
  sprog_SpiSelectDevice(SPI_VIDEO);
  // SW reset on
  sprog_SpiSendByte(0xC1U);
  sprog_SpiSendByte(0x00U);
  // SW reset off
  sprog_SpiSendByte(0xC0U);
  sprog_SpiSendByte(0x00U);
  sprog_SpiSelectDevice(SPI_NONE);
  // Power down the internal video camera (pin19 - SEPOUT)
  sprog_SpiSelectDevice(SPI_VIDEO);
  sprog_SpiSendByte(0xF8U);  // 0xFC will power on
  sprog_SpiSendByte(0x00U);
  sprog_SpiSelectDevice(SPI_NONE);
}

//==========================FV 300 REF from Atmel==============================
/*
   __C_task void sprog_main_ref(void) SEGMENT_LOCATION
   {
    unsigned int intval,address,data,i;
    unsigned char val, ldata, idletime;
    unsigned char buf[24];

    void (*funcptr)( void ) = 0x0000;        // Set up function pointer to RESET vector
    sprog_KickWdt();                         // kick watchdog
    sprog_inituart();                        // initialize uart
    sprog_SpiInitPort();                     // initialize SPI port (all devices deselected)

    // Timer 2 settings close to main application except that interrupt not used and the
    // output compare register set to give full scale counting.
    // Set timer 2 mode and prescalar to divide by 1024
    // With a 4.6864MHz crystal, this gives 3600Hz
    TCCR2 |= (0x05<<CS20);
    // Set the clear timer on compare mode (CTC)
    TCCR2 |= (0x01<<WGM21);
    TCCR2 &= ~(0x01<<WGM20);
    // Clear the output compare register - let timer go to ff
    OCR2=(0xff);

    TCNT2 = 0;
    idletime = 0;

    val=sprog_recchar();
    if(!val)
      val='S';

    while(idletime < 30)                    // 2 seconds maximum idle time
    {
      sprog_KickWdt();                      // kick watchdog
      if(val)
      {
        idletime=0;
        if(val=='a')                        //Autoincrement?
        {
          address++;
          sprog_sendchar((char)(address>>8));
          sprog_sendchar((char)(address));
        }

        else if(val=='A')                   //write address
        {
          address=sprog_getchar();                //read address 8 MSB
          address=(address<<8)|sprog_getchar();

          if(address >= 0x7FFF)
            RAMPZ = (1<<RAMPZ0);
          else
            RAMPZ = 0;

          address = address << 1;            //convert from word address to byte address
          sprog_sendchar('\r');
        }

        else if(val=='c')                   //Write program memory, low byte
        {
          ldata=sprog_getchar();
          sprog_sendchar('\r');
        }

        else if(val== 'C')                  //Write program memory, high byte
        {
          data=ldata | (sprog_getchar() << 8);
          sprog_KickWdt();                      // kick watchdog
          sprog_fill_temp_buffer(data,(address)); //call asm routine.
          address += 2;
          sprog_sendchar('\r');
        }

        else if (val == 'd')
        {
          EEARL = address;
          EEARH = (address >> 8);
          address++;
          EECR |= (1<<EERE);
          sprog_sendchar(EEDR);
        }

        else if (val == 'D')
        {
          EEARL = address;
          EEARH = (address >> 8);
          address++;
          EEDR = sprog_getchar();
          sprog_KickWdt();                      // kick watchdog
          EECR |= (1<<EEMWE);
          EECR |= (1<<EEWE);
          while (EECR & (1<<EEWE))
            ;
          sprog_sendchar('\r');
        }

        else if(val=='e')                   //Chip erase
        {
          RAMPZ = 0x00;                //SPM uses Z pointer but the pointer is only 16bit and
                                       //can only address up to 64Kbytes FLASH to address higher
                                       //location it require the use of RAMPZ

          for(address=0;((address < (unsigned int)(APP_END&0xFFFF))|(RAMPZ == 0x00));address += PAGESIZE)
          {
            sprog_KickWdt();                // kick watchdog
            sprog_write_page(address,(1<<PGERS) + (1<<SPMEN));              //Perform page erase
            sprog_write_page(address, (1<<REENABLE_RWW_BIT) + (1<<SPMEN));  //Re-enable the RWW section

            if(address >=(0xFFFF-PAGESIZE)) //Last section on lower 64k segment is erased
               RAMPZ = (1<<RAMPZ0);         //RAMPZ has to incremented into upper 64k segment
          }
          RAMPZ = 0x00;                     //Clear RAMPZ pointer

          sprog_sendchar('\r');
        }

        else if(val=='F')                   // read fuse bits
        {
          RAMPZ = 0;
          sprog_sendchar(sprog_read_program_memory(0x0000,0x09));
        }

        else if(val=='l')                   // write lockbits
        {
          sprog_write_lock_bits(sprog_getchar());
          sprog_sendchar('\r');
        }

        else if(val== 'm')                  // write page
        {
          sprog_write_page(address, (1<<PGWRT) + (1<<SPMEN));       //Perform page write
          sprog_write_page(address, (1<<REENABLE_RWW_BIT) + (1<<SPMEN));  //Re-enable the RWW section

          sprog_sendchar('\r');
        }

        else if(val=='N')                   // read high fuse bits
        {
          RAMPZ = 0;
          sprog_sendchar(sprog_read_program_memory(0x0003,0x09));
        }

        else if(val=='O')                   // Read all of program memory in pause mode - use any key to continue, esc to quit.
        {
          // Line up address for start of new line
          address &= 0xFFF0;
          for(; val!=0x1b; address+=2)
          {
             sprog_KickWdt();               // kick watchdog
             // format output '\r0x0001F800   FF FF FF FF  FF FF FF FF  FF FF FF FF  FF FF FF FF   ................'
             if((address & 0xF)==0)
             {
                sprog_sendchar('\r');
                sprog_sendchar('\n');
                sprog_sendchar('0');
                sprog_sendchar('x');
                sprog_sendchar('0');
                sprog_sendchar('0');
                sprog_sendchar('0');
                val = (char)(RAMPZ & (1<<RAMPZ0));
                val += 0x30;
                sprog_sendchar(val);
                val = (char)(address>>12);
                val += 0x30;
                if(val>0x39)
                   val += 7;
                sprog_sendchar(val);
                val = (char)((address>>8)&0xF);
                val += 0x30;
                if(val>0x39)
                   val += 7;
                sprog_sendchar(val);
                val = (char)((address>>4)&0xF);
                val += 0x30;
                if(val>0x39)
                   val += 7;
                sprog_sendchar(val);
                val = (char)(address &0xF);
                val += 0x30;
                if(val>0x39)
                   val += 7;
                sprog_sendchar(val);
                sprog_sendchar(0x20);
                sprog_sendchar(0x20);
                sprog_sendchar(0x20);
             }

             intval = sprog_read_program_memory(address,0x00);
             buf[(address & 0xF)] = (char)intval;           // LSB;
             buf[(address & 0xF)+1] = (char)(intval>>8);    // MSB;

             val = (char)((intval>>4)&0xF);
             val += 0x30;
             if(val>0x39)
                val += 7;
             sprog_sendchar(val);
             val = (char)(intval &0xF);
             val += 0x30;
             if(val>0x39)
                val += 7;
             sprog_sendchar(val);
             sprog_sendchar(0x20);
             val = (char)(intval>>12);
             val += 0x30;
             if(val>0x39)
                val += 7;
             sprog_sendchar(val);
             val = (char)((intval>>8)&0xF);
             val += 0x30;
             if(val>0x39)
                val += 7;
             sprog_sendchar(val);
             sprog_sendchar(0x20);

             if((address & 0xF)==0xE)
             {
                sprog_sendchar(0x20);
                sprog_sendchar(0x20);
                for(i=0;i<16;i++)
                {
                   if(buf[i]<0x20 || buf[i]>0x80)
                      sprog_sendchar('.');
                   else
                      sprog_sendchar(buf[i]);
                }
                if((address & 0x3FF)==0x3FE)
                   val=sprog_getchar();
             }
                                              // SPM uses Z pointer but the pointer is only 16bit
             if(address == 0xFFFE)            // higher location it require the use of RAMPZ
             {
               if(RAMPZ)
                  val=0x1b;                   // (esc)
               RAMPZ++;                       // RAMPZ has to incremented into upper 64k of FLASH memory
               RAMPZ &=(1<<RAMPZ0);           // Mask away any non used bits
             }
          }
          sprog_sendchar('\r');
          sprog_sendchar('\n');
        }

        else if(val=='r')                   // read lock bits
        {
          RAMPZ = 0;
          sprog_sendchar(sprog_read_program_memory(0x0001,0x09));
        }

        else if(val=='R')                   //Read program memory
        {
          intval = sprog_read_program_memory(address,0x00);
          sprog_sendchar((char)(intval>>8));      //send MSB
          sprog_sendchar((char)intval);           //send LSB

          address+=2;                       //SPM uses Z pointer but the pointer is only 16bit
          if(address == 0x0000)             //higher location it require the use of RAMPZ
          {
            RAMPZ++;                       //RAMPZ has to incremented into upper 64k of FLASH memory
            RAMPZ &=(1<<RAMPZ0);           //Mask away any non used bits
          }
        }

        else if (val=='S')                  // Return software identifier
        {
          sprog_sendchar('\r');
          sprog_sendchar('S');
          sprog_sendchar('3');
          sprog_sendchar('0');
          sprog_sendchar('0');
          sprog_sendchar('B');
          sprog_sendchar('O');
          sprog_sendchar('O');
          sprog_sendchar('T');
          sprog_sendchar('\r');
        }

        else if (val=='V')                  // Return Software Version
        {
          sprog_sendchar('1');
          sprog_sendchar('0');
        }

        else if (val=='z')                  //Change z pointer
        {
          RAMPZ++;                          //RAMPZ toggle, upper/lower 64k of FLASH memory
          RAMPZ &=(1<<RAMPZ0);              //Mask away any non used bits
          sprog_sendchar('\r');
        }

        else if(val!=0x1b)                  // if not esc
        {
          sprog_sendchar('?');
        }
      }
      else
   ++idletime;
      val=sprog_recchar();
    }
    sprog_KickWdt();                        // kick watchdog
    sprog_update_application();
    funcptr();       // Jump to Reset vector 0x0000 in Application Section
   }
 */



//==========================   E - O - F  =============================
