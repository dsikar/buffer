/***************************************************************************
 * @file  defs.h  
 * @brief       System Non-Volatile Memory Store Objects - Definitions
 *  Used On     FV400
 ***************************************************************************/

#ifndef INC_DEFS
#define INC_DEFS

#define JTAGS_OFF true
#ifndef JTAGS_OFF
   #define JTAGS_ON true
#endif

#define ADC_PRESCALAR 4U
#define false 0U
#define true  1U

typedef unsigned char   BYTEu;     /*  8 bit  unsigned byte */
typedef unsigned short  INT16u;    /* 16 bit  unsigned integer */
typedef unsigned long   INT32u;    /* 32 bit  unsigned integer */
typedef signed char     BYTEs;     /*  8 bit  signed byte    */
typedef signed short    INT16s;    /* 16 bit  signed integer   */
typedef signed long     INT32s;    /* 32 bit  signed integer   */

#define FACTORYCLEANVALUE 300U
#define FDEBLOCKSIZE 128U
#define SIZEOF_VERSION           6U
#define SIZEOF_BUILDDATE        18U
#define SIZEOF_BUILDCOMMENT     18U
#define SIZEOF_SHAREDCOMMENT   (SIZEOF_BUILDCOMMENT/2)
#define OFFSET_SHAREDCOMMENT    SIZEOF_SHAREDCOMMENT
#define SIZEOFUNITSERIAL        19U 
#define SIZEOFARRAYSERIAL       16U 
#define SENSORIDENTITY_RESERVED  6U 
#define TEST_RESERVED  8U
#define SIZEOFDATE 6U

/*/ Controller board packet transfer state machine exit conditions*/
enum{
   SPIRET_UNKNOWN,
   SPIRET_TIMEOUT,
   SPIRET_DSPTXFLUSHFAIL,
   SPIRET_RETRIESEXCEEDED,
   SPIRET_EXCEPTIONERROR,
   SPIRET_MESSAGERECEIVED,
   SPIRET_RESERVED,
   SPIRET_ERRORRSPRECEIVED,
   SPIRET_RESPSIZEMISSMATCH
};

enum{ 
   TEST_SENSORINITIAL,
   TEST_SENSORSECOND,
   TEST_CONTROLLER,
   TEST_TOPCASEFUNC,
   TEST_ENVIRONMENTAL,
   TEST_POTTING,
   /*/ <- Insert new TestType here*/
   TEST_ENDMARKER
};

enum { 
  ERROR_NONE, 
  ERROR_ARRAYACTIVE, 
  ERROR_COMMS 
};

enum{
   ERROR = -1,
   start,
   restart,
   get_key,
   wait_for_RS485,
   check_string,
   working,
   finished
};

#define PASS 1U
#define FAIL 0U

#define LOW 0U
#define HIGH 1U

#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
#define BITVAL(x,y) (((x)>>(y)) & 1)

enum{
   UART0,
   UART1,/*/in use for HART*/
   PCM
};

/*/ Base address of compiled in text string*/
#define TEXTLOCATION 0x100L

/*/ UPDATE relevant fields before software release
// Use build comment for the software stock code number*/

/*/ Software Revision ----->      <-- (SIZEOF_VERSION chars) */
/*/#define CONT_VERSION      "99.997"  */
#define PRODUCT_NAME      "FV421 - debug Interrupt LED" 
#define CONT_VERSION      "0.028" /* to be IS SIL 2 */



/*/ DO NOT INCLUDE COMMAS IN THESE COMMENTS AS THEY CAUSE PROBLEMS WHEN CONVERTING
// LOGFILES TO CSV FORMAT
// Only these num chars -->                  <-- (SIZEOF_BUILDCOMMENT 18 chars)  */
#define CONT_BUILDCOMMENT "136-212-292       " /* set something */

/*/ Company name ---------->                       <-- (24 chars inc. \0)  */
#define CONT_COMPANY      "Tyco Fire Products     "

/*/ Product name ---------->                       <-- (24 chars inc. \0)  */
#define CONT_PRODUCT      "FLAMEVision FV421i     "


#endif   
