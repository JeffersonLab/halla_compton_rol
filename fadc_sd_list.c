/*************************************************************************
 *
 *  test_fadc.c - Library of routines for readout and buffering of
 *                     events using a JLAB Trigger Interface V3 (TI) with
 *                     a Linux VME controller in CODA 3.0.
 *
 *                     This for a TI in Master Mode controlling multiple ROCs
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* Define TI Type (TI_MASTER or TI_SLAVE) */
#define TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x4A

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "fadcLib.h"        /* library of FADC250 routines */
#include "sdLib.h"

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 4

/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     1
/* Address of first fADC250 */
#define FADC_ADDR 0x400000
/* Increment address to find next fADC250 */
#define FADC_INCR 0x080000

#define FADC_WINDOW_LAT    500
#define FADC_WINDOW_WIDTH  460
#define FADC_MODE           10

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;
int useSD = 1;  /* Decision to use SD module */

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int ifa, stat;
  unsigned short iflag;

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;


  /*****************
   *   TI SETUP
   *****************/

  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
  tiSetTriggerSource(TI_TRIGGER_FPTRG);

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set the sync delay width to 0x40*32 = 2.048us */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Init the SD library so we can figure out how to initialize the FADC */
  if(useSD)
    stat = sdInit(0);

  if((stat != OK) || (useSD == 0))
  {
  	if(stat != OK)
		{
	  	printf("%s: WARNING: sdInit() returned %d\n",
		 __func__, stat);
		}
    printf("\t Will try to use SDC at 0xea00\n");
    useSD = 0;

    /* FADC Initialization flags */
    iflag = 0xea00; /* SDC Board address */
    iflag |= FA_INIT_EXT_SYNCRESET;  /* Front panel sync-reset */
    iflag |= FA_INIT_FP_TRIG;  /* Front Panel Input trigger source */
    iflag |= FA_INIT_FP_CLKSRC;  /* Internal 250MHz Clock source */
  }
  else
  {
  	printf("\t Will try to use SD in Switch Slot\n");

    /* FADC Initialization flags */
    iflag = 0; /* NO SDC */
    iflag |= (1<<0);  /* VXS sync-reset */
    iflag |= FA_INIT_VXS_TRIG;  /* VXS trigger source */
    iflag |= FA_INIT_VXS_CLKSRC;  /* VXS 250MHz Clock source */
  }

  /*****************
   *   FADC SETUP
   *****************/

  fadcA32Base = 0x09000000; /* Set the Base address of the FADC block data registers */
  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);
  vmeSetQuietFlag(0);

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      faEnableBusError(faSlot(ifa));

      /* Set input DAC level */
      faSetDAC(faSlot(ifa), 3250, 0);

      /*  Set All channel thresholds to 150 */
      faSetThreshold(faSlot(ifa), 150, 0xffff);

      /*********************************************************************************
       * faSetProcMode(int id, int pmode, unsigned int PL, unsigned int PTW,
       *    int NSB, unsigned int NSA, unsigned int NP,
       *    unsigned int NPED, unsigned int MAXPED, unsigned int NSAT);
       *
       *  id    : fADC250 Slot number
       *  pmode : Processing Mode
       *          9 - Pulse Parameter (ped, sum, time)
       *         10 - Debug Mode (9 + Raw Samples)
       *    PL : Window Latency
       *   PTW : Window Width

       *   NSB : Number of samples before pulse over threshold
       *   NSA : Number of samples after pulse over threshold
       *    NP : Number of pulses processed per window
       *  NPED : Number of samples to sum for pedestal
       *MAXPED : Maximum value of sample to be included in pedestal sum
       *  NSAT : Number of consecutive samples over threshold for valid pulse
       */
      faSetProcMode(faSlot(ifa),
		    FADC_MODE,   /* Processing Mode */
		    FADC_WINDOW_LAT, /* PL */
		    FADC_WINDOW_WIDTH,  /* PTW */
		    3,   /* NSB */
		    6,   /* NSA */
		    1,   /* NP */
		    0);   /* bank (not used) */

    }

  if (useSD)
    {
      /***************************************
       *   SD SETUP
       ***************************************/
      sdSetActiveVmeSlots(faScanMask()); /* tell the SD where to find the FADCs */
      sdStatus();
    }
  else
    {
      faSDC_Status(0);
    }

  /* Prints status for FADC and TI*/
  faGStatus(0);
  tiStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  unsigned short iflag;
  int ifa, stat;
  int islot;


  /* Unlock the VME Mutex */
  vmeBusUnlock();

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  /* Program/Init VME Modules Here */
  for(ifa=0; ifa < nfadc; ifa++)
    {
      faSoftReset(faSlot(ifa),0);
      faResetTriggerCount(faSlot(ifa));
    }

  tiStatus(0);
  faGStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int islot;

  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

  /* Enable/Set Block Level on modules, if needed, here */
  faGSetBlockLevel(blockLevel);

  if(FADC_MODE == 9)
    {
      /*
	 Block header
	 + blocklevel *
	 (Event Header + TriggerTime * 2 + MAX_PULSES * MAX_CHAN * 3 * PulseParameter
	 + Event Trailer)
	 + Block Trailer
	 + 3 * FillerWord
	 = 1 + blocklevel*(1 + 2 + 4 * 16 * 3 * 1 + 1) + 1 + 3
      */
      MAXFADCWORDS = 5 + blockLevel * (196) + 4; /* 4 = fudge */
    }
  else /* FADC_MODE == 10 */
    {
      /*
	All from 9, and raw window data (2 samples per word)
	5 + blocklevel * (196 + WindowWidth / 2);
      */
//      MAXFADCWORDS = 5 + blockLevel * (196 + (FADC_WINDOW_WIDTH >> 1) ) + 4; /* 4 = fudge */
			MAXFADCWORDS = 4000;
    }

  /*  Enable FADC */
  faGEnable(0, 0);

  /* Interrupts/Polling enabled after conclusion of rocGo() */

  /* Example: How to start internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
  tiSetRandomTrigger(1,0x7);
#elif defined (INTFIXEDPULSER)
  /* Enable fixed rate with period (ns) 120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
     - Generated 1000 times */
  tiSoftTrig(1,1000,700,0);
#endif
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{
  int islot;

  /* Example: How to stop internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Disable random trigger */
  tiDisableRandomTrigger();
#elif defined (INTFIXEDPULSER)
  /* Disable Fixed Rate trigger */
  tiSoftTrig(1,0,700,0);
#endif

  /* FADC Disable */
  faGDisable(0);

  /* FADC Event status - Is all data read out */
  faGStatus(0);
  /* Prints status of TI */
  tiStatus(0);

  sdStatus(0);
  /* Reset FADC
     faGReset(0); */

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, islot;
  int ifa, nwords, blockError, stat, dCnt, len=0, idata;
  unsigned int val;
  unsigned int *start;
  unsigned int datascan, scanmask, roCount;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  roCount = tiGetIntCount(); //Get the TI trigger count

  /* Readout the trigger block from the TI
     Trigger Block MUST be reaodut first */
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("No TI Trigger data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }

  /* EXAMPLE: How to open a bank (type=5) and add data words by hand */
  BANKOPEN(5,BT_UI4,blockLevel);
  *dma_dabufp++ = tiGetIntCount();
  *dma_dabufp++ = 0xdead;
  *dma_dabufp++ = 0xcebaf111;
  *dma_dabufp++ = 0xcebaf222;
  BANKCLOSE;

  /* fADC250 Readout */
  BANKOPEN(3,BT_UI4,blockLevel);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBready(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  nwords = faReadBlock(faSlot(ifa), dma_dabufp, MAXFADCWORDS, 1);

	  /* Check for ERROR in block read */
	  blockError = faGetBlockError(1);

	  if(blockError)
	    {
	      printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		     faSlot(ifa), roCount, nwords);

	      if(nwords > 0)
		dma_dabufp += nwords;
	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  BANKCLOSE;

  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int islot=0;

  printf("%s: Reset all FADCs\n",__FUNCTION__);

  faGReset(1);
}
