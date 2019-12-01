/*************************************************************************
 *
 *  vtpCompton_list.c - Compton readout list.
 *             Configure: fADC250, 2 VETROC, TI, SD, VTP
 *             Readout:   fADC250, 2 VETROC, TI
 *
 *     TI delivers accepted Triggers, Clocks, and SyncReset to
 *       fADC250, VETROC, SD, and VTP
 *
 *     SD combines BUSY from fADC250, VETROC and sends to TI (SWB_BUSY)
 *
 *     VTP BUSY sent directly to TI (SWA_BUSY)
 *
 *     VTP (configured and readout with it's own ROC) will generate
 *       triggers using the VETROC (and possible fADC250) data.  This
 *       output will go into TS Input #1 (???)
 *
 *     fADC250 Must be in slot 3.
 *     VETROCs must be in slot 13 and 14.
 *
 * Hall A Compton DAQ Upgrade Crew:
 *   Alex Camsonne
 *   Robert Michaels
 *   Iris Halilovic
 *   Jinlong Zhang
 *   Ben Raydo
 *   Bryan Moffit
 *
 *************************************************************************/

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, external triggers */
#define TI_ADDR  0 /* Auto initialize (search for TI by slot */

/* VETROC definitions */
#define USE_VETROC
#define MAXVETROCDATA 1000
#define VETROC_SLOT 13					/* slot of first vetroc */
#define VETROC_SLOT_INCR 1			/* slot increment */
#define NVETROC	2								/* number of vetrocs used */

/* FADC definitions */
#define USE_FADC
#define NFADC     1							/* number of fadcs used */
#define FADC_ADDR (3<<19)			/* address of first fADC250 */
#define FADC_INCR (1<<19)			/* increment address to find next fADC250 */
#define FADC_WINDOW_LAT    500
#define FADC_WINDOW_WIDTH  500
#define FADC_MODE        		 1

/* VTP definitions */
#define USE_VTP

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x4A

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 1

/* Include */
#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "vetrocLib.h"      /* VETROC library */
#include "fadcLib.h"        /* library of FADC250 routines */
#include "sdLib.h"

/* VETROC variables */
static unsigned int vetrocSlotMask=0;
int nvetroc=0;		// number of vetrocs in the crate
unsigned int *tdcbuf;
extern int vetrocA32Base;                      /* Minimum VME A32 Address for use by VETROCs */

/* FADC variables */
extern int fadcA32Base, nfadc;
unsigned int MAXFADCWORDS=0;	/* for calculation of max words in the block transfer */

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int ifa, stat;
  unsigned short faflag;

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
  tiSetTriggerSource(TI_TRIGGER_FPTRG);  //Front panel trg;

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Init the SD library so we can get status info */
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
      tiSetBusySource(TI_BUSY_LOOPBACK,1);
    }
  else
    {
      printf("Will try to use SD in Switch Slot\n");
#ifdef USE_FADC
      /* FADC Initialization flags */
      faflag = 0; /* NO SDC */
      faflag |= (1<<0);  /* VXS sync-reset */
      faflag |= FA_INIT_VXS_TRIG;  /* VXS trigger source */
      faflag |= FA_INIT_VXS_CLKSRC;  /* VXS 250MHz Clock source */
#endif
      tiSetBusySource(TI_BUSY_SWB,1);

    }

  /*****************
   *   FADC SETUP
   *****************/
#ifdef USE_FADC
  /* FADC Initialization flags */
  //faflag = 0; /* NO SDC */
  //faflag |= (1<<0);  /* VXS sync-reset */
  //faflag |= FA_INIT_VXS_TRIG;  /* VXS trigger source */
  //faflag |= FA_INIT_VXS_CLKSRC;  /* VXS 250MHz Clock source */

  fadcA32Base = 0x09000000; /* Set the Base address of the FADC block data registers */

  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC, faflag);
  vmeSetQuietFlag(0);



  sdSetActiveVmeSlots(faScanMask()); /* Tell the sd where to find the fadcs */

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      faEnableSyncReset(faSlot(ifa));
      faEnableBusError(faSlot(ifa));

      /* Set input DAC level */
      faSetDAC(faSlot(ifa), 3250, 0);

      /*  Set All channel thresholds to 150 */
      faSetThreshold(faSlot(ifa), 400, 0xffff);

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
		    4,   /* NSA */
		    1,   /* NP */
		    0);

      faSoftReset(faSlot(ifa),0);
      faResetTriggerCount(faSlot(ifa));

    }

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
      //    MAXFADCWORDS = 5 + blockLevel * (196 + (FADC_WINDOW_WIDTH >> 1) ) + 4; /* 4 = fudge */
      MAXFADCWORDS = 4000;
    }
#endif

#ifdef USE_FADC
  faGStatus(0);
#endif

  /*****************
   *   VTP SETUP
   *****************/
#ifdef USE_VTP
  tiRocEnable(2);
  tiSetBusySource(TI_BUSY_SWA, 0);
#endif

  sdStatus(0);
  tiStatus(0);

  printf("rocDownload: User Download Executed\n");
}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  int ivt;
  unsigned short vtflag;

  /*****************
   *   VETROC SETUP
   *****************/
#ifdef USE_VETROC
  tdcbuf = (unsigned int *)malloc(MAXVETROCDATA*sizeof(unsigned int));

  /* 0 = software synch-reset, FP input 1, internal clock */
  //	vtflag = 0x20;  /* FP 1  0x020;  MAY NEED TO BE CHANGED*/
  vtflag = 0x111; /* vxs sync-reset, trigger, clock */

  vetrocA32Base = 0x0A000000;
  nvetroc = vetrocInit((VETROC_SLOT<<19),(VETROC_SLOT_INCR<<19) , NVETROC, vtflag);
  if (nvetroc <= 0) {
    printf("ERROR: no VETROC !!! \n");
  }

  for(ivt=0; ivt<nvetroc; ivt++)
    {
      vetrocSlotMask |= (1<<vetrocSlot(ivt)); /* Add it to the mask */
    }
  printf("vetrocSlotMask=0x%08x\n", vetrocSlotMask);

  vetrocGSetProcMode(2000, 2000); // ns units

  for(ivt=0; ivt<nvetroc; ivt++)
    {
      vetrocLinkReset(vetrocSlot(ivt));
      vetrocClear(vetrocSlot(ivt));
    }
#endif
  /* Print status for all boards */
#ifdef USE_VETROC
  vetrocGStatus(0);
#endif

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  /* Print status for all boards */
#ifdef USE_FADC
  faEnableSyncReset(faSlot(0));
  faGStatus(0);
#endif
  sdStatus(0);
  tiStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

#ifdef USE_FADC
  /* Enable/Set Block Level on modules, if needed, here */
  faGSetBlockLevel(blockLevel);

  /*  Enable FADC */
  faGEnable(0, 0);
#endif

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
  int ivt, islot;

  /* Example: How to stop internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Disable random trigger */
  tiDisableRandomTrigger();
#elif defined (INTFIXEDPULSER)
  /* Disable Fixed Rate trigger */
  tiSoftTrig(1,0,700,0);
#endif

#ifdef USE_FADC
  /* FADC Disable */
  faGDisable(0);
#endif

  /* Print status for all boards */
#ifdef USE_VETROC
  vetrocGStatus(0);
#endif
#ifdef USE_FADC
  faGStatus(0);
#endif
  sdStatus(0);
  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, gbready, itime, read_stat, stat;
  int ivt, ifa, nwords_fa, nwords_vt, blockError, dCnt, len=0, idata;
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
  *dma_dabufp++ = LSWAP(tiGetIntCount());
  *dma_dabufp++ = LSWAP(0xdead);
  *dma_dabufp++ = LSWAP(0xcebaf111);
  *dma_dabufp++ = LSWAP(0xcebaf222);
  BANKCLOSE;

#ifdef USE_FADC
  /* fADC250 Readout */
  BANKOPEN(3,BT_UI4,blockLevel);
  *dma_dabufp++ = LSWAP(0xb0b0b0b5); /* First word */

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  nwords_fa = faReadBlock(faSlot(ifa), dma_dabufp, MAXFADCWORDS, 1);
	  *dma_dabufp++ = LSWAP(nwords_fa);

	  /* Check for ERROR in block read */
	  blockError = faGetBlockError(1);

	  if(blockError)
	    {
	      printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		     faSlot(ifa), roCount, nwords_fa);

	      if(nwords_fa > 0)
		dma_dabufp += nwords_fa-1;
	    }
	  else
	    {
	      dma_dabufp += nwords_fa-1;
	    }
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n", roCount, datascan, scanmask);
    }
  BANKCLOSE;
#endif

#ifdef USE_VETROC
  /* Bank for VETROC data */
  BANKOPEN(4,BT_UI4,0);

  /* Check for valid data in VETROC */
  read_stat = 0;

  for(itime=0; itime<1000; itime++)
    {
      gbready = vetrocGBready();
      read_stat = (gbready == vetrocSlotMask);

      if (read_stat>0)
	{
	  break;
	}
    }

  if(read_stat>0)
    { /* read the data here */
      for(ivt=0; ivt<nvetroc; ivt++)
	{
	  nwords_vt = vetrocReadFIFO(vetrocSlot(ivt), &tdcbuf[0], MAXVETROCDATA, 1);

	  *dma_dabufp++ = LSWAP(0xb0b0b0b4); /* First word */
	  *dma_dabufp++ = LSWAP(nwords_vt);

	  for (ii=0; ii<nwords_vt; ii++)
	    {
	      *dma_dabufp++ = tdcbuf[ii];

	    }
	}
    }
  BANKCLOSE;
#endif

  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{

  printf("%s: Reset \n",__FUNCTION__);
#ifdef TI_MASTER
  /* Disable tiLive() wrapper function */
  vmeBusLock();

  tsLiveCalc = 0;
  tsLiveFunc = NULL;

  /* Disable TI library */
  tiUnload(1);

  vmeBusUnlock();
#endif /* TI_MASTER */

}

#ifdef TI_MASTER
extern int tsLiveCalc;
extern FUNCPTR tsLiveFunc;
/*
   tiLive() wrapper allows the Live Time display in rcGUI to work
*/
int
tsLive(int sflag)
{
  unsigned int retval = 0;

  vmeBusLock();
  if(tsLiveFunc != NULL)
    {
      retval = tiLive(sflag);
    }
  vmeBusUnlock();

  return retval;
}
#endif /* TI_MASTER */

/*
  Local Variables:
  compile-command: "make -k vtpCompton_list.so"
  End:
 */
