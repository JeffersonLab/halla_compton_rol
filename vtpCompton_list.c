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

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL  1
#define BUFFERLEVEL 4

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   4000000 /* Size in Bytes - 4194304 is max allowable */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, external triggers */
#define TI_ADDR  0 /* Auto initialize (search for TI by slot */
//#define INTRANDOMPULSER
#define TI_DATA_READOUT

/* VETROC definitions *///#define USE_VETROC
#define USE_VETROC
#define MAXVETROCDATA 1200*BLOCKLEVEL
#define VETROC_SLOT 13					/* slot of first vetroc */
#define VETROC_SLOT_INCR 1			/* slot increment */
#define NVETROC	4								/* number of vetrocs used */
#define VETROC_ROMODE 1  /* Readout Mode: 0 = SCT, 1 = Single Board DMA, 2 = MultiBoard DMA */
#define VETROC_READ_CONF_FILE {			\
    vetrocConfig("");				\
    if(rol->usrConfig)				\
      vetrocConfig(rol->usrConfig);		\
  }

/* FADC definitions */
#define USE_FADC
#define NFADC     1							/* number of fadcs used */
#define FADC_ADDR (3<<19)			/* address of first fADC250 */
#define FADC_INCR (1<<19)			/* increment address to find next fADC250 */
#define FADC_WINDOW_LAT    500
#define FADC_WINDOW_WIDTH  500
#define FADC_MODE        		 1
#define FADC_READ_CONF_FILE {			\
    fadc250Config("");				\
    if(rol->usrConfig)				\
      fadc250Config(rol->usrConfig);		\
  }

/* VTP definitions */
#define USE_VTP

/* Scaler definitions */
#define SCAL_ADDR 0xa10000		/* Defined in SIS3801.h */

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x4A

/* Include */
#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "vetrocLib.h"      /* VETROC library */
#include "vetrocConfig.h"
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"
#include "sdLib.h"
#include "SIS3801.h"        /* 3801 scaler library */
#include "SIS.h"            /* 3801 scaler library */

/* SD variables */
static unsigned int sdScanMask = 0;

/* VETROC variables */
static unsigned int vetrocSlotMask=0;
int nvetroc=0;		// number of vetrocs in the crate
unsigned int *tdcbuf;
extern int vetrocA32Base;                      /* Minimum VME A32 Address for use by VETROCs */

/* FADC variables */
extern int fadcA32Base, nfadc;
unsigned int MAXFADCWORDS = 2100*BLOCKLEVEL;	/* for calculation of max words in the block transfer */

/* Scaler variables */
int use_3801=1;

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
	 *  SIS3801 SETUP
	 *****************/
	if (use_3801)
	{
		initSIS();
		clrAllCntSIS();
	}


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
#ifdef INTRANDOMPULSER
  tiSetTriggerSource(TI_TRIGGER_PULSER);
#else
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);  //TS Inputs trigger;
#endif

  tiFakeTriggerBankOnError(0);

  tiSetTriggerLatchOnLevel(1);
  tiSetFPInputReadout(1);
  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_ALL);
//	tiDisableTSInput( TI_TSINPUT_6 );
//  tiEnableTSInput( TI_TSINPUT_6);
//  tiSetInputPrescale(6,1); 

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  //  tiSetTriggerHoldoff(1,10,0);	// no more than 1 triggers in 10*16ns  - VETORC fails (BLOCKLEVEL=8,BUFFERLEVEL=4)
//  tiSetTriggerHoldoff(1,10,1);	// no more than 1 triggers in 10*480ns - VETROC fails (BLOCKLEVEL=8,BUFFERLEVEL=4)
//  tiSetTriggerHoldoff(1,11,1);	// no more than 1 triggers in 11*480ns - VETROC works (BLOCKLEVEL=8,BUFFERLEVEL=4)
//  tiSetTriggerHoldoff(1,12,1);	// no more than 1 triggers in 12*480ns - VETROC works (BLOCKLEVEL=8,BUFFERLEVEL=4)
  // tiSetTriggerHoldoff(1,15,1);	// no more than 1 triggers in 15*480ns - VETROC works (BLOCKLEVEL=8,BUFFERLEVEL=4)
  tiSetTriggerHoldoff(1,31,1);	// no more than 1 triggers in 31*480ns - VETROC works (BLOCKLEVEL=8,BUFFERLEVEL=4)

  tiSetTriggerHoldoff(2,10,0);	// no more than 2 triggers in 10*16ns

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

	/* Enable ti data readout */
	tiEnableDataReadout();

  /* BR: enable busy when buffer level is exceeded */
//  tiBusyOnBufferLevel(1);

  /* Init the SD library so we can get status info */
  sdScanMask = 0;
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
      tiSetBusySource(TI_BUSY_LOOPBACK,1);
    }
  else
    {
      printf("Will try to use SD in Switch Slot\n");
      sdSetActiveVmeSlots(0);	// clear active slots
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

  fadcA32Base = 0x08800000; /* Set the Base address of the FADC block data registers */

  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC, faflag);
  vmeSetQuietFlag(0);

  // We will set the busy out to the SD after the vetroc is added

  /* Just one FADC250 */
  faDisableMultiBlock();

  /* configure all modules based on config file */
  FADC_READ_CONF_FILE;

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      /* Bus errors to terminate block transfers (preferred) */
      faEnableBusError(faSlot(ifa));

      /*trigger-related*/
      faResetMGT(faSlot(ifa),1);
      faSetTrigOut(faSlot(ifa), 7);
    }

  faGStatus(0);
#endif

  /*****************
   *   VTP SETUP
   *****************/
#ifdef USE_VTP
  tiRocEnable(2);
/* BR: Added TI_BUSY_LOOPBACK and TI_BUSY_SWB here for testing - seems like it may be missing before or after unless this is done */
  tiSetBusySource(TI_BUSY_LOOPBACK | TI_BUSY_SWB | TI_BUSY_SWA, 0);
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
//  tdcbuf = (unsigned int *)malloc(MAXVETROCDATA*sizeof(unsigned int));

  /* 0 = software synch-reset, FP input 1, internal clock */
  //	vtflag = 0x20;  /* FP 1  0x020;  MAY NEED TO BE CHANGED*/
  vtflag = 0x111; /* vxs sync-reset, trigger, clock */

  vetrocA32Base = 0x09000000;
  nvetroc = vetrocInit((VETROC_SLOT<<19),(VETROC_SLOT_INCR<<19) , NVETROC, vtflag);
  if (nvetroc <= 0) {
    printf("ERROR: no VETROC !!! \n");
  }

  for(ivt=0; ivt<nvetroc; ivt++)
    {
      vetrocSlotMask |= (1<<vetrocSlot(ivt)); /* Add it to the mask */
    }
  printf("vetrocSlotMask=0x%08x\n", vetrocSlotMask);

  sdScanMask |= vetrocScanMask();
  sdSetActiveVmeSlots(sdScanMask); /* Tell the sd where to find the vetrocs */

  for(ivt=0; ivt<nvetroc; ivt++)
    {
      vetrocLinkReset(vetrocSlot(ivt));
      vetrocClear(vetrocSlot(ivt));
    }

  /* configure all modules based on config file */
  VETROC_READ_CONF_FILE;


#if(VETROC_ROMODE==2)
  vetrocEnableMultiBlock();
#endif

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
  /* Add FADC slot as a busy source to the SD */
  sdSetBusyVmeSlots(faScanMask(), 0 /* 0 = do not reset current busy settings */);
  faEnableSyncReset(faSlot(0));
  faGStatus(0);
#endif
  sdStatus(0);
  tiStatus(1);
/* TEMPORARY Ben/William test */
//  tiSyncReset(1); /* set the Block Level */
//  tiStatus(1);
/************/

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

	if (use_3801)
	{
		printf("Clearing scalers \n");
		runStartClrSIS();

	}

#ifdef USE_VETROC
  vetrocGSetBlockLevel(blockLevel);
#endif

  /* Interrupts/Polling enabled after conclusion of rocGo() */

  /* Example: How to start internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^N): N=7: ~3.9kHz, N=3: ~62kHz  */
  tiSetRandomTrigger(1,0x7);
#elif defined (INTFIXEDPULSER)
  /* Enable fixed rate with period (ns) 120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
     - Generated 1000 times */
  tiSoftTrig(1,1000,700,0);
#endif

  tiSetBlockLimit(0);

  tiStatus(1);
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

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
  tiStatus(1);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, gbready, itime, read_stat, stat;
  int ivt = 0, ifa, nwords_fa, nwords_vt, blockError, dCnt;
  unsigned int val;
  unsigned int datascan, scanmask, roCount;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  roCount = tiGetIntCount(); //Get the TI trigger count

  /* Readout the trigger block from the TI
     Trigger Block MUST be reaodut first */
//  dCnt = tiReadBlock(dma_dabufp,8+(5*blockLevel),1);trigBankType
	/* Open a trigger bank */
	/* BANKOPEN(0xff11, BT_SEG, blockLevel); */
/*	for (jj = 0; jj < blockLevel; jj++)
	{
		*dma_dabufp++ = (arg<<24)|(0x01<<16)|(1);
	}*/

  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("%d: No TI Trigger data or error.  dCnt = %d\n",roCount,dCnt);
      for (ii = 0; ii < 10; ii++)
	{
	  val = *dma_dabufp++;
	  printf(" data[%2d] = 0x%08x\n", ii, LSWAP(val));
	}

      printf("Press Enter to continue\n");
      getchar();
      tiSetBlockLimit(1);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */

      dma_dabufp += dCnt;
    }

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
	  nwords_fa = faReadBlock(faSlot(ifa), dma_dabufp + 1, MAXFADCWORDS, 1);
	  *dma_dabufp++ = LSWAP(nwords_fa);

	  /* Check for ERROR in block read */
	  blockError = faGetBlockError(1);

	  if(blockError)
	    {
	      printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		     faSlot(ifa), roCount, nwords_fa);

	      if(nwords_fa > 0)
		dma_dabufp += nwords_fa;
	    }
	  else
	    {
	      dma_dabufp += nwords_fa;
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
  *dma_dabufp++ = LSWAP(0xb0b0b0b4); /* First word */
  dCnt = 0;

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
#if(VETROC_ROMODE==2)
      ivt = 0;
#else
      for(ivt=0; ivt<nvetroc; ivt++)
#endif
	{
	  /* skip 1 word so nwords_vt is written before buffer to keep same format as before */
	  nwords_vt = vetrocReadBlock(vetrocSlot(ivt), dma_dabufp + 1,
				      MAXVETROCDATA, VETROC_ROMODE);
	  *dma_dabufp++ = LSWAP(nwords_vt);

          dma_dabufp+= nwords_vt;
	}
    }
  else
    {
      printf("Missed VETROC event data: gbready=0x%08X, vetrocSlotMask=0x%08X\n", gbready, vetrocSlotMask);
      fflush(stdout);
      tiSetBlockLimit(1);
      vetrocGStatus(1);
    }
  BANKCLOSE;
#endif

	/* Scaler readout */
	if (use_3801)
	{	
		BANKOPEN(6,BT_UI4,0);
		int k=0;

//SISFIFO_start(); //what is this one for?
//				if(SISFIFO_Check()||tiGetIntCount()==2) //why is this necessary?
		if(SISFIFO_Check())
		{
			*dma_dabufp++ = LSWAP(0xb0b0b0b6);

			for (k=0; k<1; k++)
			{
				int not_empty=SISFIFO_Read();
//							printf("3801 empty or not? %d\n", not_empty); 
				if(not_empty==0) break;
				else 
				{
					*dma_dabufp++ = LSWAP(0xb2b2b000|k);
//						*dma_dabufp++ = LSWAP(Read3801(0,0)&0xc0000000);

					for (ii=0; ii<32; ii++)
					{
							*dma_dabufp++ = LSWAP( Read3801(0,ii)&DATA_MASK);
//								*dma_dabufp++ = LSWAP( Read3801(0,ii));
					}
					*dma_dabufp++ = LSWAP(0xda0000aa);
/*								*dma_dabufp++ = LSWAP((Read3801(0,0)&UPBIT_MASK)>>24);
								*dma_dabufp++ = LSWAP((Read3801(0,0)&QRT_MASK)>>31);
								*dma_dabufp++ = LSWAP((Read3801(0,0)&HELICITY_MASK)>>30);
*/			}
			}
		}

		*dma_dabufp++ = LSWAP(0xda0000ff);  /* Event EOB */
		BANKCLOSE;

	}

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
