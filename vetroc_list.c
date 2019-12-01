/*************************************************************************
 *
 *  vetroc_list.c - Library of routines for readout and buffering of 
 *                     events using a JLAB Trigger Interface V3 (TI) with 
 *                     a Linux VME controller in CODA 3.0.
 *
 *************************************************************************/

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, external triggers */
#define TI_ADDR  0 /* Auto initialize (search for TI by slot */           

/* Vetroc definitions */
#define MAXVETROCDATA 1000
#define VETROC_SLOT 15					/* of first vetroc in crate */
#define VETROC_SLOT_INCR 2			/* slot spacing of vetrocs */
#define NVETROC	2								/* number of vetrocs used */

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x4A  

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "vetrocLib.h"      /* VETROC library */

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 4

static unsigned int vetrocSlotMask=0;
int nvetroc=0;		// number of vetrocs in the crate
unsigned long *tdcbuf;

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int ivt, stat;
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

  /* Set the sync delay width to 0x40*32 = 2.048us */
  tiSetSyncDelayWidth(0x54, 0x40, 1);
  
  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Init the SD library so we can get status info */
  stat = sdInit();
  if(stat==0) 
    {
      tiSetBusySource(TI_BUSY_SWB,1);
      sdSetActiveVmeSlots(0);
      sdStatus(0);
    }
  else
    { /* No SD or the TI is not in the Proper slot */
      tiSetBusySource(TI_BUSY_LOOPBACK,1);
    }


  /*****************
   *   VETROC SETUP
   *****************/
	
	tdcbuf = (unsigned long *)malloc(MAXVETROCDATA*sizeof(unsigned long));

/* 0 = software synch-reset, FP input 1, internal clock */
//	iflag = 0x20;  /* FP 1  0x020;  MAY NEED TO BE CHANGED*/
	iflag = 0x111; /* vxs sync-reset, trigger, clock */

	nvetroc = vetrocInit((VETROC_SLOT<<19),(VETROC_SLOT_INCR<<19) , NVETROC, iflag);
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
		vetrocStatus(vetrocSlot(ivt), 0);
	}

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
  int ivt, vtslot;

  /* Unlock the VME Mutex */
  vmeBusUnlock();

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

/* Print status for VETROC and TI */
	for(ivt=0; ivt<nvetroc; ivt++)
	{
		vetrocStatus(vetrocSlot(ivt), 0);
	}
  tiStatus(0);	

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

/* Print status for VETROC and TI */
	for(ivt=0; ivt<nvetroc; ivt++)
	{
		vetrocStatus(vetrocSlot(ivt), 0);
	}
  tiStatus(0);	

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());
  
}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, gbready, itime, read_stat;
  int ivt, nwords, blockError, dCnt, len=0, idata;
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

	/* Bank for VETROC data */
	BANKOPEN(3,BT_UI4,0);

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
		
//	*dma_dabufp++ = LSWAP(0xb0b0b0b4); /* First word */ 
//	*dma_dabufp++ = LSWAP(read_stat);
	
	if(read_stat>0) 
	{ /* read the data here */
		for(ivt=0; ivt<nvetroc; ivt++)
		{
			nwords = vetrocReadFIFO(vetrocSlot(ivt), &tdcbuf[0], MAXVETROCDATA, 1);

			*dma_dabufp++ = LSWAP(0xb0b0b0b4); /* First word */
			*dma_dabufp++ = LSWAP(nwords);

			for (ii=0; ii<nwords; ii++) 
			{
  			*dma_dabufp++ = tdcbuf[ii];
 
			}
		}
	}
	BANKCLOSE;

  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int islot=0;

  printf("%s: Reset \n",__FUNCTION__);

}
