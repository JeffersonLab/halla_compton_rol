/*****************************************************************
 *
 * tiprimary_list.c - "Primary" Readout list routines for tiprimary
 *
 * Usage:
 *
 *    #include "tiprimary_list.c"
 *
 *  then define the following routines:
 *
 *    void rocDownload();
 *    void rocPrestart();
 *    void rocGo();
 *    void rocEnd();
 *    void rocTrigger();
 *
 */

#define ROL_NAME__ "TIPRIMARY"

#include <unistd.h>
#include <stdio.h>
#ifdef LINUX
#include <pthread.h>
#endif
#ifdef VXWORKS
#include <intLib.h>
#include <taskLib.h>
#endif

#include <string.h>
#include <rol.h>
#include "jvme.h"
#include "tiLib.h"
#ifdef LINUX
#include "dmaBankTools.h"
#endif
#include <TIPRIMARY_source.h>

#if ((TI_READOUT == TI_READOUT_EXT_POLL) || (TI_READOUT == TI_READOUT_TS_POLL) || (defined LINUX))
#define POLLING___
#endif

#if (TI_READOUT == TI_READOUT_TS_POLL) || (TI_READOUT == TI_READOUT_TS_INT)
static int const tsCrate = 0;
#else
static int const tsCrate = 1;
#endif

extern void daLogMsg (char *severity, char *fmt,...);
extern int  partStatsAll();

extern int bigendian_out;
extern int tiFiberLatencyOffset; /* offset for fiber latency */

extern int tsLiveCalc;
extern FUNCPTR tsLiveFunc;

extern int tiDoAck;
extern unsigned int tiIntCount;
#ifdef LINUX
extern int tiNeedAck;

/* Condition Variable for Readout Acknowledge */
pthread_mutex_t ack_mutex=PTHREAD_MUTEX_INITIALIZER;
#define ACKLOCK {				\
    if(pthread_mutex_lock(&ack_mutex)<0)	\
      perror("pthread_mutex_lock");		\
}
#define ACKUNLOCK {				\
    if(pthread_mutex_unlock(&ack_mutex)<0)	\
      perror("pthread_mutex_unlock");		\
}
pthread_cond_t ack_cv = PTHREAD_COND_INITIALIZER;
#define ACKWAIT {					\
    if(pthread_cond_wait(&ack_cv, &ack_mutex)<0)	\
      perror("pthread_cond_wait");			\
}
#define ACKSIGNAL {					\
    if(pthread_cond_signal(&ack_cv)<0)			\
      perror("pthread_cond_signal");			\
  }
int ack_runend=0;

#ifdef LINUX
pthread_cond_t endrun_cv = PTHREAD_COND_INITIALIZER;
struct timespec endrun_waittime;
int endrun_timedwait_ret=0;
#define ENDRUN_TIMEDWAIT(__x) {						\
    clock_gettime(CLOCK_REALTIME, &endrun_waittime);			\
    endrun_waittime.tv_sec += __x;					\
    endrun_timedwait_ret = pthread_cond_timedwait(&endrun_cv, &ack_mutex, &endrun_waittime); \
    if(endrun_timedwait_ret<0)						\
      perror("pthread_cond_timedwait");					\
  }
#define ENDRUN_SIGNAL {					\
    if(pthread_cond_signal(&endrun_cv)<0)		\
      perror("pthread_cond_signal");			\
  }

#endif

/*! Buffer node pointer */
extern DMANODE *the_event;
/*! Data pointer */
extern unsigned int *dma_dabufp;
#define RESET_EVTYPE(a) {						\
    the_event->data[0] = (the_event->data[0] &~ 0x00ff0000) | ((a & 0xff)<<16); \
}
#else
#define dma_dabufp (rol->dabufp)
#define the_event (rol)
#define EVENTOPEN(a,b)  CEOPEN(a,b)
#define EVENTCLOSE      CECLOSE
#define BANKOPEN(a,b,c) CBOPEN(a,b,c)
#define BANKCLOSE       CBCLOSE
#define RESET_EVTYPE(a) {						\
    __the_event__->data[0] = (__the_event__->data[0] &~ 0x00ff0000) | ((a & 0xff)<<16);	\
  }
DANODE *end_event[256]; /* Pointers to end event buffers */
int nend_event; /* Number of event event buffers actually acquired */
#endif

/* ROC Function prototypes defined by the user */
void rocDownload();
void rocPrestart();
void rocGo();
void rocEnd();
void rocTrigger();
void rocCleanup();
#ifdef LINUX
int  getOutQueueCount();
int  getInQueueCount();
#endif

/* Asynchronous (to tiprimary rol) trigger routine, connects to rocTrigger */
void asyncTrigger();

/* Input and Output Partitions for Linux VME Readout */
#ifdef LINUX
DMA_MEM_ID vmeIN, vmeOUT;
int emptyCount = 0;   /* Count the number of times event buffers are empty */
int errCount = 0;     /* Count the number of times no buffer available from vmeIN */
#endif

/**
 *  DOWNLOAD
 */
static void __download()
{
  int status = OK;

  daLogMsg("INFO","Readout list compiled %s", DAYTIME);
#ifdef POLLING___
  rol->poll = 1;
#endif
  *(rol->async_roc) = 0; /* Normal ROC */

#ifdef LINUX
  bigendian_out=1;
#else
  bigendian_out=0;
#endif

#ifdef LINUX
  pthread_mutex_init(&ack_mutex, NULL);
  pthread_cond_init(&ack_cv,NULL);
  pthread_cond_init(&endrun_cv,NULL);

  /* /\* Open the default VME windows *\/ */
  /* vmeOpenDefaultWindows(); */

  /* Initialize memory partition library */
  dmaPartInit();

  /* Setup Buffer memory to store events */
  dmaPFreeAll();
  vmeIN  = dmaPCreate("vmeIN",MAX_EVENT_LENGTH,MAX_EVENT_POOL,0);
  vmeOUT = dmaPCreate("vmeOUT",0,0,0);

  if(vmeIN == 0)
    daLogMsg("ERROR", "Unable to allocate memory for event buffers");

  /* Reinitialize the Buffer memory */
  dmaPReInitAll();
  dmaPStatsAll();
#else
  partStatsAll();
#endif

  /* Initialize VME Interrupt interface - use defaults */
  tiSetFiberLatencyOffset_preInit(FIBER_LATENCY_OFFSET);

#ifndef TI_SLAVE_PORT
#define TI_SLAVE_PORT 1
#endif

  tiSetFiberIn_preInit(TI_SLAVE_PORT);

#ifndef TI_INIT_FLAGS
#define TI_INIT_FLAGS 0
#endif

  status = tiInit(TI_ADDR,TI_READOUT,TI_INIT_FLAGS);
  if(status == -1)
    daLogMsg("ERROR","Unable to initialize TI board");

  tiSetCrateID(ROCID);
  /* Set timestamp format 48 bits */
  tiSetEventFormat(3);

  /* Execute User defined download */
  rocDownload();

  daLogMsg("INFO","Download Executed");

  tiDisableVXSSignals();

  /* If the TI Master, send a Clock and Trig Link Reset */
  if(tsCrate)
    {
      printf("%s: TI Clock Reset.  Trig Link Reset\n",
	     __func__);
      tiClockReset();
      taskDelay(2);
      tiTrigLinkReset();
    }
  tiEnableVXSSignals();


} /*end download */

/**
 *  PRESTART
 */
static void __prestart()
{
#ifdef LINUX
  ACKLOCK;
  ack_runend=0;
  ACKUNLOCK;
#endif
  vmeCheckMutexHealth(10);

  CTRIGINIT;
  *(rol->nevents) = 0;
  daLogMsg("INFO","Entering Prestart");

  TIPRIMARY_INIT;
#ifdef LINUX
  CTRIGRSS(TIPRIMARY,1,linuxusrtrig,linuxusrtrig_done);
  tiIntConnect(TI_INT_VEC,asyncTrigger,0);
#else

#if (TI_READOUT == TI_READOUT_EXT_INT) || (TI_READOUT == TI_READOUT_TS_INT)
  CTRIGRSA(TIPRIMARY,1,vxworksusrtrig,vxworksusrtrig_done);
  tiIntConnect(TI_INT_VEC,theIntHandler,TIPRIMARY_handlers);
#else
  CTRIGRSS(TIPRIMARY,1,vxworksusrtrig,vxworksusrtrig_done);
#endif

#endif
  CRTTYPE(1,TIPRIMARY,1);

  /* Execute User defined prestart */
  rocPrestart();

  /* If the TI Master, send a Sync Reset */
  if(tsCrate)
    {
      printf("%s: Sending sync as TI master\n",__FUNCTION__);
      sleep(1);
      tiSyncReset(1);
      taskDelay(2);
    }

  daLogMsg("INFO","Prestart Executed");

  if (__the_event__) WRITE_EVENT_;
  *(rol->nevents) = 0;
  rol->recNb = 0;
}

/**
 *  PAUSE
 */
static void __pause()
{
  CDODISABLE(TIPRIMARY,1,0);
  daLogMsg("INFO","Pause Executed");

  if (__the_event__) WRITE_EVENT_;
}

/**
 *  GO
 */
static void __go()
{
#ifdef VXWORKS
  int iev;
#endif

  daLogMsg("INFO","Entering Go");
#ifdef LINUX
  ACKLOCK;
  ack_runend=0;
  ACKUNLOCK;
  emptyCount=0;
  errCount=0;
#endif

  CDOENABLE(TIPRIMARY,1,1);
  rocGo();

#ifdef VXWORKS
  if( MAX_EVENT_POOL == (BUFFERLEVEL * 2) )
    nend_event = BUFFERLEVEL;
  else if( MAX_EVENT_POOL > BUFFERLEVEL )
    nend_event = MAX_EVENT_POOL - BUFFERLEVEL;
  else
    nend_event = 0;

  for(iev = 0; iev < nend_event; iev++)
    partGetItem(rol->pool, end_event[iev]);
#endif

  tsLiveCalc = 1;
  tsLiveFunc = (FUNCPTR)&tiLive;

  tiIntEnable(1);

  if (__the_event__) WRITE_EVENT_;
}

/**
 *  END
 */
static void __end()
{
#ifndef LINUX
  int iev=0;
#endif
  unsigned int blockstatus=0;
  int bready=0;

  if(tsCrate)
    {
      tiDisableTriggerSource(1);
    }

  bready=tiBReady();
  printf("---- Starting purge of TI blocks (%d)\n",
	 bready);
#ifdef LINUX
  blockstatus = tiBlockStatus(0,0);
  printf("__end: blockstatus=%d\n",blockstatus);

  ACKLOCK;
  ack_runend=1;
  if(blockstatus)
    {
      printf("%s: Clearing data from TI (blockstatus = 0x%x)\n",__FUNCTION__, blockstatus);
      ENDRUN_TIMEDWAIT(30);
      printf("%s: endrun_timedwait_ret = %d   blockstatus = 0x%x\n",
	     __FUNCTION__,endrun_timedwait_ret,tiBlockStatus(0,0));
    }
  ACKUNLOCK;

  INTLOCK;
  INTUNLOCK;

#else
  for(iev = 0; iev < nend_event; iev++)
    {
      if(end_event[iev])
	partFreeItem(end_event[iev]);
    }

  if(rol->doDone)
    {
      LOCKINTS;
      currEvMask = 0;
      __done();
      rol->doDone = 0;
      UNLOCKINTS;
    }

#ifdef POLLING___
  int iwait=0;
  bready = tiBReady();
  while(bready)
    {
      /* printf("\n\niwait = %d   Blocks ready = %d\n", iwait, bready); */
      /* printf("\n\ncurrEvMask = %d   poolEmpty = %d  rol->doDone = %d\n", */
      /* 	     currEvMask, poolEmpty, rol->doDone); */
      __poll();
      bready = tiBReady();
      if(iwait++ > (4 * BUFFERLEVEL)) break;
    }

#endif

#endif
  printf("---- DONE with purge of TI blocks\n");

  tiIntDisable();
  tiIntDisconnect();

  /* Execute User defined end */
  rocEnd();

  CDODISABLE(TIPRIMARY,1,0);

#ifdef LINUX
  dmaPStatsAll();
#else
  partStatsAll();
#endif

  daLogMsg("INFO","End Executed");

  if (__the_event__) WRITE_EVENT_;
}

#ifdef LINUX
void linuxusrtrig(unsigned long EVTYPE,unsigned long EVSOURCE)
{
  int ii, len;
  int syncFlag=0;
  unsigned int currentWord=0;
  DMANODE *outEvent;

  outEvent = dmaPGetItem(vmeOUT);

  if(outEvent != NULL)
    {
      len = outEvent->length;
      CEOPEN(ROCID, BT_BANK, blockLevel);

      if(rol->dabufp != NULL)
	{
	  for(ii=0;ii<len;ii++)
	    {
	      currentWord = (outEvent->data[ii]);
	      *rol->dabufp++ = currentWord;
	    }
	}
      else
	{
	  printf("tiprimary_list: ERROR rol->dabufp is NULL -- Event lost\n");
	}

      CECLOSE;

      ACKLOCK;

      dmaPFreeItem(outEvent);

      if(tiNeedAck>0)
	{
	  tiNeedAck=0;
	  ACKSIGNAL;
	}

      if(ack_runend)
	{
	  if(tiBlockStatus(0,0)==0)
	    ENDRUN_SIGNAL;
	}

      ACKUNLOCK;
    }
  else
    {
      logMsg("Error: no Event in vmeOUT queue\n",0,0,0,0,0,0);
    }

} /*end trigger */

void asyncTrigger()
{
  int intCount=0;
  int length,size;

  intCount = tiGetIntCount();

  /* grap a buffer from the queue */
  GETEVENT(vmeIN,intCount);
  if(the_event == NULL)
    {
      if(errCount == 0)
	daLogMsg("ERROR","asyncTrigger: No DMA Buffer Available. Events could be out of sync!");
      printf("asyncTrigger:ERROR: No buffer available!\n");
      errCount++;
      return;
    }

  if(the_event->length!=0)
    {
      printf("Interrupt Count = %d\t",intCount);
      printf("the_event->length = %lu\n",the_event->length);
    }

  /* Execute user defined Trigger Routine */
  rocTrigger();

  /* Put this event's buffer into the OUT queue. */
  ACKLOCK;
  PUTEVENT(vmeOUT);

  /* Check if the event length is larger than expected */
  length = (((int)(dma_dabufp) - (int)(&the_event->length))) - 4;
  size = the_event->part->size - sizeof(DMANODE);

  if(length>size)
    {
      printf("rocLib: ERROR: Event length > Buffer size (%d > %d).  Event %ld\n",
	     length,size,the_event->nevent);
      daLogMsg("WARN", "Event buffer overflow");
    }

  if(dmaPEmpty(vmeIN))
    {

      printf("WARN: vmeIN out of event buffers (intCount = %d).\n",intCount);

      if((ack_runend == 0) || (tiBReady() > 0))
	{
	  /* Set the NeedAck for Ack after a buffer is freed */
	  tiNeedAck = 1;

	  /* Wait for the signal indicating that a buffer has been freed */
	  ACKWAIT;
	}

    }

  ACKUNLOCK;
}

void linuxusrtrig_done()
{
} /*end done */
#else

void vxworksusrtrig(unsigned long EVTYPE,unsigned long EVSOURCE)
{
  long EVENT_LENGTH;
  {  /* begin user */
    unsigned long ii, event_ty, event_no;
    int length,size;

    event_ty = EVTYPE;
    event_no = *rol->nevents;
    rol->dabufp = (long *) 0;
    tiDoAck = 0;

#ifdef POLLING___
    tiIntCount++;
#endif

    rocTrigger(EVTYPE);

    /* Check if the event length is larger than expected */
    length = (((int)(rol->dabufp) - (int)(&__the_event__->length))) - 4;
    size = __the_event__->part->size - sizeof(DANODE);

    if(length>size)
      {
	printf("rocLib: ERROR: Event length > Buffer size (%d > %d).  Event %d\n",
	       length,size,__the_event__->nevent);
	daLogMsg("WARN", "Event buffer overflow");
      }

  }  /* end user */
} /*end trigger */

void vxworksusrtrig_done()
{
} /*end done */

#endif


void __done()
{
#ifdef VXWORKS
  tiIntAck();
  tiDoAck = 0;
#endif
  poolEmpty = 0; /* global Done, Buffers have been freed */
} /*end done */

void __status()
{
} /* end status */

#ifdef LINUX
int
getOutQueueCount()
{
  if(vmeOUT)
    return(dmaPNodeCount(vmeOUT));
  else
    return(0);
}

int
getInQueueCount()
{
  if(vmeIN)
    return(dmaPNodeCount(vmeIN));
  else
    return(0);
}
#endif

/* This routine is automatically executed just before the shared libary
   is unloaded.

   Clean up memory that was allocated
*/
#if 0
__attribute__((destructor)) void end (void)
{
  static int ended=0;
  extern volatile struct TI_A24RegStruct  *TIp;

  if(ended==0)
    {
      tsLiveCalc = 0;
      tsLiveFunc = NULL;

      printf("ROC Cleanup\n");
      rocCleanup();
#ifdef LINUX
      TIp = NULL;

      vmeCloseDefaultWindows();
#endif
      ended=1;
    }

}
#endif

void __reset()
{
  dmaPFreeAll();
  rocCleanup();
}
