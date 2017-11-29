/*
 * file: os_cpu_c.c
 *
 * PowerPC Specific C Language Functions for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */
#include <string.h>
#include "includes.h"

       INT32U  msrvalue;         // crt0.s
       INT32U  stkpv;
extern INT32U _SDA_BASE_;
extern INT32U _SDA2_BASE_;

static void waithere(void)
{
    for (;;)                    // a task has apparently returned
    {
	    printf("_____waithere\n");
    }
}
/***********************************************************
*                    INITIALIZE A TASK STACK
************************************************************/
OS_STK *OSTaskStkInit (void (*task)(void *pd), void *pdata, OS_STK *ptos, INT16U opt)
{
    //
    // This struct must match context save and restore in os_cpu_a.s
    //
    struct   stk
    {
        INT32U 
            R01, BLK, 
            R00,       R02,  R03, R04, R05, R06, R07,
            R08, R09,  R10,  R11, R12, R13, R14, R15,
            R16, R17,  R18,  R19, R20, R21, R22, R23,
            R24, R25,  R26,  R27, R28, R29, R30, R31,
            CR,  SRR0, SRR1, CTR, XER, LR;
    } *stkp = (struct stk*)(void*)ptos;  

    // Note: Since *ptos points to the last usable stack word rather than the
    // first unusable word, this initial stack frame wastes the very top word

    stkp--;                                 // create stack frame

    URP(opt);                               // parameter opt not used

    //HHTECH
    //memset(stkp, 0, sizeof(struct stk));
    {
	    unsigned char *p=(unsigned char *)stkp;
	    int i,size=sizeof(struct stk);
	    for(i=0;i<size;i++)
		    p[i]=0;

    }

    stkp->R13  = (INT32U)&_SDA_BASE_;       // r13
    stkp->R03  = (INT32U)pdata;             // r03
    stkp->R02  = (INT32U)&_SDA2_BASE_;      // r02
    stkp->LR   = (INT32U)waithere;          // LR 
    stkp->SRR1 = msrvalue | 0x8000;         // SRR1     
    stkp->SRR0 = (INT32U)task;              // SRR0 
    stkp->R01  = (INT32U)ptos;              // Stack Ptr

    return (OS_STK*)(void *)stkp;
}

void DelayNusec (INT32U delayTime)
{
    INT32U   start = readtimer32();

    delayTime *= 25;
    while ((readtimer32() - start) < delayTime);
}
void DelayNmsec (INT32U delayTime)
{
    DelayNusec(delayTime * 1000);
}

/*$PAGE*/
/* Code from here to bottom of file is taken directly from 
 * Jean J. Labrosse's example file IX86L/OS_CPU_C.C
 */ 

/*
*********************************************************************************************************
*                                               uC/OS-II
*                                         The Real-Time Kernel
*
*                        (c) Copyright 1992-1998, Jean J. Labrosse, Plantation, FL
*                                          All Rights Reserved
*
* File : OS_CPU_C.C
* By   : Jean J. Labrosse
*********************************************************************************************************
*/
#if 1 //OS_CPU_HOOKS_EN

void OSTCBInitHook  (OS_TCB *ptcb) {URP(ptcb);} // in file os_core.o
void OSInitHookBegin(void) {} // in file os_core.o
void OSTaskIdleHook (void) {} // in file os_core.o
void OSInitHookEnd  (void) {} // in file os_core.o

/*
*********************************************************************************************************
*                                          TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
void OSTaskCreateHook (OS_TCB *ptcb)
{
    URP(ptcb);
}


/*
*********************************************************************************************************
*                                           TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
void OSTaskDelHook (OS_TCB *ptcb)
{
    URP(ptcb);
}

/*
*********************************************************************************************************
*                                           TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other
*              operations during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                 will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the 
*                 task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/
void TOSTaskSwHook (INT32U srr0,INT32U srr1,INT32U msr)
{
	static int c=0;
	volatile unsigned int *p,*s;
	printf("srr0=0x%x,srr1=0x%x,msr=0x%x\n",srr0,srr1,msr);
	c++;
	printf("sw %d prioh=0x%x,prioc=0x%x,tcbh=0x%x,tcbc=0x%x\n",c,OSPrioHighRdy,OSPrioCur,OSTCBHighRdy,OSTCBCur);
	
	
}
void INTOSTaskSwHook(INT32U srr0,INT32U srr1,INT32U msr)
{
	static int c=0;
	volatile unsigned int *p,*s;
	int t;
	c++;
	printf("_srr0=0x%x,srr1=0x%x,msr=0x%x\n",srr0,srr1,msr);
	printf("intsw %d prioh=0x%x,prioc=0x%x,tcbh=0x%x,tcbc=0x%x\n",c,OSPrioHighRdy,OSPrioCur,OSTCBHighRdy,OSTCBCur);
}
void DEC_TEST()
{
	extern unsigned char OSTaskStarting;
	static int c=0;

}

void exception_handle(INT32U srr0,INT32U srr1,INT32U msr,INT32U vector)
{
	printf("srr0=0x%x,srr1=0x%x,msr=0x%x,vector=0x%x\n",srr0,srr1,msr,vector);
	while(1);
}
/*
*********************************************************************************************************
*                                           STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-II's statistics task.  This allows your 
*              application to add functionality to the statistics task.
*
* Arguments  : none
*********************************************************************************************************
*/
void OSTaskStatHook (void)
{
//	printf("__OSTaskStatHook\n");
}

/*
*********************************************************************************************************
*                                               TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
void OSTimeTickHook (void)
{
//	printf("OSTimeTickHook\n");
}
#endif
/*
 * *********************************************************************************************************
 * *                                             SET ISR VECTOR
 * *
 * * Description: This function is called to set the contents of an exception vector.  The function assumes
 * *              that the VBR (Vector Base Register) is set to 0x00000000.
 * *   
 * * Arguments  : vect     is the vector number
 * *              addr     is the address of the ISR handler
 * *
 * * Note(s)    : 1) Interrupts are disabled during this call
 * *********************************************************************************************************
 * */  
void OSVectSet (INT8U vect, void (*addr)(void))
{
	    INT32U *pvect;
#if OS_CRITICAL_METHOD == 3                      /* Allocate storage for CPU status register           */
	    OS_CPU_SR  cpu_sr;
#endif 

	    pvect = (INT32U *)((INT32U)vect);//changed by xuetao for writable Vector
	    OS_ENTER_CRITICAL();
	    *pvect = (INT32U)addr;
	    //    printp("vector at %08X is %08X.\n", (((INT32U)vect * 4) + 0x20000000), (INT32U)addr);
	    OS_EXIT_CRITICAL();
}

