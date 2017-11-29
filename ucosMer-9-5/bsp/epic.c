/*
 * file: epic.c
 *
 * PowerPC 8240  interrupt controller support
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */
#include "includes.h"

#define PRIO (--pri << 16)      // used by epic_InitEPIC


/*
 * *********************************************************************************************************
 * *                                         INTERRUPT CONTROLLER (IPIC)
 * *********************************************************************************************************
 * */

#define     BSP_IPIC_IRQS                       64                     /* Number of interrupt request lines available (i.e. number of possible vectors in IVOR field from the SIVCR register)             */

/*
 * *********************************************************************************************************
 * *                                           VARIABLES
 * *********************************************************************************************************
 * */


CPU_FNCT_VOID  BSP_IPIC_Handlers[BSP_IPIC_IRQS];              /* table of interrupt handlers                              */

static CPU_INT32U interrupt_ID;                               /* For external interrupt handling */

/* Interrupt Controller.
 * */
typedef struct interrupt_controller {
	INT16U  ic_sicr;
	INT8S   res1[2];
	INT32U  ic_sivec;
	INT32U  ic_sipnrh;
	INT32U  ic_sipnrl;
	INT32U  ic_siprr;
	INT32U  ic_scprrh;
	INT32U  ic_scprrl;
	INT32U  ic_simrh;
	INT32U  ic_simrl;
	INT32U  ic_siexr;
	INT8S   res2[88];
} intctl8260_t;

intctl8260_t * ipic;

static  void  BSP_IPIC_IntUnhandled (void)
{
	    while (1) {
		            ;                                                               /* Catch all unregistered interrupts  from the IPIC           */
	    }
}

// End EPIC hardware structure

/*
    \brief Initializes the EPIC
*/
void epic_InitEPIC(void)
{
    INT32S  i, pri = 16;
    ipic = (intctl8260_t *)0xf0010c00;
    ipic->ic_sicr = 0;
    ipic->ic_siprr = 0x0530b370;
    ipic->ic_scprrh = 0x0530b370;
    ipic->ic_scprrl = 0x0530b370;

    for (i = 0; i < BSP_IPIC_IRQS; i++) {                        /* initialize all irq service request handlers              */
	        BSP_IPIC_Handlers[i] = BSP_IPIC_IntUnhandled;
	        }

    //ipic->ic_simrl = 0x1e;
    ipic->ic_simrh = 0x0; 	//IRQ6 TMCNT PIT 
    ipic->ic_simrl = 0x0;	//0xA0000000;	//FCC1	FCC3 
}

int register_irq(INT32S irq,CPU_FNCT_VOID handler)
{
	BSP_IPIC_Handlers[irq] = handler;
	return irq;
}
/*
 * *********************************************************************************************************
 * *                                       PROCESSOR  ISR
 * *
 * * Description: This function handles the interrupts forwarded by the IPIC to the e300c1 core.
 * *
 * * Arguments  : none
 * *
 * * Note(s)    : 
 * *********************************************************************************************************
 * */

void  BSP_ProcessorISR (void)
{

	    interrupt_ID = (ipic->ic_sivec)>>26;   /* Read register SIVCR (IPIC) to know which handler to call */
		//printf("sivec is 0x%x\n", ipic->ic_sivec);
		//serial_getc();
	            (*(BSP_IPIC_Handlers[interrupt_ID])) ();                /* Call the corresponding handler */
}

/*
INT32U readtimer32()
{
	    asm("
			        mfspr       3, 268"
				    );
}
*/
void EIE_Hdlr(void)
{
	/*
    INT32U epicIntSrc;

    epicIntSrc = epicC->IACK;           // fetch highest priority vector

    if (epicIntSrc != MAX_HANDLERS)
    {
        irqHandler_t *hndlentry;
        hndlentry = &interruptHandlers[epicIntSrc];
        (*hndlentry->handler) (hndlentry->arg);
    }
    epicC->EOI = 0;
    */
	*(volatile unsigned short *)0xf0010240 |= 0x80;
	ipic->ic_sipnrh = 0x2;	//clear pit int flag 
    	//SetDecrementer(1650000000);
    	SetDecrementer(0x70000000);
	OSTimeTick();
	//*(volatile unsigned int *)0xb00000 += 0x1;
}


/* End of Source */
