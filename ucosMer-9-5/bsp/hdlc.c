#if 0
#include <common.h>
#include <asm/cpm_8260.h>
#else
#include <includes.h>
#include <cpm_8260.h>
#endif
#include <mpc8260.h>
#include "masks8260.h"

#define NUM_RXBDS 12
#define NUM_TXBDS 12
#define SPACER 1     /* need at least 32 byte space between Tx
		      * buffer and Rx buffer space to allow for 32 byte IDMA overrun behavior.*/

#define HDLC_C_MASK     0x0000F0B8    /* CRC Constant */
#define HDLC_C_PRES     0x0000FFFF    /* CRC Preset */
#define MAX_MRBLR_SIZE  1100
#define MAX_MFLR_SIZE  1070


INT8U fcc1_cur_tx=0,fcc3_cur_tx=0;
INT8U fcc1_cur_rx=0,fcc3_cur_rx=0;
INT8U globle_bd_num_0=0;
INT8U globle_bd_num_1=0;

CPU_INT32U fcc1_rx_packets=0,fcc3_rx_packets=0;
CPU_INT32U fcc1_tx_packets=0,fcc3_tx_packets=0;
extern INT8U HDLC_channel_select;
extern struct frame_node *timer[12];
extern void hdlc_timer_init();

#define      MSG_QUEUE_SIZE    10
void        *receivedDataArray0[MSG_QUEUE_SIZE];
void        *receivedDataArray1[MSG_QUEUE_SIZE];
OS_EVENT*    pFifo_Rx0;
OS_EVENT*    pFifo_Rx1;
OS_EVENT*    Hdlc_Flag;


typedef struct BufferDescriptor

{
	ushort bd_cstatus;     /* control and status */
	ushort bd_length;      /* transfer length */
	unsigned char  *buf_addr;        /* buffer address */

} BD;
BD *bd_0[12],*bd_1[12];

/*--------------------------*/
/* Buffer Descriptor Format */
/*--------------------------*/

typedef struct BufferDescRings

{
	BD      FCC1_RxBD[NUM_RXBDS];    /* Rx BD ring */
	BD      FCC1_TxBD[NUM_TXBDS];    /* Tx BD ring */
	BD      FCC3_RxBD[NUM_RXBDS];    /* Rx BD ring */
	BD      FCC3_TxBD[NUM_TXBDS];    /* Tx BD ring */

} BDRINGS;

BDRINGS *RxTxBD;     /* buffer descriptors base pointer */
#define BASE_OF_BDS 0xf0005000
#define STADDR    0x19	//0x98   /* HDLC Address for this receiver (station) */
#define FCC_INDEX      0         /* FCC1 Index into FCC Regs Array  */  
/*--------------------------------*/
/* Size of buffers in buffer pool */
/*--------------------------------*/

#define RCV_BUFFER_SIZE 1200
#define SND_BUFFER_SIZE	1200

typedef unsigned char LB_RCV[RCV_BUFFER_SIZE];
typedef unsigned char LB_SND[SND_BUFFER_SIZE];
LB_RCV FCC1_RcvBufferPool[NUM_RXBDS] ;
LB_RCV FCC3_RcvBufferPool[NUM_RXBDS] ;
LB_SND FCC1_SndBufferPool[NUM_TXBDS] ;
LB_SND FCC3_SndBufferPool[NUM_TXBDS] ;
volatile immap_t *IMM;

void  LoadTxBuffers()
{
	unsigned short   index, pattern;

	/*----------------*/
	/* Load 254 0x55s */
	/*----------------*/

	/*------------------------------*/
	/* Load the destination address */
	/*------------------------------*/

	for (index = 0; index < NUM_RXBDS; index++) 
	{
		FCC1_SndBufferPool[index][0] = (STADDR%256); /* Load the low byte first */
		FCC3_SndBufferPool[index][0] = (STADDR%256); /* Load the low byte first */
	}
	//BufferPool[FIRST_TX_BUF][1] = (STADDR/256); /* Load the high byte */

/*	for (index = 1; index < BUFFER_SIZE; index++)

	{
		BufferPool[FIRST_TX_BUF][index] = (index+1) & 0xff;
	}
*/

	FCC1_SndBufferPool[0][1] = 0x1;
	FCC1_SndBufferPool[0][2] = 0x2;
	FCC1_SndBufferPool[0][3] = 0x3;
	FCC1_SndBufferPool[0][4] = 0x4;
	FCC1_SndBufferPool[0][5] = 0x5;

	FCC3_SndBufferPool[0][1] = 0x1;
	FCC3_SndBufferPool[0][2] = 0x2;
	FCC3_SndBufferPool[0][3] = 0x3;
	FCC3_SndBufferPool[0][4] = 0x4;
	FCC3_SndBufferPool[0][5] = 0x5;


}
void InitBDs()

{
   
	unsigned short  index;



   /*-------------------*/
   /* Initialize RxBDs. */
   /*-------------------*/

#if 0
	printf("111\n");
    IMM->im_fcc[FCC_INDEX].fcc_gfmr &= DISABLE_TX_RX;

   /*--------------------------------------*/
   /* Issue Init Stop TX Command for FCC1. */
   /*--------------------------------------*/

   while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);

   IMM->im_cpm.cp_cpcr = CPCR_STOP_TX |
	                   FCC1_PAGE_SUBBLOCK |
			                   CPCR_FLG;              /* ISSUE STOP TX COMMAND */

   while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);
#endif
   for (index = 0; index < NUM_RXBDS; index++) 
    
   {
      /*--------------------------*/
      /* Allocate Receive Buffers */
      /*--------------------------*/

	   //printf("buf_addr=0x%x,RcvBufferPool=0x%x\n",&RxTxBD->FCC1_RxBD[index].buf_addr,FCC1_RcvBufferPool[index]);
      RxTxBD->FCC1_RxBD[index].buf_addr = (unsigned char *)&FCC1_RcvBufferPool[index];
      RxTxBD->FCC1_RxBD[index].bd_length = 0;    /* reset */
      RxTxBD->FCC3_RxBD[index].buf_addr = (unsigned char *)&FCC3_RcvBufferPool[index];
      RxTxBD->FCC3_RxBD[index].bd_length = 0;    /* reset */

      if( index != (NUM_RXBDS-1) )

      {
         RxTxBD->FCC1_RxBD[index].bd_cstatus = 0x9000;    /* Empty */
         RxTxBD->FCC3_RxBD[index].bd_cstatus = 0x9000;    /* Empty */
      }

      else

      {
         /*-------------------------------------------------------------*/
         /* Last RX BD. Set the empty bit and the wrap bit because it's */
         /* the last buffer descriptor in the BD ring.                  */
         /*-------------------------------------------------------------*/

         RxTxBD->FCC1_RxBD[index].bd_cstatus = 0xB000;    
         RxTxBD->FCC3_RxBD[index].bd_cstatus = 0xB000;    
      }
   }

   /*-------------------*/
   /* Initialize TxBDs. */
   /*-------------------*/

   for (index=0; index < NUM_TXBDS; index++) 
   
   {
      /*--------------------------------------------------------------------*/
      /* load the buffer length - 2 bytes because the FCC will need to write*/
      /* the two CRC bytes to Rx buffer.                                    */
      /*--------------------------------------------------------------------*/

      RxTxBD->FCC1_TxBD[index].bd_length = 6;		//(BUFFER_SIZE-1);
      RxTxBD->FCC3_TxBD[index].bd_length = 6;		//(BUFFER_SIZE-1);

      /*--------------------------------------------------------*/
      /* load the address of the data buffer in external memory */
      /*--------------------------------------------------------*/

      RxTxBD->FCC1_TxBD[index].buf_addr = (unsigned char *)&FCC1_SndBufferPool[index];
      RxTxBD->FCC3_TxBD[index].buf_addr = (unsigned char *)&FCC3_SndBufferPool[index];
         


      if (index != (NUM_TXBDS-1))

      {
         /*---------------------------------*/
         /* Set Ready, Tx CRC, and Last bit */
         /*---------------------------------*/

         RxTxBD->FCC1_TxBD[index].bd_cstatus = 0x0000;     
         RxTxBD->FCC3_TxBD[index].bd_cstatus = 0x0000;     
      }

      else

      {
         /*-----------------------------------------------*/
         /* Set Ready, Tx CRC, Last bit, and the Wrap bit */
         /*-----------------------------------------------*/

         RxTxBD->FCC1_TxBD[index].bd_cstatus = 0x2000;     
         RxTxBD->FCC3_TxBD[index].bd_cstatus = 0x2000;     
      }
   }

} /* end InitBDs */

void InitParallelPorts()

{
	int	i;
	while (1)
	{
		for (i = 0; i< 10; i++);
		*(volatile unsigned int *)(0xF0010D70) |= 0x00000C00;
	    	for (i = 0; i< 10; i++);
		*(volatile unsigned int *)(0xF0010D70) &= 0xfffff3ff;
	}
		
#if 0
    /* Clear the Port Pin Assignment Registers */

    IMM->io_regs[PORT_A].ppar = 0x00000000;
    IMM->io_regs[PORT_B].ppar = 0x00000000;
    IMM->io_regs[PORT_C].ppar = 0x00000000;
    IMM->io_regs[PORT_D].ppar = 0x00000000;

    /* Clear the Port Data Direction Registers */
    
    IMM->io_regs[PORT_A].pdir = 0x00000000;
    IMM->io_regs[PORT_B].pdir = 0x00000000;
    IMM->io_regs[PORT_C].pdir = 0x00000000;
    IMM->io_regs[PORT_D].pdir = 0x00000000;

    /* Program the Port Special Options Registers */
 
    IMM->io_regs[PORT_A].psor = 0x00000000;
    IMM->io_regs[PORT_B].psor = 0x00000000;
    IMM->io_regs[PORT_C].psor = 0x00000000;
    IMM->io_regs[PORT_D].psor = 0x00000000;

    /* Program the Port Data Direction Registers */
    
    IMM->io_regs[PORT_A].pdir = 0x00000000;
    IMM->io_regs[PORT_B].pdir = 0x00000000;
    IMM->io_regs[PORT_C].pdir = 0x00000000;
    IMM->io_regs[PORT_D].pdir = 0x00000000;

    /* Program the Port Open-Drain Registers */

    IMM->io_regs[PORT_A].podr = 0x00000000;
    IMM->io_regs[PORT_B].podr = 0x00000000;    
    IMM->io_regs[PORT_C].podr = 0x00000000;
    IMM->io_regs[PORT_D].podr = 0x00000000;

    /* Program the Port Pin Assignment Registers */

    IMM->io_regs[PORT_A].ppar = 0x00000000;
    IMM->io_regs[PORT_B].ppar = 0x00000000;
    IMM->io_regs[PORT_C].ppar = 0x00000000;    
    IMM->io_regs[PORT_D].ppar = 0x00000000;
    
#endif
}  /* end InitParallelPorts() */

/*-------------------------------------------------------------------------
*
* FUNCTION NAME:  FCC1HInit 
*
* DESCRIPTION:
*
*  FCC1 HDLC Initialization Routine. 
*                 
* EXTERNAL EFFECT:
*
*  Parameter Ram and various registers on the MPC8260 including interrupt 
*  related registers and port registers. This function, when complete will 
*  initiate or start the transfer of 8 HDLC frames of data.
*
* PARAMETERS: None
*
* RETURNS: None 
*
*------------------------------------------------------------------------*/

void FCC1HInit()

{
   
volatile immap_t *immr = (immap_t *)CFG_IMMR;
   /*----------------------------------------------------------*/
   /* set up an abbreviated pointer for FCC HDLC Parameter Ram */
   /*----------------------------------------------------------*/

   fcc_hdlc_t* FCC1HDLC;
   fcc_hdlc_t* FCC2HDLC;

	       /* 28.9 - (7): initialise parameter ram */
   FCC1HDLC = (fcc_hdlc_t *)&(immr->im_dprambase[PROFF_FCC1]);
   FCC2HDLC = (fcc_hdlc_t *)&(immr->im_dprambase[PROFF_FCC3]);



   /*-------------------------------------------------------------------*/
   /* Initialize baud rate generator.  Motorola ADS target is           */
   /* configured to have a 132Mhz clock out of the CPM PLL. The 133Mhz  */
   /* internal clock will be divided down by 8 defined by a power-on    */
   /* reset value bit field value of 01 in the SCCR register to give a  */
   /* BRG (baud rate generator) clock of ~16.5Mhz. This clock is fed to */
   /* the baud rate generator. The baud rate generator is configured to */
   /* received it's input clock from the BRG divider. Then the clock    */
   /* will be divided down by 1 through the DIV16 field. The CD bits    */
   /* will be programmed to 64. They are not programmed to 65 because   */
   /* the total divide ratio is CD value+1 {can never divide by 0}.     */
   /* This will give a baud rate of approximately 256 Khz.              */
   /*-------------------------------------------------------------------*/

#if 0
   IMM->im_brgc5 = (0x00010080);  /* Enable BRG with division factor
                                     * of 65 
                                     */
#endif
   //IMM->im_brgc6 = (0x000101AD);  /* Enable BRG with division factor*/	//19.2KHz by ml
   //IMM->im_brgc6 = (0x000101AD);  /* Enable BRG with division factor*/
   //IMM->im_brgc7 = (0x00010080);  /* Enable BRG with division factor*/
   //IMM->im_brgc8 = (0x000101AD);  /* Enable BRG with division factor*/

   /*---------------------------------------------------------*/
   /* Initialize the FCC CPM MUX for FCC1.                    */
   /*                                                         */
   /* - Connect FCC1 to NMSI,                                 */
   /* - Transmit Clock = BRG5, Receive Clock = BRG5           */
   /*---------------------------------------------------------*/

   //IMM->im_cpmux.cmx_fcr = 0x0F001F00;
   //IMM->im_cpmux.cmx_fcr = 0x39003300;
   //IMM->im_cpmux.cmx_fcr = 0x39003b00;
   IMM->im_cpmux.cmx_fcr = 0x3E003E00;	//all from clks


   /***********************************************/
   /* HDLC Specific Parameter RAM Initialization. */
   /***********************************************/
         

   FCC1HDLC->c_mask = HDLC_C_MASK; 
   FCC2HDLC->c_mask = HDLC_C_MASK;

   FCC1HDLC->c_pres = HDLC_C_PRES; /* CRC Preset */
   FCC2HDLC->c_pres = HDLC_C_PRES; /* CRC Preset */

   FCC1HDLC->disfc = 0;  /* Clear Discard Frame Counter */
   FCC2HDLC->disfc = 0;  /* Clear Discard Frame Counter */

   FCC1HDLC->crcec = 0;  /* Clear CRC Error Counter */
   FCC2HDLC->crcec = 0;  /* Clear CRC Error Counter */

   FCC1HDLC->abtsc = 0;  /* Clear Abort Sequence Counter */
   FCC2HDLC->abtsc = 0;  /* Clear Abort Sequence Counter */

   FCC1HDLC->nmarc = 0;  /* Clear Nonmatching RX Address Counter */
   FCC2HDLC->nmarc = 0;  /* Clear Nonmatching RX Address Counter */

   FCC1HDLC->rfthr = 1; /* Rx Frames Threshold */
   FCC2HDLC->rfthr = 1; /* Rx Frames Threshold */

   FCC1HDLC->rfcnt = 0; /* Rx Frames Count */
   FCC2HDLC->rfcnt = 0; /* Rx Frames Count */

   FCC1HDLC->mflr = MAX_MFLR_SIZE; /* Maximum Frame Length */
   FCC2HDLC->mflr = MAX_MFLR_SIZE; /* Maximum Frame Length */

   /*-------------------*/
   /* Mask all the bits */
   /*-------------------*/

   FCC1HDLC->hmask = 0xFF;
   FCC2HDLC->hmask = 0xFF;

   /*-----------------------------------------------------------------------*/
   /* Establish addresses that the HDLC controller will be watching out for */
   /*-----------------------------------------------------------------------*/

   FCC1HDLC->haddr1 = 0xFF;  /* Broadcast Address */
   FCC1HDLC->haddr2 = STADDR;  /* Rx Station Address */
   FCC1HDLC->haddr3 = 0xFF;  /* Dummy */
   FCC1HDLC->haddr4 = 0xFF;  /* Dummy */

   FCC2HDLC->haddr1 = 0xFF;  /* Broadcast Address */
   FCC2HDLC->haddr2 = STADDR;  /* Rx Station Address */
   FCC2HDLC->haddr3 = 0xFF;  /* Dummy */
   FCC2HDLC->haddr4 = 0xFF;  /* Dummy */

  /*------------------------------------------------------*/
   /* Common Parameter RAM Area Initialization             */
   /*------------------------------------------------------*/


   FCC1HDLC->hdlc_genfcc.fcc_riptr = 0x3000;
   FCC1HDLC->hdlc_genfcc.fcc_tiptr = 0xb000;

   FCC2HDLC->hdlc_genfcc.fcc_riptr = 0x3800;    //0x3000-0x4000
   FCC2HDLC->hdlc_genfcc.fcc_tiptr = 0xb800;    //0xb000-0xc000


   /*--------------------------------------*/
   /* Set RFCR,TFCR -- Rx,Tx Function Code */
   /*--------------------------------------*/

   /*---------------------------------------------------------------------*/
   /* Motorola byte ordering, Snooping Disabled, Data and BDs on 60x bus. */
   /*---------------------------------------------------------------------*/

   FCC1HDLC->hdlc_genfcc.fcc_tstate = 0x10000000;
   FCC2HDLC->hdlc_genfcc.fcc_tstate = 0x10000000;

   FCC1HDLC->hdlc_genfcc.fcc_rstate = 0x10000000;
   FCC2HDLC->hdlc_genfcc.fcc_rstate = 0x10000000;

   /*------------------------------------------------*/
   /* Set RBASE,TBASE -- Rx,Tx Buffer Base Addresses */
   /*------------------------------------------------*/

   /*-------------------------------*/
   /* Set RXBD tbl start on 60x bus */
   /*-------------------------------*/

   FCC1HDLC->hdlc_genfcc.fcc_rbase = (uint)(&RxTxBD->FCC1_RxBD[0]);
   FCC2HDLC->hdlc_genfcc.fcc_rbase = (uint)(&RxTxBD->FCC3_RxBD[0]);

   /*-------------------------------*/
   /* Set TXBD tbl start on 60x bus */
   /*-------------------------------*/

   FCC1HDLC->hdlc_genfcc.fcc_tbase = (uint)(&RxTxBD->FCC1_TxBD[0]);
   FCC2HDLC->hdlc_genfcc.fcc_tbase = (uint)(&RxTxBD->FCC3_TxBD[0]);


   /*-----------------------------------------*/
   /* Set MRBLR -- Max. Receive Buffer Length */
   /*-----------------------------------------*/

   FCC1HDLC->hdlc_genfcc.fcc_mrblr = MAX_MRBLR_SIZE;
   FCC2HDLC->hdlc_genfcc.fcc_mrblr = MAX_MRBLR_SIZE;

   /*----------------------------------------------------------------*/
   /* Initialize GFMR 32-Bits Settings. Everything at default except */
   /* local loopback set and HDLC selected.                          */
   /*----------------------------------------------------------------*/

   IMM->im_fcc[FCC_INDEX].fcc_gfmr = 0x0000000;
      //IMM->im_fcc[FCC_INDEX].fcc_gfmr = 0x40000000;
   //IMM->im_fcc[2].fcc_gfmr = 0x40000000;
   IMM->im_fcc[2].fcc_gfmr = 0x0000000;


   //IMM->im_fcc[FCC_INDEX].fcc_gfmr |= GFMR_INT_LB; /* int loopback */

   /*-----------------------------------------*/
   /* Clear FCCE Register by writing all 1's. */
   /*-----------------------------------------*/

   IMM->im_fcc[FCC_INDEX].fcc_fcce = 0xFFFFFFFF;
   IMM->im_fcc[2].fcc_fcce = 0xFFFFFFFF;

   IMM->im_fcc[0].fcc_fccm = (FCC_ENET_TXE | FCC_ENET_RXF | FCC_ENET_TXB);
   IMM->im_fcc[2].fcc_fccm = (FCC_ENET_TXE | FCC_ENET_RXF | FCC_ENET_TXB);

   /*---------------------------------------------------------------------*/
   /* Initialize the FCC Data Syncronization Reg (FDSR). Program 7E flags */
   /* as syncronization flags.                                            */
   /*---------------------------------------------------------------------*/

   IMM->im_fcc[FCC_INDEX].fcc_fdsr = FDSR_HDLC;
   IMM->im_fcc[2].fcc_fdsr = FDSR_HDLC;

   /*------------------------------------------------*/
   /* Program PSMR. 16-bit CCITT CRC. 2 flag minimum */
   /*------------------------------------------------*/

   IMM->im_fcc[FCC_INDEX].fcc_fpsmr = HDLC_FPSMR_NOF_0; 
   IMM->im_fcc[2].fcc_fpsmr = HDLC_FPSMR_NOF_0;
                   
   /*----------------------------------------------------------------------*/
   /* Issue Init RX & TX Parameters Command for FCC1. This command to the  */
   /* CP lets it know to reinitialize FCC1 with the new parameter RAM      */
   /* values. When the ENT/ENR bits are set below Hunt Mode will begin     */
   /* automatically.                                                       */
   /*----------------------------------------------------------------------*/

   while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 

   /*-------------------------------------*/
   /* ISSUE INIT TX/RX PARAMETERS COMMAND */
   /*-------------------------------------*/

   IMM->im_cpm.cp_cpcr = CPCR_INIT_TX_RX_PARAMS |
                   FCC1_PAGE_SUBBLOCK | 
                   CPCR_FLG;      
   while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 

   IMM->im_cpm.cp_cpcr = CPCR_INIT_TX_RX_PARAMS |
                   FCC3_PAGE_SUBBLOCK | 
                   CPCR_FLG;      

   while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 

   /*-------------------------------------------------------------*/
   /* Set the ENT/ENR bits in the GFMR -- Enable Transmit/Receive */
   /*-------------------------------------------------------------*/

#if 1
   IMM->im_fcc[FCC_INDEX].fcc_gfmr |= GFMR_ENT | GFMR_ENR;
   IMM->im_fcc[2].fcc_gfmr |= GFMR_ENT | GFMR_ENR;
   //IMM->im_fcc[2].fcc_gfmr |= GFMR_ENT;// | GFMR_ENR;
#else
//   IMM->im_fcc[2].fcc_gfmr |= GFMR_ENR;
//   IMM->im_fcc[FCC_INDEX].fcc_gfmr |= GFMR_ENT;// | GFMR_ENR;
#endif

} /* end FCC1HInit() */

unsigned int kkk()
{
	asm(" mfmsr  3");
}
hdlc_init()
{
	unsigned short i;
	unsigned short j;

	IMM  = (immap_t *)CFG_IMMR;
	RxTxBD = (BDRINGS *)(BASE_OF_BDS);   /*  Get pointer to BD area 
					       *  on DPRAM  
					       */
	
	/*------------------------------------------------*/
	/* Load the Tx buffer pool with the test patterns */
	/*------------------------------------------------*/


#if 0
	scc4_serial_init();
	printf("SCC4 Init Passed.\n");

	printf("SCC4 RX Test, Press a key: ");
	printf("\nYour Input is : %c\n", scc4_serial_getc());


	scc4_serial_puts("SCC4 Serial Test!\n");
#endif
	
	


	//printf("msr value=0x%x\n",kkk());
	//*(volatile unsigned int *)0x80000000;
	LoadTxBuffers();

	/*--------------------------------------------------------------------*/
	/* First let's make sure the FCC1 functions are off while we program  */
	/* the buffer descriptors and the parameter ram.                      */
	/*--------------------------------------------------------------------*/

	/*----------------------------------------------------------------*/
	/* Clear the ENT/ENR bits in the GFMR -- disable Transmit/Receive */
	/*----------------------------------------------------------------*/
	IMM->im_fcc[FCC_INDEX].fcc_gfmr &= DISABLE_TX_RX;
	IMM->im_fcc[2].fcc_gfmr &= DISABLE_TX_RX;

	//IMM->fcc_regs[FCC1].gfmr &= DISABLE_TX_RX;

	/*--------------------------------------*/
	/* Issue Init Stop TX Command for FCC1. */
	/*--------------------------------------*/

	//return;
	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 

	IMM->im_cpm.cp_cpcr = CPCR_STOP_TX |
		FCC1_PAGE_SUBBLOCK | 
		CPCR_FLG;              /* ISSUE STOP TX COMMAND */

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 

	IMM->im_cpm.cp_cpcr = CPCR_STOP_TX |
		FCC3_PAGE_SUBBLOCK | 
		CPCR_FLG;              /* ISSUE STOP TX COMMAND */

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD); 


	InitBDs(); /* Initialize RX and TX BDs */

	/*----------------------------------*/
	/* Initialize the parallel I/O pins */
	/*----------------------------------*/

	//InitParallelPorts();
	/*---------------------------------------------------------------------*/
	/* Initialize and enable FCC1 in HDLC mode, internal loopback, and for */
	/* External Interrupt                                                  */
	/*---------------------------------------------------------------------*/

	FCC1HInit();
	//while (RxTxBD->RxBD[0].bd_cstatus | 0x8000 != 0);
#if 0
	printf("rx_bd length=%d\n",RxTxBD->FCC1_RxBD[0].bd_length);
	printf("rx_bd addr= %x\n", (ushort)&RxTxBD->FCC1_RxBD[0]);
      	printf("rx_buffer addr=0x%x\n",RxTxBD->FCC1_RxBD[0].buf_addr);
      	printf("end\n");
#endif

}
/**/


void  fcc1_hdlc_rx()
{

	int i;
	CPU_INT32U bd_addr;

	        
			                
		if (!(RxTxBD->FCC1_RxBD[fcc1_cur_rx].bd_cstatus & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO | BD_ENET_RX_CR
						| BD_ENET_RX_OV)))
		{
			fcc1_rx_packets++;
			bd_addr = &(RxTxBD->FCC1_RxBD[fcc1_cur_rx]);
			//bd_0=(BD *)bd_addr;
			
			if(HDLC_channel_select==0)
			OSQPost(pFifo_Rx0, bd_addr);  
			//OSSemPost(Hdlc_Flag);
			RxTxBD->FCC1_RxBD[fcc1_cur_rx].bd_cstatus |= BD_ENET_RX_EMPTY;
			

			//OSQPost(pFifo_Rx0, bd_addr);                                   /* Wake up the task pending for data */
		}

		/* Clear the status flags for this buffer. */
		RxTxBD->FCC1_RxBD[fcc1_cur_rx].bd_cstatus &= ~BD_ENET_RX_STATS;

		/* Mark the buffer empty. */
		RxTxBD->FCC1_RxBD[fcc1_cur_rx].bd_cstatus |= BD_ENET_RX_EMPTY;

		/* Update BD pointer to next entry. */
		if (RxTxBD->FCC1_RxBD[fcc1_cur_rx].bd_cstatus & BD_ENET_RX_WRAP)
			fcc1_cur_rx = 0;
		else
			fcc1_cur_rx++;
		
	

}
void  fcc3_hdlc_rx()
{

	int i;
	CPU_INT32U bd_addr;

	
	      
			                
		if (!(RxTxBD->FCC3_RxBD[fcc3_cur_rx].bd_cstatus & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO | BD_ENET_RX_CR
						| BD_ENET_RX_OV)))
		{
			fcc3_rx_packets++;
			bd_addr = &RxTxBD->FCC3_RxBD[fcc3_cur_rx];
			//bd_1[globle_bd_num_1]=(BD *)bd_addr;
			
			if(HDLC_channel_select==1)
			


			OSQPost(pFifo_Rx1, bd_addr);   
			//OSSemPost(Hdlc_Flag);                                /* Wake up the task pending for data */
			RxTxBD->FCC3_RxBD[fcc3_cur_rx].bd_cstatus |= BD_ENET_RX_EMPTY;
		

		}

		/* Clear the status flags for this buffer. */
		RxTxBD->FCC3_RxBD[fcc3_cur_rx].bd_cstatus &= ~BD_ENET_RX_STATS;

		/* Mark the buffer empty. */
		RxTxBD->FCC3_RxBD[fcc3_cur_rx].bd_cstatus |= BD_ENET_RX_EMPTY;

		/* Update BD pointer to next entry. */
		if (RxTxBD->FCC3_RxBD[fcc3_cur_rx].bd_cstatus & BD_ENET_RX_WRAP)
			fcc3_cur_rx = 0;
		else

			fcc3_cur_rx++;

	

}
 
void fcc1_hdlc_interrupt()
{
	CPU_INT16U fcce;
	fcce = IMM->im_fcc[0].fcc_fcce;
	IMM->im_fcc[0].fcc_fcce = fcce;
	
	if(fcce & FCC_ENET_RXF)
		fcc1_hdlc_rx();
	else

	//printf("**********\n");

	/* Transmit OK, or non-fatal error.  Update the buffer descriptors.
	 *         */
	if (fcce & (FCC_ENET_TXE | FCC_ENET_TXB)) {
#if 0
		while((RxTxBD->FCC1_TxBD[fcc1_cur_tx].bd_cstatus & BD_ENET_TX_READY)==0) {
			/* Update pointer to next buffer descriptor to be transmitted. */
			if (RxTxBD->FCC1_TxBD[fcc1_cur_tx].bd_cstatus & BD_ENET_TX_WRAP)
				fcc1_cur_tx = 0;
			else
				fcc1_cur_tx++;
			fcc1_tx_packets++;
		}		
#endif		
		fcc1_tx_packets++;
	}

}
void fcc3_hdlc_interrupt()
{
	CPU_INT16U fcce;
	fcce = IMM->im_fcc[2].fcc_fcce;
	IMM->im_fcc[2].fcc_fcce = fcce;
	//printf("-----------fcc3 int\n");
	if(fcce & FCC_ENET_RXF)
		fcc3_hdlc_rx();

	/* Transmit OK, or non-fatal error.  Update the buffer descriptors.
	 *         */
	if (fcce & (FCC_ENET_TXE | FCC_ENET_TXB)) {
#if 0		
		while((RxTxBD->FCC3_TxBD[fcc3_cur_tx].bd_cstatus & BD_ENET_TX_READY)==0) {
			/* Update pointer to next buffer descriptor to be transmitted. */
			if (RxTxBD->FCC3_TxBD[fcc3_cur_tx].bd_cstatus & BD_ENET_TX_WRAP)
				fcc3_cur_tx = 0;
			else
				fcc3_cur_tx++;
			fcc3_tx_packets++;
		}		
#endif		
		fcc3_tx_packets++;
	}

}
int  fcc1_hdlc_send_data(unsigned char *buf, int len)
{

	int i;

	if(len >=SND_BUFFER_SIZE)
	{
		printf("send data size must be little %d\n",SND_BUFFER_SIZE);
		len  = SND_BUFFER_SIZE-1;
	}
               
	for(i=0;i<len;i++)
		FCC1_SndBufferPool[fcc1_cur_tx][i+1] = buf[i];
	RxTxBD->FCC1_TxBD[fcc1_cur_tx].bd_length = len+1;
	
	RxTxBD->FCC1_TxBD[fcc1_cur_tx].bd_cstatus |= (BD_ENET_TX_READY | BD_ENET_TX_INTR | BD_ENET_TX_LAST | BD_ENET_TX_TC);
	if(RxTxBD->FCC1_TxBD[fcc1_cur_tx].bd_cstatus & BD_ENET_TX_WRAP)
		fcc1_cur_tx = 0;
	else
		fcc1_cur_tx++;
	//printf("fcc1_hdlc_send\n");
	return 0;
   	

}
int  fcc3_hdlc_send_data(unsigned char *buf, int len)
{

	int i;

	if(len >=SND_BUFFER_SIZE)
	{
		printf("send data size must be little %d\n",SND_BUFFER_SIZE);
		len  = SND_BUFFER_SIZE-1;
	}
	for(i=0;i<len;i++)
		FCC3_SndBufferPool[fcc3_cur_tx][i+1] = buf[i];
	RxTxBD->FCC3_TxBD[fcc3_cur_tx].bd_length = len+1;
	RxTxBD->FCC3_TxBD[fcc3_cur_tx].bd_cstatus |= (BD_ENET_TX_READY | BD_ENET_TX_INTR | BD_ENET_TX_LAST | BD_ENET_TX_TC);
	if(RxTxBD->FCC3_TxBD[fcc3_cur_tx].bd_cstatus & BD_ENET_TX_WRAP)
		fcc3_cur_tx = 0;
	else
		fcc3_cur_tx++;

	return 0;

}
#if 0
void hdlc_app_rx0()
{
	CPU_INT32U *bd_addr;
	INT8U err, i;
	BD *bd;
for(;;){
		bd_addr = (CPU_INT32U *)OSQPend(pFifo_Rx0, 0, &err);              /* Wait for a character received in the DUART */
		bd = (BD *)bd_addr;
		bd_0=bd;
	    if (HDLC_channel_select==0)	{
	    	OSSemPost(Hdlc_Flag);
		printf("0: bd_length=%x,bd_buf_addr=%x\n",bd->bd_length,bd->buf_addr);
		
		//for (i = 0; i<bd->bd_length; i++)
		//	printf("%x ", *(volatile INT8U *)(bd->buf_addr+i));
		//printf("\n");
	    }
		//after process receive data then set bd status to empty flag
		bd->bd_cstatus |= BD_ENET_RX_EMPTY;

}
}

void hdlc_app_rx1()
{
	CPU_INT32U *bd_addr;
	INT8U err, i;
	BD *bd;
for(;;){
		bd_addr = (CPU_INT32U *)OSQPend(pFifo_Rx1, 0, &err);              /* Wait for a character received in the DUART */

		bd = (BD *)bd_addr;
		bd_1=bd;
	    if (HDLC_channel_select==1)	{
	    	OSSemPost(Hdlc_Flag);
		//printf("1: bd_length=%x,bd_buf_addr=%x\n",bd->bd_length,bd->buf_addr);
		
		//for (i = 0; i<bd->bd_length; i++)
			//printf("%x ", *(volatile INT8U *)(bd->buf_addr+i));
		//printf("\n");
	    }
		//after process receive data then set bd status to empty flag
		bd->bd_cstatus |= BD_ENET_RX_EMPTY;

}
}
#endif
void hdlc_app_init()
{
	pFifo_Rx0 = OSQCreate(&receivedDataArray0[0], sizeof(CPU_INT32U));
	pFifo_Rx1 = OSQCreate(&receivedDataArray1[0], sizeof(CPU_INT32U));
	Hdlc_Flag=OSSemCreate(0);	
        	
        HDLC_channel_select = 0;
	register_irq(32, fcc1_hdlc_interrupt);
        register_irq(34, fcc3_hdlc_interrupt);
	hdlc_init();
	//hdlc_timer_init();
	//fcc1_hdlc_send_data("hello",5);
}

void hdlc_fcc1_reset()
{
	IMM->im_fcc[0].fcc_gfmr &= DISABLE_TX_RX;

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);

	IMM->im_cpm.cp_cpcr = CPCR_STOP_TX |
	FCC1_PAGE_SUBBLOCK |
	CPCR_FLG; /* ISSUE STOP TX COMMAND */

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);

	IMM->im_cpm.cp_cpcr = CPCR_INIT_TX_RX_PARAMS |
	FCC1_PAGE_SUBBLOCK |
	CPCR_FLG;
	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);
}

void hdlc_fcc3_reset()
{
	
	IMM->im_fcc[2].fcc_gfmr &= DISABLE_TX_RX;

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);

	IMM->im_cpm.cp_cpcr = CPCR_STOP_TX |
	FCC3_PAGE_SUBBLOCK |
	CPCR_FLG; /* ISSUE STOP TX COMMAND */

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);

	IMM->im_cpm.cp_cpcr = CPCR_INIT_TX_RX_PARAMS |
	FCC3_PAGE_SUBBLOCK |
	CPCR_FLG;

	while ((IMM->im_cpm.cp_cpcr & CPCR_FLG) != READY_TO_RX_CMD);
}
