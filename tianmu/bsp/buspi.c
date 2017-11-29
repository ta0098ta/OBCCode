/*Tian Ning 20130319*/
//#include <common.h>
//#include <asm/cpm_8260.h>
#include <linux/ctype.h>
//#include <malloc.h>
//#include <post.h>
//#include <net.h>


#include <includes.h>
#include <mpc8260.h>
#include <cpm_8260.h>
#include <immap_8260.h>
//#include <stdlib.h>
//#include <malloc.h>

typedef unsigned  char  uchar;


#define PROFF_SPI	((uint)0x0180)
#define SPI_EMASK		0x37	/* Event Mask				*/
#define SPI_MME			0x20	/* Multi-Master Error			*/
#define SPI_TXE			0x10	/* Transmit Error			*/
#define SPI_BSY			0x04	/* Busy					*/
#define SPI_TXB			0x02	/* Tx Buffer Empty			*/
#define SPI_RXB			0x01	/* RX Buffer full/closed		*/
#define SPI_STR			0x80	/* SPCOM: Start transmit		*/




/* ---------------------------------------------------------------
 * Offset for initial SPI buffers in DPRAM:
 * We need a 520 byte scratch DPRAM area to use at an early stage.
 * It is used between the two initialization calls (spi_init_f()
 * and spi_init_r()).
 * The value 0x2000 makes it far enough from the start of the data
 * area (as well as from the stack pointer).
 * --------------------------------------------------------------- */
/*
#ifndef	CONFIG_SYS_SPI_INIT_OFFSET
#define	CONFIG_SYS_SPI_INIT_OFFSET	0x2000
#endif
*/
#define	CONFIG_SYS_SPI_INIT_OFFSET	0x2000
#define CPM_SPI_BASE 0x100
/* -------------------
 * Function prototypes
 * ------------------- */


void spi_init (void);


void    spi_init_f(void);
void    spi_init_r(void);
int     spi_write (uchar *addr, int alen, uchar *buffer, int len);
int     spi_write_test (uchar testdata);
int     spi_read (uchar *addr, int alen, uchar *buffer, int len);
int     spi_xfer (int count);


void udelay (unsigned int z) {
        int i,j;
        for (i = 0; i < z ; i++) 
                for (j = 100; j >0; j--);
}

#if 1

char * memcpy (char * dest, const char  *src, unsigned int count) {
        while (count--)
                *dest ++ = *src ++;
        return dest;
}

#endif



#if 1

#define NULL 0
#define MEMSIZE 4096
//#define MEMSIZE 4000

typedef double Align;
typedef union header
{
        struct{
                union header* next;
                unsigned usedsize;
                unsigned  freesize;
        }s;
        Align a;
}Header;
static Header mem[MEMSIZE];
static Header * memptr=NULL;
void* malloc(unsigned nbytes)
{
        Header *p,*newp;
        unsigned nunits;
        nunits = (nbytes+sizeof(Header)-1)/sizeof(Header)+1;
        if(memptr==NULL)
        {
                memptr->s.next=memptr=mem;
                memptr->s.usedsize=1;
                memptr->s.freesize=MEMSIZE-1;
        }
        for(p=memptr;(p->s.next!=memptr)&&(p->s.freesize<nunits);p=p->s.next);
        if(p->s.freesize<nunits) return NULL;
        newp=p+p->s.usedsize;
        newp->s.usedsize=nunits;
        newp->s.freesize=p->s.freesize-nunits;
        newp->s.next=p->s.next;
        p->s.freesize=0;
        p->s.next=newp;
        memptr=newp;
        return(void*)(newp+1);
}

#endif








/* -------------------
 * Variables
 * ------------------- */
#define MAX_BUFFER	0x104

/* ----------------------------------------------------------------------
 * Initially we place the RX and TX buffers at a fixed location in DPRAM!
 * ---------------------------------------------------------------------- */
static uchar *rxbuf =
/* (uchar *)&((immap_t *)CONFIG_SYS_IMMR)->im_dprambase
   [CONFIG_SYS_SPI_INIT_OFFSET];*/
//(uchar *)&((immap_t *)CFG_IMMR)->im_dprambase[CFG_IMMR];//2013
(uchar *)&((immap_t *)CFG_IMMR)->im_dprambase[CONFIG_SYS_SPI_INIT_OFFSET];
/*
   static uchar *txbuf =
   (uchar *)&((immap_t *)CONFIG_SYS_IMMR)->im_dprambase
   [CONFIG_SYS_SPI_INIT_OFFSET+MAX_BUFFER];
   */
// static uchar *txbuf =(uchar *)&((immap_t *)CFG_IMMR)->im_dprambase[CFG_IMMR+MAX_BUFFER];//2013
static uchar *txbuf =(uchar *)&((immap_t *)CFG_IMMR)->im_dprambase[CONFIG_SYS_SPI_INIT_OFFSET+MAX_BUFFER];
/* **************************************************************************
 *
 *  Function:    spi_init_f
 *
 *  Description: Init SPI-Controller (ROM part)
 *
 *  return:      ---
 *
 * *********************************************************************** */
void    spi_init_f(void)/*2013*/
{	unsigned int dpaddr;
        volatile spi_t *spi;
        volatile immap_t *immr;
        volatile cpm8260_t *cp;
        volatile cbd_t *tbdf, *rbdf;

        //immr = (immap_t *)  CONFIG_SYS_IMMR;
        immr=(immap_t*) CFG_IMMR;
        cp   = (cpm8260_t *) &immr->im_cpm;
        *(ushort *)(&immr->im_dprambase[PROFF_SPI_BASE]) = PROFF_SPI;
        spi  = (spi_t *)&immr->im_dprambase[PROFF_SPI];
        /* 1 */
        /* ------------------------------------------------
         * Initialize Port D SPI pins
         * (we are only in Master Mode !)
         * ------------------------------------------------ */
        /* --------------------------------------------
         * GPIO or per. Function
         * PPARD[16] = 1 [0x00008000] (SPIMISO)
         * PPARD[17] = 1 [0x00004000] (SPIMOSI)
         * PPARD[18] = 1 [0x00002000] (SPICLK)
         * PPARD[12] = 0 [0x00080000] -> GPIO: (CS for ATC EEPROM)
         * PPARD[19] = 0 [0x00001000] -> GPIO: (CS for ATC EEPROM)
         * -------------------------------------------- */	
        immr->im_ioport.iop_ppard |=  0x0000E000;	/* set  bits	*/
        //immr->im_ioport.iop_ppard &= ~0x00008000;	/* reset bit	*///2013
        immr->im_ioport.iop_ppard &= ~0x00001000;	/* reset bit	*///2013
        /* ----------------------------------------------
         * In/Out or per. Function 0/1
         * PDIRD[16] = 0 [0x00008000] -> PERI1: SPIMISO
         * PDIRD[17] = 0 [0x00004000] -> PERI1: SPIMOSI
         * PDIRD[18] = 0 [0x00002000] -> PERI1: SPICLK
         * PDIRD[12] = 1 [0x00080000] -> GPIO OUT: CS for ATC EEPROM
         * ---------------------------------------------- */

        immr->im_ioport.iop_pdird &= ~0x0000E000;	
        //immr->im_ioport.iop_pdird |= 0x0000E000;	
        immr->im_ioport.iop_pdird |= 0x00001000;

        /* ----------------------------------------------
         * special option reg.
         * PSORD[16] = 1 [0x00008000] -> SPIMISO
         * PSORD[17] = 1 [0x00004000] -> SPIMOSI
         * PSORD[18] = 1 [0x00002000] -> SPICLK
         * ---------------------------------------------- */
        immr->im_ioport.iop_psord |= 0x0000E000;//2013
        //  immr->im_ioport.iop_psord |= 0x0000F000;
        /* Initialize the parameter ram.
         * We need to make sure many things are initialized to zero
         */
        spi->spi_rstate	= 0;
        spi->spi_rdp	= 0;
        spi->spi_rbptr	= 0;
        spi->spi_rbc	= 0;
        spi->spi_rxtmp	= 0;
        spi->spi_tstate	= 0;
        spi->spi_tdp	= 0;
        spi->spi_tbptr	= 0;
        spi->spi_tbc	= 0;
        spi->spi_txtmp	= 0;

        /* Allocate space for one transmit and one receive buffer
         * descriptor in the DP ram
         */
#ifdef CONFIG_SYS_ALLOC_DPRAM
        dpaddr = m8260_cpm_dpalloc (sizeof(cbd_t)*2, 8);
#else
        dpaddr = CPM_SPI_BASE;
#endif

        /* 3 */
        /* Set up the SPI parameters in the parameter ram */
        spi->spi_rbase = dpaddr;
        spi->spi_tbase = dpaddr + sizeof (cbd_t);

        /***********IMPORTANT******************/

        /*
         * Setting transmit and receive buffer descriptor pointers
         * initially to rbase and tbase. Only the microcode patches
         * documentation talks about initializing this pointer. This
         * is missing from the sample I2C driver. If you dont
         * initialize these pointers, the kernel hangs.
         */
        spi->spi_rbptr = spi->spi_rbase;
        spi->spi_tbptr = spi->spi_tbase;

        /* 4 */
        /* Init SPI Tx + Rx Parameters */
        while (cp->cp_cpcr & CPM_CR_FLG)
                ;
        cp->cp_cpcr = mk_cr_cmd(CPM_CR_SPI_PAGE, CPM_CR_SPI_SBLOCK,
                        0, CPM_CR_INIT_TRX) | CPM_CR_FLG;
        while (cp->cp_cpcr & CPM_CR_FLG)
                ;

        /* 6 */
        /* Set to big endian. */
        spi->spi_tfcr = CPMFCR_EB;
        spi->spi_rfcr = CPMFCR_EB;

        /* 7 */
        /* Set maximum receive size. */
        spi->spi_mrblr = MAX_BUFFER;

        /* 8 + 9 */
        /* tx and rx buffer descriptors */
        tbdf = (cbd_t *) & immr->im_dprambase[spi->spi_tbase];
        rbdf = (cbd_t *) & immr->im_dprambase[spi->spi_rbase];

        tbdf->cbd_sc &= ~BD_SC_READY;
        rbdf->cbd_sc &= ~BD_SC_EMPTY;

        /* Set the bd's rx and tx buffer address pointers */

        //rbdf->cbd_bufaddr = (ulong) rxbuf;
        //tbdf->cbd_bufaddr = (ulong) txbuf;

        rbdf->cbd_bufaddr = dpaddr;
        tbdf->cbd_bufaddr = dpaddr + 0x100;
        
        /* 10 + 11 */
        //immr->im_spi.spi_spie = SPI_EMASK;		/* 2013Clear all SPI events	*/
        immr->im_spi.spi_spim = 0x00;			/* Mask  all SPI events */
        printf ("*** spi_init_flask:^_^!!!!!!\n");

        return;
}
/* ---------------------------------------------------------------
 * Offset for initial SPI buffers in DPRAM:
 * We need a 520 byte scratch DPRAM area to use at an early stage.
 * It is used between the two initialization calls (spi_init_f()
 * and spi_init_r()).
 * The value 0x2000 makes it far enough from the start of the data
 * area (as well as from the stack pointer).
 * --------------------------------------------------------------- */
void spi_init_r()/*2013*/
{
        volatile spi_t *spi;
        volatile immap_t *immr;
        volatile cbd_t *tbdf, *rbdf;
        //immr = (immap_t *)  CONFIG_SYS_IMMR;
        immr = (immap_t *)  CFG_IMMR;

        spi  = (spi_t *)&immr->im_dprambase[PROFF_SPI];

        /* tx and rx buffer descriptors */
        tbdf = (cbd_t *) & immr->im_dprambase[spi->spi_tbase];
        rbdf = (cbd_t *) & immr->im_dprambase[spi->spi_rbase];

        /* Allocate memory for RX and TX buffers */
        //rxbuf = (uchar *) malloc (MAX_BUFFER);
        //txbuf = (uchar *) malloc (MAX_BUFFER);
        
        //rxbuf = 0x0111cd;
        //txbuf = 0x01131d;

        rxbuf = immr->res4;
        txbuf = immr->res4a;


        rbdf->cbd_bufaddr = (ulong) rxbuf;
        tbdf->cbd_bufaddr = (ulong) txbuf;
        printf ("*** spi_init_ram:okokok!!!!!!\n");
        return;

}
/****************************************************************************
 *  Function:    spi_write
 **************************************************************************** */
int spi_write (uchar *addr, int alen, uchar *buffer, int len)
{
        int i;

        memset(rxbuf, 0, MAX_BUFFER);
        memset(txbuf, 0, MAX_BUFFER);
        *txbuf = 0x01;		/* write enable		*/
        spi_xfer(1);
        memcpy(txbuf, addr, alen);
        *txbuf = 0x01;		/* WRITE memory array	*/
        memcpy(alen + txbuf, buffer, len);
        spi_xfer(alen + len);
        /* ignore received data	*/
        for (i = 0; i < 1000; i++) {
                *txbuf = 0x01;	/* read status		*/
                txbuf[1] = 0;
                spi_xfer(2);
                if (!(rxbuf[1] & 1)) {
                        break;
                }
                udelay(1000);
        }
        if (i >= 1000) {
                printf ("*** spi_write: Time out while writing!\n");
        }

        return len;
}
/*spi_write_test 2013*/
int spi_write_test (uchar testdata)
{
        int i;

        //memset(rxbuf, 0, MAX_BUFFER);
        memset(rxbuf, 0, 1);
        //memset(txbuf, 0, MAX_BUFFER);
        //memset(txbuf, 1, MAX_BUFFER);
        memset(txbuf, 1, 1);
        *txbuf = testdata;		/* write enable		*/

        spi_xfer(1);
        //spi_xfer(2);
        //if(!(rxbuf[1]&1))
        // {	
        //printf("rxbuf[0]=%x\n",rxbuf[0]);
        //printf("rxbuf[1]=%x\n",rxbuf[1]);

        // }


        /* ignore received data	*/
        /*
           for (i = 0; i < 1000; i++) {
         *txbuf = 0x01;	
         txbuf[1] = 0;
         spi_xfer(2);
         if (!(rxbuf[1] & 1)) {
         break;
         }
         udelay(1000);
         }
         */


        //if (i >= 1000) {
        //	printf("*** spi_write: Time out while writing!\n");
        //}

        return rxbuf[0];
}

/****************************************************************************
 *  Function:    spi_read
 **************************************************************************** */
int spi_read (uchar *addr, int alen, uchar *buffer, int len)
{
        memset(rxbuf, 0, MAX_BUFFER);
        //memset(txbuf, 0, MAX_BUFFER);//2013
        memset(txbuf, 1, MAX_BUFFER);//2013
        memcpy(txbuf, addr, alen);
        *txbuf = 0x01;		/* READ memory array	*/

        /*
         * There is a bug in 860T (?) that cuts the last byte of input
         * if we're reading into DPRAM. The solution we choose here is
         * to always read len+1 bytes (we have one extra byte at the
         * end of the buffer).
         */
        spi_xfer(alen + len + 1);
        memcpy(buffer, alen + rxbuf, len);

        return len;
}
/****************************************************************************
 *  Function:    spi_xfer
 **************************************************************************** */
int spi_xfer (int count)
{
        volatile immap_t *immr;
        volatile spi_t *spi;
        cbd_t *tbdf, *rbdf;
        int tm;

        //printf (("*** spi_xfer entered ***\n"));

        immr = (immap_t *) CFG_IMMR;

        spi  = (spi_t *)&immr->im_dprambase[PROFF_SPI];

        tbdf = (cbd_t *) & immr->im_dprambase[spi->spi_tbase];
        rbdf = (cbd_t *) & immr->im_dprambase[spi->spi_rbase];

        /* Board-specific: Set CS for device (ATC EEPROM) */
        //immr->im_ioport.iop_pdatd &= ~0x00080000;
        immr->im_ioport.iop_pdatd &= ~0x00001000;//2016
        /* Setting tx bd status and data length */
        tbdf->cbd_sc  = BD_SC_READY | BD_SC_LAST | BD_SC_WRAP;
        tbdf->cbd_datlen = count;//2013
        //tbdf->cbd_datlen =0x01;

        //printf (("*** spi_xfer: Bytes to be xferred: %d ***\n",tbdf->cbd_datlen));
        //printf (("*** spi_xfer: Bytes to be xferred:%d ***\n",tbdf->cbd_datlen));//2013
        //printf ("spi_xfer: Bytes to be xferred=%x \n",tbdf->cbd_datlen);//2013
        //printf ("spi_xfer2: Bytes to be xferred=%x \n",tbdf->cbd_datlen);//2013
        /* Setting rx bd status and data length */
        rbdf->cbd_sc = BD_SC_EMPTY | BD_SC_WRAP;
        rbdf->cbd_datlen = 0;	 /* rx length has no significance */

        //immr->im_spi.spi_spmode = SPMODE_REV	|
        //		SPMODE_MSTR	|
        //		SPMODE_EN	|
        //		SPMODE_LEN(8)	|	/* 8 Bits per char */
        //		SPMODE_PM(0x8) ;	/* medium speed */
        immr->im_spi.spi_spmode = SPMODE_REV	|
                SPMODE_CP	|
                SPMODE_MSTR	|
                SPMODE_EN	|
                SPMODE_LEN(8)	|	/* 8 Bits per char */
                SPMODE_PM(0xf) ;	/* low speed */
        //SPMODE_PM(0x8) ;	/* medium speed */


        immr->im_spi.spi_spie = SPI_EMASK;		/* Clear all SPI events	*/
        immr->im_spi.spi_spim = 0x00;			/* Mask  all SPI events */

        /* start spi transfer */
        //printf(("*** spi_xfer: Performing transfer ...\n"));	
        immr->im_spi.spi_spcom |= SPI_STR;		/* Start transmit */

        /* --------------------------------
         * Wait for SPI transmit to get out
         * or time out (1 second = 1000 ms)
         * -------------------------------- */
        //for (tm=0; tm<1000; ++tm) {
        for (tm=0; tm<100; ++tm) {
                if (immr->im_spi.spi_spie & SPI_TXB) {	/* Tx Buffer Empty */
                        //printf (("*** spi_xfer: Tx buffer empty\n"));
                        break;
                }
                if ((tbdf->cbd_sc & BD_SC_READY) == 0) {
                        //printf (("*** spi_xfer: Tx BD done\n"));
                        break;
                }
                //udelay (1000);
                udelay (1);
        }
        if (tm >= 1000) {
                //	printf ("*** spi_xfer: Time out while xferring to/from SPI!\n");
        }
        //printf (("*** spi_xfer: ... transfer ended\n"));

#ifdef	DEBUG
        printf ("\nspi_xfer: txbuf after xfer\n");
        memdump ((void *) txbuf, 16);	/* dump of txbuf before transmit */
        printf ("spi_xfer: rxbuf after xfer\n");
        memdump ((void *) rxbuf, 16);	/* dump of rxbuf after transmit */
        printf ("\n");
#endif

        /* Clear CS for device */
        //immr->im_ioport.iop_pdatd |= 0x00080000;
        immr->im_ioport.iop_pdatd |= 0x00001000;//2016
        return count;
}



