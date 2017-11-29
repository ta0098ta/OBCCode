/*
 * (C) Copyright 2000, 2001, 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Hacked for MPC8260 by Murray.Jensen@cmst.csiro.au, 19-Oct-00, with
 * changes based on the file arch/ppc/mbxboot/m8260_tty.c from the
 * Linux/PPC sources (m8260_tty.c had no copyright info in it).
 */

/*
 * Minimal serial functions needed to use one of the SMC ports
 * as serial console interface.
 */

#include <includes.h>
#include <mpc8260.h>
#include <cpm_8260.h>


//static unsigned  char  sendbuf[20];
//static unsigned  char  rcvbuf[20];

#define SMC_INDEX               0
#define PROFF_SMC_BASE          PROFF_SMC1_BASE
#define PROFF_SMC               PROFF_SMC1
#define CPM_CR_SMC_PAGE         CPM_CR_SMC1_PAGE
#define CPM_CR_SMC_SBLOCK       CPM_CR_SMC1_SBLOCK
#define CMXSMR_MASK             (CMXSMR_SMC1|CMXSMR_SMC1CS_MSK)
#define CMXSMR_VALUE            CMXSMR_SMC1CS_BRG7

/* map rs_table index to baud rate generator index */
static unsigned char brg_map[] = {
	6,	/* BRG7 for SMC1 */
	7,	/* BRG8 for SMC2 */
	0,	/* BRG1 for SCC1 */
	1,	/* BRG1 for SCC2 */
	2,	/* BRG1 for SCC3 */
	3,	/* BRG1 for SCC4 */
};

void
smc1_serial_setbrg (void)
{
        gd_t * gd=ptr_gd;

        //m8260_cpm_setbrg(brg_map[SMC_INDEX], gd->baudrate);
        /*  zwj  2017  */
        //m8260_cpm_setbrg(brg_map[0], gd->baudrate);
        m8260_cpm_setbrg(6, 9600);
}

static uint addr;
int smc1_app_init (void)
{
        volatile immap_t *im = (immap_t *)CFG_IMMR;
	volatile smc_t *sp;
	volatile smc_uart_t *up;
	volatile cbd_t *tbdf, *rbdf;
	volatile cpm8260_t *cp = &(im->im_cpm);
	uint	dpaddr;

        /*****************************/





        /*****************************/

	/* initialize pointers to SMC */

	sp = (smc_t *) &(im->im_smc[SMC_INDEX]);
	*(ushort *)(&im->im_dprambase[PROFF_SMC_BASE]) = PROFF_SMC;
	up = (smc_uart_t *)&im->im_dprambase[PROFF_SMC];

		
//        printf ("smc_smcmr = %x\n", sp->smc_smcmr);

	/* Disable transmitter/receiver.
	*/
	sp->smc_smcmr &= ~(SMCMR_REN | SMCMR_TEN);

//        printf ("smc_smcmr = %x\n", sp->smc_smcmr);
		

	/* NOTE: I/O port pins are set up via the iop_conf_tab[] table */

	/* Allocate space for two buffer descriptors in the DP ram.
	 * damm: allocating space after the two buffers for rx/tx data
	 */
		
	dpaddr = addr;

		

	/* Set the physical address of the host memory buffers in
	 * the buffer descriptors.
	 */
	rbdf = (cbd_t *)&im->im_dprambase[dpaddr];
	rbdf->cbd_bufaddr = (uint) (rbdf+2);
	rbdf->cbd_sc = 0;
	tbdf = rbdf + 1;
	tbdf->cbd_bufaddr = ((uint) (rbdf+2)) + 1;
	tbdf->cbd_sc = 0;

 //       printf ("rbdf->cbd_sc = %x\n", rbdf->cbd_sc);
  //      printf ("tbdf->cbd_sc = %x\n", tbdf->cbd_sc);


	/* Set up the uart parameters in the parameter ram.
	*/
	up->smc_rbase = dpaddr;
	up->smc_tbase = dpaddr+sizeof(cbd_t);
	up->smc_rfcr = CPMFCR_EB;
	up->smc_tfcr = CPMFCR_EB;
	up->smc_brklen = 0;
	up->smc_brkec = 0;
	up->smc_brkcr = 0;

   //     printf ("up->smc_rbase = %x\n", up->smc_rbase);
    //    printf ("up->smc_tbase = %x\n", up->smc_tbase);
	/* Set UART mode, 8 bit, no parity, one stop.
	 * Enable receive and transmit.
	 */
	sp->smc_smcmr = smcr_mk_clen(9) |  SMCMR_SM_UART;

    //    printf ("smc_smcmr = %x\n", sp->smc_smcmr);

	/* Mask all interrupts and remove anything pending.
	*/
	sp->smc_smcm = 0;
	sp->smc_smce = 0xff;

	/* put the SMC channel into NMSI (non multiplexd serial interface)
	 * mode and wire either BRG7 to SMC1 or BRG8 to SMC1 (15-17).
	 */
	im->im_cpmux.cmx_smr = (im->im_cpmux.cmx_smr&~CMXSMR_MASK)|CMXSMR_VALUE;

     //   printf ("cmx_smr = %x\n", im->im_cpmux.cmx_smr);
	/* Set up the baud rate generator.
	*/
	smc1_serial_setbrg ();


	/* Make the first buffer the only buffer.
	*/
	rbdf->cbd_sc |= BD_SC_EMPTY | BD_SC_WRAP;
	tbdf->cbd_sc |= BD_SC_WRAP;

      //  printf ("rbdf->cbd_sc = %x\n", rbdf->cbd_sc);
       // printf ("tbdf->cbd_sc = %x\n", tbdf->cbd_sc);

	/* Single character receive.
	*/
	up->smc_mrblr = 1;
	up->smc_maxidl = 0;

	/* Initialize Tx/Rx parameters.
	*/
        //printf ("cp->cp_cpcr = %x\n", cp->cp_cpcr);
	while (cp->cp_cpcr & CPM_CR_FLG)  /* wait if cp is busy */
	  ;
        //printf ("cp->cp_cpcr = %x\n", cp->cp_cpcr);

	cp->cp_cpcr = mk_cr_cmd(CPM_CR_SMC_PAGE, CPM_CR_SMC_SBLOCK,
					0, CPM_CR_INIT_TRX) | CPM_CR_FLG;

       // printf ("cp->cp_cpcr = %x\n", cp->cp_cpcr);
	while (cp->cp_cpcr & CPM_CR_FLG)  /* wait if cp is busy */
	  ;
	
        //printf ("cp->cp_cpcr = %x\n", cp->cp_cpcr);
	/* Enable transmitter/receiver.
	*/
	sp->smc_smcmr |= SMCMR_REN | SMCMR_TEN;
        //printf ("smc_smcmr = %x\n", sp->smc_smcmr);

	//printf("dpaddr=0x%x,up=0x%x,rbdf=0x%x,tbdf=0x%x,sp=0x%x,cp=0x%x\n",dpaddr,up,rbdf,tbdf,sp,cp);

        //printf ("smcmr = %x  ,smce = %x  ,smcm = %x   , rstate = %x   tstate = %x  , brklen = %x  , brkec = %x  , brkcr  = %x   , rbdf__sc = %x , tbdf__sc = %x , rbdf_buf = %x , tbdf_buf = %x\n", sp->smc_smcmr, sp->smc_smce,sp->smc_smcm, up->smc_rstate ,up->smc_tstate , up->smc_brklen, up->smc_brkec,up->smc_brkcr,   rbdf->cbd_sc  , tbdf->cbd_sc, rbdf->cbd_bufaddr, tbdf->cbd_bufaddr);
	return (0);
}

int smc1_serial_init (void)
{
        volatile immap_t *im = (immap_t *)CFG_IMMR;
	volatile smc_t *sp;
	volatile smc_uart_t *up;
	volatile cbd_t *tbdf, *rbdf;
	volatile cpm8260_t *cp = &(im->im_cpm);
	uint	dpaddr;


	/* initialize pointers to SMC */

	sp = (smc_t *) &(im->im_smc[SMC_INDEX]);
	*(ushort *)(&im->im_dprambase[PROFF_SMC_BASE]) = PROFF_SMC;
	up = (smc_uart_t *)&im->im_dprambase[PROFF_SMC];

		

	/* Disable transmitter/receiver.
	*/
	sp->smc_smcmr &= ~(SMCMR_REN | SMCMR_TEN);

		

	/* NOTE: I/O port pins are set up via the iop_conf_tab[] table */

	/* Allocate space for two buffer descriptors in the DP ram.
	 * damm: allocating space after the two buffers for rx/tx data
	 */
		
	dpaddr = m8260_cpm_dpalloc((2 * sizeof (cbd_t)) + 2, 16);
        addr = dpaddr;

		

	/* Set the physical address of the host memory buffers in
	 * the buffer descriptors.
	 */
	rbdf = (cbd_t *)&im->im_dprambase[dpaddr];
	rbdf->cbd_bufaddr = (uint) (rbdf+2);
	//rbdf->cbd_bufaddr = (uint) rcvbuf;
	rbdf->cbd_sc = 0;
	tbdf = rbdf + 1;
	tbdf->cbd_bufaddr = ((uint) (rbdf+2)) + 1;
	//tbdf->cbd_bufaddr = (uint) sendbuf;
	tbdf->cbd_sc = 0;

	/* Set up the uart parameters in the parameter ram.
	*/
	up->smc_rbase = dpaddr;
	up->smc_tbase = dpaddr+sizeof(cbd_t);
	up->smc_rfcr = CPMFCR_EB;
	up->smc_tfcr = CPMFCR_EB;
	up->smc_brklen = 0;
	up->smc_brkec = 0;
	up->smc_brkcr = 0;

	/* Set UART mode, 8 bit, no parity, one stop.
	 * Enable receive and transmit.
	 */
	sp->smc_smcmr = smcr_mk_clen(9) |  SMCMR_SM_UART;

	/* Mask all interrupts and remove anything pending.
	*/
	sp->smc_smcm = 0;
	sp->smc_smce = 0xff;

	/* put the SMC channel into NMSI (non multiplexd serial interface)
	 * mode and wire either BRG7 to SMC1 or BRG8 to SMC1 (15-17).
	 */
	im->im_cpmux.cmx_smr = (im->im_cpmux.cmx_smr&~CMXSMR_MASK)|CMXSMR_VALUE;

	/* Set up the baud rate generator.
	*/
	smc1_serial_setbrg ();


	/* Make the first buffer the only buffer.
	*/
	tbdf->cbd_sc |= BD_SC_WRAP;
	rbdf->cbd_sc |= BD_SC_EMPTY | BD_SC_WRAP;

	/* Single character receive.
	*/
	up->smc_mrblr = 1;
	up->smc_maxidl = 0;

	/* Initialize Tx/Rx parameters.
	*/
	while (cp->cp_cpcr & CPM_CR_FLG)  /* wait if cp is busy */
	  ;

	cp->cp_cpcr = mk_cr_cmd(CPM_CR_SMC_PAGE, CPM_CR_SMC_SBLOCK,
					0, CPM_CR_INIT_TRX) | CPM_CR_FLG;

	while (cp->cp_cpcr & CPM_CR_FLG)  /* wait if cp is busy */
	  ;
	
	/* Enable transmitter/receiver.
	*/
	sp->smc_smcmr |= SMCMR_REN | SMCMR_TEN;

	//printf("dpaddr=0x%x,up=0x%x,rbdf=0x%x,tbdf=0x%x,sp=0x%x,cp=0x%x\n",dpaddr,up,rbdf,tbdf,sp,cp);

	return (0);
}

void
smc1_serial_putc(const char c)
{
        volatile immap_t *im = (immap_t *)CFG_IMMR;
	volatile smc_t *sp;
	volatile smc_uart_t *up;
	volatile cbd_t *tbdf, *rbdf;
	volatile cpm8260_t *cp = &(im->im_cpm);
	volatile unsigned char *buf;



	sp = (smc_t *) &(im->im_smc[SMC_INDEX]);
	*(ushort *)(&im->im_dprambase[PROFF_SMC_BASE]) = PROFF_SMC;
	up = (smc_uart_t *)&(im->im_dprambase[PROFF_SMC]);
	tbdf = (cbd_t *)&im->im_dprambase[up->smc_tbase];
	rbdf = (cbd_t *)&im->im_dprambase[up->smc_rbase];
	
        //printf ("smcmr = %x  ,smce = %x  ,smcm = %x   , rstate = %x   tstate = %x  , brklen = %x  , brkec = %x  , brkcr  = %x   , rbdf__sc = %x , tbdf__sc = %x , rbdf_buf = %x , tbdf_buf = %x\n", sp->smc_smcmr, sp->smc_smce,sp->smc_smcm, up->smc_rstate ,up->smc_tstate , up->smc_brklen, up->smc_brkec,up->smc_brkcr,   rbdf->cbd_sc  , tbdf->cbd_sc, rbdf->cbd_bufaddr, tbdf->cbd_bufaddr);


//        printf ("____________\n");

 //       printf ("tbdf->cbd_sc = %x\n", tbdf->cbd_sc);
	/* Wait for last character to go.
	*/
	buf = (char *)tbdf->cbd_bufaddr;
  //      printf ("tbdf->cdb_sc & BD_SC_READY = %x\n", tbdf->cbd_sc & BD_SC_READY);
	while (tbdf->cbd_sc & BD_SC_READY)
		;

   //     printf ("tbdf->cdb_sc & BD_SC_READY = %x\n", tbdf->cbd_sc & BD_SC_READY);
    //    printf ("tbdf->cbd_sc = %x\n", tbdf->cbd_sc);
	*buf = c;
	tbdf->cbd_datlen = 1;
	tbdf->cbd_sc |= BD_SC_READY;
	
        //printf ("smcmr = %x  ,smce = %x  ,smcm = %x   , rstate = %x   tstate = %x  , brklen = %x  , brkec = %x  , brkcr  = %x   , rbdf__sc = %x , tbdf__sc = %x , rbdf_buf = %x , tbdf_buf = %x\n", sp->smc_smcmr, sp->smc_smce,sp->smc_smcm, up->smc_rstate ,up->smc_tstate , up->smc_brklen, up->smc_brkec,up->smc_brkcr,   rbdf->cbd_sc  , tbdf->cbd_sc, rbdf->cbd_bufaddr, tbdf->cbd_bufaddr);

      //  printf ("rbdf->cbd_sc = %x\n", rbdf->cbd_sc);
}

void
smc1_serial_puts (const char *s)
{
	while (*s) {
		smc1_serial_putc (*s++);
	}
}

int
smc1_serial_getc(void)
{
        volatile immap_t *im = (immap_t *)CFG_IMMR;
	volatile smc_t *sp;
	volatile smc_uart_t *up;
	volatile cbd_t *tbdf, *rbdf;
	volatile cpm8260_t *cp = &(im->im_cpm);

	volatile unsigned char *buf, c;


	sp = (smc_t *) &(im->im_smc[SMC_INDEX]);
	*(ushort *)(&im->im_dprambase[PROFF_SMC_BASE]) = PROFF_SMC;
	up = (smc_uart_t *)&(im->im_dprambase[PROFF_SMC]);
       // printf ("***********************\n");

	tbdf = (cbd_t *)&im->im_dprambase[up->smc_tbase];
	rbdf = (cbd_t *)&im->im_dprambase[up->smc_rbase];
        //printf ("rbdf->cbd_sc = %x\n", rbdf->cbd_sc);

	/* Wait for character to show up.
	*/

        //printf ("smcmr = %x  ,smce = %x  ,smcm = %x   , rstate = %x   tstate = %x  , brklen = %x  , brkec = %x  , brkcr  = %x   , rbdf__sc = %x , tbdf__sc = %x , rbdf_buf = %x , tbdf_buf = %x\n", sp->smc_smcmr, sp->smc_smce,sp->smc_smcm, up->smc_rstate ,up->smc_tstate , up->smc_brklen, up->smc_brkec,up->smc_brkcr,   rbdf->cbd_sc  , tbdf->cbd_sc, rbdf->cbd_bufaddr, tbdf->cbd_bufaddr);


	buf = (unsigned char *)rbdf->cbd_bufaddr;
        //printf ("rbdf->cdb_sc & BD_SC_READY = %x\n", rbdf->cbd_sc & BD_SC_EMPTY);
	while (rbdf->cbd_sc & BD_SC_EMPTY)
		;
       // printf ("rbdf->cdb_sc & BD_SC_READY = %x\n", rbdf->cbd_sc & BD_SC_EMPTY);
        //printf ("rbdf->cbd_sc = %x\n", rbdf->cbd_sc);
        //printf ("*buf = %x\n", *buf);
	c = *buf;
	rbdf->cbd_sc |= BD_SC_EMPTY;
        //printf ("rbdf->cbd_sc = %x\n", rbdf->cbd_sc);
        
        //printf ("smcmr = %x  ,smce = %x  ,smcm = %x   , rstate = %x   tstate = %x  , brklen = %x  , brkec = %x  , brkcr  = %x   , rbdf__sc = %x , tbdf__sc = %x , rbdf_buf = %x , tbdf_buf = %x\n", sp->smc_smcmr, sp->smc_smce,sp->smc_smcm, up->smc_rstate ,up->smc_tstate , up->smc_brklen, up->smc_brkec,up->smc_brkcr,   rbdf->cbd_sc  , tbdf->cbd_sc, rbdf->cbd_bufaddr, tbdf->cbd_bufaddr);


	return(c);
}

int
smc1_serial_tstc()
{
	volatile cbd_t		*rbdf;
	volatile smc_uart_t	*up;
        volatile immap_t	*im = (immap_t *)CFG_IMMR;

	up = (smc_uart_t *)&(im->im_dprambase[PROFF_SMC]);

	rbdf = (cbd_t *)&im->im_dprambase[up->smc_rbase];

	return(!(rbdf->cbd_sc & BD_SC_EMPTY));
}

/* we use this so that we can do without the ctype library */
//#define is_digit(c)     ((c) >= '0' && (c) <= '9')

#if 0

#include <stdarg.h>
void printf (const char *fmt, ...)
{
	va_list args;
	uint i;
	char printbuffer[256];

	va_start (args, fmt);

	/* For this to work, printbuffer must be larger than
	 *          *          * anything we ever want to print.
	 *                   *                   */
	i = vsprintf (printbuffer, fmt, args);
	va_end (args);

	/* Print the string */
	serial_puts (printbuffer);
}
#endif



