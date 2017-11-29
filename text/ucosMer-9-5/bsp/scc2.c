/*
 * (C) Copyright 2000
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
 * Hacked for MPC8260 by Murray.Jensen@cmst.csiro.au, 19-Oct-00.
 */

/*
 * Minimal serial functions needed to use one of the SCC ports
 * as serial console interface.
 */

#include <includes.h>
#include <mpc8260.h>
#include <cpm_8260.h>

#define SCC_INDEX2		1
#define CMXSCR_MASK2		(CMXSCR_GR2|CMXSCR_SC2|\
					CMXSCR_RS2CS_MSK|CMXSCR_TS2CS_MSK)
#define CMXSCR_VALUE2		(CMXSCR_RS2CS_BRG2|CMXSCR_TS2CS_BRG2)

	void
scc2_serial_setbrg (void)
{
	gd_t * gd=ptr_gd;

	//m8260_cpm_setbrg(SCC_INDEX2, gd->baudrate);
	m8260_cpm_setbrg(SCC_INDEX2, 9600);
}


int scc2_serial_init (void)
{
        volatile immap_t *im = (immap_t *)CFG_IMMR;
	volatile scc_t *sp;
	volatile scc_uart_t *up;
	volatile cbd_t *tbdf, *rbdf;
	volatile cpm8260_t *cp = &(im->im_cpm);
	uint	dpaddr;

	/* initialize pointers to SCC */

	sp = (scc_t *) &(im->im_scc[SCC_INDEX2]);
	up = (scc_uart_t *)&im->im_dprambase[PROFF_SCC2];

	/* Disable transmitter/receiver.
	*/
	sp->scc_gsmrl &= ~(SCC_GSMRL_ENR | SCC_GSMRL_ENT);

	/* put the SCC channel into NMSI (non multiplexd serial interface)
	 * mode and wire the selected SCC Tx and Rx clocks to BRGx (15-15).
	 */
	im->im_cpmux.cmx_scr = (im->im_cpmux.cmx_scr&~CMXSCR_MASK2)|CMXSCR_VALUE2;

	/* Set up the baud rate generator.
	*/
	scc2_serial_setbrg ();

	/* Allocate space for two buffer descriptors in the DP ram.
	 * damm: allocating space after the two buffers for rx/tx data
	 */

	dpaddr = m8260_cpm_dpalloc((2 * sizeof (cbd_t)) + 2, 16);

	/* Set the physical address of the host memory buffers in
	 * the buffer descriptors.
	 */
	rbdf = (cbd_t *)&im->im_dprambase[dpaddr];
	rbdf->cbd_bufaddr = (uint) (rbdf+2);
	rbdf->cbd_sc = BD_SC_EMPTY | BD_SC_WRAP;
	tbdf = rbdf + 1;
	tbdf->cbd_bufaddr = ((uint) (rbdf+2)) + 1;
	tbdf->cbd_sc = BD_SC_WRAP;

	/* Set up the uart parameters in the parameter ram.
	*/
	up->scc_genscc.scc_rbase = dpaddr;
	up->scc_genscc.scc_tbase = dpaddr+sizeof(cbd_t);
	up->scc_genscc.scc_rfcr = CPMFCR_EB;
	up->scc_genscc.scc_tfcr = CPMFCR_EB;
	up->scc_genscc.scc_mrblr = 1;
	up->scc_maxidl = 0;
	up->scc_brkcr = 1;
	up->scc_parec = 0;
	up->scc_frmec = 0;
	up->scc_nosec = 0;
	up->scc_brkec = 0;
	up->scc_uaddr1 = 0;
	up->scc_uaddr2 = 0;
	up->scc_toseq = 0;
	up->scc_char1 = up->scc_char2 = up->scc_char3 = up->scc_char4 = 0x8000;
	up->scc_char5 = up->scc_char6 = up->scc_char7 = up->scc_char8 = 0x8000;
	up->scc_rccm = 0xc0ff;

	/* Mask all interrupts and remove anything pending.
	*/
	sp->scc_sccm = 0;
	sp->scc_scce = 0xffff;

	/* Set 8 bit FIFO, 16 bit oversampling and UART mode.
	*/
	sp->scc_gsmrh = SCC_GSMRH_RFW;	/* 8 bit FIFO */
	sp->scc_gsmrl = \
		SCC_GSMRL_TDCR_16 | SCC_GSMRL_RDCR_16 | SCC_GSMRL_MODE_UART;

	/* Set CTS flow control, 1 stop bit, 8 bit character length,
	 * normal async UART mode, no parity
	 */
	sp->scc_psmr = SCU_PSMR_FLC | SCU_PSMR_CL;

	/* execute the "Init Rx and Tx params" CP command.
	*/

	while (cp->cp_cpcr & CPM_CR_FLG)  /* wait if cp is busy */
	  ;

	cp->cp_cpcr = mk_cr_cmd(CPM_CR_SCC2_PAGE, CPM_CR_SCC2_SBLOCK,
					0, CPM_CR_INIT_TRX) | CPM_CR_FLG;

	while (cp->cp_cpcr & CPM_CR_FLG)  /* wait if cp is busy */
	  ;

	/* Enable transmitter/receiver.
	*/
	sp->scc_gsmrl |= SCC_GSMRL_ENR | SCC_GSMRL_ENT;

	return (0);
}

void
scc2_serial_putc(const char c)
{
	volatile scc_uart_t	*up;
	volatile cbd_t		*tbdf;
        volatile immap_t	*im;
#if 0
	if (c == '\n')
		scc2_serial_putc ('\r');
#endif
        im = (immap_t *)CFG_IMMR;
	up = (scc_uart_t *)&im->im_dprambase[PROFF_SCC2];
	tbdf = (cbd_t *)&im->im_dprambase[up->scc_genscc.scc_tbase];

	/* Wait for last character to go.
	 */
	while (tbdf->cbd_sc & BD_SC_READY)
		;

	/* Load the character into the transmit buffer.
	 */
	*(volatile char *)tbdf->cbd_bufaddr = c;
	tbdf->cbd_datlen = 1;
	tbdf->cbd_sc |= BD_SC_READY;
}

void
scc2_serial_puts (const char *s)
{
	while (*s) {
		scc2_serial_putc (*s++);
	}
}

int
scc2_serial_getc(void)
{
	volatile cbd_t		*rbdf;
	volatile scc_uart_t	*up;
        volatile immap_t	*im;
	unsigned char		c;

        im = (immap_t *)CFG_IMMR;
	up = (scc_uart_t *)&im->im_dprambase[PROFF_SCC2];
	rbdf = (cbd_t *)&im->im_dprambase[up->scc_genscc.scc_rbase];

	/* Wait for character to show up.
	 */
	while (rbdf->cbd_sc & BD_SC_EMPTY)
		;

	/* Grab the char and clear the buffer again.
	 */
	c = *(volatile unsigned char *)rbdf->cbd_bufaddr;
	rbdf->cbd_sc |= BD_SC_EMPTY;

	return (c);
}

int
scc2_serial_tstc()
{
	volatile cbd_t		*rbdf;
	volatile scc_uart_t	*up;
        volatile immap_t	*im;

        im = (immap_t *)CFG_IMMR;
	up = (scc_uart_t *)&im->im_dprambase[PROFF_SCC2];
	rbdf = (cbd_t *)&im->im_dprambase[up->scc_genscc.scc_rbase];

	return ((rbdf->cbd_sc & BD_SC_EMPTY) == 0);
}


