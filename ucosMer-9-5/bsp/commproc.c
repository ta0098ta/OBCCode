/*
 * This file is based on "arch/ppc/8260_io/commproc.c" - here is it's
 * copyright notice:
 *
 * General Purpose functions for the global management of the
 * 8260 Communication Processor Module.
 * Copyright (c) 1999 Dan Malek (dmalek@jlc.net)
 * Copyright (c) 2000 MontaVista Software, Inc (source@mvista.com)
 *	2.3.99 Updates
 *
 * In addition to the individual control of the communication
 * channels, there are a few functions that globally affect the
 * communication processor.
 *
 * Buffer descriptors must be allocated from the dual ported memory
 * space.  The allocator for that is here.  When the communication
 * process is reset, we reclaim the memory available.  There is
 * currently no deallocator for this memory.
 */
#include <includes.h>
#include <cpm_8260.h>
#include <mpc8260.h>

/*
 * because we have stack and init data in dual port ram
 * we must reduce the size
 */
#undef	CPM_DATAONLY_SIZE
#define CPM_DATAONLY_SIZE	((uint)(8 * 1024) - CPM_DATAONLY_BASE)

gd_t gd_data;

void * memset(void * s,int c,uint count)
{
	char *xs = (char *) s;

	while (count--)
		*xs++ = c;

	return s;
}

#define __stringify_1(x)        #x
#define __stringify(x)          __stringify_1(x)

#define mfspr(rn)       ({unsigned int rval; \
		                        asm volatile("mfspr %0," __stringify(rn) \
						                                     : "=r" (rval)); rval;})
#define mtspr(rn, v)    asm volatile("mtspr " __stringify(rn) ",%0" : : "r" (v))

#define HID0 0x3F0
unsigned int hid0;

void cpu_reboot()
{
	volatile immap_t *immr = (immap_t *)CFG_IMMR;
        immr->im_clkrst.car_rmr |= 0x01;
        hid0 = mfspr(HID0);
        hid0 |= 0x30000000;
        mtspr(HID0, hid0);

}
void m8260_cpm_reset(void)
{
	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	volatile ulong count;
	gd_t *gd;
	ptr_gd = &gd_data;
	gd = ptr_gd;

	/* Reclaim the DP memory for our use.
	*/
	gd->dp_alloc_base = CPM_DATAONLY_BASE + 1024;//8 * sizeof (cbd_t);
	gd->dp_alloc_top = gd->dp_alloc_base + CPM_DATAONLY_SIZE;
	gd->baudrate = UART_BAUD_RATE;//9600;

#if 10
	/*
	 * Reset CPM
	 */
	immr->im_cpm.cp_cpcr = CPM_CR_RST;
	count = 0;
	do {			/* Spin until command processed		*/
		__asm__ __volatile__ ("eieio");
	} while ((immr->im_cpm.cp_cpcr & CPM_CR_FLG) && ++count < 1000000);
#endif

}
/* Bus-to-Core Multiplier */
#define _1x     2
#define _1_5x   3
#define _2x     4
#define _2_5x   5
#define _3x     6
#define _3_5x   7
#define _4x     8
#define _4_5x   9
#define _5x     10
#define _5_5x   11
#define _6x     12
#define _6_5x   13
#define _7x     14
#define _7_5x   15
#define _8x     16
#define _byp    -1
#define _off    -2
#define _unk    -3

typedef struct {
	int b2c_mult;
	int vco_div;
	char *freq_60x;
	char *freq_core;
} corecnf_t;

/*
 *  * this table based on "Errata to MPC8260 PowerQUICC II User's Manual",
 *   * Rev. 1, 8/2000, page 10.
 *    */

corecnf_t corecnf_tab[] = {
	{ _1_5x,  4, " 33-100", " 33-100" },    /* 0x00 */
	{   _1x,  4, " 50-150", " 50-150" },    /* 0x01 */
	{   _1x,  8, " 25-75 ", " 25-75 " },    /* 0x02 */
	{  _byp, -1, "  ?-?  ", "  ?-?  " },    /* 0x03 */
	{   _2x,  2, " 50-150", "100-300" },    /* 0x04 */
	{   _2x,  4, " 25-75 ", " 50-150" },    /* 0x05 */
	{ _2_5x,  2, " 40-120", "100-240" },    /* 0x06 */
	{ _4_5x,  2, " 22-65 ", "100-300" },    /* 0x07 */
	{   _3x,  2, " 33-100", "100-300" },    /* 0x08 */
	{ _5_5x,  2, " 18-55 ", "100-300" },    /* 0x09 */
	{   _4x,  2, " 25-75 ", "100-300" },    /* 0x0A */
	{   _5x,  2, " 20-60 ", "100-300" },    /* 0x0B */
	{ _1_5x,  8, " 16-50 ", " 16-50 " },    /* 0x0C */
	{   _6x,  2, " 16-50 ", "100-300" },    /* 0x0D */
	{ _3_5x,  2, " 30-85 ", "100-300" },    /* 0x0E */
	{  _off, -1, "  ?-?  ", "  ?-?  " },    /* 0x0F */
	{   _3x,  4, " 16-50 ", " 50-150" },    /* 0x10 */
	{ _2_5x,  4, " 20-60 ", " 50-120" },    /* 0x11 */
	{ _6_5x,  2, " 15-46 ", "100-300" },    /* 0x12 */
	{  _byp, -1, "  ?-?  ", "  ?-?  " },    /* 0x13 */
	{   _7x,  2, " 14-43 ", "100-300" },    /* 0x14 */
	{   _2x,  4, " 25-75 ", " 50-150" },    /* 0x15 */
	{ _7_5x,  2, " 13-40 ", "100-300" },    /* 0x16 */
	{ _4_5x,  2, " 22-65 ", "100-300" },    /* 0x17 */
	{  _unk, -1, "  ?-?  ", "  ?-?  " },    /* 0x18 */
	{ _5_5x,  2, " 18-55 ", "100-300" },    /* 0x19 */
	{   _4x,  2, " 25-75 ", "100-300" },    /* 0x1A */
	{   _5x,  2, " 20-60 ", "100-300" },    /* 0x1B */
	{   _8x,  2, " 12-38 ", "100-300" },    /* 0x1C */
	{   _6x,  2, " 16-50 ", "100-300" },    /* 0x1D */
	{ _3_5x,  2, " 30-85 ", "100-300" },    /* 0x1E */
	{  _off, -1, "  ?-?  ", "  ?-?  " },    /* 0x1F */
};
#define CFG_SCCR                0x00000000
int get_clocks (void)
{
	gd_t *gd = ptr_gd;

	volatile immap_t *immap = (immap_t *) CFG_IMMR;
	ulong clkin;
	ulong sccr, dfbrg;
	ulong scmr, corecnf, busdf, cpmdf, plldf, pllmf;
	corecnf_t *cp;

#if !defined(CONFIG_8260_CLKIN)
#error clock measuring not implemented yet - define CONFIG_8260_CLKIN
#else
	clkin = CONFIG_8260_CLKIN;
#endif

	//immap->im_clkrst.car_sccr = CFG_SCCR;
	sccr = immap->im_clkrst.car_sccr;
	dfbrg = (sccr & SCCR_DFBRG_MSK) >> SCCR_DFBRG_SHIFT;

	scmr = immap->im_clkrst.car_scmr;
	corecnf = (scmr & SCMR_CORECNF_MSK) >> SCMR_CORECNF_SHIFT;
	busdf = (scmr & SCMR_BUSDF_MSK) >> SCMR_BUSDF_SHIFT;
	cpmdf = (scmr & SCMR_CPMDF_MSK) >> SCMR_CPMDF_SHIFT;
	plldf = (scmr & SCMR_PLLDF) ? 1 : 0;
	pllmf = (scmr & SCMR_PLLMF_MSK) >> SCMR_PLLMF_SHIFT;

	cp = &corecnf_tab[corecnf];

	gd->vco_out = (clkin * 2 * (pllmf + 1)) / (plldf + 1);


	gd->cpm_clk = gd->vco_out / 2;
	gd->bus_clk = clkin;
	gd->scc_clk = gd->vco_out / 4;
	gd->brg_clk = gd->vco_out / (1 << (2 * (dfbrg + 1)));
	//printf("brg_clk=%d,scc_clk=%d,bus_clk=%d\n",gd->brg_clk,gd->scc_clk,gd->bus_clk);
	//gd->brg_clk = 37500000;

	if (cp->b2c_mult > 0) {
		gd->cpu_clk = (clkin * cp->b2c_mult) / 2;
	} else {
		gd->cpu_clk = clkin;
	}

	return (0);
}

/* Allocate some memory from the dual ported ram.
 * To help protocols with object alignment restrictions, we do that
 * if they ask.
 */
uint
m8260_cpm_dpalloc(uint size, uint align)
{
	gd_t *gd = ptr_gd;

	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	uint	retloc;
	uint	align_mask, off;
	uint	savebase;
	
	align_mask = align - 1;
	savebase = gd->dp_alloc_base;


	if ((off = (gd->dp_alloc_base & align_mask)) != 0)
		gd->dp_alloc_base += (align - off);

	if ((off = size & align_mask) != 0)
		size += align - off;
	
	if ((gd->dp_alloc_base + size) >= gd->dp_alloc_top) {
		gd->dp_alloc_base = savebase;
		printf("m8260_cpm_dpalloc: ran out of dual port ram!");
	}
	
	retloc = gd->dp_alloc_base;
	gd->dp_alloc_base += size;

	memset((void *)&immr->im_dprambase[retloc], 0, size);


	return(retloc);
}

/* We also own one page of host buffer space for the allocation of
 * UART "fifos" and the like.
 */
uint
m8260_cpm_hostalloc(uint size, uint align)
{
	/* the host might not even have RAM yet - just use dual port RAM */
	return (m8260_cpm_dpalloc(size, align));
}

/* Set a baud rate generator.  This needs lots of work.  There are
 * eight BRGs, which can be connected to the CPM channels or output
 * as clocks.  The BRGs are in two different block of internal
 * memory mapped space.
 * The baud rate clock is the system clock divided by something.
 * It was set up long ago during the initial boot phase and is
 * is given to us.
 * Baud rate clocks are zero-based in the driver code (as that maps
 * to port numbers).  Documentation uses 1-based numbering.
 */
#define BRG_INT_CLK	gd->brg_clk
#define BRG_UART_CLK	((BRG_INT_CLK + 15) / 16)

/* This function is used by UARTS, or anything else that uses a 16x
 * oversampled clock.
 */
void
m8260_cpm_setbrg(uint brg, uint rate)
{
	gd_t *gd = ptr_gd;

	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	volatile uint	*bp;

	/* This is good enough to get SMCs running.....
	*/
	if (brg < 4) {
		bp = (uint *)&immr->im_brgc1;
	}
	else {
		bp = (uint *)&immr->im_brgc5;
		brg -= 4;
	}
	bp += brg;
	*bp = (((((BRG_UART_CLK+rate-1)/rate)-1)&0xfff)<<1)|CPM_BRG_EN;
//	if(brg==0)
//		printf("____bp=0x%x,0x%x\n",bp,*bp);
	//*bp = 0x00010046;
}

/* This function is used to set high speed synchronous baud rate
 * clocks.
 */
void
m8260_cpm_fastbrg(uint brg, uint rate, int div16)
{
	gd_t *gd = ptr_gd;

	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	volatile uint	*bp;

	/* This is good enough to get SMCs running.....
	*/
	if (brg < 4) {
		bp = (uint *)&immr->im_brgc1;
	}
	else {
		bp = (uint *)&immr->im_brgc5;
		brg -= 4;
	}
	bp += brg;
	*bp = (((((BRG_INT_CLK+rate-1)/rate)-1)&0xfff)<<1)|CPM_BRG_EN;
	if (div16)
		*bp |= CPM_BRG_DIV16;
}

/* This function is used to set baud rate generators using an external
 * clock source and 16x oversampling.
 */

void
m8260_cpm_extcbrg(uint brg, uint rate, uint extclk, int pinsel)
{
	volatile immap_t *immr = (immap_t *)CFG_IMMR;
	volatile uint	*bp;

	if (brg < 4) {
		bp = (uint *)&immr->im_brgc1;
	}
	else {
		bp = (uint *)&immr->im_brgc5;
		brg -= 4;
	}
	bp += brg;
	*bp = ((((((extclk/16)+rate-1)/rate)-1)&0xfff)<<1)|CPM_BRG_EN;
	if (pinsel == 0)
		*bp |= CPM_BRG_EXTC_CLK3_9;
	else
		*bp |= CPM_BRG_EXTC_CLK5_15;
}

#ifdef CONFIG_POST

void post_word_store (ulong a)
{
	volatile ulong *save_addr =
		(volatile ulong *)(CFG_IMMR + CPM_POST_WORD_ADDR);

	*save_addr = a;
}

ulong post_word_load (void)
{
	volatile ulong *save_addr =
		(volatile ulong *)(CFG_IMMR + CPM_POST_WORD_ADDR);

	return *save_addr;
}

#endif	/* CONFIG_POST */
