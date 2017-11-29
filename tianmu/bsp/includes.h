/*
*********************************************************************************************************
*                                                uC/OS-II
*                                          The Real-Time Kernel
*                                     Freescale  MPC8349E Specific code
*
*                                 (c) Copyright 2007; Micrium; Weston, FL
*                                           All Rights Reserved
*
* File    : INCLUDES.H
* By      : Cedric Migliorini
*********************************************************************************************************
*/

//#include      <stdarg.h>

#if 0
typedef struct  global_data {
	unsigned long   baudrate;
	unsigned long   cpu_clk;        /* CPU clock in Hz!             */
	unsigned long   bus_clk;
	/* There are many clocks on the MPC8260 - see page 9-5 */
	unsigned long   vco_out;
	unsigned long   cpm_clk;
	unsigned long   scc_clk;
	unsigned long   brg_clk;
	unsigned long   ram_size;       /* RAM size */
	unsigned long   reloc_off;      /* Relocation Offset */
	unsigned long   reset_status;   /* reset status register at boot        */
	unsigned long   env_addr;       /* Address  of Environment struct       */
	unsigned long   env_valid;      /* Checksum of Environment valid?       */
	unsigned long   have_console;   /* serial_init() was called             */
	unsigned int    dp_alloc_base;
	unsigned int    dp_alloc_top;
} gd_t;
gd_t *ptr_gd;
#endif
#ifndef __MPC8260_INC__
#define __MPC8260_INC__

#include      <cpu.h>

//#include      <lib_def.h>
//#include      <lib_mem.h>
//#include      <lib_str.h>

//#include      <app_cfg.h>
#include      <ucos_ii.h>

//#include      "8349ITX_registers.h"
#include      <bsp.h>

#if (uC_PROBE_OS_PLUGIN > 0)
#include    <os_probe.h>
#endif

#if (uC_PROBE_COM_MODULE > 0)
#include    <probe_com.h>

#if (PROBE_COM_METHOD_RS232 > 0)
#include    <probe_rs232.h>
#endif
#endif // ENDIF (uC_PROBE_COM_MODULE)

#endif
