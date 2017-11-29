/*
 * file: bsp.h
 *
 * PowerPC Header for Board Support routines
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */
#ifndef BSP_H
#define BSP_H

//#include <includes.h>
#define EUMB_BASE    0xfce00000
#define CONFIG_CONS_ON_SCC      1
//#define CONFIG_CONS_ON_SMC      1
#define CONFIG_CONS_INDEX       1
#define UART_BAUD_RATE		9600 //115200
#define CFG_IMMR                0xf0000000
//#define CONFIG_8260_CLKIN       100000000
//#define CONFIG_8260_CLKIN       66000000
#define CONFIG_8260_CLKIN       33000000



void IRQLedManipulate(INT8U hex);
void LedManipulate(INT8U hex);

void i2c_Enable(void);
void i2c_SendChar(INT8U);
void i2c_Init(void);
void decr_Start (INT32S ticks_per_sec);
//
//  Interrupt Support
//
//  Note that UART0 and UART1 are on the 8241/8245 - they aren't part of the 8240
//
enum IRQ_VECTORS     // hardware layout - do not change order
{  
        IRQ0, IRQ1, IRQ2, IRQ3, IRQ4,
        I2C,  DMA0, DMA1, dum1, dum2, MSGU, dum3, dum4, UART0, UART1,
        TMR0, TMR1, TMR2, TMR3,
        MAX_HANDLERS
};

void epic_AddHandler(
                INT32U   id,             // Interrupt vector number
                void     (*handler) (),  // Pointer to Interrupt Handler Function
                INT32U   arg             // Argument to pass to the Handler
                );
void epic_DisbleInterrupt(INT32U vector);
void epic_EnableInterrupt(INT32U vector);
void epic_InitEPIC(void);
void serial_puts (const char *s);
//
//  End Interrupt Support
//
void DelayNusec (INT32U delayTime);
void DelayNmsec (INT32U delayTime);

// The decrementer count rate with a 100 mhz peripheral clock.
#define TMBCLKS_PER_SEC     25000000

extern INT32U dec_init;

INT32U readtimer32();
void OSCtxSw();
void DECIntr(); 
void EIEIntr();
void ExtIntHandle();
void OSCtxSwHandle();
void MachineCheck();
void SystemManage();
void DecIntHan();
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

/*
   {
   asm("
   mfspr       3, 268"
   );
   }
   */
#endif // BSP_H

/* End of Source */
