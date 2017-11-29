/*
 * file: decr.c
 *
 * PowerPC 8240  decrementer support 
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */
#include "includes.h"

INT32U  decTimeCst=165000;           // public so interrupt routines can use it

/*
asm void SetDecrementer (INT32U decValue)
{
%reg decValue;
    mtdec       decValue
}
*/

#define CFG_HZ                  1000    /* decrementer freq: 1 ms ticks */    
/*
    \breif initializes the PowerPC decrementer
*/ 
void decr_Start (INT32S ticks_per_second)
{
    //decTimeCst = TMBCLKS_PER_SEC / ticks_per_second;
    //decTimeCst = 165000;
    //SetDecrementer(decTimeCst); 
    //*(volatile unsigned int *)0xa00000=0x11;
    //SetDecrementer(165000); 
     gd_t * gd=ptr_gd;
    decTimeCst = (gd->bus_clk / 4) / CFG_HZ;
    SetDecrementer(decTimeCst); 
}


/* End of Source */
