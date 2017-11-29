/*
 * file: leds.c
 *
 * PowerPC 8240  LED support for bank of 8 leds on Windriver sbc8240
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */
#include "includes.h"

static INT8U image;                         // shadow image of LED
//static INT8U *LEDs = (INT8U*)0xffe80000;    // physical address of LEDs
static INT8U *LEDs = (INT8U*)0xa00000;    // physical address of LEDs

/*
    \breif complements the specified LED bit - used by IRQ
*/ 
void IRQLedManipulate(INT8U hex)
{
    image ^= hex;
    *LEDs = image;
}
/*
    \breif complements the specified LED bit - use by tasks
*/ 
void LedManipulate(INT8U hex)
{
    MSR_SAVE;

    OS_ENTER_CRITICAL();
    image ^= hex;
    *LEDs = image;
    OS_EXIT_CRITICAL();
}


/* End of Source */
