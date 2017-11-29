/*
 * file: os_cpu.h
 *
 * PowerPC Header for port to 603 core
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */

#ifndef OS_CPU_H
#define OS_CPU_H

#define URP(x) (void)x;

//
//  The following structure and macros allow reverse-endian operations
//  to and from memory and I/O addresses
//



#define OS_CRITICAL_METHOD 3        // 3 does not work yet

/* Macros */ 

#if OS_CRITICAL_METHOD == 3
//#define disable_int(x) x=int_disable()
/*
asm unsigned long int_disable()
{
    mfmsr       r3                  // fetch MSR to reg x
    rlwinm      r0,r3,0,17,15       // reset EE bit
    mtmsr       r0                  // turn off EE in MSR, if it was on
    xor         r3,r3,r0            // isolate original EE bit
}*/

/*
    Complement to disable above.    Restores EE bit to its previous state.
    This does not reenable interrupts when the caller may have had them
    disabled.  Overhead: 3 micro instructions
*/
/*
asm enable_int(x)
{
% reg x
    mfmsr       r0                  // get current MSR
    or          r0,r0,x             // merge in prior state
    mtmsr       r0                  // put back in MSR
}
*/

#define MSR_SAVE    INT32U cpu_sr
#define OS_ENTER_CRITICAL()  { cpu_sr = int_disable(); } //disable_int(cpu_sr)
#define OS_EXIT_CRITICAL()  { enable_int(cpu_sr); } //enable_int(cpu_sr)

#else

#define disable_int() int_disable()
asm void int_disable()
{
    mfmsr       r3                  // fetch MSR to reg x
    rlwinm      r0,r3,0,17,15       // reset EE bit
    mtmsr       r0                  // turn off EE in MSR, if it was on
}
/*
    Complement to disable above.    Restores EE bit to its previous state.
    This does not reenable interrupts when the caller may have had them
    disabled.  Overhead: 3 micro instructions
*/
asm enable_int()
{
    mfmsr       r0                  // get current MSR
    ori         r0,r0,0x8000        // merge in prior state
    mtmsr       r0                  // put back in MSR
}

#define MSR_SAVE    (void)0
#define OS_ENTER_CRITICAL() disable_int()
#define OS_EXIT_CRITICAL()  enable_int()

#endif

#define OS_CPU_SR   INT32U

#define OS_TASK_SW()        asm (" sc "); //asm("   sc");
//#define OS_TASK_SW()        OSCtxSw()

#define OS_STK_GROWTH       1   /* Stack grows from HIGH to LOW memory on PPC  */

#ifndef  FALSE
#define  FALSE    0
#endif

#ifndef  TRUE
#define  TRUE     1
#endif

/* Typedefs */ 

/* 
 * The following #define's are the UCOS-II typedefs
 */ 

typedef unsigned char  BOOLEAN;
typedef unsigned char  INT8U;                    /* Unsigned  8 bit quantity                           */
typedef signed   char  INT8S;                    /* Signed    8 bit quantity                           */
typedef unsigned short INT16U;                   /* Unsigned 16 bit quantity                           */
typedef signed   short INT16S;                   /* Signed   16 bit quantity                           */
typedef unsigned long  INT32U;                   /* Unsigned 32 bit quantity                           */
typedef signed   long  INT32S;                   /* Signed   32 bit quantity                           */
typedef float          FP32;                     /* Single precision floating point                    */
typedef double         FP64;                     /* Double precision floating point                    */
typedef unsigned long  OS_STK;                   /* Each stack entry is 16-bit wide                    */
#ifndef __utype_defined
#define __utype_defined
typedef INT8U	u_char;
typedef INT16U  ushort;
typedef INT32U  uint;
typedef INT32U  ulong;
#endif

void  OSCtxSw(void);
void  OSIntCtxSw(void);
void  OSStartHighRdy(void);


#endif // OS_CPU_H

/* End of Source */
