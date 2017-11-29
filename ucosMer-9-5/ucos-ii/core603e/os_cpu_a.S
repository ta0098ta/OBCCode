#
# file: os_cpu_a.s
#
# PowerPC assembly language routines for task switching & interrupts
# for UCOS-II
#
# Author: Ernie Price
# eprice@bitwise.net
#
#########################################################################
#include <ppc_asm.tmpl>

        .extern OSTCBHighRdy            /*pointer to highest priority ready task*/
        .extern OSTCBCur                /*pointer to current task's TCB*/
        .extern OSTaskSwHook
        .extern OSRunning
        .extern decTimeCst
        .extern OSIntEnter
        .extern OSIntExit
	.extern EIE_Hdlr
	.extern ExtIntHandle

        .global OSStartHighRdy
        .global OSCtxSw
        .global OSIntCtxSw
        .global DECIntr
        .global EIEIntr


        .text
        .align  2


#****************************************************************
#
#  match all of the following with stack frame in os_cpu_c.c
#
      	.equ  STK_FRM_SZ,  (39 * 4)        # 32 gpr + 6 spr + sp  
     	.equ  XTR_REG_OFS,  (33 * 4)
        .equ  R0_OFFS, 8
        .equ  R2_OFFS,  12

# Diab register usage

#r0              Scratch register.
#r1              Stack pointer.
#r2              Global pointer to the Small Constant Area (called SDA2 in EABI).
#r3 - r12        Temporary registers. Not preserved by functions. 
#r13             Global pointer to the Small Data Area (SDA) in EABI.
#r14 - r31       Preserved registers. Saved when used by functions.

    .macro SAVE_CONTEXT
        stwu    r1,-STK_FRM_SZ(r1)      # create stack frame
        stw     r0,R0_OFFS(r1)          # save r0
        stmw    r2,R2_OFFS(r1)          # save regs r2 through r31
        mflr    r31                     # fetch LR
        mfxer   r30                     # fetch XER
        mfctr   r29                     # fetch CTR
        mfsrr1  r28                     # fetch MSR (from normal interrupt)
        mfsrr0  r27                     # fetch PC  (from normal interrupt)
        mfcr    r26                     # fetch CR
        stmw    r26,XTR_REG_OFS(r1)     # save regs r26 - r31
        #mfmsr   r3
        #ori     r3, r3, 0x10            # turn on data translation
        #mtmsr   r3
    .endm

 .macro RESTORE_CONTEXT
        lmw     r26,XTR_REG_OFS(r1)     # restore regs r26 through r31
        mtcr    r26                     # restore CR
        mtsrr0  r27                     # restore PC
        mtsrr1  r28                     # restore MSR
        mtctr   r29                     # restore CTR
        mtxer   r30                     # restore XER
        mtlr    r31                     # restore LR
        lmw     r2,R2_OFFS(r1)          # restore regs r2 through r31
        lwz     r0,R0_OFFS(r1)          # restore r0
        lwz     r1,0(r1)                # restore stack pointer
        sync
        isync                   
        .endm
#########################################################################
#       Start multitasking
#       void OSStartHighRdy(void)
#########################################################################
.align  8
OSStartHighRdy:

	#bl      OSTaskSwHook                            # Call OSTaskSwHook
        li      r0,1
    	lis     r11,OSRunning@ha
    	stb     r0, OSRunning@l(r11)


	lis     r11,OSTCBHighRdy@ha                     # Get pointer to ready task TCB
	lwz     r11,OSTCBHighRdy@l(r11)      

	lwz     r1, 0(r11)                              # Get new stack pointer
        RESTORE_CONTEXT                     # Restore context - macro     
	rfi

	#b       join       


#########################################################################
#       Perform a context switch from task level
#       void OSCtxSw(void)                                      
#########################################################################
.align  8
OSCtxSw:
        SAVE_CONTEXT                        # Save context - macro
                
        lis r11,0x80
        lwz r12,0(r11)
	addi r12,r12,1
        #lis r11,0xa0
	stw r12,0(r11)

        #lwz     r11, OSTCBCur@sdax(r13)     # Get pointer to current TCB
        #stw     r1,0(r11)                   # Save stack pointer in current TCB

	lis     r11,OSTCBCur@ha              # Get pointer to current TCB
	lwz     r11,OSTCBCur@l(r11)
	stw     r1, 0(r11)                   # Save stack pointer in current TCB

join:
        bl      OSTaskSwHook                # Call OSTaskSwHook

        #lwz     r11,OSTCBHighRdy@sdax(r13)  # Get pointer to ready task TCB
        #stw     r11,OSTCBCur@sdax(r13)      # Save as current task TCB ptr.

    	lis     r11,OSTCBHighRdy@ha          # Get pointer to ready task TCB
    	lwz     r11,OSTCBHighRdy@l(r11)

    	lis     r12,OSTCBCur@ha              # Save as current task TCB ptr.
    	stw     r11,OSTCBCur@l(r12)


        #lbz     r10,OSPrioHighRdy@sdax(r13) # Get High Ready Priority
        #stb     r10,OSPrioCur@sdax(r13)     # Save as Current Priority

    	lis     r12,OSPrioHighRdy@ha                    # Get High Ready Priority
    	lbz     r10,OSPrioHighRdy@l(r12)

    	lis     r12,OSPrioCur@ha                        # Save as Current Priority
    	stb     r10,OSPrioCur@l(r12)
                                           
        lwz     r1,0(r11)                   # Get new stack pointer

        RESTORE_CONTEXT                     # Restore context - macro     

        rfi

#########################################################################
#       Perform a context switch from an ISR
#       void OSIntCtxSw(void)                                    
#########################################################################

.align  8
OSIntCtxSw:

        #addi    r1,r1,0x10                  # discard function on stack (OSInitExit)

        #lwz     r11, OSTCBCur@sdax(r13)     # Get pointer to current TCB
        #stw     r1,0(r11)                   # Save stack pointer in current TCB
    	#lis     r12,OSTCBCur@ha                         # Save as current task TCB ptr.
	#stw 	r1,OSTCBCur@l(r12)

        lis r11,0xa0
        lwz r12,4(r11)
	addi r12,r12,1
        #lis r11,0xa0
	stw r12,4(r11)

        bl      OSTaskSwHook                # Call OSTaskSwHook

        #lwz     r11,OSTCBHighRdy@sdax(r13)  # Get pointer to ready task TCB
        #stw     r11,OSTCBCur@sdax(r13)      # Save as current task TCB ptr.
    	lis     r11,OSTCBHighRdy@ha                     # Get pointer to ready task TCB
    	lwz     r11,OSTCBHighRdy@l(r11)

    	lis     r12,OSTCBCur@ha                         # Save as current task TCB ptr.
    	stw     r11,OSTCBCur@l(r12)


        #lbz     r10,OSPrioHighRdy@sdax(r13) # Get High Ready Priority
        #stb     r10,OSPrioCur@sdax(r13)     # Save as Current Priority
    	lis     r12,OSPrioHighRdy@ha                    # Get High Ready Priority
    	lbz     r10,OSPrioHighRdy@l(r12)

    	lis     r12,OSPrioCur@ha                        # Save as Current Priority
    	stb     r10,OSPrioCur@l(r12)

                                           
        lwz     r1,0(r11)                   # Get new stack pointer

        RESTORE_CONTEXT                     # Restore context - macro     

        rfi

#########################################################################
#       Decrementer Interrupt Handler
#       void DECIntr (void)
#########################################################################

        .extern IRQLedManipulate
.align  8
DECIntr:
#if 0
        xor r12,r12,r12
	ori r12,r12,0x55
        lis r11,0xa0
        stw r12,0(r11)
#endif

        SAVE_CONTEXT

        #lwz      r4, decTimeCst@sdax(r13)
        #lis     r4,decTimeCst@ha                         # Save as current task TCB ptr.
        #lwz     r4,decTimeCst@l(r4)
	lis	r4,0x2
	ori	r4,r4,0x8488

        #mfdec    r3                        # Read the decrementer
        #add.     r3, r3, r4                # compute new
        #mtdec    r3                        # load new timeout
	mtdec 	r4
	sync
	isync


    lis     r11,OSTaskStarting@ha                     # load OSIntNesting
    lbz     r3, OSTaskStarting@l(r11)

        lis r11,0xb0
        lwz r12,0(r11)
	addi r12,r12,1
        #lis r11,0xa0
	stw r12,4(r11)
	#b decexit
    cmpwi   r3, 1                                   # if (OSIntNesting != 1) ...
    bne decexit    

        #xor r12,r12,r12
	#ori r12,r12,0x55
        lis r11,0xb0
        lwz r12,0(r11)
	addi r12,r12,1
        #lis r11,0xa0
	stw r12,0(r11)


        bl       OSIntEnter

        bl       OSTimeTick		   # let OS count time

        #li      r3, 0x40
        #bl      IRQLedManipulate

        bl       OSIntExit
decexit:

        RESTORE_CONTEXT

        rfi

#########################################################################
#       External Interrupt Handler
#       void EIEIntr (void)
#########################################################################

#if 10
EIEIntr:
#ExtIntHandle:
#if 0
        xor r12,r12,r12
	ori r12,r12,0x55
        lis r11,0xa0
        #lwz r12,0(r11)
	#ori r12,r12,1
        #lis r11,0xa0
	stw r12,0(r11)
	rfi
	#b EIEIntr
#endif
        SAVE_CONTEXT

        bl              OSIntEnter

        bl              EIE_Hdlr        # external (or internal 8240) interrupt

        bl              OSIntExit

        RESTORE_CONTEXT

        rfi
#endif
        .end

# End of source
