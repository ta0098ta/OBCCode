#
# file: cpusetup.s
#
# PowerPC Startup - initialize BATs, PCI, cache
# for UCOS-II
#
# Author: Ernie Price
# eprice@bitwise.net
#
#########################################################################

#include <ppc_asm.tmpl>

	.data
	#.align 8


        .macro  LA reg,symbol
        lis     reg,%hiadj(symbol)
        addi    reg,reg,%lo(symbol)
        .endm
                
        .macro  LWI reg,symbol
        lis     reg,%hiadj(symbol)
        addi    reg,reg,%lo(symbol)
        .endm

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



########################################################################
#
#       Enter here from cpuvecs.s
#
.text
        .global bsp_Start
	.global int_disable
	.global enable_int
	.global SetDecrementer
	.global readtimer32
	.global readtime
	.global  CPU_IntDis
	.global  CPU_IntEn
	.global ExtIntHandle
	.global OSCtxSwHandle
	.global MachineCheck
	.global SystemManage
	.global DecIntHan
	.global OSIntCtxSw
	.global OSStartHighRdy

        #.extern Send, Sbase                     # from the linker command file
        .extern msrvalue
        .extern main

. = 0x100
 .globl  _start
bsp_Start:
	nop
	mfmsr r3
	mtsrr1  r3
	sync
	isync

#ClearBSS:

#
#       Now clear the C uninitialized data area
#
	lis 	r3,Sbase@ha
	addi	r3,r3,Sbase@l

	lis 	r4,Send@ha
	addi 	r4,r4,Send@l

	
        cmplw   0, r3, r4 
        beq     6f 
 
        li      r0, 0 
5: 
        stw     r0, 0(r3) 
        addi    r3, r3, 4 
        cmplw   0, r3, r4 
        bne     5b 
        #blt     5b 
6:

        #mfmsr   r3
	#xori	r3,r3,r3
        #ori     r3,r3,0xb002            # enable machine check and external exceptions
        #ori     r3,r3,0x3002            # enable machine check and external exceptions
	#isync
        #mtmsr   r3                      # 
	#sync

#
#       Now setup the stack and the Diab (also EABI) small data areas
#

	lis 	r3,0x20
	lis 	r4,0x30
	
        cmplw   0, r3, r4 
        beq     6f 
 
        li      r0, 0 
5: 
        stw     r0, 0(r3) 
        addi    r3, r3, 4 
        cmplw   0, r3, r4 
        bne     5b 
6:

        lis     1, 0x20                # initial stack at 1 mB - only temp til OS starts

	mflr	r0
        addis   13,0,_SDA_BASE_@ha    # Initialize r13 to sdata base, load top 16 bits
        #lis   13,_SDA_BASE_@ha    # Initialize r13 to sdata base, load top 16 bits
        addi    13,13,_SDA_BASE_@l    # (provided by linker).load bottom 16 bits
        addis   2,0,_SDA2_BASE_@ha    # Initialize r2 to sdata2 base
        #lis   2,_SDA2_BASE_@ha    # Initialize r2 to sdata2 base
        addi    2,2,_SDA2_BASE_@l     # (provided by linker).

        mfmsr   3
        #stw     r3, msrvalue@sdax(r13)  # partial MSR value for task stack initialization
        lis     11, msrvalue@ha          # Get pointer to ready task TCB
        stw     3, msrvalue@l(11)


#
# Insert other initialization code here.
#
        bl      main
        b       .                       # in case main() returns

##############################################################
#$PAGE^L
#*********************************************************************************************************
#*                                      DISABLE/ENABLE INTERRUPTS
#*
#* Description : Disable/Enable interrupts by setting or clearing the global interrupt mask in the cpu
#*               status register.
#*
#*                    void  CPU_IntDis (void)
#*                           Set global interrupt mask bit on MSR
#*                           Return
#*
#*                    void  CPU_IntEn (void)
#*                           Clear global interrupt mask bit on MSR
#*                           Return
#*********************************************************************************************************


. = 0x200
MachineCheck:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x200
	b exception_handle

        #ori r3,r3,0x51
        #lis r4,0xf001
        #lwz r3,0x0c04(r4)
	#xor r4,r4,r4
	#lis r4,0x50
	#stw r3,0(r4)
        xor r3,r3,r3
        ori r3,r3,0x56
        lis r4,0x50
        stw r3,0(r4)
	rfi

. = 0x300
MachineCheck1:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x300
	b exception_handle

        xor r3,r3,r3
        ori r3,r3,0x57
        lis r4,0x50
        stw r3,0(r4)
	rfi

. = 0x400
MachineCheck2:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x400
	b exception_handle

        mfsrr0  r27                     # fetch PC  (from normal interrupt)
        mfsrr1  r28                     # fetch MSR (from normal interrupt)
	mfmsr	r29
        xor r3,r3,r3
        ori r3,r3,0x58
        lis r4,0x50
	stw r3,0(r4)
        stw r27,4(r4)
        stw r28,8(r4)
        stw r29,0xc(r4)
	rfi

. = 0x500
ExtIntHandle:

        SAVE_CONTEXT
        
	//lis     11, stkpv@ha          # Get pointer to ready task TCB
	//stw     1, stkpv@l(11)

        //lis     r11,OSTaskStarting@ha                     # load OSIntNesting
        //lbz     r3, OSTaskStarting@l(r11)
        //cmpwi   r3, 1                                   # if (OSTaskStarting!= 1) ...
        //bne ExtIntExit


	lis     r11,OSIntNesting@ha                     # load OSIntNesting
	lbz     r3, OSIntNesting@l(r11)
	addi    r0, r3, 1
	stb     r0, OSIntNesting@l(r11)                 # OSIntNesting++

	cmpwi   r0, 1                                   # if (OSIntNesting != 1) ...
	bne     OSExtIntISR_NotSaveSP1                   # do not save the stackpointer

	lis     r11,OSTCBCur@ha                         # Get pointer to current TCB
	lwz     r11,OSTCBCur@l(r11)
	stw     r1, 0(r11)                              # Save stack pointer in current TCB

OSExtIntISR_NotSaveSP1:                                                                                                        

	bl 	BSP_ProcessorISR

	bl       OSIntExit

ExtIntExit:
	RESTORE_CONTEXT
	rfi

. = 0x600
MachineCheck3:
        #xor r3,r3,r3
        #ori r3,r3,0x59
        #lis r4,0x50
        #stw r3,0(r4)
	#rfi
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x600
	b exception_handle


. = 0x700
MachineCheck4:
        #xor r3,r3,r3
        #ori r3,r3,0x60
        #lis r4,0x50
        #stw r3,0(r4)
        #stw r27,4(r4)
        #stw r28,8(r4)
        #stw r29,0xc(r4)
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x700
	b exception_handle
	rfi

. = 0x800
SystemManage:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x800
	b exception_handle
        xor r3,r3,r3
        ori r3,r3,0x53
        lis r4,0x50
        stw r3,0(r4)
	rfi

. = 0x900
DecIntHan:

        SAVE_CONTEXT

        #lwz      r4, decTimeCst@sdax(r13)
        #lis     r4,decTimeCst@ha                         # Save as current task TCB ptr.
        #lwz     r4,decTimeCst@l(r4)
	#lis	r4,0x2
	#ori	r4,r4,0x8488

        lis     11, stkpv@ha          # Get pointer to ready task TCB
        stw     1, stkpv@l(11)

	lis	r4,0x3
	ori	r4,r4,0xd090

        #mfdec    r3                        # Read the decrementer
        #add.     r3, r3, r4                # compute new
        #mtdec    r3                        # load new timeout
	mtdec 	r4
	sync
	isync
	bl DEC_TEST
    	lis     r11,OSTaskStarting@ha                     # load OSIntNesting
    	lbz     r3, OSTaskStarting@l(r11)

        #lis r11,0xb0
        #lwz r12,0(r11)
	#addi r12,r12,1
	#stw r12,4(r11)
    	cmpwi   r3, 1                                   # if (OSIntNesting != 1) ...
    	bne decexit    

        #lis r11,0xb0
        #lwz r12,0(r11)
	#addi r12,r12,1
	#stw r12,0(r11)

    lis     r11,OSIntNesting@ha                     # load OSIntNesting
    lbz     r3, OSIntNesting@l(r11)
    addi    r0, r3, 1
    stb     r0, OSIntNesting@l(r11)                 # OSIntNesting++

    cmpwi   r0, 1                                   # if (OSIntNesting != 1) ...
    bne     OSExtIntISR_NotSaveSP                   # do not save the stackpointer

    lis     r11,OSTCBCur@ha                         # Get pointer to current TCB
    lwz     r11,OSTCBCur@l(r11)
    stw     r1, 0(r11)                              # Save stack pointer in current TCB

OSExtIntISR_NotSaveSP:                                                                                                        


        #bl       OSIntEnter

        bl       OSTimeTick		   # let OS count time

        #li      r3, 0x40
        #bl      IRQLedManipulate

        bl       OSIntExit
decexit:

        RESTORE_CONTEXT

        rfi

. = 0xC00
OSCtxSwHandle:
        #xor r3,r3,r3
        #ori r3,r3,0x55
        #lis r4,0xa0
	#lwz r3,0(r4)
	#ori r3,r3,1
        #stw r3,0(r4)
	#rfi
        SAVE_CONTEXT                        # Save context - macro
                
        #lis r11,0x80
        #lwz r12,0(r11)
	#addi r12,r12,1
	#stw r12,0(r11)

        #lwz     r11, OSTCBCur@sdax(r13)     # Get pointer to current TCB
        #stw     r1,0(r11)                   # Save stack pointer in current TCB

	lis     r11,OSTCBCur@ha              # Get pointer to current TCB
	lwz     r11,OSTCBCur@l(r11)
	stw     r1, 0(r11)                   # Save stack pointer in current TCB

join:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
        #bl      TOSTaskSwHook                # Call OSTaskSwHook

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

	#lis	r3,OSCtxSw@ha
	#ori	r3,r3,OSCtxSw@l
        #mtlr    r3
        #blrl
. = 0xd00
MachineCheck5:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0xd00
	b exception_handle

        xor r3,r3,r3
        ori r3,r3,0x62
        lis r4,0x50
        stw r3,0(r4)
	rfi

. = 0xe00
MachineCheck6:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0xe00
	b exception_handle

. = 0x1000
MachineCheck7:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x1000
	b exception_handle

. = 0x1100
MachineCheck8:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x1100
	b exception_handle

. = 0x1200
MachineCheck9:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x1200
	b exception_handle

. = 0x1300
MachineCheck10:
        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5
	li	r6,0x1300
	b exception_handle

#if 0
EIEIntr:
        xor r12,r12,r12
        ori r12,r12,0x55
        lis r11,0xa0
        #lwz r12,0(r11)
        #ori r12,r12,1
        #lis r11,0xa0
        stw r12,0(r11)
	rfi
#endif
int_disable:
#if 10
    mfmsr   r3
    mfmsr   r5                              # Preparation for disabling interrupts (1)
    andi.   r5,r5,0x7fff                    # Preparation for disabling interrupts : set EE (bit #16) to '0' (2)
    #sync
    #isync
    mtmsr   r5                              # Disable interrupts
    sync
    isync
#endif
#if 0
        mfmsr   r3                     /* Disable interrupts */
	mr 	r4,r3
        li      r5,0
        ori     r5,r5,0x8000
        andc    r4,r4,r5
        sync                            /* Some chip revs need this... */
        mtmsr   r4
#endif
        #sync
    	blr

/*    	blr*/
enable_int:
	mtmsr   r3
	sync
	isync
    	blr

SetDecrementer:
	mtdec  r3
	isync
	blr
	/*lis r15,0x22*/
	#mfdec r15
redo:
        #mfspr   r15,22
	mfdec r15
	lis 11,0xa0
	stw r15,0(11)
	#mtspr 22,r3
	b redo
	blr
readtime:
	#mfdec r3
	mfmsr r3
	#mfdec r3
	blr
readtimer32:
    	mfspr   3, 268
    	blr
CPU_IntDis:
    mfmsr   r5                              # Preparation for disabling interrupts (1)
    andi.   r5,r5,0x7fff                    # Preparation for disabling interrupts : set EE (bit #16) to '0' (2)
    mtmsr   r5                              # Disable interrupts
    sync
    blr


CPU_IntEn:
    mfmsr   r5                              # Preparation for enabling interrupts (1)
    ori     r5,r5,0x8000                    # Preparation for enabling interrupts : set EE (bit #16) to '1'(2)
    sync
    isync 
    mtmsr   r5                              # Enable interrupts
    sync
    isync
    blr

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

#########################################################################
#       Perform a context switch from an ISR
#       void OSIntCtxSw(void)                                    
#########################################################################

OSIntCtxSw:
        lis     11, stkpv@ha          # Get pointer to ready task TCB
        stw     1, stkpv@l(11)

        #addi    r1,r1,0x10                  # discard function on stack (OSInitExit)

        #lwz     r11, OSTCBCur@sdax(r13)     # Get pointer to current TCB
        #stw     r1,0(r11)                   # Save stack pointer in current TCB
    	#lis     r12,OSTCBCur@ha                         # Save as current task TCB ptr.
	#stw 	r1,OSTCBCur@l(r12)
	sync
	isync

        mfsrr0  r3                     # fetch PC  (from normal interrupt)
        mfsrr1  r4                     # fetch MSR (from normal interrupt)
	mfmsr	r5

        #bl      INTOSTaskSwHook                # Call OSTaskSwHook

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



        .end 
 
