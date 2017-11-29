#
# file: mmu_cache.s
#
# PowerPC Setup the BATS and cache 
# for UCOS-II
#
# Author: Ernie Price
# eprice@bitwise.net
#
#########################################################################
        .text
        .align  2

        .global BAT_Enable
        .global InitSRs
        .global ClearBATs
        .global IcachEnable
        .global DcachEnable


HID0    =       1008
        HID0_ICE     = 0x00008000
        HID0_DCE     = 0x00004000
        HID0_ILOCK   = 0x00002000
        HID0_DLOCK   = 0x00001000
        HID0_ICFI    = 0x00000800
        HID0_DCFI    = 0x00000400

#
#       We enable Block Address Translation so that we can have data and/or instruction
#       caching active but not in regions where there are physical devices.  The mapping
#       is one to one in that we directly map logical to physical addresses for
#       both instruction and data.
#
BAT_0_VAL       .equ    0x00000000      # DRAM for a max of 256 Mb
BAT_1_VAL       .equ    0xe0000000      # PCI mapping (can extend into 0xfxxxxxx)
BAT_2_VAL       .equ    0x70000000      # Extended ROM regions on 8241/8245
BAT_3_VAL       .equ    0xf0000000      # ROM and some 8-bit devices for 256 MB

DBATS = 1                               # 1 == use data address translation
IBATS = 0                               # 1 == use inst address translation

PS128K  .equ 000000000000b
PS256K  .equ 000000000001b
PS512K  .equ 000000000011b
PS1M    .equ 000000000111b
PS2M    .equ 000000001111b
PS4M    .equ 000000011111b
PS8M    .equ 000000111111b
PS16M   .equ 000001111111b
PS32M   .equ 000011111111b
PS64M   .equ 000111111111b
PS128M  .equ 001111111111b
PS256M  .equ 011111111111b
PA_W    .equ 01000000b                  # Write through
PA_I    .equ 00100000b                  # Inhibit caching
PA_M    .equ 00010000b                  # Memory Coherency
PA_G    .equ 00001000b                  # Guarded
NO_ACCESS       .equ 00b
READ_ONLY       .equ 01b
READ_WRITE      .equ 10b

        macro BATVALUE  eabase, size, realbase, attrib
        .uword          eabase   | (size << 2) | 3
        .uword          realbase | attrib | READ_WRITE
        endm

        macro           loadbat whichbat
        lwzu            r4, 4(r3)
        mtspr           whichbat&&U, r4
        lwzu            r4, 4(r3)
        mtspr           whichbat&&L, r4
        endm

###############################################################################
#
#       Load the Block Address Translation registers
#
BAT_Enable:
        mflr    r30

        bl      batload         # branch around the values getting the
                                # address of the values in LR

        .uword  0               # dummy because lwzu increments first
.if DBATS == 1
        BATVALUE        BAT_0_VAL, PS256M, BAT_0_VAL, PA_M
        BATVALUE        BAT_1_VAL, PS256M, BAT_1_VAL, <PA_I | PA_G>
        BATVALUE        BAT_2_VAL, PS256M, BAT_2_VAL, <PA_I | PA_G>
        BATVALUE        BAT_3_VAL, PS256M, BAT_3_VAL, <PA_I | PA_G>
.endif
.if IBATS == 1
        BATVALUE        BAT_0_VAL, PS256M, BAT_0_VAL, 0
        BATVALUE        BAT_1_VAL, PS256M, BAT_1_VAL, PA_I
        BATVALUE        BAT_2_VAL, PS256M, BAT_2_VAL, PA_I
        BATVALUE        BAT_3_VAL, PS256M, BAT_3_VAL, PA_I
.endif

batload:
        mflr    r3

        tvalue  .set 0
.if DBATS == 1
        tvalue .set  tvalue | 0x10      # enable Data Translation
        loadbat         DBAT0
        loadbat         DBAT1
        loadbat         DBAT2
        loadbat         DBAT3
.endif
.if IBATS == 1
        tvalue .set tvalue | 0x20       # enable Instruction Translation
        loadbat         IBAT0
        loadbat         IBAT1
        loadbat         IBAT2
        loadbat         IBAT3
.endif

.if tvalue 
        mfmsr   r3
        ori     r3,r3, tvalue           # Enable translation
        mtmsr   r3
.endif

        mtlr            r30
        blr

###############################################################################
#
#       Clear the Block Address Translation registers
#
        .align  2
ClearBATs:

batld   macro   wbat
        mtspr   wbat&&U, r3     
#       sync
        mtspr   wbat&&L, r3     
#       sync
        endm

        li      r3, 0
        
        batld   IBAT0
        batld   IBAT1
        batld   IBAT2
        batld   IBAT3
        batld   DBAT0
        batld   DBAT1
        batld   DBAT2
        batld   DBAT3
#
        blr

###############################################################################
#
#       Initialize the Segment Registers
#
InitSRs:
#
sreg    macro   reg
        mtsr    reg, r3
#       sync
        addi    r3, r3, 1
        endm

        lis     r3, 0x2000              # SR0 value

        sreg    sr0                     # Load SR0
        sreg    sr1                     # Load SR1
        sreg    sr2                     # Load SR2
        sreg    sr3                     # Load SR3
        sreg    sr4                     # Load SR4
        sreg    sr5                     # Load SR5
        sreg    sr6                     # Load SR6
        sreg    sr7                     # Load SR7
        sreg    sr8                     # Load SR8
        sreg    sr9                     # Load SR9
        sreg    sr10                    # Load SR10
        sreg    sr11                    # Load SR11
        sreg    sr12                    # Load SR12
        sreg    sr13                    # Load SR13
        sreg    sr14                    # Load SR14
        sreg    sr15                    # Load SR15

        blr

###############################################################################
#
#       Common initialize macro for enabling cache
#
cacheEnab       macro   enab, inval, lock

        mfspr   r3, HID0                # get HID0
        andi.   r4, r3, enab            # if it is already ON
        bne     done\@                  #    branch to done

        li      r4, inval               # invalidate bit
        or      r5, r3, r4              # merge invalidate bit to r5
        andc    r4, r3, r4              # reset invalidate bit in r4

        isync
        mtspr   HID0, r5                # write HID0 with inval on

        isync
        mtspr   HID0, r4                # write HID0 with inval off

        li      r3, lock                # lock bit
        andc    r3, r4, r3              # reset lock bit
        isync
        mtspr   HID0, r3                # write HID0 with lock off

        ori     r3, r3, enab
        isync
        mtspr   HID0, r3                # with enab on
done\@:
        blr
        endm

###############################################################################
#
#       Enable instruction caching
#
IcachEnable:

        cacheEnab       HID0_ICE, HID0_ICFI, HID0_ILOCK

###############################################################################
#
#       Enable data caching
#
DcachEnable:

        cacheEnab       HID0_DCE, HID0_DCFI, HID0_DLOCK

