#
# file: cpuvecs.s
#
# PowerPC interrupt and trap vectors 
# for UCOS-II
#
# Author: Ernie Price
# eprice@bitwise.net
#
#########################################################################
        .text
        .align 2

        .extern bsp_Start
#
#       WindRiver visionPROBE/convert/visionClick does not like .org statements.
#       With .space it works but does not properly show source
#
setorigin       macro   ofst
        .space ofst - (. - codebase)
        endm

vector  macro   ofst, action, label
        setorigin ofst
        if NARG > 2
label:
        .endif
        b       action
        endm

        
codebase:

        vector  0x100, bsp_Start, poreset       # POR - bsp_Start in bspsetup.s
        
        vector  0x200, .                # Machine Check
                                        
        vector  0x300, .                # DSI Exception

        vector  0x400, .                # ISI Exception

        vector  0x500, EIEIntr          # process external interrupt

        vector  0x600, .                # Alignment Exception

        vector  0x700, .                # Program Exception 

        vector  0x800, .                # Floating Point - not available

        vector  0x900, DECIntr          # decrementer interrupt in os_cpu_a.s

        vector  0xa00, .                # Reserved

        vector  0xb00, .                # Reserved

        vector  0xC00, OSCtxSw          # System Call 

        vector  0xD00, .                # Trace Exception

        vector  0xE00, .                # Floating Point assist N/A on 8240

        vector  0xF00, .                # Reserved

        vector  0x1000, .               # Instruction Translation Miss

        vector  0x1100, .               # Data Translation Miss - load

        vector  0x1200, .               # Data Translation Miss - store

        vector  0x1300, .               # Instruction Breakpoint

        vector  0x1400, .               # System Management

        setorigin 0x2000                # some padding

        .end
