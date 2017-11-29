CROSS_COMPILE = /LinuxPPC/CDK/bin/powerpc-linux-
CC      = $(CROSS_COMPILE)gcc
AS      = $(CROSS_COMPILE)as
LD      = $(CROSS_COMPILE)ld
AR      = $(CROSS_COMPILE)ar
NM      = $(CROSS_COMPILE)nm
STRIP   = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
RANLIB  = $(CROSS_COMPILE)RANLIB
CFLAGS :=  -Os  -fno-builtin -ffixed-r14 -meabi -ffixed-r14 -meabi -D__KERNEL__ -DTEXT_BASE=0x0 -DCONFIG_PPC -D__powerpc__ -DCONFIG_8260 -ffixed-r2 -ffixed-r29 -mstring -I$(TOPDIR)/bsp -I$(TOPDIR)/ucos-ii/core603e -I$(TOPDIR)/ucos-ii/source -I$(TOPDIR)/app
#CFLAGS :=  -Os  -msoft-float -D__KERNEL__ -DTEXT_BASE=0x0 -DCONFIG_PPC -D__powerpc__ -DCONFIG_8260 -mstring -I$(TOPDIR)/bsp -I$(TOPDIR)/ucos-ii/core603e -I$(TOPDIR)/ucos-ii/source -I$(TOPDIR)/app
AFLAGS = -D__ASSEMBLY__ $(CFLAGS) # -DCONFIG_8260 -ffixed-r2 -ffixed-r29 -mstring 

%.s:    %.S
	        $(CPP) $(AFLAGS) -o $@ $(CURDIR)/$<
%.o:    %.S
	        $(CC) $(AFLAGS) -c -o $@ $(CURDIR)/$<
%.o:    %.c
	        $(CC) $(CFLAGS) -c -o $@ $<

