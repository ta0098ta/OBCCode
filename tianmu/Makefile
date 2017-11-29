TOPDIR  := $(shell if [ "$$PWD" != "" ]; then echo $$PWD; else pwd; fi)
export  TOPDIR
CROSS_COMPILE = /LinuxPPC/CDK/bin/powerpc-linux-
#CROSS_COMPILE = /opt/crosstool/gcc-4.1.0-glibc-2.3.6/powerpc-603-linux-gnu/bin/powerpc-603-linux-gnu-
#CROSS_COMPILE = /LinuxPPC/toolchain/gcc-3.4.5-glibc-2.3.6/powerpc-603-linux-gnu/bin/powerpc-603-linux-gnu-
CC      = $(CROSS_COMPILE)gcc
AS      = $(CROSS_COMPILE)as
LD      = $(CROSS_COMPILE)ld
AR      = $(CROSS_COMPILE)ar
NM      = $(CROSS_COMPILE)nm
STRIP   = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
RANLIB  = $(CROSS_COMPILE)RANLIB
TEXT_BASE = 0x0

OBJS = bsp/cpusetup.o
LIBS = bsp/lib8260.a
LIBS += ucos-ii/source/libucos.a
LIBS += ucos-ii/core603e/lib603e.a
#LIBS +=/LinuxPPC/CDK/powerpc-linux/lib/libc.a
#LIBS +=/opt/crosstool/gcc-4.1.0-glibc-2.3.6/powerpc-603-linux-gnu/powerpc-603-linux-gnu/lib/libc.a
#LIBS +=/LinuxPPC/CDK/powerpc-linux/lib/libm.a
#LIBS +=/LinuxPPC/CDK/powerpc-linux/lib/mylibm.a
LIBS += /home/huangtuo/work/fdlibm/libm.a
#LIBS += /mnt/hgfs/work/TM/ucosTM/fdlibm/libm.a
#LIBS +=/opt/crosstool/gcc-4.1.0-glibc-2.3.6/powerpc-603-linux-gnu/powerpc-603-linux-gnu/lib/libm.a
LIBS += app/libapp.a
LDSCRIPT := bootlink.lds 
LDFLAGS += -Bstatic -T $(LDSCRIPT) -Ttext $(TEXT_BASE)
#LDFLAGS += -T $(LDSCRIPT) -Ttext $(TEXT_BASE)

SUBDIRS = bsp \
	  ucos-ii/core603e \
	  ucos-ii/source \
	  app \
	  fdlibm

#ppcboot.bin:	ppcboot
#		$(OBJCOPY) ${OBJCFLAGS} -O binary $< $@

#ppcboot.dis:	ppcboot
#		$(OBJDUMP) -d $< > $@

%.o:	%.s
	$(CC) $(AFLAGS) -c -o $@ $(CURDIR)/$<
%.o:	%.c
	$(CC) $(CFLAGS) -c -o $@ $<

ppcboot.bin:    ppcboot
	                $(OBJCOPY) ${OBJCFLAGS} -O binary $< $@
			cp -f ppcboot.bin /mnt/hgfs/share/ucos-zwj.bin

ppcboot:	subdirs $(OBJS) $(LIBS) $(LDSCRIPT)
	       $(LD) $(LDFLAGS) $(OBJS) $(LIBS) $(LIBS) -Map ppcboot.map -L /LinuxPPC/CDK/powerpc-linux/lib/gcc-lib/powerpc-linux/2.95.2/ -lgcc -o ppcboot
subdirs:
		@for dir in $(SUBDIRS) ; do $(MAKE) -C $$dir || exit 1 ; done


clean:
	find . -type f \
		\( -name 'core' -o -name '*~' \
		-o -name '*.o'  -o -name '*.a'  \) -print \
		| xargs rm -f

