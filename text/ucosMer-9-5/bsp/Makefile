include $(TOPDIR)/config.mk


LIB	= lib8260.a

START	= cpusetup.o 
OBJS	= epic.o decr.o commproc.o serial_smc.o smc1.o smc2.o serial_scc.o scc4.o scc3.o scc2.o hdlc.o vsprintf.o uart_print.o

all:	.depend $(START) $(LIB)
#all:	 $(START) $(LIB)

$(LIB):	$(OBJS)
	$(AR) crv $@ $(OBJS) 

.depend:        Makefile $(START:.o=.S) $(OBJS:.o=.c)
	                $(CC) -M $(CFLAGS) $(START:.o=.S) $(OBJS:.o=.c) > $@


