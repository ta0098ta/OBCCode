include $(TOPDIR)/config.mk

LIB	= libapp.a

OBJS	= main.o interface.o hdlc_handle.o wod.o ADCS.o ADCS_TTC.o DLL_MIMU_RF.o gps.o LockLS.o ProgramContrl.o Starsensor.o MEMSGyro.o BDGPS.o 

#$(LIB): $(OBJS) $(SOBJS)
#	$(AR) crv $@ $^

$(LIB): .depend $(OBJS) 
	$(AR) crv $@ $(OBJS)

clean:
	rm -f $(SOBJS) $(OBJS)

distclean:      clean
	        rm -f $(LIB) core *.bak .depend

.depend:        Makefile $(OBJS:.o=.c)
	                $(CC) -M $(CFLAGS) $(OBJS:.o=.c) > $@

