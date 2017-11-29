include $(TOPDIR)/config.mk

LIB	= libapp.a

OBJS	= main.o spi_handle.o link_handle.o gps.o Starsensor.o

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

