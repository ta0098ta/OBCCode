include $(TOPDIR)/config.mk

LIB	= lib603e.a

OBJS	:= os_cpu_c.o 
#SOBJS   := os_cpu_a.o

#$(LIB): $(OBJS) $(SOBJS)
#	$(AR) crv $@ $^

$(LIB): $(OBJS) $(SOBJS)
	$(AR) crv $@ $(OBJS) $(SOBJS)

clean:
	rm -f $(SOBJS) $(OBJS)

distclean:      clean
	        rm -f $(LIB) core *.bak .depend

