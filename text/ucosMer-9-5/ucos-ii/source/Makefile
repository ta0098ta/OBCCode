include $(TOPDIR)/config.mk

LIB	= libucos.a

OBJS	= os_core.o os_flag.o os_mem.o os_q.o os_task.o os_tmr.o \
os_dbg_r.o os_mbox.o os_mutex.o os_sem.o os_time.o ucos_ii.o

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

