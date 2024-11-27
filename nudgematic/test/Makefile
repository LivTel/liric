# Makefile
include ../../../Makefile.common
include ../../usb_pio/Makefile.common
include ../../Makefile.common
include ../Makefile.common

INCDIR 		= $(NUDGEMATIC_SRC_HOME)/include
TESTDIR 	= test
BINDIR 		= $(NUDGEMATIC_BIN_HOME)/$(TESTDIR)/$(HOSTTYPE)
DOCSDIR 	= $(NUDGEMATIC_DOC_HOME)/$(TESTDIR)

# usb_pio
#USB_PIO_CFLAGS	= -I$(USB_PIO_SRC_HOME)/include
#USB_PIO_LDFLAGS = -l$(USB_PIO_LIBNAME)

# nudgematic
NUDGEMATIC_LDFLAGS = -l$(NUDGEMATIC_LIBNAME)

CFLAGS 		= -g -I$(INCDIR) $(LOG_UDP_CFLAGS)
# $(USB_PIO_CFLAGS) 
LDFLAGS		= -L$(LT_LIB_HOME) $(NUDGEMATIC_LDFLAGS)
# $(USB_PIO_LDFLAGS)

DOCFLAGS 	= -static

SRCS 		= nudgematic_send_command_low_level.c nudgematic_send_command.c nudgematic_test_position_set.c nudgematic_test_position_get.c  nudgematic_test_temperature_get.c
OBJS 		= $(SRCS:%.c=$(BINDIR)/%.o)
PROGS 		= $(SRCS:%.c=$(BINDIR)/%)
DOCS 		= $(SRCS:%.c=$(DOCSDIR)/%.html)
SCRIPT_SRCS	= 
SCRIPT_BINS	= $(SCRIPT_SRCS:%=$(BINDIR)/%)

top: $(PROGS) scripts docs

$(BINDIR)/%: $(BINDIR)/%.o
	$(CC) -o $@ $< $(TIMELIB) $(SOCKETLIB) -lm -lc $(LDFLAGS)

$(BINDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@  

scripts: $(SCRIPT_BINS)

$(BINDIR)/%.csh:%.csh
	$(CP) $< $@

$(BINDIR)/%:%
	$(CP) $< $@

docs: $(DOCS)

$(DOCS): $(SRCS)
	-$(CDOC) -d $(DOCSDIR) -h $(INCDIR) $(DOCFLAGS) $(SRCS)

depend:
	makedepend $(MAKEDEPENDFLAGS) -- $(CFLAGS) -- $(SRCS)

clean:
	$(RM) $(RM_OPTIONS) $(OBJS) $(PROGS) $(TIDY_OPTIONS)

tidy:
	$(RM) $(RM_OPTIONS) $(TIDY_OPTIONS)
