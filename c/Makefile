include ../../Makefile.common
include ../Makefile.common
include ../detector/Makefile.common
include ../filter_wheel/Makefile.common
include ../nudgematic/Makefile.common

BINDIR			= $(LIRIC_BIN_HOME)/c/$(HOSTTYPE)
INCDIR 			= $(LIRIC_SRC_HOME)/include
DOCSDIR 		= $(LIRIC_DOC_HOME)/cdocs

DEBUG_CFLAGS		= -DLIRIC_DEBUG=10

# log_udp library
LOG_UDP_CFLAGS		= -I$(LOG_UDP_SRC_HOME)/include
LOG_UDP_LDFLAGS		= -l$(LOG_UDP_HOME)

# command_server library
COMMAND_SERVER_CFLAGS 	= -I$(COMMANDSERVER_SRC_HOME)/include
COMMAND_SERVER_LDFLAGS 	= -lcommandserver

# detector library
DETECTOR_CFLAGS 	= -I$(DETECTOR_SRC_HOME)/include
DETECTOR_LDFLAGS 	= -l$(DETECTOR_LIBNAME) $(XCLIB_LDFLAGS)

# filter wheel library
FILTER_WHEEL_CFLAGS 	= -I$(FILTER_WHEEL_SRC_HOME)/include
FILTER_WHEEL_LDFLAGS 	= -l$(FILTER_WHEEL_LIBNAME)

# Nudgematic library
NUDGEMATIC_CFLAGS 	= -I$(NUDGEMATIC_SRC_HOME)/include
NUDGEMATIC_LDFLAGS 	= -l$(NUDGEMATIC_LIBNAME) 

# USB-PIO I/O board library
#USB_PIO_CFLAGS 		= -I$(USB_PIO_SRC_HOME)/include
#USB_PIO_LDFLAGS 	= -l$(USB_PIO_LIBNAME) 

# CFITSIO
CFITSIO_CFLAGS		= -I$(CFITSIOINCDIR) 
CFITSIO_LDFLAGS		= -lcfitsio

# NGAT Astro MJD link informatiom
MJD_CFLAGS 		= -I${LT_SRC_HOME}/ngatastro/include
MJD_LDFLAGS		= -lngatastro

# estar_config
CONFIG_CFLAGS		= -I$(eSTAR_SRC_HOME)/config/include/
CONFIG_LDFLAGS		= -L$(eSTAR_LIB_HOME) -lestar_config

CFLAGS 			= -g -I$(INCDIR) $(DEBUG_CFLAGS) $(LOG_UDP_CFLAGS) \
				$(DETECTOR_CFLAGS) $(FILTER_WHEEL_CFLAGS) $(NUDGEMATIC_CFLAGS) \
				$(COMMAND_SERVER_CFLAGS) $(CFITSIO_CFLAGS) $(MJD_CFLAGS) $(CONFIG_CFLAGS)
# $(USB_PIO_CFLAGS) 
DOCFLAGS 		= -static

EXE_SRCS		= liric_main.c
OBJ_SRCS		= liric_general.c liric_config.c liric_server.c liric_fits_header.c liric_command.c \
			  liric_multrun.c liric_bias_dark.c



SRCS			= $(EXE_SRCS) $(OBJ_SRCS)
HEADERS			= $(OBJ_SRCS:%.c=$(INCDIR)/%.h)
EXE_OBJS		= $(EXE_SRCS:%.c=$(BINDIR)/%.o)
OBJS			= $(SRCS:%.c=$(BINDIR)/%.o)
EXES			= $(BINDIR)/liric
DOCS 			= $(SRCS:%.c=$(DOCSDIR)/%.html)
CONFIG_SRCS		= liric1.liric.c.properties liric2.liric.c.properties
CONFIG_BINS		= $(CONFIG_SRCS:%.properties=$(BINDIR)/%.properties)


top: $(EXES) $(CONFIG_BINS) docs

$(BINDIR)/liric: $(OBJS)
	$(CC) $^ -o $@  -L$(LT_LIB_HOME) $(COMMAND_SERVER_LDFLAGS) \
		$(DETECTOR_LDFLAGS) $(FILTER_WHEEL_LDFLAGS) $(NUDGEMATIC_LDFLAGS) \
		$(LOG_UDP_LDFLAGS)  $(CFITSIO_LDFLAGS)  $(MJD_LDFLAGS) \
		$(CONFIG_LDFLAGS) $(TIMELIB) $(SOCKETLIB) -lm -lc 
# $(USB_PIO_LDFLAGS)

$(BINDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@  

$(BINDIR)/%.properties: %.properties
	$(CP) $< $@  

docs: $(DOCS)

$(DOCS): $(SRCS)
	-$(CDOC) -d $(DOCSDIR) -h $(INCDIR) $(DOCFLAGS) $(SRCS)

$(DOCS) : $(SRCS)

depend:
	makedepend $(MAKEDEPENDFLAGS) -p$(BINDIR)/ -- $(CFLAGS) -- $(SRCS)

clean:
	$(RM) $(RM_OPTIONS) $(EXES) $(OBJS) $(TIDY_OPTIONS)

tidy:
	$(RM) $(RM_OPTIONS) $(TIDY_OPTIONS)

# DO NOT DELETE
