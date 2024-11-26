# Makefile
include ../Makefile.common
include ../../../../../Makefile.common
include	../../../../Makefile.common

PACKAGEDIR	= ngat/liric/command
BINDIR 		= $(LIRIC_BIN_HOME)/java/$(PACKAGEDIR)
SRCS 		= Command.java BooleanReplyCommand.java DoubleReplyCommand.java IntegerReplyCommand.java \
		AbortCommand.java \
		ConfigFilterCommand.java ConfigNudgematicOffsetSizeCommand.java ConfigCoaddExposureLengthCommand.java \
		FitsHeaderAddCommand.java FitsHeaderClearCommand.java FitsHeaderDeleteCommand.java \
		MultrunCommand.java \
		ShutdownCommand.java \
		StatusExposureCountCommand.java StatusExposureIndexCommand.java StatusExposureCoaddCountCommand.java  \
		StatusExposureLengthCommand.java StatusExposureStartTimeCommand.java StatusExposureCoaddLengthCommand.java \
		StatusExposureMultrunCommand.java StatusExposureRunCommand.java StatusExposureStatusCommand.java \
		StatusFilterWheelFilterCommand.java StatusFilterWheelPositionCommand.java \
		StatusFilterWheelStatusCommand.java \
		StatusNudgematicPositionCommand.java StatusNudgematicStatusCommand.java \
		StatusNudgematicOffsetSizeCommand.java StatusNudgematicTemperatureCommand.java \
		StatusTemperatureGetCommand.java \
		MultbiasCommand.java MultdarkCommand.java

OBJS 		= $(SRCS:%.java=$(BINDIR)/%.class)

top: $(OBJS)

$(BINDIR)/%.class:%.java
	$(JAVAC) -sourcepath .:$(LIRIC_SRC_HOME)/java \
		-classpath $(LIRIC_BIN_HOME)/java:$(CLASSPATH) $(JAVAC_OPTIONS) $(JAVAC_VERSION_OPTIONS) -d $(LIRIC_BIN_HOME)/java $<

clean: tidy
	-$(RM) $(RM_OPTIONS) $(OBJS)

tidy:
	-$(RM) $(RM_OPTIONS) $(TIDY_OPTIONS)
