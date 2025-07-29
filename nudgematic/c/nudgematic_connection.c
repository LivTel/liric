/* nudgematic_connection.c
** Nudgematic mechanism library
*/
/**
 * Routines handling connecting to the Nudgematic mechanism..
 * @author Chris Mottram
 * @version $Revision$
 */
/**
 * Define Default Source to get BSD prototypes, including cfmakeraw.
 */
/*#define _BSD_SOURCE*/
#define _DEFAULT_SOURCE
/**
 * This hash define is needed before including source files give us POSIX.4/IEEE1003.1b-1993 prototypes.
 */
#define _POSIX_SOURCE 1
/**
 * This hash define is needed before including source files give us POSIX.4/IEEE1003.1b-1993 prototypes.
 */
#define _POSIX_C_SOURCE 199309L

#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <stdarg.h>
#include <unistd.h>
#ifdef MUTEXED
#include <pthread.h>
#endif

#include "log_udp.h"

#include "nudgematic_general.h"
#include "nudgematic_connection.h"

/* hash defines */
/**
 * A timeout (in seconds), how long we wait for the Arduino to return a complete line.
 */
#define CONNECTION_READ_LINE_TIMEOUT_S  (10.0)

/* internal data structures */
/**
 * Structure holding information on the conenction to the Arduino Mega.
 * <dl>
 * <dt>Serial_Fd</dt> <dd>The serial file descriptor for communications with the Arduino Mega.</dd>
 * <dt>Serial_Options_Saved</dt> <dd>The saved set of serial options.</dd>
 * <dt>Serial_Options</dt> <dd>The set of serial options configured.</dd>
 * <dt></dt> <dd></dd>
 * </dl>
 */
struct Nudgematic_Connection_Struct
{
	int Serial_Fd;
	struct termios Serial_Options_Saved;
	struct termios Serial_Options;
};

/* internal variables */
/**
 * Revision Control System identifier.
 */
static char rcsid[] = "$Id$";
/**
 * Instance of the structure holding information on the connection to the Arduino Mega.
 * @see #Nudgematic_Connection_Struct
 */
static struct Nudgematic_Connection_Struct Nudgematic_Connection_Data =
{
	-1,{0,0,0,0,' ',{' '},0,0},{0,0,0,0,' ',{' '},0,0}
};

/**
 * Variable holding error code of last operation performed.
 */
static int Connection_Error_Number = 0;
/**
 * Local variable holding description of the last error that occured.
 * @see nudgematic_general.html#NUDGEMATIC_GENERAL_ERROR_STRING_LENGTH
 */
static char Connection_Error_String[NUDGEMATIC_GENERAL_ERROR_STRING_LENGTH] = "";

/* --------------------------------------------------------
** External Functions
** -------------------------------------------------------- */
/**
 * Routine to open a connection to the Nudgematic. 
 * @param device_name A string representing the serial device filename presented by the plugged in Arduino Mega 
 *                    (e.g. "/dev/ttyACM0").
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Nudgematic_Connection_Data
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Log_Format
 */
int Nudgematic_Connection_Open(const char* device_name)
{
	int open_errno,retval,file_flags;

	if(device_name == NULL)
	{
		Connection_Error_Number = 1;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Open:device_name was NULL.");
		return FALSE;
	}
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_TERSE,
				      "Nudgematic_Connection_Open: Opening connection todevice name '%s'.",device_name);
#endif /* LOGGING */
	/* open the serial connection, read/write, not a controlling terminal, don't wait for DCD signal line. */
	Nudgematic_Connection_Data.Serial_Fd = open(device_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if(Nudgematic_Connection_Data.Serial_Fd < 0)
	{
		open_errno = errno;
		Connection_Error_Number = 2;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Open:Device '%s' failed to open (%d).",
			device_name,open_errno);
		return FALSE;
		
	}
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,
				      "Nudgematic_Connection_Open: Using Serial FD %d.",Nudgematic_Connection_Data.Serial_Fd);
#endif /* LOGGING */
	/* make non-blocking */
	file_flags = fcntl(Nudgematic_Connection_Data.Serial_Fd, F_GETFL);
	file_flags |= FNDELAY;
	retval = fcntl(Nudgematic_Connection_Data.Serial_Fd, F_SETFL, file_flags);
	if(retval != 0)
	{
		open_errno = errno;
		close(Nudgematic_Connection_Data.Serial_Fd);
		Connection_Error_Number = 3;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Open:fcntl set flags (%#x) failed (%d).",
			file_flags,open_errno);
		return FALSE;
	}
	/* get current serial options */
	retval = tcgetattr(Nudgematic_Connection_Data.Serial_Fd,&(Nudgematic_Connection_Data.Serial_Options_Saved));
	if(retval != 0)
	{
		open_errno = errno;
		Connection_Error_Number = 5;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Open: tcgetattr failed (%d).",open_errno);
		return FALSE;
	}
	/* initialise new serial options */
	bzero(&(Nudgematic_Connection_Data.Serial_Options), sizeof(Nudgematic_Connection_Data.Serial_Options));
	/* set control flags and baud rate */
	/*Nudgematic_Connection_Data.Serial_Options.c_cflag |= B19200 | CS8 | CLOCAL | CREAD;*/
	/* Try running the Nudgematic at a lower baud rate to stop kernel panics (fault 2856) . */
	Nudgematic_Connection_Data.Serial_Options.c_cflag |= B9600 | CS8 | CLOCAL | CREAD;
#ifdef CNEW_RTSCTS
	Nudgematic_Connection_Data.Serial_Options.c_cflag &= ~CNEW_RTSCTS;/* disable flow control */
#endif
#ifdef CRTSCTS
	Nudgematic_Connection_Data.Serial_Options.c_cflag &= ~CRTSCTS;/* disable flow control */
#endif
	/* select raw input, not line input. */
	Nudgematic_Connection_Data.Serial_Options.c_lflag = 0;
	/* ignore parity errors */
	Nudgematic_Connection_Data.Serial_Options.c_iflag = IGNPAR;
	/* set raw output */
	Nudgematic_Connection_Data.Serial_Options.c_oflag = 0;
	/* blocking read until 0 char arrives */
	Nudgematic_Connection_Data.Serial_Options.c_cc[VMIN]=0;
	Nudgematic_Connection_Data.Serial_Options.c_cc[VTIME]=0;	
#if LOGGING > 9
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,
	      "Nudgematic_Connection_Open:New Attr:Input:%#x,Output:%#x,Local:%#x,Control:%#x,Min:%c,Time:%c.",
		       Nudgematic_Connection_Data.Serial_Options.c_iflag,Nudgematic_Connection_Data.Serial_Options.c_oflag,
		       Nudgematic_Connection_Data.Serial_Options.c_lflag,Nudgematic_Connection_Data.Serial_Options.c_cflag,
		       Nudgematic_Connection_Data.Serial_Options.c_cc[VMIN],Nudgematic_Connection_Data.Serial_Options.c_cc[VTIME]);
#endif /* LOGGING */
 	/* set new options */
#if LOGGING > 5
	Nudgematic_General_Log(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Open:Setting serial options.");
#endif /* LOGGING */
	tcflush(Nudgematic_Connection_Data.Serial_Fd, TCIFLUSH);
	retval = tcsetattr(Nudgematic_Connection_Data.Serial_Fd,TCSANOW,&(Nudgematic_Connection_Data.Serial_Options));
	if(retval != 0)
	{
		open_errno = errno;
		Connection_Error_Number = 6;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Open: tcsetattr failed (%d).",open_errno);
		return FALSE;
	}
	/* re-get current serial options to see what was set */
#if LOGGING > 5
	Nudgematic_General_Log(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Open:Re-Getting new serial options.");
#endif /* LOGGING */
	retval = tcgetattr(Nudgematic_Connection_Data.Serial_Fd,&(Nudgematic_Connection_Data.Serial_Options));
	if(retval != 0)
	{
		open_errno = errno;
		Connection_Error_Number = 7;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Open: re-get tcgetattr failed (%d).",open_errno);
		return FALSE;
	}
#if LOGGING > 5
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,
	      "Nudgematic_Connection_Open:New Get Attr:Input:%#x,Output:%#x,Local:%#x,Control:%#x,Min:%c,Time:%c.",
		       Nudgematic_Connection_Data.Serial_Options.c_iflag,Nudgematic_Connection_Data.Serial_Options.c_oflag,
		       Nudgematic_Connection_Data.Serial_Options.c_lflag,Nudgematic_Connection_Data.Serial_Options.c_cflag,
		       Nudgematic_Connection_Data.Serial_Options.c_cc[VMIN],Nudgematic_Connection_Data.Serial_Options.c_cc[VTIME]);
#endif /* LOGGING */
	/* clean I & O device */
	tcflush(Nudgematic_Connection_Data.Serial_Fd,TCIOFLUSH);
#if LOGGING > 0
	Nudgematic_General_Log(LOG_VERBOSITY_TERSE,"Nudgematic_Connection_Open:Finished.");
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to close the connection to the Nudgematic. 
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Nudgematic_Connection_Data
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Log_Format
 */
int Nudgematic_Connection_Close(void)
{
	int retval,close_errno;

#if LOGGING > 0
	Nudgematic_General_Log(LOG_VERBOSITY_TERSE,"Nudgematic_Connection_Close:Started.");
#endif /* LOGGING */
#if LOGGING > 5
	Nudgematic_General_Log(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Close:Flushing serial line.");
#endif /* LOGGING */
	tcflush(Nudgematic_Connection_Data.Serial_Fd, TCIFLUSH);
#if LOGGING > 5
	Nudgematic_General_Log(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Close:Resetting serial options.");
#endif /* LOGGING */
	retval = tcsetattr(Nudgematic_Connection_Data.Serial_Fd,TCSANOW,&(Nudgematic_Connection_Data.Serial_Options_Saved));
	if(retval != 0)
	{
		close_errno = errno;
		Connection_Error_Number = 8;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Close: tcsetattr failed (%d).",close_errno);
		return FALSE;
	}
#if LOGGING > 1
	Nudgematic_General_Log(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Close:Closing file descriptor.");
#endif /* LOGGING */
	retval = close(Nudgematic_Connection_Data.Serial_Fd);
	if(retval < 0)
	{
		close_errno = errno;
		Connection_Error_Number = 9;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Close: close failed (%d,%d,%d).",Nudgematic_Connection_Data.Serial_Fd,retval,close_errno);
		return FALSE;
	}
	return TRUE;
}

/**
 * Write some data to the open connection.
 * @param message An allocated buffer containing the data to write.
 * @param message_length The length of data in the buffer, in bytes.
 * @return TRUE if succeeded, FALSE otherwise.
 * @see #Nudgematic_Connection_Data
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Log_Format
 */
int Nudgematic_Connection_Write(void *message,size_t message_length)
{
	int write_errno,retval;

	if(Nudgematic_Connection_Data.Serial_Fd < 0)
	{
		Connection_Error_Number = 10;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Write: connection not opened (%d).",
			Nudgematic_Connection_Data.Serial_Fd);
		return FALSE;		
	}
	if(message == NULL)
	{
		Connection_Error_Number = 11;
		sprintf(Connection_Error_String,"Arcom_ESS_Serial_Write:Message was NULL.");
		return FALSE;
	}
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Write(%d bytes).",
				      message_length);
#endif /* LOGGING */
	retval = write(Nudgematic_Connection_Data.Serial_Fd,message,message_length);
#if LOGGING > 1
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Write returned %d.",retval);
#endif /* LOGGING */
	if(retval != message_length)
	{
		write_errno = errno;
		Connection_Error_Number = 12;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Write: failed (%d,%d,%d).",
			Nudgematic_Connection_Data.Serial_Fd,retval,write_errno);
		return FALSE;
	}
	/* wait until all output written to fd */
	retval = tcdrain(Nudgematic_Connection_Data.Serial_Fd);
	if(retval != 0)
	{
		write_errno = errno;
		Connection_Error_Number = 21;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Write: tcdrain failed (%d,%d,%d).",
			Nudgematic_Connection_Data.Serial_Fd,retval,write_errno);
		return FALSE;
	}
#if LOGGING > 0
	Nudgematic_General_Log(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Write:Finished.");
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to read a message/some bytes from the serial link.
 * @param message A buffer of message_length bytes, to fill with any serial data returned.
 * @param message_length The length of the message buffer.
 * @param bytes_read The address of an integer. On return this will be filled with the number of bytes read from
 *        the serial interface. The address can be NULL, if this data is not needed.
 * @return TRUE if succeeded, FALSE otherwise.
 * @see #Nudgematic_Connection_Data
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Log_Format
 */
int Nudgematic_Connection_Read(void *message,size_t message_length, int *bytes_read)
{
	int read_errno,retval;

	/* check serial link is open */
	if(Nudgematic_Connection_Data.Serial_Fd < 0)
	{
		Connection_Error_Number = 4;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Read: connection not opened (%d).",
			Nudgematic_Connection_Data.Serial_Fd);
		return FALSE;		
	}
	/* check input parameters */
	if(message == NULL)
	{
		Connection_Error_Number = 13;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Read:Message was NULL.");
		return FALSE;
	}
	if(message_length < 0)
	{
		Connection_Error_Number = 14;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Read:Message length was too small:%ld.",
			message_length);
		return FALSE;
	}
	/* initialise bytes_read */
	if(bytes_read != NULL)
		(*bytes_read) = 0;
#if LOGGING > 11
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Read:Max length %d.",
				      message_length);
#endif /* LOGGING */
	retval = read(Nudgematic_Connection_Data.Serial_Fd,message,message_length);
#if LOGGING > 11
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Read:returned %d.",retval);
#endif /* LOGGING */
	if(retval < 0)
	{
		read_errno = errno;
		/* if the errno is EAGAIN, a non-blocking read has failed to return any data. */
		if(read_errno != EAGAIN)
		{
			Connection_Error_Number = 15;
			sprintf(Connection_Error_String,"Nudgematic_Connection_Read: failed (%d,%d,%d).",
				Nudgematic_Connection_Data.Serial_Fd,retval,read_errno);
			return FALSE;
		}
		else
		{
			if(bytes_read != NULL)
				(*bytes_read) = 0;
		}
	}
	else
	{
		if(bytes_read != NULL)
			(*bytes_read) = retval;
	}
#if LOGGING > 11
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Read:returned %d of %d.",
				      retval,message_length);
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to read a message/some bytes from the serial link, until a newline is found or a timeout occurs.
 * @param message A buffer of message_length bytes, to fill with any serial data returned.
 * @param message_length The length of the message buffer.
 * @param bytes_read The address of an integer. On return this will be filled with the number of bytes read from
 *        the serial interface. The address can be NULL, if this data is not needed.
 * @return TRUE if succeeded, FALSE otherwise.
 * @see #CONNECTION_READ_LINE_TIMEOUT_S
 * @see #Nudgematic_Connection_Read
 * @see #Nudgematic_Connection_Data
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Error
 * @see nudgematic_general.html#Nudgematic_General_Log_Format
 */
int Nudgematic_Connection_Read_Line(char *message,size_t message_length, int *return_bytes_read)
{
	struct timespec read_start_time,sleep_time,current_time;
	int read_errno,sleep_errno,retval,done,bytes_read,total_bytes_read;

	/* check serial link is open */
	if(Nudgematic_Connection_Data.Serial_Fd < 0)
	{
		Connection_Error_Number = 16;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Read: connection not opened (%d).",
			Nudgematic_Connection_Data.Serial_Fd);
		return FALSE;		
	}
	/* check input parameters */
	if(message == NULL)
	{
		Connection_Error_Number = 17;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Read:Message was NULL.");
		return FALSE;
	}
	if(message_length < 0)
	{
		Connection_Error_Number = 18;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Read:Message length was too small:%ld.",
			message_length);
		return FALSE;
	}
	/* initialise bytes_read */
	if(return_bytes_read != NULL)
		(*return_bytes_read) = 0;
	done = FALSE;
	total_bytes_read = 0;
	bytes_read = 0;
	clock_gettime(CLOCK_REALTIME,&read_start_time);
	while(done == FALSE)
	{
		/* read some bytes into message */
		retval = Nudgematic_Connection_Read(message+total_bytes_read,message_length-bytes_read,&bytes_read);
		if(retval == FALSE)
		{
			return FALSE;
		}
		/* update total bytes read and add a terminator to the string */
		total_bytes_read += bytes_read;
		message[total_bytes_read] = '\0';
		/* we have finished if we have received a '\n' */
		done = (strchr(message,'\n') != NULL);
		/* sleep a bit */
		sleep_time.tv_sec = 0;
		sleep_time.tv_nsec = NUDGEMATIC_GENERAL_ONE_MILLISECOND_NS;
		retval = nanosleep(&sleep_time,NULL);
		if(retval != 0)
		{
			sleep_errno = errno;
			Connection_Error_Number = 19;
			sprintf(Connection_Error_String,"Nudgematic_Connection_Read_Line: sleep error (%d).",
				sleep_errno);
			Nudgematic_General_Error();
			/* not a fatal error, don't return */
		}
		/* check for a timeout */
		clock_gettime(CLOCK_REALTIME,&current_time);
		if(fdifftime(current_time,read_start_time) > CONNECTION_READ_LINE_TIMEOUT_S)
		{
			Connection_Error_Number = 20;
			sprintf(Connection_Error_String,"Nudgematic_Connection_Read_Line: timeout after %.2f seconds (%s).",
				fdifftime(current_time,read_start_time),message);
			done  = TRUE;
			return FALSE;
		}
	}/* end while */
	if(return_bytes_read != NULL)
		(*return_bytes_read) = total_bytes_read;
#if LOGGING > 4
	if(total_bytes_read > 0)
	{
		Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Read_Line:Read '%s'.",
					      message);
	}
#endif /* LOGGING */
	return TRUE;
}

/**
 * Send a command string to the nudgematic, and read a newline terminated reply. We lock a mutex around sending a command to the
 * nudgematic and receiving a reply.
 * @param command_string The string containing the command to send.
 * @param reply_string An allocated memory buffer of reply_length chars, on exit from this routine and reply read will
 *        be stored in this string.
 * @param reply_length The size of reply_string in bytes.
 * @return TRUE if succeeded, FALSE otherwise.
 * @see #Nudgematic_Connection_Write
 * @see #Nudgematic_Connection_Read_Line
 * @see #Nudgematic_Connection_Data
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Error
 * @see nudgematic_general.html#Nudgematic_General_Log_Format
 * @see nudgematic_general.html#Nudgematic_General_Mutex_Lock
 * @see nudgematic_general.html#Nudgematic_General_Mutex_Unlock
 */
int Nudgematic_Connection_Send_Command(char *command_string,char *reply_string,size_t reply_length)
{
	int bytes_read;

	/* mutex at this level around one command write/read reply */
	if(!Nudgematic_General_Mutex_Lock())
	{
		Connection_Error_Number = 22;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Send_Command: Failed to lock mutex for '%s'.",
			command_string);
		return FALSE;
	}
	/* send command */
#if LOGGING > 1
	Nudgematic_General_Log_Format(LOG_VERBOSITY_VERY_VERBOSE,"Nudgematic_Connection_Send_Command:Sending command '%s'.",
				      command_string);
#endif /* LOGGING */
	if(!Nudgematic_Connection_Write(command_string,strlen(command_string)))
	{
		Nudgematic_General_Mutex_Unlock();
		return FALSE;
	}
	/* read a reply */
	if(!Nudgematic_Connection_Read_Line(reply_string,reply_length,&bytes_read))
	{
		Nudgematic_General_Mutex_Unlock();
		return FALSE;
	}
	/* release mutex */
	if(!Nudgematic_General_Mutex_Unlock())
	{
		Connection_Error_Number = 23;
		sprintf(Connection_Error_String,"Nudgematic_Connection_Send_Command: Failed to unlock mutex for '%s'.",
			command_string);
		return FALSE;
	}		
	return TRUE;
}

/**
 * Get the current value of the error number.
 * @return The current value of the error number.
 * @see #Connection_Error_Number
 */
int Nudgematic_Connection_Get_Error_Number(void)
{
	return Connection_Error_Number;
}

/**
 * The error routine that reports any errors occuring in a standard way.
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Get_Current_Time_String
 */
void Nudgematic_Connection_Error(void)
{
	char time_string[32];

	Nudgematic_General_Get_Current_Time_String(time_string,32);
	/* if the error number is zero an error message has not been set up
	** This is in itself an error as we should not be calling this routine
	** without there being an error to display */
	if(Connection_Error_Number == 0)
		sprintf(Connection_Error_String,"Logic Error:No Error defined");
	fprintf(stderr,"%s Nudgematic_Connection:Error(%d) : %s\n",time_string,
		Connection_Error_Number,Connection_Error_String);
}

/**
 * The error routine that reports any errors occuring in a standard way. This routine places the
 * generated error string at the end of a passed in string argument.
 * @param error_string A string to put the generated error in. This string should be initialised before
 * being passed to this routine. The routine will try to concatenate it's error string onto the end
 * of any string already in existance.
 * @see #Connection_Error_Number
 * @see #Connection_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Get_Current_Time_String
 */
void Nudgematic_Connection_Error_To_String(char *error_string)
{
	char time_string[32];

	Nudgematic_General_Get_Current_Time_String(time_string,32);
	/* if the error number is zero an error message has not been set up
	** This is in itself an error as we should not be calling this routine
	** without there being an error to display */
	if(Connection_Error_Number == 0)
		sprintf(Connection_Error_String,"Logic Error:No Error defined");
	sprintf(error_string+strlen(error_string),"%s Nudgematic_Connection:Error(%d) : %s\n",time_string,
		Connection_Error_Number,Connection_Error_String);
}
