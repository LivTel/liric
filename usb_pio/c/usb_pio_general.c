/* usb_pio_general.c
** USB-PIO IO board communication library
*/
/**
 * Error and Log handlers.
 * @author Chris Mottram
 * @version $Revision$
 */
/**
 * This hash define is needed before including source files give us POSIX.4/IEEE1003.1b-1993 prototypes.
 */
#define _POSIX_SOURCE 1
/**
 * This hash define is needed before including source files give us POSIX.4/IEEE1003.1b-1993 prototypes.
 */
#define _POSIX_C_SOURCE 199309L

#include <errno.h>   /* Error number definitions */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>
#include <unistd.h>
#ifdef MUTEXED
#include <pthread.h>
#endif
#include "usb_pio_general.h"
#include "usb_pio_command.h"
#include "usb_pio_connection.h"

/* defines */
/**
 * How long some buffers are when generating logging messages.
 */
#define LOG_BUFF_LENGTH           (4096)

/* data types */
/**
 * Data type holding local data to usb_pio_general. This consists of the following:
 * <dl>
 * <dt>Mutex</dt> <dd>Optionally compiled mutex locking over sending commands over the USB/serial connection 
 *                    and receiving a reply.</dd>
 * <dt>Log_Handler</dt> <dd>Function pointer to the routine that will log messages passed to it.</dd>
 * <dt>Log_Filter</dt> <dd>Function pointer to the routine that will filter log messages passed to it.
 * 		The funtion will return TRUE if the message should be logged, and FALSE if it shouldn't.</dd>
 * <dt>Log_Filter_Level</dt> <dd>A globally maintained log filter level. 
 * 		This is set using USB_PIO_General_Set_Log_Filter_Level.
 * 		USB_PIO_Log_General_Filter_Level_Absolute and USB_PIO_General_Log_Filter_Level_Bitwise 
 *              test it against message levels to determine whether to log messages.</dd>
 * </dl>
 * @see #USB_PIO_General_Log
 * @see #USB_PIO_General_Set_Log_Filter_Level
 * @see #USB_PIO_General_Log_Filter_Level_Absolute
 * @see #USB_PIO_General_Log_Filter_Level_Bitwise
 */
struct General_Struct
{
#ifdef MUTEXED
	pthread_mutex_t Mutex;
#endif
	void (*Log_Handler)(int level,char *string);
	int (*Log_Filter)(int level,char *string);
	int Log_Filter_Level;
};


/* internal variables */
/**
 * Revision Control System identifier.
 */
static char rcsid[] = "$Id$";
/**
 * The instance of General_Struct that contains local data for this module.
 * This is statically initialised to the following:
 * <dl>
 * <dt>Mutex</dt> <dd>If compiled in, PTHREAD_MUTEX_INITIALIZER</dd>
 * <dt>Log_Handler</dt> <dd>NULL</dd>
 * <dt>Log_Filter</dt> <dd>NULL</dd>
 * <dt>Log_Filter_Level</dt> <dd>0</dd>
 * </dl>
 * @see #General_Struct
 */
static struct General_Struct General_Data = 
{
#ifdef MUTEXED
	PTHREAD_MUTEX_INITIALIZER,
#endif
	NULL,NULL,0,
};

/**
 * Variable holding error code of last operation performed.
 */
static int General_Error_Number = 0;
/**
 * Local variable holding description of the last error that occured.
 * @see #USB_PIO_GENERAL_ERROR_STRING_LENGTH
 */
static char General_Error_String[USB_PIO_GENERAL_ERROR_STRING_LENGTH] = "";

/* --------------------------------------------------------
** External Functions
** -------------------------------------------------------- */
/**
 * Routine to return whether an error has been set in the library.
 * @return The routine returns TRUE if an error has been set, and FALSE if no error has been set.
 * @see #General_Error_Number
 * @see usb_pio_command.html#USB_PIO_Command_Get_Error_Number
 * @see usb_pio_connection.html#USB_PIO_Connection_Get_Error_Number
 */
int USB_PIO_General_Is_Error(void)
{
	int found = FALSE;

	if(USB_PIO_Command_Get_Error_Number() != 0)
		found = TRUE;
	if(USB_PIO_Connection_Get_Error_Number() != 0)
		found = TRUE;
	if(General_Error_Number != 0)
		found = TRUE;
	return found;
}

/**
 * Basic error reporting routine, to stderr.
 * @see #General_Error_Number
 * @see #General_Error_String
 * @see #USB_PIO_General_Get_Current_Time_String
 * @see usb_pio_command.html#USB_PIO_Command_Get_Error_Number
 * @see usb_pio_command.html#USB_PIO_Command_Error
 * @see usb_pio_connection.html#USB_PIO_Connection_Get_Error_Number
 * @see usb_pio_connection.html#USB_PIO_Connection_Error
 */
void USB_PIO_General_Error(void)
{
	char time_string[32];
	int found = FALSE;

	if(USB_PIO_Command_Get_Error_Number() != 0)
	{
		found = TRUE;
		USB_PIO_Command_Error();
	}
	if(USB_PIO_Connection_Get_Error_Number() != 0)
	{
		found = TRUE;
		USB_PIO_Connection_Error();
	}
	if(General_Error_Number != 0)
	{
		found = TRUE;
		fprintf(stderr,"\t\t\t");
		USB_PIO_General_Get_Current_Time_String(time_string,32);
		fprintf(stderr,"%s USB_PIO_General:Error(%d) : %s\n",time_string,General_Error_Number,
			General_Error_String);
	}
	if(!found)
	{
		fprintf(stderr,"%s USB_PIO_General_Error:An unknown error has occured.\n",time_string);
	}
}

/**
 * Basic error reporting routine, to the specified string.
 * @param error_string Pointer to an already allocated area of memory, to store the generated error string. 
 *        This should be at least 256 bytes long.
 * @see #General_Error_Number
 * @see #General_Error_String
 * @see #USB_PIO_General_Get_Current_Time_String
 * @see usb_pio_command.html#USB_PIO_Command_Get_Error_Number
 * @see usb_pio_command.html#USB_PIO_Command_Error_String
 * @see usb_pio_connection.html#USB_PIO_Connection_Get_Error_Number
 * @see usb_pio_connection.html#USB_PIO_Connection_Error_String
 */
void USB_PIO_General_Error_To_String(char *error_string)
{
	char time_string[32];

	strcpy(error_string,"");
	if(USB_PIO_Command_Get_Error_Number() != 0)
	{
		USB_PIO_Command_Error_String(error_string);
	}
	if(USB_PIO_Connection_Get_Error_Number() != 0)
	{
		USB_PIO_Connection_Error_String(error_string);
	}
	if(General_Error_Number != 0)
	{
		USB_PIO_General_Get_Current_Time_String(time_string,32);
		sprintf(error_string+strlen(error_string),"%s USB_PIO_General:Error(%d) : %s\n",time_string,
			General_Error_Number,General_Error_String);
	}
	if(strlen(error_string) == 0)
	{
		sprintf(error_string,"%s Error:USB_PIO_General:Error not found\n",time_string);
	}
}


/**
 * Routine to return the current value of the error number.
 * @return The value of General_Error_Number.
 * @see #General_Error_Number
 */
int USB_PIO_General_Get_Error_Number(void)
{
	return General_Error_Number;
}

/**
 * Routine to get the current time in a string. The string is returned in the format
 * '01-01-2000T13:59:59.123 UTC'.
 * The time is in UTC.
 * @param time_string The string to fill with the current time.
 * @param string_length The length of the buffer passed in. It is recommended the length is at least 20 characters.
 * @see #USB_PIO_GENERAL_ONE_MILLISECOND_NS
 */
void USB_PIO_General_Get_Current_Time_String(char *time_string,int string_length)
{
	char timezone_string[16];
	char millsecond_string[8];
	struct timespec current_time;
	struct tm *utc_time = NULL;

	clock_gettime(CLOCK_REALTIME,&current_time);
	utc_time = gmtime(&(current_time.tv_sec));
	strftime(time_string,string_length,"%d-%m-%YT%H:%M:%S",utc_time);
	sprintf(millsecond_string,"%03ld",(current_time.tv_nsec/USB_PIO_GENERAL_ONE_MILLISECOND_NS));
	strftime(timezone_string,16,"%z",utc_time);
	if((strlen(time_string)+strlen(millsecond_string)+strlen(timezone_string)+3) < string_length)
	{
		strcat(time_string,".");
		strcat(time_string,millsecond_string);
		strcat(time_string," ");
		strcat(time_string,timezone_string);
	}
}

/**
 * Routine to log a message to a defined logging mechanism. This routine has an arbitary number of arguments,
 * and uses vsprintf to format them i.e. like fprintf. 
 * USB_PIO_General_Log is then called to handle the log message.
 * @param level An integer, used to decide whether this particular message has been selected for
 * 	logging or not.
 * @param format A string, with formatting statements the same as fprintf would use to determine the type
 * 	of the following arguments.
 * @see #USB_PIO_General_Log
 * @see #LOG_BUFF_LENGTH
 */
void USB_PIO_General_Log_Format(int level,char *format,...)
{
	char buff[LOG_BUFF_LENGTH];
	va_list ap;

/* format the arguments */
	va_start(ap,format);
	vsprintf(buff,format,ap);
	va_end(ap);
/* call the log routine to log the results */
	USB_PIO_General_Log(level,buff);
}

/**
 * Routine to log a message to a defined logging mechanism. If the string or General_Data.Log_Handler are NULL
 * the routine does not log the message. If the General_Data.Log_Filter function pointer is non-NULL, the
 * message is passed to it to determine whether to log the message.
 * @param level An integer, used to decide whether this particular message has been selected for
 * 	logging or not.
 * @param string The message to log.
 * @see #General_Data
 */
void USB_PIO_General_Log(int level,char *string)
{
/* If the string is NULL, don't log. */
	if(string == NULL)
		return;
/* If there is no log handler, return */
	if(General_Data.Log_Handler == NULL)
		return;
/* If there's a log filter, check it returns TRUE for this message */
	if(General_Data.Log_Filter != NULL)
	{
		if(General_Data.Log_Filter(level,string) == FALSE)
			return;
	}
/* We can log the message */
	(*General_Data.Log_Handler)(level,string);
}

/**
 * Routine to set the General_Data.Log_Handler used by USB_PIO_General_Log.
 * @param log_fn A function pointer to a suitable handler.
 * @see #General_Data
 * @see #USB_PIO_General_Log
 */
void USB_PIO_General_Set_Log_Handler_Function(void (*log_fn)(int level,char *string))
{
	General_Data.Log_Handler = log_fn;
}

/**
 * Routine to set the General_Data.Log_Filter used by USB_PIO_General_Log.
 * @param log_fn A function pointer to a suitable filter function.
 * @see #General_Data
 * @see #USB_PIO_General_Log
 */
void USB_PIO_General_Set_Log_Filter_Function(int (*filter_fn)(int level,char *string))
{
	General_Data.Log_Filter = filter_fn;
}

/**
 * A log handler to be used for the General_Data.Log_Handler function.
 * Prints the message to stdout, terminated by a newline and Prepended by a timestamp generated by 
 * USB_PIO_General_Get_Current_Time_String.
 * @param level The log level for this message.
 * @param string The log message to be logged. 
 * @see #USB_PIO_General_Get_Current_Time_String
 */
void USB_PIO_General_Log_Handler_Stdout(int level,char *string)
{
	char time_string[32];

	if(string == NULL)
		return;
	USB_PIO_General_Get_Current_Time_String(time_string,32);
	fprintf(stdout,"%s %s\n",time_string,string);
}

/**
 * Routine to set the General_Data.Log_Filter_Level.
 * @see #General_Data
 */
void USB_PIO_General_Set_Log_Filter_Level(int level)
{
	General_Data.Log_Filter_Level = level;
}

/**
 * A log message filter routine, to be used for the General_Data.Log_Filter function pointer.
 * @param level The log level of the message to be tested.
 * @param string The log message to be logged, not used in this filter. 
 * @return The routine returns TRUE if the level is less than or equal to the General_Data.Log_Filter_Level,
 * 	otherwise it returns FALSE.
 * @see #General_Data
 */
int USB_PIO_General_Log_Filter_Level_Absolute(int level,char *string)
{
	return (level <= General_Data.Log_Filter_Level);
}

/**
 * A log message filter routine, to be used for the General_Data.Log_Filter function pointer.
 * @param level The log level of the message to be tested.
 * @param string The log message to be logged, not used in this filter. 
 * @return The routine returns TRUE if the level has bits set that are also set in the 
 * 	General_Data.Log_Filter_Level, otherwise it returns FALSE.
 * @see #General_Data
 */
int USB_PIO_General_Log_Filter_Level_Bitwise(int level,char *string)
{
	return ((level & General_Data.Log_Filter_Level) > 0);
}

#ifdef MUTEXED
/**
 * Routine to lock the access mutex. This will block until the mutex has been acquired,
 * unless an error occurs.
 * @return Returns TRUE if the mutex has been locked for access by this thread,
 * 	FALSE if an error occured.
 * @see #General_Data
 */
int USB_PIO_General_Mutex_Lock(void)
{
	int error_number;

	error_number = pthread_mutex_lock(&(General_Data.Mutex));
	if(error_number != 0)
	{
		General_Error_Number = 1;
		sprintf(General_Error_String,"USB_PIO_General_Mutex_Lock:Mutex lock failed '%d'.",error_number);
		return FALSE;
	}
	return TRUE;
}

/**
 * Routine to unlock the access mutex. 
 * @return Returns TRUE if the mutex has been unlocked, FALSE if an error occured.
 * @see #General_Data
 */
int USB_PIO_General_Mutex_Unlock(void)
{
	int error_number;

	error_number = pthread_mutex_unlock(&(General_Data.Mutex));
	if(error_number != 0)
	{
		General_Error_Number = 2;
		sprintf(General_Error_String,"USB_PIO_General_Mutex_Unlock:Mutex unlock failed '%d'.",
			error_number);
		return FALSE;
	}
	return TRUE;
}
#endif
