/* nudgematic_command.c
** Nudgematic mechanism library
*/
/**
 * Routines handling commands to the Nudgematic mechanism.
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
#include "log_udp.h"

#include "nudgematic_general.h"
#include "nudgematic_command.h"
#include "nudgematic_connection.h"

/* hash defines */
/**
 * Index used in array for variables pertaining to the vertical cam of the nudgematic. 
 */
#define NUDGEMATIC_VERTICAL          (0)
/**
 * Index used in array for variables pertaining to the horizontal cam of the nudgematic. 
 */
#define NUDGEMATIC_HORIZONTAL          (1)
/**
 * The number of cams/motors in the nudgematic. 
 */
#define NUDGEMATIC_CAM_COUNT           (2)
/**
 * How many offsets are in NUDGEMATIC_OFFSET_SIZE_ENUM  (there is actually 3, as OFFSET_SIZE_NONE is an offset).
 * @see #NUDGEMATIC_OFFSET_SIZE_ENUM 
 */
#define NUDGEMATIC_OFFSET_SIZE_COUNT   (3)
/**
 * How many offset positions there are per offset size.
 */
#define NUDGEMATIC_POSITION_COUNT      (9)
/**
 * A length of time to sleep between querying the nudgematics position, in the move (Position_Set) loop.
 */
#define COMMAND_MOVE_SLEEP_TIME_MS     (50)
/**
 * How long to wait for the nudgematic to move to a new position before we time out, in seconds.
 */
#define COMMAND_POSITION_SET_TIMEOUT_S (10.0)

/**
 * The length of some internal strings.
 */
#define STRING_LENGTH                (256)

/* data types */
/**
 * Data type holding local data to nudgematic_command. This consists of the following:
 * <dl>
 * <dt>Offset_Size</dt> <dd>The current set size of offsets the nudgematic will perform.</dd>
 * <dt>Target_Position</dt> <dd>The postion (0..9) we are currently trying to attain.</dd>
 * </dl>
 * @see #NUDGEMATIC_OFFSET_SIZE_T
 */
struct Command_Struct
{
	NUDGEMATIC_OFFSET_SIZE_T Offset_Size;
	int Target_Position;
};

/* internal variables */
/**
 * Revision Control System identifier.
 */
static char rcsid[] = "$Id$";
/**
 * The instance of Command_Struct that contains local data for this module.
 * This is statically initialised to the following:
 * <dl>
 * <dt>Offset_Size</dt> <dd>OFFSET_SIZE_NONE</dd>
 * <dt>Target_Position</dt> <dd>-1</dd>
 * </dl>
 * @see #Command_Struct
 */
static struct Command_Struct Command_Data = 
{
	OFFSET_SIZE_NONE,-1
};

/**
 * List of characters representing commands to retrieve the current status (where the cam is) for each cam.
 * @see #NUDGEMATIC_CAM_COUNT
 * @see #NUDGEMATIC_VERTICAL
 * @see #NUDGEMATIC_HORIZONTAL
 */
static char Where_Command_List[NUDGEMATIC_CAM_COUNT] = {'w','W'};
/**
 * List of move commands to send, for each cam, for each offset size, for each position.
 * @see #NUDGEMATIC_POSITION_COUNT
 * @see #NUDGEMATIC_OFFSET_SIZE_COUNT
 * @see #NUDGEMATIC_CAM_COUNT
 */
static char Move_Command_List[NUDGEMATIC_POSITION_COUNT][NUDGEMATIC_OFFSET_SIZE_COUNT][NUDGEMATIC_CAM_COUNT] =
{     /* NONE(V,H),SMALL(V,H),LARGE(V,H) */
	{{'c','C'},{'c','C'},{'c','C'}}, /* centre */
	{{'c','C'},{'b','B'},{'a','A'}}, /* top left */
	{{'c','C'},{'d','D'},{'e','E'}}, /* bottom right */
	{{'c','C'},{'b','D'},{'a','E'}}, /* top right */
	{{'c','C'},{'d','B'},{'e','A'}}, /* bottom left */
	{{'c','C'},{'c','B'},{'c','A'}}, /* centre left */
	{{'c','C'},{'c','D'},{'c','E'}}, /* centre right */
	{{'c','C'},{'b','C'},{'a','C'}}, /* top centre */
	{{'c','C'},{'d','C'},{'e','C'}}  /* bottom centre */
};

/**
 * Variable holding error code of last operation performed.
 */
static int Command_Error_Number = 0;
/**
 * Local variable holding description of the last error that occured.
 * @see nudgematic_general.html#NUDGEMATIC_GENERAL_ERROR_STRING_LENGTH
 */
static char Command_Error_String[NUDGEMATIC_GENERAL_ERROR_STRING_LENGTH] = "";

/* internal functions */
static int Command_Parse_Reply_String(char *reply_string,char *position_char,int *position_adu,int *position_error,
				      int *nudges,int *time_ms);

/* =======================================
**  external functions 
** ======================================= */
/**
 * Routine to move the nudgematic to the specified position.
 * @param position The position to move to.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #NUDGEMATIC_CAM_COUNT
 * @see #COMMAND_MOVE_SLEEP_TIME_MS
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see #Command_Data
 * @see #Move_Command_List
 * @see #Where_Command_List
 * @see nudgematic_connection.html#Nudgematic_Connection_Send_Command
 */
int Nudgematic_Command_Position_Set(int position)
{
	struct timespec start_time,sleep_time,current_time;
	char command_string[STRING_LENGTH];
	char reply_string[STRING_LENGTH];
	char horizontal_cam_command, vertical_cam_command,position_char;
	char cam_command[NUDGEMATIC_CAM_COUNT];
	int retval,cam,position_adu,position_error,nudges,time_ms,sleep_errno;
	int done[NUDGEMATIC_CAM_COUNT];
	
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Position_Set: Started with position %d and offset size '%s'.",
				      position,Nudgematic_Command_Offset_Size_To_String(Command_Data.Offset_Size));
#endif /* LOGGING */
	if((position < 0)||(position >= NUDGEMATIC_POSITION_COUNT))
	{
		Command_Error_Number = 7;
		sprintf(Command_Error_String,"Nudgematic_Command_Position_Set:position %d was out of range (0,%d).",
			position,NUDGEMATIC_POSITION_COUNT);
		return FALSE;		
	}
	Command_Data.Target_Position = position;
	horizontal_cam_command = Move_Command_List[Command_Data.Target_Position][Command_Data.Offset_Size][NUDGEMATIC_HORIZONTAL];
	vertical_cam_command = Move_Command_List[Command_Data.Target_Position][Command_Data.Offset_Size][NUDGEMATIC_VERTICAL];
	cam_command[NUDGEMATIC_HORIZONTAL] = Move_Command_List[Command_Data.Target_Position][Command_Data.Offset_Size][NUDGEMATIC_HORIZONTAL];
	cam_command[NUDGEMATIC_VERTICAL] = Move_Command_List[Command_Data.Target_Position][Command_Data.Offset_Size][NUDGEMATIC_VERTICAL];
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Position_Set: Position %d and Offset Size '%s' maps to "
				      "horizontal cam command '%c' and vertical cam command '%c'.",
				      Command_Data.Target_Position,Nudgematic_Command_Offset_Size_To_String(Command_Data.Offset_Size),
				      horizontal_cam_command,vertical_cam_command);
#endif /* LOGGING */
	/* move horizontal cam */
	command_string[0] = horizontal_cam_command;
	command_string[1] = '\n';
	command_string[2] = '\0';
	if(!Nudgematic_Connection_Send_Command(command_string,reply_string,STRING_LENGTH))
	{
		Command_Error_Number = 8;
		sprintf(Command_Error_String,"Nudgematic_Command_Position_Set:Failed to send horizontal command string '%s'.",
			command_string);
		return FALSE;		
	}
	/* parse reply string */
	if(!Command_Parse_Reply_String(reply_string,&position_char,&position_adu,&position_error,&nudges,&time_ms))
	{
		return FALSE;
	}
	/* check we are moving to the right position */
	if(position_char != horizontal_cam_command)
	{
		Command_Error_Number = 21;
		sprintf(Command_Error_String,"Nudgematic_Command_Position_Set:Horizontal move command reply does not match "
			"('%c' vs '%c').",position_char,horizontal_cam_command);
		return FALSE;		
	}
	/* move vertical cam */
	command_string[0] = vertical_cam_command;
	command_string[1] = '\n';
	command_string[2] = '\0';
	if(!Nudgematic_Connection_Send_Command(command_string,reply_string,STRING_LENGTH))
	{
		Command_Error_Number = 9;
		sprintf(Command_Error_String,"Nudgematic_Command_Position_Set:Failed to send vertical command string '%s'.",
			command_string);
		return FALSE;		
	}
	/* parse reply string */
	if(!Command_Parse_Reply_String(reply_string,&position_char,&position_adu,&position_error,&nudges,&time_ms))
	{
		return FALSE;
	}
	/* check we are moving to the right position */
	if(position_char != vertical_cam_command)
	{
		Command_Error_Number = 22;
		sprintf(Command_Error_String,"Nudgematic_Command_Position_Set:Vertical move command reply does not match "
			"('%c' vs '%c').",position_char,vertical_cam_command);
		return FALSE;		
	}
	/* monitor for completion of move */
	for(cam = NUDGEMATIC_VERTICAL; cam < NUDGEMATIC_CAM_COUNT; cam++)
	{
		done[cam] = FALSE;
	}
	clock_gettime(CLOCK_REALTIME,&start_time);
	while((done[NUDGEMATIC_VERTICAL] == FALSE)||(done[NUDGEMATIC_HORIZONTAL] == FALSE))
	{
		for(cam = NUDGEMATIC_VERTICAL; cam < NUDGEMATIC_CAM_COUNT; cam++)
		{
			/* send a where command for this cam */
			command_string[0] = Where_Command_List[cam];
			command_string[1] = '\n';
			command_string[2] = '\0';
			if(!Nudgematic_Connection_Send_Command(command_string,reply_string,STRING_LENGTH))
			{
				Command_Error_Number = 10;
				sprintf(Command_Error_String,"Nudgematic_Command_Position_Set:Failed to send where command string '%s'.",
					command_string);
				return FALSE;		
			}
			/* parse reply string */
			if(!Command_Parse_Reply_String(reply_string,&position_char,&position_adu,&position_error,&nudges,&time_ms))
			{
				return FALSE;
			}
			/* are we in position? */
			if(position_char == cam_command[cam])
			{
				done[cam] = TRUE;
#if LOGGING > 0
				Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Position_Set: Position %d: Cam %d acheived position '%c' "
				      "with ADU %d error %d nudges %d and time(ms) %d.",
				      Command_Data.Target_Position,cam,position_char,position_adu,position_error,nudges,time_ms);
#endif /* LOGGING */
			} /* end if cam is in position */
		}/* end for on cam */
		/* check for a timeout */
		clock_gettime(CLOCK_REALTIME,&current_time);
		if(fdifftime(current_time,start_time) > COMMAND_POSITION_SET_TIMEOUT_S)
		{
			Command_Error_Number = 18;
			sprintf(Command_Error_String,"Nudgematic_Command_Position_Set: timeout after %.2f seconds.",
				fdifftime(current_time,start_time));
			return FALSE;
		}
		/* sleep a bit */
		sleep_time.tv_sec = 0;
		sleep_time.tv_nsec = COMMAND_MOVE_SLEEP_TIME_MS*NUDGEMATIC_GENERAL_ONE_MILLISECOND_NS;
		retval = nanosleep(&sleep_time,NULL);
		if(retval != 0)
		{
			sleep_errno = errno;
			Command_Error_Number = 19;
			sprintf(Command_Error_String,"Nudgematic_Command_Position_Set: sleep error (%d).",
				sleep_errno);
			Nudgematic_General_Error();
			/* not a fatal error, don't return */
		}
	}/* end while not done[] */
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Position_Set: Now in Position %d with Offset Size '%s'.",
				      Command_Data.Target_Position,
				      Nudgematic_Command_Offset_Size_To_String(Command_Data.Offset_Size));
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to get the current position of the nudgematic.
 * @param position The address of an integer, on return containing the current position, or '-1' if the mechanism
 *        is not in a defined position/moving.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #NUDGEMATIC_POSITION_COUNT
 * @see #NUDGEMATIC_CAM_COUNT
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see #Move_Command_List
 * @see #Where_Command_List
 * @see #Command_Parse_Reply_String
 * @see #Nudgematic_Command_Offset_Size_To_String
 * @see nudgematic_connection.html#Nudgematic_Connection_Send_Command
 */
int Nudgematic_Command_Position_Get(int *position)
{
	char command_string[STRING_LENGTH];
	char reply_string[STRING_LENGTH];
	char position_char[NUDGEMATIC_CAM_COUNT];
	int found,i,offset_size,retval,cam,position_adu,position_error,nudges,time_ms;
	
	if(position == NULL)
	{
		Command_Error_Number = 1;
		sprintf(Command_Error_String,"Nudgematic_Command_Position_Get:position was NULL.");
		return FALSE;
	}
	/* initialise position to moving */
	(*position) = -1;
	/* get the current position of each cam */
	for(cam = NUDGEMATIC_VERTICAL; cam < NUDGEMATIC_CAM_COUNT; cam++)
	{
		/* send a where command for this cam */
		command_string[0] = Where_Command_List[cam];
		command_string[1] = '\n';
		command_string[2] = '\0';
		if(!Nudgematic_Connection_Send_Command(command_string,reply_string,STRING_LENGTH))
		{
			Command_Error_Number = 20;
			sprintf(Command_Error_String,"Nudgematic_Command_Position_Get:Failed to send where command string '%s'.",
				command_string);
			return FALSE;		
		}
		/* parse reply string */
		if(!Command_Parse_Reply_String(reply_string,&(position_char[cam]),&position_adu,&position_error,&nudges,&time_ms))
		{
			return FALSE;
		}
	}/* end for on cam */
#if LOGGING > 1
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Position_Get: Current cam positions '%c', '%c'.",
				      position_char[NUDGEMATIC_VERTICAL],position_char[NUDGEMATIC_HORIZONTAL]);
#endif /* LOGGING */
	/* find the current position based on cam positions */
	i = 0;
	found = FALSE;
	while((i < NUDGEMATIC_POSITION_COUNT)&&(found == FALSE))
	{
		offset_size = OFFSET_SIZE_SMALL;
		while((offset_size < NUDGEMATIC_OFFSET_SIZE_COUNT) && (found == FALSE))
		{

			if((Move_Command_List[i][offset_size][NUDGEMATIC_HORIZONTAL] == position_char[NUDGEMATIC_HORIZONTAL]) &&
			   (Move_Command_List[i][offset_size][NUDGEMATIC_VERTICAL] == position_char[NUDGEMATIC_VERTICAL]))
			{
				(*position) = i;
				found = TRUE;
#if LOGGING > 1
				Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				    "Nudgematic_Command_Position_Get: Found a position match at position %d, offset size %s.",
							      i,Nudgematic_Command_Offset_Size_To_String(offset_size));
#endif /* LOGGING */
			}
			offset_size ++;
		}/* end while on offset_size */
		i++;
	}/* end while on i (position) */
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,"Nudgematic_Command_Position_Get: Current position %d.",
				      (*position));
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to get the current temperature from 3 Dallas 18B60 1-wire temperature sensors connected to the nudgematic.
 * @param temp1 The address of an double, on return containing one of the temperature sensor's temperature (nominally 'Internal'
 *        but this may change).
 * @param temp2 The address of an double, on return containing one of the temperature sensor's temperature (nominally 'External'
 *        but this may change).
 * @param temp3 The address of an double, on return containing one of the temperature sensor's temperature (nominally 'Radiator'
 *        but this may change).
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see #Command_Parse_Reply_String
 * @see #Nudgematic_Command_Offset_Size_To_String
 * @see nudgematic_connection.html#Nudgematic_Connection_Send_Command
 */
int Nudgematic_Command_Temperature_Get(double *temp1,double *temp2,double *temp3)
{
	char command_string[STRING_LENGTH];
	char reply_string[STRING_LENGTH];
	int retval;

	if(temp1 == NULL)
	{
		Command_Error_Number = 23;
		sprintf(Command_Error_String,"Nudgematic_Command_Temperature_Get:temp1 was NULL.");
		return FALSE;		
	}
	if(temp2 == NULL)
	{
		Command_Error_Number = 24;
		sprintf(Command_Error_String,"Nudgematic_Command_Temperature_Get:temp2 was NULL.");
		return FALSE;		
	}
	if(temp3 == NULL)
	{
		Command_Error_Number = 25;
		sprintf(Command_Error_String,"Nudgematic_Command_Temperature_Get:temp3 was NULL.");
		return FALSE;		
	}
	/* query temperatures */
	command_string[0] = 't';
	command_string[1] = '\n';
	command_string[2] = '\0';
	if(!Nudgematic_Connection_Send_Command(command_string,reply_string,STRING_LENGTH))
	{
		Command_Error_Number = 26;
		sprintf(Command_Error_String,"Nudgematic_Command_Temperature_Get:Failed to send command string '%s'.",
			command_string);
		return FALSE;		
	}
	/* parse reply string */
	retval = sscanf(reply_string,"%lf %lf %lf",temp1,temp2,temp3);
	if(retval != 3)
	{
		Command_Error_Number = 27;
		sprintf(Command_Error_String,"Nudgematic_Command_Temperature_Get:Failed to parse reply_string '%s' (%d).",
			reply_string,retval);
		return FALSE;		
	}
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Temperature_Get: Returned temperatures %.2f, %.2f, %.2f.",
				      (*temp1),(*temp2),(*temp3));
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to set the offset size of the positions of the Nudgematic.
 * @param size The size of the offset to use, either OFFSET_SIZE_NONE, OFFSET_SIZE_SMALL or OFFSET_SIZE_LARGE.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Nudgematic_Command_Offset_Size_To_String
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see #NUDGEMATIC_OFFSET_SIZE_T
 */
int Nudgematic_Command_Offset_Size_Set(NUDGEMATIC_OFFSET_SIZE_T size)
{
	if((size != OFFSET_SIZE_NONE)&&(size != OFFSET_SIZE_SMALL)&&(size != OFFSET_SIZE_LARGE))
	{
		Command_Error_Number = 2;
		sprintf(Command_Error_String,"Nudgematic_Command_Offset_Size_Set:Illegal size '%d'.",size);
		return FALSE;
	}
	Command_Data.Offset_Size = size;
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Offset_Size_Set: Offset size set to to %d (%s).",
				      size,Nudgematic_Command_Offset_Size_To_String(size));
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to get the current offset size of the positions of the Nudgematic.
 * @param size The address of a NUDGEMATIC_OFFSET_SIZE_T, on a successful return should contain 
 *             the size of the offset to use, either OFFSET_SIZE_NONE, OFFSET_SIZE_SMALL or OFFSET_SIZE_LARGE.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Nudgematic_Command_Offset_Size_To_String
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see #NUDGEMATIC_OFFSET_SIZE_T
 */
int Nudgematic_Command_Offset_Size_Get(NUDGEMATIC_OFFSET_SIZE_T *size)
{
	if(size == NULL)
	{
		Command_Error_Number = 3;
		sprintf(Command_Error_String,"Nudgematic_Command_Offset_Size_Get:size was NULL.");
		return FALSE;
	}
	(*size) = Command_Data.Offset_Size;
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Offset_Size_Get: Current offset size %d '%s'.",(*size),
				      Nudgematic_Command_Offset_Size_To_String((*size)));
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to parse the specified string into a valid instance of NUDGEMATIC_OFFSET_SIZE_T.
 * @param size The address of a NUDGEMATIC_OFFSET_SIZE_T, on a successful return should contain 
 *             an offset size, either OFFSET_SIZE_NONE, OFFSET_SIZE_SMALL or OFFSET_SIZE_LARGE.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see #NUDGEMATIC_OFFSET_SIZE_T
 */
int Nudgematic_Command_Offset_Size_Parse(char *offset_size_string, NUDGEMATIC_OFFSET_SIZE_T *size)
{
	if(offset_size_string == NULL)
	{
		Command_Error_Number = 4;
		sprintf(Command_Error_String,"Nudgematic_Command_Offset_Size_Parse:offset_size_string was NULL.");
		return FALSE;
	}
	if(size == NULL)
	{
		Command_Error_Number = 5;
		sprintf(Command_Error_String,"Nudgematic_Command_Offset_Size_Parse:size was NULL.");
		return FALSE;
	}
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Offset_Size_Parse: Parsing offset size '%s'.",offset_size_string);
#endif /* LOGGING */
	if((strcmp(offset_size_string,"none") == 0)||(strcmp(offset_size_string,"NONE") == 0))
		(*size) = OFFSET_SIZE_NONE;
	else if((strcmp(offset_size_string,"small") == 0)||(strcmp(offset_size_string,"SMALL") == 0))
		(*size) = OFFSET_SIZE_SMALL;
	else if((strcmp(offset_size_string,"large") == 0)||(strcmp(offset_size_string,"LARGE") == 0))
		(*size) = OFFSET_SIZE_LARGE;
	else
	{
		Command_Error_Number = 6;
		sprintf(Command_Error_String,"Nudgematic_Command_Offset_Size_Parse:failed to parse size '%s'.",
			offset_size_string);
		return FALSE;
	}
#if LOGGING > 0
	Nudgematic_General_Log_Format(LOG_VERBOSITY_INTERMEDIATE,
				      "Nudgematic_Command_Offset_Size_Parse: Parsed offset size '%s' to %d.",
				      offset_size_string,(*size));
#endif /* LOGGING */
	return TRUE;
}

/**
 * Routine to convert the specified offset size into a string.
 * @param size The offset size, of type NUDGEMATIC_OFFSET_SIZE_T, to convert.
 * @return The routine returns a string based on the offset size, one of: "SMALL", "LARGE", "NONE", "ERROR".
 */
char *Nudgematic_Command_Offset_Size_To_String(NUDGEMATIC_OFFSET_SIZE_T size)
{
	switch(size)
	{
		case OFFSET_SIZE_NONE:
			return "NONE";
		case OFFSET_SIZE_SMALL:
			return "SMALL";
		case OFFSET_SIZE_LARGE:
			return "LARGE";
		default:
			return "ERROR";
	}
}

/**
 * Get the current value of the error number.
 * @return The current value of the error number.
 * @see #Command_Error_Number
 */
int Nudgematic_Command_Get_Error_Number(void)
{
	return Command_Error_Number;
}

/**
 * The error routine that reports any errors occuring in a standard way.
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Get_Current_Time_String
 */
void Nudgematic_Command_Error(void)
{
	char time_string[32];

	Nudgematic_General_Get_Current_Time_String(time_string,32);
	/* if the error number is zero an error message has not been set up
	** This is in itself an error as we should not be calling this routine
	** without there being an error to display */
	if(Command_Error_Number == 0)
		sprintf(Command_Error_String,"Logic Error:No Error defined");
	fprintf(stderr,"%s Nudgematic_Command:Error(%d) : %s\n",time_string,
		Command_Error_Number,Command_Error_String);
}

/**
 * The error routine that reports any errors occuring in a standard way. This routine places the
 * generated error string at the end of a passed in string argument.
 * @param error_string A string to put the generated error in. This string should be initialised before
 * being passed to this routine. The routine will try to concatenate it's error string onto the end
 * of any string already in existance.
 * @see #Command_Error_Number
 * @see #Command_Error_String
 * @see nudgematic_general.html#Nudgematic_General_Get_Current_Time_String
 */
void Nudgematic_Command_Error_To_String(char *error_string)
{
	char time_string[32];

	Nudgematic_General_Get_Current_Time_String(time_string,32);
	/* if the error number is zero an error message has not been set up
	** This is in itself an error as we should not be calling this routine
	** without there being an error to display */
	if(Command_Error_Number == 0)
		sprintf(Command_Error_String,"Logic Error:No Error defined");
	sprintf(error_string+strlen(error_string),"%s Nudgematic_Command:Error(%d) : %s\n",time_string,
		Command_Error_Number,Command_Error_String);
}

/* =======================================
**  internal functions 
** ======================================= */
/**
 * Parse a reply string from the nudgematic. These are usually in the form: '&lt;pc&gt; &lt;adu&gt; &lt;pe&gt; &lt;nc&gt; &lt;t&gt;' where
 * <dl>
 * <dt>pc</dt> <dd>Position character, one of 'a','b','c','d','e','A','B','C','D','E','w','W'</dd>
 * <dt>adu</dt> <dd>An integer, the ADU of the position potentiometer for this cam.</dd>
 * <dt>pe</dt> <dd>The error (number of ADUs) between the attained position and it's ideal position.</dd>
 * <dt>nc</dt> <dd>The number of nudges used to attain the position.</dd>
 * <dt>t</dt> <dd>How long it took to attain the position, in milliseconds.</dd>
 * </dl>
 * @param reply_string The reply received from the Arduino.
 * @param position_char The address of a character, on return from the routine the position character.
 * @param position_adu The address of an integer, on return from the routine the relevant potentiometer ADUS (proxy for angle).
 * @param position_error The address of an integer, on return from the routine error (number of ADUs) between the 
 *                       attained position and it's ideal position.
 * @param nudges The address of an integer, on return from the routine the number of nudges used to attain the position.
 * @param time_ms The address of an integer, on return how long it took to attain the position, in milliseconds.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Command_Error_Number
 * @see #Command_Error_String
 */
static int Command_Parse_Reply_String(char *reply_string,char *position_char,int *position_adu,int *position_error,int *nudges,int *time_ms)
{
	int retval;
	
	if(reply_string == NULL)
	{
		Command_Error_Number = 11;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:reply_string was NULL.");
		return FALSE;		
	}
	if(position_char == NULL)
	{
		Command_Error_Number = 12;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:position_char was NULL.");
		return FALSE;		
	}
	if(position_adu == NULL)
	{
		Command_Error_Number = 13;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:position_adu was NULL.");
		return FALSE;		
	}
	if(position_error == NULL)
	{
		Command_Error_Number = 14;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:position_error was NULL.");
		return FALSE;		
	}
	if(nudges == NULL)
	{
		Command_Error_Number = 15;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:nudges was NULL.");
		return FALSE;		
	}
	if(time_ms == NULL)
	{
		Command_Error_Number = 16;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:time_ms was NULL.");
		return FALSE;		
	}
	retval = sscanf(reply_string,"%c %d %d %d %d",position_char,position_adu,position_error,nudges,time_ms);
	if(retval != 5)
	{
		Command_Error_Number = 17;
		sprintf(Command_Error_String,"Command_Parse_Reply_String:Failed to parse reply_string '%s' (%d).",reply_string,retval);
		return FALSE;		
	}
	return TRUE;
}
