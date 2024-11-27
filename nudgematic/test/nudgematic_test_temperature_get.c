/* nudgematic_test_temperature_get.c */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "log_udp.h"
#include "nudgematic_general.h"
#include "nudgematic_connection.h"
#include "nudgematic_command.h"

/**
 * Test program to get the temperature data served by the nudgematic mechanism.
 */
/**
 * Length of some of the strings used in this program.
 */
#define STRING_LENGTH        (256)
/**
 * Verbosity log level : initialised to LOG_VERBOSITY_VERY_VERBOSE.
 */
static int Log_Level = LOG_VERBOSITY_VERY_VERBOSE;
/**
 * The Nudgematic Arduino USB device to connect to.
 * @see #STRING_LENGTH
 */
static char Device_Name[STRING_LENGTH];

/* internal functions */
static int Parse_Arguments(int argc, char *argv[]);
static void Help(void);

/* ------------------------------------------------------------------
**          External functions 
** ------------------------------------------------------------------ */

/**
 * Main program.
 * @param argc The number of arguments to the program.
 * @param argv An array of argument strings.
 * @see #Parse_Arguments
 * @see #Device_Name
 * @see #Position
 * @see #Log_Level
 * @see ../cdocs/nudgematic_general.html#Nudgematic_General_Set_Log_Filter_Level
 * @see ../cdocs/nudgematic_general.html#Nudgematic_General_Set_Log_Filter_Function
 * @see ../cdocs/nudgematic_general.html#Nudgematic_General_Log_Filter_Level_Absolute
 * @see ../cdocs/nudgematic_general.html#Nudgematic_General_Set_Log_Handler_Function
 * @see ../cdocs/nudgematic_general.html#Nudgematic_General_Log_Handler_Stdout
 * @see ../cdocs/nudgematic_connection.html#Nudgematic_Connection_Open
 * @see ../cdocs/nudgematic_connection.html#Nudgematic_Connection_Close
 * @see ../cdocs/nudgematic_command.html#Nudgematic_Command_Temperature_Get
 */
int main(int argc, char *argv[])
{
	double temp1,temp2,temp3;
	
       	/* parse arguments */
	fprintf(stdout,"nudgematic_test_temperature_get : Parsing Arguments.\n");
	if(!Parse_Arguments(argc,argv))
		return 1;
	/* setup logging */
	Nudgematic_General_Set_Log_Filter_Level(Log_Level);
	Nudgematic_General_Set_Log_Filter_Function(Nudgematic_General_Log_Filter_Level_Absolute);
	Nudgematic_General_Set_Log_Handler_Function(Nudgematic_General_Log_Handler_Stdout);
         /* open device */
	Nudgematic_General_Log(LOG_VERBOSITY_TERSE,"nudgematic_test_temperature_get : Connecting to controller.");
	if(!Nudgematic_Connection_Open(Device_Name))
	{
		Nudgematic_General_Error();
		return 2;
	}
	/* get temperatures */
	if(!Nudgematic_Command_Temperature_Get(&temp1,&temp2,&temp3))
	{
		Nudgematic_General_Error();
		return 4;
	}
	Nudgematic_General_Log_Format(LOG_VERBOSITY_TERSE,
		       "nudgematic_test_temperature_get : Nudgematic returned temperatures %.2f,%.2f,%.2f.",
				      temp1,temp2,temp3);
	Nudgematic_General_Log(LOG_VERBOSITY_TERSE,"nudgematic_test_temperature_get:Closing connection.");
	if(!Nudgematic_Connection_Close())
	{
		Nudgematic_General_Error();
		return 5;
	}		
	return 0;
}

/* ------------------------------------------------------------------
**          Internal functions 
** ------------------------------------------------------------------ */
/**
 * Routine to parse command line arguments.
 * @param argc The number of arguments sent to the program.
 * @param argv An array of argument strings.
 * @see #Device_Name
 * @see #Log_Level
 */
static int Parse_Arguments(int argc, char *argv[])
{
	int i,retval;

	for(i=1;i<argc;i++)
	{
		if((strcmp(argv[i],"-d")==0)||(strcmp(argv[i],"-device_name")==0))
		{
			if((i+1)<argc)
			{
				strncpy(Device_Name,argv[i+1],STRING_LENGTH-1);
				Device_Name[STRING_LENGTH-1] = '\0';
				i++;
			}
			else
			{
				fprintf(stderr,"Parse_Arguments:device_name requires a USB device name.\n");
				return FALSE;
			}
		}
		else if((strcmp(argv[i],"-help")==0))
		{
			Help();
			return FALSE;
		}
		else if((strcmp(argv[i],"-l")==0)||(strcmp(argv[i],"-log_level")==0))
		{
			if((i+1)<argc)
			{
				retval = sscanf(argv[i+1],"%d",&Log_Level);
				if(retval != 1)
				{
					fprintf(stderr,"Parse_Arguments:Failed to parse log level %s.\n",argv[i+1]);
					return FALSE;
				}
				i++;
			}
			else
			{
				fprintf(stderr,"Parse_Arguments:-log_level requires a number 0..5.\n");
				return FALSE;
			}
		}
		else
		{
			fprintf(stderr,"Parse_Arguments:argument '%s' not recognized.\n",argv[i]);
			return FALSE;
		}
	}
	return TRUE;
}

/**
 * Help routine.
 */
static void Help(void)
{
	fprintf(stdout,"Nudgematic Test Program to get the current temperatures from the nudgematicArduino :Help.\n");
	fprintf(stdout,"nudgematic_test_temperature_get -d[evice_name] <USB device> [-help]\n");
	fprintf(stdout,"\t[-l[og_level] <0..5>].\n");
}
