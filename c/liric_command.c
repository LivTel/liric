/* liric_command.c
** Liric command routines
*/
/**
 * Command routines for Liric.
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

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/**
 * Add more fields to struct tm (tm_tm_zone).
 */
#define _BSD_SOURCE
#include <time.h>
#include <unistd.h>

#include "log_udp.h"

#include "detector_exposure.h"
#include "detector_fits_filename.h"
#include "detector_fits_header.h"
#include "detector_general.h"
#include "detector_setup.h"
#include "detector_temperature.h"

#include "command_server.h"

#include "filter_wheel_command.h"
#include "filter_wheel_config.h"
#include "filter_wheel_general.h"

#include "nudgematic_command.h"

#include "liric_bias_dark.h"
#include "liric_command.h"
#include "liric_config.h"
#include "liric_fits_header.h"
#include "liric_multrun.h"
#include "liric_general.h"
#include "liric_server.h"

/* hash defines */
/**
 * Timezone offset for 1 hour.
 */
#define TIMEZONE_OFFSET_HOUR (3600)
/**
 * Timezone offset for BST.
 */
#define TIMEZONE_OFFSET_BST  (TIMEZONE_OFFSET_HOUR)

/* internal data */
/**
 * Revision Control System identifier.
 */
static char rcsid[] = "$Id$";

/* internal functions */
static int Command_Parse_Date(char *time_string,int *time_secs);

/* ----------------------------------------------------------------------------
** 		external functions 
** ---------------------------------------------------------------------------- */
/**
 * Handle a command of the form: "abort".
 * <ul>
 * <li>We abort any running multrun's by calling Liric_Multrun_Abort.
 * <li>We abort any running multbias/multdarks by calling Liric_Bias_Dark_Abort.
 * <li>We abort any running exposures by calling Detector_Exposure_Abort.
 * <li>We set the reply_string to a successful message.
 * </ul>
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_multrun.html#Liric_Multrun_Abort
 * @see liric_bias_dark.html#Liric_Bias_Dark_Abort
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Abort
 */
int Liric_Command_Abort(char *command_string,char **reply_string)
{
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Abort",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* abort multrun */
#if LIRIC_DEBUG > 5
	Liric_General_Log("command","liric_command.c","Liric_Command_Abort",LOG_VERBOSITY_INTERMEDIATE,
			   "COMMAND","Aborting multrun.");
#endif
	Liric_Multrun_Abort();
#if LIRIC_DEBUG > 5
	Liric_General_Log("command","liric_command.c","Liric_Command_Abort",LOG_VERBOSITY_INTERMEDIATE,
			   "COMMAND","Aborting bias/darks.");
#endif
	Liric_Bias_Dark_Abort();
	/* abort exposure */
#if LIRIC_DEBUG > 5
	Liric_General_Log("command","liric_command.c","Liric_Command_Abort",LOG_VERBOSITY_INTERMEDIATE,
			   "COMMAND","Aborting multrun.");
#endif
	Detector_Exposure_Abort();
	/* return success */
	if(!Liric_General_Add_String(reply_string,"0 Multrun/Bias/Dark aborted."))
		return FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Abort",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Handle config commands of the forms:
 * <ul>
 * <li>"config coadd_exp_len <short|long>"
 * <li>"config filter <filtername>"
 * <li>"config nudgematic <none|small|large>"
 * </ul>
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #Liric_Command_Initialise_Detector
 * @see liric_config.html#Liric_Config_Nudgematic_Is_Enabled
 * @see liric_config.html#Liric_Config_Filter_Wheel_Is_Enabled
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_general.html#Liric_General_Add_Integer_To_String
 * @see ../filter_wheel/cdocs/filter_wheel_config.html#Filter_Wheel_Config_Name_To_Position
 * @see ../filter_wheel/cdocs/filter_wheel_command.html#Filter_Wheel_Command_Move
 * @see ../nudgematic/cdocs/nudgematic_command.html#NUDGEMATIC_OFFSET_SIZE_T
 * @see ../nudgematic/cdocs/nudgematic_command.html#Nudgematic_Command_Offset_Size_Set
 * @see ../nudgematic/cdocs/nudgematic_command.html#Nudgematic_Command_Offset_Size_To_String
 */
int Liric_Command_Config(char *command_string,char **reply_string)
{
	NUDGEMATIC_OFFSET_SIZE_T offset_size;
	int retval,bin,parameter_index,filter_position;
	double camera_exposure_length;
	char filter_string[32];
	char coadd_exposure_length_keyword_string[32];
	char coadd_exposure_length_string[32];
	char nudgematic_offset_size_string[32];
	char sub_config_command_string[16];

#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Config",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* parse command */
	retval = sscanf(command_string,"config %15s %n",sub_config_command_string,&parameter_index);
	if(retval != 1)
	{
		Liric_General_Error_Number = 501;
		sprintf(Liric_General_Error_String,"Liric_Command_Config:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_Config",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Config",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse config command."))
			return FALSE;
		return TRUE;
	}
#if LIRIC_DEBUG > 9
	Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",LOG_VERBOSITY_VERY_VERBOSE,
				  "COMMAND","Sub config command string: %s, parameter index %d.",
				  sub_config_command_string,parameter_index);
#endif
        if(strcmp(sub_config_command_string,"coadd_exp_len") == 0)
	{
		retval = sscanf(command_string+parameter_index,"%31s",coadd_exposure_length_string);
		if(retval != 1)
		{
			Liric_General_Error_Number = 502;
			sprintf(Liric_General_Error_String,"Liric_Command_Config:"
				"Failed to parse command %s (%d).",command_string,retval);
			Liric_General_Error("command","liric_command.c","Liric_Command_Config",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log("command","liric_command.c","Liric_Command_Config",
					   LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse config coadd_exp_len command."))
				return FALSE;
			return TRUE;
		}
		/* re-initialise the connection to the detector, using the
		** new coadd_exposure length */
		if(!Liric_Command_Initialise_Detector(coadd_exposure_length_string))
		{
			Liric_General_Error_Number = 506;
			sprintf(Liric_General_Error_String,"Liric_Command_Config:"
				"Failed to initialise detector with coadd exposure length: '%s'.",
				coadd_exposure_length_string);
			Liric_General_Error("command","liric_command.c","Liric_Command_Config",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
						  LOG_VERBOSITY_TERSE,"COMMAND",
						  "Failed to initialise detector with coadd exposure length: '%s'.",
						  coadd_exposure_length_string);
#endif
			if(!Liric_General_Add_String(reply_string,
						      "1 Failed to initialise detector with coadd exposure length:"))
				return FALSE;
			if(!Liric_General_Add_String(reply_string,coadd_exposure_length_string))
				return FALSE;
			return TRUE;
		}
		if(!Liric_General_Add_String(reply_string,"0 Coadd exposure length set to:"))
			return FALSE;
		if(!Liric_General_Add_String(reply_string,coadd_exposure_length_string))
			return FALSE;
	}
	else if(strcmp(sub_config_command_string,"filter") == 0)
	{
		/* copy rest of command as filter name - filter names have spaces in them! */
		strncpy(filter_string,command_string+parameter_index,31);
#if LIRIC_DEBUG > 5
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
					  LOG_VERBOSITY_VERBOSE,"COMMAND","Setting Filter to: %s.",filter_string);
#endif
		if(Liric_Config_Filter_Wheel_Is_Enabled())
		{
			/* string to position conversion */
			if(!Filter_Wheel_Config_Name_To_Position(filter_string,&filter_position))
			{
				Liric_General_Error_Number = 503;
				sprintf(Liric_General_Error_String,"Liric_Command_Config:"
					"Failed to convert filter name '%s' to a valid filter position.",filter_string);
				Liric_General_Error("command","liric_command.c","Liric_Command_Config",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to convert filter name '%s' to a valid filter position.",
							  filter_string);
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to convert filter name:"))
					return FALSE;
				if(!Liric_General_Add_String(reply_string,filter_string))
					return FALSE;
				return TRUE;
			}
#if LIRIC_DEBUG > 9
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
					  LOG_VERBOSITY_VERY_VERBOSE,"COMMAND","Filter position: %d.",filter_position);
#endif
			/* actually move filter wheel */
			if(!Filter_Wheel_Command_Move(filter_position))
			{
				Liric_General_Error_Number = 504;
				sprintf(Liric_General_Error_String,"Liric_Command_Config:"
					"Failed to move filter wheel to filter '%s', position %d.",
					filter_string,filter_position);
				Liric_General_Error("command","liric_command.c","Liric_Command_Config",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to move filter wheel to filter '%s', position %d.",
							  filter_string,filter_position);
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to move filter wheel to filter:"))
					return FALSE;
				if(!Liric_General_Add_String(reply_string,filter_string))
					return FALSE;
				return TRUE;
			}
			/* success */
			if(!Liric_General_Add_String(reply_string,"0 Filter wheel moved to position:"))
				return FALSE;
			if(!Liric_General_Add_String(reply_string,filter_string))
				return FALSE;
		}
		else /* filter wheel is not enabled */
		{
			/* success */
			if(!Liric_General_Add_String(reply_string,"0 Filter Wheel not enabled."))
				return FALSE;
		}
	}
	else if(strcmp(sub_config_command_string,"nudgematic") == 0)
	{
		retval = sscanf(command_string+parameter_index,"%31s",nudgematic_offset_size_string);
		if(retval != 1)
		{
			Liric_General_Error_Number = 533;
			sprintf(Liric_General_Error_String,"Liric_Command_Config:"
				"Failed to parse command %s (%d).",command_string,retval);
			Liric_General_Error("command","liric_command.c","Liric_Command_Config",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log("command","liric_command.c","Liric_Command_Config",
					   LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse config nudgematic command."))
				return FALSE;
			return TRUE;
		}
#if LIRIC_DEBUG > 5
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
					  LOG_VERBOSITY_VERBOSE,"COMMAND","Setting nudgematic offset size to: %s.",
					  nudgematic_offset_size_string);
#endif
		if(strcmp(nudgematic_offset_size_string,"none") == 0)
		{
			offset_size = OFFSET_SIZE_NONE;
		}
		else if(strcmp(nudgematic_offset_size_string,"small") == 0)
		{
			offset_size = OFFSET_SIZE_SMALL;
		}
		else if(strcmp(nudgematic_offset_size_string,"large") == 0)
		{
			offset_size = OFFSET_SIZE_LARGE;
		}
		else
		{
			Liric_General_Error_Number = 534;
			sprintf(Liric_General_Error_String,"Liric_Command_Config:Unknown nudgematic offset size %s.",
				nudgematic_offset_size_string);
			Liric_General_Error("command","liric_command.c","Liric_Command_Config",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
						  LOG_VERBOSITY_TERSE,"COMMAND",
						  "finished (Unknown nudgematic offset size %s).",
						  nudgematic_offset_size_string);
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse config nudgematic command:"))
				return FALSE;
			if(!Liric_General_Add_String(reply_string,command_string))
				return FALSE;
			return TRUE;
		}
		/* actually configure offset size, if nudgematic is enabled */
		if(Liric_Config_Nudgematic_Is_Enabled())
		{
			if(!Nudgematic_Command_Offset_Size_Set(offset_size))
			{
				Liric_General_Error_Number = 508;
				sprintf(Liric_General_Error_String,"Liric_Command_Config:Failed to configure offset size %s.",
					Nudgematic_Command_Offset_Size_To_String(offset_size));
				Liric_General_Error("command","liric_command.c","Liric_Command_Config",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Config",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "finished Failed to configure offset size %s.",
							  Nudgematic_Command_Offset_Size_To_String(offset_size));
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to parse config nudgematic command:"))
					return FALSE;
				if(!Liric_General_Add_String(reply_string,command_string))
					return FALSE;
				return TRUE;
			}
			/* cache the nudgematic offset size setting in the multrun data for FITS header generation */
		}/* end if nudgematic is enabled */
		if(!Liric_General_Add_String(reply_string,"0 Config nudgematic completed."))
			return FALSE;
	}
	else
	{
		if(!Liric_General_Add_String(reply_string,"1 Unknown config sub-command:"))
			return FALSE;
		if(!Liric_General_Add_String(reply_string,sub_config_command_string))
			return FALSE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Config",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Command to turn the camera head fan on or off: "fan <on|off>".
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_general.html#Liric_General_Add_Integer_To_String
 * @see ../detector/cdocs/detector_temperature.html#Detector_Temperature_Set_Fan
 */
int Liric_Command_Fan(char *command_string,char **reply_string)
{
	int retval,onoff,parameter_index;
	char onoff_string[16];

#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Fan",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* parse command */
	retval = sscanf(command_string,"fan %15s",onoff_string);
	if(retval != 1)
	{
		Liric_General_Error_Number = 548;
		sprintf(Liric_General_Error_String,"Liric_Command_Fan:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_Fan",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Fan",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse fan command."))
			return FALSE;
		return TRUE;
	}
	/* parse onoff string */
	if(strcmp(onoff_string,"on")==0)
	{
		onoff = TRUE;
	}
	else if(strcmp(onoff_string,"off")==0)
	{
		onoff = FALSE;
	}
	else
	{
		Liric_General_Error_Number = 549;
		sprintf(Liric_General_Error_String,"Liric_Command_Fan:"
			"Unknown fan state %s:Failed to parse command %s.",onoff_string,command_string);
		Liric_General_Error("command","liric_command.c","Liric_Command_Fan",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fan",
					  LOG_VERBOSITY_TERSE,
					  "COMMAND","Unknown fan state %s:Failed to parse command %s.",
					  onoff_string,command_string);
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse fan command: Unknown fan state."))
			return FALSE;
		return TRUE;
	}
	/* actually change the fan state */
	if(!Detector_Temperature_Set_Fan(onoff))
	{
		Liric_General_Error_Number = 550;
		sprintf(Liric_General_Error_String,"Liric_Command_Fan:"
			"Failed to set fan state.");
		Liric_General_Error("command","liric_command.c","Liric_Command_Fan",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Fan",
				   LOG_VERBOSITY_TERSE,"COMMAND","Failed to set fan state.");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to set fan state."))
			return FALSE;
		return TRUE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Fan",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Implementation of FITS Header commands.
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_fits_header.html#Liric_Fits_Header_Logical_Add
 * @see liric_fits_header.html#Liric_Fits_Header_Float_Add
 * @see liric_fits_header.html#Liric_Fits_Header_Integer_Add
 * @see liric_fits_header.html#Liric_Fits_Header_String_Add
 * @see liric_fits_header.html#Liric_Fits_Header_Clear
 * @see liric_fits_header.html#Liric_Fits_Header_Delete
 */
int Liric_Command_Fits_Header(char *command_string,char **reply_string)
{
	char operation_string[8];
	char keyword_string[13];
	char type_string[8];
	char value_string[80];
	int retval,command_string_index,ivalue,value_index;
	double dvalue;

	/* parse command to retrieve operation*/
	retval = sscanf(command_string,"fitsheader %6s %n",operation_string,&command_string_index);
	if((retval != 1)&&(retval != 2)) /* sscanf isn't sure whether %n increments returned value! */
	{
		Liric_General_Error_Number = 517;
		sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
			"Failed to parse command %s (%d).",command_string,retval);
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Fits_Header",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse fitsheader command."))
			return FALSE;
		return TRUE;
	}
	/* do operation */
	if(strncmp(operation_string,"add",3) == 0)
	{
		retval = sscanf(command_string+command_string_index,"%12s %7s %n",keyword_string,type_string,
				&value_index);
		if((retval != 3)&&(retval != 2)) /* %n may or may not increment retval*/
		{
			Liric_General_Error_Number = 518;
			sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
				"Failed to parse add command %s (%d).",command_string,retval);
			Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log("command","liric_command.c","Liric_Command_Fits_Header",
					   LOG_VERBOSITY_TERSE,"COMMAND","finished (add command parse failed).");
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse fitsheader add command."))
				return FALSE;
			return TRUE;
		}
		strncpy(value_string,command_string+command_string_index+value_index,79);
		value_string[79] = '\0';
		if(strncmp(type_string,"boolean",7)==0)
		{
			/* parse value */
			if(strncmp(value_string,"true",4) == 0)
				ivalue = TRUE;
			else if(strncmp(value_string,"false",5) == 0)
				ivalue = FALSE;
			else
			{
				Liric_General_Error_Number = 519;
				sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
					"Add boolean command had unknown value %s.",value_string);
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Add boolean command had unknown value %s.",value_string);
#endif
				if(!Liric_General_Add_String(reply_string,
							   "1 Failed to parse fitsheader add boolean command value."))
					return FALSE;
				return TRUE;
			}
			/* do operation */
			if(!Liric_Fits_Header_Logical_Add(keyword_string,ivalue,NULL))
			{
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to add boolean to FITS header.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to add boolean fits header."))
					return FALSE;
				return TRUE;
			}
		}
		else if(strncmp(type_string,"comment",7)==0)
		{
			/* do operation */
			if(!Liric_Fits_Header_Add_Comment(keyword_string,value_string))
			{
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to add comment to FITS header.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to add comment to fits header."))
					return FALSE;
				return TRUE;
			}
		}
		else if(strncmp(type_string,"float",5)==0)
		{
			/* parse value */
			retval = sscanf(value_string,"%lf",&dvalue);
			if(retval != 1)
			{
				Liric_General_Error_Number = 520;
				sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
					"Add float command had unknown value %s.",value_string);
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Add float command had unknown value %s.",value_string);
#endif
				if(!Liric_General_Add_String(reply_string,
							   "1 Failed to parse fitsheader add float command value."))
					return FALSE;
				return TRUE;
			}
			/* do operation */
			if(!Liric_Fits_Header_Float_Add(keyword_string,dvalue,NULL))
			{
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to add float to FITS header.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to add float fits header."))
					return FALSE;
				return TRUE;
			}
		}
		else if(strncmp(type_string,"integer",7)==0)
		{
			/* parse value */
			retval = sscanf(value_string,"%d",&ivalue);
			if(retval != 1)
			{
				Liric_General_Error_Number = 521;
				sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
					"Add integer command had unknown value %s.",value_string);
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Add integer command had unknown value %s.",value_string);
#endif
				if(!Liric_General_Add_String(reply_string,
							   "1 Failed to parse fitsheader add integer command value."))
					return FALSE;
				return TRUE;
			}
			/* do operation */
			if(!Liric_Fits_Header_Integer_Add(keyword_string,ivalue,NULL))
			{
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to add integer to FITS header.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to add integer fits header."))
					return FALSE;
				return TRUE;
			}
		}
		else if(strncmp(type_string,"string",6)==0)
		{
			/* do operation */
			if(!Liric_Fits_Header_String_Add(keyword_string,value_string,NULL))
			{
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to add string to FITS header.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to add string fits header."))
					return FALSE;
				return TRUE;
			}
		}
		else if(strncmp(type_string,"units",5)==0)
		{
			/* do operation */
			if(!Liric_Fits_Header_Add_Units(keyword_string,value_string))
			{
				Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to add units to FITS header.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to add units to fits header."))
					return FALSE;
				return TRUE;
			}
		}
		else
		{
			Liric_General_Error_Number = 522;
			sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
				"Add command had unknown type %s.",type_string);
			Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
				       LOG_VERBOSITY_TERSE,"COMMAND","Add command had unknown type %s.",type_string);
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse fitsheader add command type."))
				return FALSE;
			return TRUE;
		}
	}
	else if(strncmp(operation_string,"delete",6) == 0)
	{
		retval = sscanf(command_string+command_string_index,"%12s",keyword_string);
		if(retval != 1)
		{
			Liric_General_Error_Number = 523;
			sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
				"Failed to parse delete command %s (%d).",command_string,retval);
			Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log("command","liric_command.c","Liric_Command_Fits_Header",
					   LOG_VERBOSITY_TERSE,"COMMAND","finished (delete command parse failed).");
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse fitsheader delete command."))
				return FALSE;
			return TRUE;
		}
		/* do delete */
		if(!Liric_Fits_Header_Delete(keyword_string))
		{
			Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
						  LOG_VERBOSITY_TERSE,"COMMAND",
						  "Failed to delete FITS header with keyword '%s'.",keyword_string);
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to delete fits header."))
				return FALSE;
			return TRUE;
		}
	}
	else if(strncmp(operation_string,"clear",5) == 0)
	{
		/* do clear */
		if(!Liric_Fits_Header_Clear())
		{
			Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log("command","liric_command.c","Liric_Command_Fits_Header",
					   LOG_VERBOSITY_TERSE,"COMMAND","Failed to clear FITS header.");
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to clear fits header."))
				return FALSE;
			return TRUE;
		}
	}
	else
	{
		Liric_General_Error_Number = 524;
		sprintf(Liric_General_Error_String,"Liric_Command_Fits_Header:"
			"Unknown operation %s:Failed to parse command %s.",operation_string,command_string);
		Liric_General_Error("command","liric_command.c","Liric_Command_Fits_Header",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Fits_Header",
					  LOG_VERBOSITY_TERSE,
					  "COMMAND","Unknown operation %s:Failed to parse command %s.",
					  operation_string,command_string);
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse fitsheader command: Unknown operation."))
			return FALSE;
		return TRUE;
	}
	if(!Liric_General_Add_String(reply_string,"0 FITS Header command succeeded."))
		return FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Fits_Header",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Handle a command of the form: "multrun <length> <count> <standard>".
 * <ul>
 * <li>The multrun command is parsed to get the exposure length, count and standard (true|false) values.
 * <li>We call Liric_Multrun to take the multrun images.
 * <li>The reply string is constructed of the form "0 <filename count> <multrun number> <last FITS filename>".
 * <li>We log the returned filenames.
 * <li>We free the returned filenames.
 * </ul>
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_multrun.html#Liric_Multrun
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Multrun_Get
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_List_Free
 */
int Liric_Command_Multrun(char *command_string,char **reply_string)
{
	struct timespec start_time = {0L,0L};
	char **filename_list = NULL;
	char standard_string[8];
	char count_string[16];
	int i,retval,exposure_length,exposure_count,filename_count,do_standard,multrun_number;

#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Multrun",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* parse command */
	retval = sscanf(command_string,"multrun %d %d %7s",&exposure_length,&exposure_count,standard_string);
	if(retval != 3)
	{
		Liric_General_Error_Number = 505;
		sprintf(Liric_General_Error_String,"Liric_Command_Multrun:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_Multrun",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Multrun",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse multrun command."))
			return FALSE;
		return TRUE;
	}
	/* parse standard string */
	if(strcmp(standard_string,"true") == 0)
		do_standard = TRUE;
	else if(strcmp(standard_string,"false") == 0)
		do_standard = FALSE;
	else
	{
		Liric_General_Error_Number = 539;
		sprintf(Liric_General_Error_String,"Liric_Command_Multrun:Illegal standard value '%s'.",
			standard_string);
		Liric_General_Error("command","liric_command.c","Liric_Command_Multrun",
				     LOG_VERBOSITY_TERSE,"COMMAND");
		if(!Liric_General_Add_String(reply_string,"1 Multrun failed:Illegal standard value."))
			return FALSE;
		return TRUE;
	}
	/* do multrun */
	retval = Liric_Multrun(exposure_length,exposure_count,do_standard,&filename_list,&filename_count);
	if(retval == FALSE)
	{
		Liric_General_Error("command","liric_command.c","Liric_Command_Multrun",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Multrun",
				   LOG_VERBOSITY_TERSE,"COMMAND","Multrun failed.");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Multrun failed."))
			return FALSE;
		return TRUE;
	}
	/* success */
	if(!Liric_General_Add_String(reply_string,"0 "))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* add number of FITS images */
	sprintf(count_string,"%d ",filename_count);
	if(!Liric_General_Add_String(reply_string,count_string))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* get multrun number */
	multrun_number = Detector_Fits_Filename_Multrun_Get();
	sprintf(count_string,"%d ",multrun_number);
	if(!Liric_General_Add_String(reply_string,count_string))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* add last filename */
	if(filename_count > 0)
	{
		if(!Liric_General_Add_String(reply_string,filename_list[filename_count-1]))
		{
			Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
			return FALSE;
		}
	}
	else
	{
		if(!Liric_General_Add_String(reply_string,"none"))
		{
			Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
			return FALSE;
		}
	}
	/* log filenames returned */
	for(i=0; i < filename_count; i++)
	{
#if LIRIC_DEBUG > 8
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Multrun",
					  LOG_VERBOSITY_VERY_VERBOSE,"COMMAND","Filename %d : %s",i,filename_list[i]);
#endif
	}
	if(!Detector_Fits_Filename_List_Free(&filename_list,&filename_count))
	{
		Liric_General_Error_Number = 510;
		sprintf(Liric_General_Error_String,"Liric_Command_Multrun:Detector_Fits_Filename_List_Free failed.");
		Liric_General_Error("command","liric_command.c","Liric_Command_Multrun",
				     LOG_VERBOSITY_TERSE,"COMMAND");
		if(!Liric_General_Add_String(reply_string,"1 Multrun failed (freeing filename list)."))
			return FALSE;
		return TRUE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Multrun",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Handle a command of the form: "multbias <count>".
 * <ul>
 * <li>The multbias command is parsed to get the exposure count value.
 * <li>We call Liric_Bias_Dark_MultBias to take the multiple bias images.
 * <li>The reply string is constructed of the form "0 <filename count> <multrun number> <last FITS filename>".
 * <li>We log the returned filenames.
 * <li>We free the returned filenames.
 * </ul>
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_bias_dark.html#Liric_Bias_Dark_MultBias
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Multrun_Get
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_List_Free
 */
int Liric_Command_MultBias(char *command_string,char **reply_string)
{
	struct timespec start_time = {0L,0L};
	char **filename_list = NULL;
	char standard_string[8];
	char count_string[16];
	int i,retval,exposure_length,exposure_count,filename_count,do_standard,multrun_number;

#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_MultBias",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* parse command */
	retval = sscanf(command_string,"multbias %d",&exposure_count);
	if(retval != 1)
	{
		Liric_General_Error_Number = 544;
		sprintf(Liric_General_Error_String,"Liric_Command_MultBias:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_MultBias",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_MultBias",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse multbias command."))
			return FALSE;
		return TRUE;
	}
	/* do multbias */
	retval = Liric_Bias_Dark_MultBias(exposure_count,&filename_list,&filename_count);
	if(retval == FALSE)
	{
		Liric_General_Error("command","liric_command.c","Liric_Command_MultBias",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_MultBias",
				   LOG_VERBOSITY_TERSE,"COMMAND","MultBias failed.");
#endif
		if(!Liric_General_Add_String(reply_string,"1 MultBias failed."))
			return FALSE;
		return TRUE;
	}
	/* success */
	if(!Liric_General_Add_String(reply_string,"0 "))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* add number of FITS images */
	sprintf(count_string,"%d ",filename_count);
	if(!Liric_General_Add_String(reply_string,count_string))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* get multrun number */
	multrun_number = Detector_Fits_Filename_Multrun_Get();
	sprintf(count_string,"%d ",multrun_number);
	if(!Liric_General_Add_String(reply_string,count_string))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* add last filename */
	if(filename_count > 0)
	{
		if(!Liric_General_Add_String(reply_string,filename_list[filename_count-1]))
		{
			Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
			return FALSE;
		}
	}
	else
	{
		if(!Liric_General_Add_String(reply_string,"none"))
		{
			Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
			return FALSE;
		}
	}
	/* log filenames returned */
	for(i=0; i < filename_count; i++)
	{
#if LIRIC_DEBUG > 8
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_MultBias",
					  LOG_VERBOSITY_VERY_VERBOSE,"COMMAND","Filename %d : %s",i,filename_list[i]);
#endif
	}
	if(!Detector_Fits_Filename_List_Free(&filename_list,&filename_count))
	{
		Liric_General_Error_Number = 545;
		sprintf(Liric_General_Error_String,"Liric_Command_MultBias:Detector_Fits_Filename_List_Free failed.");
		Liric_General_Error("command","liric_command.c","Liric_Command_MultBias",
				     LOG_VERBOSITY_TERSE,"COMMAND");
		if(!Liric_General_Add_String(reply_string,"1 MultBias failed (freeing filename list)."))
			return FALSE;
		return TRUE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_MultBias",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Handle a command of the form: "multdark <length> <count>".
 * <ul>
 * <li>The multdark command is parsed to get the exposure length, and exposure count values.
 * <li>We call Liric_Bias_Dark_MultDark to take the dark images.
 * <li>The reply string is constructed of the form "0 <filename count> <multrun number> <last FITS filename>".
 * <li>We log the returned filenames.
 * <li>We free the returned filenames.
 * </ul>
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_bias_dark.html#Liric_Bias_Dark_MultDark
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Multrun_Get
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_List_Free
 */
int Liric_Command_MultDark(char *command_string,char **reply_string)
{
	struct timespec start_time = {0L,0L};
	char **filename_list = NULL;
	char count_string[16];
	int i,retval,exposure_length,exposure_count,filename_count,multrun_number;

#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_MultDark",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* parse command */
	retval = sscanf(command_string,"multdark %d %d",&exposure_length,&exposure_count);
	if(retval != 2)
	{
		Liric_General_Error_Number = 546;
		sprintf(Liric_General_Error_String,"Liric_Command_MultDark:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_MultDark",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_MultDark",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse multdark command."))
			return FALSE;
		return TRUE;
	}
	/* do multdark */
	retval = Liric_Bias_Dark_MultDark(exposure_length,exposure_count,&filename_list,&filename_count);
	if(retval == FALSE)
	{
		Liric_General_Error("command","liric_command.c","Liric_Command_MultDark",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_MultDark",
				   LOG_VERBOSITY_TERSE,"COMMAND","MultDark failed.");
#endif
		if(!Liric_General_Add_String(reply_string,"1 MultDark failed."))
			return FALSE;
		return TRUE;
	}
	/* success */
	if(!Liric_General_Add_String(reply_string,"0 "))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* add number of FITS images */
	sprintf(count_string,"%d ",filename_count);
	if(!Liric_General_Add_String(reply_string,count_string))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* get multrun number */
	multrun_number = Detector_Fits_Filename_Multrun_Get();
	sprintf(count_string,"%d ",multrun_number);
	if(!Liric_General_Add_String(reply_string,count_string))
	{
		Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
		return FALSE;
	}
	/* add last filename */
	if(filename_count > 0)
	{
		if(!Liric_General_Add_String(reply_string,filename_list[filename_count-1]))
		{
			Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
			return FALSE;
		}
	}
	else
	{
		if(!Liric_General_Add_String(reply_string,"none"))
		{
			Detector_Fits_Filename_List_Free(&filename_list,&filename_count);
			return FALSE;
		}
	}
	/* log filenames returned */
	for(i=0; i < filename_count; i++)
	{
#if LIRIC_DEBUG > 8
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_MultDark",
					  LOG_VERBOSITY_VERY_VERBOSE,"COMMAND","Filename %d : %s",i,filename_list[i]);
#endif
	}
	if(!Detector_Fits_Filename_List_Free(&filename_list,&filename_count))
	{
		Liric_General_Error_Number = 547;
		sprintf(Liric_General_Error_String,"Liric_Command_MultDark:Detector_Fits_Filename_List_Free failed.");
		Liric_General_Error("command","liric_command.c","Liric_Command_MultDark",
				     LOG_VERBOSITY_TERSE,"COMMAND");
		if(!Liric_General_Add_String(reply_string,"1 MultDark failed (freeing filename list)."))
			return FALSE;
		return TRUE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_MultDark",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Handle a status command. Possible forms: 
 * <ul>
 * <li>status temperature [get|status]
 * <li>status filterwheel [filter|position|status]
 * <li>status nudgematic [position|status|offsetsize|temperature]
 * <li>status exposure [status|count|length|start_time]
 * <li>status exposure [index|multrun|run]
 * </ul>
 * <ul>
 * <li>The status command is parsed to retrieve the subsystem (1st parameter).
 * <li>Based on the subsystem, further parsing occurs.
 * <li>The relevant status is retrieved, and a suitable reply constructed.
 * </ul>
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_bias_dark.html#Liric_Bias_Dark_In_Progress
 * @see liric_bias_dark.html#Liric_Bias_Dark_Count_Get
 * @see liric_bias_dark.html#Liric_Bias_Dark_Exposure_Index_Get
 * @see liric_config.html#Liric_Config_Filter_Wheel_Is_Enabled
 * @see liric_config.html#Liric_Config_Nudgematic_Is_Enabled
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_general.html#Liric_General_Get_Time_String
 * @see liric_general.html#Liric_General_Get_Current_Time_String
 * @see liric_multrun.html#Liric_Multrun_In_Progress
 * @see liric_multrun.html#Liric_Multrun_Count_Get
 * @see liric_multrun.html#Liric_Multrun_Exposure_Index_Get
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Exposure_Length_Get
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Start_Time_Get
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_In_Progress
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Multrun_Get
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Run_Get
 * @see ../detector/cdocs/detector_temperature.html#Detector_Temperature_Get
 * @see ../detector/cdocs/detector_temperature.html#Detector_Temperature_PCB_Get
 * @see ../filter_wheel/cdocs/filter_wheel_command.html#Filter_Wheel_Command_Get_Position
 * @see ../nudgematic/cdocs/nudgematic_command.html#Nudgematic_Command_Position_Get
 * @see ../nudgematic/cdocs/nudgematic_command.html#Nudgematic_Command_Temperature_Get
 * @see ../nudgematic/cdocs/nudgematic_command.html#NUDGEMATIC_OFFSET_SIZE_T
 * @see ../nudgematic/cdocs/nudgematic_command.html#Nudgematic_Command_Offset_Size_Get
 */
int Liric_Command_Status(char *command_string,char **reply_string)
{
	NUDGEMATIC_OFFSET_SIZE_T offset_size;
	struct timespec status_time;
	char time_string[32];
	char return_string[128];
	char subsystem_string[32];
	char get_set_string[16];
	char key_string[64];
	char temperature_status_string[32];
	char filter_name_string[32];
	char *camera_name_string = NULL;
	int retval,command_string_index,ivalue,filter_wheel_position,nudgematic_position;
	double temperature,temp1,temp2,temp3;

	/* parse command */
	retval = sscanf(command_string,"status %31s %n",subsystem_string,&command_string_index);
	if((retval != 1)&&(retval != 2)) /* sscanf isn't sure whether %n increments returned value! */
	{
		Liric_General_Error_Number = 511;
		sprintf(Liric_General_Error_String,"aptor_Command_Status:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_Status",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Status",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse status command."))
			return FALSE;
		return TRUE;
	}
	
	/* initialise return string */
	strcpy(return_string,"0 ");
	/* parse subsystem */
	if(strncmp(subsystem_string,"exposure",8) == 0)
	{
		if(strncmp(command_string+command_string_index,"status",6)==0)
		{
			if(Detector_Exposure_In_Progress())
				strcat(return_string,"true");
			else
				strcat(return_string,"false");
			/*
			if(Liric_Multrun_In_Progress())
				strcat(return_string,"true");
			else if(Liric_Bias_Dark_In_Progress())
				strcat(return_string,"true");
			else
				strcat(return_string,"false");
			*/
		}
		else if(strncmp(command_string+command_string_index,"coadd-length",12)==0)
		{
			ivalue = Detector_Exposure_Coadd_Frame_Exposure_Length_Get();
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else if(strncmp(command_string+command_string_index,"coadd-count",12)==0)
		{
			ivalue = Detector_Exposure_Coadd_Count_Get();
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else if(strncmp(command_string+command_string_index,"count",5)==0)
		{
			if(Liric_Multrun_In_Progress())
				ivalue = Liric_Multrun_Count_Get();
			else if(Liric_Bias_Dark_In_Progress())
				ivalue = Liric_Bias_Dark_Count_Get();
			else
				ivalue = 0;
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else if(strncmp(command_string+command_string_index,"length",6)==0)
		{
			ivalue = Detector_Exposure_Exposure_Length_Get();
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else if(strncmp(command_string+command_string_index,"start_time",10)==0)
		{
			status_time = Detector_Exposure_Start_Time_Get();
			Liric_General_Get_Time_String(status_time,time_string,31);
			sprintf(return_string+strlen(return_string),"%s",time_string);
		}
		else if(strncmp(command_string+command_string_index,"index",5)==0)
		{
			if(Liric_Multrun_In_Progress())
				ivalue = Liric_Multrun_Exposure_Index_Get();
			else if(Liric_Bias_Dark_In_Progress())
				ivalue = Liric_Bias_Dark_Exposure_Index_Get();
			else
				ivalue = 0;
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else if(strncmp(command_string+command_string_index,"multrun",7)==0)
		{
			ivalue = Detector_Fits_Filename_Multrun_Get();
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else if(strncmp(command_string+command_string_index,"run",3)==0)
		{
			ivalue = Detector_Fits_Filename_Run_Get();
			sprintf(return_string+strlen(return_string),"%d",ivalue);
		}
		else
		{
			Liric_General_Error_Number = 512;
			sprintf(Liric_General_Error_String,"Liric_Command_Status:"
				"Failed to parse exposure status command %s.",command_string+command_string_index);
			Liric_General_Error("command","liric_command.c","Liric_Command_Status",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",
						  LOG_VERBOSITY_TERSE,"COMMAND",
						  "Failed to parse exposure status command %s.",
						  command_string+command_string_index);
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse exposure status command."))
				return FALSE;
			return TRUE;
		}
	}
	else if(strncmp(subsystem_string,"filterwheel",11) == 0)
	{
		if(Liric_Config_Filter_Wheel_Is_Enabled())
		{
			if(!Filter_Wheel_Command_Get_Position(&filter_wheel_position))
			{
				Liric_General_Error_Number = 509;
				sprintf(Liric_General_Error_String,"Liric_Command_Status:"
					"Failed to get filter wheel position.");
				Liric_General_Error("command","liric_command.c","Liric_Command_Status",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log("command","liric_command.c","Liric_Command_Status",
						   LOG_VERBOSITY_TERSE,"COMMAND","Failed to get filter wheel position.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to get filter wheel position."))
					return FALSE;
				return TRUE;
			}
		}/* end if filter wheel is enabled */
		else
		{
#if LIRIC_DEBUG > 5
			Liric_General_Log("command","liric_command.c","Liric_Command_Status",
					   LOG_VERBOSITY_VERBOSE,"COMMAND",
					   "Liric filter wheel is NOT enabled, faking filter wheel position to 0 (moving).");
#endif
			/* we pretend the filter wheel is moving when it is not enabled */
			filter_wheel_position = 0;
		}
		if(strncmp(command_string+command_string_index,"filter",6)==0)
		{
			if(filter_wheel_position == 0) /* moving */
			{
				strcpy(filter_name_string,"moving");
			}
			else
			{
				if(!Filter_Wheel_Config_Position_To_Name(filter_wheel_position,filter_name_string))
				{
					Liric_General_Error_Number = 514;
					sprintf(Liric_General_Error_String,"Liric_Command_Status:"
						"Failed to get filter wheel filter name from position %d.",
						filter_wheel_position);
					Liric_General_Error("command","liric_command.c","Liric_Command_Status",
							     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
					Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",
								  LOG_VERBOSITY_TERSE,"COMMAND",
						  "Failed to get filter wheel filter name from position %d.",
								  filter_wheel_position);
#endif
					if(!Liric_General_Add_String(reply_string,
							"1 Failed to get filter wheel filter name from position:"))
						return FALSE;
					if(!Liric_General_Add_Integer_To_String(reply_string,filter_wheel_position))
						return FALSE;
					return TRUE;
				}
			}
			strcat(return_string,filter_name_string);
		}
		else if(strncmp(command_string+command_string_index,"position",8)==0)
		{
			sprintf(return_string+strlen(return_string),"%d",filter_wheel_position);
		}
		else if(strncmp(command_string+command_string_index,"status",6)==0)
		{
			if(filter_wheel_position == 0)/* moving */
				strcat(return_string,"moving");
			else
				strcat(return_string,"in_position");
		}
		else
		{
			Liric_General_Error_Number = 525;
			sprintf(Liric_General_Error_String,"Liric_Command_Status:"
				"Failed to parse filterwheel command %s.",command_string+command_string_index);
			Liric_General_Error("command","liric_command.c","Liric_Command_Status",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",
						  LOG_VERBOSITY_TERSE,"COMMAND",
						  "Failed to parse filterwheel command %s.",
						  command_string+command_string_index);
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse filterwheel status command."))
				return FALSE;
			return TRUE;
		}
	}
	else if(strncmp(subsystem_string,"nudgematic",7) == 0)
	{
		if(Liric_Config_Nudgematic_Is_Enabled())
		{
			if(!Nudgematic_Command_Position_Get(&nudgematic_position))
			{
				Liric_General_Error_Number = 541;
				sprintf(Liric_General_Error_String,"Liric_Command_Status:"
					"Failed to get nudgematic position.");
				Liric_General_Error("command","liric_command.c","Liric_Command_Status",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log("command","liric_command.c","Liric_Command_Status",
						   LOG_VERBOSITY_TERSE,"COMMAND","Failed to get nudgematic position.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to get nudgematic position."))
					return FALSE;
				return TRUE;
			}
			if(strncmp(command_string+command_string_index,"position",8)==0)
			{
				sprintf(return_string+strlen(return_string),"%d",nudgematic_position);			
			}
			else if(strncmp(command_string+command_string_index,"status",6)==0)
			{
				if(nudgematic_position == -1)
					strcat(return_string,"moving");
				else
					strcat(return_string,"stopped");
			}
			else if(strncmp(command_string+command_string_index,"offsetsize",10)==0)
			{
				/* this should always work if offset_size is non-NULL */
				Nudgematic_Command_Offset_Size_Get(&offset_size);
				if(offset_size == OFFSET_SIZE_NONE)
				{
					strcat(return_string,"none");
				}
				else if(offset_size == OFFSET_SIZE_SMALL)
				{
					strcat(return_string,"small");
				}
				else if(offset_size == OFFSET_SIZE_LARGE)
				{
					strcat(return_string,"large");
				}
				else
				{
					strcat(return_string,"UNKNOWN");
				}
			}
			else if(strncmp(command_string+command_string_index,"temperature",11)==0)
			{
				if(!Nudgematic_Command_Temperature_Get(&temp1,&temp2,&temp3))
				{
					Liric_General_Error_Number = 553;
					sprintf(Liric_General_Error_String,"Liric_Command_Status:"
						"Failed to get nudgematic temperatures.");
					Liric_General_Error("command","liric_command.c","Liric_Command_Status",
							    LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
					Liric_General_Log("command","liric_command.c","Liric_Command_Status",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to get nudgematic temperatures.");
#endif
					if(!Liric_General_Add_String(reply_string,
								     "1 Failed to get nudgematic temperatures."))
						return FALSE;
					return TRUE;
				}
				Liric_General_Get_Current_Time_String(time_string,31);
				sprintf(return_string+strlen(return_string),"%s %.2f %.2f %.2f",time_string,
					temp1,temp2,temp3);
			}
			else
			{
				Liric_General_Error_Number = 543;
				sprintf(Liric_General_Error_String,"Liric_Command_Status:"
					"Failed to parse nudgematic command %s.",command_string+command_string_index);
				Liric_General_Error("command","liric_command.c","Liric_Command_Status",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to parse status nudgematic command %s.",
							  command_string+command_string_index);
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to parse status nudgematic command."))
					return FALSE;
				return TRUE;
			}
		}
		else /* nudgematic is _not_ enabled */
		{
			if(strncmp(command_string+command_string_index,"position",8)==0)
			{
				nudgematic_position = -1; /* moving */
				sprintf(return_string+strlen(return_string),"%d",nudgematic_position);			
			}
			else if(strncmp(command_string+command_string_index,"status",6)==0)
			{
				strcat(return_string,"stopped"); 
			}
			else if(strncmp(command_string+command_string_index,"offsetsize",10)==0)
			{
				strcat(return_string,"UNKNOWN"); 
			}
			else if(strncmp(command_string+command_string_index,"temperature",11)==0)
			{
				strcat(return_string,"UNKNOWN"); 
			}
			else
			{
				Liric_General_Error_Number = 542;
				sprintf(Liric_General_Error_String,"Liric_Command_Status:"
					"Failed to parse nudgematic command %s.",command_string+command_string_index);
				Liric_General_Error("command","liric_command.c","Liric_Command_Status",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",
							  LOG_VERBOSITY_TERSE,"COMMAND",
							  "Failed to parse status nudgematic command %s.",
							  command_string+command_string_index);
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to parse status nudgematic command."))
					return FALSE;
				return TRUE;
			}			
		}
	}
	else if(strncmp(subsystem_string,"temperature",11) == 0)
	{
		retval = sscanf(command_string,"status temperature %15s",get_set_string);
		if(retval != 1)
		{
			Liric_General_Error_Number = 526;
			sprintf(Liric_General_Error_String,"Liric_Command_Status:"
				"Failed to parse command %s (%d).",command_string,retval);
			Liric_General_Error("command","liric_command.c","Liric_Command_Status",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log("command","liric_command.c","Liric_Command_Status",
					   LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
			if(!Liric_General_Add_String(reply_string,
						      "1 Failed to parse status temperature ."))
				return FALSE;
			return TRUE;
		}
		/* check subcommand */
		if(strncmp(get_set_string,"get",3)==0)
		{
			if(!Detector_Temperature_Get(&temperature))
			{
				Liric_General_Error_Number = 513;
				sprintf(Liric_General_Error_String,"Liric_Command_Status:"
					"Failed to get temperature.");
				Liric_General_Error("command","liric_command.c","Liric_Command_Status",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log("command","liric_command.c","Liric_Command_Status",
						   LOG_VERBOSITY_TERSE,"COMMAND","Failed to get temperature.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to get temperature."))
					return FALSE;
				return TRUE;
			}
			Liric_General_Get_Current_Time_String(time_string,31);
			sprintf(return_string+strlen(return_string),"%s %.2f",time_string,temperature);
		}
		else if(strncmp(get_set_string,"pcb",6)==0)
		{
			if(!Detector_Temperature_PCB_Get(&temperature))
			{
				Liric_General_Error_Number = 507;
				sprintf(Liric_General_Error_String,"Liric_Command_Status:"
					"Failed to get PCB temperature.");
				Liric_General_Error("command","liric_command.c","Liric_Command_Status",
						     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
				Liric_General_Log("command","liric_command.c","Liric_Command_Status",
						   LOG_VERBOSITY_TERSE,"COMMAND","Failed to get PCB temperature.");
#endif
				if(!Liric_General_Add_String(reply_string,"1 Failed to get PCB temperature."))
					return FALSE;
				return TRUE;
			}
			Liric_General_Get_Current_Time_String(time_string,31);
			sprintf(return_string+strlen(return_string),"%s %.2f",time_string,temperature);
		}
		else
		{
			Liric_General_Error_Number = 515;
			sprintf(Liric_General_Error_String,"Liric_Command_Status:"
				"Failed to parse temperature command %s from %d.",command_string,command_string_index);
			Liric_General_Error("command","liric_command.c","Liric_Command_Status",
					     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
			Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",
						  LOG_VERBOSITY_TERSE,"COMMAND",
						  "Failed to parse temperature command %s from %d.",
						  command_string,command_string_index);
#endif
			if(!Liric_General_Add_String(reply_string,"1 Failed to parse temperature status command."))
				return FALSE;
			return TRUE;
		}
	}
	else
	{
		Liric_General_Error_Number = 516;
		sprintf(Liric_General_Error_String,"Liric_Command_Status:"
			"Unknown subsystem %s:Failed to parse command %s.",subsystem_string,command_string);
		Liric_General_Error("command","liric_command.c","Liric_Command_Status",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Status",LOG_VERBOSITY_TERSE,
					  "COMMAND","Unknown subsystem %s:Failed to parse command %s.",
					  subsystem_string,command_string);
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse status command."))
			return FALSE;
		return TRUE;
	}
	/* success */
	if(!Liric_General_Add_String(reply_string,return_string))
		return FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Status",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * Command to set the detectors target temperature : "temperature <degrees centigrade>".
 * Note this will only take effect until the next Config/coadd exposure length change occurs,
 * as loading a new .fmt file will reset the target to that contained in the .fmt file.
 * @param command_string The command. This is not changed during this routine.
 * @param reply_string The address of a pointer to allocate and set the reply string.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Add_String
 * @see liric_general.html#Liric_General_Add_Integer_To_String
 * @see ../detector/cdocs/detector_temperature.html#Detector_Temperature_Set_TEC_Setpoint
 */
int Liric_Command_Temperature(char *command_string,char **reply_string)
{
	int retval,parameter_index;
	double target_temperature;

#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Temperature",LOG_VERBOSITY_TERSE,
			   "COMMAND","started.");
#endif
	/* parse command */
	retval = sscanf(command_string,"temperature %lf",&target_temperature);
	if(retval != 1)
	{
		Liric_General_Error_Number = 551;
		sprintf(Liric_General_Error_String,"Liric_Command_Temperature:"
			"Failed to parse command %s (%d).",command_string,retval);
		Liric_General_Error("command","liric_command.c","Liric_Command_Temperature",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Temperature",
				       LOG_VERBOSITY_TERSE,"COMMAND","finished (command parse failed).");
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to parse temperature command."))
			return FALSE;
		return TRUE;
	}
	/* change the detector's target temperature
	** Note this will only take effect until the next Config/coadd exposure length change occurs,
	** as loading a new .fmt file will reset the target to that contained in the .fmt file. */
	if(!Detector_Temperature_Set_TEC_Setpoint(target_temperature))
	{
		Liric_General_Error_Number = 552;
		sprintf(Liric_General_Error_String,"Liric_Command_Temperature:"
			"Failed to set detector target temperature to %.2f C.",target_temperature);
		Liric_General_Error("command","liric_command.c","Liric_Command_Temperature",
				     LOG_VERBOSITY_TERSE,"COMMAND");
#if LIRIC_DEBUG > 1
		Liric_General_Log_Format("command","liric_command.c","Liric_Command_Temperature",
					 LOG_VERBOSITY_TERSE,"COMMAND",
					 "Failed to set detector target temperature to %.2f C.",
					 target_temperature);
#endif
		if(!Liric_General_Add_String(reply_string,"1 Failed to set detector target temperatur."))
			return FALSE;
		return TRUE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Temperature",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/**
 * On a change in coadd exposure length, the whole connection to xclib / the detector needs to be closed and
 * re-opened again: the library needs to read a different format '.fmt' file.
 * <ul>
 * <li>We retrieve the "detector.enable" flag from config. If it is FALSE (detector not enabled) we just return from
 *     this routine successfully with a suitable log message.
 * <li>We construct a suitable config keyword ""detector.coadd_exposure_length." with the coadd_exposure_length_string
 *     argument appended.
 * <li>We retrieve a suitable config exposure length (in milliseconds) from the constructed keyword.
 * <li>We retrieve the detector format file directory from the "detector.format_dir" config item.
 * <li>We construct a suitable format_filename from the above information.
 * <li>We call Detector_Setup_Startup with the specified format_filename.
 * <li>We call Detector_Exposure_Set_Coadd_Frame_Exposure_Length so the detector exposure code knows what
 *     the new coadd exposure length is.
 * </ul>
 * @param coadd_exposure_length_string A string representing the length of coadd exposure, should normally be
 *        one of "short" or "long".
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see liric_config.html#Liric_Config_Get_Integer
 * @see liric_config.html#Liric_Config_Get_Boolean
 * @see liric_config.html#Liric_Config_Get_String
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Log_Format
 * @see ../detector/cdocs/detector_setup.html#Detector_Setup_Startup
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Set_Coadd_Frame_Exposure_Length
 */
int Liric_Command_Initialise_Detector(char *coadd_exposure_length_string)
{
	int enabled,coadd_exposure_length;
	char format_filename[256];
	char keyword_string[64];
	char* format_dir_string = NULL;
	
	if(coadd_exposure_length_string == NULL)
	{
		Liric_General_Error_Number = 527;
		sprintf(Liric_General_Error_String,"Liric_Command_Initialise_Detector:"
			"coadd_exposure_length_string was NULL.");
		return FALSE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log_Format("command","liric_command.c","Liric_Command_Initialise_Detector",LOG_VERBOSITY_TERSE,
				  "COMMAND","Started with exposure length '%s'.",coadd_exposure_length_string);
#endif
	/* do we want to initialise the detector.
	** The C layer always has a detector attached, but if it is unplugged/broken, setting "detector.enable" to FALSE
	** allows the C layer to initialise to enable control of other mechanisms from the C layer. */
	if(!Liric_Config_Get_Boolean("detector.enable",&enabled))
	{
		Liric_General_Error_Number = 535;
		sprintf(Liric_General_Error_String,"Liric_Command_Initialise_Detector:"
			"Failed to get whether the detector is enabled for initialisation.");
		return FALSE;
	}
	/* if we don't want to initialise the detector, just return here. */
	if(enabled == FALSE)
	{
#if LIRIC_DEBUG > 1
		Liric_General_Log("command","liric_command.c","Liric_Command_Initialise_Detector",
				   LOG_VERBOSITY_TERSE,"COMMAND","Finished (Detector NOT enabled).");
#endif
		return TRUE;
	}
	/* get the coadd exposure length from the passed in string, via the config. */
	sprintf(keyword_string,"detector.coadd_exposure_length.%s",coadd_exposure_length_string);
	if(!Liric_Config_Get_Integer(keyword_string,&coadd_exposure_length))
	{
		Liric_General_Error_Number = 536;
		sprintf(Liric_General_Error_String,
			"Liric_Command_Initialise_Detector:Failed to get coadd exposure length for keyword '%s'.",
			keyword_string);
		return FALSE;
	}
	/* get the '.fmt' format directory from config */
	if(!Liric_Config_Get_String("detector.format_dir",&format_dir_string))
	{
		Liric_General_Error_Number = 537;
		sprintf(Liric_General_Error_String,
			"Liric_Command_Initialise_Detector:Failed to get detector format directory.");
		return FALSE;
	}
	sprintf(format_filename,"%s/rap_%dms.fmt",format_dir_string,coadd_exposure_length);	
	if(format_dir_string != NULL)
		free(format_dir_string);
	/* actually do initialisation of the detector library */
#if LIRIC_DEBUG > 1
	Liric_General_Log_Format("command","liric_command.c","Liric_Command_Initialise_Detector",LOG_VERBOSITY_TERSE,
				  "COMMAND","Calling Detector_Setup_Startup with format filename '%s'.",
				  format_filename);
#endif
	if(!Detector_Setup_Startup(format_filename))
	{
		Liric_General_Error_Number = 538;
		sprintf(Liric_General_Error_String,
			"Liric_Command_Initialise_Detector:Detector_Setup_Startup failed.");
		return FALSE;
	}
	/* setup coadd exposure length */
	if(!Detector_Exposure_Set_Coadd_Frame_Exposure_Length(coadd_exposure_length))
	{
		Liric_General_Error_Number = 540;
		sprintf(Liric_General_Error_String,
			"Liric_Command_Initialise_Detector:Detector_Exposure_Set_Coadd_Frame_Exposure_Length failed.");
		return FALSE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log("command","liric_command.c","Liric_Command_Initialise_Detector",LOG_VERBOSITY_TERSE,
			   "COMMAND","finished.");
#endif
	return TRUE;
}

/* ----------------------------------------------------------------------------
** 		internal functions 
** ---------------------------------------------------------------------------- */
/**
 * Parse a date of the form "2007-05-03T07:38:48.099 UTC" into number of seconds since 1970 (unix time).
 * @param time_string The string.
 * @param time_secs The address of an integer to store the number of seconds.
 * @return The routine returns TRUE on success and FALSE on failure.
 * @see #TIMEZONE_OFFSET_BST
 */
static int Command_Parse_Date(char *time_string,int *time_secs)
{
	struct tm time_data;
	int year,month,day,hours,minutes,retval;
	double seconds;
	char timezone_string[16];
	time_t time_in_secs;

	/* check parameters */
	if(time_string == NULL)
	{
		Liric_General_Error_Number = 528;
		sprintf(Liric_General_Error_String,"Command_Parse_Date:time_string was NULL.");
		return FALSE;
	}
	if(time_secs == NULL)
	{
		Liric_General_Error_Number = 529;
		sprintf(Liric_General_Error_String,"Command_Parse_Date:time_secs was NULL.");
		return FALSE;
	}
#if LIRIC_DEBUG > 9
	Liric_General_Log_Format("command","liric_command.c","Command_Parse_Date",LOG_VERBOSITY_TERSE,
				  "COMMAND","Parsing date/time '%s'.",time_string);
#endif
	/* parse time_string into fields */
	strcpy(timezone_string,"UTC");
	retval = sscanf(time_string,"%d-%d-%d T %d:%d:%lf %15s",&year,&month,&day,
			&hours,&minutes,&seconds,timezone_string);
	if(retval < 6)
	{
		Liric_General_Error_Number = 530;
		sprintf(Liric_General_Error_String,
			"Command_Parse_Date:Failed to parse '%s', only parsed %d fields: year=%d,month=%d,day=%d,"
			"hour=%d,minute=%d,second=%.2f,timezone_string=%s.",time_string,retval,year,month,day,
			hours,minutes,seconds,timezone_string);
		return FALSE;
	}
#if LIRIC_DEBUG > 9
	Liric_General_Log_Format("command","liric_command.c","Command_Parse_Date",LOG_VERBOSITY_TERSE,"COMMAND",
			    "Date/time '%s' has year=%d,month=%d,day=%d,hour=%d,minute=%d,seconds=%.2lf,timezone=%s.",
				  time_string,year,month,day,hours,minutes,seconds,timezone_string);
#endif
	/* construct tm */
	time_data.tm_year  = year-1900; /* years since 1900 */
	time_data.tm_mon = month-1; /* 0..11 */
	time_data.tm_mday  = day; /* 1..31 */
	time_data.tm_hour  = hours; /* 0..23 */
	time_data.tm_min   = minutes;
	time_data.tm_sec   = seconds;
	time_data.tm_wday  = 0;
	time_data.tm_yday  = 0;
	time_data.tm_isdst = 0;
	/* BSD extension stuff */
	/*
	time_data.tm_gmtoff = 0;
	time_data.tm_zone = strdup(timezone_string);
	*/
	/* create time in UTC */
	time_in_secs = mktime(&time_data);
	if(time_in_secs < 0)
	{
		Liric_General_Error_Number = 532;
		sprintf(Liric_General_Error_String,"Command_Parse_Date:mktime failed.");
		return FALSE;
	}
	(*time_secs) = (int)time_in_secs;
	if(strcmp(timezone_string,"UTC") == 0)
	{
		/* do nothing */
		(*time_secs) = (*time_secs);
	}
	else if(strcmp(timezone_string,"GMT") == 0)
	{
		/* do nothing */
		(*time_secs) = (*time_secs);
	}
	else if(strcmp(timezone_string,"BST") == 0)
	{
		(*time_secs) += TIMEZONE_OFFSET_BST;
	}
	else
	{
		Liric_General_Error_Number = 531;
		sprintf(Liric_General_Error_String,"Command_Parse_Date:Unknown timezone '%s'.",timezone_string);
		return FALSE;
	}
	return TRUE;
}
