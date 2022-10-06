/* raptor_multrun.c
** Raptor multrun routines
*/
/**
 * Multrun routines for the raptor program.
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
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "log_udp.h"

#include "detector_buffer.h"
#include "detector_exposure.h"
#include "detector_fits_filename.h"
#include "detector_fits_header.h"
#include "detector_setup.h"
#include "detector_temperature.h"

#include "filter_wheel_command.h"
#include "filter_wheel_config.h"

#include "nudgematic_command.h"

#include "raptor_config.h"
#include "raptor_fits_header.h"
#include "raptor_general.h"
#include "raptor_multrun.h"

/* hash defines */
/**
 * Length of FITS filename string.
 */
#define MULTRUN_FITS_FILENAME_LENGTH  (256)

/* data types */
/**
 * Data type holding local data to raptor multruns.
 * <dl>
 * <dt>CCD_Temperature</dt> <dd>A copy of the current CCD temperature, taken at the start of a multrun. 
 *                              Used to populate FITS headers.</dd>
 * <dt>Image_Index</dt> <dd>Which frame in the multrun we are currently working on.</dd>
 * <dt>Image_Count</dt> <dd>The number of FITS images we are expecting to generate in the current multrun.</dd>
 * <dt>Multrun_Start_Time</dt> <dd>A timestamp taken the first time an exposure was started in the multrun.</dd>
 * </dl>
 */
struct Multrun_Struct
{
	double CCD_Temperature;
	int Image_Index;
	int Image_Count;
	struct timespec Multrun_Start_Time;
};

/* internal data */
/**
 * Revision Control System identifier.
 */
static char rcsid[] = "$Id$";
/**
 * Multrun Data holding local data to raptor multruns.
 * <dl>
 * <dt>CCD_Temperature</dt>               <dd>0.0</dd>
 * <dt>Image_Index</dt>                   <dd>0</dd>
 * <dt>Image_Count</dt>                   <dd>0</dd>
 * <dt>Multrun_Start_Time</dt>            <dd>{0,0}</dd>
 * </dl>
 * @see #Multrun_Struct
 */
static struct Multrun_Struct Multrun_Data =
{
	0.0,0,0,{0,0}
};

/**
 * Is a multrun in progress.
 */
static int Multrun_In_Progress = FALSE;
/**
 * Abort any multrun currently in progress.
 */
static int Moptop_Abort = FALSE;

/* ----------------------------------------------------------------------------
** 		external functions 
** ---------------------------------------------------------------------------- */
/**
 * Routine to perform a multrun.
 * <ul>
 * <li>We check the input arguments are valid.
 * <li>We initialise the internal variables.
 * <li>We call Detector_Fits_Filename_Next_Multrun to generate FITS filenames for a new Multrun.
 * <li>We figure out the DETECTOR_FITS_FILENAME_EXPOSURE_TYPE to use, based on the do_standard flag.
 * <li>TODO make any per-multrun FITS header changes here.
 * <li>We take a multrun start timestamp.
 * <li>We enter a for loop, looping Multrun_Data.Image_Index over Multrun_Data.Image_Count.
 *     <ul>
 *     <li>We check Moptop_Abort to see if the multrun has been aborted by another command thread.
 *     <li>We move the nudgematic by calling Nudgematic_Command_Position_Set.
 *     <li>We call Detector_Fits_Filename_Next_Run to increment the run number in the FITS filename generation code.
 *     <li>We call Detector_Fits_Filename_Get_Filename to generate a suitable FITS image filename.
 *     <li>We check Moptop_Abort to see if the multrun has been aborted by another command thread.
 *     <li>TODO Make any per-exposure FITS header changes here.
 *     <li>We call Detector_Exposure_Expose to take the image (a series of coadds) and save it to the FITS image filename.
 *     <li>We call Detector_Fits_Filename_List_Add to add the new FITS image filename to the return list of filenames.
 *     <li>We increment, and potentially reset the nudgematic position to use for the next exposure in the multrun.
 *     </ul>
 * <li>We set Multrun_In_Progress to FALSE, to indicate we have finished the Multrun.
 * </ul>
 * @param exposure_length_ms The exposure length of an individual frame in the multrun (itself consisting of a number
 *        of coadds) in milliseconds.
 * @param exposure_count The number of exposure to perform in the multrun.
 * @param do_standard A boolea, TRUE if the multrun is a standard, and FALSE if it is not. This effacts the resultant
 *        FITS image filenames.
 * @param filename_list The address of a list of strings, on a successful return from this routine an allocated list 
 *        of strings will be returned (of length exposure_count), each string containing a FITS image filename
 *        of one frame/exposure in the multrun. This list will need freeing.
 * @param filename_count The address of an integer, on a successful return from this routine contains the
 *        number of filenames in filename_list.
 * @return The routine returns TRUE on sucess and FALSE on failure. On failure, Raptor_General_Error_Number and
 *         Raptor_General_Error_String should be set.
 * @see #Moptop_Abort
 * @see #Multrun_In_Progress
 * @see #Multrun_Data
 * @see raptor_general.htmlAPTOR_GENERAL_IS_BOOLEAN
 * @see raptor_general.html#Raptor_General_Error_Number
 * @see raptor_general.html#Raptor_General_Error_String
 * @see raptor_general.html#Raptor_General_Log
 * @see raptor_general.html#Raptor_General_Log_Format
 * @see ../nudgematic/cdocs/nudgematic_command.html#NUDGEMATIC_POSITION_COUNT
 * @see ../nudgematic/cdocs/nudgematic_command.html#Nudgematic_Command_Position_Set
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Expose
 * @see ../detector/cdocs/detector_fits_filename.html#DETECTOR_FITS_FILENAME_PIPELINE_FLAG
 * @see ../detector/cdocs/detector_fits_filename.html#DETECTOR_FITS_FILENAME_EXPOSURE_TYPE
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Next_Multrun
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Next_Run
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Get_Filename
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_List_Add
 */
int Raptor_Multrun(int exposure_length_ms,int exposure_count,int do_standard,
		   char ***filename_list,int *filename_count)
{
	char fits_filename[256];
	enum DETECTOR_FITS_FILENAME_EXPOSURE_TYPE fits_filename_exposure_type;
	int nudgematic_position_index = 0;
	
	/* check arguments */
	if(exposure_length_ms < 1)
	{
		Raptor_General_Error_Number = 600;
		sprintf(Raptor_General_Error_String,
			"Raptor_Multrun:exposure length was too short (%d).",exposure_length_ms);
		return FALSE;
	}
	if(exposure_count < 1)
	{
		Raptor_General_Error_Number = 601;
		sprintf(Raptor_General_Error_String,
			"Raptor_Multrun:exposure count was too small (%d).",exposure_count);
		return FALSE;
	}
	if(!RAPTOR_GENERAL_IS_BOOLEAN(do_standard))
	{
		Raptor_General_Error_Number = 602;
		sprintf(Raptor_General_Error_String,
			"Raptor_Multrun:do_standard was not a valid boolean (%d).",do_standard);
		return FALSE;
	}
	if(filename_list == NULL)	
	{
		Raptor_General_Error_Number = 603;
		sprintf(Raptor_General_Error_String,"Raptor_Multrun:filename_list was NULL.");
		return FALSE;
	}
	if(filename_count == NULL)	
	{
		Raptor_General_Error_Number = 604;
		sprintf(Raptor_General_Error_String,"Raptor_Multrun:filename_count was NULL.");
		return FALSE;
	}
#if RAPTOR_DEBUG > 1
	Raptor_General_Log_Format("multrun","raptor_multrun.c","Raptor_Startup_Detector",LOG_VERBOSITY_TERSE,
				  "MULTRUN","Started with exposure_length %d ms, exposure count %d.",
				  exposure_length_ms,exposure_count);
#endif
	/* initialise internal variables */
	Multrun_In_Progress = TRUE;
	Moptop_Abort = FALSE;
	Multrun_Data.Image_Count = exposure_count;
	nudgematic_position_index = 0;
	(*filename_list) = NULL;
	(*filename_count) = 0;
	/* intialise FITS filenames for new multrun*/
	if(!Detector_Fits_Filename_Next_Multrun())
	{
		Multrun_In_Progress = FALSE;
		Raptor_General_Error_Number = 605;
		sprintf(Raptor_General_Error_String,"Raptor_Multrun:Failed to initialise FITS filename multrun.");
		return FALSE;
	}
	if(do_standard)
		fits_filename_exposure_type = DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_STANDARD;
	else
		fits_filename_exposure_type = DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_EXPOSURE;
	/* do any per-multrun FITS header changes here */
	/* diddly */
	/* take a multrun start timestamp */
	clock_gettime(CLOCK_REALTIME,&(Multrun_Data.Multrun_Start_Time));
	/* start multrun for loop */
	for(Multrun_Data.Image_Index = 0; Multrun_Data.Image_Index < Multrun_Data.Image_Count;
	    Multrun_Data.Image_Index++)
	{
		/* check for aborts */
		if(Moptop_Abort)
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 606;
			sprintf(Raptor_General_Error_String,"Raptor_Multrun:Aborted.");
			return FALSE;
		}
		/* move to next nudgematic position */
		/* diddly we have bypassed nudgematic.enable here - fix! */
		if(!Nudgematic_Command_Position_Set(nudgematic_position_index))
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 607;
			sprintf(Raptor_General_Error_String,
				"Raptor_Multrun:Failed to move Nudgematic to position %d.",
				nudgematic_position_index);
			return FALSE;
		}
		/* generate new FITS image filename */
		if(!Detector_Fits_Filename_Next_Run())
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 608;
			sprintf(Raptor_General_Error_String,"Raptor_Multrun:Failed to generate next FITS filename run number.");
			return FALSE;
		}
		if(!Detector_Fits_Filename_Get_Filename(fits_filename_exposure_type,
							DETECTOR_FITS_FILENAME_PIPELINE_FLAG_UNREDUCED,
							fits_filename,256))
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 609;
			sprintf(Raptor_General_Error_String,"Raptor_Multrun:Failed to generate next FITS filename.");
			return FALSE;
		}
		/* check for aborts */
		if(Moptop_Abort)
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 610;
			sprintf(Raptor_General_Error_String,"Raptor_Multrun:Aborted.");
			return FALSE;
		}
		/* do any per-multrun frame FITS header changes here */
		/* diddly */
		/* take an exposure */
		if(!Detector_Exposure_Expose(exposure_length_ms,fits_filename))
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 611;
			sprintf(Raptor_General_Error_String,
				"Raptor_Multrun:Failed to take exposure %d of %d ms with filename '%s'.",
				Multrun_Data.Image_Index,exposure_length_ms,fits_filename);
			return FALSE;
		}
		/* add fits image to list */
		if(!Detector_Fits_Filename_List_Add(fits_filename,filename_list,filename_count))
		{
			Multrun_In_Progress = FALSE;
			Raptor_General_Error_Number = 612;
			sprintf(Raptor_General_Error_String,"Raptor_Multrun:Failed to add filename '%s' to list of length %d.",
				fits_filename,(*filename_count));
			return FALSE;
		}
		/* increment nudgematic position index */
		nudgematic_position_index++;
		if(nudgematic_position_index == NUDGEMATIC_POSITION_COUNT)
			nudgematic_position_index = 0;
	}/* end for on Multrun_Data.Image_Index */
	/* we have finished the multrun */
	Multrun_In_Progress = FALSE;
#if RAPTOR_DEBUG > 1
	Raptor_General_Log("multrun","raptor_multrun.c","Raptor_Startup_Detector",LOG_VERBOSITY_TERSE,"MULTRUN",
			   "Finished.");
#endif		
	return TRUE;
}

/**
 * Routine to abort an in-progress multrun.
 * @return The routine nominally returns TRUE on success and FALSE on failure. 
 *         However, it currently always succeeds (returns TRUE).
 * @see #Moptop_Abort
 */
int Raptor_Multrun_Abort(void)
{
	Moptop_Abort = TRUE;
	return TRUE;
}

/**
 * Return whether we are currently performing a multrun or not.
 * @return A boolean, TRUE if a multrun is in progress and FALSE if it is not in progress..
 * @see #Multrun_In_Progress
 */
int Raptor_Multrun_In_Progress(void)
{
	return Multrun_In_Progress;
}

/**
 * Return the total number of exposures expected to be generated in the current/last multrun.
 * @return The number of images/frames expected.
 * @see #Multrun_Data
 */
int Raptor_Multrun_Count_Get(void)
{
	return Multrun_Data.Image_Count;
}

/**
 * Return which exposure in the multrun we are on.
 * @return The exposure index in the multrun.
 * @see #Multrun_Data
 */
int Raptor_Multrun_Exposure_Index_Get(void)
{
	return Multrun_Data.Image_Index;
}
