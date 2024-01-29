/* liric_bias_dark.c
** Liric bias and dark routines
*/
/**
 * Bias and Dark routines for the liric program.
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


#include "liric_command.h"
#include "liric_config.h"
#include "liric_fits_header.h"
#include "liric_general.h"
#include "liric_bias_dark.h"

/* hash defines */
/**
 * Length of FITS filename string.
 */
#define MULTRUN_FITS_FILENAME_LENGTH  (256)
/**
 * Conversion from degrees centigrade to Kelvin.
 */
#define CENTIGRADE_TO_KELVIN          (273.15)

/* data types */
/**
 * Data type holding local data to liric biases and darks.
 * <dl>
 * <dt>CCD_Temperature</dt> <dd>A copy of the current CCD temperature, taken at the start of the bias/dark. 
 *                              Used to populate FITS headers. In degrees centigrade.</dd>
 * <dt>Image_Index</dt> <dd>Which frame in the multbias/multdark we are currently working on.</dd>
 * <dt>Image_Count</dt> <dd>The number of FITS images we are expecting to generate in the current 
 *                          multbias/multdark.</dd>
 * <dt>Bias_Dark_Start_Time</dt> <dd>A timestamp taken the first time an exposure was started in the 
 *                                   multbias/multdark.</dd>
 * </dl>
 */
struct Bias_Dark_Struct
{
	double CCD_Temperature;
	int Image_Index;
	int Image_Count;
	struct timespec Bias_Dark_Start_Time;
};

/* internal data */
/**
 * Revision Control System identifier.
 */
static char rcsid[] = "$Id$";
/**
 * Bias/Dark Data holding local data to liric multbias/multdarks.
 * <dl>
 * <dt>CCD_Temperature</dt>               <dd>0.0</dd>
 * <dt>Image_Index</dt>                   <dd>0</dd>
 * <dt>Image_Count</dt>                   <dd>0</dd>
 * <dt>Bias_Dark_Start_Time</dt>            <dd>{0,0}</dd>
 * </dl>
 * @see #Bias_Dark_Struct
 */
static struct Bias_Dark_Struct Bias_Dark_Data =
{
	0.0,0,0,{0,0}
};

/**
 * Is a bias/dark in progress.
 */
static int Bias_Dark_In_Progress = FALSE;
/**
 * Abort any bias/dark currently in progress.
 */
static int Moptop_Abort = FALSE;

/* internal function declarations */
static int Bias_Dark_Fits_Headers_Set(int is_bias,int exposure_count);
static int Bias_Dark_Exposure_Fits_Headers_Set(void);

/* ----------------------------------------------------------------------------
** 		external functions 
** ---------------------------------------------------------------------------- */
/**
 * Routine to perform a multbias.
 * <ul>
 * <li>We check the input arguments are valid.
 * <li>We initialise the internal variables.
 * <li>We retrieve the multrun flipping configuration fron config ("liric.multrun.image.flip.[x|y]" and configure
 *     the detector exposure code appropriately (Detector_Exposure_Flip_Set).
 * <li>We move the filter wheel (if configured) to the mirror position.
 * <li>We re-configure the detector to use coadds of a minimum per-coadd exposure length, 
 *     by calling Liric_Command_Initialise_Detector with coadd exposure length string "bias".
 * <li>We call Detector_Fits_Filename_Next_Multrun to generate FITS filenames for a new Multbias.
 * <li>We call Bias_Dark_Fits_Headers_Set to make any per-multbias FITS header changes here.
 * <li>We take a multbias start timestamp.
 * <li>We enter a for loop, looping Bias_Dark_Data.Image_Index over Bias_Dark_Data.Image_Count.
 *     <ul>
 *     <li>We check Moptop_Abort to see if the multrun has been aborted by another command thread.
 *     <li>We call Detector_Fits_Filename_Next_Run to increment the run number in the FITS filename generation code.
 *     <li>We call Detector_Fits_Filename_Get_Filename to generate a suitable FITS image filename.
 *     <li>We check Moptop_Abort to see if the multdark has been aborted by another command thread.
 *     <li>We call Bias_Dark_Exposure_Fits_Headers_Set to make any per-exposure FITS header changes here.
 *     <li>We call Detector_Exposure_Bias to take the image (a single frame/coadd) and save it to the FITS image filename.
 *     <li>We call Detector_Fits_Filename_List_Add to add the new FITS image filename to the return list of filenames.
 *     </ul>
 * <li>We set Bias_Dark_In_Progress to FALSE, to indicate we have finished the Multbias.
 * </ul>
 * @param exposure_count The number of dark exposure to perform in the multbias.
 * @param filename_list The address of a list of strings, on a successful return from this routine an allocated list 
 *        of strings will be returned (of length exposure_count), each string containing a FITS image filename
 *        of one frame/exposure in the multbias. This list will need freeing.
 * @param filename_count The address of an integer, on a successful return from this routine contains the
 *        number of filenames in filename_list.
 * @return The routine returns TRUE on sucess and FALSE on failure. On failure, Liric_General_Error_Number and
 *         Liric_General_Error_String should be set.
 * @see #Moptop_Abort
 * @see #Bias_Dark_In_Progress
 * @see #Bias_Dark_Data
 * @see #Bias_Dark_Fits_Headers_Set
 * @see #Bias_Dark_Exposure_Fits_Headers_Set
 * @see liric_command.html#Liric_Command_Initialise_Detector
 * @see liric_config.html#Liric_Config_Get_Boolean
 * @see liric_config.html#Liric_Config_Filter_Wheel_Is_Enabled
 * @see liric_general.html#LIRIC_GENERAL_IS_BOOLEAN
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Log_Format
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Flip_Set
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Bias
 * @see ../detector/cdocs/detector_fits_filename.html#DETECTOR_FITS_FILENAME_PIPELINE_FLAG
 * @see ../detector/cdocs/detector_fits_filename.html#DETECTOR_FITS_FILENAME_EXPOSURE_TYPE
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Next_Multrun
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Next_Run
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Get_Filename
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_List_Add
 * @see ../filter_wheel/cdocs/filter_wheel_config.html#Filter_Wheel_Config_Name_To_Position
 * @see ../filter_wheel/cdocs/filter_wheel_command.html#ilter_Wheel_Command_Move
 */
int Liric_Bias_Dark_MultBias(int exposure_count,char ***filename_list,int *filename_count)
{
	char fits_filename[256];
	int flip_x,flip_y,mirror_filter_wheel_position;
	
	/* check arguments */
	if(exposure_count < 1)
	{
		Liric_General_Error_Number = 711;
		sprintf(Liric_General_Error_String,
			"Liric_Bias_Dark_MultBias:exposure count was too small (%d).",exposure_count);
		return FALSE;
	}
	if(filename_list == NULL)	
	{
		Liric_General_Error_Number = 712;
		sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultBias:filename_list was NULL.");
		return FALSE;
	}
	if(filename_count == NULL)	
	{
		Liric_General_Error_Number = 713;
		sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultBias:filename_count was NULL.");
		return FALSE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log_Format("multbias","liric_bias_dark.c","Liric_Bias_Dark_MultBias",LOG_VERBOSITY_TERSE,
				  "MULTBIAS","Started with exposure count %d.",exposure_count);
#endif
	/* initialise internal variables */
	Bias_Dark_In_Progress = TRUE;
	Moptop_Abort = FALSE;
	Bias_Dark_Data.Image_Count = exposure_count;
	(*filename_list) = NULL;
	(*filename_count) = 0;
	/* configure flipping of output image */
	if(!Liric_Config_Get_Boolean("liric.multrun.image.flip.x",&flip_x))
		return FALSE;		
	if(!Liric_Config_Get_Boolean("liric.multrun.image.flip.y",&flip_y))
		return FALSE;		
	Detector_Exposure_Flip_Set(flip_x,flip_y);
	/* move filter wheel to mirror position */
	if(Liric_Config_Filter_Wheel_Is_Enabled())
	{
		/* which filter position contains the AuMirror filter */
		if(!Filter_Wheel_Config_Name_To_Position("AuMirror",&mirror_filter_wheel_position))
		{
			Liric_General_Error_Number = 714;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultBias:Failed to find AuMirror filter wheel position.");
			return FALSE;
		}
		/* move filter wheel */
		if(!Filter_Wheel_Command_Move(mirror_filter_wheel_position))
		{
			Liric_General_Error_Number = 715;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultBias:Failed to move filter wheel to AuMirror position %d.",
				mirror_filter_wheel_position);
			return FALSE;
		}
	}
	/* setup detector to do minimum coadd exposure lengths for a bias */
	if(!Liric_Command_Initialise_Detector("bias"))
	{
		Bias_Dark_In_Progress = FALSE;
		return FALSE;
	}
	/* intialise FITS filenames for new multrun*/
	if(!Detector_Fits_Filename_Next_Multrun())
	{
		Bias_Dark_In_Progress = FALSE;
		Liric_General_Error_Number = 716;
		sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultBias:Failed to initialise FITS filename multrun.");
		return FALSE;
	}
	/* do any per-multbias FITS header changes here */
	if(!Bias_Dark_Fits_Headers_Set(TRUE,exposure_count))
	{
		Bias_Dark_In_Progress = FALSE;
		return FALSE;
	}
	/* take a multrun start timestamp */
	clock_gettime(CLOCK_REALTIME,&(Bias_Dark_Data.Bias_Dark_Start_Time));
	/* start multbias for loop */
	for(Bias_Dark_Data.Image_Index = 0; Bias_Dark_Data.Image_Index < Bias_Dark_Data.Image_Count;
	    Bias_Dark_Data.Image_Index++)
	{
		/* check for aborts */
		if(Moptop_Abort)
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 717;
			sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultBias:Aborted.");
			return FALSE;
		}
		/* generate new FITS image filename */
		if(!Detector_Fits_Filename_Next_Run())
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 718;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultBias:Failed to generate next FITS filename run number.");
			return FALSE;
		}
		if(!Detector_Fits_Filename_Get_Filename(DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_BIAS,
							DETECTOR_FITS_FILENAME_PIPELINE_FLAG_UNREDUCED,
							fits_filename,256))
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 719;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultBias:Failed to generate next FITS filename.");
			return FALSE;
		}
		/* check for aborts */
		if(Moptop_Abort)
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 710;
			sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultBias:Aborted.");
			return FALSE;
		}
		/* do any per-multbias frame FITS header changes here */
		if(!Bias_Dark_Exposure_Fits_Headers_Set())
		{
			Bias_Dark_In_Progress = FALSE;
			return FALSE;
		}
		/* take an exposure */
		if(!Detector_Exposure_Bias(fits_filename))
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 720;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultBias:Failed to take bias exposure %d with filename '%s'.",
				Bias_Dark_Data.Image_Index,fits_filename);
			return FALSE;
		}
		/* add fits image to list */
		if(!Detector_Fits_Filename_List_Add(fits_filename,filename_list,filename_count))
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 721;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultBias:Failed to add filename '%s' to list of length %d.",
				fits_filename,(*filename_count));
			return FALSE;
		}
	}/* end for on Bias_Dark_Data.Image_Index */
	/* we have finished the multbias */
	Bias_Dark_In_Progress = FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("multbias","liric_bias_dark.c","Liric_Bias_Dark_MultBias",LOG_VERBOSITY_TERSE,"MULTBIAS",
			   "Finished.");
#endif		
	return TRUE;
}

/**
 * Routine to perform a multdark.
 * <ul>
 * <li>We initialise the internal variables.
 * <li>We retrieve the multrun flipping configuration fron config ("liric.multrun.image.flip.[x|y]" and configure
 *     the detector exposure code appropriately (Detector_Exposure_Flip_Set).
 * <li>We move the filter wheel (if configured) to the mirror position.
 * <li>We call Detector_Fits_Filename_Next_Multrun to generate FITS filenames for a new MultDark.
 * <li>We call Bias_Dark_Fits_Headers_Set to make any per-multdark FITS header changes here.
 * <li>We take a multdark start timestamp.
 * <li>We enter a for loop, looping Bias_Dark_Data.Image_Index over Bias_Dark_Data.Image_Count.
 *     <ul>
 *     <li>We check Moptop_Abort to see if the multrun has been aborted by another command thread.
 *     <li>We call Detector_Fits_Filename_Next_Run to increment the run number in the FITS filename generation code.
 *     <li>We call Detector_Fits_Filename_Get_Filename to generate a suitable FITS image filename.
 *     <li>We check Moptop_Abort to see if the multdark has been aborted by another command thread.
 *     <li>We call Bias_Dark_Exposure_Fits_Headers_Set to make any per-exposure FITS header changes here.
 *     <li>We call Detector_Exposure_Expose to take the image (a series of coadds) and save it to the FITS image filename.
 *     <li>We call Detector_Fits_Filename_List_Add to add the new FITS image filename to the return list of filenames.
 *     </ul>
 * <li>We set Bias_Dark_In_Progress to FALSE, to indicate we have finished the Multdark.
 * </ul>
 * @param exposure_length_ms The exposure length of an individual frame in the multdark (itself consisting of a number
 *        of coadds) in milliseconds.
 * @param exposure_count The number of dark exposure to perform in the multdark.
 * @param filename_list The address of a list of strings, on a successful return from this routine an allocated list 
 *        of strings will be returned (of length exposure_count), each string containing a FITS image filename
 *        of one frame/exposure in the multrun. This list will need freeing.
 * @param filename_count The address of an integer, on a successful return from this routine contains the
 *        number of filenames in filename_list.
 * @return The routine returns TRUE on sucess and FALSE on failure. On failure, Liric_General_Error_Number and
 *         Liric_General_Error_String should be set.
 * @see #Moptop_Abort
 * @see #Bias_Dark_In_Progress
 * @see #Bias_Dark_Data
 * @see #Bias_Dark_Fits_Headers_Set
 * @see #Bias_Dark_Exposure_Fits_Headers_Set
 * @see liric_config.html#Liric_Config_Get_Boolean
 * @see liric_config.html#Liric_Config_Filter_Wheel_Is_Enabled
 * @see liric_general.html#LIRIC_GENERAL_IS_BOOLEAN
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Log_Format
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Flip_Set
 * @see ../detector/cdocs/detector_exposure.html#Detector_Exposure_Expose
 * @see ../detector/cdocs/detector_fits_filename.html#DETECTOR_FITS_FILENAME_PIPELINE_FLAG
 * @see ../detector/cdocs/detector_fits_filename.html#DETECTOR_FITS_FILENAME_EXPOSURE_TYPE
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Next_Multrun
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Next_Run
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Get_Filename
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_List_Add
 * @see ../filter_wheel/cdocs/filter_wheel_config.html#Filter_Wheel_Config_Name_To_Position
 * @see ../filter_wheel/cdocs/filter_wheel_command.html#ilter_Wheel_Command_Move
 */
int Liric_Bias_Dark_MultDark(int exposure_length_ms,int exposure_count,char ***filename_list,int *filename_count)
{
	char fits_filename[256];
	int flip_x,flip_y,mirror_filter_wheel_position;
	
	/* check arguments */
	if(exposure_length_ms < 1)
	{
		Liric_General_Error_Number = 700;
		sprintf(Liric_General_Error_String,
			"Liric_Bias_Dark_MultDark:exposure length was too short (%d).",exposure_length_ms);
		return FALSE;
	}
	if(exposure_count < 1)
	{
		Liric_General_Error_Number = 701;
		sprintf(Liric_General_Error_String,
			"Liric_Bias_Dark_MultDark:exposure count was too small (%d).",exposure_count);
		return FALSE;
	}
	if(filename_list == NULL)	
	{
		Liric_General_Error_Number = 702;
		sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultDark:filename_list was NULL.");
		return FALSE;
	}
	if(filename_count == NULL)	
	{
		Liric_General_Error_Number = 703;
		sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultDark:filename_count was NULL.");
		return FALSE;
	}
#if LIRIC_DEBUG > 1
	Liric_General_Log_Format("multdark","liric_bias_dark.c","Liric_Bias_Dark_MultDark",LOG_VERBOSITY_TERSE,
				  "MULTDARK","Started with exposure_length %d ms, exposure count %d.",
				  exposure_length_ms,exposure_count);
#endif
	/* initialise internal variables */
	Bias_Dark_In_Progress = TRUE;
	Moptop_Abort = FALSE;
	Bias_Dark_Data.Image_Count = exposure_count;
	(*filename_list) = NULL;
	(*filename_count) = 0;
	/* configure flipping of output image */
	if(!Liric_Config_Get_Boolean("liric.multrun.image.flip.x",&flip_x))
		return FALSE;		
	if(!Liric_Config_Get_Boolean("liric.multrun.image.flip.y",&flip_y))
		return FALSE;
	Detector_Exposure_Flip_Set(flip_x,flip_y);
	/* move filter wheel to mirror position */
	if(Liric_Config_Filter_Wheel_Is_Enabled())
	{
		/* which filter position contains the AuMirror filter */
		if(!Filter_Wheel_Config_Name_To_Position("AuMirror",&mirror_filter_wheel_position))
		{
			Liric_General_Error_Number = 722;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultDark:Failed to find AuMirror filter wheel position.");
			return FALSE;
		}
		/* move filter wheel */
		if(!Filter_Wheel_Command_Move(mirror_filter_wheel_position))
		{
			Liric_General_Error_Number = 723;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultDark:Failed to move filter wheel to AuMirror position %d.",
				mirror_filter_wheel_position);
			return FALSE;
		}
	}
	/* intialise FITS filenames for new multrun*/
	if(!Detector_Fits_Filename_Next_Multrun())
	{
		Bias_Dark_In_Progress = FALSE;
		Liric_General_Error_Number = 704;
		sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultDark:Failed to initialise FITS filename multrun.");
		return FALSE;
	}
	/* do any per-multdark FITS header changes here */
	if(!Bias_Dark_Fits_Headers_Set(FALSE,exposure_count))
	{
		Bias_Dark_In_Progress = FALSE;
		return FALSE;
	}
	/* take a multrun start timestamp */
	clock_gettime(CLOCK_REALTIME,&(Bias_Dark_Data.Bias_Dark_Start_Time));
	/* start multdark for loop */
	for(Bias_Dark_Data.Image_Index = 0; Bias_Dark_Data.Image_Index < Bias_Dark_Data.Image_Count;
	    Bias_Dark_Data.Image_Index++)
	{
		/* check for aborts */
		if(Moptop_Abort)
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 705;
			sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultDark:Aborted.");
			return FALSE;
		}
		/* generate new FITS image filename */
		if(!Detector_Fits_Filename_Next_Run())
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 706;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultDark:Failed to generate next FITS filename run number.");
			return FALSE;
		}
		if(!Detector_Fits_Filename_Get_Filename(DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_DARK,
							DETECTOR_FITS_FILENAME_PIPELINE_FLAG_UNREDUCED,
							fits_filename,256))
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 707;
			sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultDark:Failed to generate next FITS filename.");
			return FALSE;
		}
		/* check for aborts */
		if(Moptop_Abort)
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 708;
			sprintf(Liric_General_Error_String,"Liric_Bias_Dark_MultDark:Aborted.");
			return FALSE;
		}
		/* do any per-multdark frame FITS header changes here */
		if(!Bias_Dark_Exposure_Fits_Headers_Set())
		{
			Bias_Dark_In_Progress = FALSE;
			return FALSE;
		}
		/* take an exposure */
		if(!Detector_Exposure_Expose(exposure_length_ms,fits_filename))
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 709;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultDark:Failed to take exposure %d of %d ms with filename '%s'.",
				Bias_Dark_Data.Image_Index,exposure_length_ms,fits_filename);
			return FALSE;
		}
		/* add fits image to list */
		if(!Detector_Fits_Filename_List_Add(fits_filename,filename_list,filename_count))
		{
			Bias_Dark_In_Progress = FALSE;
			Liric_General_Error_Number = 724;
			sprintf(Liric_General_Error_String,
				"Liric_Bias_Dark_MultDark:Failed to add filename '%s' to list of length %d.",
				fits_filename,(*filename_count));
			return FALSE;
		}
	}/* end for on Bias_Dark_Data.Image_Index */
	/* we have finished the multdark */
	Bias_Dark_In_Progress = FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("multdark","liric_bias_dark.c","Liric_Bias_Dark_MultDark",LOG_VERBOSITY_TERSE,"MULTDARK",
			   "Finished.");
#endif		
	return TRUE;
}

/**
 * Routine to abort an in-progress bias/dark.
 * @return The routine nominally returns TRUE on success and FALSE on failure. 
 *         However, it currently always succeeds (returns TRUE).
 * @see #Moptop_Abort
 */
int Liric_Bias_Dark_Abort(void)
{
	Moptop_Abort = TRUE;
	return TRUE;
}

/**
 * Return whether we are currently performing a multbias/multdark or not.
 * @return A boolean, TRUE if a multbias/multdark is in progress and FALSE if it is not in progress..
 * @see #Bias_Dark_In_Progress
 */
int Liric_Bias_Dark_In_Progress(void)
{
	return Bias_Dark_In_Progress;
}

/**
 * Return the total number of exposures expected to be generated in the current/last multbias/multdark.
 * @return The number of images/frames expected.
 * @see #Bias_Dark_Data
 */
int Liric_Bias_Dark_Count_Get(void)
{
	return Bias_Dark_Data.Image_Count;
}

/**
 * Return which exposure in the multbias/multdark we are on.
 * @return The exposure index in the multbias/multdark.
 * @see #Bias_Dark_Data
 */
int Liric_Bias_Dark_Exposure_Index_Get(void)
{
	return Bias_Dark_Data.Image_Index;
}

/**
 * Routine to collect and insert FITS headers pertaining to the whole bias/dark multrun.
 * @param is_bias A boolean, set to TRUE for biases and FALSE for darks. Used to set the OBSTYPE FITS header.
 * @param exposure_count An integer, the number of individual exposures/darks in the bias/dark multrun.
 * @see #CENTIGRADE_TO_KELVIN
 * @see #Bias_Dark_Data
 * @see liric_config.html#Liric_Config_Filter_Wheel_Is_Enabled
 * @see liric_fits_header.html#Liric_Fits_Header_Integer_Add
 * @see liric_fits_header.html#Liric_Fits_Header_Logical_Add
 * @see liric_fits_header.html#Liric_Fits_Header_String_Add
 * @see liric_general.html#LIRIC_GENERAL_IS_BOOLEAN
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Log_Format
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Multrun_Get
 * @see ../detector/cdocs/detector_setup.html#Detector_Setup_Get_Sensor_Size_X
 * @see ../detector/cdocs/detector_setup.html#Detector_Setup_Get_Sensor_Size_Y
 * @see ../filter_wheel/cdocs/filter_wheel_command.html#Filter_Wheel_Command_Get_Position
 * @see ../filter_wheel/cdocs/filter_wheel_config.html#Filter_Wheel_Config_Position_To_Name
 * @see ../filter_wheel/cdocs/filter_wheel_config.html#Filter_Wheel_Config_Position_To_Id
 */
static int Bias_Dark_Fits_Headers_Set(int is_bias,int exposure_count)
{
	char filter_name_string[32];
	double temperature;
	int filter_wheel_position,retval;

#if LIRIC_DEBUG > 1
	Liric_General_Log("biasdark","liric_bias_dark.c","Bias_Dark_Fits_Headers_Set",LOG_VERBOSITY_TERSE,"BIASDARK",
			   "Bias_Dark_Fits_Headers_Set started.");
#endif
	if(exposure_count < 1)
	{
		Liric_General_Error_Number = 725;
		sprintf(Liric_General_Error_String,
			"Bias_Dark_Fits_Headers_Set:exposure count was too small (%d).",exposure_count);
		return FALSE;
	}
	if(!LIRIC_GENERAL_IS_BOOLEAN(is_bias))
	{
		Liric_General_Error_Number = 726;
		sprintf(Liric_General_Error_String,
			"Bias_Dark_Fits_Headers_Set:is_bias was not a valid boolean (%d).",is_bias);
		return FALSE;
	}
	/* OBSTYPE */
	if(is_bias)
		retval = Liric_Fits_Header_String_Add("OBSTYPE","BIAS",NULL);
	else
		retval = Liric_Fits_Header_String_Add("OBSTYPE","DARK",NULL);
	if(retval == FALSE)
		return FALSE;
	/* filter wheel position keywords */
	if(Liric_Config_Filter_Wheel_Is_Enabled())
	{
		if(!Filter_Wheel_Command_Get_Position(&filter_wheel_position))
		{
			Liric_General_Error_Number = 727;
			sprintf(Liric_General_Error_String,"Bias_Dark_Fits_Headers_Set:"
					"Failed to get filter wheel position.");
			return FALSE;
			
		}
		/* FILTER1 */
		if(!Filter_Wheel_Config_Position_To_Name(filter_wheel_position,filter_name_string))
		{
			Liric_General_Error_Number = 731;
			sprintf(Liric_General_Error_String,"Bias_Dark_Fits_Headers_Set:"
				"Failed to get filter wheel name from position %d.",filter_wheel_position);
			return FALSE;
		}
		if(!Liric_Fits_Header_String_Add("FILTER1",filter_name_string,NULL))
			return FALSE;
		/* FILTERI1 */
		if(!Filter_Wheel_Config_Position_To_Id(filter_wheel_position,filter_name_string))
		{
			Liric_General_Error_Number = 728;
			sprintf(Liric_General_Error_String,"Bias_Dark_Fits_Headers_Set:"
				"Failed to get filter wheel Id from position %d.",filter_wheel_position);
			return FALSE;
		}
		if(!Liric_Fits_Header_String_Add("FILTERI1",filter_name_string,NULL))
			return FALSE;
	}
	else
	{
		/* FILTER1 */
		if(!Liric_Fits_Header_String_Add("FILTER1","UNKNOWN",NULL))
			return FALSE;
		/* FILTERI1 */
		if(!Liric_Fits_Header_String_Add("FILTERI1","UNKNOWN",NULL))
			return FALSE;
	}
	/* RUNNUM */
	if(!Liric_Fits_Header_Integer_Add("RUNNUM",Detector_Fits_Filename_Multrun_Get(),"Number of Multrun"))
		return FALSE;
	/* EXPTOTAL */
	if(!Liric_Fits_Header_Integer_Add("EXPTOTAL",Bias_Dark_Data.Image_Count,
					   "Total number of exposures within Multrun"))
		return FALSE;
	/* CONFIGID diddly TODO in Java layer */
	/* CONFNAME diddly TODO in Java layer */
	/* CCDSTEMP */
	if(!Detector_Temperature_Get_TEC_Setpoint(&temperature))
	{
		Liric_General_Error_Number = 729;
		sprintf(Liric_General_Error_String,"Bias_Dark_Fits_Headers_Set:Failed to get TEC set-point.");
		return FALSE;
	}
	if(!Liric_Fits_Header_Float_Add("CCDSTEMP",temperature+CENTIGRADE_TO_KELVIN,"[Kelvin] Required temperature."))
		return FALSE;	
	/* CCDATEMP */
	if(!Detector_Temperature_Get(&(Bias_Dark_Data.CCD_Temperature)))
	{
		Liric_General_Error_Number = 730;
		sprintf(Liric_General_Error_String,"Bias_Dark_Fits_Headers_Set:Failed to get detector temperature.");
		return FALSE;
	}
	if(!Liric_Fits_Header_Float_Add("CCDATEMP",Bias_Dark_Data.CCD_Temperature+CENTIGRADE_TO_KELVIN,
					 "[Kelvin] Actual temperature."))
		return FALSE;	
	/* DETECTOR diddly TODO in Java layer */
	/* CCDXBIN */
	if(!Liric_Fits_Header_Integer_Add("CCDXBIN",1,"X binning factor"))
		return FALSE;
	/* CCDYBIN */
	if(!Liric_Fits_Header_Integer_Add("CCDYBIN",1,"Y binning factor"))
		return FALSE;
	/* CCDWMODE */
	if(!Liric_Fits_Header_Logical_Add("CCDWMODE",FALSE,"Using a Window (always false for Liric)"))
		return FALSE;
	/* CCDXIMSI */
	if(!Liric_Fits_Header_Integer_Add("CCDXIMSI",Detector_Setup_Get_Sensor_Size_X(),"[pixels] X image size"))
		return FALSE;
	/* CCDYIMSI */
	if(!Liric_Fits_Header_Integer_Add("CCDYIMSI",Detector_Setup_Get_Sensor_Size_Y(),"[pixels] Y image size"))
		return FALSE;
	/* CCDWXOFF */
	if(!Liric_Fits_Header_Integer_Add("CCDWXOFF",0,"[pixels] X window offset"))
		return FALSE;
	/* CCDWYOFF */
	if(!Liric_Fits_Header_Integer_Add("CCDWYOFF",0,"[pixels] Y window offset"))
		return FALSE;
	/* CCDWXSIZ */
	if(!Liric_Fits_Header_Integer_Add("CCDWXSIZ",Detector_Setup_Get_Sensor_Size_X(),"[pixels] X window size"))
		return FALSE;
	/* CCDWYSIZ */
	if(!Liric_Fits_Header_Integer_Add("CCDWYSIZ",Detector_Setup_Get_Sensor_Size_Y(),"[pixels] Y window size"))
		return FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("biasdark","liric_bias_dark.c","Bias_Dark_Fits_Headers_Set",LOG_VERBOSITY_TERSE,"BIASDARK",
			   "Bias_Dark_Fits_Headers_Set finished.");
#endif		
	return TRUE;
}

/**
 * Routine to collect and insert FITS headers pertaining to the current exposure in the multrun.
 * @return The routine returns TRUE on sucess and FALSE on failure. On failure, Liric_General_Error_Number and
 *         Liric_General_Error_String should be set.
 * @see liric_fits_header.html#Liric_Fits_Header_Integer_Add
 * @see liric_fits_header.html#Liric_Fits_Header_String_Add
 * @see liric_general.html#Liric_General_Error_Number
 * @see liric_general.html#Liric_General_Error_String
 * @see liric_general.html#Liric_General_Log
 * @see liric_general.html#Liric_General_Log_Format
 * @see ../detector/cdocs/detector_fits_filename.html#Detector_Fits_Filename_Run_Get
 */
static int Bias_Dark_Exposure_Fits_Headers_Set(void)
{
#if LIRIC_DEBUG > 1
	Liric_General_Log("biasdark","liric_bias_dark.c","Bias_Dark_Exposure_Fits_Headers_Set",LOG_VERBOSITY_TERSE,"BIASDARK",
			   "Bias_Dark_Exposure_Fits_Headers_Set started.");
#endif
	/* EXPNUM */
	if(!Liric_Fits_Header_Integer_Add("EXPNUM",Detector_Fits_Filename_Run_Get(),
					   "Number of exposure within MultBias/MultDark"))
		return FALSE;
	/* NUDGEPOS */
	if(!Liric_Fits_Header_Integer_Add("NUDGEPOS",-1,"Nudgematic position unknown"))
		return FALSE;
	/* NUDGEOFF */
	if(!Liric_Fits_Header_String_Add("NUDGEOFF","UNKNOWN",NULL))
		return FALSE;
#if LIRIC_DEBUG > 1
	Liric_General_Log("biasdark","liric_bias_dark.c","Bias_Dark_Exposure_Fits_Headers_Set",LOG_VERBOSITY_TERSE,"BIASDARK",
			   "Bias_Dark_Exposure_Fits_Headers_Set finished.");
#endif		
	return TRUE;
}

