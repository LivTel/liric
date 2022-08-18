/* detector_fits_filename.h */

#ifndef DETECTOR_FITS_FILENAME_H
#define DETECTOR_FITS_FILENAME_H

/**
 * Enum defining types of exposure to put in the exposure code part of a LT FITS filename.
 * <ul>
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_ARC
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_BIAS
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_DARK
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_EXPOSURE
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_SKYFLAT
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_STANDARD
 * <li>DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_LAMPFLAT
 * </ul>
 */
enum DETECTOR_FITS_FILENAME_EXPOSURE_TYPE
{
	DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_ARC=0,DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_BIAS,
	DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_DARK,DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_EXPOSURE,
	DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_SKYFLAT,DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_STANDARD,
	DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_LAMPFLAT
};

/**
 * Macro to check whether the parameter is a valid exposure type.
 * @see #DETECTOR_FITS_FILENAME_EXPOSURE_TYPE
 */
#define DETECTOR_FITS_FILENAME_IS_EXPOSURE_TYPE(value)	(((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_ARC)|| \
							 ((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_BIAS)|| \
							 ((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_DARK)|| \
							 ((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_EXPOSURE)|| \
							 ((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_SKYFLAT)|| \
							 ((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_STANDARD)|| \
							 ((value) == DETECTOR_FITS_FILENAME_EXPOSURE_TYPE_LAMPFLAT))
/**
 * Enum defining the pipeline processing flag to put in the pipeline flag part of a LT FITS filename.
 * <ul>
 * <li>DETECTOR_FITS_FILENAME_PIPELINE_FLAG_UNREDUCED
 * <li>DETECTOR_FITS_FILENAME_PIPELINE_FLAG_REALTIME
 * <li>DETECTOR_FITS_FILENAME_PIPELINE_FLAG_OFFLINE
 * </ul>
 */
enum DETECTOR_FITS_FILENAME_PIPELINE_FLAG
{
	DETECTOR_FITS_FILENAME_PIPELINE_FLAG_UNREDUCED=0,
	DETECTOR_FITS_FILENAME_PIPELINE_FLAG_REALTIME=1,
	DETECTOR_FITS_FILENAME_PIPELINE_FLAG_OFFLINE=2
};

/**
 * Macro to check whether the parameter is a valid pipeline flag.
 * @see #DETECTOR_FITS_FILENAME_PIPELINE_FLAG
 */
#define DETECTOR_FITS_FILENAME_IS_PIPELINE_FLAG(value)	(((value) == DETECTOR_FITS_FILENAME_PIPELINE_FLAG_UNREDUCED)|| \
							 ((value) == DETECTOR_FITS_FILENAME_PIPELINE_FLAG_REALTIME)|| \
							 ((value) == DETECTOR_FITS_FILENAME_PIPELINE_FLAG_OFFLINE))

/**
 * Default instrument code, used as first character as LT FITS filename for Raptor.
 */
#define DETECTOR_FITS_FILENAME_DEFAULT_INSTRUMENT_CODE ('j')

extern int Detector_Fits_Filename_Initialise(char instrument_code,const char *data_dir);
extern int Detector_Fits_Filename_Next_Multrun(void);
extern int Detector_Fits_Filename_Next_Run(void);
extern int Detector_Fits_Filename_Next_Window(void);
extern int Detector_Fits_Filename_Get_Filename(enum DETECTOR_FITS_FILENAME_EXPOSURE_TYPE type,
					       enum DETECTOR_FITS_FILENAME_PIPELINE_FLAG pipeline_flag,
					       char *filename,int filename_length);
extern int Detector_Fits_Filename_List_Add(char *filename,char ***filename_list,int *filename_count);
extern int Detector_Fits_Filename_List_Free(char ***filename_list,int *filename_count);
extern int Detector_Fits_Filename_Multrun_Get(void);
extern int Detector_Fits_Filename_Run_Get(void);
extern int Detector_Fits_Filename_Window_Get(void);
extern int Detector_Fits_Filename_Lock(char *filename);
extern int Detector_Fits_Filename_UnLock(char *filename);
extern int Detector_Fits_Filename_Get_Error_Number(void);
extern void Detector_Fits_Filename_Error(void);
extern void Detector_Fits_Filename_Error_String(char *error_string);

#endif
