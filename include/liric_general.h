/* liric_general.h
*/
#ifndef LIRIC_GENERAL_H
#define LIRIC_GENERAL_H

#include <pthread.h>

/* hash defines */
/**
 * TRUE is the value usually returned from routines to indicate success.
 */
#ifndef TRUE
#define TRUE 1
#endif
/**
 * FALSE is the value usually returned from routines to indicate failure.
 */
#ifndef FALSE
#define FALSE 0
#endif

/**
 * Macro to check whether the parameter is either TRUE or FALSE.
 */
#define LIRIC_GENERAL_IS_BOOLEAN(value)	(((value) == TRUE)||((value) == FALSE))

#ifndef MAX
/**
 * Return maximum of two values passed in.
 */
#define MAX(A,B) ((A)>(B)?(A):(B))
#endif
#ifndef MIN
/**
 * Return minimum of two values passed in.
 */
#define MIN(A,B) ((A)<(B)?(A):(B))
#endif

/**
 * This is the length of error string of modules in the library.
 */
#define LIRIC_GENERAL_ERROR_STRING_LENGTH	(1024)

/**
 * The number of nanoseconds in one second. A struct timespec has fields in nanoseconds.
 */
#define LIRIC_GENERAL_ONE_SECOND_NS	        (1000000000)
/**
 * The number of nanoseconds in one millisecond. A struct timespec has fields in nanoseconds.
 */
#define LIRIC_GENERAL_ONE_MILLISECOND_NS	(1000000)
/**
 * The number of milliseconds in one second.
 */
#define LIRIC_GENERAL_ONE_SECOND_MS	        (1000)
/**
 * The number of nanoseconds in one microsecond.
 */
#define LIRIC_GENERAL_ONE_MICROSECOND_NS	(1000)

#ifndef fdifftime
/**
 * Return double difference (in seconds) between two struct timespec's.
 * @param t0 A struct timespec.
 * @param t1 A struct timespec.
 * @return A double, in seconds, representing the time elapsed from t0 to t1.
 * @see #LIRIC_GENERAL_ONE_SECOND_NS
 */
#define fdifftime(t1, t0) (((double)(((t1).tv_sec)-((t0).tv_sec))+(double)(((t1).tv_nsec)-((t0).tv_nsec))/LIRIC_GENERAL_ONE_SECOND_NS))
#endif

/* external variabless */
extern int Liric_General_Error_Number;
extern char Liric_General_Error_String[];

/* external functions */
extern void Liric_General_Error(char *sub_system,char *source_filename,char *function,int level,char *category);
extern void Liric_General_Error_To_String(char *sub_system,char *source_filename,char *function,int level,
					       char *category,char *error_string);

/* routine used by other modules error code */
extern void Liric_General_Get_Time_String(struct timespec timestamp,char *time_string,int string_length);
extern void Liric_General_Get_Current_Time_String(char *time_string,int string_length);

/* logging routines */
extern void Liric_General_Log_Format(char *sub_system,char *source_filename,char *function,int level,
					  char *category,char *format,...);
extern void Liric_General_Log(char *sub_system,char *source_filename,char *function,int level,char *category,
				   char *string);
extern void Liric_General_Call_Log_Handlers(char *sub_system,char *source_filename,char *function,int level,
					  char *category,char *message);
extern void Liric_General_Call_Log_Handlers_Detector(int level,char *message);
extern void Liric_General_Call_Log_Handlers_Filter_Wheel(int level,char *message);
extern void Liric_General_Call_Log_Handlers_Nudgematic(int level,char *message);
extern int Liric_General_Add_Log_Handler_Function(void (*log_fn)(char *sub_system,char *source_filename,
							char *function,int level,char *category,char *message));
extern void Liric_General_Set_Log_Filter_Function(int (*filter_fn)(char *sub_system,char *source_filename,
						       char *function,int level,char *category,char *message));
extern int Liric_General_Log_Set_Directory(char *directory);
extern int Liric_General_Log_Set_Root(char *filename_root);
extern int Liric_General_Log_Set_Error_Root(char *filename_root);
extern int Liric_General_Log_Set_UDP(int active,char *hostname,int port_number);
extern void Liric_General_Log_Handler_Stdout(char *sub_system,char *source_filename,char *function,int level,
						  char *category,char *message);
extern void Liric_General_Log_Handler_Log_Fp(char *sub_system,char *source_filename,char *function,int level,
						  char *category,char *message);
extern void Liric_General_Log_Handler_Log_Hourly_File(char *sub_system,char *source_filename,char *function,
							   int level,char *category,char *message);
extern void Liric_General_Log_Handler_Log_UDP(char *sub_system,char *source_filename,char *function,
						   int level,char *category,char *message);
extern void Liric_General_Set_Log_Filter_Level(int level);
extern int Liric_General_Log_Filter_Level_Absolute(char *sub_system,char *source_filename,char *function,
							int level,char *category,char *message);
extern int Liric_General_Log_Filter_Level_Bitwise(char *sub_system,char *source_filename,char *function,
						   int level,char *category,char *message);

/* utility routines */
extern int Liric_General_Add_String(char **string,char *add);
extern int Liric_General_Add_Integer_To_String(char **string,int i);
extern int Liric_General_Int_List_Add(int add,int **list,int *count);
extern int Liric_General_Int_List_Sort(const void *f,const void *s);
extern int Liric_General_Mutex_Lock(pthread_mutex_t *mutex);
extern int Liric_General_Mutex_Unlock(pthread_mutex_t *mutex);
extern int Liric_General_Thread_Priority_Set_Normal(void);
extern int Liric_General_Thread_Priority_Set_Exposure(void);
extern int Liric_General_Thread_Priority_Set(int priority);

extern int Liric_General_Set_Config_Filename(char *filename);
extern char *Liric_General_Get_Config_Filename(void);

#endif
