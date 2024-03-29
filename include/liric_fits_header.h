/* liric_fits_header.h */
#ifndef LIRIC_FITS_HEADER_H
#define LIRIC_FITS_HEADER_H

/* external functions */
extern int Liric_Fits_Header_Initialise(void);
extern int Liric_Fits_Header_String_Add(char *keyword,char *value, char *comment);
extern int Liric_Fits_Header_Integer_Add(char *keyword,int value, char *comment);
extern int Liric_Fits_Header_Long_Long_Integer_Add(char *keyword,long long int value, char *comment);
extern int Liric_Fits_Header_Float_Add(char *keyword,double value, char *comment);
extern int Liric_Fits_Header_Logical_Add(char *keyword,int value, char *comment);
extern int Liric_Fits_Header_Add_Comment(char *keyword,char *comment);
extern int Liric_Fits_Header_Add_Units(char *keyword,char *units);
extern int Liric_Fits_Header_Delete(char *keyword);
extern int Liric_Fits_Header_Clear(void);
extern void Liric_Fits_Header_TimeSpec_To_Date_String(struct timespec time,char *time_string);
extern void Liric_Fits_Header_TimeSpec_To_Date_Obs_String(struct timespec time,char *time_string);
extern void Liric_Fits_Header_TimeSpec_To_UtStart_String(struct timespec time,char *time_string);
extern int Liric_Fits_Header_TimeSpec_To_Mjd(struct timespec time,int leap_second_correction,double *mjd);

#endif
