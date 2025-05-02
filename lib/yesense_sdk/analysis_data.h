#ifndef ANALYSIS_DATA_H
#define ANALYSIS_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE			(unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		(unsigned char)0x53
#define YIS_OUTPUT_MIN_BYTES		7

/*------------------------------------------------Type define--------------------------------------------------*/
typedef enum
{
	crc_err 		= -3,
	data_len_err 	= -2,
	para_err 		= -1,
	analysis_ok 	= 0,
	analysis_done 	= 1
}analysis_res_t;

#pragma pack(1)

typedef struct
{
	unsigned char 	header1;	/*0x59*/
	unsigned char 	header2;	/*0x53*/
	unsigned short 	tid;		/*1 -- 60000*/
	unsigned char 	len;		/*length of payload, 0 -- 255*/
}output_data_header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}payload_data_t;

/*-------------------------------------*/
typedef struct
{
	float x;
	float y;
	float z;
}axis_data_t;

typedef struct
{
	float pitch;						/*unit: ° (deg)*/
	float roll;
	float yaw;

	float quaternion_data0;
	float quaternion_data1;
	float quaternion_data2;
	float quaternion_data3;
}attitude_data_t;

typedef struct
{
	double latitude;					/*unit: deg*/
	double longtidue;					/*unit: deg*/
	float  altidue;						/*unit: m*/
}location_t;

typedef struct
{
	unsigned int	msecond;
	unsigned short 	year;
	unsigned char 	month;
	unsigned char 	day;
	unsigned char 	hour;
	unsigned char 	minute;
	unsigned char	second;
}utc_time_t;

typedef struct
{
	float vel_n;						/*unit: m/s */
	float vel_e;
	float vel_d;
}velocity_t;

typedef struct
{
	unsigned char fusion:4;
	unsigned char gnss:4;
}nav_status_t;

/*-------------------------------------*/
typedef struct
{
	float 			sensor_temp;	/*unit: ℃*/
	axis_data_t 	accel;			/*unit: m/s2*/
	axis_data_t 	angle_rate;		/*unit: ° (deg)/s*/
	axis_data_t 	norm_mag;		/*unit: 归一化值*/
	axis_data_t 	raw_mag;		/*unit: mGauss*/

	attitude_data_t attitude;

	location_t 		location;
	velocity_t		vel;
	utc_time_t 		utc;
	nav_status_t	status;

	unsigned int 	sample_timestamp;		/*unit: us*/
	unsigned int 	data_ready_timestamp;	/*unit: us*/
}protocol_info_t;

#pragma pack()

/*------------------------------------------------------------------------------------------------------------*/

/*------------------------------------------------Functions declare--------------------------------------------*/
int analysis_data(unsigned char *data, short len, protocol_info_t *info);

#ifdef __cplusplus
}
#endif

#endif
