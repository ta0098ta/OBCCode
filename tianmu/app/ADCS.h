#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/*=============================================================================
	程序名: ADCS.c <NS-2(MEMSst)ADCS星上软件头文件> 
	编 辑	: 师帅
	日 期 : 20150715
=============================================================================*/
/////////////////
// 间接遥控指令
#define	ADDR_CMD_ADCS_ONOFF			0x00
#define	ADDR_CMD_CFG_UPDATE			0x01
#define	ADDR_CMD_OCO_UPDATE			0x02
#define	ADDR_CMD_LOG_ONOFF			0x03
#define	ADDR_CMD_WORK_MODE			0x04
#define	ADDR_CMD_TLM_SOURCE			0x05
//#define	ADDR_CMD_TLM_CAN_ONOFF	0x06    暂时保留0x06
#define	ADDR_CMD_FILTER					0x07
#define	ADDR_CMD_CONTROL				0x08
#define	ADDR_CMD_PERIOD_NOM			0x09
#define	ADDR_CMD_PITCH_REF			0x0A
#define	ADDR_CMD_Adc_G					0x0B
#define	ADDR_CMD_Adc_G1					0x0C
#define	ADDR_CMD_Adc_G2					0x0D
#define	ADDR_CMD_Adc_wy					0x0E
#define	ADDR_CMD_Adc_Ks1				0x0F
#define	ADDR_CMD_Adc_Kd1				0x10
#define	ADDR_CMD_Adc_Kd_0				0x11
#define	ADDR_CMD_Adc_Kd_1				0x12
#define	ADDR_CMD_Adc_Kd_2				0x13
#define	ADDR_CMD_Adc_Kp_0				0x14
#define	ADDR_CMD_Adc_Kp_1				0x15
#define ADDR_CMD_Adc_PitchMask  0x16
#define ADDR_CMD_Adc_TStep      0x17
#define ADDR_CMD_MWY_ON_OFF     0x18
#define	ADDR_CMD_MWy_RPM_EMERGENT	   0x19
#define	ADDR_CMD_MWY_RPM_RATING			 0x1A  
//#define ADDR_CMD_MWY_CONTROL_MODE    0x1B  暂时保留
//#define ADDR_CMD_MWY_PERIOD        	 0x1C
#define ADDR_CMD_Adc_Kpw            0x1D
#define ADDR_CMD_Adc_Kdw            0x1E
//#define ADDR_CMD_Adc_W_step          0x1F
#define ADDR_CMD_MW_RPM_Max          0x1F
#define	ADDR_CMD_MWy_COEF_K1				 0x20
//#define	ADDR_CMD_MWy_COEF_K2			   0x21
#define	ADDR_CMD_MT_CONTROL_MODE			0x22
#define	ADDR_CMD_MT_PWMX_EMERGENT	0x23
#define	ADDR_CMD_MT_PWMY_EMERGENT	0x24
#define	ADDR_CMD_MT_PWMZ_EMERGENT	0x25
//#define	ADDR_CMD_MT_I_SAMPLE			0x26     暂时保留
#define	ADDR_CMD_MM_CORRECT_X			0x27
#define	ADDR_CMD_MM_CORRECT_Y			0x28
#define	ADDR_CMD_MM_CORRECT_Z			0x29


#define ADCS_REALTIME_DATA      0x201A58FA      //ADCS数据起始地址      50K

#define	NO_FILTER		0			/*	no estimator					*/
#define	RATE_FILTER		1			/*	rate filter						*/
#define	PITCH_FILTER	2			/*	pitch filter					*/
#define	HYBRID_FILTER	3			/*	rate and pitch filter			*/
#define	ATACQ_FILTER	4			/*	oak-like and pitch filter		*/
#define	OAK_FILTER		5			/*	oak-like filter	 (navmag)		*/
#define	Sun_Filter		6			/*	SunSensor+MM filter		*/
#define	SS_Filter		7


#define	C1				1			/*	de-libration only				*/
#define	C2				2			/*	de-libration and y-spin	control	*/
#define	C3				3			/*	de-libration and y-spin	control	*/
#define	C4				4			/*	pitch-wheel	spun-up	control		*/
#define	C5				5			/*	pitch-wheel	nominal	control		*/

#define	ON				1			/*	flag on							*/
#define	OFF				0			/*	flag off						*/
#define	TwoPI			6.28318530717959

#define	ZERO			1.e-24		/*	zero							*/
#define	MICRO			1.e-6		/*	unit conversion	<micro>			*/

#define	TURNON			2			/*	Momentum-Wheel trun on			*/
#define	BREAKING		3			/*	Momentum-Wheel breaking mode	*/
#define	TURNOFF			4			/*	Momentum-Wheel trun off			*/

#define	NMAX			8			/*	igrf harmonic order(old	is 8)	*/
#define AE				6.3712e6	/*  igrf defined mean earth radius  */

/* Added by	LXF	*/
double LXF_sun[3];				    /* solar vecter in orbital coordinate */
int    LXF_shade;					/*eclipse determinor*/
double Sun_i[3];
#define	PI				3.14159265358979
#define GPS_ZERO_MJD	44244.0		/* modified Julian dates of Greewich Mean Time zero hour UTC for Jan/6th/1980 */
/* End of added	by LXF */


#define A0  1.753368559233265e+000
#define A1  1.720279180530708e-002
#define A2  5.075209994113592e-015
#define A3 -9.253097568194335e-024

#define B0  6.300388098984891e+000
#define B1  1.015046221041584e-014
#define B2 -2.778762509236278e-023

/* quasi-function micros used by orbit block and geomagnetic block */
#define telaps(mjd)		(double)((mjd)-51544.5)
#define rotn(t,dt,e)	(fmod(A0+(t)*(A1+(t)*(A2+(t)*A3)),TwoPI) + (B0+(t)*(B1+(t)*B2))*(dt) + (e))

#define	min(a,b)	(((a) <	(b)) ? (a) : (b))

double  Data_Alpha;
double  Data_Beta;


INT8U	ADCS_Err;


int	ADCS_OnOff;             //00H
short	CMD_CFG_UPDATE;			//01H   /* configure update */
short	CMD_OCO_UPDATE;			//02H   /* orbit coefficient update */
short	CMD_LOG_ONOFF;			//03H   /* 数据包请求	*/
short	WorkMode;			    //04H   /* the work mode of ADCS*/
short	TlmSource;			    //05H   /* data source of TLM(MCU1 or MCU2)*/
short	CMD_MM_SEL;		        //06H
short	TlmCANOnOff;            //07H
short	CMD_FILTER;             //08H
short	CMD_CONTROL;            //09H
short	CMD_PERIOD_NOM;         //0AH
short	CMD_PITCH_REF;          //0BH
float	CMD_Adc_G;              //0CH
float	CMD_Adc_G1;             //0DH
float	CMD_Adc_G2;             //0EH
float	CMD_Adc_wy;             //0FH

float	CMD_Adc_Ks1;            //10H
float	CMD_Adc_Kd1;            //11H
float	CMD_Adc_Kd_0;           //12H
float	CMD_Adc_Kd_1;           //13H
float	CMD_Adc_Kd_2;           //14H
float	CMD_Adc_Kp_0;           //15H
float	CMD_Adc_Kp_1;           //16H
float	CMD_Adc_PitchMask;       //17H
float  	CMD_Adc_TStep;           //18H

short   	CMD_MWY_ON_OFF;         //1AH

short	CMD_MWY_RPM_RATING;     //20H

short   CMD_MWY_CONTROL_MODE;    //23H

short	CMD_MWY_PERIOD;         //26H


short   CMD_Adc_Kpw;           //29H

short   CMD_Adc_Kdw;           //2CH

short   CMD_Adc_W_step;         //2EH


short tcMW_kpY;               //30H

char   CMD_MT_CONTROL_MODE;		//35H
char	CMD_MTX_TIME;		//36H
char	CMD_MTY_TIME;		//37H
char	CMD_MTZ_TIME;		//38H
char	CMD_MTX_PWM;		//39H
char	CMD_MTY_PWM;		//3AH
char	CMD_MTZ_PWM;		//3BH
short	MMCorrectX;       //3CH/* correct value of MMx(MM1 or MM2)*/
short	MMCorrectY;       //3DH/* correct value of MMy(MM1 or MM2)*/
short	MMCorrectZ;       //3EH/* correct value of MMz(MM1 or MM2)*/
char	CMD_MMS_ERASE;	//3FH

char	CMD_MCU1;		//4AH
char	CMD_MCU2;		//4BH


#if 1
short	CMD_MWX_RPM_MAX;        //28H
short	CMD_MWY_RPM_MAX;        //29H
short	CMD_MWZ_RPM_MAX;        //2AH
short tcMCU2MTPWMx;           //5AH
short tcMCU2MTPWMy;           //5BH
short tcMCU2MTPWMz;           //5CH
short	CMD_MT_I_SAMPLE;        //5DH
#endif

short	LogOpen;				/* switch of log file*/

float	R_ECEF[3],V_ECI[3];


int		ZKC_Counter;
INT8U	data_got[20];


unsigned long	gps_weeks_ADCS;
unsigned long	gps_seconds_ADCS;

/* modified GPS time weeks and its   seconds, 'modified' means leaping-  second modified GPS time. this   work is accomplished by OBC	*/

int		seconds80;
		/* time in seconds from 1980		 */
int		seconds80_Prev;
		/* Previous time in seconds from 1980*/


short	Control;					/* Attitude	controller selection	*/
short	Filter;					/* estimation filter selection		*/

int		Pitch_Ref;			/* Pitch reference angle: 0.01 Deg	*/

double	MT_Cmd[3];			/*	magnetorquer command in	Am2		*/

short	MW_RPMCmd16;				/*	MW speed Cmd in RPM (Round Per Minute)	*/
short	MT_PWMCmd16[3];				/*  MT commands for ADCS	CAN	node*/
short	MW_RPMTlm16;				/*  MW speed in RPM 				*/
double	MM_Tlm16[3];				/*  B-field TLM value from magnetometer	*/
double	MM_Tlm16Prev[3];			/*	previous magnav	telemetry		*/
double  Q_oi[4];
//short	MM_Sel;						/*  MM selection					*/

double	MT_Xmax;					/* MT x saturation limit			*/
double	MT_Ymax;					/* MT y saturation limit			*/
double	MT_Zmax;					/* MT z saturation limit			*/
double	BfMM_Max;					/* max value in microT for B mesured*/
double	BfMM_Min;					/* min value in microT for B mesured*/

//double	MW_CmdRPSX;					/*	MW speed Cmd in RPS (Rad Per Second)*/
double	MW_CmdRPSY;
//double	MW_CmdRPSZ;
double	MW_RPS;						/*	MW speed in RPS					*/
double dMW_RPS;						/*	MW acceleration					*/
double	MW_RPS_Prev;				/*	previous MW speed in RPS		*/
double	MW_T_Max;					/* wheel torque saturation limit	*/
double		MW_RPM_Max;					/* wheel speed saturation limit in RPM	*/
double	MW_RPS_Max;					/* wheel speed saturation limit	in RPS	*/
double		MW_RPM_Rating;				/* wheel reference speed int RPM	*/
double	MW_RPS_Rating;				/* wheel reference speed int RPS	*/

int     MW_statusY;        /*	wheel status flag				*/
				

double	BfMM[4];					/*	B-field from MM					*/
double	BfMM_prev[4];				/*	previous B-field from MM		*/
double	BfLO[4];					/*	B-field	from IGRF				*/
double	BfLO_prev[4];				/*	previous B-field from IGRF		*/
double  Aoi_orbit[9];

double  SUN_V[4],Sm_V[4];
double	P0[6];						/*	rate filter	covariance matrix	*/
double	P1[16];						/*	quaternion filter covariance	*/
double	P2[12];						/*	quaternion filter covariance	*/
double	P3[9];						/*	quaternion filter covariance	*/
double	SSP1[16];						/*	quaternion filter covariance	*/
double	SSP2[12];						/*	quaternion filter covariance	*/
double	SSP3[9];						/*	quaternion filter covariance	*/
double	RPY[3];						/*	roll/pitch/yaw					*/
double	SRPY[3];
double	RPY_ST[3];

double	Pre_SRPY[3];
double	SW[3];
double	Wbo_real[3];
double  Wbo_ST[3];

double	Wbo[3];						/*	orbit-coord. ref. ang. vel		*/
double	O[3];						/*	observed measurement			*/
double	C_pred[3];					/*	predicted measurement			*/
double	X[7];						/*	state vector (rate & oak-like)	*/
double	SX[7];
double	Y[2];						/*	state vector (pitch	filter)	俯仰角及俯仰角速率	*/
double  SJ_SS_Q[4];
double Q_oi[4];
double  Sm[4];
//int     TestNum=0;
int  Test[10]={0};
double  Test1[3],TEST2[3];
double  TEST3[3];
double  NUM=0;
double  SK1[12],SK2[12],SK3[9],SK4[9],TESTK[25],DD,DTEST[16],Sb,H2[8],H1[12];
double	BYLO_TMP[9];
double Qbo_prev[4];
double dQbo[4];
				/*=NOTE===================================================
				 	the definition of state vector X is as follows
						X[0] : q1
						X[1] : q2
						X[2] : q3
						X[3] : q4
						X[4] : wx
						X[5] : wy
						X[6] : wz
					the	state vector Y is completely reserved for
					the pitch filter
						Y[0] : pitch
						Y[1] : pitch rate

					hence if rate-filter only then the buffers [4][5][6]
					are valid and if oak-filter, all buffers are val
				========================================================*/

double	Ix;							/*	momentum inertia around	x axis	*/
double	Iy;							/*	momentum inertia around	y axis	*/
double	Iz;							/*	momentum inertia around	z axis	*/
double	J_MW;						/*	pitch wheel	mometum	inertia		*/

short	Period_Nom;					/*  Control period in theory		*/
double	Period_True;				/*  Control period in fact			*/

// ??? by ZKC
int		Period_, P_counter;			/*	for	coarse orbital period		*/

double	HP[16];						/*	filter matrix buffer			*/
double	K[21];						/*	kalman gain	matrix buffer		*/
double	R1[3];						/*	msmt noise variance	(navmag)	*/
double  SR1[3];
double	Q1[3];						/*	process	noise variance			*/
double	Q2[3];						/*	process	noise variance (oak2)	*/
double	SSQ2[3];						/*	process	noise variance (oak2)	*/
double	dX[7];						/*	back-point table for integrator	*/
double	dBeta,dBeta0,dBeta2;	    /*	dbeta/dt						*/
double	Beta_prev;					/*	previous beta-angle				*/
double	W0;							/*	orbital	rate (mean motion)		*/
double	GG;							/*	gravity	gradient constant		*/
double	Mjd;						/*	modified julian	date			*/
double  Rate_prev;
double	BYLO[9];					/*	attitude matrix					*/
int		Filter_Init;				/*	filter init	flag				*/
int		Filter_Sts;					/*	filter status flag				*/
int		NoMMTlm;					/*	no telemetry received			*/
int		NoMMTlm_Counter;			/*	no telemetry counter			*/
struct	tm	*utc;					/*	utc	timer*/

double	ModGH[82];

struct	ADCS_Parameters
{
	float	G;					/*	beta angle smoother	gain		*/
	float	G1;					/*	herman's pitch estimator gain 1	*/
	float	G2;					/*	herman's pitch estimator gain 2	*/
	float	wy;					/*	y-spin rate	command				*/
	float	Ks1;				/*	spin cntl gain (b-dot)			*/
	float	Kd1;				/*	delib cntl gain	(b-dot)			*/
	float	Kd[3];				/*	derivative gains (cpl)			*/
	float	Kp[2];				/*	proportional gains (cpl)		*/
	float	Kpw;				/*	p-gain for pd-cntl (wheel)		*/
	float	Kdw;				/*	d-gain for pd-cntl (wheel)		*/
	float	W_step;				/*	for	wheel spun-up control		*/
	float	PitchMask;			/*	yw cntl	spun-up	==>	nominal		*/
	float	TStep;				/*	integration	step for oakfilter	*/
} Adc;


struct	Orbit_Parameters
{
	double	l2epoc;				/*	2-line element epoch in	mjd		*/
	double	K2;					/*	j2 term	param					*/
	double	A30;				/*	j3 term	param					*/
	double	ke;					/*	sqrt(gravity const)				*/
	double	n0;					/*	"mean" mean	motion				*/
	double	e0;					/*	"mean" eccentricity				*/
	double	i0;					/*	"mean" inclination				*/
	double	an0;				/*	"mean" ascending node			*/
	double	ap0;				/*	"mean" argument	of perigee		*/
	double	M0;					/*	"mean" mean	anomary				*/
	double	n0dot;				/*	"mean" mean	motion dot /2		*/
	double	n0dot2;				/*	"mean" mean	motion dotdot /6	*/
	double	a1;					/*	"mean" semi-major axis			*/
	double	d1;					/*	correction term	delta1			*/
	double	a0;					/*	recovered original semi-major ax*/
	double	p0;					/*	a*(1 - e*e)						*/
	double	q0;					/*	perigee	height					*/
	double	L0;					/*	M +	ap + an	at epoch			*/
	double	dan;				/*	ascending node secular
									perturbation due to	j2			*/
	double	dap;				/*	argument of	perigee	secular
									perturbation due to	j2			*/
	double  UTC2UT1;            /*  utc-ut1 time offset in day      */
	double  Eeqnox;             /*  equation of equinox             */
} Eph;


	short obctozpf1;
	short obctozpf2;
	short obctozpf3;
	char obctozpf1c;
	char obctozpf2c;
	char obctozpf3c;
	char obctozpf4c;
	char obctozpf5c;
	char obctozpf6c;
	unsigned long obctozpflong;
	double	obctozpfdouble;
	double  zpf_N[3];
	double  MM[3];
	double  MMtemp[3];
	unsigned long pre_second;
	unsigned long pre_week;
	int	gps_sign;
	double   MW_temp;
    double   doubledummy;
    short    shortdummy;

    double SStemp[4];
    double SS[4];

/*====================
 * function	prototype
 *====================*/
void	M_Add( int n,  double *a, double *b, double *c );
void	FPF( int n, int m, double *f, double *p, double *a );
void	M_Product( int n, int m, int l, double *a, double *b, double *c );
void	M_Transpose( int n, int m, double *a, double *b );
double	M_Inverse3x3( double *x, double *y );
double	M_Inverse4x4( double *x);
double	M_Inverse5x5( double *x, double *y );
int		Sign( double arg );
void	xBYLO( double *q, double *a );
int		Q_Normaliser( double *q );
void	P_Normaliser( void );
double	Angle_Normaliser( double arg );
void	Qtn2RPY( double *a, double *e );
int		RPY2Qtn( double *a, double *q );
int		X_prop( double dt, double *N );
void	P_prop( double dt );
void	MT_FiringEffect( double *cmd, double *N );
double	dBeta_Filter( double dt, double beta, double dbeta );
void	Cmd_Limiter( double *cmd );
void	MT_Controller( double dt, double *cmd );
int		YW_Controller (double dt);
void	MM_MalTLM( void );
void	BF_Measured( void );
void	TC_Actuator( void );
void Q_Inverse(double *a, double *b);
void Q_Product(double *a, double *b, double *c);
double	mgps2mjd( unsigned int weeks,unsigned long seconds );
void	Sun( double mjd, double *r );
int		eclipse( double *r, double *s );
int     Sgp (double dt, double *x, double *v);
void    BF_Predicted(double *r, double *z,double *b);
void	igrf_b_field(double *gh,double *r,double *b);
int		Orbit( double *r, double *z );/* Modified by LXF */


void	NoFilter( double dt );
int		RateFilter( double dt );
void	PitchFilter( double dt, double Ny );
int		OakFilter( double *o, double *c, double *r , double *so, double *sc);
int     SunFilter( double *o, double *c,double *so, double *sc, double *r,double *sr,double *sv);
void    StarFilter();
void	ADCS_Initialise( void );
int		ADCS_Models( void );
int		ADCS_Filter( void );
int		ADCS_Control( void );
int		ADCS_LoadCfgFile( void );
void	ADCS_Close( void );
void	ADCS_AutoRunning( void );
void ADCS_CfgUpdate(void);

//int		ADCS_Main( void	);

int		TLM_MM( void );
int		TLM_MW_RPM( void );
int		TC_MTCmd( void );
int		TC_MWCmd( void );
int		TX_TimeSyn( void );
int		TX_Synchronous( void );
//int		TX_ZKC_Test1( void );
//int		TX_ZKC_Test2( void );

//命令分配，地址，值1，值2

int	CmdDisposal(INT8U addr_cmd, short value_c, short value_s/*, unsigned short value_us, float value_sin, short value2*/);
int	TLM_MCUData( void );
int	WriteTlmData( void );
int	TLM_ADopen( void );

typedef union REAL_F
{
    float f;
    INT8U ui[4];
} real_f;
typedef union REAL_D
{
    double dbl;
	INT8U un[8];
} real_d;

void    ZPFMeasure(void );
short   ByteToShort(INT8U* pc);
float   ByteToFloat(INT8U* pc);
double  ByteToDouble(const INT8U* pc, int n);

