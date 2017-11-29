#include "ADCS.h"
#include "ADCS_TTC.h"
#include "ucos_ii.h"
#include <math.h>
/*=============================================================================
  程序名: ADCS.c< NS-2(MEMSst)ADCS星上软件> 
  编 辑 : 师帅
  日 期 : 20150715
  =============================================================================*/

//GPS时间,来源于操作系统：
extern INT16U gps_week;
extern INT32U gps_second;
//操作系统事件：
extern	OS_EVENT *ADCS_FLAG;
extern	OS_EVENT *ACSQCOM;
extern	OS_EVENT *CAN_Q_Rec;

//////////////////////////////////////////////////////////////////
//来自ADCS_TTC.c的变量

//到MW的遥控命令
//extern float    u[25];
extern   short    tcMWRPMy;
extern   short    tcMWRPMyDr;
extern   short	  tcMW_kpy;
//extern   short	tcMW_kiy;

//到MCU2的遥控命令
extern   short	tcMCU2MTPWMx;
extern   short	tcMCU2MTPWMy;
extern   short	tcMCU2MTPWMz;

//动量轮的遥测量
extern   short    tlmMWRPMy;
extern   short    tlmMWRPMyDr;

//MCU1的遥测量
extern   short		tlmMCU1MM1x;
extern   short		tlmMCU1MM1y;
extern   short		tlmMCU1MM1z;
extern   short		tlmMCU1MM1Ts;
extern   short		tlmMCU1MMS;
extern   short		tlmMCU1MMSTs;
extern   short    tlmMCU1ASSPx;
extern   short    tlmMCU1ASSNx;
extern   short    tlmMCU1ASSPy;
extern   short    tlmMCU1ASSNy;
extern   short    tlmMCU1ASSTs;
extern   short		tlmMCU1MT1IMon;
extern   short		tlmMCU1MT2IMon;
extern   short		tlmMCU1MT3IMon;
extern   short		tlmMCU1MTVref;
extern   short		tlmMCU1Vref;
extern   short		tlmMCU1P5VMon;
extern   short    tlmMCU1P5VIMon;
extern   short		tlmMCU1P12VMon;
extern   short    tlmMCU1P12VIMon;
extern   short		tlmMCU1N12VMon;
extern   short    tlmMCU1N12VIMon;
extern   short    tlmMCU1GYRO;
extern   short    tlmMCU1TsVref;

//MCU2的遥测量
extern   short		tlmMCU2MMS;
extern   short		tlmMCU2MMSTs;
extern   short    tlmMCU2ASSPx;
extern   short    tlmMCU2ASSNx;
extern   short    tlmMCU2ASSPy;
extern   short    tlmMCU2ASSNy;
extern   short    tlmMCU2ASSTs;
extern   short		tlmMCU2MT1IMon;
extern   short		tlmMCU2MT2IMon;
extern   short		tlmMCU2MT3IMon;
extern   short		tlmMCU2MTVref;
extern   short		tlmMCU2Vref;
extern   short		tlmMCU2P5VMon;
extern   short    tlmMCU2P5VIMon;
extern   short		tlmMCU2P12VMon;
extern   short    tlmMCU2P12VIMon;
extern   short		tlmMCU2N12VMon;
extern   short    tlmMCU2N12VIMon;
extern   short    tlmMCU2GYRO;
extern   short    tlmMCU2TsVref;
/////////////////////////////////////////////////////////////////

//系统控制对共享资源的访问
extern INT8U ERR_SHARE_PRINTF;
extern OS_EVENT *SHARE_PRINTF;

//为更新配置文件所设置的变量，包括数组buffer及标志位Flag
extern   INT8U  adcsbuf1[1300];
extern   INT8U  adcsbuf2[200];
extern int ACSFL_Flag;
extern int ACSGD_Flag;

//其他变量
//double Y0,Y1,X4,X5,X6;//仿真用，由于系统只能打印整数，此Y0、Y1是俯仰滤波器y轴欧拉角和角速度乘以1000后的整数部分，单位rad（rad/s）

//extern   OS_EVENT *oneQ;

void Q_Inverse(double *a, double *b)
{
        b[0] = -a[0];
        b[1] = -a[1];
        b[2] = -a[2];
        b[3] = a[3];
}

void Q_Product(double *a, double *b, double *c)
{
        c[0] = b[3]*a[0]-b[2]*a[1]+b[1]*a[2]+b[0]*a[3];
        c[1] = b[2]*a[0]+b[3]*a[1]-b[0]*a[2]+b[1]*a[3];
        c[2] = -b[1]*a[0]+b[0]*a[1]+b[3]*a[2]+b[2]*a[3];
        c[3] = -b[0]*a[0]-b[1]*a[1]-b[2]*a[2]+b[3]*a[3];

}


/*=============================================================================
function: FPF
actions	: compute F*P*transpose(F)
input	:	n	->	dimension of matrix	p (has to be square)
m	->	rows(column	dimension) of f	(row-dim has to	be n)
f	->	m x	n matrix
p	->	n x	n matrix
return	:	a	->	a =	f*p*t(f)  /	m x	m matrix
=============================================================================*/
void	FPF( int n, int m, double *f, double *p, double	*a )
{
        int		i, j, k, l;

        for( i=0; i<m; i++ )	{
                for( j=0; j<m; j++ )	{
                        a[i*m+j] = 0.0;				/* clear matrix buffer */

                        for( k=0; k<n; k++ )
                                for( l=0; l<n; l++ )
                                        a[i*m+j] +=	f[i*n+k]*p[k*n+l]*f[j*n+l];
                }
        }
} /* void FPF */



/*=============================================================================
function: M_Product
actions	: calculates product of	matrices a and b
input	:	n	:	column dimension of	matrix a
m	:	row	dimension of matrix	a
l	:	row	dimension of matrix	b
a	:	n x	m matrix
b	:	m x	l matrix
return	:	c	:	c =	a*b
=============================================================================*/
void	M_Product( int n, int m, int l, double *a, double *b, double *c )
{
        int		i, j, k;


        for( i=0; i<n; i++ )	{
                for( j=0; j<l; j++ )	{
                        c[i*l+j] = 0.0;
                        for	( k	= 0	; k	< m	; k++ )
                                c[i*l+j] +=	a[i*m+k] * b[k*l+j];
                }
        }
} /* void M_Product */

/*=============================================================================
function: M_Add
actions	: calculates product of	matrices a and b
input	:	n	:	column dimension of	matrix a
m	:	row	dimension of matrix	a
l	:	row	dimension of matrix	b
a	:	n x	m matrix
b	:	m x	l matrix
return	:	C	:	C =	a-b
=============================================================================*/
void	M_Add( int n,  double *a, double *b, double *c )//5
{
        int	 i;

        for( i=0; i<n; i++ )
        {
                c[i] =	a[i] - b[i];
        }

} /* void M_Add */

/*=============================================================================
function: M_Transpose
actions	: calculates transpose matrix of a
input	:	n	:	column dimension of	matrix a
m	:	row	dimension of matrix	a
a	:	n x	m matrix
output	:	b	:	m x	n matrix transpose(a)
=============================================================================*/
void	M_Transpose( int n, int m, double *a, double *b )
{
        int		i, j;


        for( i=0; i<m; i++ )	{
                for( j=0; j<n; j++ )	{
                        b[i*n+j] = a[j*m+i];
                }
        }
}

/*=============================================================================
function: M_Inverse3x3
actions	: compute 3x3 matrix inverse
input	:	x	:	3 x	3 matrix
output	:	y	:	un-normalised inverse matrix of	x
return	: determinant of x
=============================================================================*/
double	M_Inverse3x3( double *x, double *y )
{
        y[0]	=	x[4]*x[8] -	x[7]*x[5];
        y[3]	=	x[5]*x[6] -	x[8]*x[3];
        y[6]	=	x[3]*x[7] -	x[6]*x[4];

        y[1]	=	x[7]*x[2] -	x[1]*x[8];
        y[4]	=	x[8]*x[0] -	x[2]*x[6];
        y[7]	=	x[6]*x[1] -	x[0]*x[7];

        y[2]	=	x[1]*x[5] -	x[4]*x[2];
        y[5]	=	x[2]*x[3] -	x[5]*x[0];
        y[8]	=	x[0]*x[4] -	x[3]*x[1];

        return( x[0]*y[0] + x[1]*y[3] + x[2]*y[6] );
}

double	M_Inverse4x4( double *x)
{
        double y[4];

        y[0]= x[5]*x[10]*x[15]+x[6]*x[11]*x[13]+x[9]*x[14]*x[7]-x[13]*x[10]*x[7]-x[14]*x[11]*x[5]-x[15]*x[9]*x[6];
        y[1]= x[4]*x[10]*x[15]+x[6]*x[11]*x[12]+x[8]*x[14]*x[7]-x[12]*x[10]*x[7]-x[14]*x[11]*x[4]-x[15]*x[8]*x[6];
        y[2]= x[4]*x[9]*x[15]+x[5]*x[11]*x[12]+x[8]*x[13]*x[7]-x[12]*x[9]*x[7]-x[13]*x[11]*x[4]-x[15]*x[8]*x[5];
        y[3]= x[4]*x[9]*x[14]+x[5]*x[10]*x[12]+x[8]*x[13]*x[6]-x[12]*x[9]*x[6]-x[13]*x[10]*x[4]-x[14]*x[8]*x[5];

        return( x[0]*y[0] + x[1]*(-y[1]) + x[2]*y[2] + x[3]*(-y[3]) );
}


double	M_Inverse5x5( double *x, double *y )
{

        double temp[16];

        temp[0]=x[6];   temp[1]=x[7];    temp[2]=x[8];    temp[3]=x[9];
        temp[4]=x[11];  temp[5]=x[12];   temp[6]=x[13];   temp[7]=x[14];
        temp[8]=x[16];  temp[9]=x[17];   temp[10]=x[18];  temp[11]=x[19];
        temp[12]=x[21]; temp[13]=x[22];  temp[14]=x[23];  temp[15]=x[24];
        y[0]=M_Inverse4x4(temp);


        temp[0]=x[5];   temp[1]=x[7];    temp[2]=x[8];    temp[3]=x[9];
        temp[4]=x[10];  temp[5]=x[12];   temp[6]=x[13];   temp[7]=x[14];
        temp[8]=x[15];  temp[9]=x[17];   temp[10]=x[18];  temp[11]=x[19];
        temp[12]=x[20]; temp[13]=x[22];  temp[14]=x[23];  temp[15]=x[24];
        y[5]=-M_Inverse4x4(temp);


        temp[0]=x[5];   temp[1]=x[6];    temp[2]=x[8];    temp[3]=x[9];
        temp[4]=x[10];  temp[5]=x[11];   temp[6]=x[13];   temp[7]=x[14];
        temp[8]=x[15];  temp[9]=x[16];   temp[10]=x[18];  temp[11]=x[19];
        temp[12]=x[20]; temp[13]=x[21];  temp[14]=x[23];  temp[15]=x[24];
        y[10]=M_Inverse4x4(temp);


        temp[0]=x[5];   temp[1]=x[6];    temp[2]=x[7];     temp[3]=x[9];
        temp[4]=x[10];  temp[5]=x[11];   temp[6]=x[12];    temp[7]=x[14];
        temp[8]=x[15];  temp[9]=x[16];   temp[10]=x[17];   temp[11]=x[19];
        temp[12]=x[20]; temp[13]=x[21];  temp[14]=x[22];   temp[15]=x[24];
        y[15]=-M_Inverse4x4(temp);

        temp[0]=x[5];   temp[1]=x[6];    temp[2]=x[7];     temp[3]=x[8];
        temp[4]=x[10];  temp[5]=x[11];   temp[6]=x[12];    temp[7]=x[13];
        temp[8]=x[15];  temp[9]=x[16];   temp[10]=x[17];   temp[11]=x[18];
        temp[12]=x[20]; temp[13]=x[21];  temp[14]=x[22];   temp[15]=x[23];
        y[20]=M_Inverse4x4(temp);


        temp[0]=x[1];  temp[1]=x[2];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[11]; temp[5]=x[12];  temp[6]=x[13];    temp[7]=x[14];
        temp[8]=x[16]; temp[9]=x[17];  temp[10]=x[18];   temp[11]=x[19];
        temp[12]=x[21];temp[13]=x[22]; temp[14]=x[23];   temp[15]=x[24];
        y[1]=-M_Inverse4x4(temp);

        temp[0]=x[0];  temp[1]=x[2];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[10]; temp[5]=x[12];  temp[6]=x[13];    temp[7]=x[14];
        temp[8]=x[15]; temp[9]=x[17];  temp[10]=x[18];   temp[11]=x[19];
        temp[12]=x[20];temp[13]=x[22]; temp[14]=x[23];   temp[15]=x[24];
        y[6]=M_Inverse4x4(temp);

        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[10]; temp[5]=x[11];  temp[6]=x[13];    temp[7]=x[14];
        temp[8]=x[15]; temp[9]=x[16];  temp[10]=x[18];   temp[11]=x[19];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[23];   temp[15]=x[24];
        y[11]=-M_Inverse4x4(temp);

        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[4];
        temp[4]=x[10]; temp[5]=x[11];  temp[6]=x[12];    temp[7]=x[14];
        temp[8]=x[15]; temp[9]=x[16];  temp[10]=x[17];   temp[11]=x[19];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[22];   temp[15]=x[24];
        y[16]=M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[3];
        temp[4]=x[10]; temp[5]=x[11];  temp[6]=x[12];    temp[7]=x[13];
        temp[8]=x[15]; temp[9]=x[16];  temp[10]=x[17];   temp[11]=x[18];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[22];   temp[15]=x[23];
        y[21]=-M_Inverse4x4(temp);


        temp[0]=x[1];   temp[1]=x[2];    temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[6];   temp[5]=x[7];    temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[16];  temp[9]=x[17];   temp[10]=x[18];   temp[11]=x[19];
        temp[12]=x[21]; temp[13]=x[22];  temp[14]=x[23];   temp[15]=x[24];
        y[2]=M_Inverse4x4(temp);

        temp[0]=x[0];   temp[1]=x[2];    temp[2]=x[3];      temp[3]=x[4];
        temp[4]=x[5];   temp[5]=x[7];    temp[6]=x[8];      temp[7]=x[9];
        temp[8]=x[15];  temp[9]=x[17];   temp[10]=x[18];    temp[11]=x[19];
        temp[12]=x[20]; temp[13]=x[22];  temp[14]=x[23];    temp[15]=x[24];
        y[7]=-M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[15]; temp[9]=x[16];  temp[10]=x[18];   temp[11]=x[19];
        temp[12]=x[20]; temp[13]=x[21];temp[14]=x[23];   temp[15]=x[24];
        y[12]=M_Inverse4x4(temp);

        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[7];     temp[7]=x[9];
        temp[8]=x[15]; temp[9]=x[16];  temp[10]=x[17];   temp[11]=x[19];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[22];   temp[15]=x[24];
        y[17]=-M_Inverse4x4(temp);



        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[3];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[7];     temp[7]=x[8];
        temp[8]=x[15]; temp[9]=x[16];  temp[10]=x[17];   temp[11]=x[18];
        temp[12]=x[20]; temp[13]=x[21];temp[14]=x[22];   temp[15]=x[23];
        y[22]=M_Inverse4x4(temp);



        temp[0]=x[1];  temp[1]=x[2];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[6];  temp[5]=x[7];   temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[11]; temp[9]=x[12];  temp[10]=x[13];   temp[11]=x[14];
        temp[12]=x[21];temp[13]=x[22]; temp[14]=x[23];   temp[15]=x[24];
        y[3]=-M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[2];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[7];   temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[10]; temp[9]=x[12];  temp[10]=x[13];   temp[11]=x[14];
        temp[12]=x[20]; temp[13]=x[22];temp[14]=x[23];   temp[15]=x[24];
        y[8]=M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[10]; temp[9]=x[11];  temp[10]=x[13];   temp[11]=x[14];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[23];   temp[15]=x[24];
        y[13]=-M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[7];     temp[7]=x[9];
        temp[8]=x[10]; temp[9]=x[11];  temp[10]=x[12];   temp[11]=x[14];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[22];   temp[15]=x[24];
        y[18]=M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[3];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[7];     temp[7]=x[8];
        temp[8]=x[10]; temp[9]=x[11];  temp[10]=x[12];   temp[11]=x[13];
        temp[12]=x[20];temp[13]=x[21]; temp[14]=x[22];   temp[15]=x[23];
        y[23]=-M_Inverse4x4(temp);


        temp[0]=x[1];  temp[1]=x[2];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[6];  temp[5]=x[7];   temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[11]; temp[9]=x[12];  temp[10]=x[13];   temp[11]=x[14];
        temp[12]=x[16];temp[13]=x[17]; temp[14]=x[18];   temp[15]=x[19];
        y[4]=M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[2];   temp[2]=x[3];     temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[7];   temp[6]=x[8];     temp[7]=x[9];
        temp[8]=x[10]; temp[9]=x[12];  temp[10]=x[13];   temp[11]=x[14];
        temp[12]=x[15];temp[13]=x[17]; temp[14]=x[18];   temp[15]=x[19];
        y[9]=-M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[3];      temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[8];      temp[7]=x[9];
        temp[8]=x[10]; temp[9]=x[11];  temp[10]=x[13];    temp[11]=x[14];
        temp[12]=x[15]; temp[13]=x[16]; temp[14]=x[18];   temp[15]=x[19];
        y[14]=M_Inverse4x4(temp);

        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];      temp[3]=x[4];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[7];      temp[7]=x[9];
        temp[8]=x[10]; temp[9]=x[11];  temp[10]=x[12];    temp[11]=x[14];
        temp[12]=x[15]; temp[13]=x[16]; temp[14]=x[17];   temp[15]=x[19];
        y[19]=-M_Inverse4x4(temp);


        temp[0]=x[0];  temp[1]=x[1];   temp[2]=x[2];     temp[3]=x[3];
        temp[4]=x[5];  temp[5]=x[6];   temp[6]=x[7];     temp[7]=x[8];
        temp[8]=x[10]; temp[9]=x[11];  temp[10]=x[12];   temp[11]=x[13];
        temp[12]=x[15];temp[13]=x[16]; temp[14]=x[17];   temp[15]=x[18];
        y[24]=M_Inverse4x4(temp);

        return( x[0]*y[0] + x[1]*y[5] + x[2]*y[10] + x[3]*y[15] + x[4]*y[20]);
}


/*=============================================================================
function: Sign
actions	: sign of argument
input	: argument (arg)
return	: sign of argument
=============================================================================*/
int		Sign( double arg )
{
        if( arg > 0.0 )		return(  1 );
        if( arg < 0.0 )		return( -1 );

        return( 0 );
}

/*=============================================================================
function: Q_Normaliser
actions	: normalise	quaternion
input	: non-normalised quaternion
output	:     normalised quaternion
date	: 30/09/98
=============================================================================*/
int		Q_Normaliser( double *q )
{
        double	x;


        x	= sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );

        if (x <	ZERO)	return( 21 );

        q[0]	/= x;
        q[1]	/= x;
        q[2]	/= x;
        q[3]	/= x;

        return( 0 );
} /* void Q_Normaliser */

/*=============================================================================
function: P_Normaliser
actions	: normalise	covariance matrix to be	always symmetric
input	:
output	:
=============================================================================*/
void	P_Normaliser( void )
{
        double	x;
        int		i, j;


        for( i=0; i<4; i++ )	{
                for( j=0; j<4; j++ )	{
                        if( i != j )	{
                                x	= 0.5 * ( P1[4*i+j] + P1[4*j+i] );
                                P1[4*i+j]	= P1[4*j+i]	= x;
                        }
                }
        }

        for( i=0; i<3; i++ )	{
                for( j=0; j<3; j++ )	{
                        if( i != j )	{
                                x	= 0.5 * ( P3[3*i+j] + P3[3*j+i] );
                                P3[3*i+j]	= P3[3*j+i]	= x;
                        }
                }
        }

}	/*	void P_Normaliser */



/*=============================================================================
function: Angle_Normaliser
actions	: normalise	angle
input	: arg :	  angle	to be normalised
return	: normalised (-pi to pi) angle
=============================================================================*/
double	Angle_Normaliser( double arg )
{
        return( atan2( sin(arg), cos(arg) ) );
}

/*=============================================================================
function: xBYLO
actions	: compute attitude matrix
input	: q	 :	 quaternion
output	: a	 :	 attitude matrix
=============================================================================*/
void	xBYLO( double *q, double *a )
{
        a[0]	= q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
        a[1]	= 2.0*(q[0]*q[1] + q[2]*q[3]);
        a[2]	= 2.0*(q[0]*q[2] - q[1]*q[3]);

        a[3]	= 2.0*(q[0]*q[1] - q[2]*q[3]);
        a[4]	= q[1]*q[1] - q[2]*q[2] + q[3]*q[3] - q[0]*q[0];
        a[5]	= 2.0*(q[1]*q[2] + q[0]*q[3]);

        a[6]	= 2.0*(q[0]*q[2] + q[1]*q[3]);
        a[7]	= 2.0*(q[1]*q[2] - q[0]*q[3]);
        a[8]	= q[2]*q[2] + q[3]*q[3] - q[0]*q[0] - q[1]*q[1];
}

/*=============================================================================
function: Qtn2rpy
actions	: quaternion to	euler angle
input	: a	 :	attitude matrix
output	: e	 :	 roll/pitch/yaw
=============================================================================*/
void Qtn2RPY( double *a, double *e )
{
        double	a7;


        a7		= (double) Sign( a[7] ) * min( fabs(a[7]), 1.0 );

        e[2]	= atan2( a[1], a[4] );
        e[0]	= asin ( -a7);
        e[1]	= atan2( a[6], a[8] );	/*	euler 2-1-3	 */

} /* void Qtn2rpy */


/*=============================================================================
function: RPY2Qtn
actions	: attitude matrix to quaternion
input	: a	 :	 attitude matrix
output	: q	 :	 quaternion
return	: error	code
=============================================================================*/
int		RPY2Qtn( double *a, double *q )
{
        q[3]	= 0.5 * sqrt( 1.0+a[0]+a[4]+a[8] );

        if( fabs(q[3]) < ZERO )		 return( 20 );

        q[0]	= 0.25 * ( a[5]-a[7] ) / q[3];
        q[1]	= 0.25 * ( a[6]-a[2] ) / q[3];
        q[2]	= 0.25 * ( a[1]-a[3] ) / q[3];
        /* derive quaternion vector */
        return( 0 );
} /* int RPY2Qtn */



/*=============================================================================
function: X_prop
action	: propagate	attitude state by simple adams 2nd integrator
input	:	dt :   measurement sampling	step (in seconds)
N  :   averaged	torque vector induced by mt	torquer
output	:
=============================================================================*/
int		X_prop( double dt, double *N )
{
        /*************************************
note:
X[0]:	q1
X[1]:	q2
X[2]:	q3
X[3]:	q4
X[4]:	wx
X[5]:	wy
X[6]:	wz
         *************************************/
        int		i;
        int		err;
        double	hyx, hyz;
        double	dhy;					/*	attitude equation constant		*/
        double	lyz, lzx, lxy;			/*	attitude equation constant		*/
        double	dx[7];					/*	state dot vector buffer			*/
        double	t;						/*	timer for integrator			*/
        double	k;						/*	gravity	gradient torque	const.	*/


        lyz		= (Iz - Iy) / Ix;
        lzx		= (Ix - Iz) / Iy;
        lxy		= (Iy - Ix) / Iz;

        switch( Filter )	{
                case RATE_FILTER:
                case HYBRID_FILTER:

                        hyx = J_MW / Ix *  MW_RPS;
                        dhy	= J_MW / Iy * dMW_RPS;
                        hyz	= J_MW / Iz *  MW_RPS;

                        t	= 0.0;

                        do	{
                                dx[4]	= N[0]/Ix - lyz*X[5]*X[6] + hyx*X[6];
                                dx[5]	= N[1]/Iy - lzx*X[6]*X[4] - dhy;
                                dx[6]	= N[2]/Iz - lxy*X[4]*X[5] - hyz*X[4];
                                /*	euler angle	dot	dot				*/
                                for( i = 4; i < 7; i++ )	{
                                        X[i]   += (3.0*dx[i] - dX[i]) * Adc.TStep / 2.0;
                                        dX[i]	= dx[i];
                                }						/*	integration	ahead: 2nd adams	*/
                                /*	update adams back-point	table	*/
                                t	+= Adc.TStep;
                        } while( t < dt );

                        break;

                case OAK_FILTER:
                case ATACQ_FILTER:
                case Sun_Filter:
                        /************************************
                          state propagation
                         ************************************/

                        hyx	= J_MW *MW_RPS / Ix;
                        hyz	= J_MW *MW_RPS / Iz;
                        dhy	= J_MW * dMW_RPS / Iy;

                        t	= 0.0;
                        k	= 3.0 * GG * GG;

                        do	{
                                dx[0]	= 0.5 * ( Wbo[2]*X[1] - Wbo[1]*X[2] + Wbo[0]*X[3]);
                                dx[1]	= 0.5 * (-Wbo[2]*X[0] + Wbo[0]*X[2] + Wbo[1]*X[3]);
                                dx[2]	= 0.5 * ( Wbo[1]*X[0] - Wbo[0]*X[1] + Wbo[2]*X[3]);
                                dx[3]	= 0.5 * (-Wbo[0]*X[0] - Wbo[1]*X[1] - Wbo[2]*X[2]);
                                /*	quaternion-dot					*/
                                dx[4]	= -lyz*X[5]*X[6] + hyx*X[6];
                                dx[5]	= -lzx*X[6]*X[4] - dhy;
                                dx[6]	= -lxy*X[4]*X[5] - hyz*X[4];
                                /*	3 wheels dynamics				*/
                                /*	gyroscopic terms				*/
                                dx[4]	+= k*lyz*BYLO[5]*BYLO[8];
                                dx[5]	+= k*lzx*BYLO[8]*BYLO[2];
                                dx[6]	+= k*lxy*BYLO[2]*BYLO[5];
                                /*	gravity	gradient torque			*/
                                dx[4]	+= N[0] / Ix;
                                dx[5]	+= N[1] / Iy;
                                dx[6]	+= N[2] / Iz;
                                /*	torquer	vector by controller	*/
                                /*	and	finally	omega-dot			*/
                                for	( i = 0; i < 7; i++ )	{
                                        X[i]   += (3.0*dx[i] - dX[i]) * Adc.TStep / 2.0;
                                        dX[i]	= dx[i];
                                }
                                /*	integration	ahead: 2nd adams	*/
                                /*	update adams back-point	table	*/
                                err	= Q_Normaliser( X );
                                if( err != 0 )		return( err );
                                /*	normalise quaternion			*/

                                xBYLO( X, BYLO );	/*	recover	attitude matrix			*/

                                Wbo[0]	= X[4] + BYLO[1]*W0;
                                Wbo[1]	= X[5] + BYLO[4]*W0;
                                Wbo[2]	= X[6] + BYLO[7]*W0;
                                /*	recover	orbit ref. velocity		*/
                                t	+= Adc.TStep;
                        }	while( t < dt );

                        break;
        } /* end of switch( Filter ) */

        return( 0 );

} /* int X_prop */



/*=============================================================================
function: P_prop
actions	: propagate	covariance matrix
input	: dt  :	  propagation time in second
output	:
=============================================================================*/
void	P_prop( double dt )
{
        int		i;
        double	x[12], y[16];
        double	lyz, lzx, lxy;			/*	attitude equation constant		*/


        lyz		= (Iz - Iy) / Ix;
        lzx		= (Ix - Iz) / Iy;
        lxy		= (Iy - Ix) / Iz;

        switch( Filter )	{
                case RATE_FILTER:
                case HYBRID_FILTER:

                        y[0]	= 1.0;
                        y[1]	= (-lyz*X[6])*dt;
                        y[2]	= (-lyz*X[5] + J_MW*MW_RPS/Ix)*dt;

                        y[3]	= (-lzx*X[6])*dt;
                        y[4]	= 1.0;
                        y[5]	= (-lzx*X[4])*dt;

                        y[6]	= (-lxy*X[5] - J_MW*MW_RPS/Iz)*dt;
                        y[7]	= (-lxy*X[4])*dt;
                        y[8]	= 1.0;
                        /************************************
                          state transition matrix
                          X =	I +	F*dt
                          F =	d(dwx,dwy,dwz)/d(wx,wy,wz)
                         ************************************/

                        K[0]	= P0[0];
                        K[1]	= P0[1];
                        K[2]	= P0[2];

                        K[3]	= P0[1];
                        K[4]	= P0[3];
                        K[5]	= P0[4];

                        K[6]	= P0[2];
                        K[7]	= P0[4];
                        K[8]	= P0[5];		/*	recover	3x3	covariance matrix	*/

                        FPF( 3, 3, y, K, x );	/*	x: ^P0 propagated !				*/

                        P0[0]	= x[0] + Q1[0]*dt;
                        P0[1]	= x[1];
                        P0[2]	= x[2];
                        P0[3]	= x[4] + Q1[1]*dt;
                        P0[4]	= x[5];
                        P0[5]	= x[8] + Q1[2]*dt;
                        /*	add	process	noise				*/

                        break;


                case OAK_FILTER:
                case ATACQ_FILTER:
                        /*************************************
                          quaternion filter entry

note:
for	the	covariance propagation
the	effect of gravity gradient
is ignored in order	to save
the	complexity
                         **************************************/

                        /*====================================
                          1 +	d(qtn-dot)/d(qtn)*dt
                          ====================================*/
                        x[1]	= X[2]*W0;

                        y[0]	= 1.0 + X[0]*x[1]*dt;
                        y[4]	= (X[1]*x[1] - 0.5*Wbo[2]) * dt;
                        y[8]	= (X[2]*x[1] + 0.5*Wbo[1] - W0) * dt;
                        y[12]	= (X[3]*x[1] - 0.5*Wbo[0]) * dt;

                        x[1]	= X[3]*W0;

                        y[1]	= (X[0]*x[1] + 0.5*Wbo[2]) * dt;
                        y[5]	= 1.0	+ X[1]*x[1]*dt;
                        y[9]	= (X[2]*x[1] - 0.5*Wbo[0]) * dt;
                        y[13]	= (X[3]*x[1] - 0.5*Wbo[1] - W0) * dt;

                        x[1]	= -X[0]*W0;

                        y[2]	= (X[0]*x[1] - 0.5*Wbo[1] + W0) * dt;
                        y[6]	= (X[1]*x[1] + 0.5*Wbo[0]) * dt;
                        y[10]	= 1.0 + X[2]*x[1]*dt;
                        y[14]	= (X[3]*x[1] - 0.5*Wbo[2]) * dt;

                        x[1]	= -X[1]*W0;

                        y[3]	= (X[0]*x[1] + 0.5*Wbo[0]) * dt;
                        y[7]	= (X[1]*x[1] + 0.5*Wbo[1] + W0) * dt;
                        y[11]	= (X[2]*x[1] + 0.5*Wbo[2]) * dt;
                        y[15]	= 1.0 + X[3]*x[1]*dt;

                        M_Product( 4, 4, 3, y, P2, K );
                        /*	K	=	F44*P43					*/
                        FPF( 4, 4, y, P1, HP );	/*	HP	=	F44*P44*t(F44)			*/

                        /*====================================
                          d(qtn-dot)/d(w)*dt
                          ====================================*/

                        x[1]	=  0.5*dt;

                        y[0]	=  X[3] * x[1];
                        y[3]	=  X[2] * x[1];
                        y[6]	= -X[1] * x[1];
                        y[9]	= -X[0] * x[1];

                        y[1]	= -X[2] * x[1];
                        y[4]	=  X[3] * x[1];
                        y[7]	=  X[0] * x[1];
                        y[10]	= -X[1] * x[1];

                        y[2]	=  X[1] * x[1];
                        y[5]	= -X[0] * x[1];
                        y[8]	=  X[3] * x[1];
                        y[11]	= -X[2] * x[1];

                        M_Product( 4, 3, 3, y, P3, P2 );
                        for( i = 0; i < 12; i++ )	P2[i]	+= K[i];
                        /*	P2	=	F44*P43	+ F43*P33		*/
                        M_Transpose( 4, 3, P2, x );
                        M_Product( 4, 3, 4, y, x, P1 );
                        for( i = 0; i < 16; i++ )	HP[i]	+= P1[i];
                        /*	HP	=	F44*P44*F44	+ F43*t(P1)	*/
                        M_Transpose( 4, 3, y, x );
                        M_Product( 4, 3, 4, K, x, P1 );
                        for( i = 0; i < 16; i++ )	P1[i]	+= HP[i];
                        /*	P1	=	^P1	propagated !		*/

                        /*====================================
                          1 +	d(w-dot)/d(w)*dt
                          ====================================*/

                        y[0]	= 1.0;
                        y[1]	= (-lyz*X[6])*dt;
                        y[2]	= (-lyz*X[5] + J_MW*MW_RPS/Ix)*dt;

                        y[3]	= (-lzx*X[6])*dt;
                        y[4]	= 1.0;
                        y[5]	= (-lzx*X[4])*dt;

                        y[6]	= (-lxy*X[5] - J_MW*MW_RPS/Iz)*dt;
                        y[7]	= (-lxy*X[4])*dt;
                        y[8]	= 1.0;

                        FPF( 3, 3, y, P3, x );	/*	x	=	^P3	propagated			*/
                        for( i = 0; i < 9; i++ )	P3[i]	= x[i];

                        M_Transpose( 3, 3, y, x );
                        M_Product( 4, 3, 3, P2, x, y );
                        for( i = 0; i < 12; i++ )	P2[i]	= y[i];
                        /*	P2	=	^P2	propagated !		*/

                        x[0]	= X[0] * X[0];
                        x[1]	= X[1] * X[1];
                        x[2]	= X[2] * X[2];
                        x[3]	= X[3] * X[3];
                        x[4]	= dt * dt * dt / 12.0;

                        P1[0]	+= (x[3]*Q2[0] + x[2]*Q2[1] + x[1]*Q2[2]) * x[4];
                        P1[5]	+= (x[2]*Q2[0] + x[3]*Q2[1] + x[0]*Q2[2]) * x[4];
                        P1[10]	+= (x[1]*Q2[0] + x[0]*Q2[1] + x[3]*Q2[2]) * x[4];
                        P1[15]	+= (x[0]*Q2[0] + x[1]*Q2[1] + x[2]*Q2[2]) * x[4];
                        P3[0]	+= Q2[0]*dt;
                        P3[4]	+= Q2[1]*dt;
                        P3[8]	+= Q2[2]*dt;
                        /*====================================
                          add	process	noise matrix
                          diagonal terms only
                          ====================================*/
                        break;
                case Sun_Filter:
                        /*************************************
                          quaternion filter entry

note:
for	the	covariance propagation
the	effect of gravity gradient
is ignored in order	to save
the	complexity
                         **************************************/

                        /*====================================
                          1 +	d(qtn-dot)/d(qtn)*dt
                          ====================================*/
                        x[1]	= X[2]*W0;

                        y[0]	= 1.0 + X[0]*x[1]*dt;
                        y[4]	= (X[1]*x[1] - 0.5*Wbo[2]) * dt;
                        y[8]	= (X[2]*x[1] + 0.5*Wbo[1] - W0) * dt;
                        y[12]	= (X[3]*x[1] - 0.5*Wbo[0]) * dt;

                        x[1]	= X[3]*W0;

                        y[1]	= (X[0]*x[1] + 0.5*Wbo[2]) * dt;
                        y[5]	= 1.0	+ X[1]*x[1]*dt;
                        y[9]	= (X[2]*x[1] - 0.5*Wbo[0]) * dt;
                        y[13]	= (X[3]*x[1] - 0.5*Wbo[1] - W0) * dt;

                        x[1]	= -X[0]*W0;

                        y[2]	= (X[0]*x[1] - 0.5*Wbo[1] + W0) * dt;
                        y[6]	= (X[1]*x[1] + 0.5*Wbo[0]) * dt;
                        y[10]	= 1.0 + X[2]*x[1]*dt;
                        y[14]	= (X[3]*x[1] - 0.5*Wbo[2]) * dt;

                        x[1]	= -X[1]*W0;

                        y[3]	= (X[0]*x[1] + 0.5*Wbo[0]) * dt;
                        y[7]	= (X[1]*x[1] + 0.5*Wbo[1] + W0) * dt;
                        y[11]	= (X[2]*x[1] + 0.5*Wbo[2]) * dt;
                        y[15]	= 1.0 + X[3]*x[1]*dt;

                        M_Product( 4, 4, 3, y, P2, K );
                        /*	K	=	F44*P43					*/
                        FPF( 4, 4, y, P1, HP );	/*	HP	=	F44*P44*t(F44)			*/

                        /*====================================
                          d(qtn-dot)/d(w)*dt
                          ====================================*/

                        x[1]	=  0.5*dt;

                        y[0]	=  X[3] * x[1];
                        y[3]	=  X[2] * x[1];
                        y[6]	= -X[1] * x[1];
                        y[9]	= -X[0] * x[1];

                        y[1]	= -X[2] * x[1];
                        y[4]	=  X[3] * x[1];
                        y[7]	=  X[0] * x[1];
                        y[10]	= -X[1] * x[1];

                        y[2]	=  X[1] * x[1];
                        y[5]	= -X[0] * x[1];
                        y[8]	=  X[3] * x[1];
                        y[11]	= -X[2] * x[1];

                        M_Product( 4, 3, 3, y, P3, P2 );
                        for( i = 0; i < 12; i++ )	P2[i]	+= K[i];
                        /*	P2	=	F44*P43	+ F43*P33		*/
                        M_Transpose( 4, 3, P2, x );
                        M_Product( 4, 3, 4, y, x, P1 );
                        for( i = 0; i < 16; i++ )	HP[i]	+= P1[i];
                        /*	HP	=	F44*P44*F44	+ F43*t(P1)	*/
                        M_Transpose( 4, 3, y, x );
                        M_Product( 4, 3, 4, K, x, P1 );
                        for( i = 0; i < 16; i++ )	P1[i]	+= HP[i];
                        /*	P1	=	^P1	propagated !		*/

                        /*====================================
                          1 +	d(w-dot)/d(w)*dt
                          ====================================*/

                        y[0]	= 1.0;
                        y[1]	= (-lyz*X[6])*dt;
                        y[2]	= (-lyz*X[5] + J_MW*MW_RPS/Ix)*dt;

                        y[3]	= (-lzx*X[6])*dt;
                        y[4]	= 1.0;
                        y[5]	= (-lzx*X[4])*dt;

                        y[6]	= (-lxy*X[5] - J_MW*MW_RPS/Iz)*dt;
                        y[7]	= (-lxy*X[4])*dt;
                        y[8]	= 1.0;

                        FPF( 3, 3, y, P3, x );	/*	x	=	^P3	propagated			*/
                        for( i = 0; i < 9; i++ )	P3[i]	= x[i];

                        M_Transpose( 3, 3, y, x );
                        M_Product( 4, 3, 3, P2, x, y );
                        for( i = 0; i < 12; i++ )	P2[i]	= y[i];
                        /*	P2	=	^P2	propagated !		*/

                        x[0]	= X[0] * X[0];
                        x[1]	= X[1] * X[1];
                        x[2]	= X[2] * X[2];
                        x[3]	= X[3] * X[3];
                        x[4]	= dt * dt * dt / 12.0;

                        P1[0]	+= (x[3]*Q2[0] + x[2]*Q2[1] + x[1]*Q2[2]) * x[4];
                        P1[5]	+= (x[2]*Q2[0] + x[3]*Q2[1] + x[0]*Q2[2]) * x[4];
                        P1[10]	+= (x[1]*Q2[0] + x[0]*Q2[1] + x[3]*Q2[2]) * x[4];
                        P1[15]	+= (x[0]*Q2[0] + x[1]*Q2[1] + x[2]*Q2[2]) * x[4];
                        P3[0]	+= Q2[0]*dt;
                        P3[4]	+= Q2[1]*dt;
                        P3[8]	+= Q2[2]*dt;
                        /*====================================
                          add	process	noise matrix
                          diagonal terms only
                          ====================================*/
                        break;

        } /* end of switch( Filter ) */

} /* void P_prop */



/*=============================================================================
function: MM_MalTLM
actions	: detect MM unhealty measurement
input	:
output	:
=============================================================================*/
void	MM_MalTLM( void )
{
        int		i;
        double	b[3];


        if( MM_Tlm16[0]*MM_Tlm16[1]*MM_Tlm16[2] == 0 )	{
                NoMMTlm	= ON;
                NoMMTlm_Counter  += 1;
        }
        else if( MM_Tlm16[0]==MM_Tlm16Prev[0] && MM_Tlm16[1]==MM_Tlm16Prev[1]
                        && MM_Tlm16[2]==MM_Tlm16Prev[2] )
        {
                NoMMTlm	= ON;
                NoMMTlm_Counter  += 1;
        }
        else
                NoMMTlm	= OFF;
        /*==================================
          this section need to be	altered
          whenever the thsat orbit is
          comfirmed
          ==================================*/

        if( NoMMTlm == OFF )	{
                for( i=0; i<3; i++ )	{
                        b[i] = MM_Tlm16[i];
                }
                /*	magnav calibration				*/
                b[0]	= sqrt (b[0]*b[0] +	b[1]*b[1] +	b[2]*b[2]);
                /*	measured field strength			*/

                if( (b[0]<BfMM_Min) || (b[0]>BfMM_Max) )	{
                        NoMMTlm	= ON;
                        NoMMTlm_Counter  += 1;
                }
                /*==================================
                  Caution	!
                  should not do strength-check in
                  bf_measured	function in	order
                  to save	the	computational
                  demand ! could be Dangerous	!
                  ==================================*/
        } /* if( NoMMTlm == OFF ) */

        MM_Tlm16Prev[0]	= MM_Tlm16[0];
        MM_Tlm16Prev[1]	= MM_Tlm16[1];
        MM_Tlm16Prev[2]	= MM_Tlm16[2];

} /* void MM_MalTLM */



/*=============================================================================
function: BF_Measured
actions	: recover original magnetometer	measurement
input	:
output	:
=============================================================================*/
void	BF_Measured( void )
{
        /*====================================================
          Caution	!!!

          if unauthorised	mm_sel (has	to be 1, 2)	be given,
          then atacq may crash obc !
          ==================================================*/
        int		i;


        if( NoMMTlm==OFF )	{
                for( i=0; i<3; i++ )	{
                        BfMM_prev[i] = BfMM[i];
                        BfMM[i]		 = ( (double)MM_Tlm16[i] ) * 250.0 / 8192.0;
                }	/* MM calibration */
                BfMM_prev[3] = BfMM[3];
                BfMM[3] = sqrt( BfMM[0]*BfMM[0] + BfMM[1]*BfMM[1] + BfMM[2]*BfMM[2] );
        }

}	/*	void	BF_Measured	*/

/*=============================================================================
function: mgps2mjd
actions : convert from leaped gps time to modified julian date
Gps seconds of the week is modified by adding leaping seconds (which meeans
Mgps time is the same as the Utc time);
author  : Li Xin Feng
date    : 11/19/003
input	: weeks (unsigned int 2 BYTE)
return	: corresponding mjd
=============================================================================*/
double	mgps2mjd( unsigned int weeks,unsigned long seconds )
{
        double	dates,sec_of_day,local_mjd;

        dates    = floor( seconds / 86400.0 );

        sec_of_day = seconds - dates*86400;

        local_mjd = (double)(weeks)*7.0 + dates + GPS_ZERO_MJD + (sec_of_day/86400);

        return( local_mjd );
} /* double mgps2mjd() */
/*=============================================================================
function: sgp
actions	: norad	recommended	orbit propagator
input	:	dt	:	time since norad epoch
output	:	r	:	pointer	to position	vector
v	:	pointer	to velocity	vector
=============================================================================*/
int     Sgp( double dt, double *x, double *v )
{
        /*=======================================================================
          SGP was developed by Hilton & Kuhlman (1966) and it used for the work
          of Kozai (1959) for its gravitational model and it takes the drag effect
          on mean motion as linear in time. This assumption dictates a quadratic
          variation of mean anomaly with time. The drag effect on eccentricity is
          modeled in such a way that perigee height remains constant.
          ========================================================================*/
        /*************************************
note:
orbit parameters will be set
into global structure
         *************************************/

        double  a;                          /*  Semi-major Axis                 */
        double  e;                          /*  Eccentricity                    */
        double  p;                          /*  a*(1.0 - e*e)                   */
        double  anSCL;                      /*  Ascending Node Secular Effect   */
        double  apSCL;                      /*  Argment of Perigee Secular Eff. */
        double  Ls;                         /*  M + ap + an : Secular Effect    */
        double  axNSL;                      /*  Long-period Periodics           */
        double  ayNSL;                      /*  Long-period Periodics           */
        double  L;                          /*  M + ap + an : Long-period Effect*/
        double  U;                          /*  E + ap Epoch Value              */
        double  EpW;                        /*  E + ap (i+1)-th Value           */
        double  EpWprev;                    /*  E + ap i-th Value               */
        double  dEpW;                       /*  d(E + ap)                       */
        double  ecosE;                      /*  e*cos(eccentric_anomaly)        */
        double  esinE;                      /*  e*sin(eccentric_anomaly)        */
        double  eLSQ;                       /*  Eccentricity Square : Long-peri */
        double  PL;                         /*  a*(1 - e*e) : Long-period       */
        double  r;                          /*  magnitude of Position Vector    */
        double  rdot;                       /*  dr/dt                           */
        double  rvdot;                      /*  r*dv/dt                         */
        double  sinu;                       /*  sin(true_anomaly)               */
        double  cosu;                       /*  cos(true_anomaly)               */
        double  u;                          /*  True Anomaly                    */
        double  rk;                         /*  Final Result : Position VctMag  */
        double  uk;                         /*  Final Result : True Anomaly     */
        double  ank;                        /*  Final Result : Ascending Node   */
        double  ik;                         /*  Final Result : Inclination      */
        double  ux;                         /*  Unit Orientation Vector Comps.  */
        double  uy;                         /*  Unit Orientation Vector Comps.  */
        double  uz;                         /*  Unit Orientation Vector Comps.  */
        double  vx;                         /*  Unit Orientation Vector Comps.  */
        double  vy;                         /*  Unit Orientation Vector Comps.  */
        double  vz;                         /*  Unit Orientation Vector Comps.  */
        double  dt2;                        /*  time_since_epoch Parameters     */
        double  dt3;                        /*  time_since_epoch Parameters     */
        double  sini0;                      /*  sin(i0)                         */
        double  cosi0;                      /*  cos(i0)                         */
        int     loop;                       /*  Loop Counter In Kepler's Eq     */

        /* ----------------------------------------------------------------------- */
        /*******  Start Procedure *************************************************/

        /**************************************
          The secular effects of atmospheric
          drag and gravitation are included
          through the equations
         **************************************/

        dt2 =   dt*dt;
        dt3 =   dt*dt2;

        a   =   Eph.n0/(Eph.n0 + 2.0*Eph.n0dot*dt + 3.0*Eph.n0dot2*dt2);
        a   =   Eph.a0*pow(a, 2.0/3.0);

        if (a > Eph.q0)     e   =   1.0 - Eph.q0/a;
        else                e   =   1.e-6;

        p   =   a*(1.0 - e*e);

        anSCL   =   Eph.an0 + Eph.dan*dt;
        apSCL   =   Eph.ap0 + Eph.dap*dt;
        Ls      =   Eph.L0  + (Eph.n0 + Eph.dap + Eph.dan)*dt
                + Eph.n0dot*dt2 + Eph.n0dot2*dt3;

        Ls      =   Angle_Normaliser (Ls);
        if (Ls < 0.0)       Ls  +=  TwoPI;
        /**************************************
          Long-period periodics are included
          through the equations
         **************************************/
        sini0   =   sin(Eph.i0);
        cosi0   =   cos(Eph.i0);

        axNSL   =   e*cos(apSCL);
        ayNSL   =   e*sin(apSCL) - 0.5*Eph.A30*sini0/p;

        L       =   (3.0 + 5.0*cosi0)/(1.0 + cosi0)*sini0;
        L       =   Ls - 0.25*Eph.A30*axNSL*L/p;

        L       =   Angle_Normaliser (L);
        if (L < 0.0)        L   +=  TwoPI;
        /************************************
          Solve Kepler's equation for E + ap
          (by iteration to the desired
          accuracy)
         ************************************/

        loop    =   0;
        U       =   L - anSCL;

        U       =   Angle_Normaliser (U);
        if (U < 0.0)        U   +=  TwoPI;

        EpW     =   U;

        do
        {
                EpWprev =   EpW;
                dEpW    =   U - ayNSL*cos(EpWprev) + axNSL*sin(EpWprev) - EpWprev;
                dEpW    /=  -ayNSL*sin(EpWprev) - axNSL*cos(EpWprev) + 1.0;
                EpW     +=  dEpW;
                loop    +=  1;
                if (loop == 50)                     return (3);
        }   while (fabs(EpW - EpWprev) > MICRO);
        /************************************
          Then calculate the intermediate
          (partially osculating) quantities
         *************************************/

        ecosE   =   axNSL*cos(EpW) + ayNSL*sin(EpW);
        esinE   =   axNSL*sin(EpW) - ayNSL*cos(EpW);

        eLSQ    =   axNSL*axNSL + ayNSL*ayNSL;
        PL      =   a*(1.0 - eLSQ);
        r       =   a*(1.0 - ecosE);
        rdot    =   Eph.ke*sqrt(a)*esinE/r;
        rvdot   =   Eph.ke*sqrt(PL)/r;

        sinu    =   sin(EpW) - ayNSL - axNSL*esinE/(1.0 + sqrt(1.0 - eLSQ));
        sinu    *=  a/r;

        cosu    =   cos(EpW) - axNSL + ayNSL*esinE/(1.0 + sqrt(1.0 - eLSQ));
        cosu    *=  a/r;

        u       =   atan2 (sinu, cosu);
        if (u < 0.0)        u   +=  TwoPI;

        /************************************
          Short-period perturbations are
          now induced by
         *************************************/
        sinu    =   sin(2.0*u);
        cosu    =   cos(2.0*u);

        rk      =   r + 0.5*Eph.K2*sini0*sini0*cosu/PL;
        uk      =   u - 0.25*Eph.K2*(7.0*cosi0*cosi0 - 1.0)*sinu/PL/PL;
        ank     =   anSCL + 1.5*Eph.K2*cosi0*sinu/PL/PL;
        ik      =   Eph.i0 + 1.5*Eph.K2*sini0*cosi0*cosu/PL/PL;

        /************************************
          Then unit orientation vectors are
          calculated by
         *************************************/
        sini0   =   sin(ik);
        cosi0   =   cos(ik);

        sinu    =   sin(uk);
        cosu    =   cos(uk);

        esinE   =   sin(ank);
        ecosE   =   cos(ank);


        ux      =  -esinE*cosi0*sinu + ecosE*cosu;
        uy      =   ecosE*cosi0*sinu + esinE*cosu;
        uz      =   sini0*sinu;

        vx      =  -esinE*cosi0*cosu - ecosE*sinu;
        vy      =   ecosE*cosi0*cosu - esinE*sinu;
        vz      =   sini0*cosu;

        /************************************
          Then position and velocity are
          given by
         *************************************/
        x[0]    =   rk*ux;
        x[1]    =   rk*uy;
        x[2]    =   rk*uz;

        v[0]    =   rdot*ux + rvdot*vx;
        v[1]    =   rdot*uy + rvdot*vy;
        v[2]    =   rdot*uz + rvdot*vz;

        R_ECEF[0] = x[0];
        R_ECEF[1] = x[1];
        R_ECEF[2] = x[2];
        V_ECI[0]  = v[0];
        V_ECI[1]  = v[1];
        V_ECI[2]  = v[2];


        return( 0 );
} /* int sgp */
/***************************************************************************
function: Sun
actions : compute solar vector
author  : capt dave vallado USAFA/DFAS 25 aug 1988
converted from FORTRAN code by un poulet erotique
date    : 23/11/93
input	:	mjd	-> modified julian date in utc
output	:	r	-> solar unit position vector with respect to eci
 ****************************************************************************/
void    Sun( double mjd, double *r )
{
        /*************************************
          This Subroutine calculates the
          Geocentric Equatorial position
          vector for the Sun given the
          Julian Date. This is the low
          precision formula and is valid for
          years from 1950 to 2050. Accuracy
          of apparent coordinates is 0.01
          degrees.
         *************************************/

        double  N           =   telaps(mjd);
        /*  N  = JD - 2451545.0             */
        /*  JD = MJD + 2400000.5            */
        double  EclpLong    =   280.460 + 0.9856474*N;
        double  MeanAnomaly =   357.528 + 0.9856003*N;
        double  Obliquity;
        double  D2R         =   TwoPI/360.0;

        EclpLong    =   fmod (EclpLong, 360.0);

        MeanAnomaly =   fmod (MeanAnomaly*D2R, TwoPI);
        if (MeanAnomaly < 0.0)  MeanAnomaly +=  TwoPI;

        EclpLong    +=  1.915*sin(MeanAnomaly) + 0.020*sin(2.0*MeanAnomaly);
        Obliquity   =   23.439 - 0.0000004*N;

        EclpLong    *=  D2R;
        Obliquity   *=  D2R;

        if (EclpLong < 0.0)     EclpLong    +=  TwoPI;

        r[0]    =   cos(EclpLong);
        r[1]    =   cos(Obliquity)*sin(EclpLong);
        r[2]    =   sin(Obliquity)*sin(EclpLong);

} /* void Sun */



/***************************************************************************
function: eclipse
actions	: detect eclipse or	sunlit
author	: un poulet	erotique
date	: 25/07/97
 ****************************************************************************/
int		eclipse( double *r, double *s )
{
        /*==================================================================
          input
r		:	satellite position vector w.r.t. eci
s		:	solar unit vector w.r.t. eci
return
eclipse/sunlit flag
====================================================================*/

        double	r2;						/*	norm of	uosat position vector	*/
        double	d1;
        double	d2;						/*	eclipse/sunlit determinators	*/

        r2	=	r[0]*r[0] +	r[1]*r[1] +	r[2]*r[2];

        d1	=	r[0]*s[0] +	r[1]*s[1] +	r[2]*s[2];
        /*	eclipse/sunlit determinator	1	*/
        d2	=	sqrt (r2 - d1*d1);
        /*	eclipse/sunlit determinator	2	*/
        if( (d1<0.0) && (d2<AE) )		return( 1 );

        return( 0 );
} /* int eclipse */



/***************************************************************************
function: orbit
actions : compute orbital parameters and transformation
original author  : un poulet erotique
original date    : 26/05/99
revising author  : Li Xinfeng
revising date    : 24/10/03
output			 :	r	-> satellite position vector w.r.t. eci
z	-> transformation from eci to local orbit
 ****************************************************************************/
int	Orbit( double *r, double *z	)
{
        double	t1,	t2;
        double	v[3];
        int		err;

        /************************************
          compute	current	orbit
         ************************************/
        Mjd	= mgps2mjd( gps_weeks_ADCS, gps_seconds_ADCS );

        t1	= (Mjd - Eph.l2epoc) * 86400.0;
        /*	sgp	propagator i/o				*/

        err	= Sgp (t1, r, v);
        if( err != 0 )		return( err );

        t1	= sqrt( r[0]*r[0] + r[1]*r[1] + r[2]*r[2] );
        if( t1 < ZERO )		return( 4 );
        /*	zero position vector given		*/
        /************************************
          compute	transformation matrix
          from eci to	local orbit	(T)
         ************************************/
        z[6]	= -r[0] / t1;
        z[7]	= -r[1] / t1;
        z[8]	= -r[2] / t1;
        /*	3rd	column vector derived		*/
        z[3]	= r[2]*v[1] - r[1]*v[2];
        z[4]	= r[0]*v[2] - r[2]*v[0];
        z[5]	= r[1]*v[0] - r[0]*v[1];
        /*	anti-orbit normal vector		*/
        t2		= sqrt( z[3]*z[3] + z[4]*z[4] + z[5]*z[5] );
        if (t2 < ZERO)		return( 4 );
        /*	norm of	orbit normal vector		*/
        z[3]   /= t2;
        z[4]   /= t2;
        z[5]   /= t2;
        /*	2nd	column vector derived		*/
        z[0]	= z[4]*z[8] - z[5]*z[7];
        z[1]	= z[5]*z[6] - z[3]*z[8];
        z[2]	= z[3]*z[7] - z[4]*z[6];
        /*	1st	column vector derived		*/

        GG	= Eph.ke / sqrt(t1) / t1;	/*	gravity	gradient coefficient	*/
        W0	= t2 / t1 / t1;				/*	orbital	rate					*/
        /*==================================
          df/dt =	h/(r^2)	model is used
          ==================================*/
        //	Sun (Mjd, Sun_i);
        //	LXF_shade = eclipse (r, SUN_V);
        LXF_shade =0;
        return( 0 );
} /* int Orbit */
void  igrf_b_field(double *gh,double *r,double *bb)
{
        double	AA[(NMAX+1)*(NMAX+2)/2],CC[NMAX+1],SS[NMAX+1];
        double	R,x,y,z,u;
        double	aRn2,Am1,Part1,Part2,Part3,Part4,gnm,hnm,prod,Cm1,Sm1;
        int		n,m,i,j,k,igh;


        R	= sqrt( r[0]*r[0] + r[1]*r[1] + r[2]*r[2] );
        x	= r[0] / R;
        y	= r[1] / R;
        z	= r[2] / R;
        u	= z;	/* sin(lamda) */

        prod	= 1.0;
        AA[0]	= 1.0;
        CC[0]	= 1.0;
        SS[0]	= 0.0;
        aRn2	= AE * AE / R / R;

        i = j = k = 0;

        bb[0] = bb[1] = bb[2] = 0.0;

        for( n=1; n<=NMAX; n++ )	{

                aRn2 = aRn2 * (AE/R);
                SS[n] = y*CC[n-1] + x*SS[n-1];
                CC[n] = x*CC[n-1] - y*SS[n-1];

                k = j;
                j = i;
                i = i + n;

                prod *= (double)(2*n - 1);
                // index can use addition which have a better spead
                //i	=		n*(n+1)/2;  //index of AA[n][0] from ;
                //j=(n-1) *  n  /2;	    //index of AA[n-1][0];
                //k=(n-2) *(n-1)/2;	    //index of AA[n-2][0];
                // index of G[n][0]
                igh = n*n;

                AA[i+n]	 = prod;				//index of AA[n][n];
                AA[i+n-1] = u*prod;				//index of AA[n][n-1];

                for(m=0;m<=n-2;m++)	{	//index from AA[n][0] to AA[n][n-2]
                        AA[i+m] = ( (2*n-1)*u*AA[j+m] - ((double)(n+m-1))*AA[k+m] )
                                / (double)(n-m);
                }

                // these can not be engaged into one loop!
                for( m=0; m<=n; m++ )	{	//index from AA[n][0] to AA[n][n] if AA
                        if( m+1 > n )	Am1 = 0.0;
                        else			Am1 = AA[i+m+1];

                        if( m==0 )	{	//variable loaded
                                gnm = gh[igh+m];
                                hnm = 0;
                                Cm1 = 0;
                                Sm1 = 0;
                        }
                        else	{
                                gnm = gh[igh+2*m-1];
                                hnm = gh[igh+2*m]  ;
                                Cm1 = CC[m-1];
                                Sm1 = SS[m-1];
                        }

                        Part1 = gnm * CC[m]  +  hnm * SS[m];
                        Part2 = u * Am1 + ((double)(n+m+1)) * AA[i+m];
                        Part3 = gnm * Cm1 +  hnm * Sm1;
                        Part4 = hnm * Cm1 -  gnm * Sm1;

                        bb[0] += aRn2 * ( Part1*Part2*x - m*AA[i+m]*Part3 );	/* X */
                        bb[1] += aRn2 * ( Part1*Part2*y - m*AA[i+m]*Part4 );	/* Y */
                        bb[2] += aRn2 * ( Part1 * (Part2*z-Am1) );			/* Z */

                }/* end of for( m=0; m<=n; m++ ) */

        } /* end of for( n=1; n<=NMAX; n++ ) */

}


/***************************************************************************
function:
actions : predicted magnetometer measurement
author  : un poulet erotique
date    : 12/02/98
//-----------------------------------
revising auther  : Li Xinfeng
revising date    : 27/10/03

 ****************************************************************************/
void    BF_Predicted(double *r, double *z,double *bo)
{
        /*==================================================================
          input
r       :   satellite position vector  w.r.t. eci
z       :   transformation from eci to local-orbit
output
bo	   :	B field of local orbit, the last one is


====================================================================*/

        double  t1, t2;
        double  st;
        double  b[3];
        double  x[3];
        /************************************
          convert eci position vector
          to ecef position vector (v)
         ************************************/
        t2  =   Mjd - Eph.UTC2UT1;		//

        t1  =   (double)((long)t2);
        t2  =   t2 - t1;
        t1  =   telaps (t1);

        st  =   rotn (t1, t2, Eph.Eeqnox);

        t1  =   cos(st);
        t2  =   sin(st);

        x[0]    =   r[0]*t1 + r[1]*t2;
        x[1]    =  -r[0]*t2 + r[1]*t1;
        x[2]    =   r[2];
        /************************************
          compute earth magnetic field
          vector w.r.t. ecef (x)
         ************************************/
        igrf_b_field(ModGH, x, b);
        /************************************
          xfer b-field vector to eci (b)
         ************************************/
        x[0]    =   b[0]*t1 - b[1]*t2;
        x[1]    =   b[0]*t2 + b[1]*t1;
        x[2]    =   b[2];
        /************************************
          xfer b-field vector to lo
         ************************************/

        bo[0] = ( z[0]*x[0] + z[1]*x[1] + z[2]*x[2] ) * 0.001;
        bo[1] = ( z[3]*x[0] + z[4]*x[1] + z[5]*x[2] ) * 0.001;
        bo[2] = ( z[6]*x[0] + z[7]*x[1] + z[8]*x[2] ) * 0.001;

        bo[3] = sqrt( bo[0]*bo[0] + bo[1]*bo[1] + bo[2]*bo[2] );

}/* void BF_Predicted() */

/*=============================================================================
function: NoFilter
actions	: for just after separation	from launcher
input	: dt  :	  data sampling	time
output	:
=============================================================================*/
void	NoFilter( double dt )
{
        int		i;
        double	wy;


        for( i=0; i<3; i++ )	O[i] = BfMM[i] - BfMM_prev[i];

        if( P_counter >	Period_ )	{
                P_counter	= 0;
                C_pred[0] = C_pred[1]	= C_pred[2] = 0.0;
        }

        for( i=0; i<3; i++ )	C_pred[i] = Adc.G*O[i] +	(1.0-Adc.G)*C_pred[i];

        wy		= BfMM[0]*BfMM[0] +	BfMM[2]*BfMM[2];

        if( wy > ZERO )
                wy = ( BfMM[0]*C_pred[2] - BfMM[2]*C_pred[0] ) / wy / dt;

        X[5]	= Adc.G*wy + (1.0-Adc.G) * X[5];
        /*==================================
          very coase estimation of
          y axis spin	rate
          ==================================*/
        P_counter	+= (int) ( dt+0.5 );


} /* void NoFilter */



/*=============================================================================
function: RateFilter
actions	: estimate angular velocity	vector
input	: dt	:	measurement	sampling time
r		:	measurement	noise variance
return	: error	code
=============================================================================*/
int		RateFilter( double dt )
{
        int		i, j;
        double	Bx,	By,	Bz;				/*	temp. msmst	parameter			*/
        double	x[9];					/*	temp. 3	x 3	matrix buffer		*/
        double	D;						/*	matrix determinant				*/


        if( NoMMTlm == ON )	return( 0 );	/*	should not update the state	*/

        if( Filter_Sts%10 == OFF )	{
                X[4]	= 0.0;
                X[5]	= 0.0;
                X[6]	= 0.0;

                Filter_Sts	+= ON;

                return( 0 );
        }
        /************************************
          rate filter
          compute	HP matrix where	H is
          observation	matrix and P is
          covariance matrix
         ************************************/

        for( i=0; i<3; i++ )	O[i] = BfMM[i] - BfMM_prev[i];

        Bx	= BfMM_prev[0] * dt;
        By	= BfMM_prev[1] * dt;
        Bz	= BfMM_prev[2] * dt;

        HP[0]	= By*P0[2] - Bz*P0[1];
        HP[1]	= By*P0[4] - Bz*P0[3];
        HP[2]	= By*P0[5] - Bz*P0[4];

        HP[3]	= Bz*P0[0] - Bx*P0[2];
        HP[4]	= Bz*P0[1] - Bx*P0[4];
        HP[5]	= Bz*P0[2] - Bx*P0[5];

        HP[6]	= Bx*P0[1] - By*P0[0];
        HP[7]	= Bx*P0[3] - By*P0[1];
        HP[8]	= Bx*P0[4] - By*P0[2];

        /************************************
          rate filter
          compute	kalman gain	matrix K
         ************************************/
        for( i=0; i<3; i++ )	{
                j		= 3*i;

                K[j	 ]	= By*HP[j+2] - Bz*HP[j+1];
                K[j+1]	= Bz*HP[j  ] - Bx*HP[j+2];
                K[j+2]	= Bx*HP[j+1] - By*HP[j	];
        }

        K[0]	+= R1[0];
        K[4]	+= R1[1];
        K[8]	+= R1[2];

        D	= M_Inverse3x3( K, x );
        if( fabs(D) < ZERO )	return (5);

        for( i=0; i<3; i++ )	{
                for( j=0; j<3; j++ )	{
                        K[3*i+j]  = HP[i]*x[j] + HP[i+3]*x[j+3] + HP[i+6]*x[j+6];
                        K[3*i+j] /= D;
                }
        }
        /************************************
          rate filter
          update state vector
         ************************************/
        C_pred[0]	= By*X[6] -	Bz*X[5];
        C_pred[1]	= Bz*X[4] -	Bx*X[6];
        C_pred[2]	= Bx*X[5] -	By*X[4];	/*	this is	predicted measurement	*/

        for( i=0; i<3; i++ )	x[i] = O[i]	- C_pred[i];
        /*	this is	so called o-c vector	*/
        for( i=0; i<3; i++ )	{
                j	= 3*i;
                X[i+4] += K[j]*x[0]	+ K[j+1]*x[1] +	K[j+2]*x[2];
        }
        /************************************
          rate filter
          update covariance matrix
         ************************************/
        for( i=0; i<3; i++ )	{
                j	= 3*i;

                x[j	 ]	= K[j+2]*By	- K[j+1]*Bz;
                x[j+1]	= K[j  ]*Bz	- K[j+2]*Bx;
                x[j+2]	= K[j+1]*Bx	- K[j  ]*By;
        }

        x[0]   += 1.0;
        x[4]   += 1.0;
        x[8]   += 1.0;					/*	x =	I -	KH matrix				*/

        P0[0]	= x[0]*P0[0] + x[1]*P0[1] +	x[2]*P0[2];
        Bx		= P0[1];				/*	push original P0[1]	value		*/
        P0[1]	= x[0]*P0[1] + x[1]*P0[3]	+ x[2]*P0[4];
        P0[3]	= x[3]*Bx +	x[4]*P0[3] + x[5]*P0[4];
        Bx		= P0[2];				/*	push original P0[2]	value		*/
        P0[2]	= x[0]*P0[2] + x[1]*P0[4] +	x[2]*P0[5];
        By		= P0[4];				/*	push original P0[4]	value		*/
        P0[4]	= x[3]*Bx +	x[4]*P0[4] + x[5]*P0[5];
        P0[5]	= x[6]*Bx +	x[7]*By	+ x[8]*P0[5];


        return( 0 );
} /* int ratefilter */
/*=============================================================================
function: PitchFilter
actions	: estimate pitch and pitch rate
input	: dt  :	  measurement sampling time
Ny  :	  y-axis torque	vector component induced by	mt
return	: error	code
=============================================================================*/
void	PitchFilter( double dt,	double Ny )
{

        double	theta_err;				/*	pitch angle	error				*/
        double	Wy_prev;				/*	previous wy	estimation			*/
        double	delta;					/*	pitch measurement update		*/
        double	sinp, cosp;


        sinp	= BfMM[2]*BfLO[0] -	BfMM[0]*BfLO[2];
        cosp	= BfMM[0]*BfLO[0] +	BfMM[2]*BfLO[2];

        if( (Filter_Sts%100)/10 == OFF && NoMMTlm == OFF )	{
                Y[0] = atan2	(sinp, cosp);
                Y[1] = 0.0;

                if( Filter_Sts%10 == ON )	Y[1] = X[5];

                Filter_Sts	+= 10*ON;
        }
        else	{
                Wy_prev	= Y[1];
                Y[1]   += (Ny -	J_MW*dMW_RPS) *	dt / Iy;
                Y[0]   += 0.5*(Y[1]	+ Wy_prev) * dt;

                if( NoMMTlm == OFF )	{
                        delta		= atan2( sinp, cosp ) - Y[0];
                        theta_err	= Angle_Normaliser( delta );
                        Y[1]  += Adc.G2 * theta_err;
                        Y[0]  += Adc.G1 * theta_err;
                }						/* if can is healthy, then update state	*/

                Y[0]  = Angle_Normaliser( Y[0] );

                /*
                   Y0=Y[0];
                   Y1=Y[1];
                ////printf("1000Y0=%d \n",(int)(Y0*1000));
                ////printf("1000Y1=%d \n",(int)(Y1*1000));
                */

        }

        /*	coarse pitch estimation	at epoch*/
        /************************************
          y-thomson attitude state at
          an epoch is	assumed	to derive
          this coarse	pitch estimation
          if not,	may	not	work
         ************************************/
} /* void PitchFilter */
/*=============================================================================
function: OakFilter
actions	: estimate full	attitude state
input	:	s	:	attitude sensor	identification
o	:	measured vector	with respect to	body
c	:	modelled vector	with respect to	local orbit
r	:	measurement	noise variance
return	: error	code
=============================================================================*/
int		OakFilter( double *o, double *c, double *r, double *so, double *sc)
{
        double	x[12], y[12], z[12],sx[12];
        double	D;						/*	matrix determinant				*/
        int		i;

        /************************************
          roll/yaw filter
          compute	HP matrix where	H is
          observation	matrix and P is
          covariance matrix
         ************************************/

        if( NoMMTlm == ON )		return( 0 );
        /*	should not update the state		*/


        for( i=0; i<3; i++ )	x[i] = 2.0 * c[i] /	c[3];

        z[0]	= X[0]*x[0]	+ X[1]*x[1]	+ X[2]*x[2];
        z[4]	= X[1]*x[0]	- X[0]*x[1]	+ X[3]*x[2];
        z[8]	= X[2]*x[0]	- X[3]*x[1]	- X[0]*x[2];

        z[1]	=-X[1]*x[0]	+ X[0]*x[1]	- X[3]*x[2];
        z[5]	= z[0];
        z[9]	= X[3]*x[0]	+ X[2]*x[1]	- X[1]*x[2];


        z[2]	= -z[8];
        z[6]	= -z[9];
        z[10]	=  z[0];

        z[3]	=  z[9];
        z[7]	= -z[8];
        z[11]	=  z[4];
        /*==================================
z: observation matrix 3x4
non-trivial sub-matrix
==================================*/
        FPF( 4, 3, z, P1, x );			/*	x	=	H34*P44*t(H34)			*/

        x[0]	+= r[0];
        x[4]	+= r[1];
        x[8]	+= r[2];


        D	= M_Inverse3x3( x, y );		/*	y: HPHt	+ R	inverse				*/

        if( fabs(D) < ZERO )		return( 5 );

        M_Transpose( 3, 4, z, K );
        M_Transpose( 4, 3, P2, x );

        M_Product( 3, 4, 3,	x, K, HP );
        M_Product( 3, 3, 3,	HP,	y, x );

        M_Product( 4, 4, 3,	P1,	K, HP );
        M_Product( 4, 3, 3,	HP,	y, K );

        for( i=0; i<9 ; i++ )	K[i+12]	= x[i];
        for( i=0; i<21; i++ )	K[i]   /= D;

        M_Product( 3, 3, 1, BYLO, c, x );

        for( i=0; i<3; i++ )	{
                O[i] = o[i];
                C_pred[i] = x[i];
        }

        M_Product( 3, 3, 1, BYLO, sc, sx );//求得估计值



        /*==================================
x: predicted measurment, now
with	respect	to body-coord.
==================================*/

        for( i=0; i<3; i++ )	{x[i] = o[i]/o[3] - x[i]/c[3];}
        /*	x: o-c vector					*/
        M_Product( 7, 3, 1, K, x, HP );		/*	HP:	inovation vector			*/

        for( i=0; i<7; i++ )	X[i] +=	HP[i];
        /*	update state vector				*/
        Q_Normaliser( X );				/*	normalise quaternion			*/

        xBYLO( X, BYLO );				/*	recover	attitude matrix			*/

        Wbo[0]	= X[4] + BYLO[1]*W0;
        Wbo[1]	= X[5] + BYLO[4]*W0;
        Wbo[2]	= X[6] + BYLO[7]*W0;	/*	recover	orbit ref. velocity		*/

        Qtn2RPY( BYLO, RPY );
        /*	recover	euler angle				*/
        for( i=0; i<12; i++ )	y[i] = K[i];
        for( i=0; i< 9; i++ )	x[i] = K[i+12];
        /*		y =	K43	/ x	= K33			*/

        M_Product( 3, 4, 4,	z, P1, K );
        M_Product( 4, 3, 4,	y, K, HP );

        for( i=0; i<16; i++ )	P1[i] -= HP[i];

        M_Product( 3, 4, 3,	z, P2, K );
        M_Product( 4, 3, 3,	y, K, HP );

        for( i=0; i<12; i++ )	P2[i] -= HP[i];

        M_Product( 3, 3, 3, x, K, HP );

        for( i=0; i<9; i++ )	P3[i] -= HP[i];
        /*		P =	(I - KH)P				*/

        P_Normaliser( );
        /*==================================
          normalise covariance matrix:
          make sure this is symmetric
          ==================================*/
        return( 0 );
} /* int OakFilter */

int SunFilter( double *o, double *c,double *so, double *sc, double *r,double *sr,double *sv)//29
{
        double	x[12], y[12], z[12],P2T[12],H2[8],H1[12];
        double	sx[12], sy[12], sz[12],H1T[12],H2T[8];
        double  KK[35],INV_KK[25],TestKK[25];
        double  delt_D[2],DK;
        double	P2H1T[9],H1P1[12],H2P1[8],P1H1T[12],P1H2T[8],P2TH1T[9],P2TH2T[6];
        double  H1P1H1T[9],H1P1H2T[6],H2P1H1T[6],H2P1H2T[4];
        double  Sbx,Sby,Sbz;
        double  SKH1[16],SKH2[16],S1[16],SKH3[12],SKH4[12],S3[12],S1P1[16],S1P2[12],S3P2[9];
        int     i;
        /************************************
          roll/yaw filter
          compute	HP matrix
          where	H is observation	matrix
          and P is covariance matrix

         ************************************/

        if (NoMMTlm == ON )	    	return( 0 );
        if (LXF_shade==ON)          return( 0 );


        for( i=0; i<3; i++ )	x[i] = 2.0 * c[i] /	c[3];

        H1[0]	= X[0]*x[0]	+ X[1]*x[1]	+ X[2]*x[2];
        H1[4]	= X[1]*x[0]	- X[0]*x[1]	+ X[3]*x[2];
        H1[8]	= X[2]*x[0]	- X[3]*x[1]	- X[0]*x[2];

        H1[1]	=-X[1]*x[0]	+ X[0]*x[1]	- X[3]*x[2];
        H1[5]	= H1[0];
        H1[9]	= X[3]*x[0]	+ X[2]*x[1]	- X[1]*x[2];


        H1[2]	= -H1[8];
        H1[6]	= -H1[9];
        H1[10]	=  H1[0];

        H1[3]	=  H1[9];
        H1[7]	= -H1[8];
        H1[11]	=  H1[4];


        Sb = 2*sc[0]*(X[0]*X[1]-X[2]*X[3])+sc[1]*(-X[0]*X[0]+X[1]*X[1]-X[2]*X[2]+X[3]*X[3])+2*sc[2]*(X[1]*X[2]+X[0]*X[3]);//sv[1]; //
        if (fabs(Sb) < ZERO)  return( 0 );
        Sb =1/(Sb*Sb);
        Sbx = sc[0]*(X[0]*X[0]-X[1]*X[1]-X[2]*X[2]+X[3]*X[3])+2*sc[1]*(X[0]*X[1]+X[2]*X[3])+2*sc[2]*(X[0]*X[2]-X[1]*X[3]);//sv[0];//
        Sby = 2*sc[0]*(X[0]*X[1]-X[2]*X[3])+sc[1]*(-X[0]*X[0]+X[1]*X[1]-X[2]*X[2]+X[3]*X[3])+2*sc[2]*(X[1]*X[2]+X[0]*X[3]);//sv[1];//
        Sbz = 2*sc[0]*(X[0]*X[2]+X[1]*X[3])+2*sc[1]*(X[1]*X[2]-X[0]*X[3])+sc[2]*(-X[0]*X[0]-X[1]*X[1]+X[2]*X[2]+X[3]*X[3]);//sv[2];//

        for( i=0; i<3; i++ )	x[i] = 2.0 * sc[i] /sc[3];

        H2[0]	= -(X[0]*x[0]+X[1]*x[1]+X[2]*x[2])/Sby+Sbx*Sb*(X[1]*x[0]-X[0]*x[1]+X[3]*x[2]);
        H2[4]	= -(X[2]*x[0]-X[3]*x[1]-X[0]*x[2])/Sby+Sbz*Sb*(X[1]*x[0]-X[0]*x[1]+X[3]*x[2]);

        H2[1]	= -(-X[1]*x[0]+X[0]*x[1]-X[3]*x[2])/Sby+Sbx*Sb*(X[0]*x[0]+X[1]*x[1]+X[2]*x[2]);
        H2[5]	= -( X[3]*x[0]+X[2]*x[1]-X[1]*x[2])/Sby+Sbz*Sb*(X[0]*x[0]+X[1]*x[1]+X[2]*x[2]);


        H2[2]	= -(-X[2]*x[0]+X[3]*x[1]+X[0]*x[2])/Sby+Sbx*Sb*(-X[3]*x[0]-X[2]*x[1]+X[1]*x[2]);
        H2[6]	= -( X[0]*x[0]+X[1]*x[1]+X[2]*x[2])/Sby+Sbz*Sb*(-X[3]*x[0]-X[2]*x[1]+X[1]*x[2]);


        H2[3]	= -(X[3]*x[0]+X[2]*x[1]-X[1]*x[2])/Sby+Sbx*Sb*(-X[2]*x[0]+X[3]*x[1]+X[0]*x[2]);
        H2[7]	= -(X[1]*x[0]-X[0]*x[1]+X[3]*x[2])/Sby+Sbz*Sb*(-X[2]*x[0]+X[3]*x[1]+X[0]*x[2]);



        /*==================================
z: observation matrix 3x4
non-trivial sub-matrix
==================================*/
        FPF( 4, 3, H1, P1, H1P1H1T );
        /*	x	=	H34*P44*t(H34)			*/
        FPF( 4, 2, H2, P1, H2P1H2T );
        /*	sx	=	H34*P44*t(H34)			*/

        M_Transpose( 3, 4, H1, H1T );         // H1 Transpose**
        M_Transpose( 2, 4, H2, H2T );         // H2 Transpose**
        M_Transpose( 4, 3, P2, P2T );

        M_Product(4,4,3,P1,H1T,P1H1T);        // P1H1T
        M_Product(2,4,3,H2,P1H1T,H2P1H1T);    // H2P1H1T

        M_Product(4,4,2,P1,H2T,P1H2T);        // P1H2T
        M_Product(3,4,2,H1,P1H2T,H1P1H2T);    // H1P1H2T

        H1P1H1T[0]	+= r[0];
        H1P1H1T[4]	+= r[1];
        H1P1H1T[8]	+= r[2];

        H2P1H2T[0]  += sr[0];
        H2P1H2T[3]  += sr[1];

        for(i=0;i<3;i++) KK[i]  =H1P1H1T[i];
        for(i=3;i<6;i++) KK[i+2]=H1P1H1T[i];
        for(i=6;i<9;i++) KK[i+4]=H1P1H1T[i];

        for(i=0;i<2;i++) KK[i+3]=H1P1H2T[i];
        for(i=2;i<4;i++) KK[i+6]=H1P1H2T[i];
        for(i=4;i<6;i++) KK[i+9]=H1P1H2T[i];

        for(i=0;i<3;i++) KK[i+15]=H2P1H1T[i];
        for(i=3;i<6;i++) KK[i+17]=H2P1H1T[i];

        for(i=0;i<2;i++) KK[i+18]=H2P1H2T[i];
        for(i=2;i<4;i++) KK[i+21]=H2P1H2T[i];

        DK = M_Inverse5x5( KK, INV_KK );

        DD =DK;

        if(fabs(DK)<ZERO) return( 5 );

        for(i=0;i<25;i++) INV_KK[i]=INV_KK[i]/DK;

        M_Product( 5, 5, 5,	KK,INV_KK,TestKK);

        M_Product( 3, 4, 3,	P2T,H1T,P2TH1T);
        M_Product( 3, 4, 2,	P2T,H2T,P2TH2T);

        for(i=0;i<3;i++)  KK[i]   = P1H1T[i];
        for(i=3;i<6;i++)  KK[i+2] = P1H1T[i];
        for(i=6;i<9;i++)  KK[i+4] = P1H1T[i];
        for(i=9;i<12;i++) KK[i+6] = P1H1T[i];

        for(i=0;i<2;i++)  KK[i+3] = P1H2T[i];
        for(i=2;i<4;i++)  KK[i+6] = P1H2T[i];
        for(i=4;i<6;i++)  KK[i+9] = P1H2T[i];
        for(i=6;i<8;i++)  KK[i+12]= P1H2T[i];


        for(i=0;i<3;i++)  KK[i+20] = P2TH1T[i];
        for(i=3;i<6;i++)  KK[i+22] = P2TH1T[i];
        for(i=6;i<9;i++)  KK[i+24] = P2TH1T[i];

        for(i=0;i<2;i++)  KK[i+23] = P2TH2T[i];
        for(i=2;i<4;i++)  KK[i+26] = P2TH2T[i];
        for(i=4;i<6;i++)  KK[i+29] = P2TH2T[i];

        M_Product( 7, 5, 5,	KK,INV_KK,K); //K阵

        M_Product( 3, 3, 1, BYLO, c,   x );
        M_Product( 3, 3, 1, BYLO, sc, sx );//求得估计值

        delt_D[0]= atan(-sx[0]/sx[1]);
        delt_D[1]= atan(-sx[2]/sx[1]);



        /*==================================

x: predicted measurment, now

with	respect	to body-coord.
=================================*/

        for( i=0; i<3; i++ )	{x[i]  = o[i]/o[3] - x[i]/c[3];}          //Test[i]=x[i];}
        for( i=0; i<2; i++ )	{sx[i] = so[i] - delt_D[i];Test[i]=sx[i];}//dletZ


        /*	x: o-c vector					*/
        for( i=0; i<2; i++ )    x[i+3]=sx[i];       /*	HP:	inovation vector			*/


        M_Product( 7, 5, 1,	K,x,HP);

        for( i=0; i<7; i++ ) X[i] += HP[i];//更新状态


        /*	update state vector				*/
        Q_Normaliser( X );
        /*	normalise quaternion			*/

        xBYLO( X, BYLO );
        /*	recover	attitude matrix			*/

        Wbo[0]	= X[4] + BYLO[1]*W0;
        Wbo[1]	= X[5] + BYLO[4]*W0;
        Wbo[2]	= X[6] + BYLO[7]*W0;
        /*	recover	orbit ref. velocity		*/

        Qtn2RPY( BYLO, RPY );
        /*	recover	euler angle				*/



        for(i=0;i<3; i++)  SK1[i]=K[i];
        for(i=3;i<6; i++)  SK1[i]=K[i+2];
        for(i=6;i<9; i++)  SK1[i]=K[i+4];
        for(i=9;i<12;i++)  SK1[i]=K[i+6];

        for(i=0;i<2; i++)  SK2[i]=K[i+3];
        for(i=2;i<4; i++)  SK2[i]=K[i+6];
        for(i=4;i<6; i++)  SK2[i]=K[i+9];
        for(i=6;i<8; i++)  SK2[i]=K[i+12];


        for(i=0;i<3; i++)  SK3[i]=K[i+20];
        for(i=3;i<6; i++)  SK3[i]=K[i+22];
        for(i=6;i<9; i++)  SK3[i]=K[i+24];


        for(i=0;i<2; i++)  SK4[i]=K[i+23];
        for(i=2;i<4; i++)  SK4[i]=K[i+26];
        for(i=4;i<6; i++)  SK4[i]=K[i+29];



        M_Product(4,3,4,SK1,H1,SKH1);
        M_Product(4,2,4,SK2,H2,SKH2);
        for(i=0;i<16;i++)  S1[i]=SKH1[i]+SKH2[i];

        M_Product(3,3,4,SK3,H1,SKH3);
        M_Product(3,2,4,SK4,H2,SKH4);
        for(i=0;i<12;i++)  S3[i]=SKH3[i]+SKH4[i];


        M_Product(4,4,4,S1,P1,S1P1);
        for(i=0;i<16;i++)  P1[i] = P1[i] - S1P1[i];
        M_Product(4,4,3,S1,P2,S1P2);
        for(i=0;i<12;i++)  P2[i] = P2[i] - S1P2[i];
        M_Product(3,4,3,S3,P2,S3P2);
        for(i=0;i<9;i++)   P3[i] = P3[i] - S3P2[i];


        /*		P =	(I - KH)P				*/

        P_Normaliser( );

        /*==================================

          normalise covariance matrix:

          make sure this is symmetric
          ==================================*/
        SW[0] = SRPY[0] - Pre_SRPY[0];
        SW[1] = SRPY[1] - Pre_SRPY[1];
        SW[2] = SRPY[2] - Pre_SRPY[2];


        Pre_SRPY[0]=SRPY[0];
        Pre_SRPY[1]=SRPY[1];
        Pre_SRPY[2]=SRPY[2];

        return( 0 );
        }

/*=============================================================================
function: MT_FiringEffect
actions	: magnetorquer firing effect
input	: cmd  :   commanded magnetic moment induced by	torquer
output	: N	   :   averaged	torque vector induced by torquer
=============================================================================*/
void	MT_FiringEffect( double *cmd, double *N )
{
        N[0]	= ( cmd[1]*BfMM[2] - cmd[2]*BfMM[1] ) * MICRO;
        N[1]	= ( cmd[2]*BfMM[0] - cmd[0]*BfMM[2] ) * MICRO;
        N[2]	= ( cmd[0]*BfMM[1] - cmd[1]*BfMM[0] ) * MICRO;
}
/*=============================================================================
function: dBeta_Filter
actions	: filter d(beta)/dt
input	:	dt		:	data sampling time
beta	:	current	beta value
dbeta	:	revious	dbeta/dt value
return	: filtered current dbeta/dt	value
=============================================================================*/
double	dBeta_Filter( double dt, double beta, double dbeta )
{
        double	db;		   /*  delta beta  */


        db		= beta - Beta_prev;
        /*
           db		= atan2( sin(db), cos(db) );
           */
        dbeta	= Adc.G*db/dt +	(1.0-Adc.G)*dbeta;

        return( dbeta );
} /* double	dBeta_Filter */



/*=============================================================================
function: Cmd Limiter
actions	: magnetorquer firing command limiter
input	: cmd	  :	  original firing command
output	: cmd	  :	  limited firing command
=============================================================================*/
void	Cmd_Limiter( double *cmd )
{
        cmd[0]	= min(fabs(cmd[0]),	MT_Xmax*0.9) * Sign(cmd[0]);
        cmd[1]	= min(fabs(cmd[1]),	MT_Ymax*0.9) * Sign(cmd[1]);
        cmd[2]	= min(fabs(cmd[2]),	MT_Zmax*0.9) * Sign(cmd[2]);
}


/*=============================================================================
function: MT_Controller
actions	: magnetorquer control logic by	b-dot &	cpl
input	: dt	  :	  data sampling	time
output	: cmd	  :	  magnetorquer command
=============================================================================*/
void	MT_Controller( double dt, double *cmd )
{
        double	beta;					/*	current	beta angle				*/
        double	e[3];					/*	error vector buffer	for	cpl		*/
        double	w0 = Eph.n0;			/*	mean motion	in second			*/
        /*==================================
          this cannot	be replaced	by W0
          because	W0 is only available
          when KepSupport	is ON
          ==================================*/


        beta	= acos( BfMM[1] / BfMM[3] );
        dBeta	= dBeta_Filter( dt, beta, dBeta );

        /*==================================
          although p4	does not use
          dbeta/dt, it is	useful to
          keep this value	just in	case
          back to	another	control	mode
          ==================================*/

        switch( Control )	{
                case C1:					/*==================================
                                                                  de-libration only My is	used
                                                                  and	no estimator
                                                                  ==================================*/
                        cmd[0]	= 0.0;
                        cmd[1]	= Adc.Kd1*(dBeta/w0);
                        cmd[2]	= 0.0;

                        break;

                case C2:					/*==================================
                                                                  de-libration & y-spin control
                                                                  spin maintenace	by Mx
                                                                  rate filter	to Wy (Woy)
                                                                  ==================================*/
                        if( Filter == NO_FILTER )
                                e[0] = 0.0;
                        else if( Filter == RATE_FILTER || Filter >= OAK_FILTER )
                                e[0] = (X[5]-Adc.wy) / w0;
                        else
                                e[0] = (Y[1]-Adc.wy) / w0;


                        cmd[0]	= Adc.Ks1*e[0]*(double)Sign(BfMM[2]);
                        cmd[1]	= Adc.Kd1*(dBeta/w0);
                        cmd[2]	= 0.0;

                        break;

                case C3:					/*==================================
                                                                  de-libration & y-spin control
                                                                  spin maintenace	by Mz
                                                                  rate filter	to Wy (woy)
                                                                  this mode is to	verify the Mz
                                                                  polarity
                                                                  ==================================*/
                        if( Filter == NO_FILTER )
                                e[2] = 0.0;
                        else if( Filter == RATE_FILTER || Filter >= OAK_FILTER )
                                e[2] = (Adc.wy-X[5]) / w0;
                        else
                                e[2] = (Adc.wy-Y[1]) / w0;

                        cmd[0]	= 0.0;
                        cmd[1]	= Adc.Kd1*(dBeta/w0);
                        cmd[2]	= Adc.Ks1*e[2]*(double)Sign(BfMM[0]);

                        break;

                case C4:					/*==================================
                                                                  wheel spun-up sequence
                                                                  pitch &	oak-like estimator should
                                                                  both be	ON
                                                                  ==================================*/
                        if( MW_statusY == ON )	{
                                e[1]	= Adc.Kd[2]*J_MW*(MW_RPS - MW_RPS_Rating);

                                cmd[0]	=  e[1]	* BfMM[2] /	BfMM[3];
                                cmd[1]	=  0.0;
                                cmd[2]	= -e[1]	* BfMM[0] /	BfMM[3];
                        }
                        else	{
                                cmd[0]	= 0.0;
                                cmd[1]	= Adc.Kd1 *	( dBeta/w0 );
                                cmd[2]	= 0.0;
                        }

                        break;

                case C5:					/*==================================
                                                                  nutation damping & wheel
                                                                  de-saturation by cross product
                                                                  law, and pitch maintenace by
                                                                  pitch wheel
                                                                  oak-like filter	should be ON
                                                                  ==================================*/
                        e[0]	= Adc.Kd[0]*Wbo[0]/w0 +	Adc.Kp[1]*RPY[2];

                        if (MW_statusY == ON)
                                e[1]	= Adc.Kd[2]	* J_MW * (MW_RPS-MW_RPS_Rating);
                        else
                                e[1]	= Adc.Kd[2]	* J_MW * MW_RPS;

                        e[2]	= Adc.Kd[1]*Wbo[2]/w0 -	Adc.Kp[0]*RPY[0];


                        cmd[0]	= (e[1]*BfMM[2]	- e[2]*BfMM[1])	/ BfMM[3];
                        cmd[1]	= (e[2]*BfMM[0]	- e[0]*BfMM[2])	/ BfMM[3];
                        cmd[2]	= (e[0]*BfMM[1]	- e[1]*BfMM[0])	/ BfMM[3];

                        break;

                default:

                        cmd[0] = cmd[1]	= cmd[2] = 0.0;

                        break;
        } /* end of	switch */

        Cmd_Limiter( cmd );			/*	converted command with limiter	*/

        Beta_prev	= beta;
        //Rate_prev = MW_RPS;
} /* void MT_Controller */



/*=============================================================================
function: YW_Controller
actions	: uosat	y-wheel	controller
input	: dt	  :	  sampling time
return	: y-wheel speed	command	in rpm
=============================================================================*/
int		YW_Controller (double dt)
{
        /*==================================
          this controller	is assuming
          the	wheel speed	tracking
          controller is working properly
          ==================================*/
        int		dir,err_tmp;
        double	Nwy,Pref;
        double outdata;
        double Q_tmp[4],Qbo_tmp[4],Q_oi_tmp[4];



        switch( MW_statusY )		{
                //////printf("YW_Controller：MW_statusY= %d\n",MW_statusY);
                case TURNON:				/*	spun up	the	wheel to nominal w	*/
                        /*	open loop						*/

                        MW_CmdRPSY	= MW_RPS_Rating;
                        if( (fabs(Y[0])<Adc.PitchMask) && (fabs(MW_RPS-MW_RPS_Rating)<1.0) )
                                MW_statusY	= ON;
                        break;

                case ON:					/*	nominal	y-wheel	control			*/ 
                case BREAKING: /*	y-wheel	breaking to	stop		*/

                        dir		= Sign (MW_CmdRPSY*60.0/TwoPI+0.5);
                        Pref	= (double) Pitch_Ref * TwoPI / 36000.0;

                        if (Filter >= OAK_FILTER)	{
                                Pref	= RPY[1] - Pref;
                                Pref	= Angle_Normaliser (Pref);
                                Nwy		= Adc.Kpw*Pref + Adc.Kdw*Wbo[1];
                        }
                        else	{
                                Pref	= Y[0]- Pref;
                                Pref	= Angle_Normaliser (Pref);
                                Nwy		= Adc.Kpw*Pref + Adc.Kdw*Y[1];
                        }

                        Nwy		= (double) Sign(Nwy)	* min (fabs(Nwy), MW_T_Max);

                        MW_CmdRPSY+= Nwy*dt/J_MW;
                        MW_CmdRPSY	= (double) Sign(MW_CmdRPSY)	*
                                min	( fabs(MW_CmdRPSY), MW_RPS_Max );
                        /*	ywmax is int and in	rpm			*/
                        /*	herman's request !				*/

                        if (dir!=Sign (MW_CmdRPSY*60.0/TwoPI+0.5))/*如果Y wheel为Breaking态同时当前控制转速与前一刻控制转速变向，则进入turnoff态*/
                        {
                                MW_CmdRPSY		= 0.0;
                                MW_statusY	= TURNOFF;
                        }

                        break;

                case TURNOFF:				/*	y-wheel	turn off				*/

                        MW_CmdRPSY	= 0.0;
                        MW_statusY	= OFF;

                        break;

                case OFF:					/*	y-wheel	not	active				*/
                        MW_CmdRPSY	= 0.0;
                default:

                        MW_CmdRPSY	= 0.0;

                        break;
        } /* end of	switch */
        outdata=MW_CmdRPSY;
        return( (int) (outdata*60.0/TwoPI + 0.5) );/*加0.5的目的是为了取整，转速控制精度为一圈*/

} /* int YW_Controller */



void	TC_Actuator( void )
{
        MT_PWMCmd16[0]	= (short) ( MT_Cmd[0] * 100.0 / (double)MT_Xmax );
        MT_PWMCmd16[1]	= (short) ( MT_Cmd[1] * 100.0 / (double)MT_Ymax );
        MT_PWMCmd16[2]	= (short) ( MT_Cmd[2] * 100.0 / (double)MT_Zmax );

        /* Update actual MT vector */
        MT_Cmd[0]	= MT_PWMCmd16[0] * (double)MT_Xmax / 100.0;
        MT_Cmd[1]	= MT_PWMCmd16[1] * (double)MT_Ymax / 100.0;
        MT_Cmd[2]	= MT_PWMCmd16[2] * (double)MT_Zmax / 100.0;
}

void SUN_Predicted( double *z, double *o,double *bo)//36
{


        if(LXF_shade==ON)	{
                bo[0]=bo[1]=bo[2]=bo[3]=0.0;
        }
        else	{

                bo[0] = ( z[0]*o[0] + z[1]*o[1] + z[2]*o[2] ) ;
                bo[1] = ( z[3]*o[0] + z[4]*o[1] + z[5]*o[2] ) ;
                bo[2] = ( z[6]*o[0] + z[7]*o[1] + z[8]*o[2] ) ;

                bo[3] = sqrt( bo[0]*bo[0] + bo[1]*bo[1] + bo[2]*bo[2] );

        }
}



/*=============================================================================
function: ADCS_Models
actions	: derive modelled vectors for filtering
input	:
return	: error	code
=============================================================================*/
int		ADCS_Models( void )//37
{
        double	r[3],Aoi[9];
        int		err,i;


        err = Orbit( r, Aoi );
        if( err != 0 )	return( err );

        BfLO_prev[0]	= BfLO[0];//Bo
        BfLO_prev[1]	= BfLO[1];
        BfLO_prev[2]	= BfLO[2];
        BfLO_prev[3]	= BfLO[3];

        if( NoMMTlm == OFF )
                BF_Predicted( r, Aoi, BfLO);

        if( (BfLO[3]<BfMM_Min) || (BfLO[3]>BfMM_Max) )	{
                BfLO[0]	= BfLO_prev[0];
                BfLO[1]	= BfLO_prev[1];
                BfLO[2]	= BfLO_prev[2];
                BfLO[3]	= BfLO_prev[3];
                return( 33 );
        }

        /*Sun (Mjd, LXF_sun);
          LXF_shade = eclipse (r, LXF_sun);
          LXF_shade=0;

          if(LXF_shade==0)
          SUN_Predicted( Aoi, LXF_sun,SUN_V);
          else
          SUN_V[0]=SUN_V[1]=SUN_V[2]=SUN_V[3]=0;
          */

        return( 0 );
} /* int ADCS_Models */



/*=============================================================================
function: ADCS_Filter
actions	: atacq	estimator
input	:
return	: error	flag
=============================================================================*/
int		ADCS_Filter( void )
{
        int		i, err;
        double	N[3];


        if( Period_True <= 0.0 )		return( 10 );
        /* no time ahead,nothing to	do	*/

        //        MM_MalTLM( );
        /*==================================
          detect
          can	telemetry no-response or
          mal-functioning	!
          for	magnetometer channels only
          ==================================*/

        MW_RPS	= (double) MW_RPMTlm16 * TwoPI / 60.0; /* 动量轮实际测得转速转化为rad/s */

        //////printf( "MW_RPMTlm16 %d\n",(INT32U)MW_RPMTlm16 );
        //////printf( "MW_RPS OUTPUT %d\n",(INT32U)MW_RPS );

        dMW_RPS		= (MW_RPS-MW_RPS_Prev) / Period_True;
        MW_RPS_Prev	= MW_RPS;


        /*==================================
          better to recover anytime
          anyway !!!
          ==================================*/

        if( Filter == NO_FILTER )	{
                BF_Measured( );

                if( NoMMTlm==ON )		return( 0 );

                NoFilter ( (double)	NoMMTlm_Counter*Period_True );
                /* estimate	very coarse	y-axis spin	rate */
        }
        else	{
                N[0] = N[1]	= N[2] = 0.0;

                if( Control != OFF )
                        MT_FiringEffect( MT_Cmd, N );

                BF_Measured( );
                /*==caution!==========================
                  recover	measurements
                  should be recovered	after
                  computing the torque vector
                  induced	by magnetorquer
                  ==================================*/

                switch( Filter )	{
                        case PITCH_FILTER:

                                PitchFilter( Period_True, N[1] );	/* Period_True correction not required */

                                Filter_Sts	= 10;

                                break;

                        case RATE_FILTER:
                        case HYBRID_FILTER:

                                P_prop( Period_True );		/* covariance matrix propagation */

                                X_prop( Period_True, N );	/*			   state propagation */

                                err		= RateFilter( (double) NoMMTlm_Counter*Period_True );

                                if( err != 0 )		return( err );

                                if( Filter == HYBRID_FILTER )	{
                                        PitchFilter( Period_True, N[1] );
                                        Filter_Sts	= 11;
                                }
                                else
                                        Filter_Sts	= 1;

                                break;

                        case OAK_FILTER:
                        case ATACQ_FILTER:
                                if( Filter_Sts/100 == OFF )		{
                                        if( (Filter_Sts%100) / 10 == ON )	{
                                                BYLO[0]	= cos(Y[0]);
                                                BYLO[1]	= 0.0;
                                                BYLO[2]	=-sin(Y[0]);

                                                BYLO[3]	= 0.0;
                                                BYLO[4]	= 1.0;
                                                BYLO[5]	= 0.0;

                                                BYLO[6]	=-BYLO[2];
                                                BYLO[7]	= 0.0;
                                                BYLO[8]	= BYLO[0];

                                                i	= RPY2Qtn (BYLO, X);

                                                if( i != 0 )	return (i);

                                                X[4]	= 0.0;
                                                X[5]	= Y[1] - W0;
                                                X[6]	= 0.0;
                                        }
                                        else	{
                                                BYLO[0]	= BfMM[2]*BfLO[0] -	BfMM[0]*BfLO[2];
                                                BYLO[2]	= BfMM[0]*BfLO[0] +	BfMM[2]*BfLO[2];
                                                BYLO[1]	= atan2	(BYLO[0], BYLO[2]);

                                                BYLO[0]	= cos(BYLO[1]);
                                                BYLO[2]	=-sin(BYLO[1]);
                                                BYLO[1]	= 0.0;

                                                BYLO[3]	= 0.0;
                                                BYLO[4]	= 1.0;
                                                BYLO[5]	= 0.0;

                                                BYLO[6]	=-BYLO[2];
                                                BYLO[7]	= 0.0;
                                                BYLO[8]	= BYLO[0];

                                                i	= RPY2Qtn( BYLO, X );

                                                if( i != 0 )	return( i );

                                                X[4]	= 0.0;
                                                X[5]	=-W0;
                                                X[6]	= 0.0;
                                        }

                                        Filter_Sts	+= 100*ON;
                                }

                                P_prop( Period_True );		/* covariance matrix propagation */

                                X_prop( Period_True, N );	/*			   state propagation */

                                if( Filter == OAK_FILTER )
                                        if( NoMMTlm == OFF ){
                                                err	= OakFilter( BfMM, BfLO, R1,Sm,SUN_V);//( Sm,SUN_V,SR1);////( Sm,SUN_V,R1);//	//err	=SunFilter( Sm,SUN_V,SR1);//
                                                if( err != 0 )	return( err );
                                        }
                                if( Filter == Sun_Filter ){
                                        if( LXF_shade == OFF){
                                                err	= OakFilter( Sm,SUN_V,R1,Sm,SUN_V);//( Sm,SUN_V,SR1);////( Sm,SUN_V,R1);//	//err	=SunFilter( Sm,SUN_V,SR1);//
                                                if( err != 0 )	return( err );}
                                }


                                if( Filter == ATACQ_FILTER )	{
                                        PitchFilter( Period_True, N[1] );
                                        Filter_Sts	= 110;
                                }
                                else
                                        Filter_Sts	= 100;

                                break;
                        case Sun_Filter:

                                P_prop( Period_True );

                                X_prop( Period_True, N );

                                if( Filter == Sun_Filter )
                                {
                                        if( LXF_shade == OFF)
                                        {
                                                err	= SunFilter(  BfMM, BfLO,Sm,SUN_V,R1,SR1,Sm_V);//( BfMM, BfLO, R1 );//( Sm,SUN_V,R1);//	//err	=SunFilter( Sm,SUN_V,SR1);//
                                                if( err != 0 )	return( err );
                                        }

                                }

                                break;

                        default:
                                return( 13 );
                } /* end of	switch */
        }

        if( NoMMTlm == OFF )	NoMMTlm_Counter = 1;
        /*	should reset at	this point		*/
        return( 0 );
} /* int ADCS_Filter */



/*=============================================================================
function: ADCS_Control
actions	:
input	:
output	:
=============================================================================*/
int		ADCS_Control( void )
{
        /************************************************************
         *	ATTITUDE CONTROLLER:									*
         *		BDOT, CPL CONTROLLER & Y-WHEEL 3-WHEEL CONTROLLER
         *
         ************************************************************/

        if( Period_True <= 0.0 )			return( 0 );


        if( NoMMTlm==ON )
                MT_Cmd[0] = MT_Cmd[1]	= MT_Cmd[2] = 0.0;
        else
                MT_Controller( (double)NoMMTlm_Counter*Period_True, MT_Cmd );


        if( Filter<PITCH_FILTER && Control>=C4 )	return( 14 );
        /*	 you cannot	start wheel-cntl	*/
        //////printf("ADCS_Control：MW_statusY= %d\n",MW_statusY);
        if( Control==C4 && MW_statusY==OFF )	{
                //////printf("Control==C4 && MW_statusY==OFF");

                MW_statusY	= TURNON;
        }

        if( Control==C5 && MW_statusY==OFF)	{

                MW_statusY	= TURNON;
        }

        /*===================================================================
          Bt	=	sqrt(BfMM[0]*BfMM[0] + BfMM[1]*BfMM[1]);
          if (Bt < (double)Adc.BZmask)	YWHEEL	=	ON;

          <reminder>
          originally (tmsat controller) we wish to start wheel over pole
          hence we have this commented part, however for uosat-12, herman
          reckon this	part is	no longer necessary	(on	05/02/99)
          ==================================================================*/

        /*
           if ((Control !=	C4 && Control != C5) &&	YWHEEL == ON)
           YWHEEL	=	BREAKING;
           */

        if( (Control!=C4 && Control!=C5) && MW_statusY==ON )
                MW_statusY	= OFF;
        /*======================================
          if ywheel == off HERE, then	ywcontroller
          below this line	will not be	called which
          keeps current value	of MW_CmdRPS
          =======================================*/
        //////printf("sssss after ADCS_Control：MW_statusY= %d\n",MW_statusY);
        //	if( MW_statusY!=OFF )	MW_RPMCmd16	= YW_Controller( Period_True);
        MW_RPMCmd16	= YW_Controller( Period_True);//去掉了原先判断MW_statusY!=OFF的条件，这样省去了BREAKING态的设置
        /*	y-wheel	control	command			*/

        return( 0 );

} /* int ADCS_Control */



void ADCS_Initialise( void )
{
        int i;

        seconds80	= gps_week * 604800 + gps_second;
        seconds80_Prev	= seconds80;                
        gps_weeks_ADCS= (unsigned long) gps_week;
        gps_seconds_ADCS= gps_second;

        MW_RPMTlm16 = MW_RPMCmd16 = 0;
        Filter		= 0;
        Control		= 0;
        Pitch_Ref	= 0;

        Period_Nom	= 2;
        Period_True = 2.0;// 默认周期设为2s

        Mjd		= 0.0;
        W0		= TwoPI / 5737.6;		/* 即 Eph.n0, 卫星的轨道角速率		*/
        GG		= 631.35 / 576000.0;

        Beta_prev	= 0.0;

        NoMMTlm_Counter	= 1;

        NoMMTlm = OFF;

        MT_PWMCmd16[0] = MT_PWMCmd16[1] = MT_PWMCmd16[2] = 0;
        MT_Cmd[0] = MT_Cmd[1] = MT_Cmd[2] = 0.0;

        MM_Tlm16[0] = MM_Tlm16Prev[0] = 0;
        MM_Tlm16[1] = MM_Tlm16Prev[1] = 0;
        MM_Tlm16[2] = MM_Tlm16Prev[2] = 0;

        BfMM[0] = BfLO[0] = BfMM_prev[0] = BfLO_prev[0] = 0.0;
        BfMM[1] = BfLO[1] = BfMM_prev[1] = BfLO_prev[1] = 0.0;
        BfMM[2] = BfLO[2] = BfMM_prev[2] = BfLO_prev[2] = 0.0;
        BfMM[3] = BfLO[3] = BfMM_prev[3] = BfLO_prev[3] = 1.0;

        RPY[0] = RPY[1] = RPY[2] = 0.0;
        Wbo[0] = Wbo[1] = Wbo[2] = 0.0;

        Y[0] = Y[1] = 0.0;
        O[0] = O[1] = O[2] = 0.0;
        C_pred[0] = C_pred[1] = C_pred[2] = 0.0;

        for( i=0; i<7;  i++ )		dX[i]	= 0.0;
        for( i=0; i<21; i++ )		K[i]	= 0.0;
        for( i=0; i<16; i++ )		HP[i]	= 0.0;
        for( i=0; i<9;  i++ )		BYLO[i]	= 0.0;

        ZKC_Counter	= 0;


        ADCS_OnOff	= OFF;

        WorkMode	= 1;
        TlmSource	= 1;
        LogOpen		= OFF;
        TlmCANOnOff	= ON;

        MMCorrectX = MMCorrectY = MMCorrectZ = 0;
        R_ECEF[0] = R_ECEF[1] = R_ECEF[2] = 0;
        V_ECI[0] = V_ECI[1] = V_ECI[2] = 0;

}



int ADCS_LoadCfgFile( void )
{
        Filter_Init	= ON;
        {
                Filter	= 1;

                X[0]	= 0;		X[1]	= 0;		X[2]	= 0;		X[3]	= 1.0;
                X[4]	= 0;		X[5]	= 0;		X[6]	= 0;

                P0[0]	= 1.e-4;	P0[1]	= 0;		P0[2]	= 0;
                P0[3]	= 1.e-4;	P0[4]	= 0;
                P0[5]	= 1.e-4;

                P1[0]	= 0.1;		P1[1]	= 0;		P1[2]	= 0;		P1[3]	= 0;
                P1[4]	= 0;		P1[5]	= 0.05;		P1[6]	= 0;		P1[7]	= 0;
                P1[8]	= 0;		P1[9]	= 0;		P1[10]	= 0.2;		P1[11]	= 0;
                P1[12]	= 0;		P1[13]	= 0;		P1[14]	= 0;		P1[15]	= 0.001;

                P2[0]	= 0;		P2[1]	= 0;		P2[2]	= 0.02;
                P2[3]	= 0;		P2[4]	= 0;		P2[5]	= 0;
                P2[6]	= -0.02;	P2[7]	= 0;		P2[8]	= 0;
                P2[9]	= 0;		P2[10]	= 0;		P2[11]	= 0;

                P3[0]	= 0.001;	P3[1]	= 0;		P3[2]	= 0;
                P3[3]	= 0;		P3[4]	= 0.001;	P3[5]	= 0;
                P3[6]	= 0;		P3[7]	= 0;		P3[8]	= 0.001;

                Filter_Sts	= OFF;
                dBeta		= 0.0;
                MW_statusY	= OFF;
                Control		= 1;
                P_counter	= 0;
        }

        //	R1[0]	= 0.0001;	R1[1]	= 0.0001;	R1[2]	= 0.0001;	// divergent
        //	R1[0]	= 0.001;	R1[1]	= 0.001;	R1[2]	= 0.001;	// divergent
        //	R1[0]	= 0.002;	R1[1]	= 0.002;	R1[2]	= 0.002;	// convergent
        //	R1[0]	= 0.003;	R1[1]	= 0.003;	R1[2]	= 0.003;	// convergent
        R1[0]	= 0.005;	R1[1]	= 0.005;	R1[2]	= 0.005;	// convergent
        //	R1[0]	= 0.008;	R1[1]	= 0.008;	R1[2]	= 0.008;	// convergent
        //	R1[0]	= 0.009;	R1[1]	= 0.009;	R1[2]	= 0.009;	// convergent
        //	R1[0]	= 0.01;		R1[1]	= 0.01;		R1[2]	= 0.01;		// divergent
        //	R1[0]	= 0.015;	R1[1]	= 0.015;	R1[2]	= 0.015;	// some divergent,TS-1
        //	R1[0]	= 0.02;		R1[1]	= 0.02;		R1[2]	= 0.02;		// some divergent
        //	R1[0]	= 0.03;		R1[1]	= 0.03;		R1[2]	= 0.03;		// convergent
        //	R1[0]	= 0.05;		R1[1]	= 0.05;		R1[2]	= 0.05;		// convergent
        //	R1[0]	= 0.06;		R1[1]	= 0.06;		R1[2]	= 0.06;		// convergent
        //	R1[0]	= 0.08;		R1[1]	= 0.08;		R1[2]	= 0.08;		// some divergent
        //	R1[0]	= 0.1;		R1[1]	= 0.1;		R1[2]	= 0.1;		// some divergent
        //	R1[0]	= 0.5;		R1[1]	= 0.5;		R1[2]	= 0.5;		// some divergent
        //	R1[0]	= 0.6;		R1[1]	= 0.6;		R1[2]	= 0.6;		// convergent
        //	R1[0]	= 0.8;		R1[1]	= 0.8;		R1[2]	= 0.8;		// convergent
        //	R1[0]	= 1.0;		R1[1]	= 1.0;		R1[2]	= 1.0;		// convergent
        //	R1[0]	= 1.1;		R1[1]	= 1.1;		R1[2]	= 1.1;		// divergent
        //	R1[0]	= 1.5;		R1[1]	= 1.5;		R1[2]	= 1.5;		// divergent
        //	R1[0]	= 2.0;		R1[1]	= 2.0;		R1[2]	= 2.0;		// divergent
        //	R1[0]	= 5.0;		R1[1]	= 5.0;		R1[2]	= 5.0;		// some divergent
        //	R1[0]	= 10;		R1[1]	= 10;		R1[2]	= 10;		// divergent


        Q1[0]	= 1.0e-8;	Q1[1]	= 1.0e-8;	Q1[2]	= 1.0e-8;
        Q2[0]	= 1.0e-6;	Q2[1]	= 1.0e-6;	Q2[2]	= 1.0e-6;

        Ix		= 0.38;
        Iy		= 0.39;
        Iz		= 0.38;	//三轴惯量更新
        J_MW	= -1.34e-4;

        MT_Xmax	= 0.7;
        MT_Ymax	= 0.7;
        MT_Zmax	= 0.7;

        MW_T_Max		= 6.7e-5;			/* 动量轮能提供的最大力矩　Nm 5.0e-5	*/
        MW_RPM_Max		= 3000;				/* 动量轮转速最大值3000 RPM		*/
        MW_RPS_Max		= 0.1047*MW_RPM_Max;
        MW_RPM_Rating	= 1500;				/* 动量轮额定转速rad/s(1500 RPM)	*/  
        MW_RPS_Rating	= 0.1047*MW_RPM_Rating;
        /* 当动量轮角动量与星体角动量正向相同时的转速为＋向				*/
        BfMM_Min	= 10.0;
        BfMM_Max	= 70.0;


        Adc.G			= 0.5;			/* beta角平滑增益		0.5			*/
        Adc.G1			= 0.6443;		/* for PitchFilter		0.6443		*/
        Adc.G2			= 0.0158;		/* for PitchFilter		0.0158		*/
        Adc.wy			= -3*3.1416/180.;	/* Y轴转动参考转速					*///和MW额定转速配套使用，Adc.wy=J_MW*MW_RPS_Rating/Iy
        Adc.Ks1			= 0.25;			/* Y轴转动控制增益		0.25		*/
        Adc.Kd1			= 1.0;			/* 章动阻尼控制增益		1.0			*/
        Adc.Kd[0]		= 0.05;			/* 微分控制增益,			2.0		*/
        Adc.Kd[1]		= 0.05;			/* 微分控制增益,			2.0		*/
        Adc.Kd[2]		= 20;			/* 微分控制增益，MW卸载,	25		*/
        Adc.Kp[0]		= 1.0;			/*				2.0					*/
        Adc.Kp[1]		= 1.0;			/*				2.0					*/
        Adc.Kpw			= 3.0e-4;		/* 动量轮P控制增益		7.15e-4		*/
        Adc.Kdw			= 0.01;			/* 动量轮D控制增益		6.26e-2		*/
        Adc.W_step		= -1.05;		/* 动量轮增速步长 rad/s	 -10RPM		*/
        Adc.PitchMask	= 0.4;			/* MW启动完成时俯仰角的条件rad 20 deg*///应当适当放大，否则MW可能一直停留在TURNON态
        Adc.TStep		= 1.0;			/* Step	in second for Adams	integration	1.0	*/

        Eph.l2epoc	= 56799;
        Eph.K2		= 22021178847.059;
        Eph.A30		= -14963.9173277076;
        Eph.ke		= 19964976.8344469;
        Eph.n0		= 0.00109749961697;
        Eph.e0		= 0.00055937;
        Eph.i0		=1.700987417018162;
        Eph.an0		= -1.337323632755615;
        Eph.ap0		= 3.280887381191461;
        Eph.M0		= 0.661804186332921;
        Eph.n0dot	= 0;
        Eph.n0dot2	= 0;
        Eph.a1		= 6916870.13262693;
        Eph.d1		= -0.000655509136258499;
        Eph.a0		= 6918378.52091338;
        Eph.p0		= 6918376.35618874;
        Eph.q0		= 6914508.58752014;
        Eph.L0		=2.605367934768767;
        Eph.dan		= 1.96657778263355e-07;
        Eph.dap		= -6.93576643008110e-07;
        Eph.UTC2UT1	= -0.45;
        Eph.Eeqnox	= 0;

        Period_	= (int)(TwoPI/Eph.n0 + 0.5);
        NoMMTlm_Counter	= 1;	/* must do it! */

        ModGH[  0]	=  0.00000000e+000;
        ModGH[  1]	=-29439.5;
        ModGH[  2]	=-1502.4;
        ModGH[  3]	=4801.1;
        ModGH[  4]	=-2453.1;
        ModGH[  5]	=1735.803584;
        ModGH[  6]	=-1629.686605;
        ModGH[  7]	=485.5804439;
        ModGH[  8]	=-184.7232186;
        ModGH[  9]	=1346.2;
        ModGH[  10]	=-957.6688398;
        ModGH[  11]	=-47.96917413;
        ModGH[  12]	=157.1398443;
        ModGH[  13]	=30.62238832;
        ModGH[  14]	=31.29073745;
        ModGH[  15]	=-28.84524272;
        ModGH[  16]	=905.6;
        ModGH[  17]	=258.9905404;
        ModGH[  18]	=91.20008772;
        ModGH[  19]	=9.100796668;
        ModGH[  20]	=-14.54934897;
        ModGH[  21]	=-6.675351783;
        ModGH[  22]	=3.633494972;
        ModGH[  23]	=0.550758856;
        ModGH[  24]	=-2.205852605;
        ModGH[  25]	=-233.6;
        ModGH[  26]	=92.87414064;
        ModGH[  27]	=12.1869876;
        ModGH[  28]	=9.407676703;
        ModGH[  29]	=9.583338716;
        ModGH[  30]	=-1.441246498;
        ModGH[  31]	=-1.131483084;
        ModGH[  32]	=-0.367642101;
        ModGH[  33]	=0.043666303;
        ModGH[  34]	=-0.000519675;
        ModGH[  35]	=0.07268021;
        ModGH[  36]	=71.3;
        ModGH[  37]	=14.64242043;
        ModGH[  38]	=-4.648041062;
        ModGH[  39]	=2.570494209;
        ModGH[  40]	=1.162760467;
        ModGH[  41]	=-0.758497061;
        ModGH[  42]	=0.342157507;
        ModGH[  43]	=-0.032441951;
        ModGH[  44]	=-0.07223321;
        ModGH[  45]	=0.002708461;
        ModGH[  46]	=0.001589262;
        ModGH[  47]	=-0.004452108;
        ModGH[  48]	=0.003709013;
        ModGH[  49]	=81.4;
        ModGH[  50]	=-14.26815886;
        ModGH[  51]	=-10.35622656;
        ModGH[  52]	=-0.198022632;
        ModGH[  53]	=-0.506629332;
        ModGH[  54]	=0.190213261;
        ModGH[  55]	=0.020367003;
        ModGH[  56]	=0.008498543;
        ModGH[  57]	=0.013378351;
        ModGH[  58]	=0.000996066;
        ModGH[  59]	=0.000274147;
        ModGH[  60]	=-4.30116E-05;
        ModGH[  61]	=-0.000523308;
        ModGH[  62]	=3.30491E-05;
        ModGH[  63]	=-1.14953E-05;
        ModGH[  64]	=23.8;
        ModGH[  65]	=1.45;
        ModGH[  66]	=1.816666667;
        ModGH[  67]	=-0.338648106;
        ModGH[  68]	=-0.37848906;
        ModGH[  69]	=-0.010298573;
        ModGH[  70]	=0.035309393;
        ModGH[  71]	=-0.006584389;
        ModGH[  72]	=-0.00487498;
        ModGH[  73]	=0.000575071;
        ModGH[  74]	=0.000755056;
        ModGH[  75]	=8.0607E-05;
        ModGH[  76]	=4.47064E-05;
        ModGH[  77]	=-2.05293E-05;
        ModGH[  78]	=-1.0883E-05;
        ModGH[  79]	=-8.34774E-07;
        ModGH[  80]	=1.14395E-06;
        ModGH[  81]	=5.4;

        return( 0 );
}

void ADCS_CfgUpdate(void)
{
        int  		cnt_zwj1 = 0;
        int  		cnt_zwj2 = 0;
        int              sscounter;
        int 		ss_filter_counter;
        int 		ss_modgh_counter;

//        //printf ("--------------------------------------\n");
 //       //printf ("ACSFL_Flag :  %d  ,,  CMD_CFG_UPDATE  :  %d\n", ACSFL_Flag, CMD_CFG_UPDATE);
  //      //printf ("--------------------------------------\n");
        if((ACSFL_Flag==1)&&(CMD_CFG_UPDATE==1)) 
        {                                                                                                                
                /* parameters of filter initialization*/                                                        
                for(ss_filter_counter=0;ss_filter_counter<7;ss_filter_counter++)
                {
                        X[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }

                for(ss_filter_counter=0;ss_filter_counter<6;ss_filter_counter++)
                {
                        P0[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }

                for(ss_filter_counter=0;ss_filter_counter<16;ss_filter_counter++)
                {
                        P1[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }	

                for(ss_filter_counter=0;ss_filter_counter<12;ss_filter_counter++)
                {
                        P2[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }	

                for(ss_filter_counter=0;ss_filter_counter<9;ss_filter_counter++)
                {
                        P3[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }	

                for(ss_filter_counter=0;ss_filter_counter<3;ss_filter_counter++)
                {
                        R1[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }	

                for(ss_filter_counter=0;ss_filter_counter<3;ss_filter_counter++)
                {
                        Q1[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }	

                for(ss_filter_counter=0;ss_filter_counter<3;ss_filter_counter++)
                {
                        Q2[ss_filter_counter]=ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                }	


                Filter_Sts	= OFF;
                dBeta		= 0.0;
                MW_statusY	= OFF;
                Filter=1;
                Control		= 1;
                P_counter	= 0;          
                /*  initialise after  parameter  assignment */

                /* parameters of satellite configuration initialization*/

                Ix      = ByteToDouble(adcsbuf1 , cnt_zwj1++);							
                Iy      = ByteToDouble(adcsbuf1 , cnt_zwj1++);							
                Iz      = ByteToDouble(adcsbuf1 , cnt_zwj1++);							
                J_MW    = ByteToDouble(adcsbuf1 , cnt_zwj1++);		
                MT_Xmax = ByteToDouble(adcsbuf1 , cnt_zwj1++);					
                MT_Ymax = ByteToDouble(adcsbuf1 , cnt_zwj1++);					
                MT_Zmax = ByteToDouble(adcsbuf1 , cnt_zwj1++);
                MW_T_Max= ByteToDouble(adcsbuf1 , cnt_zwj1++);
                MW_RPM_Max= ByteToDouble(adcsbuf1 , cnt_zwj1++);
                MW_RPM_Rating= ByteToDouble(adcsbuf1 , cnt_zwj1++);
                BfMM_Min= ByteToDouble(adcsbuf1 , cnt_zwj1++);	
                BfMM_Max= ByteToDouble(adcsbuf1 , cnt_zwj1++);		

//                //printf ("--------------------------------------\n");
 //               //printf ("Ix  :   %d   .,  MW-MAX   :  %d ,     MW   :  %d   \n", (int)(Ix*1000), MW_RPM_Max, MW_RPM_Rating);
  //              //printf ("--------------------------------------\n");

                /* parameters of ModGH initialization*/

                for(ss_modgh_counter=0;ss_modgh_counter<82;ss_modgh_counter++)   
                {
                        ModGH[ ss_modgh_counter ]=ByteToDouble(adcsbuf1 , cnt_zwj1++);
                        ////printf("ModGH[ %d ]= %d\n",ss_modgh_counter,(int)(ModGH[ ss_modgh_counter ]*1000));

                }																	
                ACSFL_Flag = 0;
                CMD_CFG_UPDATE=0;
                cnt_zwj1 = 0;

        }

        ////printf ("ACSGD_Flag = %d\n", ACSGD_Flag);

    //    //printf ("--------------------------------------\n");
     //   //printf ("ACSGD_Flag :  %d  ,,  CMD_OCO_UPDATE  :  %d\n", ACSGD_Flag, CMD_OCO_UPDATE);
      //  //printf ("--------------------------------------\n");
        if((ACSGD_Flag==1)&&(CMD_OCO_UPDATE==1))
        {

                Eph.l2epoc	= ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.K2		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.A30		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.ke		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.n0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.e0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.i0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.an0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.ap0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.M0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.n0dot	  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.n0dot2	= ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.a1		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.d1		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.a0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.p0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.q0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.L0		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.dan		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.dap		  = ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.UTC2UT1	= ByteToDouble(adcsbuf2 , cnt_zwj2++);
                Eph.Eeqnox	= ByteToDouble(adcsbuf2 , cnt_zwj2++);

                ACSGD_Flag = 0;
                CMD_OCO_UPDATE=0;
                cnt_zwj2 = 0;
        }	
}


void	ADCS_AutoRunning( void )
{

        if(	(Filter==1)&&(Control==1)&&(fabs(X[4])<0.0175)&&(fabs(X[6])<0.0175) )
        {	// 0.0175rad/s=1deg/s
                ZKC_Counter++;
                if(	ZKC_Counter>500 )	{
                        ZKC_Counter	= 0;
                        Control		= 3;
                        Filter		= 3;
                }
        }
        else if( (Filter==1)&&(Control==1) )	{
                ZKC_Counter	= 0;
        }


        /*	if(	(Filter==1)&&(Control==3)&&(fabs(X[4])<0.0035)&&(fabs(X[6])<0.0035) )
                {	// 0.0035rad/s=0.2deg/s
                ZKC_Counter++;
                if(	ZKC_Counter>200 )	{
                ZKC_Counter	= 0;
                Filter		= 3;
                }
                }
                else if( (Filter==1)&&(Control==3) )	{
                ZKC_Counter	= 0;
                }*/


        // MW spin up
        if(	(Filter==3) && (Control==3) &&
                        (Sign(BfMM[1])==Sign(BfLO[1])) && (fabs(BfMM[1]-BfLO[1])<6.) &&
                        (fabs(X[4])<0.0015) && (fabs(X[6])<0.0015) && (fabs(X[5]-Adc.wy)<0.0035) )
        {	// 0.0015rad/s=0.086deg/s,0.0035rad/s=0.2deg/s
                ZKC_Counter++;
                if(	ZKC_Counter>350 )	{
                        ZKC_Counter	= 0;
                        Control		= 4;
                }
        }
        else if( (Filter==3)&&(Control==3) )	{
                ZKC_Counter	= 0;
        }


        if(	(Filter==3)&&(Control==4)&&
                        (MW_statusY==ON)&&(fabs(Y[0])<0.1)&&(fabs(Y[1])<0.0035) )
        {	// 0.1rad=5.7deg,0.0035rad/s=0.2deg/s
                Filter		= 4;
        }


        if(	(Filter==4)&&(Control==4)&&(fabs(Y[0]-RPY[0])<0.017)&&(fabs(Y[1]-Wbo[2])<1.7e-4) )
        {	// 0.017rad=1deg, 1.7e-4rad=0.01deg/s
                ZKC_Counter++;
                if(	ZKC_Counter >= 100 )	{
                        ZKC_Counter	= 0;
                        Filter		= 5;
                        Control		= 5;
                        Period_Nom	= 10;
                        //	tcMCU2ADCSPeriod = Period_Nom;
                        //	if( TC_MCU2ADCSPeriod() != 0 )	{
                        // Err Log
                        //	}
                }
        }


}

//命令指派函数，负责解析地面发送的遥控指令
//修改时间2015.6.15 by Shi

int	CmdDisposal(INT8U addr_cmd, short value1, short value2)
{
        /*
        ////printf("addr_cmd	=%x\n",addr_cmd	);
        ////printf("value1	=%d\n",value1	);
        ////printf("value2	=%d\n",value2);
        ////printf("WorkMode=%x\n ",WorkMode );
        */
        if( ADCS_OnOff == OFF )	{
                if( addr_cmd == ADDR_CMD_ADCS_ONOFF )	{
                        if( value1 == 0 || value1 == 1 )	{
                                ADCS_OnOff	= value1;
                        }
                        else	{
                                return( ADDR_CMD_ADCS_ONOFF );
                        }
                }
        }
        else
        {
                switch( addr_cmd )
                {
                        //ADCS星上软件开关
                        case ADDR_CMD_ADCS_ONOFF:
                                if( value1 == 0 || value1 == 1 )	{
                                        ADCS_OnOff	= value1;
                                        ////printf("ADDR_CMD_ADCS_ONOFF %d\n",value1);				
                                }
                                else	{
                                        return( ADDR_CMD_ADCS_ONOFF );
                                }
                                break;

                                //ADCS配置文件更新
                        case ADDR_CMD_CFG_UPDATE:
                                if( value1 == 0 || value1 == 1 )	{
                                        CMD_CFG_UPDATE = value1;
                                        ////printf("ADDR_CMD_CFG_UPDATE %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_CFG_UPDATE );
                                }
                                break;

                                //ADCS轨道参数更新
                        case ADDR_CMD_OCO_UPDATE:
                                if( value1 == 0 || value1 == 1 )	{
                                        CMD_OCO_UPDATE = value1;
                                        ////printf("ADDR_CMD_OCO_UPDATE %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_OCO_UPDATE );
                                }
                                break;

                                //ADCS控制工作模式
                        case ADDR_CMD_WORK_MODE:
                                if( value1 == 1 || value1 == 2 || value1 == 3 )	{
                                        WorkMode		= value1;
                                        tcMCU2MTPWMx	= 0;
                                        tcMCU2MTPWMy	= 0;
                                        tcMCU2MTPWMz	= 0;
                                        tcMWRPMy		= 0;
                                        Filter=0;
                                        Control=0;
                                        ////printf("ADDR_CMD_WORK_MODE %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_WORK_MODE );
                                }
                                break;

                                //遥测数据来源
                        case ADDR_CMD_TLM_SOURCE:
                                if( value1 == 1 || value1 == 2 )	{
                                        TlmSource	= value1;
                                        ////printf("ADDR_CMD_TLM_SOURCE %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_TLM_SOURCE );
                                }
                                break;

                                //选择滤波器
                        case ADDR_CMD_FILTER:
                                if( value1 >= 0 && value1 <= 6 )	{
                                        CMD_FILTER	= value1;
                                        ////printf("ADDR_CMD_FILTER %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_FILTER );
                                }
                                break;

                                //选择控制模式
                        case ADDR_CMD_CONTROL:
                                if( value1 >= 1 && value1 <= 5)	{
                                        CMD_CONTROL	= value1;
                                        ////printf("ADDR_CMD_CONTROL %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_CONTROL );
                                }
                                break;

                                //设置ADCS控制周期
                        case ADDR_CMD_PERIOD_NOM:
                                if( value1 >= 1 && value1 <= 20  )	{
                                        Period_Nom	= value1;
                                        ////printf("ADDR_CMD_PERIOD_NOM %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_PERIOD_NOM );
                                }
                                break;

                                //设置俯仰角参考值
                        case ADDR_CMD_PITCH_REF:
                                if( value1 >= -18000 && value1 <= 18000  )	{
                                        Pitch_Ref = value1;
                                        ////printf("ADDR_CMD_PITCH_REF %d\n",value1);	
                                }
                                else	{
                                        return( ADDR_CMD_PITCH_REF );
                                }
                                break;

                                //Adc.G 粗滤波增益
                        case ADDR_CMD_Adc_G:
                                if( value1 >= 0 && value1 <= 65000 )	{
                                        CMD_Adc_G	= value1;
                                        Adc.G	=CMD_Adc_G/10000.0;
                                        ////printf("ADDR_CMD_Adc_G %d \n",CMD_Adc_G);		
                                }
                                else	{
                                        return( ADDR_CMD_Adc_G );
                                }
                                break;

                                //Adc.G1 俯仰滤波增益1
                        case ADDR_CMD_Adc_G1:
                                if( value1 >= 0 && value1 <= 65000 )	{
                                        CMD_Adc_G1	= value1;
                                        Adc.G1	=CMD_Adc_G1/10000;
                                        ////printf("ADDR_CMD_Adc_G1 %f\n",CMD_Adc_G1);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_G1 );
                                }
                                break;

                                //Adc.G2 俯仰滤波增益2
                        case ADDR_CMD_Adc_G2:
                                if( value1 >= 0 && value1 <= 65000 )	{
                                        CMD_Adc_G2	= value1;
                                        Adc.G2	=CMD_Adc_G2/10000;
                                        ////printf("ADDR_CMD_Adc_G2 %f\n",CMD_Adc_G2);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_G2 );
                                }
                                break;

                                //Adc.wy 俯仰轴参考角速率
                        case ADDR_CMD_Adc_wy:
                                if( value1 >= -20000 && value1 <= 20000 )	{
                                        CMD_Adc_wy	= value1;
                                        Adc.wy	=CMD_Adc_wy/1000;
                                        ////printf("ADDR_CMD_Adc_wy %f\n",CMD_Adc_wy);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_wy );
                                }
                                break;

                                //Adc.Ks1 俯仰轴控制增益
                        case ADDR_CMD_Adc_Ks1:
                                if( value1 >= 0 && value1 <= 10000 )	{
                                        CMD_Adc_Ks1	= value1;
                                        Adc.Ks1	=CMD_Adc_Ks1/1000;
                                        ////printf("ADDR_CMD_Adc_Ks1 %f\n",CMD_Adc_Ks1);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Ks1 );
                                }
                                break;

                                //Adc.Kd1 章动阻尼控制增益
                        case ADDR_CMD_Adc_Kd1:
                                if( value1 >= 0 && value1 <= 10000 )	{
                                        CMD_Adc_Kd1	= value1;
                                        Adc.Kd1	=CMD_Adc_Kd1/1000;
                                        ////printf("ADDR_CMD_Adc_Kd1 %f\n",CMD_Adc_Kd1);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kd1 );
                                }
                                break;

                                //Adc.Kd_0 微分增益1
                        case ADDR_CMD_Adc_Kd_0:
                                if( value1 >= 0 && value1 <= 20000 )	{
                                        CMD_Adc_Kd_0	= value1;
                                        Adc.Kd[0]	=CMD_Adc_Kd_0/1000;
                                        ////printf("ADDR_CMD_Adc_Kd_0 %f\n",CMD_Adc_Kd_0);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kd_0 );
                                }
                                break;

                                //Adc.Kd_1 微分增益2
                        case ADDR_CMD_Adc_Kd_1:
                                if( value1 >= 0 && value1 <= 20000 )	{
                                        CMD_Adc_Kd_1	= value1;
                                        Adc.Kd[1]	=CMD_Adc_Kd_1/1000;
                                        ////printf("ADDR_CMD_Adc_Kd_1 %f\n",CMD_Adc_Kd_1);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kd_1 );
                                }
                                break;

                                //Adc.Kd_2 微分增益3
                        case ADDR_CMD_Adc_Kd_2:
                                if( value1 >= 0 && value1 <= 20000 )	{
                                        CMD_Adc_Kd_2	= value1;
                                        Adc.Kd[2]	=CMD_Adc_Kd_2/1000;
                                        ////printf("ADDR_CMD_Adc_Kd_2 %f\n",CMD_Adc_Kd_2);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kd_2 );
                                }
                                break;

                                //Adc.Kp_0 比例增益1
                        case ADDR_CMD_Adc_Kp_0:
                                if( value1 >= 0 && value1 <= 20000 )	{
                                        CMD_Adc_Kp_0	= value1;
                                        Adc.Kp[0]	=CMD_Adc_Kp_0/1000;
                                        ////printf("ADDR_CMD_Adc_Kp_0 %f\n",CMD_Adc_Kp_0);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kp_0 );
                                }
                                break;

                                //Adc.Kp_1 比例增益2
                        case ADDR_CMD_Adc_Kp_1:
                                if( value1 >= 0 && value1 <= 20000 )	{
                                        CMD_Adc_Kp_1	= value1;
                                        Adc.Kp[1]	=CMD_Adc_Kp_1/1000;
                                        ////printf("ADDR_CMD_Adc_Kp_1 %f\n",CMD_Adc_Kp_1);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kp_1 );
                                }
                                break;

                                //Adc.PitchMask－俯仰角条件（Y轴）
                        case ADDR_CMD_Adc_PitchMask:
                                if( value1 >= 0 && value1 <= 18000 )	{
                                        CMD_Adc_PitchMask	= value1;
                                        Adc.PitchMask	=CMD_Adc_PitchMask/1000;
                                        ////printf("ADDR_CMD_Adc_PitchMask %f\n",CMD_Adc_PitchMask);	
                                }
                                else	{
                                        return( ADDR_CMD_Adc_PitchMask);
                                }
                                break;

                                //Adc.TStep－Y轴积分步长（Y轴）
                        case ADDR_CMD_Adc_TStep:
                                if( value1 >= 0 && value1 <= 10000 )	{
                                        CMD_Adc_TStep	= value1;
                                        Adc.TStep	=CMD_Adc_TStep/1000;
                                        ////printf("ADDR_CMD_Adc_TStep %f\n",CMD_Adc_TStep);
                                }
                                else	{
                                        return( ADDR_CMD_Adc_TStep);
                                }
                                break;

                                //动量轮使能
                        case ADDR_CMD_MWY_ON_OFF	:
                                if( value1 == 0 || value1 == 1 )	{
                                        CMD_MWY_ON_OFF	= value1;
                                        ////printf("ADDR_CMD_MWY_ON_OFF %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MWY_ON_OFF );
                                }
                                break;

                                //设置动量轮额定转速
                        case ADDR_CMD_MWY_RPM_RATING:
                                if( value1 >= 0 && value1 <= 4000 )	{
                                        MW_RPM_Rating	= value1;
                                        MW_RPS_Rating	= 0.1047*MW_RPM_Rating;
                                        ////printf("ADDR_CMD_MWY_RPM_RATING %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MWY_RPM_RATING);
                                }
                                break; 

                                //遥控设置动量轮转速
                        case ADDR_CMD_MWy_RPM_EMERGENT:
                                if( value1 >= -4000 && value1 <= 4000 )	{
                                        tcMWRPMy	= value1;
                                        ////printf("ADDR_CMD_MWy_RPM_EMERGENT %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MWy_RPM_EMERGENT );
                                }
                                break;

                                //Adc.Kpw－动量轮比例增益
                        case ADDR_CMD_Adc_Kpw:
                                if( value1 >= 0 && value1 <= 65000 )	{
                                        CMD_Adc_Kpw	= value1;
                                        Adc.Kpw	=CMD_Adc_Kpw/10000;
                                        ////printf("ADDR_CMD_Adc_Kpw %f\n",CMD_Adc_Kpw);
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kpw);
                                }
                                break;

                                //Adc.Kdw2－动量轮微分增益
                        case ADDR_CMD_Adc_Kdw:
                                if( value1 >= 0 && value1 <= 65000 )	{
                                        CMD_Adc_Kdw	= value1;
                                        Adc.Kdw	=CMD_Adc_Kdw/10000;
                                        ////printf("ADDR_CMD_Adc_Kdw %f\n",CMD_Adc_Kdw);
                                }
                                else	{
                                        return( ADDR_CMD_Adc_Kdw);
                                }
                                break;

                                //Adc_W_step－动量轮增速步长
                                /*                        case ADDR_CMD_Adc_W_step:
                                                          if( value1 >= 0 && value1 <= 500 )	{
                                                          Adc.W_step	= value1;
                                ////printf("ADDR_CMD_Adc_W_step %d\n",value1);
                                }
                                else	{
                                return( ADDR_CMD_Adc_W_step);
                                }
                                break;
                                */
                                //动量轮最大转速
                        case ADDR_CMD_MW_RPM_Max:
                                if( value1 >= 0 && value1 <= 4000 )	{
                                        MW_RPM_Max	= value1;                                     	
                                        MW_RPS_Max		= 0.1047*MW_RPM_Max;
                                        ////printf("ADDR_CMD_MW_RPM_Max %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MW_RPM_Max);
                                }
                                break;

                                //动量轮的比例因子（加速性能）
                        case ADDR_CMD_MWy_COEF_K1:
                                if( value1 >= 0 && value1 <=65535 )	{
                                        tcMW_kpy= value1;
                                        if( TC_MWPIy() != 0 )	{
                                                return( ADDR_CMD_MWy_COEF_K1 );
                                        }
                                        ////printf("ADDR_CMD_MWy_COEF_K1 %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MWy_COEF_K1 );
                                }
                                break;

                                //设置磁力矩器工作模式
                        case ADDR_CMD_MT_CONTROL_MODE:
                                if( value1 == 0 || value1 == 1 )	{
                                        CMD_MT_CONTROL_MODE	= value1;
                                        ////printf("ADDR_CMD_MT_CONTROL_MODE %d\n",value1);				
                                }
                                else	{
                                        return( ADDR_CMD_MT_CONTROL_MODE);
                                }
                                break;

                                //遥控设置磁力矩器X轴
                        case ADDR_CMD_MT_PWMX_EMERGENT:
                                if( value1 >= -127 && value1 <= 127 )	{
                                        tcMCU2MTPWMx	= value1;
                                        ////printf("ADDR_CMD_MT_PWMX_EMERGENT %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MT_PWMX_EMERGENT );
                                }
                                break;

                                //遥控设置磁力矩器Y轴
                        case ADDR_CMD_MT_PWMY_EMERGENT:
                                if( value1 >= -127 && value1 <= 127 )	{
                                        tcMCU2MTPWMy	= value1;
                                        ////printf("ADDR_CMD_MT_PWMY_EMERGENT %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MT_PWMY_EMERGENT );
                                }
                                break;

                                //遥控设置磁力矩器Z轴
                        case ADDR_CMD_MT_PWMZ_EMERGENT:
                                if( value1 >= -127 && value1 <= 127 )	{
                                        tcMCU2MTPWMz	= value1;
                                        ////printf("ADDR_CMD_MT_PWMZ_EMERGENT %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MT_PWMZ_EMERGENT );
                                }
                                break;

                                //磁强计X轴修正值
                        case ADDR_CMD_MM_CORRECT_X:
                                if( value1 >= -10000 && value1 <= 10000 )	{
                                        MMCorrectX	= value1;
                                        ////printf("ADDR_CMD_MM_CORRECT_X %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MM_CORRECT_X);
                                }
                                break;

                                //磁强计Y轴修正值
                        case ADDR_CMD_MM_CORRECT_Y:
                                if( value1 >= -10000 && value1 <= 10000 )	{
                                        MMCorrectY	= value1;
                                        ////printf("ADDR_CMD_MM_CORRECT_Y %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MM_CORRECT_Y);
                                }
                                break;

                                //磁强计Z轴修正值
                        case ADDR_CMD_MM_CORRECT_Z:
                                if( value1 >= -10000 && value1 <= 10000 )	{
                                        MMCorrectZ	= value1;
                                        ////printf("ADDR_CMD_MM_CORRECT_Z %d\n",value1);
                                }
                                else	{
                                        return( ADDR_CMD_MM_CORRECT_Z);
                                }
                                break;			


                        default:
                                return( 0xFFFF );
                                break;
                }
        }


        return( 0 );
}

//写遥测数据的具体内容，应当具体分析，进而推断出ADCS具体工作状态
int		WriteTlmData( void )
{

        INT32U	addrADCS,addrTmp , INTTEMP ;
        INT8U	  U16Tmp;
        float	  fTmp;
        int     i;
        INT8U	  temp123;

        addrADCS=ADCS_REALTIME_DATA ;

        /*
        ////printf(" for WOD test, results are: \n" );
        ////printf(" %x \n", WorkMode );
        ////printf(" %x \n", CMD_FILTER );
        ////printf(" %x \n", CMD_CONTROL );
        ////printf(" %x \n", Period_Nom);
        ////printf(" %x \n", TlmCANOnOff);
        ////printf(" %x \n\n", TlmSource );


        ////printf(" %x \n",RPY[0] );
        ////printf(" %x \n",RPY[1]);
        ////printf(" %x \n",RPY[2]);




        ////printf(" %x \n",tcMWy_ISET_MON );
        ////printf(" %x \n",tcMWy_I_MON);
        ////printf(" %x \n",tlmMWyTem);

        ////printf(" %x \n",Data_Alpha );
        ////printf(" %x \n",Data_Beta);

        ////printf(" %x \n",tlmMCU1ASSPx);
        ////printf(" %x \n",tlmMCU1ASSPy);
        ////printf(" %x \n",tlmMCU1ASSNx);
        ////printf(" %x \n",tlmMCU1ASSNy);
        ////printf(" %x \n",tlmMCU1ASSTs);

*/


        OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);

        *(volatile INT16U *)(addrADCS) = ((WorkMode&0x00ff)<<8) | ((WorkMode>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((Filter&0x00ff)<<8) | ((Filter>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((Control&0x00ff)<<8) | ((Control>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((Period_Nom&0x00ff)<<8) | ((Period_Nom>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((TlmSource&0x00ff)<<8) | ((TlmSource>>8)&0x00ff);
        addrADCS += 2;

        //滚动、俯仰、偏航角，起始于X+13
        fTmp = (float)RPY[0];  addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)RPY[1];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)RPY[2];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        //X、Y、Z轴角速率，起始于X+25
        fTmp = (float)X[4];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)X[5];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)X[6];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        //    *(volatile INT16U *)(addrADCS) = ((Pitch_Ref&0x00ff)<<8) | ((Pitch_Ref>>8)&0x00ff);
        //    addrADCS += 2;

        //俯仰角及俯仰角速率，X+39
        fTmp = (float)Y[0];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)Y[1];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        //姿态四元数，X+47
        fTmp = (float)X[0];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)X[1];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)X[2];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)X[3];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        // GPS weeks and seconds，X+63
        U16Tmp	= (INT16U)( (gps_weeks_ADCS&0xffff0000)>>16 );
        *(volatile INT16U *)(addrADCS) = ((U16Tmp&0x00ff)<<8) | ((U16Tmp>>8)&0x00ff);
        addrADCS += 2;
        U16Tmp	= (INT16U)(  gps_weeks_ADCS&0x0000ffff );
        *(volatile INT16U *)(addrADCS) = ((U16Tmp&0x00ff)<<8) | ((U16Tmp>>8)&0x00ff);
        addrADCS += 2;
        U16Tmp	= (INT16U)( (gps_seconds_ADCS&0xffff0000)>>16 );
        *(volatile INT16U *)(addrADCS) = ((U16Tmp&0x00ff)<<8) | ((U16Tmp>>8)&0x00ff);
        addrADCS += 2;
        U16Tmp	= (INT16U)(  gps_seconds_ADCS&0x0000ffff );
        *(volatile INT16U *)(addrADCS) = ((U16Tmp&0x00ff)<<8) | ((U16Tmp>>8)&0x00ff);
        addrADCS += 2;

        //轨道位置矢量ECEF，X+71
        fTmp = (float)R_ECEF[0];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)R_ECEF[1];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)R_ECEF[2];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        //轨道速度矢量ECI，X+83
        fTmp = V_ECI[0];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = V_ECI[1];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = V_ECI[2];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        //±5V，±12V电流电压等，X+95
        if( TlmSource == 1 )
        {
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1P5VMon&0x00ff)<<8) | ((tlmMCU1P5VMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1P5VIMon&0x00ff)<<8) | ((tlmMCU1P5VIMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1P12VMon&0x00ff)<<8) | ((tlmMCU1P12VMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1P12VIMon&0x00ff)<<8) | ((tlmMCU1P12VIMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1N12VMon&0x00ff)<<8) | ((tlmMCU1N12VMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1N12VIMon&0x00ff)<<8) | ((tlmMCU1N12VIMon>>8)&0x00ff);
                addrADCS += 2;
        }
        else
        {
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2P5VMon&0x00ff)<<8) | ((tlmMCU1P5VMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2P5VIMon&0x00ff)<<8) | ((tlmMCU1P5VIMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2P12VMon&0x00ff)<<8) | ((tlmMCU1P12VMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2P12VIMon&0x00ff)<<8) | ((tlmMCU1P12VIMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2N12VMon&0x00ff)<<8) | ((tlmMCU1N12VMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2N12VIMon&0x00ff)<<8) | ((tlmMCU1N12VIMon>>8)&0x00ff);
                addrADCS += 2;
        }
        //板上温度、MT参考电压、MCU1/MCU2参考电压
        if( TlmSource == 1 )
        {
                *(volatile INT16U *)(addrADCS)=((tlmMCU1TsVref&0x00ff)<<8) | ((tlmMCU1TsVref>>8)&0x00ff);
                addrADCS += 2;
        }
        else
        {
                *(volatile INT16U *)(addrADCS)=((tlmMCU2TsVref&0x00ff)<<8) | ((tlmMCU2TsVref>>8)&0x00ff);
                addrADCS += 2;
        }
        if( TlmSource == 1 )
        {
                *(volatile INT16U *)(addrADCS)=((tlmMCU1MTVref&0x00ff)<<8) | ((tlmMCU1MTVref>>8)&0x00ff);
                addrADCS += 2;
        }	
        else
        {
                *(volatile INT16U *)(addrADCS)=((tlmMCU2MTVref&0x00ff)<<8) | ((tlmMCU2MTVref>>8)&0x00ff);
                addrADCS += 2;
        }    
        *(volatile INT16U *)(addrADCS)=((tlmMCU1Vref&0x00ff)<<8) | ((tlmMCU1Vref>>8)&0x00ff);
        addrADCS += 2;


        *(volatile INT16U *)(addrADCS)=((tlmMCU2Vref&0x00ff)<<8) | ((tlmMCU2Vref>>8)&0x00ff);
        addrADCS += 2; 

        //磁强计三轴修正值，X+125
        *(volatile INT16U *)(addrADCS) = ((MMCorrectX&0x00ff)<<8) | ((MMCorrectX>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((MMCorrectY&0x00ff)<<8) | ((MMCorrectY>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((MMCorrectZ&0x00ff)<<8) | ((MMCorrectZ>>8)&0x00ff);
        addrADCS += 2;

        //解算磁场值，X+131
        fTmp = (float)BfLO[0];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)BfLO[1];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = (float)BfLO[2];	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;



        //磁强计1三轴测量值，X+143
        *(volatile INT16U *)(addrADCS) = ((tlmMCU1MM1x&0x00ff)<<8) | ((tlmMCU1MM1x>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((tlmMCU1MM1y&0x00ff)<<8) | ((tlmMCU1MM1y>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((tlmMCU1MM1z&0x00ff)<<8) | ((tlmMCU1MM1z>>8)&0x00ff);
        addrADCS += 2;


        //        *(volatile INT16U *)(addrADCS) = ((tlmMCU1MM1Ts&0x00ff)<<8) | ((tlmMCU1MM1Ts>>8)&0x00ff);
        //        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((tlmMCU1MMS&0x00ff)<<8) | ((tlmMCU1MMS>>8)&0x00ff);
        addrADCS += 2;
        //        *(volatile INT16U *)(addrADCS) = ((tlmMCU1MMSTs&0x00ff)<<8) | ((tlmMCU1MMSTs>>8)&0x00ff);
        //        addrADCS += 2;

        //三轴磁力矩器控制命令，X+161
        *(volatile INT16U *)(addrADCS) = ((tcMCU2MTPWMx&0x00ff)<<8) | ((tcMCU2MTPWMx>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((tcMCU2MTPWMy&0x00ff)<<8) | ((tcMCU2MTPWMy>>8)&0x00ff);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = ((tcMCU2MTPWMz&0x00ff)<<8) | ((tcMCU2MTPWMz>>8)&0x00ff);
        addrADCS += 2;

        //MTX,MTY,MTZ电流，X+167
        if( TlmSource == 1 )
        {
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1MT1IMon&0x00ff)<<8) | ((tlmMCU1MT1IMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1MT2IMon&0x00ff)<<8) | ((tlmMCU1MT2IMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1MT3IMon&0x00ff)<<8) | ((tlmMCU1MT3IMon>>8)&0x00ff);
                addrADCS += 2;
        }
        else
        {
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2MT1IMon&0x00ff)<<8) | ((tlmMCU2MT1IMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2MT2IMon&0x00ff)<<8) | ((tlmMCU2MT2IMon>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2MT3IMon&0x00ff)<<8) | ((tlmMCU2MT3IMon>>8)&0x00ff);
                addrADCS += 2;
        }
        //动量轮额定转速

        fTmp = (float)MW_RPM_Rating;  addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;


        //动量轮转速控制命令，X+175

        *(volatile INT16U *)(addrADCS) = ((tcMWRPMy&0x00ff)<<8) | ((tcMWRPMy>>8)&0x00ff);
        addrADCS += 2;

        //动量轮转速测量值，X+181

        *(volatile INT16U *)(addrADCS) = ((tlmMWRPMy&0x00ff)<<8) | ((tlmMWRPMy>>8)&0x00ff);
        addrADCS += 2;

        //动量轮控制电流设置值，X+187
        /*
         *(volatile INT16U *)(addrADCS) = ((tcMWy_ISET_MON&0x00ff)<<8) | ((tcMWy_ISET_MON>>8)&0x00ff);
         addrADCS += 2;


        //动量轮控制电流监测值I_MON，X+193

         *(volatile INT16U *)(addrADCS) = ((tcMWy_I_MON&0x00ff)<<8) | ((tcMWy_I_MON>>8)&0x00ff);
         addrADCS += 2;


        //动量轮温度测量值,X+199

         *(volatile INT16U *)(addrADCS) = ((tlmMWyTem&0x00ff)<<8) | ((tlmMWyTem>>8)&0x00ff);
         addrADCS += 2;
         */



        //太阳敏±X，±Y测量值和模拟太阳敏温度，X+213
        if( TlmSource == 1 )
        {
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1ASSPx&0x00ff)<<8) | ((tlmMCU1ASSPx>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1ASSPy&0x00ff)<<8) | ((tlmMCU1ASSPy>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1ASSNx&0x00ff)<<8) | ((tlmMCU1ASSNx>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1ASSNy&0x00ff)<<8) | ((tlmMCU1ASSNy>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU1ASSTs&0x00ff)<<8) | ((tlmMCU1ASSTs>>8)&0x00ff);
                addrADCS += 2;
        }
        else
        {
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2ASSPx&0x00ff)<<8) | ((tlmMCU1ASSPx>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2ASSNx&0x00ff)<<8) | ((tlmMCU1ASSNx>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2ASSPy&0x00ff)<<8) | ((tlmMCU1ASSPy>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2ASSNy&0x00ff)<<8) | ((tlmMCU1ASSNy>>8)&0x00ff);
                addrADCS += 2;
                *(volatile INT16U *)(addrADCS) = ((tlmMCU2ASSTs&0x00ff)<<8) | ((tlmMCU1ASSTs>>8)&0x00ff);
                addrADCS += 2;
        }

        //alpha角度和beta角度,X+209
        fTmp = Data_Alpha;	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;
        fTmp = Data_Beta;	addrTmp = (INT32U) &(fTmp);
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp)<<8) & 0xff00);
        addrADCS += 2;
        *(volatile INT16U *)(addrADCS) = (((INT16U)*(INT32U *)(addrTmp-2)>>8)&0x00ff) | (((INT16U)*(INT32U *)(addrTmp-2)<<8) & 0xff00);
        addrADCS += 2;

        OSSemPost(SHARE_PRINTF);

        return( 0 );
}


//遥测MCU1，2数据
int	TLM_MCUData( void )
{
        short	iTmp = 0;

        //alpha,beta 中间量
        float	Xzb = 0;
        float	Yzb = 0;

        //TLM to Reaction Wheel
        //tlmMWRPMy	= 0;
        //tlmMWRPMyDr	= 0;
        //tlmMWyAnt	= 0;
        //tlmMWyAnri	= 0;
        //tlmMWyAnti	= 0;

        // TLM to MCU1 ( 26 variables )
        tlmMCU1MM1x		= 0;
        tlmMCU1MM1y		= 0;
        tlmMCU1MM1z		= 0;
        tlmMCU1MM1Ts=0;
        tlmMCU1ASSPx= 0;
        tlmMCU1ASSNx= 0;
        tlmMCU1ASSPy= 0;
        tlmMCU1ASSNy= 0;
        tlmMCU1ASSTs= 0;
        tlmMCU1MT1IMon= 0;
        tlmMCU1MT2IMon= 0;
        tlmMCU1MT3IMon= 0;
        tlmMCU1MTVref= 0;
        tlmMCU1Vref= 0;
        tlmMCU1P5VMon= 0;
        tlmMCU1P5VIMon= 0;
        tlmMCU1P12VMon= 0;
        tlmMCU1P12VIMon= 0;
        tlmMCU1N12VMon= 0;
        tlmMCU1N12VIMon= 0;
        tlmMCU1GYRO= 0;
        tlmMCU1TsVref= 0;


        // TLM to MCU2 ( 20 variables )

        tlmMCU2MMS= 0;
        tlmMCU2MMSTs= 0;
        tlmMCU2ASSPx= 0;
        tlmMCU2ASSNx= 0;
        tlmMCU2ASSPy= 0;
        tlmMCU2ASSNy= 0;
        tlmMCU2ASSTs= 0;
        tlmMCU2MT1IMon= 0;
        tlmMCU2MT2IMon= 0;
        tlmMCU2MT3IMon= 0;
        tlmMCU2MTVref= 0;
        tlmMCU2Vref= 0;
        tlmMCU2P5VMon= 0;
        tlmMCU2P5VIMon= 0;
        tlmMCU2P12VMon= 0;
        tlmMCU2P12VIMon= 0;
        tlmMCU2N12VMon= 0;
        tlmMCU2N12VIMon= 0;
        tlmMCU2GYRO= 0;
        tlmMCU2TsVref= 0;





        if( TlmSource == 1 )
        {

                if( iTmp==0 )	{ if( TLM_MCU1Whole() != 0 )		        iTmp = 1;	}

        }
        else if( TlmSource == 2 )
        {

                if( iTmp==0 )	{ if( TLM_MCU2Whole() != 0 )		        iTmp = 1;	}
        }

        Xzb = (((tlmMCU1ASSPx & 0x3FFF) + (tlmMCU1ASSNy & 0x3FFF) - (tlmMCU1ASSNx & 0x3FFF) - (tlmMCU1ASSPy & 0x3FFF))*1.0 / (1.0*(tlmMCU1ASSPx & 0x3FFF) + 1.0*(tlmMCU1ASSNx & 0x3FFF) + (tlmMCU1ASSNy && 0x3FFF) + (tlmMCU1ASSPy & 0x3FFF))) * 7000.0 ;

        Data_Alpha = atan (Xzb/2800);


        Yzb = (((tlmMCU1ASSPx & 0x3FFF) + (tlmMCU1ASSPy & 0x3FFF) - (tlmMCU1ASSNx & 0x3FFF) - (tlmMCU1ASSNy & 0x3FFF))*1.0 / (1.0*(tlmMCU1ASSPx & 0x3FFF) + 1.0*(tlmMCU1ASSNx & 0x3FFF) + (tlmMCU1ASSNy && 0x3FFF) + (tlmMCU1ASSPy & 0x3FFF))) * 7000.0 ;

        Data_Beta = atan (Yzb/2800);


        /*	////printf("Xzb is %d\n", Xzb*10000);
        ////printf("Yzb is %d\n", Yzb*10000);
        ////printf("Data_Alpha is %d\n", Data_Alpha*10000);
        ////printf("Data_Beta is %d\n", Data_Beta*10000);
        */
        return( iTmp );
}


short ByteToShort(INT8U* pc)
{
        short ret = 0;


        ret = (*(pc + 1)) * 256 + (*(pc + 2));


        if ((*(pc )) == 1)
        {
                ret = 0 - ret;
        }

        return ret;
}



float ByteToFloat(INT8U* pc)
{
        real_f value_f;
        value_f.ui[0]= *pc; 
        value_f.ui[1]= *(pc + 1);
        value_f.ui[2]= *(pc + 2);
        value_f.ui[3]= *(pc + 3);

        return value_f.f;
}

//extern INT8U ERR_SHARE_PRINTF;
//extern OS_EVENT *SHARE_PRINTF;
double ByteToDouble(const INT8U* pc, int n)
{
        real_d value_d;
        value_d.un[0]= *(pc + 7+ 8*n);
        value_d.un[1]= *(pc + 6+ 8*n);
        value_d.un[2]= *(pc + 5+ 8*n);
        value_d.un[3]= *(pc + 4+ 8*n);
        value_d.un[4]= *(pc + 3+ 8*n);
        value_d.un[5]= *(pc + 2+ 8*n);
        value_d.un[6]= *(pc + 1+ 8*n);
        value_d.un[7]= *(pc + 0+ 8*n);
        return value_d.dbl;
}



void	ADCS_Main(void *jdata)  //姿控主程序
{
        int		is_boundary;			/* is it Period_Nom boundary?    */
        int		adcs_err;
        INT8U           err;
        INT8U	        err_tmp, flag_timer;
        INT8U	        cmd_addr;
        short	        cmd_value1,cmd_value2;
        INT8U           *pQADCS;
        //        INT8U 		temppp;



        INT8U *asmsg1;
        INT8U  err1, j; 
        /*===============================*
         * 		  初始化、载入配置文件	   *
         *===============================*/

        ADCS_Initialise();
        ADCS_LoadCfgFile();

        TLM_ADopen();

        //memset((volatile INT8U *)ADCS_REALTIME_DATA,0,1044480);

        ////printf("segment 1 \n");

        while (1)
        {
                OSMboxPend( ADCS_FLAG, 0, &flag_timer );
                ////printf("ADCS_FLAG\n");
                if( flag_timer==OS_NO_ERR )
                {
                        /*===============================*
                         * 		   处理地面遥控指令	       *
                         *===============================*/
                        while(pQADCS = (INT8U *)OSQAccept(ACSQCOM,&err))
                        {

                                ////printf("RRRRR\n");
                                cmd_addr	= *pQADCS;		pQADCS ++;                                
                                cmd_value2	= *pQADCS;  pQADCS ++;
                                if(cmd_value2==1)				cmd_value1	= *pQADCS;
                                if(cmd_value2==2)			cmd_value1=ByteToShort(pQADCS);
                                if(cmd_value2==3)			cmd_value1=ByteToFloat(pQADCS);



                                ////printf("zhiqian    %d\n",WorkMode);
                                ////printf("cmd_addr	=%x\n",cmd_addr	);
                                ////printf("cmd_value1	=%d\n",cmd_value1	);
                                ////printf("cmd_value2	=%x\n",cmd_value2	);

                                if( (err_tmp=CmdDisposal( cmd_addr, cmd_value1, cmd_value2 )) != 0 )
                                {
                                        ;
                                }
                                ////printf("zhihou    %d\n",WorkMode);
                        }

                        ////printf("SSSSSSSSSSSSSSS start\n");
                        ////printf ("ACSFL_Flag = %d\n", ACSFL_Flag);
                        ADCS_CfgUpdate();


                        ////printf("segment 2 \n");	    

                        if(ADCS_OnOff==ON)
                        {
                                /*=============================================*
                                 * 		进行时间的获取，判断是否到一个控制周期	 *
                                 *=============================================*/
                                is_boundary=0;			/* is it Period_Nom boundary?    */
                                adcs_err=0;

                                seconds80	= gps_week * 604800 + gps_second;
                                gps_weeks_ADCS=gps_week;
                                gps_seconds_ADCS=gps_second;
                                ////printf ("gps_weeks_ADCS   : %d \n", gps_weeks_ADCS);
                                ////printf ("gps_seconds_ADCS   : %d \n", gps_seconds_ADCS);

                                //	   seconds80= gps_weeks_ADCS * 604800 + gps_seconds_ADCS;

                                if( (seconds80!=seconds80_Prev) && (seconds80%(Period_Nom)==0) )
                                {
                                        is_boundary	= 1;
                                }

                                ////printf("is_boundary  is %d\n",is_boundary);
                                //temppp = ((seconds80-seconds80_Prev) >= Period_Nom);
                                ////printf("Period_Nom is %d\n", temppp);
                                ////printf("seconds80 is %d\n", seconds80);
                                ////printf("seconds80_Prev is %d\n", seconds80_Prev);
                                ////printf("segment 3 \n");


                                if( is_boundary || ( (seconds80-seconds80_Prev) >= Period_Nom) )
                                {
                                        OSQFlush( CAN_Q_Rec );	// Clear the CAN buf

                                        //  ////printf("WorkMode    %d\n",WorkMode);

                                        if ( (err_tmp=TLM_MCUData() ) != 0 )	{
                                                ;
                                        }//获取MCU遥测数据
                                        //////printf("segment 4 \n");			


                                        Period_True	= (double) (seconds80 - seconds80_Prev);
                                        seconds80_Prev	= seconds80;

                                        /*===============================*
                                         * 		 根据不同工作模式动作      *
                                         *===============================*/
                                        if( WorkMode == 3 )
                                        { 

                                                if(CMD_MT_CONTROL_MODE==0){

                                                        TC_MCU2MTCTL1( );
                                                        TC_MCU2MTCTL1( );
                                                }
                                                if(CMD_MT_CONTROL_MODE==1){

                                                        TC_MCU2MTPWM( );
                                                        TC_MCU2MTPWM( );
                                                }

                                                TC_MWRPMy( ) ;

                                                Filter=CMD_FILTER;

                                        }
                                        else 
                                        {
                                                if(WorkMode == 2)
                                                {
                                                        ADCS_AutoRunning( );
                                                }
                                                if(WorkMode == 1)
                                                {
                                                        Filter=CMD_FILTER;
                                                        Control=CMD_CONTROL;
                                                }

                                                if( adcs_err == 0 )
                                                {
                                                        adcs_err = ADCS_Control( );
                                                        TC_Actuator( );
                                                        tcMCU2MTPWMx	= MT_PWMCmd16[0];
                                                        tcMCU2MTPWMy	= MT_PWMCmd16[1];
                                                        tcMCU2MTPWMz	= MT_PWMCmd16[2];
                                                        tcMWRPMy	= MW_RPMCmd16;

                                                        if(Control>3)			TC_MWRPMy();

                                                        if(CMD_MT_CONTROL_MODE==0){

                                                                TC_MCU2MTCTL1( );
                                                                TC_MCU2MTCTL1( );
                                                        }
                                                        if(CMD_MT_CONTROL_MODE==1){

                                                                TC_MCU2MTPWM( );
                                                                TC_MCU2MTPWM( );
                                                        }

                                                        //////printf("Display Control valua:%d\n",Control);
                                                        // if( adcs_err )	////printf( "ADCS_Control() error: %d\n",adcs_err );
                                                }

                                        }
                                        //////printf("segment 3.5 \n");

                                        /*===============================*
                                         * 	写遥测数据，MM、MW遥测量获取 *
                                         *===============================*/
                                        WriteTlmData();

                                        //////printf("segment 5 \n");

                                        if(Control>=C4){

                                                TLM_MWRPMy() ;
                                        }
                                        MM_Tlm16[0]	= tlmMCU1MM1x+MMCorrectX;
                                        MM_Tlm16[1]	= tlmMCU1MM1y+MMCorrectY;
                                        MM_Tlm16[2]	= tlmMCU1MM1z+MMCorrectZ;
                                        MW_RPMTlm16	= tlmMWRPMy;

                                        //////printf("segment 6 \n");


                                        /*===============================*
                                         * 		模型与滤波解算	 *
                                         *===============================*/
                                        if( Filter >= PITCH_FILTER )
                                        {
                                                adcs_err = ADCS_Models( );
                                                //        if( adcs_err )	////printf( "ADCS_Models() error: %d\n",adcs_err );
                                        }

                                        if( adcs_err == 0 )
                                        {
                                                adcs_err = ADCS_Filter( );
                                                //////printf("Display Filter valua:%d\n",Filter);
                                                //        if( adcs_err )	////printf( "ADCS_Filter() error: %d\n",adcs_err );
                                        }
                                        //X4=X[4];
                                        //X5=X[5];
                                        //X6=X[6]; //仿真用
                                        //////printf("segment 7 \n");	
                                        ////printf("MM_Tlm value(nT):%d, %d, %d \n",tlmMCU1MM1x,tlmMCU1MM1y,tlmMCU1MM1z);	
                                        ////printf("BfMM value(nT):%d, %d, %d \n",(int)(BfMM[0]*1000),(int)(BfMM[1]*1000),(int)(BfMM[2]*1000));	


                                        //仿真CAN口传送用，给赵鹏飞环境仿真软件

                                        /*                                        
                                                                                  obctozpf1=0xffff;
                                                                                  obctozpf2=0xffff;
                                                                                  obctozpf3=0xffff;
                                                                                  OBC_ZPF (obctozpf1,obctozpf2,obctozpf3);

                                                                                  obctozpf1=tcMCU2MTPWMx&0x00ff;
                                                                                  obctozpf2=tcMCU2MTPWMy&0x00ff;
                                                                                  obctozpf3=tcMCU2MTPWMz&0x00ff;
                                                                                  OBC_ZPF (obctozpf1,obctozpf2,obctozpf3);

                                        //////printf( "MTX = %x \n",tcMCU2MTPWMx);
                                        //////printf( "MTY = %x \n",tcMCU2MTPWMy);
                                        //////printf( "MTZ = %x \n",tcMCU2MTPWMz);

                                        obctozpfdouble=zpf_N[0];
                                        OBC_ZPF4_1 (obctozpfdouble);

                                        obctozpfdouble=zpf_N[0];
                                        OBC_ZPF4_2 (obctozpfdouble);

                                        obctozpfdouble=zpf_N[1];
                                        OBC_ZPF4_1 (obctozpfdouble);

                                        obctozpfdouble=zpf_N[1];
                                        OBC_ZPF4_2 (obctozpfdouble);

                                        obctozpfdouble=zpf_N[2];
                                        OBC_ZPF4_1 (obctozpfdouble);

                                        obctozpfdouble=zpf_N[2];
                                        OBC_ZPF4_2 (obctozpfdouble);

                                        obctozpf1=(short)(Y0*1000);
                                        obctozpf2=tlmMWRPMy;
                                        obctozpf3=(short)(Y1*1000);

                                        //   obctozpf1=(short)(X4*10000);
                                        //   obctozpf2=(short)(X5*10000);
                                        //   obctozpf3=(short)(X6*10000);

                                        OBC_ZPF (obctozpf1,obctozpf2,obctozpf3);

                                        //////printf( "MWY = %x \n",tcMWRPMy);

                                        obctozpf1c=0;
                                        obctozpf2c=tlmMWRPMyDr;
                                        obctozpf3c=0;
                                        obctozpf4c=Filter;
                                        obctozpf5c=Control;
                                        obctozpf6c=0xff;
                                        OBC_ZPF2 (obctozpf1c,obctozpf2c,obctozpf3c,obctozpf4c,obctozpf5c,obctozpf6c);

                                        obctozpflong=gps_seconds_ADCS;
                                        OBC_ZPF3 (obctozpflong);

                                        obctozpflong=gps_weeks_ADCS;
                                        OBC_ZPF3 (obctozpflong);

                                        obctozpf1=0xeeee;
                                        obctozpf2=0xeeee;
                                        obctozpf3=0xeeee;
                                        OBC_ZPF (obctozpf1,obctozpf2,obctozpf3);

*/


                                        //////printf("segment 9 \n");

                                }//end of is_boundary
                        }//end of ADCS_OnOff==ON

                }//end of flag_timer==OS_NO_ERR

        }//end of while (1)

}


