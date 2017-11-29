#include "ucos_ii.h"
/*=============================================================================
	程序名: ADCS_TTC.c <NS-2(MEMSst)ADCS—CAN通讯软件 头文件> 
	编 辑	: 师帅
	日 期 : 20150715
=============================================================================*/
#ifndef __adcsrxflow_defined
#define __adcsrxflow_defined
typedef struct
{
		INT32U va;
		INT32U id;
		INT32U len;
		INT8U  d[110];
}	ADCSrxflow;
#endif
float   u[25];



/* CAN */
//TC to Reaction Wheel
int            TC_MWRPMx( void );
int            TC_MWRPMy( void );
int            TC_MWRPMz( void );
int            TC_MWPIx( void );
int            TC_MWPIy( void );
int            TC_MWPIz( void );
//TC to MCU1 ( 2 functions )
int            TC_MCU1SampleMode(void);
int            TC_MCU1Power(void);
int            TC_MCU1PowerMW(void);
// TC to MCU2 ( 3 functions )
int            TC_MCU2SampleMode(void);
int            TC_MCU2MTCTL1(void);
int            TC_MCU2MTPWM(void);
//TLM to Reaction Wheel
int            TLM_MWRPMx( void );
int            TLM_MWRPMy( void );
int            TLM_MWRPMz( void );
int            TLM_MWxANtii( void);
int            TLM_MWyANtii( void);
int            TLM_MWzANtii( void);
// TLM to MCU1 ( 5 function )
int            TLM_MCU1MM1( void );
int            TLM_MCU1MMS( void );
int            TLM_MCU1ASS( void );
int            TLM_MCU1VIMon_Ref( void );
int            TLM_MCU1Whole( void );
// TLM to MCU2 ( 5 function )

int            TLM_MCU2MM2( void );
int            TLM_MCU2MMS( void );
int            TLM_MCU2ASS( void );
int            TLM_MCU2VIMon_Ref( void );
int            TLM_MCU2Whole( void );


/* *_S for Serial */
//TC to Reaction Wheel
int            TC_MWRPMx_S( void );
int            TC_MWRPMy_S( void );
int            TC_MWRPMz_S( void );
int            TC_MWPIx_S( void );
int            TC_MWPIy_S( void );
int            TC_MWPIz_S( void );
//TC to MCU1 ( 3 functions )
int            TC_MCU1SampleMode_S(void);
int            TC_MCU1Power_S(void);
int            TC_MCU1MCUswitch_S();
// TC to MCU2 ( 3 functions )
int            TC_MCU2SampleMode_S(void);
int            TC_MCU2MTCTL1_S(void);
int            TC_MCU2MTPWM_S(void);
int            TC_MCU2MCUswitch_S();
//TLM to Reaction Wheel
int            TLM_MWRPMx_S( void );
int            TLM_MWRPMy_S( void );
int            TLM_MWRPMz_S( void );
int            TLM_MWxANtii_S( void);
int            TLM_MWyANtii_S( void);
int            TLM_MWzANtii_S( void);
// TLM to MCU1 ( 5 function )
int            TLM_MCU1MM1_S( void );
int            TLM_MCU1MMS_S( void );
int            TLM_MCU1ASS_S( void );
int            TLM_MCU1VIMon_Ref_S( void );
int            TLM_MCU1Whole_S( void );
// TLM to MCU2 ( 5 function )

int            TLM_MCU2MM2_S( void );
int            TLM_MCU2MMS_S( void );
int            TLM_MCU2ASS_S( void );
int            TLM_MCU2VIMon_Ref_S( void );
int            TLM_MCU2Whole_S( void );


int test_with_xc(void);

/*
//2012.3.12 ADCS开关CAN控制新协议函数声明//
int TC_ADCS_MCU1ASS_POWER( void );
int TC_ADCS_MCU1MM1_POWER( void );
int TC_ADCS_MCU1MM2_POWER( void );
int TC_ADCS_MCU1MMS_POWER( void );
int TC_ADCS_MCU1MWx_POWER( void );
int TC_ADCS_MCU1MWy_POWER( void );
int TC_ADCS_MCU1MWz_POWER( void );
short TC_ADCS_POWER;
*/
int OBC_ZPF (short zpf1, short zpf2, short zpf3);
int OBC_ZPF2 (char zpf1c, char zpf2c, char zpf3c, char zpf4c, char zpf5c, char zpf6c);
int OBC_ZPF3 (unsigned long zpflong);
int OBC_ZPF4_1 (double zpfdouble);
int OBC_ZPF4_2 (double zpfdouble);
