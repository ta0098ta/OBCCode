#include "ADCS_TTC.h"
#include <math.h>
#include <string.h>
/*=============================================================================
  程序名: ADCS_TTC.c <NS-2(MEMSst)ADCS—CAN通讯软件> 
  编 辑	: 师帅
  日 期 : 20150715
  =============================================================================*/
extern OS_EVENT* CAN_Q_Rec;
extern OS_EVENT* CAN_Q_Rec_DLL;
INT8U  m_err,m_S,m_A,m_C,m_U16Tmp;
// TC to Reaction Wheel
short    tcMWRPMy	= 0;
short    tcMWRPMyDr	= 0x10;
short    tcMW_kpy	= 0;
short    tcMW_kiy	= 0;
// TC to MCU1 
short    tcMCU1SampleMode=0x05;

// TC to MCU2 
short    tcMCU2SampleMode=0x07;
short    tcMCU2MT_CTLx   =0;
short    tcMCU2MT_CTLy   =0;
short    tcMCU2MT_CTLz   =0;
short    tcMCU2MTPWMx	= 0;
short    tcMCU2MTPWMy	= 0;
short    tcMCU2MTPWMz	= 0;

//TLM to Reaction Wheel
short    tlmMWRPMy;
short    tlmMWRPMyDr;
short    tlmMWyAnt;
short    tlmMWyAnri;
short    tlmMWyAnti;

// TLM to MCU1 ( 26 variables )
short    tlmMCU1MM1x;
short    tlmMCU1MM1y;
short    tlmMCU1MM1z;
short    tlmMCU1MM1Ts;
short    tlmMCU1MMS;
short    tlmMCU1MMSTs;
short    tlmMCU1ASSPx;
short    tlmMCU1ASSNx;
short    tlmMCU1ASSPy;
short    tlmMCU1ASSNy;
short    tlmMCU1ASSTs;
short    tlmMCU1MT1IMon;
short    tlmMCU1MT2IMon;
short    tlmMCU1MT3IMon;
short    tlmMCU1MTVref;
short    tlmMCU1Vref;
short    tlmMCU1P5VMon;
short    tlmMCU1P5VIMon;
short    tlmMCU1P12VMon;
short    tlmMCU1P12VIMon;
short    tlmMCU1N12VMon;
short    tlmMCU1N12VIMon;
short    tlmMCU1GYRO;
short    tlmMCU1TsVref;

// TLM to MCU2 ( 24 variables )
short    tlmMCU2MM2x;
short    tlmMCU2MM2y;
short    tlmMCU2MM2z;
short    tlmMCU2MM2Ts;
short    tlmMCU2MMS;
short    tlmMCU2MMSTs;
short    tlmMCU2ASSPx;
short    tlmMCU2ASSNx;
short    tlmMCU2ASSPy;
short    tlmMCU2ASSNy;
short    tlmMCU2ASSTs;
short    tlmMCU2MT1IMon;
short    tlmMCU2MT2IMon;
short    tlmMCU2MT3IMon;
short    tlmMCU2MTVref;
short    tlmMCU2Vref;
short    tlmMCU2P5VMon;
short    tlmMCU2P5VIMon;
short    tlmMCU2P12VMon;
short    tlmMCU2P12VIMon;
short    tlmMCU2N12VMon;
short    tlmMCU2N12VIMon;
short    tlmMCU2GYRO;
short    tlmMCU2TsVref;

extern short   	CMD_MWY_ON_OFF; 
extern short     tcMW_kpY;  

/*
//用于仿真的CAN通讯部分，借用MIMU的通道，将星上时间、控制命令、滤波控制模式传输到模飞软件//
int OBC_ZPF (short zpf1,short zpf2,short zpf3)
{
        INT8U  CAN_data[20];
        INT8U  temp[8];
        INT32U tx_va;
        INT32U tx_type;
        temp[1]=(zpf1>>8)&0x00ff;
        temp[2]=zpf1&0x00ff;
        temp[3]=(zpf2>>8)&0x00ff;
        temp[4]=zpf2&0x00ff;
        temp[5]=(zpf3>>8)&0x00ff;
        temp[6]=zpf3&0x00ff;

        tx_va   = 0x1;
        tx_type =  0x5;         

        CAN_data[0] = temp[1];       
        CAN_data[1] = temp[2];      
        CAN_data[2] = temp[3];        
        CAN_data[3] = temp[4];
        CAN_data[4] = temp[5];
        CAN_data[5] = temp[6];
        CAN_data[6] = 0x00;
        CAN_data[7] = 0x00;
        CAN_data[8] = 0x00;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        return( 0 );
}

int OBC_ZPF2 (char zpf1c, char zpf2c, char zpf3c, char zpf4c, char zpf5c, char zpf6c)
{
        INT8U  CAN_data[20];
        INT8U  temp[8];
        INT32U tx_va;
        INT32U tx_type;
        temp[1]=zpf1c;
        temp[2]=zpf2c;
        temp[3]=zpf3c;
        temp[4]=zpf4c;
        temp[5]=zpf5c;
        temp[6]=zpf6c;

        tx_va   = 0x1;
        tx_type =  0x5;          

        CAN_data[0] = temp[1];       
        CAN_data[1] = temp[2];       
        CAN_data[2] = temp[3];        
        CAN_data[3] = temp[4];
        CAN_data[4] = temp[5];
        CAN_data[5] = temp[6];
        CAN_data[6] = 0x00;
        CAN_data[7] = 0x00;
        CAN_data[8] = 0x00;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        return( 0 );
}

int OBC_ZPF3 (unsigned long zpflong)
{
        INT8U  CAN_data[20];
        INT8U  temp[8];
        INT32U tx_va;
        INT32U tx_type;
        temp[1]=(zpflong>>24)&0x000000ff;
        temp[2]=(zpflong>>16)&0x000000ff;
        temp[3]=(zpflong>>8)&0x000000ff;
        temp[4]=zpflong&0x000000ff;
        temp[5]=0xff;
        temp[6]=0xff;

        tx_va   = 0x1;
        tx_type =  0x5;          

        CAN_data[0] = temp[1];       
        CAN_data[1] = temp[2];       
        CAN_data[2] = temp[3];        
        CAN_data[3] = temp[4];
        CAN_data[4] = temp[5];
        CAN_data[5] = temp[6];
        CAN_data[6] = 0x00;
        CAN_data[7] = 0x00;
        CAN_data[8] = 0x00;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        return( 0 );
}

int OBC_ZPF4_1 (double zpfdouble)
{
        INT8U  CAN_data[20];
        INT8U  temp[8];
        INT32U tx_va;
        INT32U tx_type;
        INT32U doublep;

        doublep=(INT32U)&(zpfdouble);

        temp[1]=*(volatile INT8U *)(doublep);
        temp[2]=*(volatile INT8U *)(doublep+1);
        temp[3]=*(volatile INT8U *)(doublep+2);
        temp[4]=*(volatile INT8U *)(doublep+3);
        temp[5]=0xff;
        temp[6]=0xff;

        tx_va   = 0x1;
        tx_type =  0x5;          

        CAN_data[0] = temp[1];      
        CAN_data[1] = temp[2];       
        CAN_data[2] = temp[3];        
        CAN_data[3] = temp[4];
        CAN_data[4] = temp[5];
        CAN_data[5] = temp[6];
        CAN_data[6] = 0x00;
        CAN_data[7] = 0x00;
        CAN_data[8] = 0x00;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        return( 0 );
}

int OBC_ZPF4_2 (double zpfdouble)
{
        INT8U  CAN_data[20];
        INT8U  temp[8];
        INT32U tx_va;
        INT32U tx_type;
        INT32U doublep;

        doublep=(INT32U)&(zpfdouble);

        temp[1]=*(volatile INT8U *)(doublep+4);
        temp[2]=*(volatile INT8U *)(doublep+5);
        temp[3]=*(volatile INT8U *)(doublep+6);
        temp[4]=*(volatile INT8U *)(doublep+7);
        temp[5]=0xff;
        temp[6]=0xff;

        tx_va   = 0x1;
        tx_type =  0x5;          

        CAN_data[0] = temp[1];      
        CAN_data[1] = temp[2];       
        CAN_data[2] = temp[3];        
        CAN_data[3] = temp[4];
        CAN_data[4] = temp[5];
        CAN_data[5] = temp[6];
        CAN_data[6] = 0x00;
        CAN_data[7] = 0x00;
        CAN_data[8] = 0x00;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        return( 0 );
}

*/


//		遥控动量轮转速（方向固定为正转）tcMWRPMy
int TC_MWRPMy( void )
{
        INT8U  CAN_data[20];
        INT32U tx_va;
        INT32U tx_type;
        tx_va   = 0x01;
        tx_type = 0x4;          /*即mode,即va*/
        //printf("ADCS CAN tcMWRPMy start:\n");

        CAN_data[0] = 0x01;      /*设置转速模式*/
        CAN_data[1] = tcMWRPMyDr;        /*转动方向*/
        CAN_data[2] = ( tcMWRPMy>>8 ) & 0x00ff;       /*high byte*/
        CAN_data[3] = ( tcMWRPMy    ) & 0x00ff;       /*low byte*/
        CAN_data[4] = 0x0;
        CAN_data[5] = 0x0;
        CAN_data[6] = 0x0;
        CAN_data[7] = 0x0;
        CAN_data[8] = (CAN_data[3]+CAN_data[4]+CAN_data[5]) & 0x00ff;

        can_datagram_send(CAN_data, tx_type, tx_va) ; 

        ////printf("TC_MWRPMy send!\n");
        ////printf("tcMWRPMy=%d\n",tcMWRPMy);

        return( 0 );
}

//遥控动量轮增速比例因子
int TC_MWPIy( void )
{
        INT8U  CAN_data[20];
        INT32U tx_va;
        INT32U tx_type;
        tx_va   = 0x01;
        tx_type = 0x4;          /*即mode,即va*/
        //printf("ADCS CAN tcMCU1MW_COEF_K1 start:\n");
        CAN_data[0] = 0x05;      /*设置动量轮加速度性能(比例因子)*/
        CAN_data[1] = tcMW_kpy;        /*转动方向*/
        CAN_data[2] =  0x0;       
        CAN_data[3] =  0x0;      
        CAN_data[4] = 0x0;
        CAN_data[5] = 0x0;
        CAN_data[6] = 0x11;
        CAN_data[7] = 0x0;
        CAN_data[8] = (CAN_data[3]+CAN_data[4]+CAN_data[5]) & 0x00ff;

        can_datagram_send(CAN_data, tx_type, tx_va);
        //printf("TC_MWPIy send!\n");
        //printf("tcMW_kpy=%d\n",tcMW_kpy);
        return( 0 );
}


/*
//TC to MCU1SampleMode
int TC_MCU1SampleMode(void)
{
INT8U  CAN_data[20];
INT32U tx_va;
INT32U tx_type;
tx_va   = 0x1;
tx_type = 0x1;          
//printf("ADCS CAN tcMCU1SampleMode start:\n");
CAN_data[0] = 0x01;       
CAN_data[1] = 0x01;       
CAN_data[2] = 0x01;        
CAN_data[3] = tcMCU1SampleMode;
CAN_data[4] = 0x0;
CAN_data[5] = 0x0;
CAN_data[6] = 0x0;
CAN_data[7] = 0x0;
CAN_data[8] = tcMCU1SampleMode;

//printf("can_send test return %x.\n",can_datagram_send(CAN_data, tx_type, tx_va) );
//printf("TC_MCU1SampleMode send!\n");
return( 0 );
}

// functions TC to MCU2

int TC_MCU2SampleMode(void)
{
        INT8U  CAN_data[20];
        INT32U tx_va;
        INT32U tx_type;
        tx_va   = 0x1;
        tx_type = 0x1;          
        //printf("ADCS CAN tcMCU2SampleMode start:\n");
        CAN_data[0] = 0x02;       
        CAN_data[1] = 0x01;       
        CAN_data[2] = 0x01;        
        CAN_data[3] = tcMCU2SampleMode;
        CAN_data[4] = 0x0;
        CAN_data[5] = 0x0;
        CAN_data[6] = 0x0;
        CAN_data[7] = 0x0;
        CAN_data[8] = tcMCU2SampleMode;

        //printf("can_send test return %x.\n",can_datagram_send(CAN_data, tx_type, tx_va) );
        //printf("TC_MCU2SampleMode send!\n");
        return( 0 );
}

*/
//遥控磁力矩器（常开常闭模式）
int TC_MCU2MTCTL1(void)
{
        INT8U  CAN_data[20];
        INT32U tx_va;
        INT32U tx_type;
        short tcMCU2MTx,tcMCU2MTy,tcMCU2MTz;
        tx_va   = 0x1;
        tx_type = 0x1;          /*即mode,即va*/

        ////printf("ADCS CAN tcMCU2MTCTL1 start:\n");
        if(tcMCU2MTPWMx<0){tcMCU2MTx=0x80-tcMCU2MTPWMx;}
        else {tcMCU2MTx=tcMCU2MTPWMx;}
        if(tcMCU2MTPWMy<0){tcMCU2MTy=0x80-tcMCU2MTPWMy;}
        else {tcMCU2MTy=tcMCU2MTPWMy;}
        if(tcMCU2MTPWMz<0){tcMCU2MTz=0x80-tcMCU2MTPWMz;}
        else {tcMCU2MTz=tcMCU2MTPWMz;}
        CAN_data[0] = 0x02;       /*id*/
        CAN_data[1] = 0x01;       /*len*/
        CAN_data[2] = 0x01;        /*type*/
        CAN_data[3] = 0x02;
        CAN_data[4] = tcMCU2MTy;
        CAN_data[5] = tcMCU2MTx;
        CAN_data[6] = tcMCU2MTz;
        CAN_data[7] = 0x0;
        CAN_data[8] = (0x02+tcMCU2MTx+tcMCU2MTy+tcMCU2MTz)&0x00ff;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        ////printf("TC_MCU2MTCTL1 send!\n");
        return( 0 );
}
//遥控磁力矩器（PWM模式）
int TC_MCU2MTPWM(void)
{
        INT8U  CAN_data[20];
        INT32U tx_va;
        INT32U tx_type;
        short tcMCU2MTx,tcMCU2MTy,tcMCU2MTz;
        tx_va   = 0x1;
        tx_type = 0x1;          /*即mode,即va*/
        ////printf("ADCS CAN tcMCU2MTPWM start:\n");
        if(tcMCU2MTPWMx<0){tcMCU2MTx=0x80-tcMCU2MTPWMx;}
        else {tcMCU2MTx=tcMCU2MTPWMx;}
        if(tcMCU2MTPWMy<0){tcMCU2MTy=0x80-tcMCU2MTPWMy;}
        else {tcMCU2MTy=tcMCU2MTPWMy;}
        if(tcMCU2MTPWMz<0){tcMCU2MTz=0x80-tcMCU2MTPWMz;}
        else {tcMCU2MTz=tcMCU2MTPWMz;}
        CAN_data[0] = 0x02;       /*id*/
        CAN_data[1] = 0x01;       /*len*/
        CAN_data[2] = 0x01;        /*type*/
        CAN_data[3] = 0x03;
        CAN_data[4] = tcMCU2MTy;
        CAN_data[5] = tcMCU2MTx;
        CAN_data[6] = tcMCU2MTz;
        CAN_data[7] = 0x0;
        CAN_data[8] = (0x03+tcMCU2MTx+tcMCU2MTy+tcMCU2MTz)&0x00ff;

        can_datagram_send(CAN_data, tx_type, tx_va) ;

        ////printf("TC_MCU2MTPWM send!\n");
        return( 0 );
}


////////////TLM to Reaction Wheel/////////////
//遥测动量轮转速

int TLM_MWRPMy( void )
{
        short mwrpm;
        INT8U  CAN_data[20];
        INT8U  err;
        INT8U  *m;
        INT32U tx_va,i;
        INT32U tx_type;
        ADCSrxflow rx;
        tx_va   = 0x1;
        tx_type = 0x4;          /*即mode,即va*/
        //printf("ADCS CAN TLM_MWRPMy start:\n");


        CAN_data[0] = 0x02;      
        CAN_data[1] = 0;       
        CAN_data[2] = 0;       
        CAN_data[3] = 0;      
        CAN_data[4] = 0;
        CAN_data[5] = 0x11;
        CAN_data[6] = 0x0;
        CAN_data[7] = 0x0;
        CAN_data[8] = (CAN_data[3]+CAN_data[4]+CAN_data[5]) & 0x00ff;
        can_datagram_send(CAN_data, tx_type, tx_va) ;
        ////printf("ssssssTLM_MWRPMy send !\n");

        m = (INT8U *)OSQPend(CAN_Q_Rec_DLL,1000,&err);

        if(err==OS_NO_ERR)
        {
                OSSchedLock();
                for (i=0;i<18;i++)
                {
                        rx.d[i]=*m;
                        m++;
                }

                OSSchedUnlock();

                //printf("m copyed!\n");
                //printf("rx.d=\n");
                for (i=0;i<18;i++)
                {
                        //printf(" %x",rx.d[i]);
                }
                //printf("\n");

                if(rx.d[0]==1)
                {
                        tlmMWRPMyDr = rx.d[11];
                        mwrpm=rx.d[12];
                        tlmMWRPMy   = (mwrpm<<8) + (rx.d[13]);
                        //printf("TLM_MWRPMy completed!\n");
                }
                //printf("tlmMWRPMyDr = %d \n",tlmMWRPMyDr);
                //printf("tlmMWRPMy = %d \n",tlmMWRPMy);
        }
        return( 0 );
}
//启动MCU1 A/D
int TLM_ADopen( void )
{

        INT8U  CAN_data[20];
        INT32U tx_va;
        INT32U tx_type;
        tx_va   = 0x1;
        tx_type = 0x1;          /*即mode,即va*/
        //printf("ADopen!\n");
        CAN_data[0] = 0x01;       /*id*/
        CAN_data[1] = 0x01;       /*len*/
        CAN_data[2] = 0x05;       /*type*/
        CAN_data[3] = 0x05;
        CAN_data[4] = 0x00;
        CAN_data[5] = 0x00;
        CAN_data[6] = 0x00;
        CAN_data[7] = 0x00;
        CAN_data[8] = 0x05;
        can_datagram_send(CAN_data, tx_type, tx_va);

        //printf("ADopen send!\n");
        return( 0 );
}
//MCU1全部遥测数据，按照ADCS_CAN通讯协议
int TLM_MCU1Whole( void )
{
        INT8U  CAN_data[20];
        INT8U  err;
        INT8U  *m;
        INT32U tx_va,i;
        INT32U tx_type;
        ADCSrxflow rx;
        tx_va   = 0x1;
        tx_type = 0x1;          /*即mode,即va*/
        ////printf("ADCS CAN TLM_MCU1Whole start:\n");

        CAN_data[0] = 0x01;       /*id*/
        CAN_data[1] = 0x01;       /*len*/
        CAN_data[2] = 0x05;       /*type*/
        CAN_data[3] = 0x03;
        CAN_data[4] = 0x55;
        CAN_data[5] = 0x05;
        CAN_data[6] = 0x0;
        CAN_data[7] = 0x0;
        CAN_data[8] = 0x5D;

        can_datagram_send(CAN_data, tx_type, tx_va) ;
        ////printf("TLM_MCU1Whole send!\n");

        m = (INT8U *)OSQPend(CAN_Q_Rec,1000,&err);
        OSSchedLock();
        for (i=0;i<110;i++)
        {
                rx.d[i]=*m;
                m++;
        }
        OSSchedUnlock();

        ////printf("m copyed!\n");	
        rx.va=0x01;
        rx.id=*(m+3)/16;
        rx.len=*(m+3)%16;

        /*    //printf("rx.va=%x\n",rx.va);
        //printf("rx.id=%x\n",rx.id);
        //printf("rx.len=%x\n",rx.len);
        //printf("rx.d=\n");
        for (i=0;i<110;i++)
        {
        //printf(" %x",rx.d[i]);
        }  
        //printf("\n");
        */
        tlmMCU1Vref       = ((rx.d[ 4]&0x3F)<<8) + rx.d[5];
        //    tlmMCU1MM1x     = ((rx.d[ 6]&0x3F)<<8) + rx.d[7];
        //    tlmMCU1MM1y     = ((rx.d[ 8]&0x3F)<<8) + rx.d[15];
        tlmMCU1MM1y     = ((rx.d[ 6]&0x3F)<<8) + rx.d[7];
        tlmMCU1MM1x     = ((rx.d[ 8]&0x3F)<<8) + rx.d[15];
        tlmMCU1MM1z     = ((rx.d[ 16]&0x3F)<<8) + rx.d[17];
        tlmMCU1MMS      = ((rx.d[18]&0x3F)<<8) + rx.d[19];
        tlmMCU1ASSPx    = ((rx.d[26]&0x3F)<<8) + rx.d[27];
        tlmMCU1ASSNx    = ((rx.d[28]&0x3F)<<8) + rx.d[29];
        tlmMCU1ASSPy    = ((rx.d[30]&0x3F)<<8) + rx.d[37];
        tlmMCU1ASSNy    = ((rx.d[38]&0x3F)<<8) + rx.d[39];
        tlmMCU1MT1IMon  = ((rx.d[40]&0x3F)<<8) + rx.d[41];
        tlmMCU1MT2IMon  = ((rx.d[48]&0x3F)<<8) + rx.d[49];
        tlmMCU1MT3IMon  = ((rx.d[50]&0x3F)<<8) + rx.d[51];
        tlmMCU1MTVref   = ((rx.d[52]&0x3F)<<8) + rx.d[59];
        tlmMCU1MMSTs    = ((rx.d[60]&0x3F)<<8) + rx.d[61];//无实际意义
        tlmMCU1MM1Ts    = ((rx.d[62]&0x3F)<<8) + rx.d[63];
        tlmMCU1ASSTs    = ((rx.d[70]&0x3F)<<8) + rx.d[71];//无实际意义
        tlmMCU1P5VMon   = (rx.d[72]<<8) + (rx.d[73]&0x3);
        /*
           tlmMCU1P12VMon  = (rx.d[74]<<8) + (rx.d[81]&0x3);
           tlmMCU1N12VMon  = (rx.d[82]<<8) + (rx.d[83]&0x3);
           tlmMCU1P5VIMon  = (rx.d[84]<<8) + (rx.d[85]&0x3);
           tlmMCU1P12VIMon = (rx.d[92]<<8) + (rx.d[93]&0x3);
           tlmMCU1N12VIMon = (rx.d[94]<<8) + (rx.d[95]&0x3);
           */
        tlmMCU1P5VIMon  = (rx.d[74]<<8) + (rx.d[81]&0x3);
        tlmMCU1P12VMon  = (rx.d[82]<<8) + (rx.d[83]&0x3);
        tlmMCU1P12VIMon  = (rx.d[84]<<8) + (rx.d[85]&0x3);
        tlmMCU1N12VMon = (rx.d[92]<<8) + (rx.d[93]&0x3);
        tlmMCU1N12VIMon = (rx.d[94]<<8) + (rx.d[95]&0x3);
        tlmMCU1GYRO     = (rx.d[96]<<8) + (rx.d[103]&0x3);//无实际意义
        tlmMCU1TsVref   = (rx.d[104]<<8) + (rx.d[105]&0x3);

        //////////if the output of MM is negative, do this!!/////////////
        if(tlmMCU1MM1x>8192)
        {
                tlmMCU1MM1x=tlmMCU1MM1x-16389;
        }

        if(tlmMCU1MM1y>8192)
        {
                tlmMCU1MM1y=tlmMCU1MM1y-16389;
        }
        tlmMCU1MM1y     =-tlmMCU1MM1y ;
        if(tlmMCU1MM1z>8192)
        {
                tlmMCU1MM1z=tlmMCU1MM1z-16389;
        }
        /////////////////////////////////////////////////////////////////////
        /*    //printf("tlmMCU1MM1x=%d\n",tlmMCU1MM1x);
        //printf("tlmMCU1MM1y=%d\n",tlmMCU1MM1y);
        //printf("tlmMCU1MM1z=%d\n",tlmMCU1MM1z);
        //printf("TLM_MCU1Whole Completed!\n");
        */
        return( 0 );
}

// functions TLM to MCU2
//MCU2全部遥测数据，按照ADCS_CAN通讯协议
int TLM_MCU2Whole( void )
{
        INT8U  i, CAN_data[20];
        INT8U  err;
        INT8U  *m;
        INT32U tx_va;
        INT32U tx_type;
        ADCSrxflow rx;
        tx_va   = 0x1;
        tx_type = 0x1;          /*即mode,即va*/
        //printf("ADCS CAN MCU2Whole start:\n");
        CAN_data[0] = 0x02;       /*id*/
        CAN_data[1] = 0x01;       /*len*/
        CAN_data[2] = 0x05;       /*type*/
        CAN_data[3] = 0x04;
        CAN_data[4] = 0x55;
        CAN_data[5] = 0x05;
        CAN_data[6] = 0x0;
        CAN_data[7] = 0x0;
        CAN_data[8] = 0x5E;
        can_datagram_send(CAN_data, tx_type, tx_va);
        //printf("can_send test return %x.\n",can_datagram_send(CAN_data, tx_type, tx_va) );
        //printf("MCU2Whole send!\n");

        m = (INT8U *)OSQPend(CAN_Q_Rec,0,&err);
        //printf("m copyed!\n");

        rx.va=0x01;
        rx.id=*(m+3)/16;
        rx.len=*(m+3)%16;
        ////memcpy(&(rx.d[110]),m,0x6E);
        //memcpy(rx.d,m,110);

        for (i=0;i<110;i++)
        {
                rx.d[i]=*m;
                //printf(" %x",rx.d[i]);
                m++;
        }
        //printf("\n");
        tlmMCU2Vref     = ((rx.d[4]&0x3F)<<8) + rx.d[5];
        tlmMCU2MMS      = ((rx.d[18]&0x3F)<<8) + rx.d[19];
        tlmMCU2ASSPx    = ((rx.d[26]&0x3F)<<8) + rx.d[27];
        tlmMCU2ASSNx    = ((rx.d[28]&0x3F)<<8) + rx.d[29];
        tlmMCU2ASSPy    = ((rx.d[30]&0x3F)<<8) + rx.d[37];
        tlmMCU2ASSNy    = ((rx.d[38]&0x3F)<<8) + rx.d[39];

        tlmMCU2MT1IMon    = ((rx.d[40]&0x3F)<<8) + rx.d[41];
        tlmMCU2MT2IMon    = ((rx.d[48]&0x3F)<<8) + rx.d[49];
        tlmMCU2MT3IMon  = ((rx.d[50]&0x3F)<<8) + rx.d[51];

        tlmMCU2MTVref  = ((rx.d[52]&0x3F)<<8) + rx.d[59];

        tlmMCU2ASSTs     = ((rx.d[70]&0x3F)<<8) + rx.d[71];

        tlmMCU2P5VMon   = (rx.d[72]<<8) + (rx.d[73]&0x3);
        tlmMCU2P5VIMon  = (rx.d[74]<<8) + (rx.d[81]&0x3);
        tlmMCU2P12VMon  = (rx.d[82]<<8) + (rx.d[83]&0x3);
        tlmMCU2P12VIMon = (rx.d[84]<<8) + (rx.d[85]&0x3);
        tlmMCU2N12VMon  = (rx.d[92]<<8) + (rx.d[93]&0x3);
        tlmMCU2N12VIMon = (rx.d[94]<<8) + (rx.d[95]&0x3);

        tlmMCU2TsVref   = (rx.d[104]<<8) + (rx.d[105]&0x3);



        /*
           tlmMCU2MMSTs    = ((rx.d[26]&0x3F)<<8) + rx.d[27];
           tlmMCU2ASSPx    = ((rx.d[28]&0x3F)<<8) + rx.d[29];
           tlmMCU2ASSNx    = ((rx.d[30]&0x3F)<<8) + rx.d[37];
           tlmMCU2ASSPy    = ((rx.d[38]&0x3F)<<8) + rx.d[39];
           tlmMCU2ASSNy    = ((rx.d[40]&0x3F)<<8) + rx.d[41];
           tlmMCU2ASSTs    = ((rx.d[48]&0x3F)<<8) + rx.d[49];
           tlmMCU2MT1IMon  = ((rx.d[50]&0x3F)<<8) + rx.d[51];
           tlmMCU2MT2IMon  = ((rx.d[52]&0x3F)<<8) + rx.d[59];
           tlmMCU2MT3IMon  = ((rx.d[60]&0x3F)<<8) + rx.d[61];
           tlmMCU2MTVref   = ((rx.d[62]&0x3F)<<8) + rx.d[63];
           tlmMCU2Vref     = ((rx.d[70]&0x3F)<<8) + rx.d[71];

           tlmMCU2P5VMon   = (rx.d[72]<<8) + (rx.d[73]&0x3);
           tlmMCU2P5VIMon  = (rx.d[74]<<8) + (rx.d[81]&0x3);
           tlmMCU2P12VMon  = (rx.d[82]<<8) + (rx.d[83]&0x3);
           tlmMCU2P12VIMon = (rx.d[84]<<8) + (rx.d[85]&0x3);
           tlmMCU2N12VMon  = (rx.d[92]<<8) + (rx.d[93]&0x3);
           tlmMCU2N12VIMon = (rx.d[94]<<8) + (rx.d[95]&0x3);
           tlmMCU2GYRO     = (rx.d[96]<<8) + (rx.d[103]&0x3);
           tlmMCU2TsVref   = (rx.d[104]<<8) + (rx.d[105]&0x3);
        //printf("MCU2Whole completed!");
        */
        return( 0 );
}


