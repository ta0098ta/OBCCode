/*
 * file: main.c
 *
 * PowerPC Main application source file
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */  
#include "includes.h"
#include "Includes.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
//#include <malloc.h>

//#define SEND_LEN 175


typedef struct interrupt_controller {
        INT16U  ic_sicr;
        INT8S   res1[2];
        INT32U  ic_sivec;
        INT32U  ic_sipnrh;
        INT32U  ic_sipnrl;
        INT32U  ic_siprr;
        INT32U  ic_scprrh;
        INT32U  ic_scprrl;
        INT32U  ic_simrh;
        INT32U  ic_simrl;
        INT32U  ic_siexr;
        INT8S   res2[88];
} intctl8260_t;

intctl8260_t * ipic;
// function prototypes
void   main      (void);

static void   Task_One  (void *data);
#if 1
static void   Task_Two  (void *data);
static void   Task_Three  (void *data);
static void   Task_Four  (void *data);
static void   Task_Five  (void *data);
static void   Task_Six  (void *data);
static void   Task_Seven  (void *data);
static void   Task_Eight  (void *data);
static void   Task_Nine  (void *data);
static void   Task_Ten  (void *data);
static void   Task_Eleven  (void *data);
static void   Task_Twelve  (void *data);
static void   Task_Thirteen  (void *data);
static void   Task_Fourteen  (void *data);
static void   Task_Fifteen  (void *data);
static void   Task_Sixteen  (void *data);
#endif
/////////////////////////////////////
//extern float q0,q1,q2,q3;
INT8U startrack[13]={0x05,0x01,0x00,0x01,0x00,0x01,0x00,0x04,0x00,0x74,0xA1,0xFF,0x00};






/////////////////////////////////////
#define TASK_STK_SIZE     1024u *8

#define TASK_ONE_DLY    30u              // in ticks
#if 1
#define TASK_TWO_DLY    40u 
#define TASK_THREE_DLY  50u 
#define TASK_FOUR_DLY   60u 
#define TASK_FINE_DLY    40u 
#define TASK_SIX_DLY     50u 
#define TASK_SEVEN_DLY   60u 
#define TASK_EIGHT_DLY    40u 
#define TASK_NINE_DLY  50u 
#define TASK_TEN_DLY   60u 
#define TASK_ELEVEN_DLY    40u 
#define TASK_TWELVE_DLY  50u 
#define TASK_THIRTEEN_DLY   60u 
#define TASK_FOURTEEN_DLY    40u 
#define TASK_FIFTEEN_DLY  50u 
#define TASK_SIXTEEN_DLY   60u



#define TASK_TWO_LED    2u 
#define TASK_THREE_LED  4u 
#define TASK_FOUR_LED   8u 
#endif
// priorities
enum t_prio { TASK_ONE_PRIO = 2, TASK_TWO_PRIO, TASK_THREE_PRIO, TASK_FOUR_PRIO };

INT8U OSTaskStarting;
// module scope variables
static INT32S  ctrTask_One, ctrTask_Two, ctrTask_Three;

static OS_STK  Task_OneStk[TASK_STK_SIZE];

OS_STK Task1[TASK_STK_SIZE];
OS_STK Task14[TASK_STK_SIZE];
#if 1
OS_STK Task2[TASK_STK_SIZE];
OS_STK Task3[TASK_STK_SIZE];
OS_STK Task4[TASK_STK_SIZE*4];
OS_STK Task5[TASK_STK_SIZE*4];
OS_STK Task6[TASK_STK_SIZE*4];
OS_STK Task7[TASK_STK_SIZE];
OS_STK Task8[TASK_STK_SIZE];
OS_STK Task9[TASK_STK_SIZE];
OS_STK Task10[TASK_STK_SIZE];
OS_STK Task11[TASK_STK_SIZE];
OS_STK Task12[TASK_STK_SIZE];
OS_STK Task13[TASK_STK_SIZE];
OS_STK Task15[TASK_STK_SIZE];
OS_STK Task16[TASK_STK_SIZE];
OS_STK Task17[TASK_STK_SIZE];
OS_STK Task18[TASK_STK_SIZE];
OS_STK Task19[TASK_STK_SIZE];
OS_STK Task20[TASK_STK_SIZE];
OS_STK Task21[TASK_STK_SIZE];
#endif
static INT8U   Task_OneData;


#if 1
INT8U	HDLC_channel_select;
INT16U gps_week;
INT32U gps_second;
INT16U eis_transf_flag;
INT8U SEND_TIMEOUT;
INT8U SEND_STOP;
INT32U wod_cycle;   /*wod存储周期*/
INT32U PC_Cycle;
INT8U ERR_SHARE_PRINTF;
INT8U ERR_SHARE_SENDDOWN;
INT8U ERR_SHARE_CAN;
INT32U PCSsecond,PCFsecond;
INT16U PCSweek,PCFweek;

INT8U can_use_by_wod;
INT8U can_use_by_mimu;
INT8U can_use_by_dll;
INT8U GpsReady, LockFirst, LockTime;
INT32U NoLockTime;

INT8U LockTTCFlag = 0;
INT8U WODFlag = 0;
INT8U StarAcceptCount,PCFlag[8],PCCount;
int GpsOnOff;
INT8U StarFlag;
#endif






/* zwj  2017*/


OS_EVENT * GPSQ;
OS_EVENT * StarSQ;
OS_EVENT * SpiQ;
OS_EVENT * SendQ;
OS_EVENT * Send_rfQ;
OS_EVENT * Dsp_rfQ;

INT16U gpsbuf[50], starsbuf[50], spibuf[50], sdbuf[50], send_rf[50], dsp_rf[50];

INT8U rf_flag = 0;








#if 1
extern INT8U PCNodePointer;
extern INT8U PCNodeEnd;
OS_EVENT *CAN_Q_Rec;					//姿控数据报接收标志
OS_EVENT *CAN_Q_Rec2;					//姿控数据报接收标志
OS_EVENT *CAN_Q_GPS;					//GPS GET帧接收标志
OS_EVENT *CAN_Q_GPSBAD;			//GPS的故障诊断帧接收标志
OS_EVENT *CAN_Q_REC_GPS;			//GPS数据报接收标志
OS_EVENT *CAN_Q_REC_EIS;			//EIS数据报接收标志
OS_EVENT *CAN_Q_REC_TTC;			//TTC数据报接收标志
OS_EVENT *CAN_Q_REC_LOCK;	
OS_EVENT *RS422_Q_Rec;
OS_EVENT *ADCS_FLAG;				//422数据报接收标志
OS_EVENT *ACSQCOM;
OS_EVENT *ZPF_FLAG;
OS_EVENT *ZPF_OSQ;


OS_EVENT *ctSem;      /*时钟中断标志*/
OS_EVENT *TTC_Flag;		//WOD包任务标志
OS_EVENT *Lock_Q_TTC;	
extern OS_EVENT *Hdlc_Falg;     /*dispatch任务标志*/
OS_EVENT *HdlcQEvent;
OS_EVENT *sendQEvent;
OS_EVENT *RecMsg;
OS_EVENT *sendRecMsg;
OS_EVENT *timeoutQEvent;
OS_EVENT *EIS0_photographQ;
OS_EVENT *EIS1_photographQ;
OS_EVENT *MEMSGyroQ;
OS_EVENT *BDGPSQ;
OS_EVENT *PC_Q;
OS_EVENT *PC_UQ;
OS_EVENT *PCQFlag;
OS_EVENT *transeiver_receiveQ;
OS_EVENT *ADCS_HDLC_REC;
OS_EVENT *ACSQCOM;
OS_EVENT *ADCS_FLAG;	
OS_EVENT *DOWN_REMOTE_DATA;
OS_EVENT *gps_getQ;
OS_EVENT *LaserEventQ;
OS_EVENT *CAN_Q_Rec_MIMU;
OS_EVENT *CAN_Q_Rec_DLL;
OS_EVENT *DLL_HDLC_REC;
OS_EVENT *MIMU_HDLC_REC;
OS_EVENT *RFS_HDLC_REC;
OS_EVENT *CAN_Q_Rec_RF1;
OS_EVENT *SHARE_PRINTF;
OS_EVENT *SHARE_SENDDOWN;
OS_EVENT *SHARE_CAN;
OS_EVENT *sendMsg;
OS_EVENT *SS_OSQ;
OS_EVENT *TaskManageQueue;
  
#endif

#if 0
INT16U *mp[50],*me[100],*can_buf[20],*can_buf2[20],*ms[20],*mse[20],*msq[20],*acsq[20],*cang[20],*to[20],*gb[1];
INT16U *lo[10],*cg[10],*ce[10],*ct[10],*ac[10],*tr[100],*ad[50],*don[50], *uart_down[50],*rs422[50],*eis0[100],*eis1[100],*gps[20],*ProConFlag[20],*ProCon[20],*ProConUart[20],*laserM[20],*dll_buf[20],*mimu_buf[20],*dll_hdlc[20],*mimu_hdlc[20],*rfs[20],*rf_can[20],*stars[20], *task_queue[20], *BDGPSBuf[20], *MEMSGyroBuf[20];
#else 
INT16U mp[50],me[100],can_buf[20],can_buf2[20],ms[20],mse[20],msq[20],acsq[20],cang[20],to[20],gb[1];
INT16U lo[10],cg[10],ce[10],ct[10],ac[10],tr[100],ad[50],don[50], uart_down[50],rs422[50],eis0[100],eis1[100],gps[20],ProConFlag[20],ProCon[20],ProConUart[20],laserM[20],dll_buf[20],mimu_buf[20],dll_hdlc[20],mimu_hdlc[20],rfs[20],rf_can[20],stars[20], task_queue[20],BDGPSBuf[20], MEMSGyroBuf[20], zpf[20];
#endif

FRAME *sbuf[MAX_SEQ];
INT32U irq6_counter, timerirq_counter, timer_counter;

extern void Spi_Main(void *jdata);
extern void StarSensorTask(void *jdata);
extern void gps_serial_smc2(void);
extern void dispatch(void *jdata);
extern void dispatch_rf(void *jdata);



extern void DPRAM_IRQ(void);
#if 1
//extern void ADCS_Main(void *jdata);
//extern void wod(void *jdata);
//extern void get_HDLC_rec_channel(void *jdata);
//extern void HDLC_receive(void *jdata);
//extern void hdlc_send(void *jdata);
//extern void send_down_frame(INT8U id, INT8U recnum, INT8U sendnum, FRAME *data);
//extern void timer_isr(void *jdata);
//extern void Transceiver (void *jdata);
//extern void BDGPS_Main(void *jdata);
//extern void LaserMeasure(void *jdata);
//extern void DLL_Main(void * data);
//extern void RF_Main(void * data);
//extern void Lock(void *jdata);
//extern void ProgramControl(void *jdata);
//extern void ProgramControlUart(void *jdata);
//extern CPU_INT32U fcc1_tx_packets, fcc3_tx_packets;
//extern CPU_INT32U fcc1_rx_packets, fcc3_rx_packets;
//void ADCS_Main2 (void * data);
//void DeleteTest(void *jdata) ;
typedef struct TIMEOUT_struct //发送超时判断
{
        INT8U TIMEOUT_CHECK;//检查标志
        INT8U TIMEOUT_RETRY;//重发次数
        INT8U recnum;   //  接收序号
        INT8U sendnum;  // 发送序号
        INT8U second;

} TIMEOUT_send;


#endif
//extern TIMEOUT_send RR_TIMEOUT;


#if  0
void ADCS_Serial_Select(INT8U t)
{
        if (t==1)
                *(volatile INT32U *)(0xF0010D10) |= 0x00001000;
        else
                *(volatile INT32U *)(0xF0010D10) &= 0xffffefff;
}

#endif

void reset_8260_watchdog()
{
        *(volatile INT16U *)0xf001000E = 0x556c;
        *(volatile INT16U *)0xf001000E = 0xaa39;
}
/*
void startracker(void)
{

        //OSTimeDly(1000);
        OSQPost(transeiver_receiveQ,startrack);
}
*/

void PIT_TMCNT_init_func()
{

        //PITC
        //*(volatile unsigned short *)(0xF0010244) = 0x1fff;
        *(volatile unsigned short *)(0xF0010244) = 0x8;
        //PISCR
        *(volatile unsigned short *)(0xF0010240) = 0x7;

        //TMCNT
        *(volatile unsigned int *)(0xF0010224) = 0x1fff;
        //TMCNTSC
        *(volatile unsigned short *)(0xF0010220) = 0x008B;


}	


#if 1
void timerirq_func()	//1 second interrupt
{

        INT8U temp[50];
        INT8U temp_i;
        INT8U gpsFlag[1];
		
        INT8U LockFlag = 1;
        static INT8U PCFlag[8],PCCount = 0;
        //temp[0]=0x00;
        //temp[1]=0x03;
        gpsFlag[0] = 0x01;
        reset_8260_watchdog();
        //printf ("11111111119999999999999999\n");
        
        for( temp_i =0;temp_i<28;temp_i++)
                temp[8+temp_i] = temp_i;

//        timer_counter++;
        *(volatile INT16U *)(0xF0010220) |= 0x0080;
        /*维护GPS时间*/
#if 0
        gps_second++;
        if(gps_second>=604800)
        { 
                gps_week++;
                gps_second=0;
        }
        ///OSSemPost_i(ctSem);

/*11111111111111111111111111111111111111111111111111111111111111*/
        if((gps_second >= PCSsecond) && (gps_week >= PCSweek) && (gps_second <= PCFsecond) && (gps_week <= PCFweek))
        {
                //printf("PC Time!");
                if(gps_second == PCSsecond)//kai wai she
                {
                        PCFlag[0] = 1;
                        OSQPost(PCQFlag,(void *)&PCFlag[0]);
                }
        }


/*11111111111111111111111111111111111111111111111111111111111111*/

        if(gps_second % wod_cycle ==0)
        {		
                OSMboxPost(TTC_Flag, (void *)1);
        }
        else if(LockFirst == 1)
        {
                OSMboxPost(Lock_Q_TTC, (void *)1);
        }
        else if(LockFirst == 2)
        {
                if(LockTime < 30*60)
                {
                        OSMboxPost(Lock_Q_TTC, (void *)1);
                        LockTime++;
                }
                else
                {
                        LockTime = 0;
                        LockFirst = 0;
                        NoLockTime = 0;
                }
        }        
        else if(NoLockTime >= 24*60*60)
        {
                OSMboxPost(Lock_Q_TTC, (void *)1);
        }

        else if(LockFirst == 3)
        {
        	//printf("LockTest!\n");
        }
#endif

        NoLockTime++;


#if 0
        OSMboxPost(ADCS_FLAG, (void *)1);
        //	OSQPost(gps_getQ,(void *)&gpsFlag[0]);
        //if(gps_second  %10==0)
        //{//printf("send message to RF task\n");
        //OSQPost(RFS_HDLC_REC,&temp[8]);}
        ////printf("---------------------timerirq_func %d\n",timer_count);
        //SIPNR
        //*(volatile INT16U *)(0xF0010C10) |= 0x2;
        if(RR_TIMEOUT.TIMEOUT_CHECK==1) //判断发送超时否
        {
                RR_TIMEOUT.second++; //超时秒加1
                if ( RR_TIMEOUT.second == 3 )  //超时3秒,重发
                {
                        if(RR_TIMEOUT.TIMEOUT_RETRY==0) //重发5次仍然超时,则发送停止
                                OSQPost(sendRecMsg,&SEND_STOP); 
                        else
                        {
                                RR_TIMEOUT.TIMEOUT_RETRY--;
                                OSQPost(sendRecMsg,&SEND_TIMEOUT); 
                        }

                }
        }
#endif

}


#endif





void pitirq_func()
{
        timerirq_counter++;
        OSTimeTick();
        *(volatile INT16U *)(0xF0010240) |= 0x0080;
        //SIPNR
        //	*(volatile INT16U *)(0xF0010C10) |= 0x2;
}

#if 0
INT8U get_hdlc_channel(void)
{
        if ((*(volatile INT32U *)(0xF0010D10) & (0x00000200)) == 0)
                return 1;
        else 
                return 3;

}


#endif

/*
   main () 
   */ 
//extern INT32U serial_init (void);
void main (void)
{
        INT32U  idx;
        INT8U cc;
        INT32U i;
        INT8U temp[50];

        *(volatile INT32U *)(0xf0010000) = 0x0e200000;		//config SIUMCR to access IRQ6

#if 0
        LockFirst = 1;
        LockTime = 0;
        PCSweek = 0;
        PCSsecond = 0;
        PCFweek = 0;
        PCFsecond = 0;
        PC_Cycle = 3;
        PCCount = 0;
        gps_week = 100;
        gps_second = 50;
        GpsOnOff = 1;
        eis_transf_flag = 0;
        wod_cycle = 20;
        irq6_counter = 0;
        timerirq_counter = 0;
        timer_counter = 0;
        GpsReady = 0;	//WOD是否可获取GPS任务中数据
        NoLockTime = 0;
        StarFlag = 0;
        StarAcceptCount = 1;
        SEND_TIMEOUT=0xDD;
        SEND_STOP=0xEE;
#endif

        m8260_cpm_reset();
        get_clocks();

        epic_InitEPIC();                        // intialize the interrupt controller

        serial_init();
        m8260_cpm_setbrg(0,32768/16);	//set brgc0 to 32KHz
        smc1_serial_init();
        smc2_serial_init();
        scc2_serial_init();
        scc3_serial_init();
        scc4_serial_init();


        spi_init_f();
        spi_init_r();







        *(volatile INT32U *)(0xF0010D00) |= 0x00001000;	//PDIR_PA19
      //  ADCS_Serial_Select(1);    

       // OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
        //printf("OSTaskStarting=%d,%d\n",OSTaskStarting,Task_OneData);
       // OSSemPost(SHARE_PRINTF);
        //memset(&RR_TIMEOUT,0,5);//超时计时器清零

        //for (idx = 0u; idx < TASK_STK_SIZE; idx++)   // fill stacks with known pattern
       // {

 //               Task_OneStk  [idx] = 0xE0E1E2E3;

               // Task9 [idx] = 0xE0E1E2E3;
                //Task10 [idx] = 0xE0E1E2E3;
  //      }


        OSInit ();                              // intialize the OS

        /*****************************************************************

          OS  Event  Initialization Start


         ******************************************************************/

        

        GPSQ = OSQCreate((void *)gpsbuf,50);
        StarSQ = OSQCreate((void *)starsbuf,50);
        SpiQ = OSQCreate((void *)spibuf,50);
        SendQ = OSQCreate((void *)sdbuf, 50);
        Send_rfQ = OSQCreate ((void *)send_rf, 50);
        Dsp_rfQ = OSQCreate ((void *)dsp_rf, 50);



/*
        ACSQCOM=OSQCreate((void *)ac,20);
        RS422_Q_Rec=OSQCreate((void *)rs422,50);
        HdlcQEvent=OSQCreate((void *)me,100);
        sendQEvent=OSQCreate((void *)msq,20);
        RecMsg=OSQCreate((void *)mp,50);
        sendMsg=OSQCreate((void *)ms,20);
        SS_OSQ=OSQCreate((void *)stars,20);
        sendRecMsg=OSQCreate((void *)mse,20);
        CAN_Q_Rec=OSQCreate((void *)can_buf,20);
        CAN_Q_Rec2=OSQCreate((void *)can_buf2,20);
        // ADCS_Q_Event=OSQCreate((void *)acsq,20);
        CAN_Q_GPS=OSQCreate((void *)cang,20); 
        timeoutQEvent=OSQCreate((void *)to,20);
        // sendMessageQ=OSQCreate((void *)sbuf,MAX_SEQ); 
        CAN_Q_GPSBAD=OSQCreate((void *)gb,1);
        CAN_Q_REC_GPS=OSQCreate((void *)cg,10);
        CAN_Q_REC_EIS=OSQCreate((void *)ce,10);
        CAN_Q_Rec_DLL=OSQCreate((void *)dll_buf,20);
        CAN_Q_Rec_MIMU=OSQCreate((void *)mimu_buf,20);
        CAN_Q_REC_TTC=OSQCreate((void *)ct,10);
        CAN_Q_REC_LOCK=OSQCreate((void *)lo,10);
        // ACS_Q_COM=OSQCreate((void *)ac,10);
        LaserEventQ=OSQCreate((void *)laserM,20);
       transeiver_receiveQ=OSQCreate((void *)tr,100);
        EIS0_photographQ=OSQCreate((void *)eis0,100);
        EIS1_photographQ=OSQCreate((void *)eis1,100);
        ADCS_HDLC_REC=OSQCreate((void *)ad,50);
        DOWN_REMOTE_DATA=OSQCreate((void *)don,50);
        gps_getQ = OSQCreate((void *)gps,20);
        PCQFlag = OSQCreate((void *)ProConFlag,20);
        PC_Q = OSQCreate((void *)ProCon,20);
        PC_UQ = OSQCreate((void *)ProConUart,20);
        DLL_HDLC_REC=OSQCreate( (void *)dll_hdlc,20 );
        MIMU_HDLC_REC=OSQCreate( (void *)mimu_hdlc,20 );
        RFS_HDLC_REC=OSQCreate( (void *)rfs,20 );

        BDGPSQ = OSQCreate((void *)BDGPSBuf,20);
        MEMSGyroQ = OSQCreate((void *)MEMSGyroBuf,20);

        TaskManageQueue = OSQCreate( (void *)task_queue,20);
        ZPF_OSQ= OSQCreate((void *)zpf,20);
        // EIS_Flag=OSSemCreate(0);
        // send_flag=OSSemCreate(0);
        // Hdlc_Flag=OSSemCreate(0);
        SHARE_PRINTF=OSSemCreate(1);
        SHARE_SENDDOWN=OSSemCreate(1);
        SHARE_CAN=OSSemCreate(1);
        ctSem=OSSemCreate(0);

        //data_ok_sem=OSSemCreate(0);
        //CAN_SEM_Message=OSSemCreate(0); 
        CAN_Q_Rec_RF1=OSQCreate( (void *)rf_can,20 );
        //ADCS_FLAG=OSMboxCreate((void *)0);
        TTC_Flag=OSMboxCreate((void *)0);
        Lock_Q_TTC = OSMboxCreate((void *)0);
        ADCS_FLAG=OSMboxCreate((void *)0);
        ZPF_FLAG= OSSemCreate(0);
*/
        /*****************************************************************

          OS  Event  Initialization End.

         ******************************************************************/

        /*****************************************************************

          OS  Task  Initialization Start

          priority			Task Name
         ****************************
         3			Task_One
         ****************************
         4			timer_isr
         ****************************
         5			dispatch
         ****************************
         6			HDLC_receive
         ****************************
         7			hdlc_send_2
         ****************************
         8			ADCS_Main
         ****************************
         9			Transceiver
         ****************************

         ****************************
         11			TASK_DELETE
         ****************************
         12			wod
         ****************************
         13			MIMU_Main(MEMS wu)
         ****************************
         18			DLL_Main
         ****************************
         20			RF_Main
         ****************************

         ****************************
         29			StarSensorTask
         ****************************
         30			MEMS_Gyro
         ****************************
         31			BD/GPS
         ****************************
         32			gps_serial_scc3
         ****************************
         33			LaserMeasure(MEMS wu)
         ****************************

         ****************************

         ******************************************************************/

        printf ("UCOS start \n");
        printf ("****************************************\n");
        OSTaskCreate (  Task_One,
                        //(void *)&Task_OneData,
                        (void *)0,
                        (void *)&Task_OneStk[TASK_STK_SIZE - 1],
                        3/*TASK_ONE_PRIO*/);

        OSTaskCreate(dispatch, (void *)0, &Task5[TASK_STK_SIZE - 1],4);
        OSTaskCreate(dispatch_rf, (void *)0, &Task6[TASK_STK_SIZE - 1],6);
        OSTaskCreate(Spi_Main,(void *)0, &Task14[TASK_STK_SIZE-1],8);
        OSTaskCreate(StarSensorTask,(void *)0, &Task11[TASK_STK_SIZE - 1],10);        
        OSTaskCreate(gps_serial_smc2,(void *)0, &Task16[TASK_STK_SIZE - 1],13);

        //OSTaskCreate(send_down, (void *)0, &Task7[TASK_STK_SIZE - 1],5);
        //OSTaskCreate(HDLC_receive, (void *)0, &Task6[TASK_STK_SIZE - 1],6);
        //OSTaskCreate(Transceiver, (void *)0, &Task8[TASK_STK_SIZE - 1],7);
        //OSTaskCreate(timer_isr, (void *)0, &Task2[TASK_STK_SIZE - 1],10);
        //OSTaskCreate(DLL_Main,(void *)0,&Task17[TASK_STK_SIZE - 1],18);
        //OSTaskCreate(RF_Main,(void *)0,&Task18[TASK_STK_SIZE - 1],20); 
        //OSTaskCreate(BDGPS_Main, (void *)0, &Task15[TASK_STK_SIZE-1],26); 
        //OSTaskCreate(ProgramControlUart,(void *)0,&Task3[TASK_STK_SIZE - 1],28);         
        //OSTaskCreate(wod,(void *)0,&Task10[TASK_STK_SIZE - 1],33);
        //OSTaskCreate(Lock,(void *)0,&Task21[TASK_STK_SIZE - 1],35); 
        //OSTaskCreate(ADCS_Main, (void *)0, &Task9[TASK_STK_SIZE - 1],36);
        //OSTaskCreate(ProgramControl,(void *)0,&Task13[TASK_STK_SIZE - 1],39);


        printf ("main end \n");

        //OSTimeDly(7000);

        OSStart();                              // never returns...
        while(1);
}

/*****************************************************************

  OS  Task  Initialization End.

 ******************************************************************/

static void Task_One (void * data)
{
        INT32U i=0,k=0;
        INT8U len;
        INT8U err;
        INT8U eistestdata1[50];
        INT16U byteCount,j;
        INT16U *aaa;
        INT8U *downmsg;
        INT8U gpsFlag;
        intctl8260_t * ipic;
        INT32U test_tx;
        INT32U current;
        INT8U scc3msg[12];
        INT32U y[10];
        INT8U msg[20];
        INT8U temp[50];
        INT8U temp_i;
        float aaaa=24.23;
        INT8U cmd_dkc;
        float test_f;
        INT32U test_i;
        INT8U a0,a1,a2,a3;

#if OS_CRITICAL_METHOD == 3                                /* Allocate storage for CPU status register */
        OS_CPU_SR  cpu_sr = 0;
#endif
        URP(data);
        //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
        //printf("read first msr=0x%x\n",readtime());
        //OSSemPost(SHARE_PRINTF);

        //hdlc_app_init();

        //IRQ initialization

        //register_irq(24, DPRAM_IRQ);

        register_irq(16, timerirq_func);
        register_irq(17, pitirq_func);
        ipic = (intctl8260_t *)0xf0010c00;
        ipic->ic_simrh = 0x00000206; 	//IRQ6 TMCNT PIT 
        ipic->ic_simrl = 0xA0000000;	//0xA0000000;	//FCC1	FCC3 

        PIT_TMCNT_init_func();		// start the OS ticker
        CPU_IntEn();
        //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
        //printf("read second msr=0x%x,%d\n",readtime(),OSTaskStarting);
        //OSSemPost(SHARE_PRINTF);

#if OS_TASK_STAT_EN > 1    
        OSStatInit ();
#endif
//        PCNodePointer=0;
//        PCNodeEnd=0;
        //OSTimeDly(1000);

        //get_HDLC_rec_channel((void *)0);

        //OSTaskDel(3);

#if 0
        for(;;)
        {
                OSTimeDly(30);
                for(k=0;k<7;k++)                   /*取出一桢*/
                {

                        //data[MAX_REC_LEN*j+k]=(unsigned short)*addr;
                        data[k]= serial_getc();
                }
                if (data[0] == 0xEB90){
                        for (len = 0; len < data[6]; len ++) {
                                data[len + 7]= serial_getc();
                        }

                        for (k =0; k <( len+7);k++ ) {
                                printf ("%d  /n", data[k]);
                        }
                }
                else
                        continue;
        }
#endif

}


