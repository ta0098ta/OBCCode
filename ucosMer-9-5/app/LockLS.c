/*******************************************************

  wod.c

  WOD task, which run at every fixed interval of time, is difined here.

  WOD receives semaohore from the time isr, 
  and then communicates with TTC and restore the datas in fixed local memory.

  coded by Wang Xiaochu

  Altered by Hou Jiaming

 *********************************************************/


#include "includes.h"
#include "Includes.h"

extern INT16U gps_week;
extern INT32U gps_second;
extern INT16U eis_transf_flag;
extern INT8U LockFirst; 
extern INT32U wod_cycle;   /*wod存储周期*/ 
extern INT8U GpsReady;
extern OS_EVENT *CAN_Q_Rec;				//姿控数据报接收标志
extern OS_EVENT *CAN_Q_GPS;				//GPS GET帧接收标志
extern OS_EVENT *CAN_Q_GPSBAD;			//GPS的故障诊断帧接收标志
extern OS_EVENT *CAN_Q_REC_GPS;			//GPS数据报接收标志
extern OS_EVENT *CAN_Q_REC_EIS;			//EIS数据报接收标志
extern OS_EVENT *CAN_Q_REC_LOCK;			//TTC数据报接收标志
extern OS_EVENT *RS422_Q_Rec;				//422数据报接收标志
extern OS_EVENT *TTC_Flag;				//WOD包任务标志
extern OS_EVENT *Hdlc_Flag;
extern OS_EVENT *SHARE_CAN;  //CAN
extern OS_EVENT *Lock_Q_TTC;
extern OS_EVENT *TaskManageQueue;

extern INT8U LockTTCFlag;
extern INT8U WODFlag;

#ifndef __utype_defined
#define __utype_defined
typedef INT8U	u_char;
typedef INT16U  ushort;
typedef INT32U  uint;
typedef INT32U  ulong;
#endif
extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;
extern INT8U can_use_by_wod;
extern INT8U can_use_by_mimu;
extern INT8U can_use_by_dll;


/***************************************************************

  Lock task

 ****************************************************************/
void Lock(void *jdata)  
{
        INT16U k,temp;
        INT8U *can_p, err, i;
        INT32U id,len,test_second;
        INT32U temp_addr,acs_addr;
        INT16U timedata,temp_week,test_week;
        INT8U scc2buf[60], Buf,count; 
        INT8U LockBuf[120]; 
        void *msg;
        can_frame data_can;


        i = 0;
        count = 0;
        LockBuf[0] = 0x55;
        LockBuf[1] = 0x55;
        OSTimeDly(100);
        //printf("Lock Start!\n");
        for(;;)
        {    

                //		msg = OSMboxAccept(Lock_Q_TTC);
                //		if(msg != (void *)0)
                if(LockFirst != 3)
                {

                        id=0x01;
                        len=104;
                        data_can.t=ID_TTC2;
                        data_can.sdlc=8;
                        data_can.f=TTC_TLM_DATASTREAM;
                        data_can.d0=0;
                        data_can.d1=id>>16;
                        data_can.d2=id>>8;
                        data_can.d3=id&0xff;
                        data_can.d4=len>>16;
                        data_can.d5=len>>8;
                        data_can.d6=len&0xff;

                        if(can_datagram_send((INT8U *)&data_can,2,CANMSG)==OK)   /*请求TTC遥测数据包*/
                        {

                                //OSTimeDly(1);
                                can_p=(INT8U *)OSQPend(CAN_Q_REC_LOCK,100,&err); 

                                //OSSemPost(SHARE_CAN);	//2011_10_13
                                //if(err==OS_TIMEOUT)
                                //	{
                                //		printf("Lock TTC TIMEOUT1\n");
                                //	}
                                if(err==OS_NO_ERR)
                                {	
                                        OSSchedLock();
                                        for(i = 2; i < 106; i++)
                                        {
                                                LockBuf[i] = *(volatile INT8U*)(can_p+i+4);
                                        }

                                        if (get_hdlc_channel()==1)
                                        {
                                                fcc1_hdlc_send_data(LockBuf, 120);
                                        }
                                        else if (get_hdlc_channel()==3)
                                        {
                                                fcc3_hdlc_send_data(LockBuf, 120);
                                        }
                                        OSSchedUnlock();
                                        ////printf("Lock data is as below %x  %x.\n",LockBuf[2],LockBuf[3]);
                                }
                        }
                        OSTimeDly(800); 
                }
                else
                {
                        OSTimeDly(800); 
                }
        }
}

