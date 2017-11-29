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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>

extern INT16U gps_week;
extern INT32U gps_second;
extern INT16U eis_transf_flag;
extern INT32U wod_cycle;   /*wod存储周期*/ 
extern INT8U GpsReady;
extern OS_EVENT *CAN_Q_Rec;				//姿控数据报接收标志
extern OS_EVENT *CAN_Q_GPS;				//GPS GET帧接收标志
extern OS_EVENT *CAN_Q_GPSBAD;			//GPS的故障诊断帧接收标志
extern OS_EVENT *CAN_Q_REC_GPS;			//GPS数据报接收标志
extern OS_EVENT *CAN_Q_REC_EIS;			//EIS数据报接收标志
extern OS_EVENT *CAN_Q_REC_TTC;			//TTC数据报接收标志
extern OS_EVENT *RS422_Q_Rec;				//422数据报接收标志
extern OS_EVENT *TTC_Flag;				//WOD包任务标志
extern OS_EVENT *Hdlc_Flag;
extern OS_EVENT *SHARE_CAN;  //CAN


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
extern INT8U CAN_busy();

/*00000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

extern  INT8U STATE;

/*00000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

/***************************************************************

  WOD task

 ****************************************************************/
void wod(void *jdata)   /*WOD存储任务*/
{
        INT16U j,k,temp, i;
        INT8U *can_p, err;
        INT32U id,len,test_second;
        INT32U addr,temp_addr,acs_addr;
        INT16U timedata,temp_week,test_week;
        INT8U scc2buf[60], Buf,count; 
        INT8U scc3buf[100],_isSuccess; 
        can_frame data_can;
        can_flow data_flow;
        addr=wod_begin_addr;

        Buf = 0;
        id=0x00;
        len=29;
        data_flow.id=ID_TTC1;
        data_flow.f=TTC_TC_DATASTREAM;
        data_flow.i=id>>8;
        data_flow.i2=id&0xff;
        data_flow.l=len>>8;
        data_flow.l2=len&0xff;

        data_flow.d[0]=0xEB;
        data_flow.d[1]=0x90;
        data_flow.d[2]=0xDC;
        data_flow.d[3]=0x32;
        data_flow.d[4]=0x0F;
        data_flow.d[5]=0x09;
        data_flow.d[6]=0x09;
        data_flow.d[7]=0x09;
        data_flow.d[8]=0x09;
        data_flow.d[9]=0x09;
        data_flow.d[10]=0x09;
        data_flow.d[11]=0x15;
        data_flow.d[12]=0x11;
        data_flow.d[13]=0x50;
        data_flow.d[14]=0x40;
        data_flow.d[15]=0xDD;
        data_flow.d[16]=0xDD;
        data_flow.d[17]=0xDD;
        data_flow.d[18]=0xDD;
        data_flow.d[19]=0xDD;
        data_flow.d[20]=0xDD;
        data_flow.d[21]=0xDD;
        data_flow.d[22]=0xDD;
        data_flow.d[23]=0xDD;
        data_flow.d[24]=0xDD;
        data_flow.d[25]=0xDD;
        data_flow.d[26]=0xDD;
        data_flow.d[27]=0x09;
        data_flow.d[28]=0xD7;//CMD4 遥测通道选择TLM1
        can_datagram_send((INT8U *)&data_flow,3,CANPUT);
        can_use_by_wod=0;
        i = 0;
        count = 0;


        //printf("WOD task start. \n");

        for(;;)
        {    

                OSTimeDly(1);
                OSMboxPend(TTC_Flag,0,&err);
                //OSQFlush(CAN_Q_REC_TTC);      //12.20
                temp_addr=gps_second;                       /*存储GPS周内时间秒*/
                temp_week=gps_week;
                //printf("WOD task at time week = %d, second = %x s.\n",temp_week, temp_addr);

                /*第一包数据
                 *加时间标签,12个字节
                 */
                *(volatile INT16U *)addr = temp_week;
                addr+=2;
                *(volatile INT8U *)addr = temp_addr>>24;
                addr+=1;
                *(volatile INT8U *)addr = temp_addr>>16;
                addr+=1;
                *(volatile INT8U *)addr = temp_addr>>8;
                addr+=1;
                *(volatile INT8U *)addr = temp_addr;
                addr+=1;

                test_second = *(INT32U *)(addr-4);
                test_week = *(INT16U *)(addr-6);

                /*if(addr==wod_end_addr)
                  addr=wod_begin_addr;*/

                /***********************存储TTC数据******************/
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

                //OSSemPend(SHARE_CAN,0,&ERR_SHARE_CAN);	//2011_10_13
                OSQFlush(CAN_Q_REC_TTC);      //2011_11_4
                WODFlag = 1;

                if(can_datagram_send((INT8U *)&data_can,2,CANMSG)==OK) /*请求TTC遥测数据包*/
                {

                        //OSTimeDly(1);
                        can_p=(INT8U *)OSQPend(CAN_Q_REC_TTC,500,&err); 

                        //OSSemPost(SHARE_CAN);	//2011_10_13
                        if(err==OS_TIMEOUT)
                        {
                                //printf("WOD TTC TIMEOUT1\n");
                                for(k=0;k<TTC_data_len/2;k++)             
                                {
                                        *(volatile INT16U *)(addr)=0;
                                        addr+=2;  
                                        if(addr==wod_end_addr)
                                                addr=wod_begin_addr;
                                }
                                // continue;
                        }
                        else if(err==OS_NO_ERR)
                        {
                                can_use_by_wod=0;
                                can_p+=6;
                                temp_addr=(INT32U)can_p;  
                                //			//printf("TTC data is as below.\n");
                                for(k = 0; k < TTC_data_len/2; k++)    /*存贮间接遥测数据*/
                                {   
                                        *(volatile INT16U *)(addr)=(*(volatile INT16U *)(can_p));
                                        ////printf("%x %x ",*(volatile INT8U *)(addr),*(volatile INT8U *)(addr+1));
                                        addr+=2;  
                                        can_p+=2;
                                        if(addr==wod_end_addr)
                                                addr=wod_begin_addr;
                                }
                                //	//printf("\n");
                        }
                }
                WODFlag = 0; 
                /***********************存储GPS数据***************/
                /*-----------------------------------*/

                memset(scc3buf, 0, 100);
                //printf("WOD GPS task working...\n");


                _isSuccess = 0;
                for(i = 0; i < 100; i++)
                {
                        changemux(2);
                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) 
                        {
                                continue;
                        }
                        scc3buf[0] = scc3_serial_getc();
                        if (scc3buf[0]!=0xEB) continue;	

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        scc3buf[1] = scc3_serial_getc();
                        if (scc3buf[1]!=0x90) continue;

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        scc3buf[2] = scc3_serial_getc();
                        if (scc3buf[2]!=0xF9) continue;

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        scc3buf[3] = scc3_serial_getc();
                        if (scc3buf[3] !=0xA8) continue;


                        for (j=0; j<60; j++)
                        {
                                timedata = 0;
                                while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                                if (timedata>=1000) continue;
                                while (!scc3_serial_tstc());
                                scc3buf[4+j] = scc3_serial_getc();

                        }

                        if (scc3buf[62]!=0xFD) continue;			

                        if (scc3buf[63]!=0xFD) continue;  

                        //处理获得的gps数据

                        //printf("gps data is\n"); 
                        for(j=0;j<64;j++)
                        {	
                                //	//printf("scc3buf[%d] = %x ",j,scc3buf[j]); //调试串口输出
                                *(volatile INT8U *)(addr)=scc3buf[j];//储存在内存
                                addr += 1;
                        }

                        _isSuccess=1;
                        break;    

                }

                if(_isSuccess==0)
                {
                        for(k = 0; k <64 ; k++)
                        {
                                *(volatile INT8U *)(addr) = 0x0A;
                                addr += 1;
                        }
                        //printf("GPS rand data is all zero!\n");
                }
                else
                {

                        //printf("GPS get data!\n");
                }

                /*
                 *ADCS
                 */
                /*第二包数据*/ 
                temp_addr=gps_second;                       /*存储GPS周内时间秒*/
                temp_week=gps_week;
                //    //printf("WOD task at time week = %d, second = %x s.\n",gps_week, gps_second);

                *(volatile INT16U *)addr = temp_week;
                addr+=2;
                *(volatile INT8U *)addr = temp_addr>>24;
                addr+=1;
                *(volatile INT8U *)addr = temp_addr>>16;
                addr+=1;
                *(volatile INT8U *)addr = temp_addr>>8;
                addr+=1;
                *(volatile INT8U *)addr = temp_addr;
                addr+=1;

                //	//printf("temp_week = %d temp_second = %x\n",*(volatile INT16U *)(addr-6),*(volatile INT32U *)(addr-4));

                acs_addr=ADCS_REALTIME_DATA;

                //	//printf("acs_addr = %x \n", *(volatile INT16U *)(acs_addr) );

                for(k=0;k<ADCS_LEN/2;k++)             /*存贮补充数据*/ 
                {               
                        *(volatile INT16U *)(addr)=*(volatile INT16U *)(acs_addr);
                        //(unsigned short)*(unsigned long *)(addr)=(unsigned short)*(unsigned long *)(acs_addr);
                        //	//printf("%x %x ",*(volatile INT8U  *)(addr),*(volatile INT8U  *)(addr+1));
                        addr+=2;
                        acs_addr+=2;
                }

                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

                *(volatile INT8U *)addr = STATE;
                addr += 1;
                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

                if(addr>=wod_end_addr)
                        addr=wod_begin_addr;

                //printf("\n\nWOD end.\n\n");

        }
}

