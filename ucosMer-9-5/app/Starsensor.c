#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern OS_EVENT* sendMsg;
extern OS_EVENT* Hdlc_Flag; 
extern OS_EVENT* RecMsg; 
extern OS_EVENT* HdlcQEvent; 
extern OS_EVENT* sendRecMsg; 
extern OS_EVENT* sendQEvent; 
extern OS_EVENT* timeoutQEvent;


extern OS_EVENT* SS_OSQ;
extern OS_EVENT *MIMU_HDLC_REC;
extern OS_EVENT *SHARE_PRINTF;//共享控制
extern OS_EVENT *SHARE_SENDDOWN;
extern OS_EVENT *SHARE_CAN;  //CAN
extern OS_EVENT* transeiver_receiveQ;
extern OS_EVENT* ADCS_HDLC_REC;
extern OS_EVENT* DOWN_REMOTE_DATA;
extern OS_EVENT *RS422_Q_Rec;
extern OS_EVENT *EIS0_photographQ;
extern OS_EVENT *EIS1_photographQ;


extern OS_EVENT *LaserEventQ;
extern OS_EVENT *DLL_HDLC_REC;
extern OS_EVENT *RFS_HDLC_REC;

extern INT8U LockFirst; 

#ifndef __utype_defined
#define __utype_defined
typedef INT8U	u_char;
typedef INT16U  ushort;
typedef INT32U  uint;
typedef INT32U  ulong;
#endif

#define EIS_0	0x06
#define EIS_1	0x08
#define GPS_0	0x07
#define MIMU_0	0x0F
#define DLL_0	0x0B
#define ACS_0	0x02
#define RF_0	0x04	
#define WOD_0	0x0C
#define STAR_0	0x05
#define CJ_0	0x09

#define SEND_TIMEOUT 0xDD
#define SEND_STOP 0xEE

extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;
extern INT8U StarFlag;


extern INT8U err_fault[5];
extern void changemux(INT32U a);
extern void RS422_Data_Send( INT8U type , INT8U *data , INT8U lenH, INT8U lenL);



void StarSensorTask(void *jdata)  //星敏主任务
{
        INT8U *SSmsg,*downmsg,err,i,ErrTimes,_isSuccess;
        INT8U down_frame[2];
        INT8U datarequest[20],*databack;
        INT16U byteCount,k;
        INT16U packageNUM;
        INT32U addr,j,SmallAddr;
        INT16U id, PackNum;
        INT8U Test[27] = {0x74,0xA1,0xFF,0x16,0x00,0x64,0xBE,0x30,0x40,
                0x7D,0x65,0x18,0x00,0xFE,0xC4,0xBF,0x00,0x25,0xAD,0x60,
                0x02,0x30,0xD8,0x6,0x23,0x16,0x61};

        addr = STAR_sensor_DATA;
        //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
        ////printf("Star Sensor task start. \n");
        //OSSemPost(SHARE_SENDDOWN);

        for(;;)
        {
                SSmsg = (INT8U *)OSQPend(SS_OSQ,0,&err);

                id=*(SSmsg-6)+(*(SSmsg-5))*0x100;  //计算id
                byteCount = *SSmsg+(*(SSmsg+1))*0x100;   //计算指令长度
                //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                ////printf("id = %04x\n",id);
                ////printf("SSmsg - 6 = %04x\nSSmsg - 5 = %04x\nSSmsg - 4 = %04x\nSSmsg - 3 = %04x\nSSmsg - 2 = %04x\nSSmsg - 1 = %04x\nSSmsg= %04x\nSSmsg + 1 = %04x\nSSmsg + 2 = %04x\nSSmsg + 3 = %04x\nSSmsg + 4 = %04x\n",*(SSmsg-6),*(SSmsg-5),*(SSmsg-4),*(SSmsg-3),*(SSmsg-2),*(SSmsg-1),*(SSmsg),*(SSmsg+1),*(SSmsg+2),*(SSmsg+3),*(SSmsg+4));
                //OSSemPost(SHARE_SENDDOWN);

                switch(id)
                {
                        case 0x0100://获取星敏数据
                                //  send_down_frame(i_data,0,0,Test);
                                /**/
                                ////printf("0x01");
                                SSmsg += 2;	   		
                                ErrTimes=3;
                                _isSuccess=0;
                                while(ErrTimes>0 && _isSuccess==0)
                                {
                                        RS422_Data_Send( 2 ,(INT8U *)SSmsg, (INT8U)(byteCount>>8) , (INT8U)(byteCount&0x00FF)); 
                                        downmsg=(INT8U *)OSQPend(RS422_Q_Rec,3000,&err);	
                                        if(err==OS_NO_ERR)
                                        {
                                                ErrTimes=0;
                                                _isSuccess=1;
                                        }
                                        else
                                                ErrTimes--;
                                }
                                if(_isSuccess==1)
                                {
                                        ////printf("ram= %x %x %x %x %x %x\n",*downmsg,*(downmsg+1),*(downmsg+2),*(downmsg+3),*(downmsg+4),*(downmsg+5));
                                        //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                        //send_down_frame(i_data,0,0,downmsg);
                                        //OSSemPost(SHARE_SENDDOWN);
                                        ////printf("\n");
                                        for(j=0;j<27;j++)
                                        {
                                                ////printf("%02x ",*(downmsg+j+2));
                                                *(volatile INT8U *)(addr+j) = *(downmsg+j+2);
                                        }
                                        down_frame[0]=0x05;
                                        down_frame[1]=0x01;
                                        ////printf("0x03\n");
                                        OSQPost(sendMsg,down_frame);

                                        ////printf("\nSensor task finished\n");
                                }
                                else 
                                {
                                        //err_fault[2]=0x1;
                                        //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                        //send_down_frame(i_data,0,0,err_fault);
                                        //OSSemPost(SHARE_SENDDOWN);
                                        //printf("Star Sensor Task could not work well with Star Sensor.\n ");
                                }	

                                break;

                        case 0x0200://获取星敏图像到obc
                                //printf("0x02\n");

                                datarequest[0] = 0x74;
                                datarequest[1] = 0xA2;
                                datarequest[2] = 0xFF;
                                datarequest[3] = 0x02;

                                LockFirst = 3;

                                for(j = 0; j < 8192; j++)//循环发起【请求数据帧】命令
                                {
                                        datarequest[4] = (INT8U)( j>>8 );  //数据段高字节
                                        datarequest[5] = (INT8U)(j&0x00FF); //数据段低字节
                                        datarequest[6] = (INT8U)( (datarequest[4]+datarequest[5])&0x00FF);  //校验和
                                        _isSuccess=0;
                                        ErrTimes=3;
                                        while(ErrTimes>0 && _isSuccess==0)
                                        {
                                                RS422_Data_Send( 2 ,(INT8U *)datarequest, 0 , 7); 
                                                //printf("j = %x\n",j);
                                                //printf("%02x ",datarequest[0]);
                                                //printf("%02x ",datarequest[1]);
                                                //printf("%02x ",datarequest[2]);
                                                //printf("%02x ",datarequest[3]);
                                                //printf("%02x ",datarequest[4]);
                                                //printf("%02x ",datarequest[5]);
                                                //printf("%02x ",datarequest[6]);
                                                //printf("\n ");
                                                databack=(INT8U *)OSQPend(RS422_Q_Rec,9000,&err); //得到星敏的图像数据
                                                if(err==OS_NO_ERR )
                                                {
                                                        ErrTimes=3;
                                                        _isSuccess=1;

                                                }
                                                else
                                                        ErrTimes--;
                                        }
                                        if(_isSuccess != 1)
                                        {
                                                //printf("ST task can't connect well with ST\n");
                                                break;
                                        }

                                        for(k=0; k < 128; k++)//数据存储到6M数据区
                                        {
                                                *(volatile INT8U *)(EIS_DATA+128*j+k) = *(databack+8+k); 
                                        }

                                }
                                if( j < 8192 )
                                {
                                        LockFirst = 1;
                                        break;
                                }
                                //printf("Small OK - 1!\n");
                                SmallAddr = EIS_DATA_SMALL;
                                for(j = 0; j < 256; j++)
                                {
                                        for(k = 0; k < 256; k++)
                                                *(volatile INT8U*)(SmallAddr+j*256+k) = *(volatile INT8U *)(EIS_DATA + 1024*j*4 + k*4);
                                }
                                //printf("Small OK!\n");
                                StarFlag = 1;
                                LockFirst = 1;
                                break;

                        case 0x0300:  //传27个字节遥测数据
                                down_frame[0]=0x05;
                                down_frame[1]=0x01;
                                //printf("0x03\n");
                                OSQPost(sendMsg,down_frame);
                                break;

                        case 0x0400:   //传图像
                                down_frame[0]=0x05;
                                down_frame[1]=0x02;
                                //printf("0x04");
                                OSQPost(sendMsg,down_frame);
                                break;

                        case 0x0700:
                                down_frame[0]=0x05;
                                down_frame[1]=0x04;
                                //printf("0x07");
                                OSQPost(sendMsg,down_frame);
                                break;
                        default:
                                //printf("Default\n");
                                SSmsg += 2;	   	
                                ErrTimes=3;
                                _isSuccess=0;
                                while(ErrTimes>0 && _isSuccess==0)
                                {
                                        RS422_Data_Send( 2 ,(INT8U *)SSmsg, (INT8U)(byteCount>>8) , (INT8U)(byteCount&0x00FF));      		
                                        downmsg=(INT8U *)OSQPend(RS422_Q_Rec,3000,&err);	
                                        if(err==OS_NO_ERR)
                                        {
                                                ErrTimes=0;
                                                _isSuccess=1;
                                        }
                                        else
                                                ErrTimes--;
                                }
#if 0
                                if(_isSuccess==1)
                                {
                                        //printf("ram= %x %x %x %x %x %x\n",*downmsg,*(downmsg+1),*(downmsg+2),*(downmsg+3),*(downmsg+4),*(downmsg+5));
                                        //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                        //send_down_frame(i_data,0,0,downmsg);
                                        //OSSemPost(SHARE_SENDDOWN);
                                        //printf("\n");
                                       // for(j=0;j<27;j++)
                                       // {
                                                //printf("%02x ",*(downmsg+j+2));
                                                //*(volatile INT8U *)(addr+j) = *(downmsg+j+2);
                                       // }
                                        //printf("\nSensor task finished\n");
                                }
                                else 
                                {
                                        //err_fault[2]=0x1;
                                        //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                        //send_down_frame(i_data,0,0,err_fault);
                                        //OSSemPost(SHARE_SENDDOWN);
                                        //printf("Star Sensor Task could not work well with Star Sensor.\n ");
                                }
#endif

                                break;

                }
        }

}
