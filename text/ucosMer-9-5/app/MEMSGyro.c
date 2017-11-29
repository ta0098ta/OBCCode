#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern OS_EVENT *MEMSGyroQ;
extern OS_EVENT *RS422_Q_Rec;
extern OS_EVENT* sendMsg;

extern OS_EVENT *SHARE_PRINTF;//共享控制
extern OS_EVENT *SHARE_SENDDOWN;
extern OS_EVENT *SHARE_CAN;  //CAN

extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;



void MEMSGyro_Main(void * ppdata)
{
        INT8U err,Data = 0x67;
        INT16U i;
        INT8U *MEMSGyromsg, *FrameBack;
        INT32U addr;
        INT8U down_frame[2];

       // OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
        //printf("task MEMSGyro ready\n");
        //OSSemPost(SHARE_PRINTF);

        addr =  MEMSGyro_DATA;

        for(;;)
        {

                MEMSGyromsg = (INT8U *)OSQPend(MEMSGyroQ,0,&err);
                //printf("MemsGyro\n");
                RS422_Data_Send( 2 , &Data, 0 , 1); 
                FrameBack=(INT8U *)OSQPend(RS422_Q_Rec,2000,&err);//2000个时钟节拍超时
                if(err==OS_NO_ERR)	
                {
                        for(i = 0; i < 21; i++)
                        {
                                //printf("Gyro[%d] %02x  ",i, *(FrameBack+i));
                                *(volatile INT8U *)(addr+i) = *(FrameBack+i+2);
                        }
                        down_frame[0]=0x06;
                        down_frame[1]=0x02;
                        //printf("0x06");
                        OSQPost(sendMsg,down_frame);
                }
                else
                {
                        //printf("Wrong!");
                }
                //		OSTimeDly(OS_TICKS_PER_SEC);
        }
}

