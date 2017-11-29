
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


void Spi_Main (void * ppdata) {

        INT8U spircv[30] = {0xeb, 0x90, 0xb4, 0x1c, 0x3f, 0x00, 0x01, 0x00, 0x00};
        INT8U spisend[24]= {0xeb, 0x90, 0x00, 0x1d};
        INT8U spibuf[24];
        INT32U k,i,n;
        INT8U  * spi;
        INT32U addr;
        INT8U dest;

        for (;;) {
                spi = (INT8U *)OSQPend(MEMSGyroQ,0,&err);

                spisend[2] = *(spi + 4); // 方式字。
                dest = *(spi + 7);//目的地址 转换 

                if (dest == 0x10 || dest == 0x11 || dest == 0x12 || dest == 0x13 || dest == 0x14 || dest == 0x15 || dest == 0x16 || dest == 0x17 ||dest == 0x18 || dest == 0x1D || dest == 0x20 || dest == 0x21) {
                        spisend[4] = 0x00;
                        spisend[5] = dest;
                        spircv[5] = dest;
                        spircv[8] = 0x08;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x40){ 
                        spisend[4] = 0x02;
                        spisend[5] = 0x00;
                        spircv[5] = dest;
                        spircv[8] = 0x08;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x41){
                        spisend[4] = 0x02;
                        spisend[5] = 0x08;
                        spircv[5] = dest;
                        spircv[8] = 0x08;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x43){
                        spisend[4] = 0x02;
                        spisend[5] = 0x18;
                        spircv[5] = dest;
                        spircv[8] = 0x12;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x4B){
                        spisend[4] = 0x02;
                        spisend[5] = 0x58;
                        spircv[5] = dest;
                        spircv[8] = 0x12;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x53){
                        spisend[4] = 0x02;
                        spisend[5] = 0x98;
                        spircv[5] = dest;
                        spircv[8] = 0x12;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x33 || dest == 0x35){
                        spisend[4] = 0x00;
                        spisend[5] = 0x13;
                        spircv[5] = dest;
                        spircv[8] = 0x08;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                else if(dest == 0x34 || dest == 0x36){
                        spisend[4] = 0x00;
                        spisend[5] = 0x14;
                        spircv[5] = dest;
                        spircv[8] = 0x08;

                        spisend[0] = 0xeb;
                        spisend[1] = 0x90;
                        spisend[3] = 0x1d;
                        spircv[0] = 0xeb;
                        spircv[1] = 0x90;
                        spircv[2] = 0xb4;
                        spircv[3] = 0x1c;
                        spircv[4] = 0x3f;
                        spircv[6] = 0x00;
                        spircv[7] = 0x01;
                }
                for (k = 0; k < 8; k ++) 
                        spisend [k + 6] = *(spi + 9 + k);



                for (k = 0; k < 24 ; k ++) 
                        printf (" spisend [%d] ==  %x\n", k, spisend[k]);

                OSSchedLock();
                for (k = 0; k < 24 ; k ++) 
                        spi_write_test (spisend[k]);
                OSSchedUnlock();


                OSTimeDly(1000);

                memset (spibuf, 0, 24);

                OSSchedLock();
                for (k = 0; k < 24 ; k ++) 
                        spibuf[k] = spi_write_test (spisend[k]);
                OSSchedUnlock();


                for(k = 0; k < 24; k++) 
                        printf ("spibuf[%d] = %x\n", k , spibuf[k]);



                for (k = 0; k < 18; k ++)
                        spircv[k + 9] = spibuf[k + 6] ;

                for (k = 0; k < 30; k ++)
                        *(volatile INT8U *)(addr + k) = *(spircv + k);

                OSQPost(sendMsg,spircv);










#if 0
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
#endif


        }


}
