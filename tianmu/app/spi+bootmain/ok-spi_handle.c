
#include "includes.h"
#include "Includes.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#if 0
extern OS_EVENT *MEMSGyroQ;
extern OS_EVENT *RS422_Q_Rec;
extern OS_EVENT* sendMsg;

extern OS_EVENT *SHARE_PRINTF;//共享控制
extern OS_EVENT *SHARE_SENDDOWN;
extern OS_EVENT *SHARE_CAN;  //CAN

extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;

        INT8U err, Data = 0x67;
        INT16U i;
        INT8U *MEMSGyromsg, *FrameBack;
        INT32U addr;
        INT8U down_frame[2];

#endif


void Spi_Main (void * ppdata) {


        INT8U spircv[28],spibuf[14];
        INT8U spisend[28]={0xeb,0x90,0x03,0x04,0x05,0x06,0x11,0x11,0x11,0x11,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x88,0x99,0x99};
        INT8U spisend1[28]={0xeb,0x90,0x03,0x04,0x05,0x06,0x11,0x11,0x11,0x11,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x88,0x99,0x99};
        INT8U spisend2[28]={0xeb,0x90,0x03,0x04,0x05,0x06,0x11,0x11,0x11,0x11,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x88,0x99,0x99};
        INT8U spisend3[14]={0xeb,0x90,0x03,0x04,0x05,0x06,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
        INT32U k,i,n;




        printf ("Spi_Main\n");


        for (;;) {

                //MEMSGyromsg = (INT8U *)OSQPend(MEMSGyroQ,0,&err);
                //printf("MemsGyro\n");


                printf ("////////////////////////////////////////\n");
                printf ("////////////////////////////////////////\n");
                printf ("////////////////////////////////////////\n");


                OSSchedLock();
                for (k = 0; k < 14 ; k ++){
                        spircv[k] = spi_write_test (spisend3[k]);
                }
                OSSchedUnlock();

                for (k = 0; k < 14; k ++)
                        printf ("spircv [%d] =  %x\n", k , spircv[k]);
               
                OSTimeDly(10000);

#if 0

                printf ("***********************************************************\n");
                printf ("***********************************************************\n");
                printf ("***********************************************************\n");


                OSSchedLock();
                for (k = 0; k < 28 ; k ++)
                        spircv[k] = spi_write_test (spisend[k]);
                OSSchedUnlock();

                for (k = 0; k < 28; k ++)
                        printf ("spircv [%d] =  %x\n", k , spircv[k]);
               
                OSTimeDly(10000);



                printf ("************************************\n");
                printf ("************************************\n");


                for (k = 0 ; k < 14; k ++) {
                        spircv[k] = spi_write_test (spisend1[k]);
                }


                for (k = 0; k < 14; k++) {
                        printf ("spircv[%d]   =  %x\n",  k, spircv[k]);
                }

                for (i = 10000; i >0; i--) 
                        for (n=100;n>0;n--);

#endif
#if 0
                /*   此段 代码 有时是 可以执行的，但 修改了 函数 参数 就不可以 正确接收 发送 数据了   */
                printf ("************************************\n");
                printf ("************************************\n");
                printf ("111111111111111111111\n");
                printf ("out  55 55 55 55 55 \n");

                OSSchedLock();
                spibuf[0]=spi_write_test (0xeb);
                spibuf[1]=spi_write_test (0x90);
                spibuf[2]=spi_write_test (0x03);
                spibuf[3]=spi_write_test (0x04);
                spibuf[4]=spi_write_test (0x05);
                spibuf[5]=spi_write_test (0x06);
                spibuf[6]=spi_write_test (0x55);
                spibuf[7]=spi_write_test (0x55);
                spibuf[8]=spi_write_test (0x55);
                spibuf[9]=spi_write_test (0x55);
                spibuf[10]=spi_write_test (0x55);
                spibuf[11]=spi_write_test (0x55);
                spibuf[12]=spi_write_test (0x55);
                spibuf[13]=spi_write_test (0x55);

                OSSchedUnlock();

                printf ("spibuf[0] = %x\n", spibuf[0]);
                printf ("spibuf[1] = %x\n", spibuf[1]);
                printf ("spibuf[2] = %x\n", spibuf[2]);
                printf ("spibuf[3] = %x\n", spibuf[3]);
                printf ("spibuf[4] = %x\n", spibuf[4]);
                printf ("spibuf[5] = %x\n", spibuf[5]);
                printf ("spibuf[6] = %x\n", spibuf[6]);
                printf ("spibuf[7] = %x\n", spibuf[7]);
                printf ("spibuf[8] = %x\n", spibuf[8]);
                printf ("spibuf[9] = %x\n", spibuf[9]);
                printf ("spibuf[10] = %x\n", spibuf[10]);
                printf ("spibuf[11] = %x\n", spibuf[11]);
                printf ("spibuf[12] = %x\n", spibuf[12]);
                printf ("spibuf[13] = %x\n", spibuf[13]);


                OSTimeDly(10000);

#endif


#if 0
                printf ("************************************************************\n");
                printf ("************************************************************\n");
                printf ("11 11 11 11 22 22 22 22 \n");

                spibuf[0]=spi_write_test (spisend3[0]);
                spibuf[1]=spi_write_test (spisend3[1]);
                spibuf[2]=spi_write_test (spisend3[2]);
                spibuf[3]=spi_write_test (spisend3[3]);
                spibuf[4]=spi_write_test (spisend3[4]);
                spibuf[5]=spi_write_test (spisend3[5]);
                spibuf[6]=spi_write_test (spisend3[6]);
                spibuf[7]=spi_write_test (spisend3[7]);
                spibuf[8]=spi_write_test (spisend3[8]);
                spibuf[9]=spi_write_test (spisend3[9]);
                spibuf[10]=spi_write_test (spisend3[10]);
                spibuf[11]=spi_write_test (spisend3[11]);
                spibuf[12]=spi_write_test (spisend3[12]);
                spibuf[13]=spi_write_test (spisend3[13]);


                printf ("spibuf[0] = %x\n", spibuf[0]);
                printf ("spibuf[1] = %x\n", spibuf[1]);
                printf ("spibuf[2] = %x\n", spibuf[2]);
                printf ("spibuf[3] = %x\n", spibuf[3]);
                printf ("spibuf[4] = %x\n", spibuf[4]);
                printf ("spibuf[5] = %x\n", spibuf[5]);
                printf ("spibuf[6] = %x\n", spibuf[6]);
                printf ("spibuf[7] = %x\n", spibuf[7]);
                printf ("spibuf[8] = %x\n", spibuf[8]);
                printf ("spibuf[9] = %x\n", spibuf[9]);
                printf ("spibuf[10] = %x\n", spibuf[10]);
                printf ("spibuf[11] = %x\n", spibuf[11]);
                printf ("spibuf[12] = %x\n", spibuf[12]);
                printf ("spibuf[13] = %x\n", spibuf[13]);

                for (i = 100000; i >0; i--) 
                        for (n=100000;n>0;n--);

                for (i = 100000; i >0; i--) 
                        for (n=100000;n>0;n--);

                for (i = 100000; i >0; i--) 
                        for (n=100000;n>0;n--);

#endif

#if 0
                for (i = 10000; i >0; i--) 
                        for (n=100;n>0;n--);
                
                printf ("************************************\n");
                printf ("************************************\n");
                printf ("222222222222222222222222\n");
                printf ("out  55 55 55 55 55 \n");


                for (k = 0; k < 14; k++) {
                        printf ("spibuf[%d]   =  %x\n",  k, spibuf[k]);
                }

                for (i = 10000; i >0; i--) 
                        for (n=100;n>0;n--);

                printf ("************************************\n");
                printf ("************************************\n");
                printf ("3333333333333333333\n");
                printf ("out 11 11 11 11 22 22 22 22 \n");

                spibuf[0]=spi_write_test (spisend[0]);
                spibuf[1]=spi_write_test (spisend[2]);
                spibuf[2]=spi_write_test (spisend[3]);
                spibuf[3]=spi_write_test (spisend[4]);
                spibuf[4]=spi_write_test (spisend[5]);
                spibuf[5]=spi_write_test (spisend[6]);
                spibuf[6]=spi_write_test (spisend[7]);
                spibuf[7]=spi_write_test (spisend[8]);
                spibuf[8]=spi_write_test (spisend[9]);
                spibuf[9]=spi_write_test (spisend[10]);
                spibuf[10]=spi_write_test (spisend[11]);
                spibuf[11]=spi_write_test (spisend[12]);
                spibuf[12]=spi_write_test (spisend[13]);
                spibuf[13]=spi_write_test (spisend[14]);


                printf ("spibuf[0] =     %x\n", spibuf[0]);
                printf ("spibuf[1] =     %x\n", spibuf[1]);
                printf ("spibuf[2] =     %x\n", spibuf[2]);
                printf ("spibuf[3] =     %x\n", spibuf[3]);
                printf ("spibuf[4] =     %x\n", spibuf[4]);
                printf ("spibuf[5] =     %x\n", spibuf[5]);
                printf ("spibuf[6] =     %x\n", spibuf[6]);
                printf ("spibuf[7] =     %x\n", spibuf[7]);
                printf ("spibuf[8] =     %x\n", spibuf[8]);
                printf ("spibuf[9] =     %x\n", spibuf[9]);
                printf ("spibuf[10] =    %x\n", spibuf[10]);
                printf ("spibuf[11] =    %x\n", spibuf[11]);
                printf ("spibuf[12] =    %x\n", spibuf[12]);
                printf ("spibuf[13] =    %x\n", spibuf[13]);



                printf ("************************************\n");
                printf ("************************************\n");
                printf ("4444444444444444444444\n");
                printf ("out 11 11 11 11 22 22 22 22 \n");

                for (k = 0; k < 14; k++) {
                        printf ("spibuf[%d]   =  %x\n",  k, spibuf[k]);
                }

                for (i = 10000; i >0; i--) 
                        for (n=100;n>0;n--);

#endif



#if 0
                printf ("************************************\n");
                printf ("************************************\n");
                printf ("555555555555555555555555\n");
                printf ("out 11 11 11 11 22 22 22 22 \n");


                spi_write (spisend1,14,spisend2,14);
                for (k = 10000; k > 0 ; k--) {
                        delay (10000);
                }


                for (i = 1000000; i >0; i--) 
                        for (n=1000000;n>0;n--);
                for (i = 1000000; i >0; i--) 
                        for (n=100000;n>0;n--);


                spi_write (spisend1,14,spisend2,14);
                for (k = 10000; k > 0 ; k--) {
                        delay (10000);
                }


                for (i = 1000000; i >0; i--) 
                        for (n=100000;n>0;n--);


                spi_read (spisend1, 14, spircv, 14);
                for (k = 0; k < 14 ; k++)
                        printf ("spircv [%d] = %x\n", k , spircv[k]);


#endif




#if 0
                spircv[0]=spi_write_test (spisend[0]);
                spircv[1]=spi_write_test (spisend[1]);
                spircv[2]=spi_write_test (spisend[2]);
                spircv[3]=spi_write_test (spisend[3]);
                spircv[4]=spi_write_test (spisend[4]);
                spircv[5]=spi_write_test (spisend[5]);
                spircv[6]=spi_write_test (spisend[6]);
                spircv[7]=spi_write_test (spisend[7]);
                spircv[8]=spi_write_test (spisend[8]);
                spircv[9]=spi_write_test (spisend[9]);
                spircv[10]=spi_write_test (spisend[10]);
                spircv[11]=spi_write_test (spisend[11]);
                spircv[12]=spi_write_test (spisend[12]);
                spircv[13]=spi_write_test (spisend[13]);
                spircv[14]=spi_write_test (spisend[14]);
                spircv[15]=spi_write_test (spisend[15]);
                spircv[16]=spi_write_test (spisend[16]);
                spircv[17]=spi_write_test (spisend[17]);
                spircv[18]=spi_write_test (spisend[18]);
                spircv[19]=spi_write_test (spisend[19]);
                spircv[20]=spi_write_test (spisend[20]);
                spircv[21]=spi_write_test (spisend[21]);
                spircv[22]=spi_write_test (spisend[22]);
                spircv[23]=spi_write_test (spisend[23]);
                spircv[24]=spi_write_test (spisend[24]);
                spircv[25]=spi_write_test (spisend[25]);
                spircv[26]=spi_write_test (spisend[26]);
                spircv[27]=spi_write_test (spisend[27]);

                for (k = 0; k < 28; k ++)
                        printf ("spircv[%d] = %x\n", k, spircv[k]);

                printf ("***********************************\n");
                printf ("***********************************\n");
                for (k = 0; k < 14; k++){
                        spircv[k]=spi_write_test (spisend[k]);
                        for (i = 1000; i >0; i--) 
                                for (n=100;n>0;n--);
                }

                for (k = 0; k < 14; k ++)
                        printf ("spircv[%d] = %x\n", k, spircv[k]);

                printf ("***********************************\n");
                printf ("///////////////////////////////////\n");
                printf ("***********************************\n");
#endif


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
