
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
        INT8U spisend[14]= {0xeb, 0x90, 0x03,0x04,0x05,0x06,0x11,0x11,0x11,0x11,0x22,0x22,0x22,0x22};
        INT8U spisend1[14]= {0xeb, 0x90, 0x03,0x04,0x05,0x06,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
        INT8U spibuf[24];
        INT32U k,i,n;
        INT8U  * spi;
        INT32U addr;
        INT8U dest;
        INT8U a0=0xeb,a1=0x90, a2=0x90,a3=0x03,a4=0x05,a5=0x06,a6=0x06,a7=0x11,a8=0x12,a9=0x14,a10=0x15,a11=0x16,a12=0x17,a13=0x18;

 //       printf ("SPI_MAIN\n");
        for (;;) {

#if 0
                printf ("**********************************************************************\n");
                printf ("**********************************************************************\n");
                printf ("11 11 11 11 22 22 22 22\n");
#endif

                OSSchedLock();
                for (k = 0; k < 14; k ++) 
                        spi_write_test (spisend[k]);
                OSSchedUnlock();




                OSSchedLock();
                for (k = 0; k < 14 ; k ++) 
                        spibuf[k] = spi_write_test (spisend[k]);
                OSSchedUnlock();


    //            for(k = 0; k < 14; k++) 
     //                   printf ("spibuf[%d] = %x\n", k , spibuf[k]);

/*   正常  代码  */
      //          printf ("**********************************************************************\n");
       //         printf ("**********************************************************************\n");

                OSSchedLock();
                for (k = 0; k < 14 ; k ++) 
                        spircv[k]=spi_write_test (spisend[k]);
                OSSchedUnlock();

                OSTimeDly(500);


        //        for(k = 0; k < 14; k++) 
         //               printf ("spircv[%d] = %x\n", k , spircv[k]);






#if 1
    //            printf ("////////////////////////////////////////\n");
     //           printf ("////////////////////////////////////////\n");
      //          printf ("////////////////////////////////////////\n");


                spibuf[1]=spi_write_test (a1);
                spibuf[2]=spi_write_test (a2);
                spibuf[3]=spi_write_test (a3);
                spibuf[4]=spi_write_test (a4);
                spibuf[5]=spi_write_test (a5);
                spibuf[6]=spi_write_test (a6);
                spibuf[7]=spi_write_test (a7);
                spibuf[8]=spi_write_test (a8);
                spibuf[9]=spi_write_test (a9);
                spibuf[10]=spi_write_test (a10);
                spibuf[11]=spi_write_test (a11);
                spibuf[12]=spi_write_test (a12);
                spibuf[13]=spi_write_test (a13);




                /*

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




                printf ("***********************************************************\n");
                printf ("***********************************************************\n");
                printf ("***********************************************************\n");
*/


                spibuf[0]=spi_write_test (a0);
                spibuf[1]=spi_write_test (a1);
                spibuf[2]=spi_write_test (a2);
                spibuf[3]=spi_write_test (a3);
                spibuf[4]=spi_write_test (a4);
                spibuf[5]=spi_write_test (a5);
                spibuf[6]=spi_write_test (a6);
                spibuf[7]=spi_write_test (a7);
                spibuf[8]=spi_write_test (a8);
                spibuf[9]=spi_write_test (a9);
                spibuf[10]=spi_write_test (a10);
                spibuf[11]=spi_write_test (a11);
                spibuf[12]=spi_write_test (a12);
                spibuf[13]=spi_write_test (a13);
                



/*
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
*/

#endif

#if 1
    //            printf ("************************************\n");
     //           printf ("************************************\n");


                for (k = 0 ; k < 14; k ++) {
                        spircv[k] = spi_write_test (spisend1[k]);
                }


      //          for (k = 0; k < 14; k++) {
       //                 printf ("spircv[%d]   =  %x\n",  k, spircv[k]);
        //        }

         //       printf ("************************************\n");
          //      printf ("************************************\n");
           //     printf ("111111111111111111111\n");
            //    printf ("out  55 55 55 55 55 \n");

#endif
#if 1
                /*   正常通信代码  */

                for (k = 100; k > 0 ; k--) {
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
                }

#if 0
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



                printf ("************************************\n");
                printf ("************************************\n");
                printf ("222222222222222222222222222222\n");
                printf ("out  11 22  \n");
#endif

                for (k = 100; k > 0 ; k--) {

                        OSSchedLock();
                        spibuf[0]=spi_write_test (a0);
                        //OSTimeDly(1);
                        spibuf[1]=spi_write_test (a1);
                        spibuf[2]=spi_write_test (a2);
                        spibuf[3]=spi_write_test (a3);
                        spibuf[4]=spi_write_test (a4);
                        spibuf[5]=spi_write_test (a5);
                        spibuf[6]=spi_write_test (a6);
                        spibuf[7]=spi_write_test (a7);
                        spibuf[8]=spi_write_test (a8);
                        spibuf[9]=spi_write_test (a9);
                        spibuf[10]=spi_write_test (a10);
                        spibuf[11]=spi_write_test (a11);
                        spibuf[12]=spi_write_test (a12);
                        spibuf[13]=spi_write_test (a13);

                        OSSchedUnlock();
                }

#if 0

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


#endif
               
                /*   正常代码  结束，以上是  可以正确执行的 代码。。。*/

#endif


#if 1
    //            printf ("************************************************************\n");
     //           printf ("************************************************************\n");
      //          printf ("11 11 11 11 22 22 22 22 \n");

                spibuf[0]=spi_write_test (spisend[0]);
                spibuf[1]=spi_write_test (spisend[1]);
                spibuf[2]=spi_write_test (spisend[2]);
                spibuf[3]=spi_write_test (spisend[3]);
                spibuf[4]=spi_write_test (spisend[4]);
                spibuf[5]=spi_write_test (spisend[5]);
                spibuf[6]=spi_write_test (spisend[6]);
                spibuf[7]=spi_write_test (spisend[7]);
                spibuf[8]=spi_write_test (spisend[8]);
                spibuf[9]=spi_write_test (spisend[9]);
                spibuf[10]=spi_write_test (spisend[10]);
                spibuf[11]=spi_write_test (spisend[11]);
                spibuf[12]=spi_write_test (spisend[12]);
                spibuf[13]=spi_write_test (spisend[13]);

/*
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
                */

#endif

#if 1
                
    /*            printf ("************************************\n");
                printf ("************************************\n");
                printf ("222222222222222222222222\n");
                printf ("out  55 55 55 55 55 \n");

                for (k = 0; k < 14; k++) {
                        printf ("spibuf[%d]   =  %x\n",  k, spibuf[k]);
                }


                printf ("************************************\n");
                printf ("************************************\n");
                printf ("3333333333333333333\n");
                printf ("out 11 11 11 11 22 22 22 22 \n");

*/
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

/*
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
*/

#endif


#if 1
/*
                printf ("************************************\n");
                printf ("************************************\n");
                printf ("555555555555555555555555\n");
                printf ("out 11 11 11 11 22 22 22 22 \n");

*/



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


                for (k = 0; k < 14; k++){
                        spircv[k]=spi_write_test (spisend[k]);
                }

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
