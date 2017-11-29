
#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>


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
#endif

extern OS_EVENT * SpiQ;
extern OS_EVENT * SendQ;
extern OS_EVENT * Send_rfQ;

extern INT8U  rf_flag;



void Spi_Main (void * ppdata) {

        INT8U spircv[40] = {0xeb, 0x90, 0xb4, 0x1c, 0x3f, 0x00, 0x01, 0x00, 0x00};
        INT8U spisend[24]= {0xeb, 0x90, 0x00, 0x1d};
        INT8U spibuf[24];
        INT32U k,i,n;
        INT8U  * spi, err;
        INT32U addr;
        INT8U dest, len, spi_flag;
        //char *buf;

        for (;;) {
         //       buf = (char *) malloc(100);
                //printf (" _________ spi task\n");
                spi = (INT8U *)OSQPend(SpiQ,0,&err);
                //printf (" _________ spi start\n");

                dest = *(spi + 7);//目的地址 转换 
                len = *(spi + 8);
                spi_flag = *(spi + 4);

                memset (spisend, 0, 24);
                memset (spircv, 0, 40);

                spisend[0] = 0xeb ;
                spisend[1] = 0x90 ;
                spisend[2] = *(spi + 4); // 方式字。
                spisend[3] = 0x1d ;


                spircv[0] = 0xeb;
                spircv[1] = 0x90;
                spircv[2] = 0xb4;
                spircv[3] = 0x1c;
                spircv[4] = 0x3f;
                spircv[5] = dest;
                spircv[6] = 0x01;
                spircv[7] = 0x00;
                spircv[8] = 0x12;


                if (dest == 0x10 || dest == 0x11 || dest == 0x12 || dest == 0x13 || dest == 0x14 || dest == 0x15 || dest == 0x16 || dest == 0x17 ||dest == 0x18 || dest == 0x1D || dest == 0x20 || dest == 0x21) {
                        spisend[4] = 0x00;
                        spisend[5] = dest;
                        spircv[5] = dest;
                }
                else if(dest == 0x40){ 
                        spisend[4] = 0x02;
                        spisend[5] = 0x00;
                        spircv[5] = dest;
                }
                else if(dest == 0x41){
                        spisend[4] = 0x02;
                        spisend[5] = 0x08;
                        spircv[5] = dest;
                }
                else if(dest == 0x43){
                        spisend[4] = 0x02;
                        spisend[5] = 0x18;
                        spircv[5] = dest;
                }
                else if(dest == 0x4B){
                        spisend[4] = 0x02;
                        spisend[5] = 0x58;
                        spircv[5] = dest;
                }
                else if(dest == 0x53){
                        spisend[4] = 0x02;
                        spisend[5] = 0x98;
                        spircv[5] = dest;
                }
                else if(dest == 0x33 || dest == 0x35 || dest == 0x37 || dest == 0x39 || dest == 0x3b){
                        spisend[4] = 0x00;
                        spisend[5] = 0x13;
                        spircv[5] = dest;
                }
                else if(dest == 0x34 || dest == 0x36 || dest == 0x38 || dest == 0x3a || dest == 0x3c){
                        spisend[4] = 0x00;
                        spisend[5] = 0x14;
                        spircv[5] = dest;
                }
                else {
                        continue;
                }


                for (k = 0; k < len; k ++) 
                        spisend [k + 6] = *(spi + 9 + k);


                /*
                printf ("spi send  ::::\n");
                printf ("spi   len = %d\n", len);
                for (k = 0; k < 24 ; k ++) 
                        printf (" spi_main   send   %02x\n", spisend[k]);
                */
                

                memset (spibuf, 0, 24);



                OSSchedLock();
                for (k = 0; k < 24 ; k ++) 
                        spi_write_test (spisend[k]);
                OSSchedUnlock();


                OSTimeDly(100);


                //printf ("_____________second    send\n");
                OSSchedLock();
                for (k = 0; k < 24 ; k ++) 
                        spibuf[k] = spi_write_test (spisend[k]);
                OSSchedUnlock();



                for (k = 0; k < 18; k ++)
                        spircv[k + 9] = spibuf[k + 6] ;


                /*
                printf ("spi rcv ::::::\n");
                for(k = 0; k < 29; k++) 
                        printf ("spi_main  rcv  %02x\n" ,spircv[k]);
                printf ("\n");
                */


                //for (k = 0; k < 30; k ++)
                 //       *(volatile INT8U *)(addr + k) = *(spircv + k);


                if (rf_flag == 0)
                        OSQPost (SendQ, spircv);
                if (rf_flag == 1)
                        OSQPost (Send_rfQ, spircv);


        }


}
