#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>



extern OS_EVENT * StarSQ;
extern OS_EVENT * SendQ;
extern OS_EVENT * Send_rfQ;
extern INT8U rf_flag;

void StarSensorTask(void *jdata)  //星敏主任务
{
        INT8U *SSmsg, err, DataCount, j, ss_id, ss_tf, ss_flag, i;
        INT16U id;
        INT32U  timedata, n , num, times;

                
        INT8U ssbuf[50], ssrcvbuf[100], testbuf[50];
        INT8U len, sub_id, egse_len;


        for(;;)
        {
                //printf ("_____________STARTsensor task\n");
                SSmsg = (INT8U *)OSQPend(StarSQ , 0, &err);
                //printf ("_____________STARTsensor start\n");

                len = *(SSmsg + 8 + 2);
                ss_id = *(SSmsg + 11 + 2);
                id = *(SSmsg + 7);  //计算id
                sub_id = *(SSmsg + 10 + 2);

                num = *(SSmsg + 8);
                num = num << 8;
                num += *(SSmsg + 9);

                //printf ("SS  num = %d\n", num);

                DataCount = 0;
                ss_flag = 0;


                for (n = 0; n < len; n ++) {
                        ssbuf[n] = *(SSmsg + 11 + n);
                        //printf (" SSbuf[%d]  ==  %02x\n", n, ssbuf[n]);
                }
                
                if (sub_id == 0x90){

#if 1
                        DataCount = 0;
                        for (;;){
                                if(DataCount++ > 5) 
                                        break;

                                //printf ("____________datacount = %d \n", DataCount);
                                memset(ssrcvbuf, 0, 50);


                                /*   以下 3 行 代码，scc3_serial_tstc() 永远 为 0 ，不会跳出死循环 
                                     while (!scc3_serial_tstc()){
                                //printf ("_serial_tstc : %d\n",scc3_serial_tstc());
                                }
                                scc3buf[0] = scc3_serial_getc();
                                if (scc3buf[0]!=0xEB) continue;	
                                */

                                scc2_app_init();

                                OSSchedLock();
                                //scc2_app_init ();
                                for (n = 0; n < len; n ++)
                                        scc2_serial_putc (ssbuf[n]);

                                /*
                                timedata = 0;
                                while ((!scc2_serial_tstc())&&(timedata<(200))) 
                                        timedata++;
                                if (timedata>=(200)) continue;
                                ssrcvbuf[9] = scc2_serial_getc();
                                */


                                for (j=0; j<5; j++)
                                {
                                        /*
                                        timedata = 0;
                                        while ((!scc2_serial_tstc())&&(timedata<(10000))) 
                                               timedata++;
                                        if (timedata>=(10000)) continue;
                                        */
                                        ssrcvbuf[j + 9] = scc2_serial_getc();

                                }

                                /*

                                for (j = 0; j < 5; j ++) 
                                {
                                        while (!scc2_serial_tstc())
                                                ;
                                                ssrcvbuf[j + 9] = scc2_serial_getc();
                                }
                                */


                                OSSchedUnlock();



                                //for (j = 0 ; j < 5 ; j ++)
                                 //       printf ("________ssrcvbuf[%d]  =  %x\n", j , ssrcvbuf[j+9]);



                                /*
                                for (j = 0; j < 5;j ++) {
                                        if (ssrcvbuf[j + 9] == 0x55 || ssrcvbuf[j + 9] == 0xAA) {
                                                ss_tf = ssrcvbuf[j + 9];
                                                ssrcvbuf[9] = 0xeb;
                                                ssrcvbuf[10] = 0x90;
                                                ssrcvbuf[11] = ss_id;
                                                ssrcvbuf[12] = ss_tf;
                                                ssrcvbuf[13] = (ss_id + ss_tf);

                                                ss_flag = 1;
                                        }
                                        if (ss_flag == 1)
                                                break;
                                        else
                                                continue;
                                }


                                if (ss_flag == 1)
                                        break;

                                else 
                                        continue;


                                */
                                if (ssrcvbuf[9] == 0xeb && ssrcvbuf[10] == 0x90)
                                        break;
                                else 
                                        continue;



                        } // end  of  for
#endif


                }

                if (sub_id == 0x50) {

                        times = 0;
                        for (;;) {
                                times++;
                                if (times > num)
                                        break;
                                //printf ("_______________times  = %d\n", times);

#if 1
                                DataCount = 0;
                                for (;;){
                                        if(DataCount++ > 5) 
                                                break;

                                        //printf ("____________datacount = %d \n", DataCount);
                                        memset(ssrcvbuf, 0, 100);

                                        scc2_app_init();

                                        OSSchedLock();
                                        for (n = 0; n < len; n ++)
                                                scc2_serial_putc (ssbuf[n]);


                                        /*
                                        for (j = 0; j < 50; j ++) 
                                        {

                                                timedata = 0;
                                                while ((!scc2_serial_tstc())&&(timedata<(1000 * 8))) 
                                                        timedata++;
                                                if (timedata>=(1000 * 8)) continue;
                                                ssrcvbuf[9] = scc2_serial_getc();


                                                if (ssrcvbuf[9] == 0xeb)
                                                        break;
                                                else
                                                        continue;
                                        }



                                        for (j=0; j<35; j++)
                                        {
                                                timedata = 0;
                                                while ((!scc2_serial_tstc())&&(timedata<(5000))) 
                                                        timedata++;
                                                if(timedata>=(5000)) break;
                                                ssrcvbuf[j + 10] = scc2_serial_getc();

                                        }

                                        */


                                        ///*
                                        for (j = 0; j < 36; j ++) 
                                        {
                                                //while (!scc2_serial_tstc())
                                                 //       ;
                                                ssrcvbuf[j + 9] = scc2_serial_getc();
                                        }

                                        //*/
                                        OSSchedUnlock();


                                        //for (j = 0; j < 47; j ++)
                                         //      printf ("____________________  ssrcvbuf[%d] = %x\n", j,ssrcvbuf[j]);

                                        if ((ssrcvbuf[9] == 0xeb )&& (ssrcvbuf[10] == 0x50 )&& (ssrcvbuf[11] == 0x88 || ssrcvbuf[11] == 0x99)){
                                          //      printf ("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS   datacount  = %d\n", DataCount);
                                                break;
                                        }
                                        else {
                                                continue;
                                           //     printf ("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS      else   datacount  = %d\n", DataCount);
                                        }


                                } // end  of  for

                                ssrcvbuf[0] = 0xeb;
                                ssrcvbuf[1] = 0x90;
                                ssrcvbuf[2] = 0xb4;
                                ssrcvbuf[3] = 0x1c;
                                ssrcvbuf[4] = 0x3f;
                                ssrcvbuf[5] = id;
                                ssrcvbuf[6] = 0x01;
                                ssrcvbuf[7] = 0x00;
                                ssrcvbuf[8] = 0x24;


                                ssrcvbuf[47] = 0xdc;
                                ssrcvbuf[48] = 0x32;

                                if (times == num) {

                                ///        for (n = 0; n < 12; n ++)
                                  //              ssrcvbuf[n] = 0xff;
                                   //     ssrcvbuf[8] = 0x00;
                                        if (rf_flag == 0)
                                                OSQPost (SendQ, ssrcvbuf);
                                        if (rf_flag == 1)
                                                OSQPost (Send_rfQ, ssrcvbuf);
                                        continue;
                                }

                                if (rf_flag == 0){
                                        OSSchedLock();
                                        for (n = 0; n < 77; n ++){
                                                scc3_serial_putc (ssrcvbuf[n]);
                                                //printf ("star[%d] :  %02x\n",n, ssrcvbuf[n]);
                                        }
                                        OSSchedUnlock();
                                }
                                if (rf_flag == 1){
                                        OSSchedLock();
                                        for (n = 0; n < 77; n ++) {
                                                smc1_serial_putc (ssrcvbuf[n]);
                                                //printf ("star[%d] :  %02x\n",n, ssrcvbuf[n]);
                                        }
                                        OSSchedUnlock();

                                }

                        }
                        continue;
#endif

                }

                //for (n = 0 ; n < egse_len; n ++)
                 //      printf ("starsenor         ########       %x\n", ssrcvbuf[n + 9]);

                ssrcvbuf[0] = 0xeb;
                ssrcvbuf[1] = 0x90;
                ssrcvbuf[2] = 0xb4;
                ssrcvbuf[3] = 0x1c;
                ssrcvbuf[4] = 0x3f;
                ssrcvbuf[5] = id;
                ssrcvbuf[6] = 0x01;
                ssrcvbuf[7] = 0x00;
                if (sub_id == 0x90){
                        ssrcvbuf[8] = 0x05;
                        ssrcvbuf[16] = 0xdc;
                        ssrcvbuf[17] = 0x32;
                }
                if (sub_id == 0x50){
                        ssrcvbuf[8] = 0x24;
                        ssrcvbuf[47] = 0xdc;
                        ssrcvbuf[48] = 0x32;
                }


                if (rf_flag == 0)
                        OSQPost (SendQ, ssrcvbuf);
                if (rf_flag == 1)
                        OSQPost (Send_rfQ, ssrcvbuf);

        }

}
