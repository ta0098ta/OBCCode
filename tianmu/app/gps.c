#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>

extern OS_EVENT * GPSQ;
extern OS_EVENT * SendQ;
extern OS_EVENT * Send_rfQ;
extern INT8U  rf_flag;



void gps_serial_smc2(void)
{
        INT8U err , *gpsmsg, gpsrcvbuf[150], gpsbuf[200];
        INT8U len, id, DataCount, gpsflag,i;
        INT32U timedata, j , n, num, times;

        for(;;) {	

                printf ("_____________gps  task\n");
                gpsmsg = (INT8U *)OSQPend(GPSQ,0,&err);
                printf ("_____________gps  start \n");
                num = *(gpsmsg + 8);
                num = num << 8;
                num += *(gpsmsg + 9);
                len = *(gpsmsg + 10);
                id = *(gpsmsg + 7);

                for (n = 0; n < len; n ++)
                        gpsbuf[n] = *(gpsmsg + 11 + n);

                for (n = 0 ; n < len; n ++)
                        printf ("gpsbuf[%d] = %02x\n", n, gpsbuf[n]);
                times = 0;

                for (;;) {

                        times++;
                        if (times > num)
                                break;

                        printf ("times = %d\n", times);
                        OSSchedLock();
                        for (n = 0; n < len; n ++)
                                smc2_serial_putc (gpsbuf[n]);
                        OSSchedUnlock();


                        memset(gpsrcvbuf,0 , 150);


                        for (j = 0; j < 50; j ++) {
                                printf ("j = %d", j);
                                timedata = 0;
                                while ((!smc2_serial_tstc())&&(timedata<(10000))) 
                                        timedata++;
                                if (timedata>=(10000)) continue;
                                while (!smc2_serial_tstc());
                                gpsrcvbuf[9] = smc2_serial_getc();

                                if (gpsrcvbuf[9] != 0xeb)
                                        continue;

                                timedata = 0;
                                while ((!smc2_serial_tstc())&&(timedata<(10000))) 
                                        timedata++;
                                if (timedata>=(10000)) continue;
                                while (!smc2_serial_tstc());
                                gpsrcvbuf[10] = smc2_serial_getc();

                                if (gpsrcvbuf[10] == 0x90)
                                        break;
                                else
                                        continue;
                        }

                        for (j=0; j<62; j++)
                        {
                                timedata = 0;
                                while ((!smc2_serial_tstc())&&(timedata<(1000))) 
                                        timedata++;
                                if (timedata>=(1000)) continue;
                                while (!smc2_serial_tstc());
                                gpsrcvbuf[j+11] = smc2_serial_getc();

                        }


                        gpsrcvbuf[0] = 0xeb;
                        gpsrcvbuf[1] = 0x90;
                        gpsrcvbuf[2] = 0xb4;
                        gpsrcvbuf[3] = 0x1c;
                        gpsrcvbuf[4] = 0x3f;
                        gpsrcvbuf[5] = id;
                        gpsrcvbuf[6] = 0x01;
                        gpsrcvbuf[7] = 0x00;
                        gpsrcvbuf[8] = 0x40;


                        gpsrcvbuf[73] = 0x00;
                        gpsrcvbuf[74] = 0x00;
                        gpsrcvbuf[75] = 0xdc;
                        gpsrcvbuf[76] = 0x32;

                        if (times == num) {
                                //for (n = 0; n < 12; n ++) 
                                 //       gpsrcvbuf[n] = 0xff;
                                //gpsrcvbuf[8] = 0x00;
                                if (rf_flag == 0)
                                        OSQPost (SendQ, gpsrcvbuf);
                                if (rf_flag == 1)
                                        OSQPost (Send_rfQ, gpsrcvbuf);
                                continue;
                        }
                        if (rf_flag == 0){
                                OSSchedLock();
                                for (n = 0; n < 100; n ++){
                                        scc3_serial_putc (gpsrcvbuf[n]);
                                        printf ("gps[%d] : %02x\n",n, gpsrcvbuf[n]);
                                }
                                OSSchedUnlock();
                        }
                        if (rf_flag == 1){
                                OSSchedLock();
                                for (n = 0; n < 100; n ++){
                                        smc1_serial_putc (gpsrcvbuf[n]);
                                        printf ("gps[%d] : %02x\n",n, gpsrcvbuf[n]);
                                }
                                OSSchedUnlock();

                        }
                }
        }
}











