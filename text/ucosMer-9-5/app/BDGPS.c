/*
 *
 */


#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern OS_EVENT* sendMsg;
extern OS_EVENT *BDGPSQ;
extern OS_EVENT *RS422_Q_Rec;

extern OS_EVENT *SHARE_PRINTF;//共享控制
extern OS_EVENT *SHARE_SENDDOWN;
extern OS_EVENT *SHARE_CAN;  //CAN

extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;

/*
 * BDGPS  jie shou zheng que dai ma
 */
INT8U scc3buf[160];

INT8U GetBDGpsData(int FrameCount)
{
        INT8U i,j;
        INT16U timedata;
        if(FrameCount == 0x01)
        {
                changemux(0);
                for(i=0;i < 255;i++)
                {
                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) {timedata++;}
                        if (timedata>=1000) 
                        {
                                continue;
                        }


                        scc3buf[0] = scc3_serial_getc();
                        if (scc3buf[0]!=0x10) continue;	

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        scc3buf[1] = scc3_serial_getc();
                        if (scc3buf[1]!=0x01) continue;

                        OSSchedLock();
                        for(j = 2; j < 46; j++)
                        {
                                timedata = 0;
                                while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                                if (timedata>=1000) continue;
                                //while (!scc3_serial_tstc());
                                scc3buf[j] = scc3_serial_getc();

                        }
                        OSSchedUnlock();

                        if (scc3buf[45]!=0x10) continue;	
                        if (scc3buf[46]!=0x03) 
                        {continue;}
                        else
                        {

                                return scc3buf[3];
                        }
                }
                return i;
        }
        else
        {
                changemux(0);
                for(i=0;i < 255;i++)
                {
                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) {timedata++;}
                        if (timedata>=1000) 
                        {
                                continue;
                        }


                        scc3buf[0] = scc3_serial_getc();
                        if (scc3buf[0]!=0x10) continue;	

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        scc3buf[1] = scc3_serial_getc();
                        if (scc3buf[1]!=0x01) continue;

                        OSSchedLock();
                        for(j = 2; j < 155; j++)
                        {
                                timedata = 0;
                                while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                                if (timedata>=1000) continue;
                                //while (!scc3_serial_tstc());
                                scc3buf[j] = scc3_serial_getc();

                        }
                        OSSchedUnlock();

                        if (scc3buf[45]!=0x10) continue;	
                        if (scc3buf[46]!=0x03) continue;  
                        if (scc3buf[47]!=0x10) continue;	
                        if (scc3buf[48]!=0x02) continue; 
                        if (scc3buf[153]!=0x10) continue;	
                        if (scc3buf[154]!=0x03) 
                        {

                                continue;
                        }
                        else
                        {

                                return i;
                        }
                }
                return i;
        }

}



void BDGPS_Main(void * ppdata)
{
        INT8U *BDGPSmsg, err, ErrTimes, _isSuccess;
        INT16U id,i;
        INT8U CMD[6] = {0x10,0x00,0x00,0x00,0x10,0x03};
        INT8U sum;			
        INT8U	j,c;
        //INT8U scc3buf2[249],buf[249], *trmsg, gps_type, gps_len;
        INT16U timedata,PollCount;
        INT8U SendFrameFlag,down_frame[120],Count;
        INT32U addr;

        //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
        //printf("task BDGPS ready\n");
        //OSSemPost(SHARE_PRINTF);

        sum = 0;
        addr = BDGPS_DATA;
        for(;;)
        {
                BDGPSmsg = (INT8U *)OSQPend(BDGPSQ,0,&err);

                id=*(BDGPSmsg-6)+(*(BDGPSmsg-5))*0x100;   //计算id
                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                //printf("BDGPS id=0x%04x",id);
                //OSSemPost(SHARE_PRINTF);
                PollCount = 0;
                changemux(0);

                switch(id)	
                {
                        case 0x0100:
                                //case 0x01:

                                CMD[2] = 0xAA;
                                CMD[3] = 0xAA;
                                scc3buf[42] = 60;
                                OSSchedLock();
                                changemux(0);
                                for(j = 0; j < 6; j++)
                                {
                                        scc3_serial_putc(CMD[j]);
                                        //		printf("%x \n",CMD[j]);	
                                }

                                OSSchedUnlock();
                                i = 0;
                                while(scc3buf[42]  >= 0x05)
                                {

                                        OSSchedLock();
                                        for(j = 0; j < 6; j++)
                                                scc3_serial_putc(CMD[j]);
                                        OSSchedUnlock();
                                        GetBDGpsData(1);
                                        if(i++ > 30)
                                        {
                                                i = 0;
                                                //printf("BDGPS fu wei Send First Break\n");
                                                break;
                                        }
                                        else
                                        {
                                                //printf("BDGPS fu wei Send OK!\n");
                                        }
                                }

                                break;

                        case 0x0200:
                                //case 0x02:
                                CMD[2] = 0xBB;
                                CMD[3] = 0xBB;

                                OSSchedLock();
                                changemux(0);
                                for(j = 0; j < 6; j++)
                                {
                                        scc3_serial_putc(CMD[j]);
                                        //		printf("%x \n",CMD[j]);	
                                }
                                OSSchedUnlock();
                                i = 0;
                                while((SendFrameFlag = GetBDGpsData(1))  != 0x01)
                                {
                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                        OSSchedLock();
                                        for(j = 0; j < 6; j++)
                                                scc3_serial_putc(CMD[j]);
                                        OSSchedUnlock();
                                        if(i++ > 30)
                                        {
                                                i = 0;
                                                //printf("Dan GPS Send TimeOut\n");
                                                break;
                                        }
                                        else
                                        {
                                                //printf("Dan GPS Send OK!\n");
                                        }
                                }

                                break;

                        case 0x0300:
                                //case 0x03:
                                CMD[2] = 0xCC;
                                CMD[3] = 0xCC;
                                OSSchedLock();
                                changemux(0);
                                for(j = 0; j < 6; j++)
                                {
                                        scc3_serial_putc(CMD[j]);
                                        //	printf("%x \n",CMD[j]);	
                                }
                                OSSchedUnlock();
                                i = 0;
                                while((SendFrameFlag = GetBDGpsData(1))  != 0x02)
                                {
                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                        OSSchedLock();
                                        for(j = 0; j < 6; j++)
                                                scc3_serial_putc(CMD[j]);
                                        OSSchedUnlock();
                                        if(i++ > 30)
                                        {
                                                i = 0;
                                                //printf("Dan BD2 Send TimeOut!\n");
                                                break;
                                        }
                                        else
                                        {
                                                //printf("Dan BD2 Send OK!\n");
                                        }
                                }

                                break;

                        case 0x0400:
                                //case 0x04:
                                CMD[2] = 0xDD;
                                CMD[3] = 0xDD;

                                OSSchedLock();
                                changemux(0);
                                for(j = 0; j < 6; j++)
                                {
                                        scc3_serial_putc(CMD[j]);
                                        //	printf("%x \n",CMD[j]);	
                                }
                                OSSchedUnlock();
                                i = 0;
                                while((SendFrameFlag = GetBDGpsData(1))  != 0x03)
                                {
                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                        OSSchedLock();
                                        for(j = 0; j < 6; j++)
                                                scc3_serial_putc(CMD[j]);
                                        OSSchedUnlock();
                                        if(i++ > 30)
                                        {
                                                i = 0;
                                                //printf("BDGPS zu he Send TimeOut!\n");
                                                break;
                                        }
                                        else
                                        {
                                                //printf("BDGPS zu he Send OK!\n");
                                        }
                                }

                                break;

                        case 0x0500:
                                //case 0x05:

                                CMD[2] = 0xEE;
                                CMD[3] = 0xEE;

                                OSSchedLock();
                                changemux(0);
                                for(j = 0; j < 6; j++)
                                {
                                        scc3_serial_putc(CMD[j]);
                                        //	printf("%x \n",CMD[j]);	
                                }
                                OSSchedUnlock();
                                //printf("Li shu ca chu Send OK!\n");
                                break;

                        case 0x0600:
                                //case 0x06:

                                for(i = 0; i < 3; i++)
                                {
                                        if((SendFrameFlag = GetBDGpsData(2))  != 0xFF)
                                                break;
                                }
                                if(i < 3)
                                {
                                        //printf("BDGPS data is\n"); 

                                        for(j=0;j<155;j++)
                                        {	
                                                //printf("scc3buf[%d] = %x ",j,scc3buf[j]); //调试串口输出
                                                *(volatile INT8U *)(addr+j) = scc3buf[j];
                                        }
                                        //处理获得的gps数据
                                        down_frame[0]=0x08;
                                                down_frame[1]=0x01;
                                                
                                                OSQPost(sendMsg,(void *)&down_frame); 					


                                }
                                else
                                {
                                        //printf("Sum is Wrong!\n"); 
                                        sum = 0;
                                        //ErrTimes--;	
                                }


                                break;
                        default:
                                break;

                }
        }
        return;
}

