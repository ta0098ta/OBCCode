#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>

extern OS_EVENT* sendMsg;
extern OS_EVENT* Hdlc_Flag; 
extern OS_EVENT* RecMsg; 
extern OS_EVENT* HdlcQEvent; 
extern OS_EVENT* sendRecMsg; 
extern OS_EVENT* sendQEvent; 
extern OS_EVENT* timeoutQEvent;
extern OS_EVENT *ctSem;
extern OS_EVENT*    pFifo_Rx0;
extern OS_EVENT*    pFifo_Rx1;
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
extern OS_EVENT *gps_getQ;

extern OS_EVENT *LaserEventQ;
extern OS_EVENT *DLL_HDLC_REC;
extern OS_EVENT *RFS_HDLC_REC;

extern  int GpsOnOff;


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
extern CPU_INT32U fcc1_tx_packets, fcc3_tx_packets;
extern CPU_INT32U fcc1_rx_packets, fcc3_rx_packets;
extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;



extern INT8U globle_bd_num_0,globle_bd_num_1;
extern int fcc3_hdlc_send_data(unsigned char *buf, int len);
extern int fcc1_hdlc_send_data(unsigned char *buf, int len);
extern INT8U get_hdlc_channel(void);
extern INT8U HDLC_channel_select;
extern INT8U err_fault[5];
extern void changemux(INT32U a);
extern INT16U gps_week;
extern INT16U gps_second;
extern INT8U GpsReady;

INT8U  dkc_package_confile[249];	
INT8U RScc3Buf[300],WScc3Buf[300];	


INT8U GetGpsData(void)
{

        INT16U j,timedata;

        INT8U i;

        //printf("gps_test_start\n");
        j = 0;

        changemux(2);
        if(GpsOnOff)
        {
                for(i=0;i < 255;i++)
                {	
                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) 
                        {
                                continue;
                        }
                        WScc3Buf[0] = scc3_serial_getc();
                        if (WScc3Buf[0]!=0xEB) continue;	
                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        WScc3Buf[1] = scc3_serial_getc();
                        if (WScc3Buf[1]!=0x90) continue;

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        WScc3Buf[2] = scc3_serial_getc();
                        if (WScc3Buf[2]!=0xF9) continue;

                        timedata = 0;
                        while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) continue;

                        WScc3Buf[3] = scc3_serial_getc();
                        if (WScc3Buf[3] !=0xA8) continue;

                        OSSchedLock();

                        for (j=0; j<60; j++)
                        {
                                timedata = 0;
                                while ((!scc3_serial_tstc())&&(timedata<1000)) timedata++;
                                if (timedata>=1000) continue;
                                while (!scc3_serial_tstc());
                                WScc3Buf[4+j] = scc3_serial_getc();

                        }

                        OSSchedUnlock();	
                        if (WScc3Buf[62]!=0xFD) continue;			

                        if (WScc3Buf[63]!=0xFD) 
                        {continue; }
                        else
                        {
                                return WScc3Buf[59];
                        }
                }
                return i;	
        }
        else
                return 0xFF;

}


void gps_serial_scc3(void)
{
        INT8U SendFrameFlag;			
        INT8U j, c, err, *e, ErrTimes, _isSuccess;

        INT8U gps_type,gps_len;
        INT16U timedata, i;
        INT32U addr,id;
        INT8U down_frame[2];
        INT16U PollCount = 0;
        addr=GPS_DATA;
        OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
        //printf("gps_test_start\n");
        OSSemPost(SHARE_PRINTF);
        for(;;) {	

                e = (INT8U *)OSQPend(gps_getQ,0,&err);

                id = *(e-7);   //计算id
                //printf("GpsOnOff = %d, id = %d\n",GpsOnOff,id);
                switch(id)
                {
                        case 0x01:
                                if(GetGpsData() != 0xFF)
                                {
                                        for(j=0;j<64;j++)
                                        {	
                                                //printf("WScc3Buf[%d] = %x ",j,WScc3Buf[j]); //调试串口输出
                                                *(volatile INT8U *)(addr+j)=WScc3Buf[j];//储存在内存
                                        }
                                        down_frame[0]=0x07;
                                        down_frame[1]=1;
                                        OSQPost(sendMsg,down_frame);
                                }
                                else
                                {
                                        //printf("GPS TimeOut!\n");
                                }
                                break;

                        case 0x02:
                                gps_type = *(e+4);
                                //printf("gps_type is %x\n",gps_type);
                                for(j = 0; j<=6; j++)
                                        //printf("%x ",*(e+j));
                                //printf("\n");

                                changemux(2);
                                switch(gps_type)
                                {
                                        case 0xFF:
                                                //printf("gps_program_command received \n");

                                                for(j = 0; j < 10; j++)
                                                        RScc3Buf[j] = *(e+j);
                                                for(j = 10; j < 249; j++)
                                                        RScc3Buf[j] = 0xAA;
                                                OSSchedLock();
                                                for(j = 0; j < 249; j++)
                                                        scc3_serial_putc(RScc3Buf[j]);
                                                OSSchedUnlock();
                                                switch(RScc3Buf[6])
                                                {
                                                        case 04:
                                                                while(((SendFrameFlag = GetGpsData()) >> 6) != 0x01)
                                                                {
                                                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                                                        OSSchedLock();
                                                                        for(j = 0; j < 249; j++)
                                                                                scc3_serial_putc(RScc3Buf[j]);
                                                                        OSSchedUnlock();
                                                                        if(i++ > 30)
                                                                        {
                                                                                i = 0;
                                                                                //printf("Send First Break\n");
                                                                                break;
                                                                        }
                                                                }
                                                                break;

                                                        case 05:
                                                                while(((SendFrameFlag = GetGpsData()) >> 6) != 0x10)
                                                                {
                                                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                                                        OSSchedLock();
                                                                        for(j = 0; j < 249; j++)
                                                                                scc3_serial_putc(RScc3Buf[j]);
                                                                        OSSchedUnlock();
                                                                        if(i++ > 30)
                                                                        {
                                                                                i = 0;
                                                                                //printf("Send First Break\n");
                                                                                break;
                                                                        }
                                                                }
                                                                break;

                                                        case 06:
                                                                while(((SendFrameFlag = GetGpsData()) >> 6) != 0x11)
                                                                {
                                                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                                                        OSSchedLock();
                                                                        for(j = 0; j < 249; j++)
                                                                                scc3_serial_putc(RScc3Buf[j]);
                                                                        OSSchedUnlock();
                                                                        if(i++ > 30)
                                                                        {
                                                                                i = 0;
                                                                                //printf("Send First Break\n");
                                                                                break;
                                                                        }
                                                                }
                                                                break;
                                                }
                                                //printf("debug #gps 2\n");
                                                break;

                                        case 0x12:
                                                //printf("gps_program_data received \n");
                                                RScc3Buf[248] = 0;
                                                for(j = 0; j < 7; j++)
                                                        RScc3Buf[j] = *(e+j);
                                                gps_len = RScc3Buf[6];
                                                if (gps_len==0)
                                                {  
                                                        for(j = 7; j < 248; j++)
                                                                RScc3Buf[j] = 0xAA;
                                                }
                                                else
                                                {
                                                        for(j = 7; j < gps_len+7; j++)
                                                                RScc3Buf[j] = *(e+j);
                                                        for(j = gps_len+7; j < 248; j++)
                                                                RScc3Buf[j] = 0xAA;
                                                }
                                                for(j = 4; j < 248; j++)
                                                        RScc3Buf[248] += RScc3Buf[j];
                                                OSSchedLock();
                                                for(j = 0; j < 249; j++)
                                                        scc3_serial_putc(RScc3Buf[j]);
                                                OSSchedUnlock();
                                                while(((SendFrameFlag = GetGpsData()) & 0x38) != 0x08)
                                                {
                                                        //printf("SendFrameFlag = %x\n",SendFrameFlag);
                                                        OSSchedLock();
                                                        for(j = 0; j < 249; j++)
                                                                scc3_serial_putc(RScc3Buf[j]);
                                                        OSSchedUnlock();
                                                        if(i++ > 30)
                                                        {
                                                                i = 0;
                                                                //printf("Send First Break\n");
                                                                break;
                                                        }
                                                }

                                                break;

                                        default:
                                                break;
                                }
                                break;
                }

        }
}


