#include "includes.h"
#include "Includes.h"
extern OS_EVENT *CAN_Q_Rec_DLL;
extern OS_EVENT *MIMU_HDLC_REC;
extern OS_EVENT *CAN_Q_Rec_MIMU;
extern OS_EVENT *RFS_HDLC_REC;
extern OS_EVENT *CAN_Q_Rec_RF1;
extern OS_EVENT* sendMsg;
extern OS_EVENT* DLL_HDLC_REC;
INT8U err_fault[5]={0xff,0xff,0xff,0xff,0xff};

extern OS_EVENT *SHARE_PRINTF;//鍏变韩鎺у埗
extern OS_EVENT *SHARE_SENDDOWN;
extern OS_EVENT *SHARE_CAN;  //CAN

extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;


void DLL_Main(void * data)
{	
        INT32U  tx_va;
        INT32U  tx_mode;
        INT8U temp[5];
        INT8U err,err2;
        INT8U *m;
        INT8U *hdlcmsg,i;
        INT8U ErrTimes,_isSuccess;
        INT16U id;
                INT8U down_frame[2];
        INT32U addr;
        INT8U Data[20];
        INT8U CANData[9] = {0x01, 0x01, 0x01, 0x02, 0x05, 0xFF, 0x00, 0x00, 0x06};
        for(;;)
        {

                addr=DLL_DATA;
                hdlcmsg = (INT8U *)OSQPend(DLL_HDLC_REC,0,&err);
                id=*(hdlcmsg-6)+(*(hdlcmsg-5))*0x100;   //计算id
                hdlcmsg+=2;

                switch(id)
                {
                        case 0x0100:
                                for (i = 0; i<6 ;i++)
                                        temp[i] = *(hdlcmsg+i);
                                tx_va=0x4;
                                tx_mode=0x1;
                                OSSemPend(SHARE_CAN,0,&ERR_SHARE_CAN);	//2011_10_13
                                can_datagram_send(temp, tx_va, tx_mode);
                                ErrTimes=3;
                                _isSuccess=0;
                                while(ErrTimes>0 && _isSuccess==0)
                                {
                                        m=(INT8U *)OSQPend(CAN_Q_Rec_DLL,2000,&err2);
                                        if(err2==OS_NO_ERR)
                                        {
                                                ErrTimes=3;
                                                _isSuccess=1;
                                        }
                                        else
                                                ErrTimes--;
                                }
                                OSSemPost(SHARE_CAN);	//2011_10_13
                                for(i = 0; i <= 18; i++)
                                        Data[i] = 0;
                                if(_isSuccess==1)
                                {	
                                        for(i = 0; i <= 18; i++)
                                        {

                                                *(volatile INT8U *)(addr)=*m;
                                                Data[i] = *m;
                                                m++;
                                                addr++;
                                        }
                                        down_frame[0]=0x0B;
                                        down_frame[1] = 0x01;
                                        OSQPost(sendMsg,down_frame);
                                }
                                else
                                        break;

                        case 0x0200:
                                tx_va = 1;
                                tx_mode = 1;
                                CANData[5] = 0xFF;
                                CANData[8] = 0x06;
                                can_datagram_send(CANData, tx_va, tx_mode);

                                break;

                        case 0x0300:
                                tx_va = 1;
                                tx_mode = 1;
                                CANData[5] = 0x00;
                                CANData[8] = 0x07;
                                can_datagram_send(CANData, tx_va, tx_mode);
                                break;

                        case 0x0400:
                                down_frame[0]=0x0B;
                                down_frame[1] = 0x02;
                                OSQPost(sendMsg,down_frame);
                                break;
                        default:
                                break;
                }
        }
}

void RF_Main(void * data)
{
        INT8U *hdlcmsg,BCFlag, err , num = 0;
        INT16U id, PhotoLen = 0;
        INT8U BCFrame[20];
        INT8U *BackData, Count;
        INT8U k;
        INT32U timedata, DataCount, i, l,j;
        INT8U scc3buf[550], sum = 0, test;
        INT8U down_frame[5];
        INT32U addr;


        addr = RF_DATA; //  48  tytes
        for(;;)
        {
                DataCount = 0;
                hdlcmsg=(INT8U *)OSQPend(RFS_HDLC_REC,0,&err);
                id=*(hdlcmsg + 3);   //璁＄畻id
                sum = 0;

                switch(id)
                {
                        case 0x01:
                                BCFrame[0] = 0xEB;
                                BCFrame[1] = 0x90;
                                BCFrame[2] = 0XBB;
                                BCFrame[3] = 0x01;
                                BCFrame[4] = 0x00;
                                BCFrame[5] = 0x08;
                                BCFrame[6] = 0x90;
                                BCFrame[7] = 0x1D;


                                for (;;){
                                        if(DataCount++ > 30) {
                                                down_frame[0]=0x04;
                                                down_frame[1]=0x01;
                                                down_frame[2]=0xBB;
                                                down_frame[3]=0xBB;
                                                OSQPost(sendMsg,(void *)&down_frame); 
                                                break;
                                        }
                                        memset(scc3buf, 0, 50);
                                        //printf ("****************huo qu ji ben zhuang tai xin xi start\n");

                                        OSSchedLock();
                                        changemux(3);
                                        for(j = 0; j < 8; j++)
                                                scc3_serial_putc(BCFrame[j]);
                                        OSSchedUnlock();

                                        OSTimeDly(1800); // 一秒钟的时钟 节拍 数 是 907 



                                        /*   以下 3 行 代码，scc3_serial_tstc() 永远 为 0 ，不会跳出死循环 
                                             while (!scc3_serial_tstc()){
                                             //printf ("_serial_tstc : %d\n",scc3_serial_tstc());
                                             }
                                             scc3buf[0] = scc3_serial_getc();
                                             if (scc3buf[0]!=0xEB) continue;	
                                             */



                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(100000*5))) timedata++;
                                        if (timedata>=(100000*5)) continue;

                                        scc3buf[0] = scc3_serial_getc();
                                        if (scc3buf[0]!=0xEB) continue;	

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[1] = scc3_serial_getc();
                                        if (scc3buf[1]!=0x90) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[2] = scc3_serial_getc();
                                        if (scc3buf[2]!=BCFlag) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[3] = scc3_serial_getc();
                                        if (scc3buf[3] !=0x01) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[4] = scc3_serial_getc();
                                        if (scc3buf[4] !=0x00) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[5] = scc3_serial_getc();
                                        if (scc3buf[5] !=0x17) continue;

                                        for (j=0; j<17; j++)
                                        {
                                                timedata = 0;
                                                while ((!scc3_serial_tstc())&&(timedata<(10000*6))) 
                                                        timedata++;
                                                if (timedata>=(10000*6)) continue;
                                                while (!scc3_serial_tstc());
                                                scc3buf[6+j] = scc3_serial_getc();

                                        }

                                        if (scc3buf[21]!=0x90)
                                                continue;		

                                        if (scc3buf[22]!=0x1D) 
                                                continue; 
                                        else{
                                                //printf ("****************huo qu ji ben zhuang tai xin xi end\n");
                                                for(i = 0;i < 20; i++) {
                                                        sum += scc3buf[i]; 
                                                }

                                                if (sum == scc3buf[20]) {
                                                        down_frame[0]=0x04;
                                                        down_frame[1]=0x01;
                                                        down_frame[2]=0xAA;
                                                        down_frame[3]=0xAA;

                                                        for(i = 0; i < 23; i++){
                                                                *(volatile INT8U *)(RF_DATA+i) = scc3buf[i];
                                                                //printf("BB CC zhuangtai ::%02x \n",scc3buf[i]);
                                                        }
                                                        sum = 0;

                                                        OSQPost(sendMsg,(void *)&down_frame); 
                                                        break;
                                                }
                                                else{ 
                                                        sum = 0;
                                                        continue;
                                                }
                                        } // inter if  else  finished  


                                } // end  of  for
                                break;

                                //case 0x0200:
                        case 0x02:

                                BCFrame[0] = 0xEB;
                                BCFrame[1] = 0x90;
                                BCFrame[2] = 0XBB;
                                BCFrame[3] = 0x02;
                                BCFrame[4] = 0x00;
                                BCFrame[5] = 0x08;
                                BCFrame[6] = 0x90;
                                BCFrame[7] = 0x1D;


                                for (;;){

                                        if( ++DataCount > 30){
                                                down_frame[0]=0x04;
                                                down_frame[1]=0x02;
                                                down_frame[2]=0xBB;
                                                down_frame[3]=0xBB;
                                                OSQPost(sendMsg,(void *)&down_frame); 
                                                break;
                                        }
                                        //printf ("************************ pai zhao  start\n");
                                        memset(scc3buf, 0, 50);


                                        OSSchedLock();
                                        changemux(3);
                                        for(j = 0; j < 8; j++)
                                                scc3_serial_putc(BCFrame[j]);
                                        OSSchedUnlock();

                                        OSTimeDly(3500);

                                        /*
                                           while (!scc3_serial_tstc());
                                           scc3buf[0] = scc3_serial_getc();
                                           if (scc3buf[0]!=0xEB) continue;	
                                           */

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(100000*5))) timedata++;
                                        if (timedata>=(100000*5)) continue;

                                        scc3buf[0] = scc3_serial_getc();
                                        if (scc3buf[0]!=0xEB) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[1] = scc3_serial_getc();
                                        if (scc3buf[1]!=0x90) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[2] = scc3_serial_getc();
                                        if (scc3buf[2]!=0xBB) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[3] = scc3_serial_getc();
                                        if (scc3buf[3] !=0x02) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[4] = scc3_serial_getc();
                                        if (scc3buf[4] !=0x00) continue;

                                        timedata = 0;
                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) timedata++;
                                        if (timedata>=(10000*6)) continue;

                                        scc3buf[5] = scc3_serial_getc();
                                        if (scc3buf[5] !=0x0B) continue;


                                        for (j=0; j<5; j++)
                                        {
                                                timedata = 0;
                                                while ((!scc3_serial_tstc())&&(timedata<(10000*6))) 
                                                        timedata++;
                                                if (timedata>=(10000*6)) continue;
                                                while (!scc3_serial_tstc());
                                                scc3buf[6+j] = scc3_serial_getc();

                                        }

                                        if (scc3buf[9]!=0x90)
                                                continue;

                                        if (scc3buf[10]!=0x1D) 
                                                continue;
                                        else{
                                                //printf ("************************ pai zhao  end\n");
                                                for(i = 0;i < 8; i++) 
                                                        sum += scc3buf[i]; 

                                                if (sum == scc3buf[8]) {
                                                        down_frame[0]=0x04;
                                                        down_frame[1]=0x02;
                                                        down_frame[2]=0xAA;
                                                        down_frame[3]=0xAA;

                                                        for(i = 0; i < 11; i++){
                                                                //printf("BB CC paizhao :: %02x\n",scc3buf[i]);
                                                                *(volatile INT8U *)(RF_DATA+i) = scc3buf[i];
                                                        }
                                                        PhotoLen = 0;
                                                        PhotoLen = scc3buf[6];
                                                        PhotoLen = PhotoLen << 8;                
                                                        PhotoLen +=  scc3buf[7];
                                                        //printf ("PhotoLen  : %02x  \n", PhotoLen);
                                                        OSQPost(sendMsg,(void *)&down_frame); 
                                                        sum = 0;
                                                        break;

                                                }
                                                else {

                                                        sum = 0;
                                                        continue;
                                                }
                                        } // for  inter  if  else  finished 

                                }// end of for 
                                break;

                        case 0x03: // take  picture  to  OBC 

                                BCFrame[0] = 0xEB;
                                BCFrame[1] = 0x90;
                                BCFrame[2] = 0xbb;
                                BCFrame[3] = 0x03;
                                BCFrame[4] = 0x00;
                                BCFrame[5] = 0x0A;
                                BCFrame[8] = 0x90;
                                BCFrame[9] = 0x1D;

                                OSTimeDly(5000);

                                for(Count = 0; Count < PhotoLen; Count++)
                                {
                                        if (Count < 0) 
                                                break;
                                        BCFrame[6] = Count >> 8;
                                        BCFrame[7] = Count;

                                        DataCount = 0; 
                                        for (;;) {
                                                if( ++DataCount > 30) {
                                                        --Count;
                                                        break;
                                                }
                                                memset(scc3buf, 0, 550);
                                                //printf ("******************  dan zu shuju  chuan tu xiang dao OBC start\n");
                                                //printf ("PhotoLen   :  %d \n", PhotoLen);
                                               // for(j = 0; j < 10; j++)
                                                        //printf ("%02x ,  ",BCFrame[j]);
                                                //printf ("\n");


                                                OSSchedLock();
                                                changemux(3);
                                                for(j = 0; j < 10; j++)
                                                        scc3_serial_putc(BCFrame[j]);
                                                OSSchedUnlock();

                                                /*-1--------------------------------------------------------------------*/

                                                for (i = 0 ; i < 550; i++) {
                                                        timedata = 0;
                                                        while ((!scc3_serial_tstc())&&(timedata<(10000*6))) 
                                                                timedata++;
                                                        if (timedata>=(10000*6)) continue;
                                                        while (!scc3_serial_tstc());
                                                        scc3buf[i] = scc3_serial_getc();
                                                }
                                                /*-1--------------------------------------------------------------------*/

                                                //printf ("----------++++++++++++++++++++++------------\n");

                                                for (i = 0; i < 20; i++) {
                                                        //printf ("%02x\n",scc3buf[i]);
                                                        if (scc3buf[i] != 0)
                                                                break;
                                                }

                                                for (j = 0; j < 521; j ++) 
                                                        scc3buf[j] = scc3buf[i++];


                                                if (scc3buf[0]!=0xEB) continue;
                                                if (scc3buf[1]!=0x90) continue;
                                                if (scc3buf[2]!=0xBB) continue;
                                                if (scc3buf[3]!=0x03) continue;
                                                if (scc3buf[4]!=0x02) continue;
                                                if (scc3buf[5]!=0x09) continue;
                                                if (scc3buf[519] !=0x90) continue;
                                                if (scc3buf[520] !=0x1D) continue;


                                                /*-1--------------------------------------------------------------------*/


                                                for(i = 0; i < 518; i++)
                                                        sum += scc3buf[i];

                                                if(sum == scc3buf[518])
                                                {
                                                        sum = 0;
                                                        //printf ("cheng gong jieshou   shuju \n");
                                                        for(i = 6; i < 518; i++)
                                                                *(volatile INT8U *)(RF_DATA + 512*Count + i - 6) = scc3buf[i];
                                                        break;
                                                }
                                                else {
                                                        sum = 0;
                                                        continue; 
                                                }
                                        }//inter  most  for  finished 
                                        //printf ("******************dan zu  chuan tu xiang dao OBC end\n");

                                }//second  for  finish  ,,,picture  all  be  finished
                                //printf ("tu xiang  shuju   quanbu  obc\n");

                                if (Count == PhotoLen) 
                                {
                                        down_frame[0]=0x04;
                                        down_frame[1]=0x03;
                                        down_frame[2]=0xAA;
                                        down_frame[3]=0xAA;
                                        OSQPost(sendMsg,(void *)&down_frame); 
                                        //printf ("picture  all be  tranmisted\n");
                                        break;
                                }
                                else {
                                        down_frame[0]=0x04;
                                        down_frame[1]=0x03;
                                        down_frame[2]=0xBB;
                                        down_frame[3]=0xBB;
                                        OSQPost(sendMsg,(void *)&down_frame); 
                                        break;
                                }


                        case 0x04: //  transerved  OBC RF_picture   to  EGSG

                                down_frame[0]=0x04;
                                down_frame[1]=0x04;
                                /*5555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555*/
                                down_frame[2]=(PhotoLen & 0xFF00) >> 8;
                                down_frame[3]=(PhotoLen & 0xFF);
                                /*5555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555*/
                                //printf ("-------------------------------------------------------------------------------\n");
                                down_frame[4]=0x00;
                                OSQPost(sendMsg,(void *)&down_frame); 
                                break;

                        default:
                                break;
                }// switch  finished 
        } // the bigger  for  finished 
} 

