/*******************************************************

  hdlc_handle.c

  tasks and functions, which are related to HDLC channel, are difined here

  coded by Wang Xiaochu

  Altered by Hou Jiaming

 *********************************************************/

#include "includes.h"
#include "Includes.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*--------------------------全局start-----------------------*/

/*  ADCS  */
INT8U adcsbuf1[1300];
INT8U adcsbuf2[200];
int acsflcnts;
int acsgdcnts;
int ACSFL_Flag;
int ACSGD_Flag;


/*  ACS  and  transveiver */
extern OS_EVENT * CAN_Q_Rec;
INT32U acsaddr = ADCS_cfg_file;
INT8U acs_down_buf[4] = {};
#define ACS_0	0x09

/*  wod  */
INT8U STATE = 0;
/*-------------------------全局 end-------------------------------------*/


extern CPU_INT32U fcc1_tx_packets, fcc3_tx_packets;
extern CPU_INT32U fcc1_rx_packets, fcc3_rx_packets;
extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
extern INT8U ERR_SHARE_CAN;

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
extern OS_EVENT *MEMSGyroQ;
extern OS_EVENT *BDGPSQ;
extern OS_EVENT *gps_getQ;
extern OS_EVENT* PC_UQ;

extern OS_EVENT *LaserEventQ;
extern OS_EVENT *DLL_HDLC_REC;
extern OS_EVENT *RFS_HDLC_REC;
extern OS_EVENT *TaskManageQueue;
extern OS_EVENT *ACSQCOM;
extern OS_EVENT *ADCS_FLAG;
extern OS_EVENT *PC_Q;
extern INT32U wod_cycle; 
extern INT8U LockFirst; 
extern INT8U StarFlag;
#ifndef __utype_defined
#define __utype_defined
typedef INT8U	u_char;
typedef INT16U  ushort;
typedef INT32U  uint;
typedef INT32U  ulong;
#endif

#define EIS_0	0x06
#define EIS_1	0x08
#define MEMSGyro_0 0x06
#define BDGPS_0 	0x08
#define GPS_0	0x07
#define MIMU_0	0x0F
#define DLL_0	0x0B
#define RF_0	0x04
#define WOD_0	0x0C
#define STAR_0	0x05
#define CJ_0	0x09
#define ProgramCom_0 0x0E

#define SEND_TIMEOUT 0xDD
#define SEND_STOP 0xEE

INT8U  dkc_package_confile[249];
INT32U wod_begin;
INT32U BreakWodBegin;
INT16U BreakPointPack;


void del_ProgramComNode( void);
void add_ProgramComNode(INT8U *msg );
void PCNodeArrange(INT8U head	, INT8U len);

typedef struct BufferDescriptor
{
        ushort bd_cstatus;     /* control and status */
        ushort bd_length;      /* transfer length */
        unsigned char  *buf_addr;        /* buffer address */

} BD;

typedef struct ProgramComNode //程控指令节点
{
        INT8U status;     /* control and status. 0 means idle,1 means to be solved*/
        INT8U type;	/*command type */
        INT16U week;      /* week */
        INT32U second;        /* second */
        INT8U command[30];
} PCNode;

typedef struct TIMEOUT_struct //发送超时判断
{
        INT8U TIMEOUT_CHECK;//检查标志
        INT8U TIMEOUT_RETRY;//重发次数
        INT8U recnum;   //  接收序号
        INT8U sendnum;  // 发送序号
        INT8U second;
} TIMEOUT_send;

struct frame_node *timer[12];//设置个计时器循环使用，全局变量

extern BD *bd_0[12], *bd_1[12];
extern INT32U NoLockTime;
extern INT8U globle_bd_num_0,globle_bd_num_1;
extern int fcc3_hdlc_send_data(unsigned char *buf, int len);
extern int fcc1_hdlc_send_data(unsigned char *buf, int len);
extern INT8U get_hdlc_channel(void);
extern INT8U HDLC_channel_select;
extern INT8U err_fault[5];
extern void changemux(INT32U a);
extern INT16U gps_week;
extern INT32U gps_second;
PCNode PCNodeArray[50];
INT8U PCNodePointer;
INT8U PCNodeEnd;

TIMEOUT_send RR_TIMEOUT;
INT8U EIS_data_down1[200];
INT8U EIS_data_down2[200];
INT16U stop_id=0xffF;
INT32U MAX_TICK=0xffffffff;

///////////////半物理仿真///////////////////////
float q0,q1,q2,q3;
///////////////半物理仿真///////////////////////
float toFloat_2(INT8U a0,INT8U a1,INT8U a2,INT8U a3)
{
        float temp;
        INT32U val;
        val = (a0 & 0x7F)+0x100*a1+0x10000*a2+0x1000000*a3;
        if( (a0&0x80)==0)
                temp=val/2147483647.0;
        else
                temp=-1.0*val/2147483647.0;
        return temp;
}
///////////////半物理仿真///////////////////////

struct frame_node *head_node;
struct frame_node *hdlc_start_timer(struct frame_node *s,INT8U id);
struct frame_node *hdlc_stop_timer(struct frame_node *s,INT8U id);
void send_down_frame(INT8U id,INT8U recnum,INT8U sendnum,FRAME *data);
void HJMSend_down_frame(INT8U id,INT8U recnum,INT8U sendnum, void *data);

#if 0
void dispatch(void *jdata)
{
        INT8U data[MAX_REC_LEN*8],ctr_field,temp,c[1],j,err;
        INT16U k,len;
        INT8U *addr, scc3buf[20];
        enum myevent event_type;
        INT8U i ;
        const char buf0[10] = {'a', 'a','a','a','a','a','a','a','a','a'};
        const char * buf1 = "bbbbbbbbbb";
        INT32U  timedata;

        j=0;
        event_type=frame_arrival;
        c[0]=event_type;
        printf("dispatch!\n");

        for(;;)
        {
                printf ("dipatch  \n");
                printf ("|||||||||||||||||||||||||||||||||||||||||||||||||\n");
                /*
                   if(HDLC_channel_select == 0)
                   {
                //OSSemPend(Hdlc_Flag,0,&err);
                bd_0[0]=(BD *)OSQPend(pFifo_Rx0,0,&err);
                }
                else
                {
                //OSSemPend(Hdlc_Flag,0,&err);
                bd_0[0]=(BD *)OSQPend(pFifo_Rx1,0,&err);
                }

                addr=bd_0[0]->buf_addr;
                len=bd_0[0]->bd_length;

                if(j==8)
                {
                j=0;
                OSQFlush(RecMsg);
                OSQFlush(HdlcQEvent);
                OSQFlush(sendRecMsg);
                OSQFlush(sendQEvent);
                }
                */

                //OSSchedLock();
                //OSSchedUnlock();



                /*
                   while (!scc3_serial_tstc());
                   scc3buf[0] = scc3_serial_getc();
                   if (scc3buf[0]!=0xEB) continue;	
                   */

                scc3_serial_init ();

                for (j = 0 ; j < 10 ; j++)
                        scc3_serial_putc (buf0[j]);

                scc3_serial_puts (buf1);

                scc3_serial_puts ("c c c c c c c c c c c");





                for (j=0; j<10; j++)
                        scc3buf[j] = scc3_serial_getc();


                printf ("***************************\n");
                for (j = 0; j < 10; j ++)
                        printf ("scc3buf [%d] = %x \n", j , scc3buf[j]);





#if 0

        for(k=0;k<10;k++)                   /*取出一桢*/
        {

                        //data[MAX_REC_LEN*j+k]=(unsigned short)*addr;
                        data[k]= serial_getc();
                }
                if (data[0] == 0xEB){
                        for (len = 0; len < data[8]; len ++) {
                                data[len + 9]= serial_getc();
                        }

                        for (k = 0; k < (len+9);k++ ) {
                                printf (" data [k] = %x  /n",k, data[k]);
                        }
                }
                else
                        continue;


#endif










#if 0

                NoLockTime = 0;//

                ctr_field=data[MAX_REC_LEN*j+1];        /*分发消息*/

                //printf("Recive Data:");
                //  for(i = 0; i < 20; i++)
                //{
                //     printf("%x ", data[MAX_REC_LEN*j+i]);
                //}
                //printf("\n");

                if((ctr_field&0x01)==0)                    /*I frame*/
                {

                        OSQPost(RecMsg,&data[MAX_REC_LEN*j]);
                        OSQPost(HdlcQEvent,(void *)&c[0]);
                }
                else if((ctr_field&0x03)==0x01)          /*S frame*/
                {
                        OSQPost(sendRecMsg,&data[MAX_REC_LEN*j]);
                        OSQPost(sendQEvent,(void *)&c[0]);
                        //printf("doushibeinihaide        \n");
                }
                else if((ctr_field&0x03)==0x03)          /*U frame*/
                {
                        temp=ctr_field&0xef;
                        if((temp==0x2f)||(temp==0x43))         /*SABM or DISC*/
                        {
                                OSQPost(RecMsg,&data[MAX_REC_LEN*j]);
                                OSQPost(HdlcQEvent,(void *)&c[0]);
                        }
                        else if((temp==0x0f)||(temp==0x63))    /*UA or DM*/
                        {
                                OSQPost(sendRecMsg,&data[MAX_REC_LEN*j]);
                                OSQPost(sendQEvent,(void *)&c[0]);
                        }
                }

                j+=1;                                        /*调整指针*/
#endif

        }
}

#endif
void get_HDLC_rec_channel(void *jdata)
{

        for(;;)
        {
                //printf("get_HDLC_rec_channel \n");
                if (fcc1_rx_packets>=fcc3_rx_packets)
                {
                        HDLC_channel_select = 0;
                        if ((fcc1_rx_packets-fcc3_rx_packets)>=10)
                        {
                                hdlc_fcc3_reset();
                                fcc1_rx_packets = 0;
                                fcc3_rx_packets = 0;
                        }
                        if ((fcc1_rx_packets+fcc3_rx_packets)>=1000)
                        {
                                fcc1_rx_packets = 0;
                                fcc3_rx_packets = 0;
                        }

                }
                else
                {
                        HDLC_channel_select = 1;
                        if ((fcc3_rx_packets-fcc1_rx_packets)>=10)
                        {
                                hdlc_fcc1_reset();
                                fcc1_rx_packets = 0;
                                fcc3_rx_packets = 0;
                        }
                        if ((fcc1_rx_packets+fcc3_rx_packets)>=1000)
                        {
                                fcc1_rx_packets = 0;
                                fcc3_rx_packets = 0;
                        }

                }

                OSTimeDly(1000);
        }//for循环结束
}//【get_HDLC_rec_channel(void *jdata)】结束


void HDLC_receive(void *pdata)
{
        INT8U 	*msg,recChar,*p,data[2400],buffer[2400];
        INT8U	vr,rej_vr,ctrField,ns,mainFlag;
        INT8U 	*eventMsg,err;
        INT8U mmm;
        INT8U HWTest[27]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x01,0x02,
                0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x01,0x02,0x03};
        INT8U 	recState;	//是否建立链路的标志,0为未建立，1为已建立, 2为要拆除
        INT16U	totalFrames,curFrame,byteCount,loop_times,j;
        INT32U  k,ii,id,len;	
        can_flow data_flow;
        //INT32U  app_num;

        enum 	myevent event_type;

        pdata=pdata;
        recState=0;		//初始化状态为未建立链路
        vr=0;
        rej_vr=0;
        //app_num=0;

        //printf("HDLC_receive!\n");

        for(;;)
        {
                eventMsg=(INT8U *)OSQPend(HdlcQEvent,0,&err);//无限时等待来自dispatch的消息
                event_type=*eventMsg;
                switch(event_type)
                {
                        case frame_arrival:
                                msg=(INT8U *)OSQPend(RecMsg,0,&err);
                                if( msg!=(INT8U *)0 )
                                {
                                        msg++;
                                        recChar=*msg;   //////读取第二个数，控制位
                                        //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                        //printf("RecChar=%x  \n",recChar);
                                        //OSSemPost(SHARE_PRINTF);
                                        msg++;    ///////指针移到下一个数
                                        switch(recState)
                                        {
                                                case 0:   	//说明还没建立链路
                                                        //--------建立链路的代码块，建立成功后将recState=1 --------//
                                                        if(recChar==0x3f)          //如果是SABM帧
                                                        {
                                                                /*发送一个UA_frame*/
                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("SABM Frame received\n");
                                                                //printf("link established\n");
                                                                //OSSemPost(SHARE_PRINTF);
                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                send_down_frame(ua,0,0,0);

                                                                OSSemPost(SHARE_SENDDOWN);
                                                                vr=0;
                                                                recState=1;
                                                                len=0;
                                                                totalFrames=curFrame=byteCount=0;
                                                                p=data;
                                                        }
                                                        break;
                                                case 1:   //说明已建立链路，则开始收数据
                                                        //-----收数据的代码块，判断是否是拆除链路信号-----//
                                                        //-----    如果收到拆除链路信号，recState=1    -----//

                                                        if(recChar==0x3f) //防止地面没收到UA帧，导致程序不可控
                                                        {
                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                send_down_frame(ua,0,0,0);          /*发送一个UA_frame*/
                                                                OSSemPost(SHARE_SENDDOWN);

                                                                vr=0; //发送一个UA_frame,确保对方没收到UA而重发的SABM得到响应
                                                                len=0;
                                                                totalFrames=curFrame=byteCount=0;
                                                                p=data;
                                                                break;
                                                        }
                                                        ctrField=(recChar & 0x01);
                                                        if (ctrField==0)                //received a I frame
                                                        {
                                                                ns=((recChar&0x0e)>>1);
                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("receive a I frame %x %x %x %x %x\n",*(msg-1),*msg,*(msg+1),*(msg+2),*(msg+3));
                                                                //OSSemPost(SHARE_PRINTF);
                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                send_down_frame(ua,0,0,0);
                                                                OSSemPost(SHARE_SENDDOWN);
                                                                if(ns==vr)  //通知地面接收到对应序号的帧
                                                                {
                                                                        if(vr<7)
                                                                                vr+=1;
                                                                        else
                                                                                vr=0; //保证vr从0到7
                                                                        OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                        send_down_frame(rr,vr,0,0);//回应地面，告诉地面（vr-1）已经收到
                                                                        OSSemPost(SHARE_SENDDOWN);
                                                                        //printf("if rej_vr = %d	rr = %d\n",rej_vr,rr);
                                                                        rej_vr=0;
                                                                }
                                                                else           /////*NS不等于VR的桢抛弃掉*/////
                                                                {
                                                                        if(rej_vr==0)
                                                                        {
                                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                                send_down_frame(rej,vr,0,0);
                                                                                OSSemPost(SHARE_SENDDOWN);
                                                                                //printf("else rej_vr = %d	rr = %d\n",rej_vr,rr);
                                                                                rej_vr=1;
                                                                        }
                                                                        break;
                                                                }

                                                                //--------开始取出信息域进行处理-------//
                                                                //--------   处理 I帧 的代码块   ------//

                                                                for(ii=0;ii<9;ii++)
                                                                {
                                                                        *p++=*(msg+ii);
                                                                }
                                                                mainFlag=(*msg);
                                                                msg++;

                                                                id=*msg+(*(msg+1))*0x100;
                                                                msg+=2;
                                                                totalFrames=*msg+(*(msg+1))*0x100;
                                                                msg+=2;
                                                                curFrame=*msg+(*(msg+1))*0x100;
                                                                msg+=2;
                                                                byteCount=*msg+(*(msg+1))*0x100;
                                                                msg+=2;
                                                                loop_times=byteCount;

                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("I Frame handling mainFlag=%x\n",mainFlag);//主标识字
                                                                //printf("I Frame handling id=%x\n",id);//子标识字
                                                                //printf("I Frame handling totalFrames=%x\n",totalFrames);//总帧数
                                                                //printf("I Frame handling curFrame=%x\n",curFrame);//当前帧号
                                                                //printf("I Frame handling byteCount=%x\n",byteCount);//命令的字节长度
                                                                //OSSemPost(SHARE_PRINTF);

                                                                for(k=0;k<loop_times;k++)
                                                                {
                                                                        *p++=*msg++;//读取有效的命令
                                                                        len++;
                                                                }
                                                                //发送给transeiver_receiveQ消息，传递给transceiver任务处理
                                                                OSQPost(transeiver_receiveQ,&data[0]);
                                                                p=data;
                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("pass to transeiver OK \n");
                                                                //OSSemPost(SHARE_PRINTF);
                                                        }//【 if (ctrField==0)】结束

                                                        else if(recChar==0x53)         /*received a DISC frame*/
                                                        {
                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                send_down_frame(dm,0,0,0);
                                                                OSSemPost(SHARE_SENDDOWN);
                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("received a disc\n");
                                                                recState=2;	////////初始化状态为未建立链路
                                                                //printf("prepare to remove\n");
                                                                //OSSemPost(SHARE_PRINTF);
                                                        }
                                                        break;

                                                case 2:   //说明要拆除链路

                                                        //-------拆除链路的代码块-------//
                                                        //-------    recState=0    -------//
                                                        if(recChar==0x3f)        //收到SABM
                                                        {
                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                send_down_frame(ua,0,0,0);          //发送一个UA_frame
                                                                OSSemPost(SHARE_SENDDOWN);

                                                                OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("receive a sabm\r");
                                                                vr=0;
                                                                rej_vr=0;
                                                                recState=1;
                                                                p=data;
                                                                len=0;
                                                                totalFrames=curFrame=byteCount=0;
                                                                //printf("link established\n");
                                                                //printf("continue to receive I\n");
                                                                OSSemPost(SHARE_PRINTF);
                                                                break;
                                                        }
                                                        if(recChar==0x53)         //received a DISC frame
                                                        {
                                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                                send_down_frame(dm,0,0,0);
                                                                OSSemPost(SHARE_SENDDOWN);
                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("removed\n");
                                                                //OSSemPost(SHARE_PRINTF);
                                                                pdata=pdata;
                                                                recState=0;		//初始化状态为未建立链路
                                                                vr=0;
                                                                rej_vr=0;
                                                        }

                                                        break;

                                                default:

                                                        break;

                                        }//switch
                                }

                                break;//【case frame_arrival】分支结束
                        default:
                                break;

                }
        }
}

void HJMSend_down_frame(INT8U id,INT8U recnum,INT8U sendnum, void * data)
{
        INT8U buf[MAX_SEND_LEN],*p;
        INT16U k;
        p=(INT8U *)data;

        memset(buf,0,MAX_SEND_LEN);/*作用是将已开辟内存空间buf的首MAX_SEND_LEN个字节赋值为0*/

        switch(id)
        {
                case ua:/*加头表示类型*/
                        buf[0]=0x73;
                        break;
                case rr:
                        buf[0]=(recnum<<5)|0x11;
                        break;
                case rej:
                        buf[0]=(recnum<<5)|0x19;
                        break;
                case dm:
                        buf[0]=0x1f;
                        break;
                case sabm:
                        buf[0]=0x3f;
                        break;
                case disc:
                        buf[0]=0x53;
                        break;
                case i_data:
                        buf[0]=(recnum<<5)|(0x10)|(sendnum<<1);
                        for(k=1; k < 175; k++)
                        {
                                buf[k]=*p;
                                p++;
                        }
                        break;

        }
        OSSchedLock();/*停止任务调度，独占CPU，不管有没有其他高优先级的就绪任务。中断仍然可以被允许和执行，但不能再调用可能导致当前任务挂起的系统函数，TIMEDLY.PEND等，会导致系统死锁。必须与OSSchedUnlock配对使用*/
        if((id == ua) || (id == dm) || (id == sabm) || (id == disc) || (id == rr) || (id == rej))
        {

                if (get_hdlc_channel()==1)
                {
                        fcc1_hdlc_send_data(buf, 10);
                }
                else if (get_hdlc_channel()==3)
                {
                        fcc3_hdlc_send_data(buf, 10);
                }
        }
        else
        {
                if (get_hdlc_channel()==1)
                {
                        fcc1_hdlc_send_data(buf, 175);
                }
                else if (get_hdlc_channel()==3)
                {
                        fcc3_hdlc_send_data(buf, 175);
                }
        }


        OSSchedUnlock();
}


void send_down_frame(INT8U id,INT8U recnum,INT8U sendnum,FRAME *data)
{
        INT8U buf[MAX_SEND_LEN],*p;
        INT16U k;
        p=(INT8U *)data;

        memset(buf,0,MAX_SEND_LEN);/*作用是将已开辟内存空间buf的首MAX_SEND_LEN个字节赋值为0*/

        switch(id)
        {
                case ua:/*加头表示类型*/
                        buf[0]=0x73;
                        break;
                case rr:
                        buf[0]=(recnum<<5)|0x11;
                        break;
                case rej:
                        buf[0]=(recnum<<5)|0x19;
                        break;
                case dm:
                        buf[0]=0x1f;
                        break;
                case sabm:
                        buf[0]=0x3f;
                        break;
                case disc:
                        buf[0]=0x53;
                        break;
                case i_data:
                        buf[0]=(recnum<<5)|(0x10)|(sendnum<<1);
                        for(k=1;k<175;k++)
                        {
                                buf[k]=*p;
                                p++;
                        }
                        break;
        }
        OSSchedLock();
        /*停止任务调度，独占CPU，不管有没有其他高优先级的就绪任务。
         *中断仍然可以被允许和执行，但不能再调用可能导致当前任务挂起的系统函数
         *TIMEDLY.PEND等，会导致系统死锁。必须与OSSchedUnlock配对使用
         */
        if((id == ua) || (id == dm) || (id == sabm) || (id == disc) || (id == rr) || (id == rej))
        {
                if (get_hdlc_channel()==1)
                {
                        fcc1_hdlc_send_data(buf, 10);
                }
                else if (get_hdlc_channel()==3)
                {
                        fcc3_hdlc_send_data(buf, 10);
                }
        }
        else
        {
                if (get_hdlc_channel()==1)
                {
                        fcc1_hdlc_send_data(buf, 175);
                }
                else if (get_hdlc_channel()==3)
                {
                        fcc3_hdlc_send_data(buf, 175);
                }
        }


        OSSchedUnlock();
        //printf("SendDown!\n");
}

void send_down_frame_2(INT8U id,INT8U mainflag,INT8U sendnum,FRAME *data)
{
        INT8U buf[MAX_SEND_LEN],*p;
        INT16U k;
        p=(INT8U *)data;

        memset(buf,0,MAX_SEND_LEN);/*作用是将已开辟内存空间buf的首MAX_SEND_LEN个字节赋值为0*/

        switch(id)
        {
                case ua:/*加头表示类型*/
                        buf[0]=0x73;
                        break;

                case dm:
                        buf[0]=0x1f;
                        break;
                case sabm:
                        buf[0]=0x3f;
                        break;
                case disc:
                        buf[0]=0x53;
                        break;
                case i_data:
                        buf[0]=mainflag;
                        //printf("down--------------> %02x \n",buf[0]);

                        for(k=1;k<400;k++)
                        {
                                buf[k]=*p;
                                //printf("%02x ",buf[k]);
                                p++;
                        }
                        //printf("\n\n");
                        break;

        }
        OSSchedLock();
        /*停止任务调度，独占CPU，不管有没有其他高优先级的就绪任务。
         *中断仍然可以被允许和执行，但不能再调用可能导致当前任务挂起的系统函数
         *TIMEDLY.PEND等，会导致系统死锁。必须与OSSchedUnlock配对使用
         */

        if (get_hdlc_channel()==1)
        {
                fcc1_hdlc_send_data(buf, 183);
        }
        else if (get_hdlc_channel()==3)
        {
                fcc3_hdlc_send_data(buf, 183);
        }
        OSSchedUnlock();
}

void timer_isr(void *jdata)//时间中断响应任务
{
        enum myevent event_type;

        INT8U c[8], t[1], err, id, len, i;
        INT16U week;
        INT32U second;
        INT8U Gyro[9] ;

        can_flow data_flow;

        struct frame_node *p;
        static INT8U num=0;

        id=0x00;
        len=29;
        data_flow.id=ID_TTC1;
        data_flow.f=TTC_TC_DATASTREAM;
        data_flow.i=id>>8;
        data_flow.i2=id&0xff;
        data_flow.l=len>>8;
        data_flow.l2=len&0xff;

        for(;;)
        {
                OSSemPend(ctSem,0,&err);//等待时间中断发来的信号量
                //	OSMboxPost(ADCS_FLAG,(void*)1);
                //printf("PC Flag!\n");
                if(err==OS_NO_ERR)
                {
                        //printf("PC NO ERR!\n");
                        week=gps_week;
                        second=gps_second;

                        /*检查程控指令*/
                        if(PCNodeArray[0].week==week)
                        {
                                if(PCNodeArray[0].second == second)
                                {
                                        //printf("gps_week = %x,gps_second = %x,PCNodeArray[0].week = %x,PCNodeArray[0].second = %x\n",gps_week,gps_second,PCNodeArray[0].week,PCNodeArray[0].second);
                                        /* 执行程控指令*/
                                        if(PCNodeArray[0].status == 0)
                                        {
                                                for(i = 0; i < 29; i++)
                                                        data_flow.d[i] = PCNodeArray[0].command[i];
                                                can_datagram_send((INT8U *)&data_flow,3,CANPUT);
                                        }
                                        else if(PCNodeArray[0].status == 1)
                                        {
                                                for(i = 0; i < 9; i++)
                                                        Gyro[i] = PCNodeArray[0].command[i];
                                                can_datagram_send(Gyro,1,1);
                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                //for (i = 0; i < 9; i ++)
                                                //printf ("Gyro[%d] :  %02x\n", i , Gyro[i]);
                                                if (Gyro[5] == 0xff) 
                                                        STATE |= 0x04;
                                                else
                                                        STATE &= 0xfb;
                                                /*000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                        }
                                        else if(PCNodeArray[0].status == 2)
                                        {
                                                LockFirst = 2;
                                        }
                                        del_ProgramComNode( );

                                        //printf("Timer PCNodeEnd = %d\n",PCNodeEnd);
                                }
                        }
                }
        }
}


void Transceiver(void *jdata)
{
        /*  赵文军 ACS */
        /*------------------ACS start------------------------*/

        INT8U * acs_rec;
        INT8U acsbuf[9]= {};
        INT8U sum = 0;

        /*-------------------------ACS end---------------------*/
        int i;
        INT16U timedata;
        INT8U *trmsg,*p,mainFlag,err,*downmsg,_isSuccess,ErrTimes;
        INT16U totalFrames,curFrame,byteCount,loop_times;
        INT32U id,len,k,j;
        INT8U temp_i,temp_time[5],wod_temp_time[10];
        INT8U 	data[2048],gpsFlag[2],scc3buf[249];
        INT32U addr4 =DPRAM_RECEIVE_422_ADDR;
        INT8U command_type, gps_type, gps_len;
        INT8U down_frame[5];
        INT8U wod_temp[12];
        INT32U wod_ssecond,wod_fsecond,temp_second;
        INT32U addr,re_addr,con_gps_addr,temp_addr;
        INT16U PackNum,wod_sweek,wod_fweek,wod_num,temp,wod_flag,temp_week,wod_num_end;
        INT32U wod_addr;
        can_flow data_flow;
        gpsFlag[0] = 0xFE;//gps获取数据,存储于内存,随后下发地面
        gpsFlag[1] = 0x01;//gps获取数据,存储于内存
        memset(scc3buf, 0, 249);
        //memset(gps_type, 0, 2);
        gps_type=0;
        //printf("transeiver task ready\n");

        for(;;)
        {

                trmsg=(INT8U *)OSQPend(transeiver_receiveQ,0,&err);

                mainFlag=(*trmsg)&0x0f;//主标识字
                trmsg++;
                id=*trmsg+(*(trmsg+1))*0x100;//子标识字
                trmsg+=2;
                totalFrames=*trmsg+(*(trmsg+1))*0x100;//总帧数
                trmsg+=2;
                curFrame=*trmsg+(*(trmsg+1))*0x100;//当前帧数
                trmsg+=2;
                byteCount=*trmsg+(*(trmsg+1))*0x100;//字节数
                trmsg+=2;
                //printf("Transceiver");
                /*
                   OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);

                //printf("_at_ Transceiver start mainFlag=%x\n",mainFlag);
                //printf("_at_ Transceiver start id=%x\n",id);
                //printf("_at_ Transceiver start totalFrames=%x\n",totalFrames);
                //printf("_at_ Transceiver start curFrame=%x\n",curFrame);
                //printf("_at_ Transceiver start byteCount=%x\n",byteCount);

                OSSemPost(SHARE_PRINTF);
                */
                switch(mainFlag)
                {
                        /* 赵文军 ACS */
                        /*--------------------ACS 代码 start---------------------*/
                        case ACS:
                                switch (id) {
                                        /*  遥控指令 */
                                        /*  主要是 ADCS 主空板的遥控功能  其主标示字都是09 ，子标示字都是01 ， 都走case 0x01:分支 */
                                        case 0x01:
                                                memset(acsbuf,0,9);
                                                for (i=0; i<8; i++) {
                                                        acsbuf[i] = *(trmsg+i+2);
                                                }
                                                for (i=0; i< 5; i++) {
                                                        sum += acsbuf[3+i];
                                                }
                                                acsbuf[8] = sum;
                                                can_datagram_send(acsbuf,*(trmsg+1),*trmsg);
                                                /*0000000000000000000*/
                                                /*0000000000000000000*/
                                                /*0000000000000000000*/
                                                /*0000000000000000000*/
                                                sum = 0;
                                                //printf ("**********************\n");
                                                //printf ("open  success \n");
                                                //printf ("**********************\n");
                                                /*000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                if (acsbuf[4] == 0x01){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x01;
                                                        else
                                                                STATE &= 0xfe;
                                                }
                                                else if (acsbuf[4] == 0x02){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x02;
                                                        else
                                                                STATE &= 0xfd;
                                                }
                                                else if (acsbuf[4] == 0x03){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x04;
                                                        else
                                                                STATE &= 0xfb;
                                                }
                                                else if (acsbuf[4] == 0x04){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x08;
                                                        else
                                                                STATE &= 0xf7;
                                                }
                                                else if (acsbuf[4] == 0x05){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x10;
                                                        else
                                                                STATE &= 0xef;
                                                }
                                                else if (acsbuf[4] == 0x06){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x20;
                                                        else
                                                                STATE &= 0xdf;
                                                }
                                                else if (acsbuf[4] == 0x07){
                                                        if (acsbuf[5] == 0xff) 
                                                                STATE |= 0x40;
                                                        else
                                                                STATE &= 0xbf;
                                                }
                                                else 
                                                        break;
                                                //printf ("STATE  :  %02x \n", STATE);
                                                /*000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                break;
                                                /*  要测指令 */
                                        case 0x02:
                                                /*  clear 0 */
                                                for (i = 0 ; i < 200; i++)
                                                        *(volatile INT8U *) (acsaddr+i) = 0x00;

                                                /*  start  recive yaoce shuju */
                                                memset(acsbuf,0,9);
                                                //printf ("yao ce mingling : %02x\n",*trmsg);
                                                //printf ("yao ce mingling : %02x\n",*(trmsg + 1));
                                                for (i=0; i<8; i++) {
                                                        acsbuf[i] = *(trmsg+i+2);
                                                        //printf ("yaoce  ming ling :%02x\n",acsbuf[i]);
                                                }
                                                for (i=0; i< 5; i++) {
                                                        sum += acsbuf[3+i];
                                                }
                                                acsbuf[8] = sum;
                                                //printf ("sum = %d ,,,acsbuf[8] :  %d\n", sum, acsbuf[8]);
                                                can_datagram_send(acsbuf,*(trmsg+1),*trmsg);
                                                acs_rec = (INT8U *)OSQPend(CAN_Q_Rec,3000,&err);
                                                /*000000000000000000000000000000000000000000000000000*/
                                                /*000000000000000000000000000000000000000000000000000*/
                                                /*000000000000000000000000000000000000000000000000000*/
                                                /*000000000000000000000000000000000000000000000000000*/
                                                sum = 0;

                                                if (err == OS_NO_ERR) {
                                                        for (i = 0 ; i < 130; i++) {
                                                                *(volatile INT8U *) (acsaddr+i) = *(acs_rec + i );
                                                                //printf ("+++++---- %d : %02x\n",i,*(acs_rec + i));
                                                        }
                                                        acs_down_buf[0] = ACS_0;
                                                        acs_down_buf[1] = 0x02;
                                                        OSQPost(sendMsg,acs_down_buf);
                                                        //printf ("**********************\n");
                                                }
                                                else {}
                                                break;

                                }
                                break;
                                /*---------------ACS 代码 end--------------------------*/
                        case real_time_com:
                                switch(id>>8)
                                {
                                        case 0x03:
                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                //printf("enter case 0x03 \n");
                                                //OSSemPost(SHARE_PRINTF);
                                                id=0x00;
                                                len=29;
                                                data_flow.id=ID_TTC1;
                                                data_flow.f=TTC_TC_DATASTREAM;
                                                data_flow.i=id>>8;
                                                data_flow.i2=id&0xff;
                                                data_flow.l=len>>8;
                                                data_flow.l2=len&0xff;
                                                for(k=0,j=0;j<29;k=k+1,j=j+1)
                                                {
                                                        data_flow.d[j]=*trmsg++;
                                                        //printf("%x ", data_flow.d[j]);
                                                }
                                                //printf("\n");
                                                can_datagram_send((INT8U *)&data_flow,3,CANPUT);
                                                break;

                                        default:

                                                break;
                                }

                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);

                                //printf("real time command->ttc \n");

                                //OSSemPost(SHARE_PRINTF);

                                break;

                        case time_orbit:
                                for(j = 0; j < 5; j++)
                                {
                                        temp_time[j] = * trmsg++;
                                        //printf("temp_time = %x	",temp_time[j]);
                                }
                                //printf("\n");
                                gps_week = temp_time[0]*0x100+temp_time[1];
                                gps_second=temp_time[2]*0x10000+temp_time[3]*0x100+temp_time[4];
                                //printf("gps_week = %d	gps_second = %d\n",gps_week, gps_second);
                                break;

                        case MEMSGyro://给Gyro的命令，Gyro分支开始

                                OSQPost( MEMSGyroQ , (void *)(trmsg-2) );
                                //OSQPost( MEMSGyroQ , (void *)(trmsg-2) );?????why  trmsg-2  ?????????

                                break;//GYRO分支结束

                        case BDGPS://发给BDGPS的命令，BDGPS分支开始
                                OSQPost(BDGPSQ , (void *)(trmsg-2) );//转给BDGPS任务处理
                                break;

                        case star_req://发给星敏的命令
                                if(id == 0x0500)
                                {
                                        PackNum = (*trmsg)*0x100 + *(trmsg+1);
                                        //printf("Star PackNum = %x\n", PackNum);

                                        down_frame[0]=0x05;
                                        down_frame[1]=0x03; //wod_num-PackNum;
                                        down_frame[2]=(*trmsg);
                                        down_frame[3] = *(trmsg+1);
                                        down_frame[4] = 0x01;

                                        OSQPost(sendMsg,(void *)&down_frame); 


                                }
                                else if(id == 0x0800)
                                {
                                        PackNum = (*trmsg)*0x100 + *(trmsg+1);
                                        //printf("Star Small PackNum = %x\n", PackNum);

                                        down_frame[0]=0x05;
                                        down_frame[1]=0x03; //wod_num-PackNum;
                                        down_frame[2]=(*trmsg);
                                        down_frame[3] = *(trmsg+1);
                                        down_frame[4] = 0x02;

                                        OSQPost(sendMsg,(void *)&down_frame); 
                                }
                                else if(id == 0x0900)
                                {
                                        down_frame[0]=0x05;
                                        down_frame[1]=0x09; //
                                        down_frame[2]=StarFlag;
                                        OSQPost(sendMsg,(void *)&down_frame); 
                                        //printf("Star Small Send!\n");
                                        StarFlag = 0;
                                }
                                else if(OS_NO_ERR == OSQPost(SS_OSQ , (void *)(trmsg-2) ))
                                {
                                        //OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                        //printf("pass to star OK");
                                        //OSSemPost(SHARE_SENDDOWN);
                                }


                                break;//【发给星敏的命令】分支结束

                        case gps_data://发给gps请求数据的命令

                                OSQPost(gps_getQ,(void *)(trmsg));
                                //printf("debug #gps 1\n");

                                break;

                        case gps_program://发给gps程序控制的命令


                                break;//【发给gps程序控制的命令】分支结束

                        case rf_star_req:
                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                if(id == 0x05)
                                        OSQPost(sendMsg, (void *)trmsg);
                                else
                                        OSQPost(RFS_HDLC_REC, (void *)trmsg);

                                break;
                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

                        case dll_req: //

                                OSQPost(DLL_HDLC_REC , (void *)(trmsg-2));
                                break;

                        case program_com:
                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                if( id == 0x0300 )
                                        add_ProgramComNode(trmsg);	
                                else if(id == 0x0100)
                                        OSQPost(sendMsg, (void *)trmsg);        
                                else if((id == 0x0400) || (id == 0x0800))
                                	OSQPost(PC_UQ , (void *)trmsg);	
                                else
                                    OSQPost(PC_Q , (void *)trmsg);		
                                break;
                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/



                        case task_manage:
                                OSQPost(TaskManageQueue, (INT8U *)(trmsg) );
                                break;

                        case wod_req:
                                switch(id)
                                {
                                        case 0x0000:

                                                wod_cycle = (*trmsg)*0x100 + *(trmsg+1);
                                                //printf("wod_cycle = %d  ", wod_cycle);
                                                break;

                                        case 0x0100:
                                                wod_sweek = 0;
                                                wod_ssecond = 0;
                                                wod_fweek = 0;
                                                wod_fsecond = 0;

                                                wod_sweek = (*trmsg)*0x100+(*(trmsg+1));/*起始时间*/ 
                                                wod_ssecond=(*(trmsg+2))*0x10000+(*(trmsg+3))*0x100+(*(trmsg+4));
                                                //printf("wod_sweek = %d	wod_ssecond = %x\n",wod_sweek, wod_ssecond);

                                                wod_fweek = (*(trmsg+5))*0x100+(*(trmsg+6));/*结束时间*/  
                                                wod_fsecond=(*(trmsg+7))*0x10000+(*(trmsg+8))*0x100+(*(trmsg+9));
                                                //printf("wod_fweek = %d	wod_fsecond = %x\n",wod_fweek, wod_fsecond);

                                                if((gps_week <= wod_fweek) && (gps_second <= wod_fsecond) )
                                                {
                                                        //printf("Time is wrong!\n");
                                                        break;
                                                }

                                                wod_addr = wod_begin_addr;
                                                //printf("wod_addr = %x wod_end_addr = %x\n",wod_addr,wod_end_addr);
                                                wod_num=0;
                                                wod_flag=0;
                                                temp_second = 0;
                                                while(wod_addr < wod_end_addr)
                                                {
                                                        temp_week = *(volatile INT16U *)wod_addr;
                                                        temp_second = *(volatile INT32U *)(wod_addr+2);

                                                        //printf("temp_week = %d temp_second = %x wod_addr = %x\n",temp_week,temp_second,wod_addr);

                                                        if((wod_sweek <= temp_week) && (wod_ssecond <= temp_second))
                                                        {
                                                                if((wod_ssecond + wod_cycle) >= temp_second)
                                                                {
                                                                        wod_begin=wod_addr;      //记录初始addr
                                                                        BreakWodBegin = wod_addr;
                                                                        //printf("wod_begin = %x\n",wod_begin);
                                                                }
                                                                if((wod_fweek >= temp_week) && (wod_fsecond>= temp_second))
                                                                {
                                                                        //printf("if");
                                                                        wod_num++;
                                                                }
                                                                else
                                                                {
                                                                        //printf("else");
                                                                        break;

                                                                }
                                                        }
                                                        wod_addr+=wod_data_len; 
                                                        //printf("wod_data_len = %x\n",wod_data_len);  
                                                }
                                                /*00000000000000000000000000000000000000000000000000000000000000*/
                                                /*00000000000000000000000000000000000000000000000000000000000000*/
                                                wod_num_end = (wod_num * wod_data_len)%165;
                                                wod_num = (wod_num * wod_data_len) / 165 + 1;
                                                /*0000000000000000000000000000000000000000000000000000000000000*/
                                                /*0000000000000000000000000000000000000000000000000000000000000*/

                                                down_frame[0] = 0x0C;
                                                down_frame[1] = 0x01;
                                                down_frame[2] = wod_num >> 8;
                                                down_frame[3] = wod_num;
                                                down_frame[4] = wod_num_end;
                                                //printf("down_frame[2] = %x,down_frame[3] = %x\n",down_frame[2],down_frame[3]);

                                                OSQPost(sendMsg,(void *)down_frame);                

                                                break;

                                        case 0x0200:

                                                PackNum = (*trmsg)*0x100 + *(trmsg+1);

                                                down_frame[0] = 0x0C;
                                                down_frame[1] = 0x02;
                                                down_frame[2] = *trmsg;
                                                down_frame[3] = *(trmsg+1);
                                                //printf("PackNum = %x	WOD DownFrame = %x %x %x %x\n",PackNum,down_frame[0],down_frame[1],down_frame[2],down_frame[3]);

                                                OSQPost(sendMsg,(void *)down_frame); 
                                                break;

                                }

                                break;

                        case acs_data_req:
                                //printf("#1\n");

                                switch(id)
                                {
                                        /* 0x37   0x38    赵文军  完成   */
                                        /*---------ADCS 配置文件 及轨道更新start------------------------------------------------------------------------*/

                                        /*  配置文件 的 上传 */
                                        case 0x0037:
                                                if ((*trmsg == 0x0B)&&(*(trmsg+1)==0x01)&&(*(trmsg+2)==0xBB)&&(*(trmsg+3)==0xBB)){
                                                        ACSFL_Flag = 1; acsflcnts = 0;
                                                }
                                                else {
                                                        for (j=0;j<byteCount;j ++){
                                                                adcsbuf1[100*acsflcnts+j]=*(trmsg+j);
                                                                //printf ("adcsbuf1[%d] = %x\n",(100*acsflcnts +j),*(trmsg+j));
                                                        }
                                                        acsflcnts++;
                                                }
                                                break;

                                                /*  轨道参数 的 上传 */
                                        case 0x0038:
                                                if ((*trmsg == 0x0B)&&(*(trmsg+1)==0x01)&&(*(trmsg+2)==0xbb)&&(*(trmsg+3)==0xbb)){
                                                        ACSGD_Flag = 1; acsgdcnts = 0;
                                                }
                                                else {
                                                        for (j=0;j<byteCount;j ++){
                                                                adcsbuf2[100*acsgdcnts+j]=*(trmsg+j);
                                                                //printf ("adcsbuf2[%d] = %x\n",(100*acsgdcnts +j),*(trmsg+j));
                                                        }
                                                        acsgdcnts++;
                                                }
                                                break;

                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                        case 0x03:
                                                if (*trmsg == 0x37) {
                                                        ACSFL_Flag = 0; acsflcnts = 0;
                                                        //printf ("00000000000000000000000000000000000000000\n");
                                                        //printf ("ACSFL_Flag  :  %d\n", ACSFL_Flag);
                                                        //printf ("acsflcnts   :  %d\n", acsflcnts);
                                                        //printf ("00000000000000000000000000000000000000000\n");
                                                }
                                                else if(*trmsg == 0x38) {
                                                        ACSGD_Flag = 0; acsgdcnts = 0;
                                                        //printf ("00000000000000000000000000000000000000000\n");
                                                        //printf ("ACSGD_Flag  :  %d\n", ACSGD_Flag);
                                                        //printf ("acsgdcnts   :  %d\n", acsgdcnts);
                                                        //printf ("00000000000000000000000000000000000000000\n");
                                                }
                                                else 
                                                        break;
                                                break;
                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                /*-----------ADCS 配置文件及轨道 更新end------------*/
                                        case 0x0000:
                                                //printf("%02x ,%02x ,%02x ,%02x ,%02x ,%02x ,%02x\n",*(trmsg+0),*(trmsg+1),*(trmsg+2),*(trmsg+3),*(trmsg+4),*(trmsg+5),*(trmsg+6));
                                                OSQPost(ACSQCOM,(INT8U *)trmsg);
                                                OSMboxPost(ADCS_FLAG, (void *)1);
                                                break;

                                        case 0x0100:
                                                OSQPost(ADCS_HDLC_REC,(void *)trmsg);
#if 0
                                                ErrTimes=3;
                                                _isSuccess=0;
                                                //printf("ADCS\n");
                                                while(ErrTimes>0 && _isSuccess==0)
                                                {
                                                        downmsg=(INT8U *)OSQPend(DOWN_REMOTE_DATA,10000,&err);
                                                        if(err==OS_NO_ERR)
                                                        {
                                                                ErrTimes=0;
                                                                _isSuccess=1;
                                                        }
                                                        else
                                                                ErrTimes--;
                                                }
                                                if(_isSuccess==1)
                                                {
                                                        send_down_frame(i_data,0,0,(FRAME *)downmsg);
                                                        OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                        //printf("down=%x\n",*downmsg);
                                                        OSSemPost(SHARE_PRINTF);
                                                }
                                                else
                                                {
                                                        err_fault[2]=0x02;
                                                        OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                        send_down_frame(i_data,0,0,(FRAME *)err_fault);
                                                        OSSemPost(SHARE_SENDDOWN);
                                                }
#endif
                                                break;//case 01 break;
                                        case 0x0200:
                                                down_frame[0]=0x02;
                                                down_frame[1]=1;
                                                OSQPost(sendMsg,down_frame);

                                                break;
                                        default:
                                                break;
                                }//【switch(id)】结束
                                break;//【acs_data_req】分支结束
                        default:
                                break;
                }
        }
}

void add_ProgramComNode(INT8U *msg )
{
        INT8U i,k;
        INT8U num_t;
        INT8U type_t;
        INT16U week_t;
        INT32U second_t;
        INT32U status_t;

        if(PCNodeEnd>49)
        {
                //printf("Program Commands are too many.\n");
                return;
        }

        status_t = *msg;
        msg++;
        week_t = (*msg)*0x100 + (*(msg+1));
        second_t=(*(msg+2))*0x10000+(*(msg+3))*0x100+(*(msg+4));

        /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
        //printf("PC week_t = %x, second_t = %x\n",week_t, second_t);
        //printf(" gps_week = %x, gps_second = %x\n",gps_week, gps_second);
        //printf ("PC1\n");
        if((second_t <= gps_second)&&(week_t <= gps_week))
                /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
        {
                //printf("Early Time!\n");
                return;
        }

        //printf("PC week_t = %x, second_t = %x\n",week_t, second_t);
        if(PCNodeEnd == 0)
        {
                PCNodeArray[0].status=status_t;
                PCNodeArray[0].week=week_t;
                PCNodeArray[0].second=second_t;
                if(status_t == 0)
                {
                        for( k=0;k<30;k++)
                        {
                                PCNodeArray[0].command[k]=*(msg+5+k);
                        }
                        //printf("status_t = 0\n");
                }
                else
                {

                        for( k=0;k<9;k++)
                        {
                                PCNodeArray[0].command[k]=*(msg+5+k);
                        }
                        //printf("status_t != 0\n");

                }
                PCNodeEnd++;
        }
        else if(week_t < PCNodeArray[0].week)
        {
                PCNodeArrange( 0, PCNodeEnd);
                PCNodeArray[0].status=status_t;
                PCNodeArray[0].week=week_t;
                PCNodeArray[0].second=second_t;
                if(status_t == 0)
                {
                        for( k=0;k<30;k++)
                        {
                                PCNodeArray[0].command[k]=*(msg+5+k);
                        }
                        //printf("Week_t status_t = 0\n");
                }
                else
                {

                        for( k=0;k<9;k++)
                        {
                                PCNodeArray[0].command[k]=*(msg+5+k);
                        }
                        //printf("Week_t status_t != 0\n");
                }
                PCNodeEnd++;
        }
        else if(week_t > PCNodeArray[PCNodeEnd-1].week)
        {
                PCNodeArray[PCNodeEnd].status=status_t;
                PCNodeArray[PCNodeEnd].type=type_t;
                PCNodeArray[PCNodeEnd].week=week_t;
                PCNodeArray[PCNodeEnd].second=second_t;
                if(status_t == 0)
                {
                        for( k=0;k<30;k++)
                        {
                                PCNodeArray[PCNodeEnd].command[k]=*(msg+5+k);
                        }
                        //printf("Week status_t = 0\n");
                }
                else
                {

                        for( k=0;k<9;k++)
                        {
                                PCNodeArray[PCNodeEnd].command[k]=*(msg+5+k);
                        }
                        //printf("Week status_t != 0\n");

                }
                PCNodeEnd++;
        }
        else
        {
                for(i=0;i<PCNodeEnd;i++)
                {
                        if((PCNodeArray[i].week==week_t) && (PCNodeArray[i].second >= second_t))
                                break;
                }
                PCNodeArrange( i, PCNodeEnd-i);
                PCNodeArray[i].status=status_t;
                PCNodeArray[i].week=week_t;
                PCNodeArray[i].second=second_t;
                if(status_t == 0)
                {
                        for( k=0;k<30;k++)
                        {
                                PCNodeArray[i].command[k]=*(msg+5+k);
                        }
                        //printf("Week_t else status_t = 0\n");
                }
                else
                {

                        for( k=0;k<9;k++)
                        {
                                PCNodeArray[i].command[k]=*(msg+5+k);
                        }
                        //printf("Week_t else status_t != 0\n");

                }
                PCNodeEnd++;
        }
        //printf("PCNodeEnd = %d\n",PCNodeEnd);
        return;


}

void PCNodeArrange(INT8U head	, INT8U len)
{
        INT8U k;
        INT8U i;
        for(k=head+len-1;k>=head;k--)
        {
                PCNodeArray[k+1].status=PCNodeArray[k].status;
                PCNodeArray[k+1].type=PCNodeArray[k].type;
                PCNodeArray[k+1].week=PCNodeArray[k].week;
                PCNodeArray[k+1].second=PCNodeArray[k].second;
                for(i=0;i<30;i++)
                        PCNodeArray[k+1].command[i]=PCNodeArray[k].command[i];
        }
        return;
}

void del_ProgramComNode( void)
{
        INT8U k;
        INT8U i;
        for(k=0;k<PCNodeEnd;k++)
        {
                PCNodeArray[k].status=PCNodeArray[k+1].status;
                //      PCNodeArray[k].type=PCNodeArray[k+1].type;
                PCNodeArray[k].week=PCNodeArray[k+1].week;
                PCNodeArray[k].second=PCNodeArray[k+1].second;
                for(i=0;i<30;i++)
                        PCNodeArray[k].command[i]=PCNodeArray[k+1].command[i];
        }
        PCNodeEnd=PCNodeEnd-1;
}

/*
 * 
 */


void  hdlc_send_2(void *data) 
{
        int sendState=0,send_num=0,total_num,PCTotalNum,pack_num,add_pack_num,loop_flag=1;
        int vs=0,rr_expect=0,n2=10,old_timeout_frame,good_num=0;
        enum myevent event_type;//enum 枚举变量
        int ii,k;
        INT8U err,*eveMsg,recChar,ctrField,count,nr,id,temp_len,timeout_frame,t_vs,eis_count;
        INT8U *msg,*p,*q,*r,*toMsg;//*toMsg是什么意思
        FRAME buffer[MAX_SEQ+1];//数据的send buffer
        INT32U addr,wod_end,temp_addr;
        INT8U SendDownBuffer[180];
        INT8U end_num, BSFlag;
        INT16U PackNum,EndLen;
        int BreakSendNum;
        struct frame_node *s;
        int i,IdFlag;
        int nn, SendLen;
        /*
         * ------------------------------------------------------*/
        INT16U  rf_photolen = 0;
        INT8U * msg_ab;
        INT8U SendDownBufferBF[180];
        int  fram_counts;

        /*-----------------------------------------------------------------------------*/

        for(;;)
        {
start:
                loop_flag=0;
                OSQFlush(timeoutQEvent);                 /*清除发送及超时队列里所有消息*/
                OSQFlush(sendRecMsg);
                IdFlag = 0;
                //printf("HdlcSend Start!\n");
                msg=(INT8U *)OSQPend(sendMsg,0,&err);   /*从发送队列里读取一条消息id,len,data*/
                /*---------------------------------------------*/
                msg_ab = msg;
                /*----------------------------------------------*/
                /*  sendMsg  是 从各个任务里  发出来的 主标示字 和 子标示字 */
                //printf("send down task start.....\n");
                //printf("sendMsg = %x %x %x %x\n",*msg,*(msg+1),*(msg+2),*(msg+3));
                old_timeout_frame=100000;                /*一个足够大的数*/
                good_num=0;
                q=msg;
                id=*q++;
                if(id ==STAR_0)
                {
                        IdFlag = *q++;
                        if(IdFlag ==0x01)//传27字节
                        {
                                //printf("Star Sensor data is being sent down...\n");
                                total_num=1;
                        }
                        else if(IdFlag==0x02)//传图像
                        {
                                //	eis_count=*q;   ///*第几个1分钟的数据*/
                                total_num = 6356;
                        }
                        else if(IdFlag == 0x03)
                        {
                                BreakSendNum = *q++;
                                BreakSendNum = BreakSendNum * 0x100 + *q;

                                q++;
                                BSFlag = *q;
                                if(BSFlag == 0x01)
                                        total_num = 6356;
                                else if(BSFlag == 0x02)
                                        total_num = 398;
                                //	wod_begin = BreakWodBegin + 165 * (BreakSendNum);
                                //printf("Hdlc Start BreakSendNum = %x,*q = %x, *q-- = %x, wod_begin = %x",BreakSendNum,*q,*(q-1),wod_begin);
                        }
                        else if(IdFlag == 0x04)
                        {
                                total_num = 398;
                                //printf("Star Small Photo!\n");

                        }
                        else if(IdFlag == 0x09)
                        {
                                total_num = 1;
                                //printf("Star Photo Finished!\n");
                        }

                }
                else if(id == MEMSGyro_0)
                {
                        total_num = 0x01;
                        //printf("MEMSGyro Test!");
                }
                else if(id == BDGPS_0)
                {
                        IdFlag = *q++;
                        total_num = 0x01;

                }
                else if (id==WOD_0)
                {
                        IdFlag = *q++;
                        if(IdFlag == 0x01)
                        {
                                //printf("WOD data is being sent down...\n");
                                pack_num = *q++;
                                pack_num=pack_num * 0x100+*q++;    /*总包数*/
                                total_num=pack_num;
                                BreakPointPack = pack_num;
                                end_num = *q;
                                //printf("pack_num = %x, end_num = %x\n",pack_num, end_num);
                        }
                        else if(IdFlag == 0x02)
                        {
                                //	//printf("*q++ = %x *q = %x\n",*q++,*q);
                                BreakSendNum = *q++;
                                BreakSendNum = BreakSendNum * 0x100 + *q;
                                total_num = BreakPointPack;
                                //	wod_begin = BreakWodBegin + 165 * (BreakSendNum);
                                //printf("Hdlc Wod BreakSendNum = %x,*q = %x, *q-- = %x, wod_begin = %x",BreakSendNum,*q,*(q-1),wod_begin);
                        }

                }
                else if (id==GPS_0)
                {
                        //printf("GPS data is being sent down...\n");
                        total_num=1;
                }
                else if(id==DLL_0)
                {
                        //printf("DLL data is being sent down...\n");
                        total_num=1;
                }
                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                else if(id == ProgramCom_0)
                {
                        /*000000000000000000000000                         000000000000000000000000000000                   000000000000000000000*/
                        IdFlag = *(msg_ab+1);
                        PackNum = *(msg_ab + 3);
                        PackNum = PackNum << 8;
                        PackNum += *(msg_ab + 2);
                        /*000000000000000000000000                         000000000000000000000000000000                   000000000000000000000*/
                        //printf("IdFlag=%d, PackNum = %d",IdFlag,PackNum);
                        switch(IdFlag)
                        {
                                case 0x01:
                                        fram_counts = *(msg_ab + 3);
                                        fram_counts = fram_counts << 8;
                                        fram_counts += *(msg_ab + 2);
                                        total_num = *(msg_ab + 5);
                                        total_num = total_num << 8;
                                        total_num += *(msg_ab + 4);  
                                        total_num -= fram_counts;
                                        break;

                                case 0x05:
                                        EndLen = (PackNum * 27) % 165;
                                        total_num = ((PackNum * 27) / 165) + 1;
                                        PCTotalNum = total_num;
                                        break;

                                case 0x06:
                                        EndLen = (PackNum * 21) % 165;
                                        total_num = ((PackNum * 21) / 165) + 1;
                                        PCTotalNum = total_num;
                                        break;

                                case 0x08:
                                        EndLen = (PackNum * 155) % 165;
                                        total_num = ((PackNum * 155) / 165) + 1;
                                        PCTotalNum = total_num;		
                                        break;

                                case 0x0A:
                                        EndLen = (PackNum * 22) % 165;
                                        total_num = ((PackNum * 22) / 165) + 1;
                                        PCTotalNum = total_num;		
                                        break;

                                case 0x04:
                                        EndLen = (PackNum * 512) % 165;
                                        total_num = ((PackNum * 512) / 165) + 1;
                                        PCTotalNum = total_num;		
                                        break;

                        }
                }
                else if(id==ACS_0)
                {
                        IdFlag = *q++;
                        if (IdFlag == 0x02)
                                total_num = 1;
                }
                else if(id==RF_0)
                {

                        //printf("RF-----------------\n");
                        IdFlag = *q++;
                        if(IdFlag ==0x01)//传27字节
                                total_num=1;
                        else if(IdFlag==0x02)
                                total_num = 1;
                        else if(IdFlag == 0x03)
                                total_num = 1;
                        else if(IdFlag == 0x04) {
                                rf_photolen = ((*(msg_ab + 2))<<8) + (*(msg_ab + 3));
                                //printf ("rf_photolen  84 : %d\n", rf_photolen);
                                total_num = (rf_photolen * 512)/165 + 1;
                                //printf ("RF total_num  : %d \n", total_num);
                        }
                        else if(IdFlag == 0x05)
                        {
                                fram_counts = *(msg_ab + 3);
                                fram_counts = fram_counts << 8;
                                fram_counts += *(msg_ab + 2);
                                total_num = *(msg_ab + 5);
                                total_num = total_num << 8;
                                total_num += *(msg_ab + 4);  
                                total_num -= fram_counts;
                                //printf (" duan dian hou rf_photolen  84 : %d\n", rf_photolen);
                        }
                        else
                        {}

                }

                else
                        total_num = 0;
                /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                /*---------------------------------------------------------------*/
                if(total_num==0)
                {
                        loop_flag=1;
                        continue;
                }
                //printf("loop_flag=%x\n",loop_flag);
                sendState=1;
                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                send_down_frame(sabm,0,0,0);                /*发送一个SABM桢*/
                OSSemPost(SHARE_SENDDOWN);
                //OSTimeDly(60);
                while(loop_flag==0)                         /*发送一次数据的过程*/
                {
                        OSTimeDly(1);
                        //printf("#1# \n");
                        msg=(INT8U *)OSQPend(sendRecMsg,3000,&err);

                        if(err!=OS_NO_ERR)
                                goto start;

                        if( (*msg != SEND_TIMEOUT) &&(*msg != SEND_STOP) )//
                        {
                                //printf("#2# \n");
                                msg=msg+1;
                                recChar=*msg&(0xff);          /*Ctr field*/

                                //msg++;                                /*指针加1*/
                                switch(sendState)
                                {
                                        case 4:
                                                if(recChar==0x1F)
                                                {
                                                        total_num=0;
                                                        send_num=0;
                                                        vs=0;
                                                        rr_expect=0;
                                                        loop_flag=1;

                                                }
                                                /*  赵文军 补充 start */
                                                /*-----------------------------------------------*/
                                                goto start;
                                                /*  赵文军 补充 end */
                                                /*-----------------------------------------------*/
                                                break;

                                        case 1:                              /*发送SABM*/
                                                if (recChar==0x73)                 /*收到UA*/
                                                {
                                                        sendState=2;    /*准备发送I_frame*/
                                                        vs=0;
                                                        send_num=0;

                                                }

                                                break;

                                        case 2:                    /*发送I_frame的过程中*/
                                                ctrField=recChar&0x03;
                                                if (ctrField==0x01)              /*received a S frame*/
                                                {
                                                        ctrField=recChar&0x0c;
                                                        if (ctrField==0)
                                                        {
                                                                nr=(recChar&0xe0)>>5;        /*取出祯序号*/
                                                                //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                //printf("nr=%x\n",nr);
                                                                //printf("rr_expect=%x\n",rr_expect);
                                                                //OSSemPost(SHARE_PRINTF);
                                                                if(nr==0)
                                                                        nr=8;
                                                                if((nr-1)==rr_expect)
                                                                {
                                                                        //s=hdlc_stop_timer(head_node,nr-1);
                                                                        RR_TIMEOUT.TIMEOUT_CHECK=0;
                                                                        good_num+=1;
                                                                        //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                                        //printf("good_num=%x\n",good_num);
                                                                        //OSSemPost(SHARE_PRINTF);
                                                                        if (good_num==total_num)
                                                                        {
                                                                                sendState=3;
                                                                                break;
                                                                        }
                                                                        if(rr_expect<7)
                                                                                rr_expect+=1;
                                                                        else
                                                                                rr_expect=0;
                                                                }
                                                        }
                                                }
                                                break;

                                        case 3:

                                                break;

                                }//switch

                                switch(sendState)
                                {
                                        case 2:
                                                //total_num=0xff;
                                                memset(SendDownBuffer,0x00,180); 
                                                //printf("send_num= %x, total_num= %x\n",send_num,total_num);
                                                if (send_num<=(total_num-1))
                                                {
                                                        if (send_num==0)
                                                        {
                                                                switch(id)
                                                                {
                                                                        /*   赵文军 完成 ACS  */
                                                                        /*000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                                        case RF_0:
                                                                                if((IdFlag ==0x01) || (IdFlag == 0x02) || (IdFlag == 0x04) ||(IdFlag == 0x03))
                                                                                        addr = RF_DATA;
                                                                                if(IdFlag == 0x05)
                                                                                        addr = RF_DATA + (165*fram_counts);
                                                                                break;
                                                                        case ACS_0:           /*姿控配置数据包*/
                                                                                addr = acsaddr;
                                                                                break;

                                                                                /*0000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                                        case STAR_0:     //星敏感器数据

                                                                                if(IdFlag ==0x01)
                                                                                {
                                                                                        addr = STAR_sensor_DATA;
                                                                                }
                                                                                else if(IdFlag == 0x02)
                                                                                {
                                                                                        addr = EIS_DATA;
                                                                                }
                                                                                else if(IdFlag == 0x03)
                                                                                {
                                                                                        send_num = BreakSendNum;
                                                                                        if(BSFlag == 0x01)
                                                                                                addr = EIS_DATA;
                                                                                        else if(BSFlag == 0x02)
                                                                                                addr = EIS_DATA_SMALL;

                                                                                        //addr = EIS_DATA + PackNum * 165;
                                                                                        //printf("PackNum = %x\n",PackNum);
                                                                                }
                                                                                else if(IdFlag == 0x04)
                                                                                {
                                                                                        addr = EIS_DATA_SMALL;
                                                                                }
                                                                                break;
                                                                        case DLL_0:
                                                                                addr = DLL_DATA;
                                                                                break;
                                                                        case MEMSGyro_0:
                                                                                addr = MEMSGyro_DATA;
                                                                                break;
                                                                        case BDGPS_0:
                                                                                addr = BDGPS_DATA;
                                                                                break;
                                                                        case GPS_0:
                                                                                addr = GPS_DATA; //GPS数据
                                                                                break;
                                                                        case ProgramCom_0:
                                                                                /*0000000000000000000000000000000000000000000000000*/
                                                                                if (IdFlag == 0x01) 
                                                                                        addr = PC_DATA + (165*fram_counts);

                                                                                else
                                                                                        addr = PC_DATA;
                                                                                break;
                                                                                /*0000000000000000000000000000000000000000000000000000*/
                                                                        case WOD_0:
                                                                                addr=wod_begin; 
                                                                                //	printf("case WOD addr = %x\n",addr);
                                                                                break;
                                                                                /*-------------------------------------------*/
                                                                        default:
                                                                                break;
                                                                                /*----------------------------------*/
                                                                }//switch
                                                        }//if (send_num==0)
                                                        p=q;
                                                        r=(INT8U *)&buffer[vs];
                                                        //printf("id= %x\n",id);
                                                        /*---------------------------------------------------*/
                                                        for (nn = 0 ; nn < 180; nn ++) 
                                                                SendDownBuffer[nn] = 0x00; 
                                                        /*-------------------------------------*/
                                                        switch(id)
                                                        {
                                                                case STAR_0:
                                                                        if(IdFlag == 0x01)	
                                                                        {
                                                                                SendDownBuffer[0]=0x05;//主标识字
                                                                                SendDownBuffer[1]=0x00;
                                                                                SendDownBuffer[2]=0x03;//子标识字
                                                                                SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                                SendDownBuffer[4]=0;//包数据长度,高字节
                                                                                SendDownBuffer[5]=0;
                                                                                SendDownBuffer[6]=0;//
                                                                                SendDownBuffer[7]=0x1B;
                                                                                SendDownBuffer[8]=00;//

                                                                                OSSchedLock();
                                                                                for(nn = 0; nn < 27; nn++)
                                                                                {
                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr+nn);
                                                                                }
                                                                                OSSchedUnlock();

                                                                                //printf("addr = %x\n",addr);
                                                                        }
                                                                        else if(IdFlag == 0x09)
                                                                        {
                                                                                SendDownBuffer[0]=0x05;//主标识字
                                                                                SendDownBuffer[1]=0x00;
                                                                                SendDownBuffer[2]=0x09;//子标识字
                                                                                SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                                SendDownBuffer[4]=0;//包数据长度,高字节
                                                                                SendDownBuffer[5]=0;
                                                                                SendDownBuffer[6]=0;//
                                                                                SendDownBuffer[7]=0x1;
                                                                                SendDownBuffer[8]=00;//
                                                                                SendDownBuffer[9]=StarFlag;
                                                                                //printf("Photo Send!\n");

                                                                        }
                                                                        else 
                                                                        {

                                                                                SendDownBuffer[0]=0x05;//主标识字
                                                                                SendDownBuffer[1]=0x00;
                                                                                SendDownBuffer[2]=0x04;//子标识字
                                                                                SendDownBuffer[3]=total_num;//包数据长度,低字节
                                                                                SendDownBuffer[4]=total_num>>8;//包数据长度,高字节
                                                                                SendDownBuffer[5]=send_num;
                                                                                SendDownBuffer[6]=send_num>>8;//
                                                                                SendDownBuffer[7]=0xA5;
                                                                                SendDownBuffer[8]=00;//

                                                                                if(send_num == (total_num-1))
                                                                                {
                                                                                        if(BSFlag == 0x01)
                                                                                                end_num = (8192*128)%165;
                                                                                        else if(BSFlag == 0x02)
                                                                                                end_num = (1024*64)%165;
                                                                                        SendDownBuffer[7] = end_num;
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < end_num; nn++)
                                                                                        {
                                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                        }
                                                                                        OSSchedUnlock();
                                                                                }
                                                                                else
                                                                                {
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < 165; nn++)
                                                                                        {
                                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                        }
                                                                                        OSSchedUnlock();
                                                                                }
                                                                                //printf("addr = %x\n",addr);

                                                                        }
                                                                        break;

                                                                case DLL_0:
                                                                        SendDownBuffer[0]=0x0B;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x04;//子标识字
                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                        SendDownBuffer[4]=0;//包数据长度,高字节
                                                                        SendDownBuffer[5]=0;
                                                                        SendDownBuffer[6]=0;//
                                                                        SendDownBuffer[7]=0x11;
                                                                        SendDownBuffer[8]=00;//

                                                                        OSSchedLock();
                                                                        for(nn = 0; nn < 18; nn++)
                                                                        {
                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr+nn);;
                                                                        }
                                                                        OSSchedUnlock();

                                                                        //printf("addr = %x\n",addr);
                                                                        break;

                                                                case MEMSGyro_0:

                                                                        SendDownBuffer[0]=0x06;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x01;//子标识字
                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                        SendDownBuffer[4]=0;//包数据长度,高字节
                                                                        SendDownBuffer[5]=0;
                                                                        SendDownBuffer[6]=0;//
                                                                        SendDownBuffer[7]=0x15;
                                                                        SendDownBuffer[8]=00;//

                                                                        OSSchedLock();
                                                                        for(nn = 0; nn < 21; nn++)
                                                                        {
                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr+nn);
                                                                        }
                                                                        OSSchedUnlock();
                                                                        break;

                                                                case GPS_0:
                                                                        SendDownBuffer[0]=0x07;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x01;//子标识字
                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                        SendDownBuffer[4]=0;//包数据长度,高字节
                                                                        SendDownBuffer[5]=0;
                                                                        SendDownBuffer[6]=0;//
                                                                        SendDownBuffer[7]=0x40;
                                                                        SendDownBuffer[8]=00;//

                                                                        OSSchedLock();
                                                                        for(nn = 0; nn < 64; nn++)
                                                                        {
                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr+nn);
                                                                        }
                                                                        OSSchedUnlock();

                                                                        //printf("addr = %x\n",addr);
                                                                        break;

                                                                        /*00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                                case ProgramCom_0:

                                                                        SendDownBuffer[0]=0x0E;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x0E;//子标识字
                                                                        SendDownBuffer[3]=total_num;//包数据长度,低字节
                                                                        SendDownBuffer[4]=total_num>>8;//包数据长度,高字节
                                                                        SendDownBuffer[5]=send_num;
                                                                        SendDownBuffer[6]=send_num>>8;//
                                                                        SendDownBuffer[7]=0xA5;
                                                                        SendDownBuffer[8]=00;//

                                                                        /*-------------------------------------------*/
                                                                        if(IdFlag == 0x01)
                                                                        {
                                                                                SendDownBuffer[3]=(total_num + fram_counts);//包数据长度,低字节
                                                                                SendDownBuffer[4]=(total_num + fram_counts)>>8;//包数据长度,高字节
                                                                                SendDownBuffer[5]=(send_num + fram_counts);
                                                                                SendDownBuffer[6]=(send_num + fram_counts)>>8;
                                                                                if(send_num == (total_num-1))
                                                                                {

                                                                                        SendDownBuffer[7] = EndLen;
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < end_num; nn++)
                                                                                        {
                                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                        }

                                                                                        OSSchedUnlock();
                                                                                }
                                                                                else
                                                                                {
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < 165; nn++)
                                                                                        {
                                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                        }

                                                                                        OSSchedUnlock();
                                                                                }
                                                                        }
                                                                        else {
                                                                                if(send_num == (total_num-1))
                                                                                {
                                                                                        SendDownBuffer[7] = EndLen; 
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < SendDownBuffer[7]; nn++)
                                                                                        {
                                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                        }
                                                                                        OSSchedUnlock();
                                                                                }
                                                                                else
                                                                                {
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < 165; nn++)
                                                                                        {
                                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                        }
                                                                                        OSSchedUnlock();
                                                                                }
                                                                        }

                                                                        break;
                                                                        /*---------------------------------------*/

                                                                        /*0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

                                                                case ACS_0:
                                                                        /*   赵文军 完成 ACS  */
                                                                        /*----------------遥测 数据 start------*/
                                                                        if (IdFlag == 0x02) {
                                                                                SendDownBuffer[0]=0x09;//主标识字
                                                                                SendDownBuffer[1]=0x00;
                                                                                SendDownBuffer[2]=0x02;//子标识字
                                                                                SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                                SendDownBuffer[4]=0;//包数据长度,高字节
                                                                                SendDownBuffer[5]=0;
                                                                                SendDownBuffer[6]=0;//
                                                                                SendDownBuffer[7]=0x8B;
                                                                                SendDownBuffer[8]=00;//

                                                                                OSSchedLock();
                                                                                for(nn = 0; nn < 130; nn++)
                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr+nn);
                                                                                OSSchedUnlock();
                                                                                //printf ("SendDownBuffer *****************\n");
                                                                        }
                                                                        break;
                                                                        /*--------------遥测数据  end----------------*/

                                                                case BDGPS_0:
                                                                        SendDownBuffer[0]=0x08;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x06;//子标识字
                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                        SendDownBuffer[4]=0x00;//包数据长度,高字节
                                                                        SendDownBuffer[5]=0x00;
                                                                        SendDownBuffer[6]=0x00;//
                                                                        SendDownBuffer[7]=0x9B;
                                                                        SendDownBuffer[8]=0x00;//

                                                                        OSSchedLock();
                                                                        for(nn = 0; nn < 155; nn++)
                                                                        {
                                                                                SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + nn);
                                                                        }
                                                                        OSSchedUnlock();
                                                                        //printf("Hdlc BDGPS!\n");
                                                                        break;

                                                                case WOD_0:
                                                                        if(IdFlag == 0x02)
                                                                        {
                                                                                send_num = BreakSendNum;
                                                                                //printf("WOD_0  BreakSendNum = %x, IdFlag = %x\n",BreakSendNum,IdFlag);
                                                                                IdFlag = 0xFF;
                                                                        }

                                                                        SendDownBuffer[0]=0x0C;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x03;//子标识字
                                                                        SendDownBuffer[3]=total_num;//包数据长度,低字节
                                                                        SendDownBuffer[4]=total_num>>8;//包数据长度,高字节
                                                                        SendDownBuffer[5]=send_num;
                                                                        SendDownBuffer[6]=send_num>>8;//
                                                                        SendDownBuffer[7]=0xA5;
                                                                        SendDownBuffer[8]=00;//

                                                                        if(send_num == (total_num-1))
                                                                        {
                                                                                SendDownBuffer[7]=end_num;
                                                                                OSSchedLock();
                                                                                for(nn = 0; nn < end_num; nn++)
                                                                                {
                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                }
                                                                                OSSchedUnlock();
                                                                        }
                                                                        else
                                                                        {
                                                                                //printf("send addr = %x\n",addr);
                                                                                OSSchedLock();
                                                                                for(nn = 0; nn < 165; nn++)
                                                                                {
                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                }
                                                                                OSSchedUnlock();
                                                                        }
                                                                        //printf("send_num = %d addr = %x\n",send_num,(addr + send_num * 165 + nn));
                                                                        break;
                                                                        /*000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/
                                                                case RF_0:
                                                                        SendDownBuffer[0]=0x04;//主标识字
                                                                        SendDownBuffer[1]=0x00;
                                                                        SendDownBuffer[2]=0x00;//子标识字
                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                        SendDownBuffer[4]=0x00;//包数据长度,高字节
                                                                        SendDownBuffer[5]=0x00;
                                                                        SendDownBuffer[6]=0x00;//
                                                                        SendDownBuffer[7]=0x00;
                                                                        SendDownBuffer[8]=0x00;//
                                                                        switch (IdFlag) {
                                                                                case 0x01:
                                                                                        if (*(msg_ab+2) == 0xAA && *(msg_ab + 3) == 0XAA) {
                                                                                                SendDownBuffer[9] = 0xAA;
                                                                                                SendDownBuffer[10] = 0xAA;
                                                                                        }
                                                                                        if (*(msg_ab+2) ==  0xBB && *(msg_ab + 3) == 0xBB) {
                                                                                                SendDownBuffer[9] = 0xBB;
                                                                                                SendDownBuffer[10] = 0xBB;
                                                                                        }
                                                                                        SendDownBuffer[2]=0x01;//子标识字
                                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                                        SendDownBuffer[7]=0x19;
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < 23; nn++)
                                                                                        {
                                                                                                SendDownBuffer[11+nn] = *(volatile INT8U *)(addr+nn);
                                                                                        }
                                                                                        OSSchedUnlock();
                                                                                        break;


                                                                                case 0x02:
                                                                                        if (*(msg_ab+2) == 0xAA && *(msg_ab + 3) == 0XAA) {
                                                                                                SendDownBuffer[9] = 0xAA;
                                                                                                SendDownBuffer[10] = 0xAA;
                                                                                        }
                                                                                        if (*(msg_ab+2) ==  0xBB && *(msg_ab + 3) == 0xBB) {
                                                                                                SendDownBuffer[9] = 0xBB;
                                                                                                SendDownBuffer[10] = 0xBB;
                                                                                        }
                                                                                        SendDownBuffer[2]=0x02;//子标识字
                                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                                        SendDownBuffer[7]=0x0D;
                                                                                        OSSchedLock();
                                                                                        for(nn = 0; nn < 23; nn++)
                                                                                        {
                                                                                                SendDownBuffer[11+nn] = *(volatile INT8U *)(addr+nn);
                                                                                        }
                                                                                        OSSchedUnlock();
                                                                                        break;


                                                                                case 0x03:
                                                                                        if (*(msg_ab+2) == 0xAA && *(msg_ab + 3) == 0XAA) {
                                                                                                SendDownBuffer[9] = 0xAA;
                                                                                                SendDownBuffer[10] = 0xAA;
                                                                                        }
                                                                                        if (*(msg_ab+2) ==  0xBB && *(msg_ab + 3) == 0xBB) {
                                                                                                SendDownBuffer[9] = 0xBB;
                                                                                                SendDownBuffer[10] = 0xBB;
                                                                                        }
                                                                                        SendDownBuffer[2]=0x03;//子标识字
                                                                                        SendDownBuffer[3]=0x01;//包数据长度,低字节
                                                                                        SendDownBuffer[7]=0x02;
                                                                                        break;


                                                                                case 0x04:
                                                                                        SendDownBuffer[2]=0x04;//子标识字
                                                                                        SendDownBuffer[3]=total_num;//包数据长度,低字节
                                                                                        SendDownBuffer[4]=total_num>>8;//包数据长度,高字节
                                                                                        SendDownBuffer[5]=send_num;
                                                                                        SendDownBuffer[6]=send_num>>8;
                                                                                        SendDownBuffer[7]=0xA5;
                                                                                        if(send_num == (total_num-1))
                                                                                        {
                                                                                                end_num = (rf_photolen * 512)%165;;
                                                                                                //printf ("rf_photolen: %d\n", rf_photolen);
                                                                                                //printf ("send_num : %d \n", send_num);
                                                                                                //printf ("send_num : %d\n", send_num);
                                                                                                //printf ("total_num : %d", total_num);
                                                                                                rf_photolen = 0;
                                                                                                SendDownBuffer[7] = end_num;
                                                                                                OSSchedLock();
                                                                                                for(nn = 0; nn < end_num; nn++)
                                                                                                {
                                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                                }
                                                                                                OSSchedUnlock();
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                                OSSchedLock();
                                                                                                //printf ("rf_photolen: %d\n", rf_photolen);
                                                                                                //printf ("send_num : %d \n", send_num);
                                                                                                //printf ("send_num + fram_counts : %d\n", send_num+fram_counts);
                                                                                                //printf ("total_num + fram_counts - 1 : %d", total_num + fram_counts-1);
                                                                                                for(nn = 0; nn < 165; nn++)
                                                                                                {
                                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                                }
                                                                                                OSSchedUnlock();
                                                                                        }
                                                                                        //printf ("RF  shuju   down  success\n");
                                                                                        break;


                                                                                case 0x05:
                                                                                        SendDownBuffer[2]=0x05;//子标识字
                                                                                        SendDownBuffer[3]=(total_num + fram_counts);//包数据长度,低字节
                                                                                        SendDownBuffer[4]=(total_num + fram_counts)>>8;//包数据长度,高字节
                                                                                        SendDownBuffer[5]=(send_num + fram_counts);
                                                                                        SendDownBuffer[6]=(send_num + fram_counts)>>8;
                                                                                        SendDownBuffer[7]=0xA5;
                                                                                        if(send_num == (total_num-1))
                                                                                        {

                                                                                                end_num = (rf_photolen * 512) % 165;
                                                                                                //printf ("rf_photolen: %d\n", rf_photolen);
                                                                                                //printf ("send_num : %d \n", send_num);
                                                                                                //printf ("send_num + fram_counts : %d\n", send_num+fram_counts);
                                                                                                //printf ("total_num + fram_counts - 1 : %d", total_num + fram_counts-1);
                                                                                                rf_photolen = 0;
                                                                                                SendDownBuffer[7] = end_num;
                                                                                                OSSchedLock();
                                                                                                for(nn = 0; nn < end_num; nn++)
                                                                                                {
                                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                                }

                                                                                                OSSchedUnlock();
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                                OSSchedLock();
                                                                                                //printf ("rf_photolen: %d\n", rf_photolen);
                                                                                                //printf ("send_num : %d \n", send_num);
                                                                                                //printf ("send_num + fram_counts : %d\n", send_num+fram_counts);
                                                                                                //printf ("total_num + fram_counts - 1 : %d", total_num + fram_counts-1);
                                                                                                for(nn = 0; nn < 165; nn++)
                                                                                                {
                                                                                                        SendDownBuffer[9+nn] = *(volatile INT8U *)(addr + send_num * 165 + nn);
                                                                                                }

                                                                                                OSSchedUnlock();
                                                                                        }
                                                                                        //printf ("RF  shuju   down  success\n");
                                                                                        break;

                                                                                default:
                                                                                        break;
                                                                        }
                                                                        break;
                                                                        /*000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000*/


                                                                default:

                                                                        break;
                                                        }
                                                        OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);

                                                        //		send_down_frame(i_data,0,vs,&buffer[vs]);
                                                        HJMSend_down_frame(i_data,0,vs,SendDownBuffer);
                                                        //printf("%d\n",vs);
                                                        RR_TIMEOUT.TIMEOUT_RETRY=5;
                                                        RR_TIMEOUT.sendnum=vs;
                                                        RR_TIMEOUT.second=0;
                                                        RR_TIMEOUT.TIMEOUT_CHECK=1;

                                                        //printf("%x sent down over\n",send_num);
                                                        OSSemPost(SHARE_SENDDOWN);
                                                        //OSTimeDly(200);
                                                        //s=hdlc_start_timer(head_node,vs);
                                                        //OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
                                                        //printf("vs=%x\n",vs);
                                                        //OSSemPost(SHARE_PRINTF);
                                                        /*--------------------------------------------------------*/
                                                        for (nn = 0 ; nn < 180; nn ++) 
                                                                SendDownBufferBF[nn] = 0x00; 
                                                        for (nn = 0 ; nn < 180; nn ++) 
                                                                SendDownBufferBF[nn] = SendDownBuffer[nn]; 
                                                        /*----------------------------------------------------------*/

                                                        send_num+=1;
                                                        if(vs<7)
                                                                vs+=1;
                                                        else
                                                                vs=0;


                                                }

                                                break;
                                        case 3:
                                                OSSemPend(SHARE_SENDDOWN,0,&ERR_SHARE_SENDDOWN);
                                                send_down_frame(disc,0,0,0);
                                                OSSemPost(SHARE_SENDDOWN);
                                                //printf("send down task finished. \n");
                                                //s=hdlc_start_timer(head_node,disc_num);
                                                sendState=4;
                                                break;

                                        case 4:
                                                break;
                                }

                        }//if end
                        /*-----------------------------------------------*/
                        else if(*msg == SEND_TIMEOUT)//如果是超时信号
                        {
                                /*  以下 语句 表示 OBC 的错误处理 ，当OBC发送一个数据，但地面没有收到，则OBC会重复发送5次上次发送的信息，*/
                                /*  代码不完整，需要补充 */
                                // send_down_frame(i_data,0,RR_TIMEOUT.sendnum,&buffer[RR_TIMEOUT.sendnum]);
                                HJMSend_down_frame(i_data,0,RR_TIMEOUT.sendnum,SendDownBufferBF);
                                RR_TIMEOUT.second=0;
                                //printf("TIME OUT!\n");
                        }
                        /*------------------------------------------------*/
                        else //如果是停止信号
                        {
                                //printf("STOP!\n");
                                goto start;
                        }
                }//while(loop) end

        }//for end
}
