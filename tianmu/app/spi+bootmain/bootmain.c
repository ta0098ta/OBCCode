/*
   NS-2 OBC
   Main function of the bootloader
   By Meng Li
   2009/09/28
   By Tian Ning
   2013/05/28


   By Tian Ning
   2016/01/27
   By Tian Ning
   2017/02/07
   201702/11/01
   goto 04 
   starting application at 0x00000100... reset!!!
   */

#include "CRC.h"
#include <common.h>
#include <watchdog.h>
#include  "spi.h"/*2013*/
#include <ppc_defs.h>/*2013*/
#include <asm/processor.h>/*2013*/



#define ROM_ADDRESS 0xff000000
//#define ROM_ADDRESS 0xff820000
#define TEMP_ADDRESS 0x30000
//#define APPLICATION_LEN 0x20000
#define APPLICATION_LEN 0x40000
#define MAX_REC_LEN 132
#define MAX_SEND_LEN 1058
#define MAX_SEQ 7

#define ua 1
#define rr 2
#define rej 3
#define dm 4
#define check 5

#define OK 0
#define ERROR 1

#define TRUE 1
#define FALSE 0

#define INITFCS   0xffff  /* Initial FCS value */
#define GOODFCS   0xf0b8  /* Good final FCS value */
#define R29 register gd_t *gd_t asm("r29")	

typedef struct BufferDescriptor

{
        ushort bd_cstatus;     /* control and status */
        ushort bd_length;      /* transfer length */
        unsigned char  *buf_addr;        /* buffer address */

} BD;

int recSize,sendSize;
int hdlc_channel;
void send_down_frame(unsigned char id,unsigned char recnum);

void Can_init(void);
int get_hdlc_channel(void);
void move_eprom_ram(void);
void move_pr(unsigned int n);
int RAM_test(void);
void delay_m(void);
//ssize_t spi_write_test (uchar);
void show_regs(struct pt_regs * regs);

unsigned short c515c_state; 
unsigned int read_frame,j,ram_state,bytetotal;
unsigned int result,input;
unsigned int result1[32];
/**********************************************************
  用于系统上电后接收地面的引导命令以及接收上载的操作系统 
  入口参数： state：系统当前状态
  0--表示系统等待地面的引导命令                      
  1--表示系统等待地面上载操作系统
  返回值：boot_flag:引导方式
  1--表示从地面加载操作系统
  2--表示启动固化的操作系统
 ***********************************************************/
int link_protocol(unsigned int state)
{
        unsigned char  data[MAX_REC_LEN*16], *msg, recChar, mainFlag, ctrField;
        static char lastcommand[CFG_CBSIZE] = { 0, };
        //unsigned char  sendbuf[MAX_SEND_LEN];

        static	int	len;
        unsigned short recState,vr,ns;//,rej_vr;  2003.12.30
        unsigned short temp;
        unsigned short id,totalFrames,curFrame,byteCount,app_num,ret_flag,flag;

        unsigned char *app_addr;                 //,con_app_addr=0x10025000; 2003.12.30

        int boot_flag;
        app_addr=(unsigned char*)TEMP_ADDRESS;
        recState=0;
        //rej_vr=0;  2003.12.30
        vr=0;
        app_num=0;
        totalFrames=curFrame=byteCount=0;
        ret_flag=0;            /*返回标志清零*/
        boot_flag=0;





        if (state ==0)	printf("send down the self-state\n"); else printf("start the application update\n");
        len = 0;
        while(1)
        {  
                hdlc_channel = get_hdlc_channel();
                WATCHDOG_RESET();

#if 0

                sendbuf[0] = 0x0;
                sendbuf[1] = 0x1;
                sendbuf[2] = 0x2;
                sendbuf[3] = 0x3;
                sendbuf[4] = 0x4;
                sendbuf[5] = 0x5;
                sendbuf[6] = 0x6;
                sendbuf[7] = 0x7;
                sendbuf[8] = c515c_state & 0xff;
                sendbuf[9] = ram_state & 0xff;

                if (hdlc_channel == 1) 
                {
                        //printf("fcc1 send\n");
                        sendbuf[0] = 0;
                        fcc1_hdlc_send_data(sendbuf, 10);
                        hdlc_app_rx0(&data[MAX_REC_LEN*j], &len);
                }
                else
                {
                        //printf("fcc3 send\n");
                        sendbuf[0] = 1;
                        fcc3_hdlc_send_data(sendbuf, 10);
                        hdlc_app_rx1(&data[MAX_REC_LEN*j], &len);
                }	

                if ((len>0)&&(data[MAX_REC_LEN*j+1]==0x55))
                {
                        boot_flag=2;
                        break;
                }

                j+=1;                        /*调整指针*/
                if (j>=16) j=0;


#else


                if (hdlc_channel == 1) 
                {
                        printf("fcc1 send_receive\n");//20130529
                        hdlc_app_rx0(&data[MAX_REC_LEN*j], &len);
                        printf("len = %x, len_addr = %x\n", len, &len);//20130529
                }
                else
                {
                        printf("fcc3 send_receive\n");//20130529
                        hdlc_app_rx1(&data[MAX_REC_LEN*j], &len);
                }	

                if(len > 0)          /*接收到数据*/
                {
                        printf("received a frame\n");  //20130529  
                        msg=&data[MAX_REC_LEN*j+1];
                        recChar=*msg;                /*Ctr field*/     
                        msg++;

                        switch(recState)
                        {
                                case 0:                             /*初始状态*/
                                        if(recChar==0x3f)              /*received a SABM frame*/
                                        {
                                                send_down_frame(ua,0);         /*发送一个UA_frame*/   
                                                printf("Received a SABM frame.\n");       
                                                vr=0;
                                                recState=1;        
                                                totalFrames=curFrame=byteCount=0;
                                        }
                                        break;

                                case 1:                               /*接收数据*/
                                        if(recChar==0x3f) 
                                        {
                                                send_down_frame(ua,0);          /*发送一个UA_frame,确保对方没收到UA而重发的SABM得到响应*/
                                                vr=0;                           
                                                totalFrames=curFrame=byteCount=0;         
                                                break;
                                        }       
                                        //printf("ctrField = %x\n", recChar);
                                        ctrField=recChar&0x01;     
                                        if (ctrField==0)                  /*received a I frame*/
                                        {
                                                //printf("Received a I frame.\n");
                                                ns=(recChar&0x0e)>>1;            /*取得对方发送的序列号*/

                                                if(ns==vr)
                                                {
                                                        if(vr<7)          
                                                                vr+=1;
                                                        else
                                                                vr=0;
                                                        send_down_frame(rr,vr);
                                                }       
                                                else
                                                {            
                                                        send_down_frame(rr,vr);    /*不等于VR的桢抛弃掉*/
                                                        break;
                                                }   

                                                /************************取出信息域数据进行处理 开始************************/

                                                mainFlag=*msg;              /*取出主标识字*/           
                                                msg++;          

                                                id=*msg+(*(msg+1))*0x100;/*取出ID*/
                                                msg+=2;
                                                totalFrames=*msg+(*(msg+1))*0x100;/*取出总帧数*/
                                                msg+=2;
                                                curFrame=*msg+(*(msg+1))*0x100;/*取出第几帧*/
                                                msg+=2;
                                                byteCount=*msg+(*(msg+1))*0x100;/*取出字节数*/
                                                msg+=2;
                                                //loop_times=byteCount;


                                                //printf("Mainflag = 0x%x, id = 0x%x, totalFrames = 0x%x, curFrame = 0x%x, byteCount = 0x%x\n ", mainFlag, id, totalFrames, curFrame, byteCount);
                                                //printf("state = %x, app_num = %x\n", state, app_num);
                                                if(state==0)      /*等待地面命令*/
                                                {
                                                        if(mainFlag!=0x01)			//OBC指令
                                                                break;
                                                        if(id==0x0002)	//上注
                                                        {
                                                                boot_flag=1;       /*从地面引导操作系统*/
                                                        }
                                                        else if(id==0x000b)
                                                                boot_flag=2;       /*启动固化操作系统*/
                                                        break;
                                                }

                                                if(state==1)          /*从地面引导操作系统*/    
                                                {           
                                                        if(mainFlag!=0x01)
                                                                break;           
                                                        if(id!=0x0002)
                                                                break;
                                                        if(app_num!=curFrame)
                                                                break;    

                                                        //printf("copy to temp address, app_addr=%x\n", app_addr);
                                                        memcpy(app_addr, msg, byteCount);	//拷贝程序到临时程序区
                                                        app_addr+=byteCount;
                                                        app_num++;            /*应用程序桢数加1*/			
                                                        break;
                                                }	
                                        } 

                                        if(recChar==0x53)         /*received a DISC frame*/
                                        {
                                                send_down_frame(dm,0);
                                                printf("Received a disc frame.\n");       
                                                recState=2;                /*进入拆除状态*/

                                                if(state==0)               /*等待地面命令*/
                                                {
                                                        if(boot_flag!=0)
                                                                ret_flag=1;            /*置返回标志*/  
                                                }

                                                if(state==1)               /*从地面引导操作系统*/
                                                {
                                                        if(app_num==totalFrames)
                                                        {
                                                                bytetotal=app_addr-(unsigned char*)TEMP_ADDRESS;
                                                                printf("Application has been uploaded. byte total = %x, app_addr = %x\n", bytetotal, app_addr);  
                                                                ret_flag=1;            /*置返回标志*/    

                                                                WATCHDOG_RESET();
                                                                printf("start copy to 0x0, 0x100 = %x\n", *(volatile unsigned long *)(0x100));
                                                                printf("bytetotal = %x\n", bytetotal);
                                                                move_pr(bytetotal);
                                                                printf("finish copy to 0x0, 0x100 = %x\n", *(volatile unsigned long *)(0x100));		
                                                                strcpy(lastcommand , "go 100\r");
                                                                flag = 0;
                                                                ret_flag = run_command (lastcommand, flag);
                                                                ret_flag=1;

                                                                return;
                                                        }
                                                        else printf("app_num=%x, totalFrames=%x\n", app_num, totalFrames);
                                                }
                                        }

                                        break;

                                case 2:                    /*拆除状态*/
                                        if(recChar==0x3f)         /*收到SABM*/
                                        {
                                                send_down_frame(ua,0);   /*发送一个UA_frame*/
                                                vr=0;
                                                recState=1;  
                                                totalFrames=curFrame=byteCount=0;         
                                                //app_addr=con_app_addr;   2003.12.30  
                                                break;               
                                        }
                                        if(recChar==0x53)         /*received a DISC frame*/
                                        { 
                                                send_down_frame(dm,0);    
                                        } 
                                        break;
                                default:
                                        break;      
                        } /*switch*/  


                        j+=1;                        /*调整指针*/
                        if(j>=16) j=0;

                }

                if(ret_flag!=0)   /*如果返回标志不为零:1--表明收到地面指令,结束循环;2--操作系统接收完毕*/
                        break;    
                else              /*如果返回标志为零:1--未收到地面指令;2--正在接收操作系统*/	
                {
                        if(state==0)    /*如果未接收到地面指令,发自检帧*/   
                                send_down_frame(check,0);
                }

#endif




        }/*while*/   
        return boot_flag;
}

/************************************************************/
/*       下发一帧数据
         入口参数：  id--要下发的帧类型
         recnum--已接收的数据帧的序号
         返回值：    OK--成功发送
         ERROR--发送失败
 *************************************************************/
void send_down_frame(unsigned char id,unsigned char recnum)
{
        unsigned char buf[MAX_SEND_LEN];
        int len;  

        memset(buf,0,MAX_SEND_LEN);/*作用是将已开辟内存空间buf的首MAX_SEND_LEN个字节赋值为0*/

        switch(id)
        {
                case ua:/*加头表示类型*/
                        buf[0]=0x73;
                        len = 1;
                        break;
                case rr:
                        buf[0]=(recnum<<5)|0x11;
                        len = 1;
                        break;
                case rej:
                        buf[0]=(recnum<<5)|0x19;
                        len = 1;
                        break;
                case dm:
                        buf[0]=0x1f;
                        len = 1;
                        break;
                case check:      /*自检状态*/
                        buf[0]=0x10;
                        buf[1]=0x37;
                        buf[2]=0x0F;
                        buf[3]=0x00 ;//主信道计数
                        buf[4]=0x00;//虚拟信道计数
                        buf[5]=0x18;
                        buf[6]=0x00;
                        buf[7]=0x00;//
                        buf[8]=0x10;//应用过程标识符
                        buf[9]=0x00;
                        buf[10]=0x00;//分组过程标志
                        buf[11]=0x00;
                        buf[12]=0x02;//包数据长度
                        buf[13]=0x00;//子标示字
                        buf[14]=0x02;//子标示字
                        buf[15]=c515c_state & 0xff;
                        buf[16]=ram_state & 0xff;
                        len = 17;
                        break;     	 
        }

        if (hdlc_channel == 1)
        {
                fcc1_hdlc_send_data(buf, len);
        }
        else
        {
                fcc3_hdlc_send_data(buf, len);
        }
}

void delay_m(void)
{
        int i;
        for (i = 0; i<10; i++);
}

int RAM_test(void)
{
        unsigned long writeaddr;
        int error;
        printf("mem test :\n");
        for(writeaddr=0 ; writeaddr<0xff/*0x4000000*/ ; writeaddr+=0x1)
        {		
                *(volatile unsigned char *)writeaddr = (unsigned char)(writeaddr) ;
                WATCHDOG_RESET ();
        }
        for(error=0,writeaddr=0x0 ; writeaddr<0xff/*0x4000000*/ ; writeaddr+=0x1)
        {
                if(*(volatile unsigned char *)writeaddr != (unsigned char)(writeaddr))
                {
                        error++;
                }
        }
        return error;
        printf("The end of mem test \n");
}

int RAM_test1(void) //20130521test
{
        unsigned long writeaddr;
        int error;
        printf("new mem test :\n");
        for(writeaddr=0 ; writeaddr<0xff/*0x4000000*/ ; writeaddr+=0x4)
        {		
                *(volatile unsigned char *)writeaddr = (unsigned char)(writeaddr) ;
                printf("%x = %x\n",writeaddr,(*(volatile unsigned long *)writeaddr));
                WATCHDOG_RESET ();
        }
        for(error=0,writeaddr=0x0 ; writeaddr<0xff/*0x4000000*/ ; writeaddr+=0x4)
        {
                if(*(volatile unsigned char *)writeaddr != (unsigned char)(writeaddr))
                {
                        error++;printf("error= %x\n",error);
                }
                WATCHDOG_RESET ();
        }
        printf("The end of  new mem test \n");
        return error;


}

void MEM_test(void)
{	
        unsigned long writeaddr;
        int error;
        printf("mem test 0x55:\n");
        for(writeaddr=0x0 ; writeaddr<0x0+0x300000/*0x4000000*/ ; writeaddr+=0x4)
        {		
                *(volatile unsigned long *)writeaddr = (unsigned long)(0x55555555) ;
                WATCHDOG_RESET ();
        }
        for(error=0,writeaddr=0x0 ; writeaddr<0x0+0x300000/*0x4000000*/ ; writeaddr+=0x4)
        {
                if(*(volatile unsigned long *)writeaddr != (unsigned long)(0x55555555))
                {
                        printf("%x = %x\n",writeaddr,(*(volatile unsigned long *)writeaddr));
                        error++;
                }
                WATCHDOG_RESET ();
        }
        printf("The end of test 0x55555555\n");
        for(writeaddr=0x0 ; writeaddr<0x0+0x300000/*0x4000000*/ ; writeaddr+=0x4)
        {		
                *(volatile unsigned long *)writeaddr = (unsigned long)(0xaaaaaaaa) ;
                WATCHDOG_RESET ();
        }
        for(error=0,writeaddr=0x0 ; writeaddr<0x0+0x300000/*0x4000000*/ ; writeaddr+=0x4)
        {
                if(*(volatile unsigned long *)writeaddr != (unsigned long)(0xaaaaaaaa))
                {
                        printf("%x = %x\n",writeaddr,(*(volatile unsigned long *)writeaddr));
                        error++;
                }
                WATCHDOG_RESET ();
        }

        printf("The end of test 0xaaaaaaaa\n");

        for(writeaddr=0x0 ; writeaddr<0x0+0xff/*0x4000000*/ ; writeaddr+=0x1)
        {		
                *(volatile unsigned char *)writeaddr = (unsigned char)(writeaddr) ;
        }
        for(error=0,writeaddr=0x0 ; writeaddr<0x0+0xff/*0x4000000*/ ; writeaddr+=0x1)
        {
                if(*(volatile unsigned char *)writeaddr != (unsigned char)(writeaddr))
                {
                        printf("%x = %x\n",writeaddr,(*(volatile unsigned char *)writeaddr));
                        error++;
                }
        }
        WATCHDOG_RESET ();
        printf("3 MB RAM test end.\n");
}
void DATA_test(void)
{
        unsigned long writeaddr;
        int error;
        printf("6 MB data ram test 0x55:\n");
        for(writeaddr=0x20000000 ; writeaddr<0x20000000+0x600000/*0x4000000*/ ; writeaddr+=0x4)
        {		
                *(volatile unsigned long *)writeaddr = (unsigned long)(0x55555555) ;
                WATCHDOG_RESET ();
        }
        for(error=0,writeaddr=0x20000000 ; writeaddr<0x20000000+0x600000/*0x4000000*/ ; writeaddr+=0x4)
        {
                if(*(volatile unsigned long *)writeaddr != (unsigned long)(0x55555555))
                {
                        printf("%x = %x\n",writeaddr,(*(volatile unsigned long *)writeaddr));
                        error++;
                }
                WATCHDOG_RESET ();
        }
        printf("The end of data ram test 0x55555555\n");
        for(writeaddr=0x20000000 ; writeaddr<0x20000000+0x600000/*0x4000000*/ ; writeaddr+=0x4)
        {		
                *(volatile unsigned long *)writeaddr = (unsigned long)(0xaaaaaaaa) ;
                WATCHDOG_RESET ();
        }
        for(error=0,writeaddr=0x20000000 ; writeaddr<0x20000000+0x600000/*0x4000000*/ ; writeaddr+=0x4)
        {
                if(*(volatile unsigned long *)writeaddr != (unsigned long)(0xaaaaaaaa))
                {
                        printf("%x = %x\n",writeaddr,(*(volatile unsigned long *)writeaddr));
                        error++;
                }
                WATCHDOG_RESET ();
        }

        printf("The end of data ram test 0xaaaaaaaa\n");

        for(writeaddr=0x20000000 ; writeaddr<0x20000000+0xff/*0x4000000*/ ; writeaddr+=0x1)
        {		
                *(volatile unsigned char *)writeaddr = (unsigned char)(writeaddr) ;

        }
        for(error=0,writeaddr=0x20000000 ; writeaddr<0x20000000+0xff/*0x4000000*/ ; writeaddr+=0x1)
        {
                if(*(volatile unsigned char *)writeaddr != (unsigned char)(writeaddr))
                {
                        printf("%x = %x\n",writeaddr,(*(volatile unsigned char *)writeaddr));
                        error++;
                }
        }

        WATCHDOG_RESET ();
        printf("6 MB RAM test end.\n");
}



void Can_init(void)
{
        unsigned char a;
        a = *(volatile unsigned char *)0x10007ffe;  //清双口RAM的中断；
        *(volatile unsigned char *)(0x10006200) = 0; //清va
}

int get_hdlc_channel(void)
{
        if ((*(volatile unsigned int *)(0xF0010D10) & (0x00000200)) == 0)
                return 1;
        else 
                return 3;
}
void move_eprom_ram(void)
{
        int i;

        printf("Moving application from EPROM to Temp_RAM.\n");

        //	WATCHDOG_RESET();

        //	memcpy(TEMP_ADDRESS, ROM_ADDRESS, APPLICATION_LEN);
#if 1
        for (i = 0; i<APPLICATION_LEN; i++)
        {
                if ((i&0x00000001)==0)
                {
                        *(volatile unsigned char *)(TEMP_ADDRESS+i) = *(volatile unsigned short *)(ROM_ADDRESS+i)&0x00ff;//>>8; //&0x00ff;
                } else
                        *(volatile unsigned char *)(TEMP_ADDRESS+i) = *(volatile unsigned short *)(ROM_ADDRESS+i)>>8;//&0x00ff;  //>>8;
                WATCHDOG_RESET();
        }

#endif
}
void move_pr(unsigned int n)
{
        int i;
        printf("Moving application from Temp_RAM to Kernel address.\n");
        WATCHDOG_RESET();
        memcpy(0, TEMP_ADDRESS, n);
#if 0
        for (i = 0; i<APPLICATION_LEN; i++)
                *(volatile unsigned char *)i = *(volatile unsigned char *)(TEMP_ADDRESS+i);
#endif
}

/***********************************************
  主程序
 ************************************************/

int bootmain(void)
{
        unsigned int boot_flag,k,id,len,i;

        unsigned long addr;		//DPRAM addr
        //printf("bootmain_addr = %x\n",bootmain);

        printf("This is image from low address.\n");
        ram_state=RAM_test();
        printf("RAM_state is 0x%x.\n", ram_state);
        addr=0x10006201;
        read_frame=0;
        j=0; 
        //spi_init_f();/*2013*/
        /*同步串口、CAN初始化*/
        hdlc_init();
        hdlc_channel = get_hdlc_channel();
        printf("Hdlc channel is %d.\n", hdlc_channel);
        //printf("The PDATA is 0x%x.\n", *(volatile unsigned int *)(0xF0010D10));
        Can_init();
        printf("SPI channel is %d.\n", hdlc_channel);
        //  spi_init_f();/*2013*/
        //	spi_init_r();/*2013*/
        spi_write_test (0x01);/*2013*/
        scc2_serial_init ();/*2013*/
        scc3_serial_init ();/*2013*/
        smc1_serial_init ();/*2013*/
        smc2_serial_init ();/*2013*/
        printf("SCC2_init\n");/*2013*/
        printf("SCC3_init\n");/*2013*/
        printf("SMC1_init\n");/*2013*/
        printf("SMC2_init\n");/*2013*/
#if 0
        for (i = 0; i<APPLICATION_LEN; i++)
        {
                if ((i&0x00000001)==0)
                {
                        *(volatile unsigned char *)(TEMP_ADDRESS+i) = *(volatile unsigned short *)(ROM_ADDRESS+i)&0x00ff;//>>8; //&0x00ff;
                } else
                        *(volatile unsigned char *)(TEMP_ADDRESS+i) = *(volatile unsigned short *)(ROM_ADDRESS+i)>>8;//&0x00ff;  //>>8;
                printf("%x",*(volatile unsigned char *)(TEMP_ADDRESS+i));
                if ((i+1)%16 ==0) printf("\n");
                WATCHDOG_RESET();
        }

        for (;;)
        {
                printf("0xff000000 = %x\n", *(volatile unsigned short *)(0xff000000));
                WATCHDOG_RESET();
        }
#endif

        /*测试C515C*/
        /*发送GET帧请求TTC遥测数据报*/
        id=0x01;
        len=104;
        *(volatile unsigned char *)addr = 0x01;						//va = 1
        *(volatile unsigned char *)(addr+2) = 0x12;				
        *(volatile unsigned char *)(addr+4) = 0x08;
        *(volatile unsigned char *)(addr+6) = 0x36;
        *(volatile unsigned char *)(addr+8) = 0;
        *(volatile unsigned char *)(addr+10) = id>>16;
        *(volatile unsigned char *)(addr+12) = id>>8;
        *(volatile unsigned char *)(addr+14) = id&0xff;
        *(volatile unsigned char *)(addr+16) = len>>16;
        *(volatile unsigned char *)(addr+18) = len>>8;
        *(volatile unsigned char *)(addr+20) = len&0xff;   

        /*产生中断*/
        *(volatile unsigned short *)(0x1000fffc)=0xffff;          
        for(k=0;k<500;k++)
                delay_m();

        if(*(volatile unsigned char *)addr==0x00)
                c515c_state=0;   /*正常*/
        else
        {
                printf("*addr = %x\n", *(volatile unsigned char *)addr);
                c515c_state=0xff;
        }   
        printf("C515C_state:  0x%x\n",c515c_state);

        //#if 1
#if 0
        /*将自检状态下发,直到等到地面命令*/
        boot_flag=2;
        // boot_flag=link_protocol(0);   /*调用链路层协议*/ 
        printf("boot_flag = %x\n", boot_flag);
        if(boot_flag==1)              /*从地面加载操作系统*/
        {
                link_protocol(1);           /*调用链路层协议*/
                WATCHDOG_RESET();
                for(k=0;k<5000;k++)  
                        delay_m();                 /*延时*/
                printf("start copy to 0x0, 0x100 = %x\n", *(volatile unsigned long *)(0x100));
                printf("bytetotal = %x\n", bytetotal);
                move_pr(bytetotal);

                printf("finish copy to 0x0, 0x100 = %x\n", *(volatile unsigned long *)(0x100));

        }
        else //if(boot_flag==2)         /*启动固化操作系统*/
        {
                WATCHDOG_RESET();
                for(k=0;k<5000;k++)  
                        delay_m();                 /*延时*/ 
                move_eprom_ram();
                move_pr(APPLICATION_LEN);
        }


#else

        //WATCHDOG_RESET();//20130517
        //  move_eprom_ram();//20130517
        //  move_pr(APPLICATION_LEN);                    /*搬移程序*/

#endif
        //test code 20130517
        //# if 0	//test 2017
# if 1
        printf("enter test program \n");
        printf("1:MEM test  \n");
        printf("2:Data ram test  \n");
        printf("3:can/spi  test 0  \n");
        printf("4:can/spi  test 1  \n");
        printf("5:hdlc  test   \n");
        printf("6:scc2  test   \n");	
        printf("7:scc3  test   \n");	
        printf("8:smc1  test   \n");	
        printf("9:smc2  test   \n");
        printf("a:scc2 output test   \n");
        printf("b:scc3 output test   \n");
        printf("c:smc1 output test   \n");
        printf("d:smc2 output test   \n");
        printf("e:print list   \n");
        printf("10 20170215ucos \n");
        printf("11 SPI_serial_test \n");	
        for(;;){
                int common;
                unsigned char startup;//20170213
                int scc2input,scc3input,smc1input,smc2input;
                int	retval;
                int timedata;//20130522test
                unsigned char  data[MAX_REC_LEN*16]; 
                unsigned char spibuf[14];
                // unsigned char result1[32];
                //	

                static	int	len;
                printf("while!!!  \n");
                WATCHDOG_RESET();
                //while(!tstc()){WATCHDOG_RESET();printf("while*_*  \n");};
                while(!tstc()){WATCHDOG_RESET();};
                common=serial_getc();
                startup=0;

                printf("common=%x \n",common);
                switch(common)
                {
                        case(0x01):
                                //MEM_test();
                                //RAM_test();
                                RAM_test1();
                                common=0x00;
                                //return 0;
                                break;
                        case(0x02):
                                DATA_test();
                                common=0x00;
                                //return 0;
                                break;
                        case(0x03):
                                WATCHDOG_RESET();
                                spi_write_test (0x00);
                                common=0x00;
                                //return 0;
                                break;
                        case(0x04):
                                WATCHDOG_RESET();
                                spi_write_test (0x01);
                                common=0x00;
                                //return 0;
                                break;
                        case(0x05):
                                boot_flag=link_protocol(0);   /*调用链路层协议*/ 
                                printf("boot_flag = %x\n", boot_flag);
                                common=0x00;
                                return 0;
                                break;
                        case(0x06):
                                scc2_serial_putc(0x02);/*2013*/
                                printf("scc2_serial = %x\n", 0x02);	
                                common=0x00;
                                //return 0;
                                break;
                        case(0x07):
                                scc3_serial_putc(0x03);/*2013*/
                                printf("scc3_serial = %x\n", 0x03);
                                common=0x00;	
                                //return 0;		
                                break;
                        case(0x08):
                                smc1_serial_putc(0xc1);/*2013*/
                                printf("smc1_serial = %x\n", 0xc1);
                                common=0x00;
                                //return 0;		
                                break;
                        case(0x09):
                                smc2_serial_putc(0xc2);/*2013*/
                                printf("smc2_serial = %x\n", 0xc2);
                                common=0x00;
                                //return 0;	
                                break;
                        case(0x0a):
                                /*
                                   printf("waiting scc2 input \n"); //20130522testd
                                   while ((!scc2_serial_tstc())&&(timedata<60000)) timedata++;
                                   if (timedata>=60000) printf(" scc2 input time out \n");
                                   timedata=0;
                                   if (scc2_serial_tstc()) {scc2input=scc2_serial_getc();printf("scc2_input = %x\n",scc2input);};
                                   printf("scc2_input test end \n");
                                   */
                                timedata=0;
                                printf("waiting scc2 input \n"); //20130522testd
                                while ((!scc2_serial_tstc())&&(timedata<60000)) timedata++;
                                if (timedata>=60000) {printf(" scc2 input time out \n");printf(" timedata= %x\n",timedata);};
                                timedata=0;
                                while((!scc2_serial_tstc())&&(!tstc())) WATCHDOG_RESET();
                                if (scc2_serial_tstc()) {scc2input=scc2_serial_getc();printf("scc2_input = %x\n",scc2input);};
                                printf("scc2_input test end \n");
                                //eturn 0;
                                break;
                        case(0x0b):
                                timedata=0;
                                printf("waiting scc3 input \n"); //20130522testd
                                while ((!scc3_serial_tstc())&&(timedata<60000)) timedata++;
                                if (timedata>=60000) {printf(" scc3 input time out \n");printf(" timedata= %d\n",timedata);};
                                timedata=0;
                                while((!scc3_serial_tstc())&&(!tstc())) WATCHDOG_RESET();
                                if (scc3_serial_tstc()) {scc3input=scc3_serial_getc();printf("scc3_input = %x\n",scc3input);};
                                printf("scc3_input test end \n");
                                //return 0;		
                                break;
                        case(0x0c):
                                timedata=0;
                                printf("waiting smc1 input \n"); //20130522testd
                                while ((!smc1_serial_tstc())&&(timedata<60000)) timedata++;
                                if (timedata>=60000) {printf(" smc1 input time out \n");printf(" timedata= %d\n",timedata);};
                                timedata=0;
                                while((!smc1_serial_tstc())&&(!tstc())) WATCHDOG_RESET();
                                if (smc1_serial_tstc()) {smc1input=smc1_serial_getc();printf("smc1_input = %x\n",smc1input);};
                                printf("smc1_input test end \n");
                                //return 0;		
                                break;
                        case(0x0d):
                                timedata=0;
                                printf("waiting smc2 input \n"); //20130522testd
                                while ((!smc2_serial_tstc())&&(timedata<60000)) timedata++;
                                if (timedata>=60000) {printf(" smc2 input time out \n");printf(" timedata= %d\n",timedata);};
                                timedata=0;
                                while((!smc2_serial_tstc())&&(!tstc())) WATCHDOG_RESET();
                                if (smc2_serial_tstc()) {smc2input=smc2_serial_getc();printf("smc2_input = %x\n",smc2input);};
                                printf("smc2_input test end \n");
                                //return 0;		
                                break;
                        case(0x0e):
                                printf("enter test program \n");
                                printf("1:MEM test  \n");
                                printf("2:Data ram test  \n");
                                printf("3:can/spi  test 0  \n");
                                printf("4:can/spi  test 1  \n");
                                printf("5:hdlc  test   \n");
                                printf("6:scc2  test   \n");	
                                printf("7:scc3  test   \n");	
                                printf("8:smc1  test   \n");	
                                printf("9:smc2  test   \n");
                                printf("a:scc2 output test   \n");
                                printf("b:scc3 output test   \n");
                                printf("c:smc1 output test   \n");
                                printf("d:smc2 output test   \n");
                                printf("e:print list   \n");
                                printf("10 20170220cos \n");
                                printf("11 SPI_serial_test \n");
                                //return 0;
                                break;
                        case(0x0f): 

                                //	show_regs(0x00000000);		
                                //asm ("li 1,0");	
                                //	asm ("li ,"r"(0),0x55");			
                                //asm("eieio\n;");
                                //asm("lfd 0,0(3)\n\t");
                                //asm volatile("mtspr 0x150,%0"::"r"(0));
                                //	asm volatile("addi 4,0,14":::"r4");
                                //	asm volatile("li %r0,0x5555");			
                                //	asm volatile("add %r0,%r1,%r2");
                                //	asm volatile("li %r0,0x5555");
                                //	asm volatile("li %r1,0x1000");
                                //	asm volatile("divw %r0,%r0,%r1");
                                //asm volatile("li %1,%0":"=r"(result):"r"(input));
                                //asm volatile("li %r1,0x2");
                                //asm volatile("divw %r0,%r0,%r1");
                                //asm("lis %f0,0x5555");
                                //asm("lis %f1,0x5555");
                                //asm("fdiv. %f0,%f1,%f0 ");
                                /*20130927*/
                                show_regs(0);
                                //show_regs(0x002afc20);//20131224
                                printf("enter:GPR program   \n");				
                                printf("result= %x \n",result);	

                                printf("enter:GPR program 0  \n");
                                asm("lis 0,0x5555");
                                asm("lis 1,0x2");
                                asm("add %0,0,1":"=r"(result));		
                                printf("result= %x \n",result);	

                                printf("enter:GPR0 program 1-32  \n");		
                                //asm("lis 0,0x5555");
                                asm("lis 0,0x5555":::"r0");//20131224
                                asm("lis 1,0x2");		
                                asm("mulhw 0,0,1");
                                //asm("lis 1,0xaaaa");//20131224
                                asm("add %0,0,1":"=r"(result));		
                                printf("result GPR0= %x \n",result);

                                asm("add %0,1,0":"=r"(result));		
                                printf("result GPR1= %x \n",result);

                                asm("lis 2,0x5555");
                                asm("lis 3,0x2");		
                                asm("mulhw 2,2,3");
                                //asm("lis 3,0xaaaa");//20131224		
                                asm("add %0,2,3":"=r"(result));		
                                printf("result GPR2= %x \n",result);

                                asm("lis 3,0x5555");
                                asm("lis 4,0x2");		
                                asm("mulhw 3,3,4");
                                //asm("lis 4,0xaaaa");//20131224		
                                asm("add %0,3,4":"=r"(result));		
                                printf("result GPR3= %x \n",result);

                                asm("lis 4,0x5555");
                                asm("lis 5,0x2");		
                                asm("mulhw 4,4,5");
                                //asm("lis 5,0xaaaa");//20131224		
                                asm("add %0,4,5":"=r"(result));		
                                printf("result GPR4= %x \n",result);

                                asm("lis 5,0x5555");
                                asm("lis 6,0x2");		
                                asm("mulhw 5,5,6");
                                //asm("lis 6,0xaaaa");//20131224		
                                asm("add %0,5,6":"=r"(result));		
                                printf("result GPR5= %x \n",result);

                                asm("lis 6,0x5555");
                                asm("lis 7,0x2");		
                                asm("mulhw 6,6,7");
                                //asm("lis 7,0xaaaa");//20131224		
                                asm("add %0,6,7":"=r"(result));		
                                printf("result GPR6= %x \n",result);

                                asm("lis 7,0x5555");
                                asm("lis 8,0x2");		
                                asm("mulhw 7,7,8");
                                //asm("lis 8,0xaaaa");//20131224		
                                asm("add %0,7,8":"=r"(result));		
                                printf("result GPR7= %x \n",result);

                                asm("lis 8,0x5555");
                                asm("lis 9,0x2");		
                                asm("mulhw 8,8,9");	
                                //asm("lis 9,0xaaaa");//20131224	
                                asm("add %0,8,9":"=r"(result));		
                                printf("result GPR8= %x \n",result);

                                asm("lis 9,0x5555");
                                asm("lis 10,0x2");		
                                asm("mulhw 9,9,10");
                                //asm("lis 10,0xaaaa");//20131224		
                                asm("add %0,9,10":"=r"(result));		
                                printf("result GPR9= %x \n",result);

                                asm("lis 10,0x5555");
                                asm("lis 11,0x2");		
                                asm("mulhw 10,10,11");	
                                asm("lis 11 ,0xaaaa");//20131224	
                                //asm("add %0,10,11":"=r"(result));		
                                printf("result GPR10= %x \n",result);

                                asm("lis 11,0x5555");
                                asm("lis 12,0x2");		
                                asm("mulhw 11,11,12");
                                //asm("lis 12 ,0xaaaa");//20131224		
                                asm("add %0,11,12":"=r"(result));		
                                printf("result GPR11= %x \n",result);

                                asm("lis 12,0x5555");
                                asm("lis 13,0x2");		
                                asm("mulhw 12,12,13");
                                //asm("lis 13,0xaaaa");//20131224		
                                asm("add %0,12,13":"=r"(result));		
                                printf("result GPR12= %x \n",result);

                                asm("lis 13,0x5555");
                                asm("lis 14,0x2");		
                                asm("mulhw 13,13,14");
                                //asm("lis 14,0xaaaa");//20131224		
                                asm("add %0,13,14":"=r"(result));		
                                printf("result GPR13= %x \n",result);

                                asm("lis 14,0x5555");
                                asm("lis 15,0x2");		
                                asm("mulhw 14,14,15");
                                //asm("lis 15,0xaaaa");//20131224		
                                asm("add %0,14,15":"=r"(result));		
                                printf("result GPR14= %x \n",result);

                                asm("lis 15,0x5555");
                                asm("lis 16,0x2");		
                                asm("mulhw 15,15,16");	
                                //asm("lis 16,0xaaaa");//20131224	
                                asm("add %0,15,16":"=r"(result));		
                                printf("result GPR15= %x \n",result);

                                asm("lis 16,0x5555");
                                asm("lis 17,0x2");		
                                asm("mulhw 16,16,17");
                                //asm("lis 17,0xaaaa");//20131224		
                                asm("add %0,16,17":"=r"(result));		
                                printf("result GPR16= %x \n",result);

                                asm("lis 17,0x5555");
                                asm("lis 18,0x2");		
                                asm("mulhw 17,17,18");	
                                //asm("lis 18,0xaaaa");//20131224	
                                asm("add %0,17,18":"=r"(result));		
                                printf("result GPR17= %x \n",result);

                                asm("lis 18,0x5555");
                                asm("lis 19,0x2");		
                                asm("mulhw 18,18,19");
                                //asm("lis 19,0xaaaa");//20131224		
                                asm("add %0,18,19":"=r"(result));		
                                printf("result GPR18= %x \n",result);

                                asm("lis 19,0x5555");
                                asm("lis 20,0x2");		
                                asm("mulhw 19,19,20");
                                //asm("lis 20,0xaaaa");//20131224		
                                asm("add %0,19,20":"=r"(result));		
                                printf("result GPR19= %x \n",result);

                                asm("lis 20,0x5555");
                                asm("lis 21,0x2");		
                                asm("mulhw 20,20,21");
                                //asm("lis 21,0xaaaa");//20131224		
                                asm("add %0,20,21":"=r"(result));		
                                printf("result GPR20= %x \n",result);

                                asm("lis 21,0x5555");
                                asm("lis 22,0x2");		
                                asm("mulhw 21,21,22");
                                //asm("lis 22,0xaaaa");//20131224		
                                asm("add %0,21,22":"=r"(result));		
                                printf("result GPR21= %x \n",result);

                                asm("lis 22,0x5555");
                                asm("lis 23,0x2");		
                                asm("mulhw 22,22,23");
                                //asm("lis 23,0xaaaa");//20131224	
                                asm("add %0,22,23":"=r"(result));		
                                printf("result GPR22= %x \n",result);

                                asm("lis 23,0x5555");
                                asm("lis 24,0x2");		
                                asm("mulhw 23,23,24");
                                //asm("lis 24,0xaaaa");//20131224		
                                asm("add %0,23,24":"=r"(result));		
                                printf("result GPR23= %x \n",result);

                                asm("lis 24,0x5555");
                                asm("lis 25,0x2");		
                                asm("mulhw 24,24,25");
                                //asm("lis 25,0xaaaa");//201231224		
                                asm("add %0,24,25":"=r"(result));		
                                printf("result GPR24= %x \n",result);

                                asm("lis 25,0x5555");
                                asm("lis 26,0x2");		
                                asm("mulhw 25,25,26");
                                //asm("lis 26,0xaaaa");//20131224		
                                asm("add %0,25,26":"=r"(result));		
                                printf("result GPR25= %x \n",result);

                                asm("lis 26,0x5555");
                                asm("lis 27,0x2");		
                                asm("mulhw 26,26,27");
                                //asm("lis 27,0xaaaa");//20131224		
                                asm("add %0,26,27":"=r"(result));		
                                printf("result GPR26= %x \n",result);

                                asm("lis 27,0x5555");
                                asm("lis 28,0x2");		
                                asm("mulhw 27,27,28");
                                //asm("lis 28,0xaaaa");//20131224		
                                asm("add %0,27,28":"=r"(result));		
                                printf("result GPR27= %x \n",result);

                                asm("lis 28,0x5555");
                                asm("lis 29,0x2");		
                                asm("mulhw 28,28,29");
                                //asm("lis 29,0xaaaa");//20131224		
                                asm("add %0,28,29":"=r"(result));		
                                printf("result GPR28= %x \n",result);

                                //asm("lis 29,0x5555");
                                //asm("lis 30,0x2");		
                                //asm("mulhw 29,29,30");		
                                asm("add %0,29,30":"=r"(result));		
                                printf("result GPR29= %x \n",result);

                                //asm("lis 30,0x5555");
                                //asm("lis 31,0x2");		
                                //asm("mulhw 30,30,31");		
                                asm("add %0,30,31":"=r"(result));		
                                printf("result GPR30= %x \n",result);

                                //asm("lis 31,0x5555");
                                //asm("lis 0,0x2");		
                                //asm("mulhw 31,31,0");		
                                asm("add %0,31,0":"=r"(result));		
                                printf("result GPR31= %x \n",result);

                                //show_regs(0x002afc20);

                                asm("lis %f0,0x5555");
                                asm("lis %f1,0x2");
                                //asm("fmul %f0,0,1");
                                asm("fmul %f0,%f0,%f1");
                                asm("fadd %0,0,1":"=r"(result));
                                printf("result FLOAT FPR0= %x \n",result);


                                //asm("lis %f0,0x3333");
                                asm("lis %f0,0x5555");
                                //asm("lis %f1,0x2");
                                asm("lis %f1,0x22");
                                asm("mulhw. %f0,%f0,%f1");
                                asm("fadd %0,0,1":"=r"(result));
                                printf("result  FPR0= %x \n",result);


                                asm("lis %f2,0x5555");
                                asm("lis %f3,0x2");
                                asm("mulhwu. %f0,%f2,%f3");				
                                //asm("fadd %0,2,3":"=r"(result));//201231224
                                asm("fadd %0,0,2":"=r"(result));//20131224
                                printf("result FPR2= %x \n",result);

                                asm("lis %f3,0x5555");
                                asm("lis %f4,0x2");
                                asm("mulhw %f0,%f3,%f4");
                                //asm("fadd %0,3,4":"=r"(result));
                                asm("fadd %0,0,3":"=r"(result));//20131224
                                printf("result FPR3= %x \n",result);

                                //asm("lis %f4,0x5555");
                                asm("lis %f4,0x4444");
                                asm("lis %f5,0x2");
                                asm("mulhw %f0,%f4,%f5");
                                //asm("fadd %0,4,5":"=r"(result));
                                asm("fadd %0,0,4":"=r"(result));//20131224
                                printf("result FPR4= %x \n",result);

                                asm("lis %f5,0x5555");//0x0005
                                asm("lis %f6,0x2");
                                asm("mulhw %f0,%f5,%f6");
                                asm("fadd %0,5,6":"=r"(result));
                                printf("result FPR5= %x \n",result);

                                asm("lis %f6,0x5555");//0x0006
                                asm("lis %f7,0x2");
                                asm("mulhw %f0,%f6,%f7");
                                asm("fadd %0,6,7":"=r"(result));
                                printf("result FPR6= %x \n",result);

                                asm("lis %f7,0x5555");//0x0007
                                asm("lis %f8,0x2");
                                asm("mulhw %f0,%f7,%f8");
                                asm("fadd %0,7,8":"=r"(result));
                                printf("result FPR7= %x \n",result);

                                asm("lis %f8,0x5555");//0x0008
                                asm("lis %f9,0x2");
                                asm("mulhw %f0,%f8,%f9");
                                asm("fadd %0,8,9":"=r"(result));
                                printf("result FPR8= %x \n",result);

                                asm("lis %f9,0x5555");//0x0009
                                asm("lis %f10,0x2");
                                asm("mulhw %f0,%f9,%f10");
                                asm("fadd %0,9,10":"=r"(result));
                                printf("result FPR9= %x \n",result);

                                asm("lis %f10,0x5555");//0x000a
                                asm("lis %f11,0x2");
                                asm("mulhw %f0,%f10,%f11");
                                asm("fadd %0,10,11":"=r"(result));
                                printf("result FPR10= %x \n",result);

                                asm("lis %f11,0x5555");//0x000b
                                asm("lis %f12,0x2");
                                asm("mulhw %f0,%f11,%f12");
                                asm("fadd %0,11,12":"=r"(result));
                                printf("result FPR11= %x \n",result);

                                asm("lis %f12,0x5555");//0x000c
                                asm("lis %f13,0x2");
                                asm("mulhw %f0,%f12,%f13");
                                asm("fadd %0,12,13":"=r"(result));
                                printf("result FPR12= %x \n",result);

                                asm("lis %f13,0x5555");//0x000d
                                asm("lis %f14,0x2");
                                asm("mulhw %f0,%f13,%f14");
                                asm("fadd %0,13,14":"=r"(result));
                                printf("result FPR13= %x \n",result);

                                asm("lis %f14,0x5555");//0x000e
                                asm("lis %f15,0x2");
                                asm("mulhw %f0,%f14,%f15");
                                asm("fadd %0,14,15":"=r"(result));
                                printf("result FPR14= %x \n",result);

                                asm("lis %f15,0x5555");//0x000f
                                asm("lis %f16,0x2");
                                asm("mulhw %f0,%f15,%f16");
                                asm("fadd %0,15,16":"=r"(result));
                                printf("result FPR15= %x \n",result);

                                asm("lis %f16,0x5555");//0x0010
                                asm("lis %f17,0x2");
                                asm("mulhw %f0,%f16,%f17");
                                asm("fadd %0,16,17":"=r"(result));
                                printf("result FPR16= %x \n",result);

                                asm("lis %f17,0x5555");//0x0011
                                asm("lis %f18,0x2");
                                asm("mulhw %f0,%f17,%f18");
                                asm("fadd %0,17,18":"=r"(result));
                                printf("result FPR17= %x \n",result);

                                asm("lis %f18,0x5555");//0x0012
                                asm("lis %f19,0x2");
                                asm("mulhw %f0,%f18,%f19");
                                asm("fadd %0,18,19":"=r"(result));
                                printf("result FPR18= %x \n",result);

                                asm("lis %f19,0x5555");//0x0013
                                asm("lis %f20,0x2");
                                asm("mulhw %f0,%f19,%f20");
                                asm("fadd %0,19,20":"=r"(result));
                                printf("result FPR19= %x \n",result);

                                asm("lis %f20,0x5555");//0x0014
                                asm("lis %f21,0x2");
                                asm("mulhw %f0,%f20,%f21");
                                asm("fadd %0,20,21":"=r"(result));
                                printf("result FPR20= %x \n",result);

                                asm("lis %f21,0x5555");//0x0015
                                asm("lis %f22,0x2");
                                asm("mulhw %f0,%f21,%f22");
                                asm("fadd %0,21,22":"=r"(result));
                                printf("result FPR21= %x \n",result);

                                asm("lis %f22,0x5555");//0x0016
                                asm("lis %f23,0x2");
                                asm("mulhw %f0,%f22,%f23");
                                asm("fadd %0,22,23":"=r"(result));
                                printf("result FPR22= %x \n",result);

                                asm("lis %f23,0x5555");//0x0017
                                asm("lis %f24,0x2");
                                asm("mulhw %f0,%f23,%f24");
                                asm("fadd %0,23,24":"=r"(result));
                                printf("result FPR23= %x \n",result);

                                asm("lis %f24,0x5555");//0x0018
                                asm("lis %f25,0x2");
                                asm("mulhw %f0,%f24,%f25");
                                asm("fadd %0,24,25":"=r"(result));
                                printf("result FPR24= %x \n",result);

                                asm("lis %f25,0x5555");//0x0019
                                asm("lis %f26,0x2");
                                asm("mulhw %f0,%f25,%f26");
                                asm("fadd %0,25,26":"=r"(result));
                                printf("result FPR25= %x \n",result);

                                asm("lis %f26,0x5555");//0x001a
                                asm("lis %f27,0x2");
                                asm("mulhw %f0,%f26,%f27");
                                asm("fadd %0,26,27":"=r"(result));
                                printf("result FPR26= %x \n",result);

                                asm("lis %f27,0x5555");//0x001b
                                asm("lis %f28,0x2");
                                asm("mulhw %f0,%f27,%f28");
                                asm("fadd %0,27,28":"=r"(result));
                                printf("result FPR27= %x \n",result);

                                asm("lis %f28,0x001c");//0x001c
                                asm("lis %f29,0x2");
                                asm("mulhw %f0,%f28,%f29");
                                asm("fadd %0,28,29":"=r"(result));
                                printf("result FPR28= %x \n",result);

                                asm("lis %f29,0x001d");//0x001d
                                //asm("lis %f30,0x2");
                                //asm("mulhw %f0,%f29,%f30");
                                asm("fadd %0,29,30":"=r"(result));
                                printf("result FPR29= %x \n",result);

                                //asm("lis %f30,0x001e");
                                //asm("lis %f31,0x2");
                                //asm("mulhw %f0,%f30,%f31");
                                asm("fadd %0,30,31":"=r"(result));
                                printf("result FPR30= %x \n",result);

                                asm("lis %f31,0x5555");//0x001f
                                asm("lis %f0,0x2");
                                asm("mulhw %f0,%f31,%f0");
                                asm("fadd %0,31,0":"=r"(result));
                                printf("result FPR31= %x \n",result);



                                printf("enter:GPR program 2  \n");
                                asm("lis 2,0x5555");
                                asm("lis 3,0x2");
                                //asm("add %0,2,3":"=r"(result1[0]));		
                                asm("mulhw %0,2,3":"=r"(result1[0]));
                                printf("result0= %x \n",result1[0]);
                                asm("lis 2,0x5555");
                                asm("lis 3,0x2");
                                //asm("mullw %0,2,3":"=r"(result1[1]));
                                asm("mulhw %0,2,3":"=r"(result1[1]));		
                                printf("result1= %x \n",result1[1]);
                                result1[1]=result1[1]*65536;
                                result1[0]+=result1[1];
                                printf("final_result= %x \n",result1[0]);

                                printf("enter:GPR program 3  \n");
                                //return 0;
                                break;	
                        case(0x10): 
                                WATCHDOG_RESET();
                                printf("OS Starting from ucos\n");
                                for(k=0;k<5000;k++)  delay_m(); /*延时*/ 
                                delay_m();                 /*延时*/ 
                                move_eprom_ram();
                                move_pr(APPLICATION_LEN); 		
                                //printf("OS Starting from ucos\n");
                                //for(;;);
                                //startup=1;
                                return 1;
                                break;	
                        case(0x11): 
                                WATCHDOG_RESET();
                                spibuf[0]=spi_write_test (0xeb);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[1]=spi_write_test (0x90);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[2]=spi_write_test (0x03);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[3]=spi_write_test (0x04);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[4]=spi_write_test (0x05);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[5]=spi_write_test (0x06);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[6]=spi_write_test (0x07);
                                //for(k=0;k<5000;k++) delay_m();

                                spibuf[7]=spi_write_test (0x08);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[8]=spi_write_test (0x09);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[9]=spi_write_test (0x0a);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[10]=spi_write_test (0x0b);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[11]=spi_write_test (0x0c);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[12]=spi_write_test (0x0d);
                                //for(k=0;k<5000;k++) delay_m();
                                spibuf[13]=spi_write_test (0x0e);
                                //for(k=0;k<5000;k++) delay_m();
                                printf("rxbuf[0]=%x\n",spibuf[0]);
                                printf("rxbuf[1]=%x\n",spibuf[1]);
                                printf("rxbuf[2]=%x\n",spibuf[2]);
                                printf("rxbuf[3]=%x\n",spibuf[3]);
                                printf("rxbuf[4]=%x\n",spibuf[4]);
                                printf("rxbuf[5]=%x\n",spibuf[5]);
                                printf("rxbuf[6]=%x\n",spibuf[6]);
                                printf("rxbuf[7]=%x\n",spibuf[7]);
                                printf("rxbuf[8]=%x\n",spibuf[8]);
                                printf("rxbuf[9]=%x\n",spibuf[9]);
                                printf("rxbuf[10]=%x\n",spibuf[10]);
                                printf("rxbuf[11]=%x\n",spibuf[11]);
                                printf("rxbuf[12]=%x\n",spibuf[12]);
                                printf("rxbuf[13]=%x\n",spibuf[13]);
                                common=0x00;
                                //return 0;
                                break;                 	
                        default: 
                                //return 0;
                                break;
                }

                /*	printf("waiting scc2 input \n"); //20130522testd
                        while ((!scc2_serial_tstc())&&(timedata<1000)) timedata++;
                        if (timedata>=1000) printf(" scc2 input time out \n");
                        timedata=0;
                        if (scc2_serial_tstc()) scc2input=scc2_serial_getc();
                        printf("scc2_input = %x\n",scc2input );*/
                /*	scc2input=scc2_serial_getc();
                        if(scc2input !=0)
                        {printf("scc2_input = %x\n",scc2input );
                        scc2input=0;
                        }
                        scc3input=scc3_serial_getc();
                        if(scc3input !=0)
                        {printf("scc3_input = %x\n",scc3input );
                        scc3input=0;
                        }
                        smc1input=smc1_serial_getc();
                        if(smc1input !=0)
                        {printf("smc1_input = %x\n",smc1input );
                        smc1input=0;
                        }
                        smc2input=smc2_serial_getc();
                        if(smc2input !=0)
                        {printf("smc2_input = %x\n",smc2input );
                        smc2input=0;
                        }
                        hdlc_channel = get_hdlc_channel();
                        WATCHDOG_RESET();
                        if (hdlc_channel == 1) 
                        {
                        printf("fcc1 send_receive\n");
                //hdlc_app_rx0(&data[MAX_REC_LEN*j], &len);
                link_protocol(1);
                }
                else
                {
                printf("fcc3 send_receive\n");


                }*/
        }

#else
        //WATCHDOG_RESET();
        //  for(k=0;k<5000;k++)  
        //    delay_m();                 /*延时*/ 
        //   move_eprom_ram();
        //  move_pr(APPLICATION_LEN);                  /*搬移程序*/


#endif

        // return (0+startup);
        // return 0;
        // return 1;//20130517
}
