/*******************************************************

  interface.c

  defines the interface of the drivers to the applications

  coded by Meng Li

  includes the interfaces:

ADCS:	CAN	Serial
TTC:	CAN	Serial
EIS:	422
ST:	  422
GPS:	Serial
MT:	  Serial
RF0:	HDLC	Serial
RF1:	HDLC	Serial
IRQ:	DPRAM_IRQ TIMER_SEC_IRQ

 *********************************************************/

#include "includes.h"
#include "Includes.h"


#define OSQFlush_i OSQFlush
#define OSQPost_i OSQPost
#define OSSemPost_i OSSemPost
#define OSMboxPost_i OSMboxPost


//extern OS_EVENT *RecMsg;
//extern OS_EVENT *sendMsg;    
//extern OS_EVENT *sendRecMsg;
//extern OS_EVENT *HdlcQEvent;
//extern OS_EVENT *sendQEvent;
//extern OS_EVENT *timeoutQEvent;
//extern OS_EVENT *sendMessageQ;
extern OS_EVENT *CAN_Q_REC_LOCK;	
extern OS_EVENT *CAN_Q_Rec;					//姿控数据报接收标志
extern OS_EVENT *CAN_Q_Rec2;
extern OS_EVENT *CAN_Q_Rec_DLL;
extern OS_EVENT *CAN_Q_GPS;					//GPS GET帧接收标志
extern OS_EVENT *CAN_Q_GPSBAD;			//GPS的故障诊断帧接收标志
extern OS_EVENT *CAN_Q_REC_GPS;			//GPS数据报接收标志
extern OS_EVENT *CAN_Q_REC_EIS;			//EIS数据报接收标志
extern OS_EVENT *CAN_Q_REC_TTC;			//TTC数据报接收标志

extern OS_EVENT *RS422_Q_Rec;				//422数据报接收标志
extern OS_EVENT *CAN_Q_Rec_RF1;
extern OS_EVENT *CAN_Q_Rec_MIMU;	
extern OS_EVENT *SHARE_PRINTF;//共享控制
extern OS_EVENT *SHARE_SENDDOWN;
extern INT8U ERR_SHARE_PRINTF;
extern INT8U ERR_SHARE_SENDDOWN;
//extern OS_EVENT *Hdlc_Flag;  /*同步消息到来标志*/
extern OS_EVENT *ctSem;      /*时钟中断标志*/
extern OS_EVENT *TTC_Flag;		//WOD包任务标志
//extern OS_EVENT *data_ok_sem;
//extern OS_EVENT *CAN_SEM_Message;   /*can消息到来标志*/
//extern OS_EVENT *ADCS_FLAG;
//extern OS_EVENT *ACS_Q_COM;  /*给姿控的间接遥控命令*/

//INT32U  MAX_TICK=0xffffffff;    /*最大的tick数*/
//struct frame_node *head_node;
//struct comm_node *c_head;
//INT16U stop_id=0xff;
//INT16U com_flag=0xff;
//INT32U wod_begin;
//INT16U eis_week;
//double eis_second;
extern INT32U gps_week;
extern INT32U gps_second;
extern INT32U wod_cycle;   /*wod存储周期*/
//extern INT16U eis_transf_flag;

extern INT8U LockTTCFlag;
extern INT8U WODFlag;

INT8U MWFlag = 0;

void delay()
{
	INT32U i;
	for (i=0; i<10; i++);
}

//change the switch of the mux Serial
void changemux(INT32U a)
{
	switch (a)
	{
		case 0:	//printf("BD=%x\n", *(volatile INT32U *)(0xF0010D10));		//CJ	704BD
			*(volatile INT32U *)(0xF0010D10) &= 0xffffff0f; 
			*(volatile INT32U *)(0xF0010D10) |= 00000000;
			//printf("%x\n", *(volatile INT32U *)(0xF0010D10));
			break;
		case 2:	//printf("GPS=%x\n", *(volatile INT32U *)(0xF0010D10));		//GPS
			*(volatile INT32U *)(0xF0010D10) &= 0xffffff5f; 
			*(volatile INT32U *)(0xF0010D10) |= 0x00000050;
			//printf("%x\n", *(volatile INT32U *)(0xF0010D10)); 
			break;

		case 1:	//printf("ADCS=%x\n", *(volatile INT32U *)(0xF0010D10));	//ADCS
			*(volatile INT32U *)(0xF0010D10) &= 0xffffffaf; 
			*(volatile INT32U *)(0xF0010D10) |= 0x000000a0;
			//printf("%x\n", *(volatile INT32U *)(0xF0010D10));
			break;

		case 3:	//printf("RT=%x\n", *(volatile INT32U *)(0xF0010D10));		//MT	RF
			*(volatile INT32U *)(0xF0010D10) &= 0xffffffff; 
			*(volatile INT32U *)(0xF0010D10) |= 0x000000f0;
			//printf("%x\n", *(volatile INT32U *)(0xF0010D10)); 
			break;

		default:	break;
	}
}


char adcs_checksum(char *m, int len, char adcs_check) 	/*计算校验和*/
{
	INT32U sum,k;
	INT8U tempsum;
	sum=0;
	for (k=0;k<len;k++)
	{
		sum=sum+*m;
		m++;
	}
	tempsum=sum%256;
	printf("tempsum= %x\n",tempsum);
	if (tempsum==adcs_check) 
	{
		return 0;
	} 
	else 
	{
		return 1;	
	}
}
#if 1

	INT32U 
adcs_serial_test(INT32U adcs_flag_type,INT32U adcs_len1, INT8U *adcs_rxdata,INT32U adcs_len,INT8U *adcs_data)
{
	INT32U i,j,k,err,ticks1,ticks2;
	INT8U adcs_data1[60];
	INT8U adcs_temp1[10];
	INT8U adcs_temp_data[50];
	INT32U m,n,adcs_flag,Timer,err_num;
	INT8U adcs_temp_data_back[100];
	INT8U tempdata[10];
	adcs_flag=0;
	Timer=0;
	err_num=0;
	changemux(1);
	err=0;
	adcs_data1[0]=0xE9;				/*加起始场*/
	for (i=0;i<adcs_len1;i++)
	{
		adcs_data1[i+1]=adcs_rxdata[i];  /*将上层adcs传送给obc的说据从adcs_rxdata[i]中读出*/
	}
	adcs_data1[adcs_len1+1]=0xE9;			/*加终止场*/
	while (err<3)
	{
		for (j=0;j<adcs_len1+2;j++)		/*向下层adcs输出完整一帧数据*/
		{
			scc3_serial_putc(adcs_data1[j]);
		}
		//printf("DEBUG 1#\n");
		for (k=0;k<11;k++)			/*接收下层adcs返回数据*/
		{
			adcs_temp1[k]=scc3_serial_getc();
		}



		//printf("DEBUG 4#\n");
		//ticks1=OSTimeGet();
		if (adcs_temp1[1]!=0xAA) {err++;}	/*若下层返回BB,则重发三次*/
		else break;
	}

#if 1


#endif

	//ticks2=OSTimeGet();
	//printf("ticks1=%d\n",ticks1);
	//printf("ticks2=%d\n",ticks2);
	//printf("adcs_temp1=%x %x %x %x %x %x %x %x %x %x %x\n",adcs_temp1[0],adcs_temp1[1],adcs_temp1[2],adcs_temp1[3],adcs_temp1[4],adcs_temp1[5],adcs_temp1[6],adcs_temp1[7],adcs_temp1[8],adcs_temp1[9],adcs_temp1[10]);

#if 0
	for (i=0;i<13;i++)
	{		
		adcs_temp_data[i]=scc3_serial_getc();		
	}

#endif
	//printf("good here!\n");

	if(adcs_flag_type==2)			//如果是遥测指令
	{

		//printf("2 a!\n");

		for (m=0;m<adcs_len+4;m++)
		{		
			adcs_temp_data_back[m]=scc3_serial_getc();		
		} 

#if 1
		printf("adcs_temp_data_back= %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",adcs_temp_data_back[0],adcs_temp_data_back[1],adcs_temp_data_back[2],adcs_temp_data_back[3],adcs_temp_data_back[4],adcs_temp_data_back[5],adcs_temp_data_back[6],adcs_temp_data_back[7],adcs_temp_data_back[8],adcs_temp_data_back[9],adcs_temp_data_back[10],adcs_temp_data_back[11],adcs_temp_data_back[12],adcs_temp_data_back[13],adcs_temp_data_back[14],adcs_temp_data_back[15],adcs_temp_data_back[16],adcs_temp_data_back[17],adcs_temp_data_back[18],adcs_temp_data_back[19],adcs_temp_data_back[20],adcs_temp_data_back[21],adcs_temp_data_back[22],adcs_temp_data_back[23],adcs_temp_data_back[24],adcs_temp_data_back[25],
				adcs_temp_data_back[26],adcs_temp_data_back[27],adcs_temp_data_back[28],adcs_temp_data_back[29],adcs_temp_data_back[30],adcs_temp_data_back[31],adcs_temp_data_back[32],adcs_temp_data_back[33],adcs_temp_data_back[34],adcs_temp_data_back[35],adcs_temp_data_back[36],adcs_temp_data_back[37],adcs_temp_data_back[38],adcs_temp_data_back[39],adcs_temp_data_back[40],adcs_temp_data_back[41],adcs_temp_data_back[42],adcs_temp_data_back[43],adcs_temp_data_back[44],adcs_temp_data_back[45],adcs_temp_data_back[46],adcs_temp_data_back[47],adcs_temp_data_back[48],adcs_temp_data_back[49],adcs_temp_data_back[50],adcs_temp_data_back[51],adcs_temp_data_back[52]);
#endif

		/*判断校验和若不相等,存BB至临时数组,并读取终止场*/
		if ((adcs_temp_data_back[0]!=0xCC)||(adcs_checksum(&adcs_temp_data_back[2], adcs_len, adcs_temp_data_back[adcs_len+2])!=0)||(adcs_temp_data_back[adcs_len+3]!=0xCC))   
		{	
			tempdata[0]=0xE9;
			tempdata[1]=0xBB;
			tempdata[2]=0xBB;
			tempdata[3]=0xBB;
			tempdata[4]=0xBB;
			tempdata[5]=0xBB;
			tempdata[6]=0xBB;
			tempdata[7]=0xBB;
			tempdata[8]=0xBB;
			tempdata[9]=0xBB;
			tempdata[10]=0xE9;
			//printf("debug111!!!!\n");
			adcs_flag=1;
			err_num=err_num+1;
		} 
		else				/*若相等,则存AA至临时数组*/
		{	
			tempdata[0]=0xE9;
			tempdata[1]=0xAA;
			tempdata[2]=0xAA;
			tempdata[3]=0xAA;
			tempdata[4]=0xAA;
			tempdata[5]=0xAA;
			tempdata[6]=0xAA;
			tempdata[7]=0xAA;
			tempdata[8]=0xAA;
			tempdata[9]=0xAA;
			tempdata[10]=0xE9;
			for (m=2;m<adcs_len+2;m++)
			{
				adcs_data[m-2]=adcs_temp_data_back[m];

			}


		}

		for(n=0;n<11;n++)		/*输出AA或者BB*/
		{
			scc3_serial_putc(tempdata[n]);
		}

#if 1
		/**************开始第二次和第三次接收并判断*******************/

		while(adcs_flag==1 && err_num<3)  		//adcs_flag,若返回遥测数据正确,则为0,错误,则为1
		{
			for (m=0;m<adcs_len+4;m++)
			{
#if 0			
				Timer=0;
				while(scc3_serial_tstc() ==0 && Timer<1000000)
				{	
					Timer++;	
					printf("Timer=%d\n",Timer);
				}
#endif
				adcs_temp_data_back[m]=scc3_serial_getc();		
			} 

#if 1
			printf("adcs_temp_data_back= %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",adcs_temp_data_back[0],adcs_temp_data_back[1],adcs_temp_data_back[2],adcs_temp_data_back[3],adcs_temp_data_back[4],adcs_temp_data_back[5],adcs_temp_data_back[6],adcs_temp_data_back[7],adcs_temp_data_back[8],adcs_temp_data_back[9],adcs_temp_data_back[10],adcs_temp_data_back[11],adcs_temp_data_back[12],adcs_temp_data_back[13],adcs_temp_data_back[14],adcs_temp_data_back[15],adcs_temp_data_back[16],adcs_temp_data_back[17],adcs_temp_data_back[18],adcs_temp_data_back[19],adcs_temp_data_back[20],adcs_temp_data_back[21],adcs_temp_data_back[22],adcs_temp_data_back[23],adcs_temp_data_back[24],adcs_temp_data_back[25],
					adcs_temp_data_back[26],adcs_temp_data_back[27],adcs_temp_data_back[28],adcs_temp_data_back[29],adcs_temp_data_back[30],adcs_temp_data_back[31],adcs_temp_data_back[32],adcs_temp_data_back[33],adcs_temp_data_back[34],adcs_temp_data_back[35],adcs_temp_data_back[36],adcs_temp_data_back[37],adcs_temp_data_back[38],adcs_temp_data_back[39],adcs_temp_data_back[40],adcs_temp_data_back[41],adcs_temp_data_back[42],adcs_temp_data_back[43],adcs_temp_data_back[44],adcs_temp_data_back[45],adcs_temp_data_back[46],adcs_temp_data_back[47],adcs_temp_data_back[48],adcs_temp_data_back[49],adcs_temp_data_back[50],adcs_temp_data_back[51],adcs_temp_data_back[52]);
#endif

			printf("only one!!\n");
			if ((adcs_temp_data_back[0]!=0xCC)||(adcs_checksum(&adcs_temp_data_back[2], adcs_len, adcs_temp_data_back[adcs_len+2])!=0)||(adcs_temp_data_back[adcs_len+3]!=0xCC))   /*判断校验和若不相等,存BB至临时数组,并读取终止场*/
			{	
				tempdata[0]=0xE9;
				tempdata[1]=0xBB;
				tempdata[2]=0xBB;
				tempdata[3]=0xBB;
				tempdata[4]=0xBB;
				tempdata[5]=0xBB;
				tempdata[6]=0xBB;
				tempdata[7]=0xBB;
				tempdata[8]=0xBB;
				tempdata[9]=0xBB;
				tempdata[10]=0xE9;
				printf("debug111!!!!\n");
				adcs_flag=1;
				err_num++;
			} 
			else				/*若相等,则存AA至临时数组*/
			{	
				tempdata[0]=0xE9;
				tempdata[1]=0xAA;
				tempdata[2]=0xAA;
				tempdata[3]=0xAA;
				tempdata[4]=0xAA;
				tempdata[5]=0xAA;
				tempdata[6]=0xAA;
				tempdata[7]=0xAA;
				tempdata[8]=0xAA;
				tempdata[9]=0xAA;
				tempdata[10]=0xE9;
				adcs_flag=0;
				for (m=2;m<adcs_len+2;m++)
				{
					adcs_data[m-2]=adcs_temp_data_back[m];
				}
				//		}

		}
		//printf("tempdata= %x %x %x %x %x %x %x %x %x %x %x\n",tempdata[0],tempdata[1],tempdata[2], tempdata[3],tempdata[4],tempdata[5],tempdata[6],tempdata[7],tempdata[8],tempdata[9],tempdata[10]);
		printf("hao 2! \n");
		for(n=0;n<11;n++)		/*输出临时数组最终数据*/
		{
			scc3_serial_putc(tempdata[n]);
		}
	}

#endif

}

return err_num;
}

#endif





INT32U adcs_serial_input (INT32U adcs_len, INT8U *adcs_data)
{
	INT32U i,j,adcs_flag;
	INT8U adcs_temp_data[60];
	INT8U tempdata[10];
	//INT8U adcs_data[10];
	adcs_flag=0;
	changemux(1);			/*变换通道*/
	for (i=0;i<60;i++)			/*数组清零*/
	{ 
		adcs_temp_data[i]=0;
	}
	//i=0;
	for (i=0;i<11;i++)			/*数组清零*/
	{ 
		tempdata[i]=0;
	}
	//printf("adcs_input_start\n");

#if 0
	for (i=0;i<13;i++)
	{
		//printf("adcs_input_start111\n");
		adcs_temp_data[i]=scc3_serial_getc();
		//printf("adcs_input_start222\n");
		//printf("adcs_temp_data= %x\n",adcs_temp_data[i]);				
	}
	printf("adcs_temp_data= %x %x %x %x %x %x %x %x %x %x %x %x %x\n",adcs_temp_data[0],adcs_temp_data[1],adcs_temp_data[2],adcs_temp_data[3],adcs_temp_data[4],adcs_temp_data[5],adcs_temp_data[6],adcs_temp_data[7],adcs_temp_data[8],adcs_temp_data[9],adcs_temp_data[10],adcs_temp_data[11],adcs_temp_data[12]);

#endif



	//printf("adcs_input_start\n");
	adcs_temp_data[0]=scc3_serial_getc();	/*读取起始场数据*/
	//printf("adcs_temp t_data= %x\n",adcs_temp_data[0]);
	if (adcs_temp_data[0]!=0xCC)	/*若起始场不是CC,读取所有数据,并将BB存入临时数组*/
	{	
		tempdata[0]=0xE9;
		tempdata[1]=0xBB;
		tempdata[2]=0xBB;
		tempdata[3]=0xBB;
		tempdata[4]=0xBB;
		tempdata[5]=0xBB;
		tempdata[6]=0xBB;
		tempdata[7]=0xBB;
		tempdata[8]=0xBB;
		tempdata[9]=0xD8;
		tempdata[10]=0xE9;
		//printf("adcs_input_err\n");
		for (i=0;i<adcs_len+3;i++)
		{
			adcs_temp_data[0]=scc3_serial_getc();		
		} 
		adcs_flag=1;
		//printf("adcs_input_11\n");

	}
	else				/*若起始场正确,则将AA存入临时数组*/
	{	
		tempdata[0]=0xE9;
		tempdata[1]=0xAA;
		tempdata[2]=0xAA;
		tempdata[3]=0xAA;
		tempdata[4]=0xAA;
		tempdata[5]=0xAA;
		tempdata[6]=0xAA;
		tempdata[7]=0xAA;
		tempdata[8]=0xAA;
		tempdata[9]=0x50;
		tempdata[10]=0xE9;
	}
	//printf("tempdata= %x %x %x %x %x %x %x %x %x %x %x\n",tempdata[0],tempdata[1],tempdata[2], tempdata[3],	tempdata[4],tempdata[5],tempdata[6],tempdata[7],tempdata[8],tempdata[9],tempdata[10]);
	if (adcs_flag!=1)			/*读取数据长度,数据场,校验和与终止场*/
	{	 
		//i=1;
		for (i=1;i<adcs_len+4;i++)
		{
			//printf("DEBUG 2#\n");
			adcs_temp_data[i]=scc3_serial_getc();	
			//adcs_temp_data[k]=0xFF;
			//printf("DEBUG 3#\n");	
		} 

		printf("adcs_temp_data= %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",adcs_temp_data[1],adcs_temp_data[2],adcs_temp_data[3],adcs_temp_data[4],adcs_temp_data[5],adcs_temp_data[6],adcs_temp_data[7],adcs_temp_data[8],adcs_temp_data[9],adcs_temp_data[10],adcs_temp_data[11],adcs_temp_data[12],adcs_temp_data[13],adcs_temp_data[14],adcs_temp_data[15],adcs_temp_data[16],adcs_temp_data[17],adcs_temp_data[18],adcs_temp_data[19],adcs_temp_data[20],adcs_temp_data[21],adcs_temp_data[22],adcs_temp_data[23],adcs_temp_data[24],adcs_temp_data[25],adcs_temp_data[26],adcs_temp_data[27],adcs_temp_data[28],adcs_temp_data[29],adcs_temp_data[30],adcs_temp_data[31],adcs_temp_data[32],adcs_temp_data[33],adcs_temp_data[34],adcs_temp_data[35],adcs_temp_data[36],adcs_temp_data[37],adcs_temp_data[38],adcs_temp_data[39],adcs_temp_data[40],adcs_temp_data[41],adcs_temp_data[42],adcs_temp_data[43],adcs_temp_data[44],adcs_temp_data[45],adcs_temp_data[46],adcs_temp_data[47],adcs_temp_data[48],adcs_temp_data[49],adcs_temp_data[50],adcs_temp_data[51],adcs_temp_data[52]);
	}
	if ((adcs_checksum(&adcs_temp_data[2], adcs_len, adcs_temp_data[adcs_len+2])!=0) || (adcs_temp_data[adcs_len+3]!=0xCC))   /*判断校验和若不相等,终止场不等于CC,则存BB至临时数组*/
	{	
		tempdata[0]=0xE9;
		tempdata[1]=0xBB;
		tempdata[2]=0xBB;
		tempdata[3]=0xBB;
		tempdata[4]=0xBB;
		tempdata[5]=0xBB;
		tempdata[6]=0xBB;
		tempdata[7]=0xBB;
		tempdata[8]=0xBB;
		tempdata[9]=0xD8;
		tempdata[10]=0xE9;
		//adcs_temp_data[adcs_len+3]=scc3_serial_getc();
		//adcs_flag=1;
		printf("err\n");
	} 
	else				/*若相等,则存AA至临时数组*/
	{	
		tempdata[0]=0xE9;
		tempdata[1]=0xAA;
		tempdata[2]=0xAA;
		tempdata[3]=0xAA;
		tempdata[4]=0xAA;
		tempdata[5]=0xAA;
		tempdata[6]=0xAA;
		tempdata[7]=0xAA;
		tempdata[8]=0xAA;
		tempdata[9]=0x50;
		tempdata[10]=0xE9;
		printf("right\n");
		for (i=2;i<adcs_len+2;i++)
		{
			adcs_data[i-2]=adcs_temp_data[i];
		}
	}


	//printf("adcs_data= %x %x %x %x %x\n",adcs_data[0],adcs_data[1],adcs_data[2],adcs_data[3],adcs_data[4]);
	//printf("tempdata= %x %x %x %x %x %x %x %x %x %x %x\n",tempdata[0],tempdata[1],tempdata[2], tempdata[3],	tempdata[4],tempdata[5],tempdata[6],tempdata[7],tempdata[8],tempdata[9],tempdata[10]);
	adcs_flag=0;		/*标志清零*/
	for(j=0;j<11;j++)		/*输出临时数组最终数据*/
	{
		scc3_serial_putc(tempdata[j]);
	}

}

INT32U adcs_serial_output (INT32U adcs_len1, INT8U *adcs_rxdata)
{
	INT32U i,j,k,err;
	INT8U adcs_data1[60];
	INT8U adcs_temp1[10];
	changemux(1);
	err=0;
	adcs_data1[0]=0xE9;				/*加起始场*/
	for (i=0;i<adcs_len1;i++)
	{
		adcs_data1[i+1]=adcs_rxdata[i];	/*将上层adcs传送给obc的说据从adcs_rxdata[i]中读出*/
	}
	//adcs_data1[i+1]=adcs_makesum(&adcs_data1[1], adcs_len1);	/*加校验和*/
	adcs_data1[adcs_len1+1]=0xE9;			/*加终止场*/
	//printf("adcs_data1= %x %x %x %x %x %x %x %x %x %x %x\n",adcs_data1[0],adcs_data1[1],adcs_data1[2], adcs_data1[3],adcs_data1[4],adcs_data1[5],adcs_data1[6],adcs_data1[7],adcs_data1[8],adcs_data1[9],adcs_data1[10]);
	while (err<3)
	{
		for (j=0;j<adcs_len1+2;j++)		/*向下层adcs输出完整一帧数据*/
		{
			scc3_serial_putc(adcs_data1[j]);
		}
		//printf("DEBUG 1#\n");
		for (k=0;k<11;k++)			/*接收下层adcs返回数据*/
		{
			//printf("DEBUG 2#\n");
			adcs_temp1[k]=scc3_serial_getc();
			//printf("DEBUG 3#\n");
		}
		//printf("DEBUG 4#\n");
		//printf("adcs_temp1= %x %x %x %x %x %x %x %x %x %x %x\n",adcs_temp1[0],adcs_temp1[1],adcs_temp1[2], adcs_temp1[3],adcs_temp1[4],adcs_temp1[5],adcs_temp1[6],adcs_temp1[7],adcs_temp1[8],adcs_temp1[9],adcs_temp1[10]);

		if (adcs_temp1[1]!=0xAA) {err++;}	/*若下层返回BB,则重发三次*/
		else break;
	}  
	return err;
}

INT32U cj_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	changemux(0);
	for (i = 0; i<quantity; i++)
	{
		*s = scc3_serial_getc();
		s++;
	}
	return i;
}

INT32U cj_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i,j,err;
	INT8U cj_temp[4];
	changemux(0);
	err=0;
	for (i = 0; i<quantity; i++)
	{
		scc3_serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	for(j = 0; j<4; j++)
	{
		cj_temp[j] = scc3_serial_getc();
	}
	if (cj_temp[2] != 0x9E)      //如果模块工作不正常 , err++
		err++;
	return err;
}

INT32U gps_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	changemux(2);
	for (i = 0; i<quantity; i++)
	{
		*s = scc3_serial_getc();
		s++;
	}
	return i;
}

INT32U gps_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i;
	changemux(2);
	for (i = 0; i<quantity; i++)
	{
		scc3_serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	return i;
}

INT32U mt_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	changemux(3);
	for (i = 0; i<quantity; i++)
	{
		*s = scc3_serial_getc();
		s++;
	}
	return i;
}

INT32U mt_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i;
	changemux(3);
	for (i = 0; i<quantity; i++)
	{
		scc3_serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	return i;
}

INT32U ttc_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		*s = scc2_serial_getc();
		s++;
	}
	return i;
}

INT32U ttc_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		scc2_serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	return i;
}

INT32U rf0_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		*s = smc1_serial_getc();
		s++;
	}
	return i;
}

INT32U rf0_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		smc1_serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	return i;
}

INT32U rf1_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		*s = smc2_serial_getc();
		s++;
	}
	return i;
}

INT32U rf1_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		smc2_serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	return i;
}

/*****************************************************************

  console is on SCC1 serial channel

 ******************************************************************/

INT32U console_serial_input (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		*s = serial_getc();
		s++;
	}
	return i;
}

INT32U console_serial_output (INT32U quantity, INT8U *s)
{
	INT32U i;
	for (i = 0; i<quantity; i++)
	{
		printf("%x ", *s);
		//serial_putc(*s);	//attension '/n' will not be changed in this version.
		s++;
	}
	return i;
}



/***************************************************************

  CAN 发送函数

 *data :  CAN结构的首字节指针 can_frame, can_flow
type  :  1:姿控数据报 can_frame
2:其他数据报 can_frame
3:其他数据流 can_flow
mode :  DPRAM中对应的valid值
CANMSG 1
CANFLOW 2
CANPUT 3


 ****************************************************************/
INT32U can_datagram_send(INT8U *data,INT32U type,INT8U mode)
{
	INT32U k,len,i;
	INT8U *q;
	INT8U tempdata[20];
	INT32U can_addr1=DPRAM_SEND_CAN_ADCS_ADDR;
	INT32U can_addr2=DPRAM_SEND_CAN_DATA_ADDR;
	INT32U can_addr3=DPRAM_SEND_CAN_DLL_ADDR;
	INT32U can_addr4=DPRAM_SEND_CAN_RF_ADDR; 
	INT32U can_addr5=DPRAM_SEND_CAN_RFT_ADDR; 
	INT32U can_addr6=DPRAM_SEND_CAN_MIMU_ADDR;
	INT32U addr,temp;

	OSSchedLock();
	q=data;
	//printf("type is 0x%x\n",type);
	if (type==1)    				/*姿控数据报*/
	{
		//printf("---------1\n");
		addr=can_addr1;
		len=DATAGRAM_LEN;
		//printf("addr= %x\n",addr);
		*(volatile INT8U *)addr = mode;   /*数据区第一个字节 valid,即va*/  

		addr+=2;

		for(k=0;k<3;k++)               /*读入结构体中的id,len,type*/
		{
			*(volatile INT8U *)addr = *q;
			//printf("AA=%x\n",*(volatile INT8U *)addr);
			q++;     
			addr+=2;
		}
		addr=addr-4;

		if (*(volatile INT8U *)addr==0x01) /*判断len,若等于1,只接收一桢,若等于2,则接收两桢*/
		{
			addr+=12;			  /*addr=0x100060011*/
			//printf("addr44= %x\n",addr);
			for(k=0;k<5;k++)
			{
				*(volatile INT8U *)addr = *q;
				tempdata[k]= *(volatile INT8U *)addr;
				//printf("tempdata= %x\n",tempdata[k]);
				q++;     
				addr+=2;
			}
			//printf("addr55= %x\n",addr);
		}
		else
		{
			addr+=12;			  /*addr=0x100060011*/
			//printf("addr444= %x\n",addr);
			for(k=0;k<5;k++)
			{
				*(volatile INT8U *)addr = *q;
				tempdata[k]= *(volatile INT8U *)addr;
				//printf("tempdata= %x\n",tempdata[k]);
				q++;     
				addr+=2;
			}
			//printf("addr555= %x\n",addr);
			addr+=0x0C;
			for(k=0;k<5;k++)
			{
				*(volatile INT8U *)addr = *q;
				q++;     
				addr+=2;
			}
		}

		addr=can_addr1;
		//printf("B=%x\n",addr);
		for(i=0;i<4;i++)
		{
			tempdata[i]=*(volatile INT8U *)addr;
			addr+=2;
		}
		addr+=0x08;
		//printf("addr66=%x\n",addr);
		for(i=4;i<9;i++)
		{
			tempdata[i]=*(volatile INT8U *)addr;
			addr+=2;
		}
		addr+=0x0C;
		for(i=9;i<14;i++)
		{
			tempdata[i]=*(volatile INT8U *)addr;
			addr+=2;
		}
		//OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
		//printf("tempdata= %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",tempdata[0],tempdata[1],tempdata[2],tempdata[3],tempdata[4],tempdata[5],tempdata[6],tempdata[7],tempdata[8],tempdata[9],tempdata[10],tempdata[11],tempdata[12],tempdata[13]);   
		//OSSemPost(SHARE_PRINTF);
	}
	else if (type==4) 			/*动量轮数据*/
	{
		printf("---------4\n");
		addr=can_addr3;
		*(volatile INT8U *)addr = mode;   /*数据区第一个字节 valid,即va*/  
		addr+=2;
		MWFlag = *q;
		for(i=0;i<6;i++)
		{
			*(volatile INT8U *)addr = *q;
			q++;     
			addr+=2;
		}
		addr=can_addr3;
		for(i=0;i<7;i++)
		{
			tempdata[i]=*(volatile INT8U *)addr;
			addr+=2;
		}

		printf("tempdata= %x %x %x %x %x %x %x\n",tempdata[0],tempdata[1],tempdata[2],tempdata[3],tempdata[4],tempdata[5],tempdata[6]);
		printf("dll_send_ok!\n");

	}	
	else if (type==0x50)
	{		
		addr=can_addr4;
		OSSemPend(SHARE_PRINTF,0,&ERR_SHARE_PRINTF);
		*(volatile INT8U *)addr = mode;
		printf("mode = %02x\n",mode); 
		addr+=2;
		*(volatile INT8U *)addr = 0x00; 
		addr+=2;
		printf("can data=\n"); 

		for(i=0;i<8;i++)
		{
			*(volatile INT8U *)addr = *q;
			q++; 
			printf("%02x ",*(volatile INT8U *)addr);     
			addr+=2;
		}
		printf("\n"); 
		OSSemPost(SHARE_PRINTF);
	}
	else if (type==0x52)
	{
		addr=can_addr4;
		*(volatile INT8U *)addr = mode; 
		addr+=2;
		*(volatile INT8U *)addr = 0x02; 
		addr+=2;

		for(i=0;i<8;i++)
		{
			*(volatile INT8U *)addr = *q;
			q++;     
			addr+=2;
		}
	}
	else if (type==0x53)
	{
		addr=can_addr4;
		*(volatile INT8U *)addr = mode; 
		addr+=2;
		*(volatile INT8U *)addr = 0x03; 
		addr+=2;

		for(i=0;i<8;i++)
		{
			*(volatile INT8U *)addr = *q;
			q++;     
			addr+=2;
		}
	}
	else if (type==0x51)
	{
		addr=can_addr5;
		*(volatile INT8U *)0x10006A03=0xff;
		*(volatile INT8U *)addr = mode; 
		addr+=2;
		*(volatile INT8U *)addr = 0x01; 
		addr+=2;

		printf("type=0x51\n");

		for(i=0;i<28;i++)
		{
			*(volatile INT8U *)addr = *q;
			q++;     
			addr+=2;
		}
	}
	else if (type==0x05)	/*MIMU数据*/
	{
		addr=can_addr6;

		//printf("DPRAM_SEND_CAN_MIMU_ADDR=%x\n",*(volatile INT8U *)DPRAM_SEND_CAN_MIMU_ADDR);  

		*(volatile INT8U *)addr = mode;   /*数据区第一个字节 valid,即va*/ 
		//printf("DPRAM_SEND_CAN_MIMU_ADDR=%x\n",*(volatile INT8U *)DPRAM_SEND_CAN_MIMU_ADDR); 

		addr+=2;
		for(i=0;i<6;i++)
		{
			*(volatile INT8U *)addr = *q;
			q++;     
			addr+=2;
		}
		//addr=can_addr4;

		//printf("addr4=======%x\n",can_addr4);
		//for(i=0;i<7;i++)
		//{
		//tempdata[i]=*(volatile INT8U *)addr;
		//addr+=2;
		//}
		//printf("tempdata= %x %x %x %x %x %x  %x\n",tempdata[0],tempdata[1],tempdata[2],tempdata[3],tempdata[4],tempdata[5],tempdata[6]);
		//printf("mimu_send_ok!\n");

	} 
	else 
	{
		if (type==2)  			 /*其他数据报*/
		{
			addr=can_addr2;
			len=DATAGRAM_LEN;
		}
		if (type==3)				 /*其他数据流*/
		{
			addr=can_addr2;
			len=DATASTREAM_LEN;     
		}

		*(volatile INT8U *)addr = mode;       /*数据区第一个字节 valid*/
		addr+=2;
		temp=addr;

		for(k=0;k<len;k++)
		{
			*(volatile INT8U *)addr = 0x0;
			addr+=2;
		}
		addr=temp;
		for(k=0;k<len;k++)
		{
			*(volatile INT8U *)addr = *q;
			q++;     
			addr+=2;
		}	
	}
	*(volatile INT8U *)(DPRAM_SEND_IRQ_ADDR)=0xff;          /*产生中断*/
	for(k=0;k<300;k++)
		delay();
	OSSchedUnlock();   
	return 0; 
}  



/*******************DPRAM 中断处理函数 开始**********************/
void DPRAM_IRQ(void)
{	

	static INT8U buf[262*10],temp,id;
	INT32U k,len,i;
	static INT32U j;
	INT32U addr1 = DPRAM_RECEIVE_CAN_ADCS_ADDR;
	INT32U addr2 = DPRAM_RECEIVE_CAN_DATA_ADDR;
	INT32U addr3 = DPRAM_RECEIVE_422_ADDR;
	INT32U addr4 = DPRAM_RECEIVE_CAN_DLL_ADDR;
	//INT32U addr5 = DPRAM_RECEIVE_CAN_RF_ADDR; 
	INT32U addr6 = 0x100043FF;//DPRAM_RECEIVE_CAN_RFM_ADDR;
	INT32U addr7 = DPRAM_RECEIVE_CAN_MIMU_ADDR;
	INT32U temp1;
	INT32U temp2;
	INT32U temp3;
	INT32U temp4; 
	//INT32U temp5; 
	INT32U temp6; 
	INT32U temp7; 
	INT8U err_2,LockBuf[2];
	volatile INT8U * DLLADDR = (volatile INT8U *)DPRAM_RECEIVE_CAN_DLL_ADDR;
	//清中断
	temp = *(volatile INT8U *)(DPRAM_RECEIVE_IRQ_ADDR);
	addr1 = DPRAM_RECEIVE_CAN_ADCS_ADDR;
	addr2 = DPRAM_RECEIVE_CAN_DATA_ADDR;
	addr3 = DPRAM_RECEIVE_422_ADDR;
	addr4 = DPRAM_RECEIVE_CAN_DLL_ADDR;
	//INT32U addr5 = DPRAM_RECEIVE_CAN_RF_ADDR;
	addr6 = 0x100043FF;//DPRAM_RECEIVE_CAN_RFM_ADDR; 
	addr7 = DPRAM_RECEIVE_CAN_MIMU_ADDR;
	//printf("DPRAM INTERRUPT\n");
	temp1=*(volatile INT8U *)(addr1);
	temp2=*(volatile INT8U *)(addr2);
	temp3=*(volatile INT8U *)(addr3);

	temp4=*(volatile INT8U *)(addr4); 

	//temp5=*(volatile INT8U *)(addr5); 
	temp6=*(volatile INT8U *)(addr6); 
	temp7=*(volatile INT8U *)(addr7); 
	//printf("temp= %x\n",temp);


	if(temp1 != 0x0)                /*姿控数据报*/
	{
		OSSchedLock();
		*(volatile INT8U *)(addr1)=0x0;			/*清零*/
		buf[0]=temp1;
		addr1+=2;
		for(k=1;k<3;k++)					/*读取id,len*/
		{
			buf[k]=*(volatile INT8U *)addr1;
			addr1+=2;
		}

		len=buf[2];					/*将len(即桢数)的值赋给temp*/

		i=len*11;						/*所有桢长度数据*/
		for(k=0;k<i;k++)
		{
			buf[262*j+k]=*(volatile INT8U *)(addr1);
			addr1+=2;
		}

		OSQPost_i(CAN_Q_Rec,&buf[262*j]);
		OSQPost_i(CAN_Q_Rec2,&buf[262*j]);
		OSSchedUnlock();  
		printf("len= %x\n",len);
		printf("ADCS CAN DATA.\n");
		printf("buf[012]=%x %x %x\n",buf[0],buf[1],buf[2]);
		j+=1;
		//printf("j= %x\n",j);
		if(j>=8)
			j=0; 
		//return;
	}

	if (temp2 != 0x0)
	{  

		*(volatile INT8U *)(addr2)=0x0;

		if(temp2==0x02)   /*数据流*/
		{
			//printf("TTC\n");
			OSSchedLock();
			addr2+=2;
			for(k=0;k<262;k++)
			{
				buf[262*j+k]=*(volatile INT8U *)(addr2);
				addr2+=2;
			}
			OSSchedUnlock();  

			id=buf[262*j+1];

			if(id==ID_TTC2)
			{

				//printf("TTC interrupt sent\n");
				if(WODFlag)
				{
					WODFlag = 0;
					OSQPost_i(CAN_Q_REC_TTC,&buf[262*j]);  //发送到TTC的消息队列

				}
				else
				{
					//	LockBuf[0] = buf[262*j+109];
					//	LockBuf[1] = buf[262*j+110];
					OSQPost_i(CAN_Q_REC_LOCK,&buf[262*j]);
				}
			}
		}

		j+=1;
		//printf("j= %x\n",j);
		if(j>=8)
			j=0; 
		//return;

	}

	if (temp3 != 0x0)
	{
		OSSchedLock();
		*(volatile INT8U *)(addr3)=0x0;
		addr3+=2;
		len=*(volatile INT8U *)(addr3)*0x0100+*(volatile INT8U *)(addr3+2);

		buf[262*j]=*(volatile INT8U *)(addr3);
		buf[262*j+1]=*(volatile INT8U *)(addr3+2);
		addr3+=4;
		for(k=0;k<len;k++)
		{
			buf[262*j+k+2]=*(volatile INT8U *)(addr3);
			addr3+=2;
		}
		OSQPost_i(RS422_Q_Rec,&buf[262*j]);
		OSSchedUnlock();  
		temp=*(volatile INT8U *)(DPRAM_RECEIVE_IRQ_ADDR);


		j+=1;
		if(j>=8)
			j=0; 
	}


#if 1
	else if (temp6 != 0)
	{

		OSSchedLock();

		*(volatile INT8U *)(addr6)=0x0;

		for(i=0;i<31;i++)
		{
			buf[262*j+i]=*(volatile INT8U *)(addr6+2*i);
			//printf("%x ",buf[i]);
		}
		OSSchedUnlock(); 
		//printf("223344!\n");

		OSQPost_i(CAN_Q_Rec_RF1,&buf[262*j]);

		j+=1;
		if(j>=8)
			j=0; 
		//return;

	}
#endif
	if (temp7 !=0x0)
	{
		printf("temp7 =====\n");
		*(volatile INT8U *)(addr7)=0x0;

		addr7+=4;

		printf("temp55=%x\n",*(volatile INT8U *)(addr7));

		OSSchedLock();

		len=*(volatile INT8U *)(addr7);   /*第二个字节*/
		if(len==5) i=40;
		else	 i=8; 		
		addr7=addr7-2;			/*第一个字节*/

		for(k=0;k<i;k++)
		{
			buf[262*j+k]=*(volatile INT8U *)(addr7);
			addr7+=2;
			//    printf("buf[%d]=%x,",k,buf[262*j+k]);
		}
		OSSchedUnlock(); 
		printf("\n");
		err_2 = OSQPost_i(CAN_Q_Rec_MIMU,&buf[262*j]);
		//err_2 = OSQPostFront(CAN_Q_Rec_MIMU,&buf[262*j]);
		if(err_2==OS_NO_ERR)
			printf("post success.\n");
		printf("err_2=%x\n",err_2);	
		//printf("11223300!\n");


		j+=1;
		printf("j= %x\n",j);

		if(j>=8)
			j=0; 
		//return;
	}

	if(temp4!=0)
	{
		//temp=*(volatile INT8U *)(addr4);
		OSSchedLock();
		buf[262*j]=*(volatile INT8U *)(addr4);
		*(volatile INT8U *)(addr4)=0x0;
		//addr4+=2;

		for(k=1;k<21;k++)
		{
			buf[262*j+k]=*(volatile INT8U *)(addr4);
			addr4+=2;
		}
		OSSchedUnlock();  
                for (k = 0; k < 20 ; k ++) {
                        printf ("buf[] : %04x   ..", buf[262*j + k]);
                }

                printf ("\n");
		OSQPost_i(CAN_Q_Rec_DLL,&buf[262*j]);
		j+=1;
		//printf("j= %x\n",j);
		if(j>=8)
			j=0; 

	}	 
	return;

}
/*******************DPRAM 中断处理函数 结束**********************/
