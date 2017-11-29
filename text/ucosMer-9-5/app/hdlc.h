#define MAX_SEND_LEN (1059)     /*下行链路下发的最大字节数（a frame),不算最后4个字节的0xaaaaaaaa ??????1058-4*/
#define MAX_SEND_PAC_LEN 256      /*下行链路每包最大字节数(a packet),不含包头*/
#define MAX_REC_LEN   300       /*上行链路收到的最大字节数(a frame)*/
#define MAX_SEND_WOD 1024*1024    /*要发送的最大数据字节数*/
#define MAX_SEND_FRAME_COUNT 990  /*要发送的最大桢计数*/
#define MAX_SEQ 7
#define SEND_PRIO 5 
#define EIS_LEN 1024*1024         /*相机数据的长度*/
#define HDLC_TIME 30              /*发送时使用的超时时间(3s)*/
#define TICK_INTERVAL  30         /*每个tick的时间间隔*/
#define EIS_frame_len  256        /*相机每次传送6桢，共256个字节*/
#define EIS_frame_count 4096      /*相机的1M数据需传送4096次*/
#define TTC_data_len   104        /*ttc发过来的实时遥测数据*/
#define PHOTO_ONE_MINUTE 145      /*1分钟下传的图象桢数*/
#define LAST_EIS_NUM    8         /*最后一个1分钟数据的计数*/
#define ADCS_LEN 174


/*00000000000000000000000000000000000000000000000000000000000000000000000000000000*/

#define wod_data_len    (ADCS_LEN+TTC_data_len+64+6*2+1)   /*355 WOD数据实际有效字节数，包括GPS周及周内秒 64为GPS长度，10为GPS周和GPS秒占据的时间标签长度，每个WOD包有两个时间标签*/
/*000000000000000000000000000000000000000000000000000000000000000000000000000000000*/

#define WOD_ALL_LEN wod_data_len*1000  /*所要存储的WOD总字节数，共1000包，每包wod_data_len个字节*/
#define sabm_num 8
#define disc_num 9

#define ua 0
#define rr 1
#define rej 2
#define dm 3
#define i_data 4
#define sabm 5
#define disc 6


/*----------------上传命令--------------------*/
#define  real_time_com 0x00    /*实时、间接遥控命令*/
#define  application 0x01      /*应用程序*/
#define  acs_data_req  0x02    /*姿控数据包请求*/
#define  time_orbit	 0x03		/**/
#define rf_star_req		0x04 /*星间rf请求*/
#define star_req        0x05     /*星敏感器请求*/
#define	 MEMSGyro	0x06	/*MEMSGyro请求*/
#define  gps_data    0x07      /*得到GPS数据*/
#define		BDGPS 	0x08	/*BDGPS数据包请求*/



/*ZWJ-----------------------------------------------------------*/
#define  ACS	 0x09		/*OBC直接遥控要测ACS*/
/*----------------------------------------------------------------*/

#define task_manage    0x0F	/*任务管理*/
#define dll_req         0x0B     /*动量轮请求*/
//#define WOD_req      0x0C	/*wod*/
#define wod_req		0x0C	/*wod*/

#define  gps_program      0x0d    /*gps程序(临时用,未定)*/
#define  program_com	0x0e	//程控指令

#define task_on	0x01		/*任务恢复*/
#define task_off	0x00		/*任务删除*/


/*OS_EVENT *RecMsg,*HdlcQEvent,*sendSem,*sendMessageQ;*/

enum myevent {sabm_frame_ready,i_frame_ready,disc_frame_ready,frame_arrival,timeout};

typedef struct {             /*同步下行包结构*/
  INT16U sflag;
  INT16U ctr;
  INT16U len;
  INT16U data[128];          /*254个有效数据字节*/ 
}PACKET;

typedef struct{              /*同步下行祯结构*/
  INT16U sflag;
  INT16U bak_num:8;
  INT16U main_num:8;
  INT16U state;              /*包计数*/
  PACKET data1;
  PACKET data2;
  PACKET data3;
  PACKET data4;
  INT32U ctr_data;           /*0xaaaaaaaa*/
}FRAME;                 

typedef struct{
  INT16U id;
  INT16U num;
  INT16U msg[MAX_SEND_LEN/2];        /*??????????*/
}FrameMsg;


struct frame_node{          /*同步串口定时器链表*/
   INT32U timer;
   INT8U id;
   struct frame_node *next;
   INT8U state;   
};


struct comm_node{            /*程控命令链表*/
   double second;            /*GPS周内时间秒*/
   INT16U week;              /*GPS周*/
   INT16U command[15];       /*命令,29个字节*/
   struct comm_node *next; 
};

struct wod_data{
   INT32U second;
   INT16U week;
   INT16U data[TTC_data_len/2];
};


