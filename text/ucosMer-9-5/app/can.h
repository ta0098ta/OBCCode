typedef struct{
INT8U t;
INT8U sdlc;   /*桢类型*//*物理层数据场的长度*/     
INT8U f;           /*目的地址*/
INT8U d0;
INT8U d1;          /*用户字节，其中，第一个字节是目的地址*/ 
INT8U d2;
INT8U d3;
INT8U d4;
INT8U d5;
INT8U d6;
}can_frame; 


typedef struct{
INT8U d0;
INT8U d1;
INT8U d2;
INT8U d3;
INT8U d4;
INT8U d5;
INT8U d6;
INT8U d7;
INT8U d8;
INT8U d9;
}can_stream_frame;

typedef struct{
INT8U id;
INT8U f;
INT8U i;
INT8U i2;
INT8U l;
INT8U l2;
INT8U d[256];
}can_flow;


#define Load_Command 16
#define Load_Response 17
#define Execute_Command 18
#define Execute_Response 19
#define Dump_Command 20
#define Dump_Response 21
#define Specific_Command 22
#define Specific_Response 23
#define TLM_request 24
#define TLM_ack_response 25
#define TLM_nak_response 26
#define TC_request 27
#define TC_ack_response 28
#define TC_nak_response 29
#define TLM_put_request 30
#define TLM_put_response 31


/*#define GET 0
#define PUT 1
#define GETOK 2
#define PUTOK 3
#define Abort 4
#define BurstRequest 5
#define Done 6
#define DoneOk 7
#define AbortOk 8
#define GetReject 9
#define PutReject 10*/

#define OK 0
#define ERROR 4
#define BUF_FULL 1
#define TIMEOUT 2
#define UNKNOWN_ERROR 3
#define FIRST_FRAME_ERROR 5

/************CAN节点的ID号*************/
#define TTC 0x10

#define ADCS_DATAGRAM_OBC 0x31
#define OTHER_DATAGRAM_OBC 0x30
#define TTC_TC_DATASTREAM 0x32
#define OBC_REQ_GPS 0x34
#define GPS_REQ_OBC 0x33
#define OBC_REQ_EIS 0x35
#define TTC_TLM_DATASTREAM 0x36

#define ID_TTC1		0x11	 /*间接遥控 */
#define ID_TTC2		0x12	 /*间接遥测 */

#define ID_MCU		0x40	/* ACS数据报 */
#define ID_ATC		0x50	/* ACS数据报 */
#define ID_GPS		0x60	/* GPS数据报 */
#define ID_GPS1		0x61	/* GPS主动发GET数据流 */
#define ID_GPS2		0x62	/* OBC主动发GET数据流 */
#define ID_PHOTO	0x64	/* 相机数据报 */
#define ID_PHOTO1	0x65	/* 相机数据流 */
#define ID_EGSE		0xc0	/* EGSE地址 */

/*-----------------------------------------------------------------------------------------------------------*/
//#define ACS 0x40
/*-----------------------------------------------------------------------------------------------------------*/
#define GPS 0x80
#define EIS 0x64
#define EIS1 0x65
#define WOD 0xff

#define DATAGRAM_LEN 10
#define DATASTREAM_LEN 262

#define CANMSG 1
#define CANFLOW 2
#define CANPUT 3
/***********OBC ID****************
姿控数据报				31H
其他数据报				30H
TTC间接遥控数据流			32H
OBC请求GPS数据流			33H
GPS请求OBC数据流			34H
OBC请求相机数据流	    	35H
TTC间接遥测数据流			36H*/


/*#define ID_OBCNORMAL      0x30	 普通数据报，包括相机、GPS */
/*#define ID_OBCACS		    0x31	 ACS数据报 */
/*#define ID_OBCTTC1		0x32     间接遥控 */
/*#define ID_OBCGPS1		0x33	 GPS主动发GET数据流 */
/*#define ID_OBCGPS2		0x34	 OBC主动发GET数据流 */
/*#define ID_OBCPHOTO		0x35	 相机数据流 */
/*#define ID_OBCTTC2		0x36	 间接遥测 */


