#ifndef INCLUDES_H
#define INCLUDES_H

#if 0
#include    <stdio.h>
#include    <string.h>
//#include    <ctype.h>
#include    <stdlib.h>

#include    <app_cfg.h>
#include    <ucos_ii.h>
#include    <bsp.h>
#endif



#if 0
#define char TypeError
#define short TypeError
#define long TypeError
#define int TypeError
#define unsigned TypeError
#endif

#define DPRAM_SEND_IRQ_ADDR             0x1000fffd
#define DPRAM_RECEIVE_IRQ_ADDR          0x1000ffff

#define DPRAM_SEND_CAN_ADCS_ADDR        0x10006001
#define DPRAM_SEND_CAN_DATA_ADDR        0x10006201
#define DPRAM_SEND_422_ADDR             0x10002001
#define DPRAM_SEND_CAN_DLL_ADDR         0x10006601
#define DPRAM_SEND_CAN_RF_ADDR          0x10006A01
#define DPRAM_SEND_CAN_RFT_ADDR         0x10004801
#define DPRAM_SEND_CAN_MIMU_ADDR        0x10006e01  


#define DPRAM_RECEIVE_CAN_ADCS_ADDR     0x10004001
#define DPRAM_RECEIVE_CAN_DATA_ADDR     0x10004201
#define DPRAM_RECEIVE_422_ADDR          0x10000001
#define DPRAM_RECEIVE_CAN_DLL_ADDR      0x10004601
#define DPRAM_RECEIVE_CAN_RF_ADDR       0x10005001
#define DPRAM_RECEIVE_CAN_RFM_ADDR      0x10004C01
#define DPRAM_RECEIVE_CAN_MIMU_ADDR     0x10005201 

#define EIS_DATA                0x20000000      //星敏图像              1.5M
#define EIS_DATA_SMALL          0x20180000      //星敏缩略图            100K
#define STAR_sensor_DATA        0x20199000      //星敏感器数据起始      50个字节
#define GPS_DATA                0x20199032      //GPS数据起始,          100个字节
#define RF_DATA                 0x20199096      //星间RF数据起始地址    50K
#define MEMSGyro_DATA           0x201A5896      //MEMSGyro数据起始地址  50字节
#define DLL_DATA                0x201A58c8      //动量轮起始地址        50字节 
#define ADCS_REALTIME_DATA      0x201A58FA      //ADCS数据起始地址      50K
#define BDGPS_DATA              0x201B20FA      //BDGPS数据起始地址     200字节
#define PC_DATA                 0x201B21C2      //ProgramControl数据起始地址,500K大小
#define wod_begin_addr          0x2022F1C2      /*WOD数据存储区(2M+8)--由遥测数据+时间标签拼装而成*/
#define wod_end_addr            0x20285C7A	/*355 字节/包*1000包*/ 

#define ADCS_cfg_file           0x20288780      //ADCS 配置文件 

//#define wod_end_addr wod_begin_add+WOD_ALL_LEN /*wod_data_len 字节/包*1000包*/ 

/*	大相机 9  Byte
        小相机 11 Byte
        星敏感 27 Byte
        GPS    64 Byte
        测距   8  Byte
        MIMU  30 Byte
        TTC    96 Byte
        RF	28 Byte


        一共	273 Byte
        +时间标签 12 字节

        =	285 Byte
        */



#define ADCS_SERIALTIMEOUT 100






#include "can.h"
#include "hdlc.h"
//#include "Math_Temp.h"
#if 0
#include "bsp.h"
#include "can.h"
#include "hdlc.h"
#endif

#ifndef __adcsrxflow_defined
#define __adcsrxflow_defined
typedef struct
{
        INT32U va;
        INT32U id;
        INT32U len;
        INT8U  d[110];
}	ADCSrxflow;
#endif
#endif /* INCLUDES_H */ 
/* End of source */
