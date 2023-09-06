#ifndef DEVICE_NAME_OP_H__
#define DEVICE_NAME_OP_H__
//引用的C库头文件
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
//引用FDS头文件
#include "fds.h"

#define APP_BLE_CONN_CFG_TAG         1          //SoftDevice BLE配置标志
//定义文件ID和该文件包含的记录的KEY
#define NAME_FILE                    (0x1000)   //文件ID
#define NAME_REC_KEY                 (0x1001)   //记录KEY,该记录存放的文件ID=0x1000
#define NAME_FLASH_TYPE_NAME         0x5A5A5A5A //用来表示该记录存储是的设备名称
#define NAME_FLASH_LEN_WORDS         10         //设备名称长度

//定义文件ID和该文件包含的记录的KEY
#define DATA_FILE                    (0x2000)   //文件ID
#define DATA_REC_KEY                 (0x2001)   //记录KEY,该记录存放的文件ID=0x2000
#define DATA_FLASH_TYPE_NAME         0x6B6B6B6B //用来表示该记录存储是的标定数据
#define DATA_FLASH_LEN_WORDS         10         //标定数据长度

//设备名称信息结构体
typedef struct
{
  bool save_name_flag;                          //设备名称存储标志,主循环中检查该标志是否置位,从而判断是否有新的设备名称需要存储
  bool name_exist;                              //Flash中已经存储了设备名称
  union
  {
    uint32_t name32[NAME_FLASH_LEN_WORDS];      //存放设备名称的数组,数据类型为uint32_t。
    uint8_t name8[NAME_FLASH_LEN_WORDS*4];      //存放设备名称的数组,数据类型为uint8_t。 
  }name;
} device_name_info_t;

//重量数据信息结构体
typedef struct
{
  bool save_data_flag;                          //重量数据存储标志,主循环中检查该标志是否置位,从而判断是否有新的重量数据需要存储
  bool data_exist;                              //Flash中已经存储了重量数据
  union
  {
    uint32_t data32[DATA_FLASH_LEN_WORDS];      //存放重量数据的数组,数据类型为uint32_t。
    uint8_t data8[DATA_FLASH_LEN_WORDS*4];      //存放重量数据的数组,数据类型为uint8_t。 
  }data;
} weight_data_info_t;

//FDS异步操作标志结构体
typedef struct
{
  bool gc;         //碎片收集标志
  bool busy;       //FDS忙标志
}my_fds_info_t;

//记录m_name_record的内容,注意一定要4字节对齐
typedef struct
{
  uint8_t     name[NAME_FLASH_LEN_WORDS*4];    //设备名称存储的格式:0x5A5A5A5A + 长度 + 名称字符串
}__attribute__((aligned(4)))device_name_t;

//记录m_weight_record的内容,注意一定要4字节对齐
typedef struct
{
  uint8_t     data[DATA_FLASH_LEN_WORDS*4];    //重量数据存储的格式:0x5B5B5B5B + 长度 + 名称字符串
}__attribute__((aligned(4)))weight_data_t;


extern weight_data_info_t weight_data_info;

void ble_gatt_params_on_ble_evt(ble_evt_t const * p_ble_evt);
void advertising_start(void);
void advertising_init(void);
ret_code_t my_fds_init(void);
void device_name_set(void);
void weight_data_set(void);
void device_name_handle(void);
void weight_data_handle(void);
void ble_adv_start(void);
void ble_adv_stop(void);
void ble_adv_auto_start(void);
void ble_adv_auto_stop(void);
#endif

