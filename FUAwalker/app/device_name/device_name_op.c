//引用的C库头文件
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log需要引用的头文件
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_gap.h"
//引用FDS头文件
#include "fds.h"
#include "device_name_op.h"
//广播需要引用的头文件
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "bsp_btn_ble.h"
//串口透传需要引用的头文件
#include "my_ble_uarts.h"

#define DEFAULT_DEVICE_NAME             "Walker_FUA"                  //设备名称字符串
#define APP_ADV_INTERVAL                320                           //广播间隔 (200ms),单位0.625 ms 
#define APP_ADV_DURATION                0                             //广播持续时间,单位:10ms。设置为0表示不超时
#define UARTS_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN    //串口透传服务UUID类型:厂商自定义UUID

BLE_ADVERTISING_DEF(m_advertising);                                   //定义名称为m_advertising的广播模块实例
#define BLE_ADV_PW_LEVEL                4                             //广播发射功率
extern int16_t NO_Data;

void ble_adv_start(void)
{
  ble_adv_modes_config_t config;
  config.ble_adv_fast_enabled = true;
  config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  config.ble_adv_fast_timeout = APP_ADV_DURATION;
  config.ble_adv_on_disconnect_disabled = false;
  ble_advertising_modes_config_set(&m_advertising,&config);

  NRF_LOG_INFO("BLE Adv Start!%d",ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST));
}

void ble_adv_stop(void)
{
  ble_adv_modes_config_t config;
  config.ble_adv_fast_enabled = true;
  config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  config.ble_adv_fast_timeout = APP_ADV_DURATION;
  config.ble_adv_on_disconnect_disabled = true;
  ble_advertising_modes_config_set(&m_advertising,&config);
	
  sd_ble_gap_adv_stop(m_advertising.adv_handle);
  NRF_LOG_INFO("BLE Adv Stop!%d",ble_advertising_start(&m_advertising, BLE_ADV_MODE_IDLE));
}

//定义串口透传服务UUID列表
static ble_uuid_t m_adv_uuids[]          =                                          
{
  {BLE_UUID_UARTS_SERVICE, UARTS_SERVICE_UUID_TYPE}
};

//定义设备名称操作结构体变量
static device_name_info_t device_name_info;

//定义重量数据操作结构体变量
weight_data_info_t weight_data_info;

//定义FDS异步操作标志结构体变量
static my_fds_info_t my_fds_info;

//描述设备信息
static device_name_t device_name;

//描述重量数据
static weight_data_t weight_data;

//包含设备名称信息的记录
static fds_record_t const device_name_record =
{
  .file_id           = NAME_FILE,
  .key               = NAME_REC_KEY,
  .data.p_data       = &device_name,
  //记录的长度必须以4字节（字）为单位
  .data.length_words = (sizeof(device_name) + 3) / sizeof(uint32_t),
};

//包含重量数据信息的记录
static fds_record_t const weight_data_record =
{
  .file_id           = DATA_FILE,
  .key               = DATA_REC_KEY,
  .data.p_data       = &weight_data,
  //记录的长度必须以4字节（字）为单位
  .data.length_words = (sizeof(weight_data) + 3) / sizeof(uint32_t),
};

//FDS事件处理函数
static void fds_evt_handler(fds_evt_t const * p_evt)
{
  //判断事件类型
  switch (p_evt->id)
  {
    case FDS_EVT_INIT://FDS初始化事件
         if(p_evt->result == NRF_SUCCESS)
         {
           NRF_LOG_INFO("FDS init successfully! ");
           my_fds_info.busy = false;
         }
         break;
    case FDS_EVT_WRITE://FDS写记录事件
				 if(p_evt->result == NRF_SUCCESS)
         {
           my_fds_info.busy = false;
         }
				 break;
    case FDS_EVT_UPDATE://FDS更新记录事件
         if (p_evt->result == NRF_SUCCESS)
         {
           my_fds_info.busy = false;
         }
				 break;
    case FDS_EVT_GC://FDS碎片整理事件
				 if(p_evt->result == NRF_SUCCESS)
         {
           my_fds_info.busy = false;
         }
         break;
         default:
         break;
  }
}
	
//等待FDS初始化完成
static void wait_for_fds_ready(void)
{
  while(my_fds_info.busy)
  {
    (void)sd_app_evt_wait();
  }
}

ret_code_t my_fds_init (void)
{
  //注册FDS事件回调函数接收FdS事件,在调用fds_init()函数之前,一定要先注册
  ret_code_t ret = fds_register(fds_evt_handler);
  if(ret != NRF_SUCCESS)
  {
    return ret;	
  }
  my_fds_info.busy = true;
  ret = fds_init();//初始化FDS
  if (ret != NRF_SUCCESS)
  {
    return ret;
  }
  //FDS初始化是异步的,因此要等待FDS初始化完成
  wait_for_fds_ready();
  return NRF_SUCCESS;		
}

static ret_code_t fds_save_devicename (void)
{
  ret_code_t              err_code;
	//定义并初始化记录描述符结构体变量
  fds_record_desc_t desc = {0};	
  //定义并初始化记录查找令牌结构体变量
  fds_find_token_t  tok  = {0};
  //清零tok,从头查找
  memset(&tok, 0x00, sizeof(fds_find_token_t));
  //在DEVICE_FILE文件中查找记录m_desp_record
  err_code = fds_record_find(NAME_FILE, NAME_REC_KEY, &desc, &tok);	
  
  memcpy(&device_name, &device_name_info.name, sizeof(device_name));
	
	//没有查找到m_desp_record记录,写入记录
  if(err_code != NRF_SUCCESS)
  {
    my_fds_info.busy = true;//写记录之前置位FDS忙标志
    err_code = fds_record_write(&desc, &device_name_record);
    //根据函数返回值判断是否需要碎片收集
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;
      my_fds_info.busy = false;
    }
    else
    {
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
    NRF_LOG_INFO("write");
  }
	else//设备名称记录已经存在,更新记录
	{
    my_fds_info.busy = true;//更新记录之前置位FDS忙标志
    err_code = fds_record_update(&desc, &device_name_record);
    //根据函数返回值判断是否需要碎片收集
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;//碎片收集标志置位
      my_fds_info.busy = false;//FDS忙标志清零
    }
    else
    {
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
  }
  return NRF_SUCCESS;
}

static ret_code_t fds_save_weightdata (void)
{
	ret_code_t              err_code;
  //定义并初始化记录描述符结构体变量
  fds_record_desc_t desc = {0};	
  //定义并初始化记录查找令牌结构体变量
  fds_find_token_t  tok  = {0};
  //清零tok,从头查找
  memset(&tok, 0x00, sizeof(fds_find_token_t));
	//在DEVICE_FILE文件中查找记录m_desp_record
  err_code = fds_record_find(DATA_FILE, DATA_REC_KEY, &desc, &tok);
  memcpy(&weight_data, &weight_data_info.data, sizeof(weight_data));
  //没有查找到m_desp_record记录,写入记录
  if(err_code != NRF_SUCCESS)
  {
    my_fds_info.busy = true;//写记录之前置位FDS忙标志
    err_code = fds_record_write(&desc, &weight_data_record);
    //根据函数返回值判断是否需要碎片收集
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;
      my_fds_info.busy = false;
		}
		else
		{
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
    NRF_LOG_INFO("write");
  }
  else//设备名称记录已经存在,更新记录
  {
    my_fds_info.busy = true;//更新记录之前置位FDS忙标志
    err_code = fds_record_update(&desc, &weight_data_record);
    //根据函数返回值判断是否需要碎片收集
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;//碎片收集标志置位
      my_fds_info.busy = false;//FDS忙标志清零
    }
    else
    {
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
    NRF_LOG_INFO("flash");
  }
  return NRF_SUCCESS;
}

//从Flash中查找是否存在有效的设备名称
static uint32_t find_device_name(void)
{
  uint32_t *data;
  ret_code_t rc = FDS_ERR_NOT_FOUND;
  //定义并初始化记录描述符结构体变量
  fds_record_desc_t desc = {0};
  //定义并初始化记录查找令牌结构体变量
  fds_find_token_t  tok  = {0};
  //清零tok,从头查找
  memset(&tok, 0x00, sizeof(fds_find_token_t));
  //在DEVICE_FILE文件中查找记录m_version_record
  rc = fds_record_find(NAME_FILE, NAME_REC_KEY, &desc, &tok);
  //查找到记录后,读取记录内容
  if(rc == NRF_SUCCESS)
  {
    fds_flash_record_t temp = {0};
    //打开记录读取记录内容
		rc = fds_record_open(&desc, &temp);
		APP_ERROR_CHECK(rc);
		data = (uint32_t *) temp.p_data;
    //读取设备名称
    for (uint8_t i=0;i<temp.p_header->length_words;i++)
    {
      device_name_info.name.name32[i] = data[i];
    }				
    //读取后,关闭记录
    rc = fds_record_close(&desc);
    APP_ERROR_CHECK(rc);
  }
  return rc;
}

//从Flash中查找是否存在有效的重量数据
static uint32_t find_weight_data(void)
{
  uint32_t *data;
  ret_code_t rc = FDS_ERR_NOT_FOUND;
  //定义并初始化记录描述符结构体变量
  fds_record_desc_t desc = {0};
  //定义并初始化记录查找令牌结构体变量
  fds_find_token_t  tok  = {0};
  //清零tok,从头查找
  memset(&tok, 0x00, sizeof(fds_find_token_t));
  //在DEVICE_FILE文件中查找记录m_version_record
  rc = fds_record_find(DATA_FILE, DATA_REC_KEY, &desc, &tok);
  //查找到记录后,读取记录内容
  if(rc == NRF_SUCCESS)
  {
    fds_flash_record_t temp = {0};
    //打开记录读取记录内容
    rc = fds_record_open(&desc, &temp);
    APP_ERROR_CHECK(rc);
		data = (uint32_t *) temp.p_data;
    //读取设备名称
    for(uint8_t i=0;i<temp.p_header->length_words;i++)
    {
      weight_data_info.data.data32[i] = data[i];
    }				
    //读取后,关闭记录
    rc = fds_record_close(&desc);
    APP_ERROR_CHECK(rc);
  }
  return rc;
}

//设置GAP设备名称:如果Flash中已经存储了设备名称,读出名称进行设置,否则使用默认的名称
void device_name_set(void)
{
  ret_code_t              err_code;
  ble_gap_conn_sec_mode_t sec_mode;
  //设置GAP的安全模式
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
  //Flash中查找到了有效的设备名称,则使用该设备名称
  if(find_device_name() == NRF_SUCCESS)
  {
    if(device_name_info.name.name32[0] == NAME_FLASH_TYPE_NAME)
    {
      err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)(&device_name_info.name.name8[8]),device_name_info.name.name32[1]);
      APP_ERROR_CHECK(err_code);
      device_name_info.name_exist = true;      //标志置位,表示Flash中已经存储了设备名称。再次修改的时候更新即可而不需重新写
    }	
    else device_name_info.name_exist = false;
  }
  //Flash中没有查找到了有效的设备名称,使用默认的设备名称
  if(device_name_info.name_exist == false)
  {
    err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *) DEFAULT_DEVICE_NAME,strlen(DEFAULT_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
  }
}

//设置重量数据:如果Flash中已经存储了重量数据,读出重量进行设置,否则使用默认的重量
void weight_data_set(void)
{
	ble_gap_conn_sec_mode_t sec_mode;
  //设置GAP的安全模式
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
	//Flash中查找到了有效的重量数据,则使用该重量数据
	if(find_weight_data() == NRF_SUCCESS)
	{
		if(weight_data_info.data.data32[0] == DATA_FLASH_TYPE_NAME)
		{
      //读取数据
			weight_data_info.data_exist = true;      //标志置位,表示Flash中已经存储了重量数据。再次修改的时候更新即可而不需重新写
		}	
		else weight_data_info.data_exist = false;
	}
	//Flash中没有查找到了有效的重量数据,使用默认的重量数据
	if(weight_data_info.data_exist == false)
	{
    //读取数据时使用默认数据
		NO_Data = 1;
	}
}
//更新广播内容
static void adv_updata(void)
{
	ret_code_t err_code;
	ble_advdata_t           adv_data; //广播数据
  ble_advdata_t           sr_data;  //扫描响应数据
	
	//配置之前先清零
	memset(&adv_data, 0, sizeof(adv_data));
	memset(&sr_data, 0, sizeof(sr_data));
	//包含设备名称:全称
	adv_data.name_type               = BLE_ADVDATA_FULL_NAME;
	//是否包含外观:包含
  adv_data.include_appearance      = false;
	//Flag:一般可发现模式,不支持BR/EDR
  adv_data.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
			
  //UUID放到扫描响应里面
  sr_data.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  sr_data.uuids_complete.p_uuids  = m_adv_uuids;
  //更新广播内容
  err_code = ble_advertising_advdata_update(&m_advertising, &adv_data, &sr_data);
  APP_ERROR_CHECK(err_code);
}
void ble_gatt_params_on_ble_evt(ble_evt_t const * p_ble_evt)
{
	//定义一个GATTS写事件结构体并指向BLE事件结构体中的写事件,方便取值
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  //如果事件类型是BLE_GATTS_EVT_WRITE并且UIID是GAP服务的设备名称特征的UUID,即为写GAP设备名称特征
	if((p_evt_write->uuid.uuid == BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME) && (p_ble_evt->header.evt_id == BLE_GATTS_EVT_WRITE))
  {
    //拷贝设备名称 
    memcpy(&device_name_info.name.name8[8], p_evt_write->data, p_evt_write->len);
    //第一个字设置为名称类型
    device_name_info.name.name32[0] = NAME_FLASH_TYPE_NAME;
    //第二个字设置为名称的长度
    device_name_info.name.name32[1] = p_evt_write->len;
    //更新广播内容
    adv_updata();
    //设备名称保存标志置位,主循环中根据该标志判断是否需要将设备名称存储到Flash
    device_name_info.save_name_flag = true; 
  }	
}

//广播事件处理函数
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  //判断广播事件类型
  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST://快速广播启动事件:快速广播启动后会产生该事件
         NRF_LOG_INFO("Fast advertising.");
         break;
    case BLE_ADV_EVT_IDLE://广播IDLE事件:广播超时后会产生该事件
         break;
    default:
         break;
  }
}
//广播初始化
void advertising_init(void)
{
  ret_code_t             err_code;
  //定义保存发射功率等级的变量
  int8_t tx_power_level = BLE_ADV_PW_LEVEL;
  //定义广播初始化配置结构体变量
  ble_advertising_init_t init;
  //配置之前先清零
  memset(&init, 0, sizeof(init));
  //设备名称类型:全称
  init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  //是否包含外观:包含
  init.advdata.include_appearance      = false;
  //发送功率等级
  init.advdata.p_tx_power_level = &tx_power_level;
  //Flag:一般可发现模式,不支持BR/EDR
  init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  //UUID放到扫描响应里面
  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
	
  //设置广播模式为快速广播
  init.config.ble_adv_fast_enabled  = true;
  //设置广播间隔和广播持续时间
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
  //广播事件回调函数
  init.evt_handler = on_adv_evt;
  //初始化广播
  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);
  //设置广播配置标记。APP_BLE_CONN_CFG_TAG是用于跟踪广播配置的标记,这是为未来预留的一个参数,在将来的SoftDevice版本中,
  //可以使用sd_ble_gap_adv_set_configure()配置新的广播配置
  //当前SoftDevice版本（S112 V7.2.0版本）支持的最大广播集数量为1,因此APP_BLE_CONN_CFG_TAG只能写1。
  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
  //设置广播的发射功率
  //nRF52832可设置的值为:4,3,0,-4,-8,-12,-16,-20,-30,-40
  //nRF52840可设置的值为:8,7,6,5,4,3,2,0,-4,-8,-12,-16,-20,-30,-40
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, BLE_ADV_PW_LEVEL);
  APP_ERROR_CHECK(err_code);
}
//启动广播,该函数所用的模式必须和广播初始化中设置的广播模式一样
void advertising_start(void)
{
  //使用广播初始化中设置的广播模式启动广播
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  //检查函数返回的错误代码
  APP_ERROR_CHECK(err_code);
}
//处理设备名称存储和碎片收集
void device_name_handle(void)
{
  ret_code_t rc;
  if(my_fds_info.gc == true)//碎片收集标志置位,表示FDS的记录已满,需要执行碎片收集
  {
    if(my_fds_info.busy == false)
    {
      my_fds_info.busy = true;
      rc = fds_gc();//FDS碎片回收
      APP_ERROR_CHECK(rc);//用错误处理模块检查函数返回值
      wait_for_fds_ready();
				
      my_fds_info.gc = false;
      //碎片收集完成后,将保存设备名称标志置位,保存本次的设备名称
      device_name_info.save_name_flag = true;
    }
  }
  else//存储设备名称标志置位,将设备名称存储到Flash
  {
    if(device_name_info.save_name_flag == true)
    {
      if(my_fds_info.busy == false)
      {
        //写/更新设备名称记录
        fds_save_devicename();
        //存储设备名称标志清零
        device_name_info.save_name_flag = false;
      }
    }
  }
}
//处理重量数据存储和碎片收集
void weight_data_handle(void)
{
	ret_code_t rc;
	if(my_fds_info.gc == true)//碎片收集标志置位,表示FDS的记录已满,需要执行碎片收集
	{
    if(my_fds_info.busy == false)
    {
      my_fds_info.busy = true;
      rc = fds_gc();//FDS碎片回收
      APP_ERROR_CHECK(rc);//用错误处理模块检查函数返回值
      wait_for_fds_ready();
      my_fds_info.gc = false;
      //碎片收集完成后,将保存重量数据标志置位,保存本次的重量数据
      weight_data_info.save_data_flag = true;
    }
  }
  else//存储重量数据标志置位,将重量数据存储到Flash
  {
    if(weight_data_info.save_data_flag == true)
    {
      if(my_fds_info.busy == false)
      {
        //写/更新重量数据记录
        fds_save_weightdata();
        //存储重量数据标志清零
        weight_data_info.save_data_flag = false;
      }
    }
  }
}
