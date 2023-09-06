//引用的C库头文件
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log需要引用的头文件
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//APP定时器需要引用的头文件
#include "app_timer.h"
#include "bsp_btn_ble.h"
//广播需要引用的头文件
#include "ble_advdata.h"
#include "ble_advertising.h"
//电源管理需要引用的头文件
#include "nrf_pwr_mgmt.h"
//SoftDevice handler configuration需要引用的头文件
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
//排序写入模块需要引用的头文件
#include "nrf_ble_qwr.h"
//GATT需要引用的头文件
#include "nrf_ble_gatt.h"
//连接参数协商需要引用的头文件
#include "ble_conn_params.h"
//串口透传需要引用的头文件
#include "my_ble_uarts.h"
//引用FDS头文件
#include "fds.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_uart.h"
#include "device_name_op.h"
#include "nrf_delay.h"
#include "nrfx_wdt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"

#define AD_SCK          NRF_GPIO_PIN_MAP(0,5)
#define AD_DOUT1        NRF_GPIO_PIN_MAP(0,6)
#define AD_DOUT2        NRF_GPIO_PIN_MAP(0,12)
#define CHG_IDLE        NRF_GPIO_PIN_MAP(0,15)
#define CHG_ING         NRF_GPIO_PIN_MAP(0,16)
#define LED_GN          NRF_GPIO_PIN_MAP(0,18)
#define LED_RD          NRF_GPIO_PIN_MAP(0,20)
#define POW_BTN         NRF_GPIO_PIN_MAP(0,28)
#define POW_OUT1        NRF_GPIO_PIN_MAP(0,30)
#define POW_OUT2        NRF_GPIO_PIN_MAP(0,14)

uint8_t Reset_Count = 0;
uint8_t BattType = 0;
uint8_t Sys_State = 0;
uint8_t Charge_State = 0;
uint8_t Last_Charge_State = 0;
uint8_t Charge_Ing = 0;
uint8_t Charge_Flash = 0;
uint8_t Charge_Count = 0;
uint8_t Low_Power = 0;
uint8_t Batt_Count = 0;
uint8_t Lost_Count = 0;
uint8_t Ack_Flag = 0;
uint8_t Serial_Flag = 0;
uint8_t Cmd_Flag = 0;
uint8_t Set_Data[5] = {0};
uint8_t Led_Flash = 0;
uint8_t NO_Data = 0;
int16_t Batt_Cap = 60;
int16_t Weight_Data[5] = {0};
int16_t Adc_Value = 370;
int16_t Sleep_Count = 0;
uint8_t SENSOR_PW = 0;
uint8_t Sensor_ID = 0;
uint8_t Avg_Count = 0;
uint8_t Avg_Timer = 0;
int16_t Max_Adc_Value = 0;
uint16_t Charge_Over = 0;

#define SLEEP_VALUE 600 //600s=10min

extern void Avg_Filter(void);
extern void Read_Data(void);
extern unsigned long Uart_Send(unsigned char,unsigned char);
extern void Save_Data(unsigned char,unsigned char);
extern void Clear_Data(void);
extern void Init_Data(void);
extern void Lock_Data(void);

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(16, UNIT_1_25_MS)    //最小连接间隔
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(32, UNIT_1_25_MS)    //最大连接间隔
#define SLAVE_LATENCY                   0                                  //从机延迟 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(500, UNIT_10_MS)     //监督超时
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)              //定义首次调用sd_ble_gap_conn_param_update()函数更新连接参数延迟时间（5秒）
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)             //定义每次调用sd_ble_gap_conn_param_update()函数更新连接参数的间隔时间（30秒）
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                  //定义放弃连接参数协商前尝试连接参数协商的最大次数（3次）

#define APP_BLE_OBSERVER_PRIO           3               //应用程序BLE事件监视者优先级,应用程序不能修改该数值

#define UART_TX_BUF_SIZE                256             //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE                256             //串口接收缓存大小（字节数）

//用于stack dump的错误代码,可以用于栈回退时确定堆栈位置
#define DEAD_BEEF                       0xDEADBEEF     
#define BLE_CONN_PW_LEVEL               4
               
BLE_UARTS_DEF(m_uarts, NRF_SDH_BLE_TOTAL_LINK_COUNT);    //定义名称为m_uarts的串口透传服务实例
NRF_BLE_GATT_DEF(m_gatt);                                //定义名称为m_gatt的GATT模块实例
NRF_BLE_QWR_DEF(m_qwr);                                  //定义一个名称为m_qwr的排队写入实例

APP_TIMER_DEF(m_app_timer_id);
#define APP_TIMER_INTERVAL APP_TIMER_TICKS(100)
APP_TIMER_DEF(m_avg_timer_id);
#define AVG_TIMER_INTERVAL APP_TIMER_TICKS(10)

nrfx_wdt_channel_id m_channel_id;
nrf_saadc_value_t  saadc_val;

//该变量用于保存连接句柄,初始值设置为无连接
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
//发送的最大数据长度
static uint16_t   m_ble_uarts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;

void Set_Led(uint8_t state)
{
  if(state == 0)  //待机状态
  {
    if(Charge_Ing == 1)
    {
      nrf_gpio_pin_set(LED_RD);
      nrf_gpio_pin_set(LED_GN);
    }
    else
    {
      nrf_gpio_pin_clear(LED_GN);
      nrf_gpio_pin_clear(LED_RD);
    }
  }	
  if(state == 1)  //广播状态
  {
    if(Charge_Ing == 1)
    {
      nrf_gpio_pin_set(LED_RD);
      nrf_gpio_pin_set(LED_GN);
    }
    else if(Low_Power == 0) 
    {
      nrf_gpio_pin_set(LED_GN);
      nrf_gpio_pin_clear(LED_RD);
    }
    else if(Low_Power == 1) 
    {
      nrf_gpio_pin_set(LED_RD);
      nrf_gpio_pin_clear(LED_GN);
    }
  }
  if(state == 2 || state == 3) //连接状态
  {
    if(Charge_Ing == 1)
    {
      if(Charge_Flash == 1)
      {
        nrf_gpio_pin_set(LED_RD);
        nrf_gpio_pin_set(LED_GN);
        Charge_Flash = 0;
      }			
      else if(Charge_Flash == 0)
      {
        nrf_gpio_pin_clear(LED_RD);
        nrf_gpio_pin_clear(LED_GN);
        Charge_Flash = 1;
      }
    }
    else if(Low_Power == 0) 
    {
      nrf_gpio_pin_toggle(LED_GN);
      nrf_gpio_pin_clear(LED_RD);
    }
    else if(Low_Power == 1) 
    {
      nrf_gpio_pin_toggle(LED_RD);
      nrf_gpio_pin_clear(LED_GN);
    }
  }
}

//GAP参数初始化,该函数配置需要的GAP参数,包括设备名称,外观特征、首选连接参数
static void gap_params_init(void)
{
  ret_code_t err_code;
  //定义连接参数结构体变量
  ble_gap_conn_params_t   gap_conn_params;
  //设置设备名称:从FLash中查找设备名称记录,如果设备名称有效,使用该设备名称设置GAP设备名称,否则使用默认的设备名称
  device_name_set();																																					
  //设置首选连接参数,设置前先清零gap_conn_params
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));
  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;//最小连接间隔
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;//最小连接间隔
  gap_conn_params.slave_latency     = SLAVE_LATENCY;    //从机延迟
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; //监督超时
  //调用协议栈API sd_ble_gap_ppcp_set配置GAP参数
  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}
//GATT事件处理函数,该函数中处理MTU交换事件
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
}

//初始化GATT程序模块
static void gatt_init(void)
{
  //初始化GATT程序模块
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  //检查函数返回的错误代码
  APP_ERROR_CHECK(err_code);
  //设置ATT MTU的大小,这里设置的值为247
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

//排队写入事件处理函数,用于处理排队写入模块的错误
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  //检查错误代码
  NRF_LOG_INFO("App Reset 4");
  NVIC_SystemReset();
  APP_ERROR_HANDLER(nrf_error);
}

//串口事件回调函数,串口初始化时注册,该函数中判断事件类型并进行处理
//当接收的数据长度达到设定的最大值或者接收到换行符后,则认为一包数据接收完成,之后将接收的数据发送给主机
void uart_event_handle(app_uart_evt_t * p_event)
{
  static uint8_t data_array[BLE_UARTS_MAX_DATA_LEN];
  static uint8_t index = 0;
  //判断事件类型
  switch (p_event->evt_type)
  {
    case APP_UART_DATA_READY://串口接收事件
         UNUSED_VARIABLE(app_uart_get(&data_array[index]));
         index++;
         //接收串口数据，当接收的数据长度达到m_ble_uarts_max_data_len或者接收到换行符后认为一包数据接收完成
         if((data_array[index - 1] == '4') || (index >= m_ble_uarts_max_data_len))
         {
           if(index > 1)
           {			
             if(data_array[0] == ':' && data_array[1] == '1' && data_array[2] == 'A' && data_array[3] == '1' && data_array[4] == '4') //读取重量
             {
               Serial_Flag = 1;
             }
             if(data_array[0] == ':' && data_array[1] == '1' && data_array[2] == 'A' && data_array[3] == '3' && data_array[4] == '4') //读取设备信息
             {
               Serial_Flag = 20;
             }
           }
           index = 0;
         }
         break;
    //通讯错误事件，进入错误处理
    case APP_UART_COMMUNICATION_ERROR:
         NRF_LOG_INFO("App Reset 1");
         NVIC_SystemReset();
         APP_ERROR_HANDLER(p_event->data.error_communication);
         break;
    //FIFO错误事件，进入错误处理
    case APP_UART_FIFO_ERROR:
         NRF_LOG_INFO("App Reset 2");
         NVIC_SystemReset();
         APP_ERROR_HANDLER(p_event->data.error_code);
         break;
    default:
         break;
  }
}
//串口配置
void uart_config(void)
{
  uint32_t err_code;
  //定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,                 //定义uart接收引脚
    TX_PIN_NUMBER,                 //定义uart发送引脚
    RTS_PIN_NUMBER,                //定义uart RTS引脚,流控关闭后虽然定义了RTS和CTS引脚,但是驱动程序会忽略,不会配置这两个引脚,两个引脚仍可作为IO使用
    CTS_PIN_NUMBER,                //定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,                         //禁止奇偶检验
    NRF_UART_BAUDRATE_115200       //uart波特率设置为115200
  };
  //初始化串口，注册串口事件回调函数
  APP_UART_FIFO_INIT(&comm_params,UART_RX_BUF_SIZE,UART_TX_BUF_SIZE,uart_event_handle,APP_IRQ_PRIORITY_LOWEST,err_code);
  APP_ERROR_CHECK(err_code);
}

uint8_t ToHexStr(uint8_t num)
{
  if(num < 10)
  {
    return num + 48;
  }
  if(num >=10 && num < 16)
  {
    return num + 55;
  }
  return 48;
}

uint16_t time[3] = {0};
void Send_Edition()
{
  uint8_t Date[12] = __DATE__;
  time[0] = (Date[7] - '0')*1000 + (Date[8] - '0')*100 + (Date[9] - '0')*10 + (Date[10] - '0');
  if(Date[0] == 'J' && Date[1] == 'a' && Date[2] == 'n') time[1] = 1;
  if(Date[0] == 'F' && Date[1] == 'e' && Date[2] == 'b') time[1] = 2;
  if(Date[0] == 'M' && Date[1] == 'a' && Date[2] == 'r') time[1] = 3;
  if(Date[0] == 'A' && Date[1] == 'p' && Date[2] == 'r') time[1] = 4;
  if(Date[0] == 'M' && Date[1] == 'a' && Date[2] == 'y') time[1] = 5;
  if(Date[0] == 'J' && Date[1] == 'u' && Date[2] == 'n') time[1] = 6;
  if(Date[0] == 'J' && Date[1] == 'u' && Date[2] == 'l') time[1] = 7;
  if(Date[0] == 'A' && Date[1] == 'u' && Date[2] == 'g') time[1] = 8;
  if(Date[0] == 'S' && Date[1] == 'e' && Date[2] == 'p') time[1] = 9;
  if(Date[0] == 'O' && Date[1] == 'c' && Date[2] == 't') time[1] = 10; 
  if(Date[0] == 'N' && Date[1] == 'o' && Date[2] == 'v') time[1] = 11;
  if(Date[0] == 'D' && Date[1] == 'e' && Date[2] == 'c') time[1] = 12;
  if(Date[4] > '0')
  {
    time[2] = (Date[4] - '0')*10 + (Date[5] - '0');
  }
  else
  {
    time[2] = (Date[5] - '0');
  }
  NRF_LOG_INFO("%d-%d-%d",time[0],time[1],time[2]);
}

static void m_avg_timeout_handler(void *p_context)
{
  Avg_Timer++;
  if(Avg_Timer == 8)
  {
    Avg_Timer = 0;
    if(SENSOR_PW == 1)
    {
      Avg_Count++;
      if(Avg_Count == 1) Avg_Filter();
      if(Avg_Count == 2) Avg_Filter();
      if(Avg_Count >= 2) Avg_Count = 0;
    }
    else
    {
      nrf_gpio_pin_set(AD_SCK);
    }
    Read_Data();
  }
  if(SENSOR_PW == 1)
  {
    for(uint8_t i=1; i<=4; i++)
    {
      Weight_Data[i] = Weight_Data[i]*3/4 + Uart_Send(i,0)*1/4;
    }
  }
}
static void m_app_timeout_handler(void *p_context)
{
  UNUSED_PARAMETER(p_context); //下行
  uint8_t send_data[40]={0};
  uint8_t send_length = 0;
  uint32_t err_code;

  if(nrf_gpio_pin_read(POW_BTN) == 0)
  {
    if(Reset_Count == 100)
    {
      NRF_LOG_INFO("Btn Reset!");
    }
    if(Reset_Count++ > 110)
    {
      NVIC_SystemReset();
    }
  }
  else
  {
    Reset_Count = 0;
  }
  nrfx_wdt_channel_feed(m_channel_id);//喂狗
  if(NO_Data == 1)
  {
    NRF_LOG_INFO("No Find Data!-%d ",Batt_Count);
    SENSOR_PW = 1;
    if(Batt_Count > 90)
    {
      NO_Data = 0;
      Init_Data();
      NRF_LOG_INFO("Init Data!");
    }
  }
  Batt_Count++;
  if(Batt_Count % 10 == 0)
  {
    if(Adc_Value <= 310 && Charge_State == 4)
    {
      Sleep_Count += 20;
    }
    else if(Sys_State <= 1 && Charge_State == 4)
    {
      Sleep_Count++;
    }
    else
    {
      Sleep_Count = 0;
    }
    if(Sleep_Count > SLEEP_VALUE)//进入睡眠
    {
      Sleep_Count = 0;
      Sys_State = 0;
      Set_Led(Sys_State);
      sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      ble_adv_stop();
      nrf_gpio_pin_clear(POW_OUT1);
      nrf_gpio_pin_clear(POW_OUT2);
    }
    Weight_Data[0] = Weight_Data[1] + Weight_Data[2] + Weight_Data[3] + Weight_Data[4];
    Send_Edition();
    NRF_LOG_INFO("[Adc %d] [Cap %d] [CHG %d] [Sys %d] [Lost %d] [Weight %d] ",Adc_Value,Batt_Cap,Charge_State,Sys_State,Lost_Count,Weight_Data[0]);
    NRF_LOG_INFO("[Weight %d %d %d %d ]",Weight_Data[1],Weight_Data[2],Weight_Data[3],Weight_Data[4]);
  }
  if(Batt_Count > 100)
  {
    Batt_Count = 0;
  }
  if(Batt_Count == 10)
  {
    if(Sys_State > 0)
    {
      nrfx_saadc_sample_convert(0,&saadc_val);//读电池电量
    }
    if(saadc_val != 0) Adc_Value = (saadc_val-495)*70/98+350;
    if(BattType == 0)
    {
      if(Charge_Ing == 1)//充电下的电量计算
      {
        Adc_Value -= 10;  //基准修正
      }
      if(Adc_Value < 351 && Adc_Value != 0) Batt_Cap = 1;
      else if(Adc_Value < 357) Batt_Cap = 10;
      else if(Adc_Value < 363) Batt_Cap = 20;
      else if(Adc_Value < 370) Batt_Cap = 40;
      else if(Adc_Value < 380) Batt_Cap = 60;
      else if(Adc_Value < 395) Batt_Cap = 80;
      else Batt_Cap = 100;
    }
    if(Batt_Cap <= 10)		//电量低置报警位
    {
      Low_Power = 1;
    }
    else
    {
      Low_Power = 0;
    }
  }
  if(nrf_gpio_pin_read(CHG_ING) == 0 && nrf_gpio_pin_read(CHG_IDLE) == 0)
  {
    Charge_State = 1;
  }
  if(nrf_gpio_pin_read(CHG_ING) == 0 && nrf_gpio_pin_read(CHG_IDLE) == 1)
  {
    Charge_State = 2;//充电
  }
  if(nrf_gpio_pin_read(CHG_ING) == 1 && nrf_gpio_pin_read(CHG_IDLE) == 0)
  {
    Charge_State = 3;//充满
  }
  if(nrf_gpio_pin_read(CHG_ING) == 1 && nrf_gpio_pin_read(CHG_IDLE) == 1)
  {
    Charge_State = 4;//拔出
  }
  if(Charge_State != Last_Charge_State)
  {
    if(Charge_State == 2)//插入时显示橙色灯
    {
      Charge_Ing = 1;
      Set_Led(Sys_State);
    }
    if(Charge_State == 3)//充满时
    {
    }
    if(Charge_State == 4)
    {
      if(Last_Charge_State == 2 || Last_Charge_State == 3)//拔出时
      {
        if(Sys_State <= 1)//非工作状态关机
        {
          Charge_Ing = 0;
          Sys_State = 0;
          Set_Led(Sys_State);
          sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
          ble_adv_stop();
          nrf_gpio_pin_clear(POW_OUT1);
          nrf_gpio_pin_clear(POW_OUT2);
        }
      }
    }
    Last_Charge_State = Charge_State;
  }
  if(Sys_State <= 1)
  {
    Charge_Count++;
    if(Charge_Count >= 20)//蓝牙断开时每2s刷新一次
    {
      Set_Led(Sys_State);
      Charge_Count = 0;
      if(Charge_State != 2)
      {
        Charge_Ing = 0;
        Set_Led(Sys_State);
      }
      if(Adc_Value >= 400 && Charge_State == 2)
      {
        if(Adc_Value <= Max_Adc_Value)
        {
          Charge_Over++;
        }
        else
        {
          Max_Adc_Value = Adc_Value;
          Charge_Over = 0;
        }
        if(Charge_Over > 400)//400*2s=800s=13min
        {
          Charge_Ing = 0;
          Set_Led(Sys_State);
          Charge_Over = 0;
          Max_Adc_Value = 0;
        }
      }
      else
      {
        Charge_Over = 0;
        Max_Adc_Value = 0;				
      }
    }
  }
  if(Sys_State == 2 || Sys_State == 3)
  {
    Led_Flash++;
    if(Led_Flash >= 5)
    {
      Set_Led(Sys_State);//蓝牙连接时每0.5s刷新一次
      Led_Flash = 0;
    }
    if(Charge_State != 4)
    {
      Charge_Ing = 1;
    }
    else
    {
      Charge_Ing = 0;
    }
  }
  if(Cmd_Flag > 0 || (Ack_Flag > 0 && Ack_Flag < 18))
  {
    if(Ack_Flag == 5) Sys_State = 2;
    Lost_Count = 0;
    SENSOR_PW = 1;
  }
  if(Serial_Flag > 0 && Serial_Flag < 18)
  {
    if(Serial_Flag == 5) Sys_State = 3;
    Lost_Count = 0;
    SENSOR_PW = 1;
  }
  if(Lost_Count < 120)
  {
    Lost_Count++;
  }
  else
  {
    Lost_Count = 100;
    SENSOR_PW = 0;
  }
  if(Lost_Count > 5)
  {
    if(Sys_State == 3)
    {
      Sys_State = 1;
      Serial_Flag = 0;
    }
  }
  if(Cmd_Flag == 1) //标定
  {
    Save_Data(Sensor_ID,Set_Data[Sensor_ID]);
    NRF_LOG_INFO("Set %d-%d ",Sensor_ID,Set_Data[Sensor_ID]);
    Cmd_Flag = 0;
  }
  if(Cmd_Flag == 2) //去皮
  {
    Clear_Data();
    NRF_LOG_INFO("Clear Data!");
    Cmd_Flag = 0;
  }
  if((Ack_Flag > 0 && Ack_Flag < 18) || (Serial_Flag > 0 && Serial_Flag < 18)) //串口接收的数据发送给BLE主机//上行数据
  {
    send_length = 29;
    send_data[0] = ':';send_data[1] = '1';send_data[2] = 'A';send_data[3] = 'A';send_data[4] = '4';send_data[5] = '0';send_data[6] = '4';send_data[7] = '0';send_data[8] = '0';                       
    for (uint8_t i = 0; i < 4; i++)
    {
      if(Weight_Data[i] < 0) Weight_Data[i] = 0;
    }
    send_data[9] = ToHexStr(Weight_Data[1]/4096);       
    send_data[10] = ToHexStr(Weight_Data[1]%4096/256); 
    send_data[11] = ToHexStr(Weight_Data[1]%256/16);
    send_data[12] = ToHexStr(Weight_Data[1]%16);
		 
    send_data[13] = ToHexStr(Weight_Data[2]/4096);       
    send_data[14] = ToHexStr(Weight_Data[2]%4096/256); 
    send_data[15] = ToHexStr(Weight_Data[2]%256/16);
    send_data[16] = ToHexStr(Weight_Data[2]%16);

    send_data[17] = ToHexStr(Weight_Data[3]/4096);       
    send_data[18] = ToHexStr(Weight_Data[3]%4096/256); 
    send_data[19] = ToHexStr(Weight_Data[3]%256/16);
    send_data[20] = ToHexStr(Weight_Data[3]%16);
		 
    send_data[21] = ToHexStr(Weight_Data[4]/4096);       
    send_data[22] = ToHexStr(Weight_Data[4]%4096/256); 
    send_data[23] = ToHexStr(Weight_Data[4]%256/16);
    send_data[24] = ToHexStr(Weight_Data[4]%16);
	
    send_data[25] = '0';//校验
    send_data[26] = '0';//校验
    send_data[27] = 0x0D;
    send_data[28] = 0x0A;
    if(Ack_Flag > 0) Ack_Flag++;
    if(Serial_Flag > 0) Serial_Flag++;
  }
  if(Ack_Flag == 20 || Serial_Flag == 20)
  {
    send_length = 19;
    send_data[0] = ':';send_data[1] = '1';send_data[2] = 'A';send_data[3] = 'A';send_data[4] = '4';send_data[5] = '0';send_data[6] = '4';send_data[7] = '0';send_data[8] = '0';
    send_data[9] = '6';send_data[10] = '5';send_data[11] = '6';send_data[12] = '5';
    send_data[13] = ToHexStr(Batt_Cap/16);
    send_data[14] = ToHexStr(Batt_Cap%16);
		
    send_data[15] = '1';
    send_data[16] = '5';
    send_data[17] = 0x0D;
    send_data[18] = 0x0A;
  }
  if(Ack_Flag == 21)
  {
    send_length = 18;
    send_data[0] = ':';send_data[1] = '1';send_data[2] = 'A';send_data[3] = '5';send_data[4] = '4';
    send_data[5] = ToHexStr(time[0]%100/10);send_data[6] = ToHexStr(time[0]%100%10);//年
    send_data[7] = ToHexStr(time[1]/10);send_data[8] = ToHexStr(time[1]%10);        //月
    send_data[9] = ToHexStr(time[2]/10);send_data[10] = ToHexStr(time[2]%10);       //日
    send_data[11] = ToHexStr(1);send_data[12] = ToHexStr(0);send_data[13] = ToHexStr(0);send_data[14] = ToHexStr(0);send_data[15] = ToHexStr(0);
    send_data[16] = 0x0D;
    send_data[17] = 0x0A;
  }
  if(Ack_Flag > 0)
  {
    if(Ack_Flag == 18 || Ack_Flag == 20 || Ack_Flag == 21)
    {
      Ack_Flag = 0;
    }
    do
    {						  
      uint16_t length = (uint16_t)send_length;
      err_code = ble_uarts_data_send(&m_uarts, send_data, &length, m_conn_handle);
      if((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    while(err_code == NRF_ERROR_RESOURCES);
  }
  if(Serial_Flag > 0)
  {
    if(Serial_Flag == 18 || Serial_Flag == 20)
    {
      Serial_Flag = 0;
    }
    for(uint32_t i = 0; i < send_length; i++)
    {
      do
      {
        err_code = app_uart_put(send_data[i]);//下行数据		
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
        {
          APP_ERROR_CHECK(err_code);
        }
      }
      while (err_code == NRF_ERROR_BUSY);
    }
  }
}

//串口透传事件回调函数,串口透出服务初始化时注册
static void uarts_data_handler(ble_uarts_evt_t * p_evt)
{
  uint8_t cmd_data[20]={0};
  //判断事件类型:接收到新数据事件
  if(p_evt->type == BLE_UARTS_EVT_RX_DATA)
  {
    //串口打印出接收的数据
    for(uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
    {
      cmd_data[i] = p_evt->params.rx_data.p_data[i];
    }
    if(cmd_data[0] == 'c' && cmd_data[1] == 'l' && cmd_data[2] == 'e' && cmd_data[3] == 'a' && cmd_data[4] == 'r')
    {
      Cmd_Flag = 2;
    }
    if(cmd_data[0] == 's' && cmd_data[1] == 'e' && cmd_data[2] == 't') //标定命令
    {
      Sensor_ID = cmd_data[4] - '0';
      if(cmd_data[6] == '0') //标定置零
      {
        Set_Data[Sensor_ID] = 0;
        Cmd_Flag = 1;
      }
      if(cmd_data[6] > '0' && cmd_data[6] <= '9' && cmd_data[7] == 0) //标定重量一位数字
      {
        Set_Data[Sensor_ID] = (cmd_data[6] - '0')*1;
        Cmd_Flag = 1;
      }
      if(cmd_data[6] > '0' && cmd_data[6] <= '9' && cmd_data[7] >= '0' && cmd_data[7] <= '9' && cmd_data[8] == 0) //标定重量两位数字
      {
        Set_Data[Sensor_ID] = (cmd_data[6] - '0')*10 + (cmd_data[7] - '0');
        Cmd_Flag = 1;
      }
      if(cmd_data[6] > '0' && cmd_data[6] <= '9' && cmd_data[7] >= '0' && cmd_data[7] <= '9' && cmd_data[8] >= '0' && cmd_data[8] <= '9' && cmd_data[9] == 0) //标定重量三位数字
      {
        Set_Data[Sensor_ID] = (cmd_data[6] - '0')*100 + (cmd_data[7] - '0')*10 + (cmd_data[8] - '0')*1;
        Cmd_Flag = 1;
      }
    }
    if(cmd_data[0] == ':' && cmd_data[1] == '1' && cmd_data[2] == 'A' && cmd_data[3] == '1' && cmd_data[4] == '4') //读取重量
    {
      Ack_Flag = 1;
    }
    if(cmd_data[0] == ':' && cmd_data[1] == '1' && cmd_data[2] == 'A' && cmd_data[3] == '3' && cmd_data[4] == '4') //读取设备信息
    {
      Ack_Flag = 20;
    }
    if(cmd_data[0] == ':' && cmd_data[1] == '1' && cmd_data[2] == 'A' && cmd_data[3] == '5' && cmd_data[4] == '4') //读取版本
    {
      Ack_Flag = 21;
    }
    if(p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
    {
      while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
  }
}
//服务初始化,包含初始化排队写入模块和初始化应用程序使用的服务
static void services_init(void)
{
  ret_code_t         err_code;
  //定义串口透传初始化结构体
  ble_uarts_init_t     uarts_init;
  //定义排队写入初始化结构体变量
  nrf_ble_qwr_init_t qwr_init = {0};
  //排队写入事件处理函数
  qwr_init.error_handler = nrf_qwr_error_handler;
  //初始化排队写入模块
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  //检查函数返回值
  APP_ERROR_CHECK(err_code);
  /*------------------以下代码初始化串口透传服务-------------*/
  //清零串口透传服务初始化结构体
  memset(&uarts_init, 0, sizeof(uarts_init));
  //设置串口透传事件回调函数
  uarts_init.data_handler = uarts_data_handler;
  //初始化串口透传服务
  err_code = ble_uarts_init(&m_uarts, &uarts_init);
  APP_ERROR_CHECK(err_code);
  /*------------------初始化串口透传服务-END-----------------*/
}

//连接参数协商模块事件处理函数
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  //判断事件类型,根据事件类型执行动作
  //连接参数协商失败,断开当前连接
  if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    NRF_LOG_INFO("CONN Failed");
  }
  //连接参数协商成功
  if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
  {
  //功能代码;
  }
}

//连接参数协商模块错误处理事件,参数nrf_error包含了错误代码,通过nrf_error可以分析错误信息
static void conn_params_error_handler(uint32_t nrf_error)
{
  //检查错误代码
  NRF_LOG_INFO("App Reset 3");
  NVIC_SystemReset();
  APP_ERROR_HANDLER(nrf_error);
}

//连接参数协商模块初始化
static void conn_params_init(void)
{
  ret_code_t             err_code;
  //定义连接参数协商模块初始化结构体
  ble_conn_params_init_t cp_init;
  //配置之前先清零
  memset(&cp_init, 0, sizeof(cp_init));
  //设置为NULL,从主机获取连接参数
  cp_init.p_conn_params                  = NULL;
  //连接或启动通知到首次发起连接参数更新请求之间的时间设置为5秒
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  //每次调用sd_ble_gap_conn_param_update()函数发起连接参数更新请求的之间的间隔时间设置为:30秒
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  //放弃连接参数协商前尝试连接参数协商的最大次数设置为:3次
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  //连接参数更新从连接事件开始计时
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  //连接参数更新失败不断开连接
  cp_init.disconnect_on_fail             = false;
  //注册连接参数更新事件句柄
  cp_init.evt_handler                    = on_conn_params_evt;
  //注册连接参数更新错误事件句柄
  cp_init.error_handler                  = conn_params_error_handler;
  //调用库函数（以连接参数更新初始化结构体为输入参数）初始化连接参数协商模块
  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}
//BLE事件处理函数
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code = NRF_SUCCESS;
  ble_gatt_params_on_ble_evt(p_ble_evt);
  //判断BLE事件类型,根据事件类型执行相应操作
  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_DISCONNECTED://断开连接事件
         m_conn_handle = BLE_CONN_HANDLE_INVALID;
         NRF_LOG_INFO("Disconnected. reason: 0x%04x",p_ble_evt->evt.gap_evt.params.disconnected.reason);
         if(Sys_State == 2)
         {
           Sys_State = 1;
           Set_Led(Sys_State);
         }
         //打印提示信息
         NRF_LOG_INFO("BLE Breaked!");
         Ack_Flag = 0;     //清空发送队列
         Cmd_Flag = 0;     //清空命令队列
         break;
    case BLE_GAP_EVT_CONNECTED://连接事件
         NRF_LOG_INFO("BLE Connected!");
         SENSOR_PW = 1;
         Ack_Flag = 0;     //清空发送队列
         Cmd_Flag = 0;     //清空命令队列
         Sys_State = 2;
         //保存连接句柄
         m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
         //将连接句柄分配给排队写入实例,分配后排队写入实例和该连接关联,这样,当有多个连接的时候,通过关联不同的排队写入实例,很方便单独处理各个连接
         err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
         APP_ERROR_CHECK(err_code);
         //设置连接的发射功率
         //nRF52832可设置的值为:4,3,0,-4,-8,-12,-16,-20,-30,-40
         //nRF52840可设置的值为:8,7,6,5,4,3,2,0,-4,-8,-12,-16,-20,-30,-40
         err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, BLE_CONN_PW_LEVEL);
         APP_ERROR_CHECK(err_code);
         break;
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST://PHY更新事件
         {
           ble_gap_phys_t const phys =
           {
             .rx_phys = BLE_GAP_PHY_AUTO,
             .tx_phys = BLE_GAP_PHY_AUTO,
           };
           //响应PHY更新规程
           err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
           APP_ERROR_CHECK(err_code);
         }
         break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST://安全参数请求事件
         //不支持配对
         err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
         APP_ERROR_CHECK(err_code);
    case BLE_GATTS_EVT_SYS_ATTR_MISSING://系统属性访问正在等待中
         //系统属性没有存储,更新系统属性
         err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
         APP_ERROR_CHECK(err_code);
         break;
    case BLE_GATTC_EVT_TIMEOUT://GATT客户端超时事件
         NRF_LOG_INFO("GATTC Timeout");
         break;
    case BLE_GATTS_EVT_TIMEOUT://GATT服务器超时事件
         NRF_LOG_INFO("GATTS Timeout");
         break;
    default:
         break;
  }
}

//初始化BLE协议栈
static void ble_stack_init(void)
{
  ret_code_t err_code;
  //请求使能SoftDevice,该函数中会根据sdk_config.h文件中低频时钟的设置来配置低频时钟
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  //定义保存应用程序RAM起始地址的变量
  uint32_t ram_start = 0;
  //使用sdk_config.h文件的默认参数配置协议栈,获取应用程序RAM起始地址,保存到变量ram_start
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);
  //使能BLE协议栈
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
  //注册BLE事件回调函数
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

//初始化电源管理模块
static void power_management_init(void)
{
  ret_code_t err_code;
  //初始化电源管理
  err_code = nrf_pwr_mgmt_init();
  //检查函数返回的错误代码
  APP_ERROR_CHECK(err_code);
}
//初始化APP定时器模块
static void timers_init(void)
{
  //初始化APP定时器模块
  ret_code_t err_code = app_timer_init();
  //检查返回值
  APP_ERROR_CHECK(err_code);
  //创建用户定时任务 
  err_code = app_timer_create(&m_app_timer_id,APP_TIMER_MODE_REPEATED,m_app_timeout_handler);
  APP_ERROR_CHECK(err_code);
  //创建采样定时任务
  err_code = app_timer_create(&m_avg_timer_id,APP_TIMER_MODE_REPEATED,m_avg_timeout_handler);
  APP_ERROR_CHECK(err_code);
  //启动用户定时任务
  err_code = app_timer_start(m_app_timer_id,APP_TIMER_INTERVAL,NULL);
  APP_ERROR_CHECK(err_code);
  //启动采样定时任务
  err_code = app_timer_start(m_avg_timer_id,AVG_TIMER_INTERVAL,NULL);
  APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
  //初始化log程序模块
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
  //设置log输出终端（根据sdk_config.h中的配置设置输出终端为UART或者RTT）
  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//空闲状态处理函数。如果没有挂起的日志操作,则睡眠直到下一个事件发生后唤醒系统
static void idle_state_handle(void)
{
  //处理挂起的log
  if(NRF_LOG_PROCESS() == false)
  {
    //运行电源管理,该函数需要放到主循环里面执行
    nrf_pwr_mgmt_run();
  }
}
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)
void button_event_handler(uint8_t pin_no, uint8_t button_action)//APP按键事件回调函数
{
  if(pin_no == POW_BTN)//判断键值
  {
    if(button_action == APP_BUTTON_PUSH)   //按键按下事件
    {
      if(Sys_State == 0 && Charge_State == 4)
      {
        Sys_State = 1;
        Set_Led(Sys_State);
        ble_adv_start();
        Sleep_Count = 0;
      }
      else if(Sys_State > 0 && Charge_State == 4)
      {
        Sys_State = 0;
        Set_Led(Sys_State);
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        ble_adv_stop();
      }
    }
    if(button_action == APP_BUTTON_RELEASE)
    {
      if(Sys_State == 0 && Charge_State == 4)
      {
        nrf_gpio_pin_clear(POW_OUT1);
        nrf_gpio_pin_clear(POW_OUT2);
      }
    }
  }
}

static void buttons_init(void)
{
  ret_code_t err_code;
  //app按键数组必须定义为static类型,因为按键处理模块需要保存指向该数组的指针
  static app_button_cfg_t buttons[] =
  {
    {POW_BTN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
    {CHG_ING, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
    {CHG_IDLE,APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
  };
  //初始化APP按键
  err_code = app_button_init(buttons, ARRAY_SIZE(buttons),BUTTON_DETECTION_DELAY);
  APP_ERROR_CHECK(err_code);
  //使能APP按键
  err_code = app_button_enable();
  APP_ERROR_CHECK(err_code);
}

void IO_Init(void)
{
  //nrf_delay_ms(100);
  nrf_gpio_cfg(
  POW_OUT1,
  NRF_GPIO_PIN_DIR_OUTPUT,
  NRF_GPIO_PIN_INPUT_DISCONNECT,
  NRF_GPIO_PIN_PULLUP,
  NRF_GPIO_PIN_H0H1,
  NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(POW_OUT1);
  nrf_gpio_pin_set(POW_OUT2);

  nrf_gpio_cfg_input(AD_DOUT1,NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(AD_DOUT2,NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(AD_SCK);

  nrf_gpio_cfg_output(LED_RD);
  nrf_gpio_cfg_output(LED_GN);
  Sys_State = 1;
  Set_Led(Sys_State);
}

void wdt_event_handler(void)
{
  NRF_LOG_INFO("Watch Dog!");
}

//看门狗初始化,初始化完成后会启动看门狗,看门狗一旦启动后就无法停止
void wdt_init(void)
{
  uint32_t err_code = NRF_SUCCESS;
  //定义WDT配置结构体并使用
  nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
  //初始化WDT
  err_code = nrfx_wdt_init(&config, wdt_event_handler);
  APP_ERROR_CHECK(err_code);
  //申请喂狗通道,也就是使用哪个
  err_code = nrfx_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);
  //启动WDT
  nrfx_wdt_enable();       
}

//SAADC事件回调函数,因为是堵塞模式,所以不需要事件,这里定义了一个空的事件回调函数
void saadc_callback(nrf_drv_saadc_evt_t const * p_event){}

//初始化SAADC,配置使用的SAADC通道的参数
void saadc_init(void)
{
  ret_code_t err_code;
  //定义ADC通道配置结构体，并使用单端采样配置宏初始化，
  //NRF_SAADC_INPUT_AIN2是使用的模拟输入通道
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
  //初始化SAADC，注册事件回调函数。
  err_code = nrf_drv_saadc_init(NULL, saadc_callback);
  APP_ERROR_CHECK(err_code);
  //初始化SAADC通道0
  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);
}

//主函数
int main(void)
{
  //初始化IO
  IO_Init();
  //初始化log程序模块
  log_init();
  //初始化APP定时器
  timers_init();
  //初始化电源管理
  power_management_init();
  //初始化协议栈
  ble_stack_init();
  //初始化FDS,用于将设备名称保存到片内Flash
  my_fds_init();
  //初始化GAP参数
  gap_params_init();
  //初始化GATT
  gatt_init();
  //初始化服务
  services_init();
  //初始化广播
  advertising_init();
  //初始化连接参数协商
  conn_params_init();
  //初始化按键
  buttons_init();
  //初始化看门狗
  wdt_init();
  //启动广播
  advertising_start();
  //初始化SAADC
  saadc_init();   
  //主循环
  NRF_LOG_INFO("Initial Finish!");
  //初始化串口
  //uart_config();
  //锁定SWD口
  //Lock_Data();
  while(true)
  {
    //重量参数保存
    weight_data_handle();
    //设备名称保存
    device_name_handle();
    //处理挂起的LOG和运行电源管理
    idle_state_handle();
  }
}
