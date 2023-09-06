#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <nrf_gpio.h>
#include "nrf_delay.h"
#include "device_name_op.h"
#include "math.h"
#include "nrf_nvmc.h"

//Log需要引用的头文件
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define AD_SCK          NRF_GPIO_PIN_MAP(0,26)
#define AD_DOUT         NRF_GPIO_PIN_MAP(0,25)

typedef unsigned char BYTE;
typedef unsigned int WORD;
typedef unsigned long DWORD;

DWORD Data = 0;
DWORD Clear = 0;
DWORD Peel = 0;
DWORD Para = 0;
DWORD Padj[10] = {0};

BYTE Flag = 0;
BYTE PSet = 0;
BYTE Pt = 0;
BYTE HPadj[10],MPadj[10],LPadj[10];

#define Z_MIN       100000      //最小标定差

DWORD Read_Count()
{
  DWORD Count; 
  BYTE i,Wait;
  nrf_gpio_pin_clear(AD_SCK);
  Count = 0;
  while(nrf_gpio_pin_read(AD_DOUT))
  {
    if(Wait++ > 5)
    {
      Wait = 0;
      NRF_LOG_INFO("Read Error!");
      return Data;
    }
  }
  for(i=0; i<26; i++)
  {
    nrf_gpio_pin_set(AD_SCK);
    Count = Count<<1;
    nrf_gpio_pin_clear(AD_SCK);
    if(nrf_gpio_pin_read(AD_DOUT)) Count++;
  }
  Count >>= 1;
  nrf_gpio_pin_set(AD_SCK);
  Count &= 0x00FFFFFF;
  Count = Count^0x800000;
  nrf_gpio_pin_clear(AD_SCK);
  return(Count);
}

#define LEN   5
DWORD Arr[LEN];
DWORD Brr[LEN];
BYTE  Buf = 0;

void Avg_Filter()
{
  BYTE i,j;
  DWORD Tmp;
  Brr[Buf] = Read_Count();
  for(i=0;i<LEN;i++)
  {
    Arr[i] = Brr[i];
  }
  Buf++;
  if(Buf==LEN)
  {
    Buf = 0;
  }
	
  for(i=0;i<LEN-1;i++)
  {
    for(j=0;j<LEN-1-i;j++)
    {
      if(Arr[j]>Arr[j+1]) 
      {
        Tmp = Arr[j];
        Arr[j] = Arr[j+1];
        Arr[j+1] = Tmp;
      }
    }
  }
  Data = Arr[(LEN-1)/2];
}

BYTE Load(WORD addr)
{
  return weight_data_info.data.data8[8 + addr];
}

void Save(WORD addr, BYTE dat)
{
  weight_data_info.data.data32[0] = DATA_FLASH_TYPE_NAME;
  weight_data_info.data.data32[1] = DATA_FLASH_LEN_WORDS;
  weight_data_info.data.data8[8 + addr] = dat;
  weight_data_info.save_data_flag = true;
}

DWORD Uart_Send(BYTE j)
{
  DWORD Val;
  DWORD Adj;
  float Ratio;
	
  Val = Data;
  Adj = Data;
	
  if(Peel > Val) Val = Peel - Val;
  else           Val = Val - Peel;

  if(Peel > Adj) Adj = Peel - Adj;
  else           Adj = Adj - Peel;
	
  if(Flag != 1) Ratio = (float)(Para - Peel)/PSet/1000;
  if(Flag == 1) Ratio = (float)(Peel - Para)/PSet/1000;
	
  Val = Val/Ratio;
	
  if(Flag != 1)
  {
    if(Val >= 1000) Ratio = (float)(Padj[1] - Peel)/10/1000;
    if(Val >= 15000) Ratio = (float)(Padj[2] - Peel)/20/1000;
    if(Val >= 25000) Ratio = (float)(Padj[3] - Peel)/30/1000;
    if(Val >= 35000) Ratio = (float)(Padj[4] - Peel)/40/1000;
    if(Val >= 60000) Ratio = (float)(Padj[9] - Peel)/90/1000;
  }
  if(Flag == 1)
  {
    if(Val >= 1000) Ratio = (float)(Peel - Padj[1])/10/1000;
    if(Val >= 15000) Ratio = (float)(Peel - Padj[2])/20/1000;
    if(Val >= 25000) Ratio = (float)(Peel - Padj[3])/30/1000;
    if(Val >= 35000) Ratio = (float)(Peel - Padj[4])/40/1000;
    if(Val >= 60000) Ratio = (float)(Peel - Padj[9])/90/1000;
  }
  Adj = Adj/Ratio;

  if(((Val - Adj) < 5000 || (Adj - Val) < 5000) && LPadj[1] != 0) //10kg非线性修正
  {
    if(Val >= 1000 && Val < 15000)
    {
      Val = Adj;
    }
  }
  if(((Val - Adj) < 5000 || (Adj - Val) < 5000) && LPadj[2] != 0) //20kg非线性修正
  {
    if(Val >= 15000 && Val < 25000)
    {
      Val = Adj;
    }
  }
  if(((Val - Adj) < 5000 || (Adj - Val) < 5000) && LPadj[3] != 0) //30kg非线性修正
  {
    if(Val >= 25000 && Val < 35000)
    {
      Val = Adj;
    }
  }
  if(((Val - Adj) < 5000 || (Adj - Val) < 5000) && LPadj[4] != 0) //40kg非线性修正
  {
    if(Val >= 35000 && Val < 45000)
    {
      Val = Adj;
    }
  }
  if(((Val - Adj) < 7000 || (Adj - Val) < 7000) && LPadj[9] != 0) //90kg非线性修正
  {
    if(Val >= 60000 && Val < 70000)
    {
      Val = Val*1/2 + Adj*1/2;
    }
    if(Val >= 70000 && Val < 110000)
    {
      Val = Adj;
    }
  }
  Val = Val/10;
  if(j == 0)
  {
    if(Val >= Clear) Val = Val - Clear;
    else Val = Clear - Val;
  }
  return Val;
}

void Init_Data() //初始化参数
{
  BYTE HPeel,MPeel,LPeel;
  DWORD tmp;
  Avg_Filter();	
  tmp = Data;
  HPeel = tmp>>16;
  MPeel = tmp>>8;
  LPeel = tmp;
  Save(2, HPeel);
  Save(1, MPeel);
  Save(0, LPeel);
	
  Save(5, 255);
  Save(4, 255);
  Save(3, 255);
	
  Save(6, 255);

  Save(27, 0);
  Save(26, 0);
  Save(25, 0);
}

void Read_Data()
{
  BYTE HPeel,MPeel,LPeel;
  BYTE HPara,MPara,LPara;
  BYTE HClear,MClear,LClear;
	
  weight_data_set();//读取传感器数据
  HPeel = Load(2);
  MPeel = Load(1);
  LPeel = Load(0);

  HPara = Load(5);
  MPara = Load(4);
  LPara = Load(3);
	
  PSet = Load(6);
  Flag = Load(7);
	
  HPadj[1] = Load(12);
  MPadj[1] = Load(11);
  LPadj[1] = Load(10);
	
  HPadj[2] = Load(15);
  MPadj[2] = Load(14);
  LPadj[2] = Load(13);
	
  HPadj[3] = Load(18);
  MPadj[3] = Load(17);
  LPadj[3] = Load(16);
	
  HPadj[4] = Load(21);
  MPadj[4] = Load(20);
  LPadj[4] = Load(19);
	
  HPadj[9] = Load(24);
  MPadj[9] = Load(23);
  LPadj[9] = Load(22);
	
  HClear = Load(27);
  MClear = Load(26);
  LClear = Load(25);
	
  Clear = HClear*65536 + MClear*256 + LClear;
  Peel = HPeel*65536 + MPeel*256 + LPeel;
  Para = HPara*65536 + MPara*256 + LPara;
  Padj[1] = HPadj[1]*65536 + MPadj[1]*256 + LPadj[1];
  Padj[2] = HPadj[2]*65536 + MPadj[2]*256 + LPadj[2];
  Padj[3] = HPadj[3]*65536 + MPadj[3]*256 + LPadj[3];
  Padj[4] = HPadj[4]*65536 + MPadj[4]*256 + LPadj[4];
  Padj[9] = HPadj[9]*65536 + MPadj[9]*256 + LPadj[9];
}

void SWD_protect()
{
  nrf_delay_ms(100);
  if(NRF_UICR->APPROTECT == 0xFFFFFFFF)
  {
    nrf_nvmc_write_word((uint32_t)&(NRF_UICR->APPROTECT),0xFFFFFF00);        
    NVIC_SystemReset();
  }       
}

void Lock_Data()
{
  if(Load(28) != 1)
  {
    Save(28, 1);
    SWD_protect();
  }
}

void Clear_Data()
{
  BYTE HClear,MClear,LClear;
  DWORD tmp;
  tmp = Uart_Send(1);
  HClear = tmp>>16;
  MClear = tmp>>8;
  LClear = tmp;
  Save(27, HClear);
  Save(26, MClear);
  Save(25, LClear);
}

void Save_Data(BYTE RData)
{
  BYTE HPeel,MPeel,LPeel;
  BYTE HPara,MPara,LPara;
  BYTE tmpset;
  DWORD tmp;
	
  Save(27, 0);
  Save(26, 0);
  Save(25, 0);
  if(RData == 0)  //置零指令
  {
    tmp = Data;
    HPeel = tmp>>16;
    MPeel = tmp>>8;
    LPeel = tmp;
    Save(2, HPeel);
    Save(1, MPeel);
    Save(0, LPeel);
  }
  if(RData != 0) //重量标定指令
  {
    tmpset = RData;
    tmp = Data;
    HPara = tmp>>16;
    MPara = tmp>>8;
    LPara = tmp;
			
    if(tmp - Z_MIN > Peel && Flag != 1 && (tmpset == 10 || tmpset == 20 || tmpset == 30 || tmpset == 40 || tmpset == 90))      //修正标定(正值)
    {
      if(tmpset == 10)
      {
        Save(12, HPara);
        Save(11, MPara);
        Save(10, LPara);
      }
      if(tmpset == 20)
      {
        Save(15, HPara);
        Save(14, MPara);
        Save(13, LPara);
      }
      if(tmpset == 30)
      {
        Save(18, HPara);
        Save(17, MPara);
        Save(16, LPara);
      }
      if(tmpset == 40)
      {
        Save(21, HPara);
        Save(20, MPara);
        Save(19, LPara);
      }
      if(tmpset == 90)
      {
        Save(24, HPara);
        Save(23, MPara);
        Save(22, LPara);
	  }
    }
    else if(tmp + Z_MIN < Peel && Flag == 1 && (tmpset == 10 || tmpset == 20 || tmpset == 30 || tmpset == 40 || tmpset == 90)) //修正标定(负值)
    {				
      if(tmpset == 10)
      {
        Save(12, HPara);
        Save(11, MPara);
        Save(10, LPara);
      }
      if(tmpset == 20)
      {
        Save(15, HPara);
        Save(14, MPara);
        Save(13, LPara);
      }
      if(tmpset == 30)
      {
        Save(18, HPara);
        Save(17, MPara);
        Save(16, LPara);
      }
      if(tmpset == 40)
      {
        Save(21, HPara);
        Save(20, MPara);
        Save(19, LPara);
      }
      if(tmpset == 90)
      {
        Save(24, HPara);
        Save(23, MPara);
        Save(22, LPara);
      }
    }
    else if(tmp - Z_MIN > Peel && tmpset >= 5 && tmpset != 10 && tmpset != 20 && tmpset != 30 && tmpset != 40 && tmpset != 90) //重量数据正常时进行标定(正值)
    {
      Save(5, HPara);
      Save(4, MPara);
      Save(3, LPara);
      Save(6, tmpset);
      Save(7, 0);
    }
    else if(tmp + Z_MIN < Peel && tmpset >= 5 && tmpset != 10 && tmpset != 20 && tmpset != 30 && tmpset != 40 && tmpset != 90) //重量数据正常时进行标定(负值)
    {
      Save(5, HPara);
      Save(4, MPara);
      Save(3, LPara);
      Save(6, tmpset);
      Save(7, 1);
    }
  }
}
