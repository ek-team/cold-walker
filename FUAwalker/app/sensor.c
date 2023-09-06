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

#define AD_SCK1          NRF_GPIO_PIN_MAP(0,26)
#define AD_DOUT1         NRF_GPIO_PIN_MAP(0,25)
#define AD_SCK2          NRF_GPIO_PIN_MAP(0,28)
#define AD_DOUT2         NRF_GPIO_PIN_MAP(0,27)

typedef unsigned char BYTE;
typedef unsigned int  WORD;
typedef unsigned long DWORD;

#define Z_MIN     10000//最小标定差
#define LEN       5

DWORD Data[5] = {0};
DWORD Clear[5] = {0};
DWORD Peel[5] = {0};
DWORD Para[5][5] = {0};   //第一个数代表传感器,第二个数代表标定点
DWORD ParaDis[5][5] = {0};//第一个数代表传感器,第二个数代表标定点
DWORD Sum[5] = {0};
BYTE  PSet[5][5] = {0};   //第一个数代表传感器,第二个数代表标定点
BYTE  PSetDis[5][5] = {0};//第一个数代表传感器,第二个数代表标定点
BYTE  PSetNum[5];         //标定点指针
BYTE  DirFlag[5];         //标定方向

DWORD Arr[5][LEN];
DWORD Brr[5][LEN];
BYTE  Buf = 0;
BYTE Channel = 0;
BYTE AvgCount = 0;

void Read_Count()
{
  DWORD Count1; 
  DWORD Count2;
  BYTE i,Wait;
  if(Channel == 0)
  {
    Channel = 1;
    nrf_gpio_pin_clear(AD_SCK1);
    nrf_gpio_pin_clear(AD_SCK2);
    Count1 = 0;
    Count2 = 0;
    Wait = 0;
    while(nrf_gpio_pin_read(AD_DOUT1))
    {
      if(Wait++ > 100)
      {
        Wait = 0;
        NRF_LOG_INFO("Read4 Error!");
        break;
      }
    }
    Wait = 0;
    while(nrf_gpio_pin_read(AD_DOUT2))
    {
      if(Wait++ > 100)
      {
        Wait = 0;
        NRF_LOG_INFO("Read2 Error!");
        break;
      }
    }
    for(i=0; i<26; i++)
    {
      nrf_gpio_pin_set(AD_SCK1);
      nrf_gpio_pin_set(AD_SCK2);
      Count1 = Count1<<1;
      Count2 = Count2<<1;
      nrf_gpio_pin_clear(AD_SCK1);
      nrf_gpio_pin_clear(AD_SCK2);
      if(nrf_gpio_pin_read(AD_DOUT1)) Count1++;
      if(nrf_gpio_pin_read(AD_DOUT2)) Count2++;
    }
    nrf_gpio_pin_set(AD_SCK1);
    nrf_gpio_pin_set(AD_SCK2);
    Count1 &= 0x00FFFFFF;
    Count2 &= 0x00FFFFFF;
    Count1 = Count1^0x800000;
    Count2 = Count2^0x800000;
    nrf_gpio_pin_clear(AD_SCK1);
    nrf_gpio_pin_clear(AD_SCK2);
    Sum[4] = Count1;
    Sum[2] = Count2;
  }
  else
  {
    Channel = 0;
    nrf_gpio_pin_clear(AD_SCK1);
    nrf_gpio_pin_clear(AD_SCK2);
    Count1 = 0;
    Count2 = 0;
    Wait = 0;
    while(nrf_gpio_pin_read(AD_DOUT1))
    {
      if(Wait++ > 100)
      {
        Wait = 0;
        NRF_LOG_INFO("Read3 Error!");
        break;
      }
    }
    Wait = 0;
    while(nrf_gpio_pin_read(AD_DOUT2))
    {
      if(Wait++ > 100)
      {
        Wait = 0;
        NRF_LOG_INFO("Read1 Error!");
        break;
      }
    }
    for(i=0; i<25; i++)//通道B,增益32
    {
      nrf_gpio_pin_set(AD_SCK1);
      nrf_gpio_pin_set(AD_SCK2);
      Count1 = Count1<<1;
      Count2 = Count2<<1;
      nrf_gpio_pin_clear(AD_SCK1);
      nrf_gpio_pin_clear(AD_SCK2);
      if(nrf_gpio_pin_read(AD_DOUT1)) Count1++;
      if(nrf_gpio_pin_read(AD_DOUT2)) Count2++;
    }
    nrf_gpio_pin_set(AD_SCK1);
    nrf_gpio_pin_set(AD_SCK2);
    Count1 &= 0x00FFFFFF;
    Count2 &= 0x00FFFFFF;
    Count1 = Count1^0x800000;
    Count2 = Count2^0x800000;
    nrf_gpio_pin_clear(AD_SCK1);
    nrf_gpio_pin_clear(AD_SCK2);
    Sum[3] = Count1;
    Sum[1] = Count2;
  }
}

void Avg_Filter()//滑动中值滤波
{
  BYTE i,j,k;
  DWORD Tmp;
	
  Read_Count();
  Brr[1][Buf] = Sum[1];
  Brr[2][Buf] = Sum[2];
  Brr[3][Buf] = Sum[3];
  Brr[4][Buf] = Sum[4];
	
  for(k=1;k<=4;k++)
  {
    for(i=0;i<LEN;i++)
    {
      Arr[k][i] = Brr[k][i];
    }
  }
  Buf++;
  if(Buf==LEN)
  {
    Buf = 0;
  }
  for(k=1;k<=4;k++)
  {
    for(i=0;i<LEN-1;i++)
    {
      for(j=0;j<LEN-1-i;j++)
      {
        if(Arr[k][j]>Arr[k][j+1]) 
        {
          Tmp = Arr[k][j];
          Arr[k][j] = Arr[k][j+1];
          Arr[k][j+1] = Tmp;
        }
      }
    }
  }
  Data[1] = Arr[1][(LEN-1)/2];
  Data[2] = Arr[2][(LEN-1)/2];
  Data[3] = Arr[3][(LEN-1)/2];
  Data[4] = Arr[4][(LEN-1)/2];
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

DWORD Uart_Send(BYTE i,BYTE j)
{
  signed long long Val;
  Val = Data[i];
//  if(i == 3)
//  {
//    NRF_LOG_INFO(" %d %d %d %d %d %d \n",Peel[i],ParaDis[i][0],ParaDis[i][1],ParaDis[i][2],ParaDis[i][3],ParaDis[i][4]);
//    NRF_LOG_INFO(" %d %d %d %d %d %d \n",Val,PSetDis[i][0],PSetDis[i][1],PSetDis[i][2],PSetDis[i][3],PSetDis[i][4]);
//  }
  if(DirFlag[i] != 1)
  {
    if(Val >= ParaDis[i][3]) Val = PSetDis[i][3]*100 + 100*(Val - ParaDis[i][3])*(PSetDis[i][4] - PSetDis[i][3])/(ParaDis[i][4] - ParaDis[i][3]);
    else if(Val >= ParaDis[i][2]) Val = PSetDis[i][2]*100 + 100*(Val - ParaDis[i][2])*(PSetDis[i][3] - PSetDis[i][2])/(ParaDis[i][3] - ParaDis[i][2]);
    else if(Val >= ParaDis[i][1]) Val = PSetDis[i][1]*100 + 100*(Val - ParaDis[i][1])*(PSetDis[i][2] - PSetDis[i][1])/(ParaDis[i][2] - ParaDis[i][1]);
    else if(Val >= ParaDis[i][0]) Val = PSetDis[i][0]*100 + 100*(Val - ParaDis[i][0])*(PSetDis[i][1] - PSetDis[i][0])/(ParaDis[i][1] - ParaDis[i][0]);
    else Val = 100*(Val - Peel[i])*(+PSetDis[i][0])/(ParaDis[i][0] - Peel[i]);
  }
  if(DirFlag[i] == 1)
  {
    if(Val <= ParaDis[i][3]) Val = PSetDis[i][3]*100 + 100*(Val - ParaDis[i][3])*(PSetDis[i][3] - PSetDis[i][4])/(ParaDis[i][3] - ParaDis[i][4]);
    else if(Val <= ParaDis[i][2]) Val = PSetDis[i][2]*100 + 100*(Val - ParaDis[i][2])*(PSetDis[i][2] - PSetDis[i][3])/(ParaDis[i][2] - ParaDis[i][3]);
    else if(Val <= ParaDis[i][1]) Val = PSetDis[i][1]*100 + 100*(Val - ParaDis[i][1])*(PSetDis[i][1] - PSetDis[i][2])/(ParaDis[i][1] - ParaDis[i][2]);
    else if(Val <= ParaDis[i][0]) Val = PSetDis[i][0]*100 + 100*(Val - ParaDis[i][0])*(PSetDis[i][0] - PSetDis[i][1])/(ParaDis[i][0] - ParaDis[i][1]);
    else Val = 100*(Val - Peel[i])*(-PSetDis[i][0])/(Peel[i] - ParaDis[i][0]);
  }
  if(j == 0)
  {
    if(Val >= Clear[i]) Val = Val - Clear[i];
    else Val = Clear[i] - Val;
  }
  return Val;
}

void Init_Data()//初始化参数
{
  Save(2,  130);Save(1,  255);Save(0, 255); //Peel[1]传感器1的零位标定点AD值
  Save(5,  130);Save(4,  255);Save(3, 255); //Peel[2]传感器2的零位标定点AD值
  Save(8,  130);Save(7,  255);Save(6, 255); //Peel[3]传感器3的零位标定点AD值
  Save(11, 130);Save(10, 255);Save(9, 255); //Peel[4]传感器4的零位标定点AD值

  Save(18, 120);Save(17, 255);Save(16, 255);//Para[1][0]传感器1的五个标定点AD值
  Save(53, 110);Save(52, 255);Save(51, 255);//Para[1][1]
  Save(57, 100);Save(56, 255);Save(55, 255);//Para[1][2]
  Save(61,  90);Save(60, 255);Save(59, 255);//Para[1][3]
  Save(65,  80);Save(64, 255);Save(63, 255);//Para[1][4]
	
  Save(21, 120);Save(20, 255);Save(19, 255);//Para[2][0]传感器2的五个标定点AD值
  Save(70, 110);Save(69, 255);Save(68, 255);//Para[2][1]
  Save(74, 100);Save(73, 255);Save(72, 255);//Para[2][2]
  Save(78,  90);Save(77, 255);Save(76, 255);//Para[2][3]
  Save(82,  80);Save(81, 255);Save(80, 255);//Para[2][4]
	
  Save(24, 120);Save(23, 255);Save(22, 255);//Para[3][0]传感器3的五个标定点AD值
  Save(87, 110);Save(86, 255);Save(85, 255);//Para[3][1]
  Save(91, 100);Save(90, 255);Save(89, 255);//Para[3][2]
  Save(95,  90);Save(94, 255);Save(93, 255);//Para[3][3]
  Save(99,  80);Save(98, 255);Save(97, 255);//Para[3][4]
	
  Save(27, 120);Save(26, 255);Save(25, 255);//Para[4][0]传感器4的五个标定点AD值
  Save(104,110);Save(103,255);Save(102,255);//Para[4][1]
  Save(108,100);Save(107,255);Save(106,255);//Para[4][2]
  Save(112, 90);Save(111,255);Save(110,255);//Para[4][3]
  Save(116, 80);Save(115,255);Save(114,255);//Para[4][4]
  
  Save(32, 10);Save(50, 20);Save(54, 30);Save(58, 40);Save(62, 50);  //PSet[1][i]传感器1的五个标点显示值
  Save(33, 10);Save(67, 20);Save(71, 30);Save(75, 40);Save(79, 50);  //PSet[2][i]传感器2的五个标点显示值
  Save(34, 10);Save(84, 20);Save(88, 30);Save(92, 40);Save(96, 50);  //PSet[3][i]传感器3的五个标点显示值
  Save(35, 10);Save(101,20);Save(105,30);Save(109,40);Save(113,50);  //PSet[4][i]传感器4的五个标点显示值
	
  Save(38, 0);Save(37, 0);Save(36, 0);      //Clear[1]传感器1的去皮值
  Save(41, 0);Save(40, 0);Save(39, 0);      //Clear[2]传感器2的去皮值
  Save(44, 0);Save(43, 0);Save(42, 0);      //Clear[3]传感器3的去皮值
  Save(47, 0);Save(46, 0);Save(45, 0);      //Clear[4]传感器4的去皮值

  Save(49,0);                               //PSetNum[1]传感器1的标定序号值
  Save(66,0);                               //PSetNum[2]传感器2的标定序号值
  Save(83,0);                               //PSetNum[3]传感器3的标定序号值
  Save(100,0);                              //PSetNum[4]传感器4的标定序号值
	
  Save(28, 1);                              //传感器1的标定方向
  Save(29, 1);                              //传感器2的标定方向
  Save(30, 1);                              //传感器3的标定方向
  Save(31, 1);                              //传感器4的标定方向
}

void Read_Data()
{
  BYTE HPeel[5],MPeel[5],LPeel[5];
  BYTE HPara[5][5],MPara[5][5],LPara[5][5];
  BYTE HClear[5],MClear[5],LClear[5];
  BYTE i;
	
  weight_data_set();  //读取传感器数据
	
  HPeel[1] = Load(2);MPeel[1] = Load(1);LPeel[1] = Load(0);               //Peel[1]传感器1的零位标定点AD值
  HPeel[2] = Load(5);MPeel[2] = Load(4);LPeel[2] = Load(3);               //Peel[2]传感器2的零位标定点AD值
  HPeel[3] = Load(8);MPeel[3] = Load(7);LPeel[3] = Load(6);               //Peel[3]传感器3的零位标定点AD值
  HPeel[4] = Load(11);MPeel[4] = Load(10);LPeel[4] = Load(9);             //Peel[4]传感器4的零位标定点AD值
	
  HPara[1][0] = Load(18);MPara[1][0] = Load(17);LPara[1][0] = Load(16);   //Para[1][0]传感器1的五个标定点AD值
  HPara[1][1] = Load(53);MPara[1][1] = Load(52);LPara[1][1] = Load(51);   //Para[1][1]
  HPara[1][2] = Load(57);MPara[1][2] = Load(56);LPara[1][2] = Load(55);   //Para[1][2]
  HPara[1][3] = Load(61);MPara[1][3] = Load(60);LPara[1][3] = Load(59);   //Para[1][3]
  HPara[1][4] = Load(65);MPara[1][4] = Load(64);LPara[1][4] = Load(63);   //Para[1][4]

  HPara[2][0] = Load(21);MPara[2][0] = Load(20);LPara[2][0] = Load(19);   //Para[2][0]传感器2的五个标定点AD值
  HPara[2][1] = Load(70);MPara[2][1] = Load(69);LPara[2][1] = Load(68);   //Para[2][1]
  HPara[2][2] = Load(74);MPara[2][2] = Load(73);LPara[2][2] = Load(72);   //Para[2][2]
  HPara[2][3] = Load(78);MPara[2][3] = Load(77);LPara[2][3] = Load(76);   //Para[2][3]
  HPara[2][4] = Load(82);MPara[2][4] = Load(81);LPara[2][4] = Load(80);   //Para[2][4]
	
  HPara[3][0] = Load(24);MPara[3][0] = Load(23);LPara[3][0] = Load(22);   //Para[3][0]传感器3的五个标定点AD值
  HPara[3][1] = Load(87);MPara[3][1] = Load(86);LPara[3][1] = Load(85);   //Para[3][1]
  HPara[3][2] = Load(91);MPara[3][2] = Load(90);LPara[3][2] = Load(89);   //Para[3][2]
  HPara[3][3] = Load(95);MPara[3][3] = Load(94);LPara[3][3] = Load(93);   //Para[3][3]
  HPara[3][4] = Load(99);MPara[3][4] = Load(98);LPara[3][4] = Load(97);   //Para[3][4]
	
  HPara[4][0] = Load(27); MPara[4][0] = Load(26); LPara[4][0] = Load(25); //Para[4][0]传感器4的五个标定点AD值
  HPara[4][1] = Load(104);MPara[4][1] = Load(103);LPara[4][1] = Load(102);//Para[4][1]
  HPara[4][2] = Load(108);MPara[4][2] = Load(107);LPara[4][2] = Load(106);//Para[4][2]
  HPara[4][3] = Load(112);MPara[4][3] = Load(111);LPara[4][3] = Load(110);//Para[4][3]
  HPara[4][4] = Load(116);MPara[4][4] = Load(115);LPara[4][4] = Load(114);//Para[4][4]
	
  PSet[1][0] = Load(32);PSet[1][1] = Load(50);PSet[1][2] = Load(54);PSet[1][3] = Load(58);PSet[1][4] = Load(62);    //PSet[1][i]传感器1的五个标点显示值
  PSet[2][0] = Load(33);PSet[2][1] = Load(67);PSet[2][2] = Load(71);PSet[2][3] = Load(75);PSet[2][4] = Load(79);    //PSet[2][i]传感器2的五个标点显示值
  PSet[3][0] = Load(34);PSet[3][1] = Load(84);PSet[3][2] = Load(88);PSet[3][3] = Load(92);PSet[3][4] = Load(96);    //PSet[3][i]传感器3的五个标点显示值
  PSet[4][0] = Load(35);PSet[4][1] = Load(101);PSet[4][2] = Load(105);PSet[4][3] = Load(109);PSet[4][4] = Load(113);//PSet[4][i]传感器4的五个标点显示值
	
  HClear[1] = Load(38);MClear[1] = Load(37);LClear[1] = Load(36); //Clear[1]传感器1的去皮值
  HClear[2] = Load(41);MClear[2] = Load(40);LClear[2] = Load(39); //Clear[2]传感器2的去皮值
  HClear[3] = Load(44);MClear[3] = Load(43);LClear[3] = Load(42); //Clear[3]传感器3的去皮值
  HClear[4] = Load(47);MClear[4] = Load(46);LClear[4] = Load(45); //Clear[4]传感器4的去皮值
	
  PSetNum[1] = Load(49);  //PSetNum[1]传感器1的标定序号值
  PSetNum[2] = Load(66);  //PSetNum[2]传感器2的标定序号值
  PSetNum[3] = Load(83);  //PSetNum[3]传感器3的标定序号值
  PSetNum[4] = Load(100); //PSetNum[4]传感器4的标定序号值

  DirFlag[1] = Load(28);  //传感器1的标定方向
  DirFlag[2] = Load(29);  //传感器2的标定方向
  DirFlag[3] = Load(30);  //传感器3的标定方向
  DirFlag[4] = Load(31);  //传感器4的标定方向
	
  for(i=1;i<=4;i++)
  {
    Peel[i] = HPeel[i]*65536 + MPeel[i]*256 + LPeel[i];
    Clear[i] = HClear[i]*65536 + MClear[i]*256 + LClear[i];
		
    Para[i][0] = HPara[i][0]*65536 + MPara[i][0]*256 + LPara[i][0];
    Para[i][1] = HPara[i][1]*65536 + MPara[i][1]*256 + LPara[i][1];
    Para[i][2] = HPara[i][2]*65536 + MPara[i][2]*256 + LPara[i][2];
    Para[i][3] = HPara[i][3]*65536 + MPara[i][3]*256 + LPara[i][3];
    Para[i][4] = HPara[i][4]*65536 + MPara[i][4]*256 + LPara[i][4];
		
    ParaDis[i][0] = Para[i][0];PSetDis[i][0] = PSet[i][0];
    ParaDis[i][1] = Para[i][1];PSetDis[i][1] = PSet[i][1];
    ParaDis[i][2] = Para[i][2];PSetDis[i][2] = PSet[i][2];
    ParaDis[i][3] = Para[i][3];PSetDis[i][3] = PSet[i][3];
    ParaDis[i][4] = Para[i][4];PSetDis[i][4] = PSet[i][4];
    DWORD temp1;
    BYTE temp2,m,n;
    for(m=0; m<5-1; m++)
    {
      for(n=0; n<5-1-m; n++)
      {
        if(PSetDis[i][n] > PSetDis[i][n+1])
        {
          temp1 = ParaDis[i][n];
          temp2 = PSetDis[i][n];
          ParaDis[i][n] = ParaDis[i][n+1];
          PSetDis[i][n] = PSetDis[i][n+1];
          ParaDis[i][n+1] = temp1;
          PSetDis[i][n+1] = temp2;
        }
      }
    }
  }
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
  if(Load(48) != 1)
  {
    Save(48, 1);
    SWD_protect();
  }
}


void Clear_Data()
{
  BYTE HClear,MClear,LClear;
  DWORD tmp;
  tmp = Uart_Send(1,1);
  HClear = tmp>>16;
  MClear = tmp>>8;
  LClear = tmp;
  Save(38, HClear);
  Save(37, MClear);
  Save(36, LClear);

  tmp = Uart_Send(2,1);
  HClear = tmp>>16;
  MClear = tmp>>8;
  LClear = tmp;
  Save(41, HClear);
  Save(40, MClear);
  Save(39, LClear);
  
  tmp = Uart_Send(3,1);
  HClear = tmp>>16;
  MClear = tmp>>8;
  LClear = tmp;
  Save(44, HClear);
  Save(43, MClear);
  Save(42, LClear);
  
  tmp = Uart_Send(4,1);
  HClear = tmp>>16;
  MClear = tmp>>8;
  LClear = tmp;
  Save(47, HClear);
  Save(46, MClear);
  Save(45, LClear);
}

void Save_Data(BYTE i,BYTE RData)
{
  BYTE HPeel[5],MPeel[5],LPeel[5];
  BYTE HPara[5],MPara[5],LPara[5];
  BYTE RDSet[5];
  DWORD tmp;

  Save(38, 0);Save(37, 0);Save(36, 0);//标定时清除去皮1的数据
  Save(41, 0);Save(40, 0);Save(39, 0);//标定时清除去皮2的数据
  Save(44, 0);Save(43, 0);Save(42, 0);//标定时清除去皮3的数据
  Save(47, 0);Save(46, 0);Save(45, 0);//标定时清除去皮4的数据
  if(RData == 0)  //置零指令
  {
    tmp = Data[i];
    HPeel[i] = tmp>>16;
    MPeel[i] = tmp>>8;
    LPeel[i] = tmp;
    if(i == 1)
    {
      Save(2, HPeel[i]);
      Save(1, MPeel[i]);
      Save(0, LPeel[i]);
    }
    if(i == 2)
    {
      Save(5, HPeel[i]);
      Save(4, MPeel[i]);
      Save(3, LPeel[i]);
    }
    if(i == 3)
    {
      Save(8, HPeel[i]);
      Save(7, MPeel[i]);
      Save(6, LPeel[i]);
    }
    if(i == 4)
    {
      Save(11,HPeel[i]);
      Save(10,MPeel[i]);
      Save(9, LPeel[i]);
    }
  }
  if(RData != 0) //重量标定指令
  {
    tmp = Data[i];
    HPara[i] = tmp>>16;
    MPara[i] = tmp>>8;
    LPara[i] = tmp;
    RDSet[i] = RData;
    if(PSet[i][0] == RData) PSetNum[i] = 0;
    if(PSet[i][1] == RData) PSetNum[i] = 1;
    if(PSet[i][2] == RData) PSetNum[i] = 2;
    if(PSet[i][3] == RData) PSetNum[i] = 3;
    if(PSet[i][4] == RData) PSetNum[i] = 4;
    if(i == 1)
    {
      if(PSetNum[i] == 0)
      {
        Save(49,1);
        if(tmp - Z_MIN > Peel[1])
        {
          Save(18, HPara[i]);
          Save(17, MPara[i]);
          Save(16, LPara[i]);
          Save(32, RDSet[i]);
          Save(28, 0);
        }
        if(tmp + Z_MIN < Peel[1])
        {
          Save(18, HPara[i]);
          Save(17, MPara[i]);
          Save(16, LPara[i]);
          Save(32, RDSet[i]);
          Save(28, 1);
        }
      }
      else if(PSetNum[i] == 1)
      {
        Save(49,2);
        Save(53, HPara[i]);
        Save(52, MPara[i]);
        Save(51, LPara[i]);
        Save(50, RDSet[i]);
      }
      else if(PSetNum[i] == 2)
      {
        Save(49,3);
        Save(57, HPara[i]);
        Save(56, MPara[i]);
        Save(55, LPara[i]);
        Save(54, RDSet[i]);
      }
      else if(PSetNum[i] == 3)
      {
        Save(49,4);
        Save(61, HPara[i]);
        Save(60, MPara[i]);
        Save(59, LPara[i]);
        Save(58, RDSet[i]);
      }
      else if(PSetNum[i] == 4)
      {
        Save(49,0);
        Save(65, HPara[i]);
        Save(64, MPara[i]);
        Save(63, LPara[i]);
        Save(62, RDSet[i]);
      }
    }
    if(i == 2)
    {
      if(PSetNum[i] == 0)
      {
        Save(66,1);
        if(tmp - Z_MIN > Peel[2])
        {
          Save(21, HPara[i]);
          Save(20, MPara[i]);
          Save(19, LPara[i]);
          Save(33, RDSet[i]);
          Save(29, 0);
        }
        if(tmp + Z_MIN < Peel[2])
        {
          Save(21, HPara[i]);
          Save(20, MPara[i]);
          Save(19, LPara[i]);
          Save(33, RDSet[i]);
          Save(29, 1);
        }
      }
      else if(PSetNum[i] == 1)
      {
        Save(66,2);
        Save(70, HPara[i]);
        Save(69, MPara[i]);
        Save(68, LPara[i]);
        Save(67, RDSet[i]);
      }
      else if(PSetNum[i] == 2)
      {
        Save(66,3);
        Save(74, HPara[i]);
        Save(73, MPara[i]);
        Save(72, LPara[i]);
        Save(71, RDSet[i]);
      }
      else if(PSetNum[i] == 3)
      {
        Save(66,4);
        Save(78, HPara[i]);
        Save(77, MPara[i]);
        Save(76, LPara[i]);
        Save(75, RDSet[i]);
      }
      else if(PSetNum[i] == 4)
      {
        Save(66,0);
        Save(82, HPara[i]);
        Save(81, MPara[i]);
        Save(80, LPara[i]);
        Save(79, RDSet[i]);
      }
    }
    if(i == 3)
    {
      if(PSetNum[i] == 0)
      {
        Save(83,1);
        if(tmp - Z_MIN > Peel[3])
        {
          Save(24, HPara[i]);
          Save(23, MPara[i]);
          Save(22, LPara[i]);
          Save(34, RDSet[i]);
          Save(30, 0);
        }
        if(tmp + Z_MIN < Peel[3])
        {
          Save(24, HPara[i]);
          Save(23, MPara[i]);
          Save(22, LPara[i]);
          Save(34, RDSet[i]);
          Save(30, 1);
        }
      }
      else if(PSetNum[i] == 1)
      {
        Save(83,2);
        Save(87, HPara[i]);
        Save(86, MPara[i]);
        Save(85, LPara[i]);
        Save(84, RDSet[i]);
      }
      else if(PSetNum[i] == 2)
      {
        Save(83,3);
        Save(91, HPara[i]);
        Save(90, MPara[i]);
        Save(89, LPara[i]);
        Save(88, RDSet[i]);
      }
      else if(PSetNum[i] == 3)
      {
        Save(83,4);
        Save(95, HPara[i]);
        Save(94, MPara[i]);
        Save(93, LPara[i]);
        Save(92, RDSet[i]);
      }
      else if(PSetNum[i] == 4)
      {
        Save(83,0);
        Save(99, HPara[i]);
        Save(98, MPara[i]);
        Save(97, LPara[i]);
        Save(96, RDSet[i]);
      }
    }
    if(i == 4)
    {
      if(PSetNum[i] == 0)
      {
        Save(100,1);
        if(tmp - Z_MIN > Peel[4])
        {
          Save(27, HPara[i]);
          Save(26, MPara[i]);
          Save(25, LPara[i]);
          Save(35, RDSet[i]);
          Save(31, 0);
        }
        if(tmp + Z_MIN < Peel[4])
        {
          Save(27, HPara[i]);
          Save(26, MPara[i]);
          Save(25, LPara[i]);
          Save(35, RDSet[i]);
          Save(31, 1);
        }
      }
      else if(PSetNum[i] == 1)
      {
        Save(100,2);
        Save(104, HPara[i]);
        Save(103, MPara[i]);
        Save(102, LPara[i]);
        Save(101, RDSet[i]);
      }
      else if(PSetNum[i] == 2)
      {
        Save(100,3);
        Save(108, HPara[i]);
        Save(107, MPara[i]);
        Save(106, LPara[i]);
        Save(105, RDSet[i]);
      }
      else if(PSetNum[i] == 3)
      {
        Save(100,4);
        Save(112, HPara[i]);
        Save(111, MPara[i]);
        Save(110, LPara[i]);
        Save(109, RDSet[i]);
      }
      else if(PSetNum[i] == 4)
      {
        Save(100,0);
        Save(116, HPara[i]);
        Save(115, MPara[i]);
        Save(114, LPara[i]);
        Save(113, RDSet[i]);
      }
    }
  }
}
