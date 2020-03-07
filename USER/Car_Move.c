#include <stdint.h>
#include "stm32f4xx.h"
#include "canP_HostCom.h"
#include "delay.h"
#include "data_channel.h"
#include "roadway_check.h"
#include "bh1750.h"

#define __CAN_USER_C__
#include "can_user.h"
#include "tba.h"
#include "data_base.h"
#include "cba.h"
#include "infrared.h"
#include "syn7318.h"
#include <string.h>
#include "Timer.h"
#include "Ultrasonic.h"
#include "Rc522.h"
#include "Car_Move.h"
#include "can_user.h"
#include "stm32f4xx_gpio.h"



/*****************************************************************************
									标记与参数区
*****************************************************************************/
u8 RFID_Task_Flag=RESET;
u8 RFID_Go_Flag=RESET;
u8 UnTrack_Task_Flag1= RESET;
u8 UnTrack_Task_Flag2= RESET;
u8 UnTrack_Task_Flag3= RESET;
u32 ETC_Open_Flag = 0;
u32 Auto_Move_Flag=0;
u8 art=RESET;
int make=0;
uint8_t T_RFID[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
uint8_t RXRFID1[16];




#define t_sp 80
#define	g_mp 420
#define g_sp 50
#define	rl_sp 120
#define	rl_mp 300




															/*道路*/
/*********************************************************************************************************************************/

/*前进函数*/
void Car_Go(int spend,uint16_t temp)
{
	Roadway_mp_syn();	//码盘同步
	Stop_Flag = 0; 
	Go_Flag = 1; 
	wheel_L_Flag = 0;
	wheel_R_Flag = 0;
	wheel_Nav_Flag = 0;
	Back_Flag = 0;
	Track_Flag = 0;
	temp_MP = temp;
	Car_Spend = spend;
	Control(Car_Spend ,Car_Spend);
	while(Stop_Flag!=0x03);
	delay_ms(100);
	
}
/*后退函数*/
void Car_Back(int spend,uint16_t temp)
{
	Roadway_mp_syn();	//码盘同步
	Stop_Flag = 0; 
	Go_Flag = 0; 
	wheel_L_Flag = 0;
	wheel_R_Flag = 0;
	wheel_Nav_Flag = 0;
	Back_Flag = 1;
	Track_Flag = 0;
	temp_MP = temp;
	Car_Spend = spend;
	Control(-Car_Spend ,-Car_Spend);
	while(Stop_Flag!=0x03);
	delay_ms(100);
}

/*循迹函数*/
void Car_Track(int spend)
{
	Roadway_mp_syn();	//码盘同步
	Stop_Flag = 0; 
	Go_Flag = 0; 
	wheel_L_Flag = 0;
	wheel_R_Flag = 0;
	wheel_Nav_Flag = 0;
	Back_Flag = 0;
	Track_Flag = 1;
	Car_Spend = spend;
	Control(Car_Spend ,Car_Spend);
	while(Stop_Flag!=0x01);
	delay_ms(100);
}

/*左转函数*/
void Car_L(int spend)
{
	Roadway_mp_syn();	//码盘同步
	Stop_Flag = 0; 
	Go_Flag = 0; 
	wheel_L_Flag = 1;
	wheel_R_Flag = 0;
	wheel_Nav_Flag = 0;
	Back_Flag = 0;
	Track_Flag = 0;
	Car_Spend = spend;
	Control(-Car_Spend ,Car_Spend);
	while(Stop_Flag!=0x02);
	delay_ms(100);
}

/*右转函数*/
void Car_R(int spend)
{
	Roadway_mp_syn();	//码盘同步
	Stop_Flag = 0; 
	Go_Flag = 0; 
	wheel_L_Flag = 0;
	wheel_R_Flag = 1;
	wheel_Nav_Flag = 0;
	Back_Flag = 0;
	Track_Flag = 0;
	Car_Spend = spend;
	Control(Car_Spend ,-Car_Spend);
	while(Stop_Flag!=0x02);
	delay_ms(100);
}

/*左旋转函数*/
void Car_Left_Nav(uint16_t time)
{	
	Control(-90,90);
	delay_ms(time);
	Control(0,0);
}

/*右旋转函数*/
void Car_Right_Nav(uint16_t time)
{	
	Control(90,-90);
	delay_ms(time);
	Control(0,0);
}
															/*任务*/
/********************************************************************************************************************************/

/******************************************************
 * 函数名：	Gate  向道闸发送开关指令函数
 * 参  数： swch  SET==开，RESET==关
 * 返回值：	无
******************************************************/

void Gate(uint8_t swch)
{
	uint8_t data[]={0x55,0x03,0x01,0x00,0x00,0x00,0x00,0xBB};
	
	if(SET)
	{
		data[3]=0x01;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}else if(RESET)
	{
		data[3]=0x02;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}			
}

/******************************************************
 * 函数名：	Gate_plate  向道闸发送6字节车牌数据开启道闸函数
 * 参  数： plate  		车牌文本数组
 * 返回值： 无
******************************************************/
void Gate_plate(u8* plate)
{
	uint8_t data[]={0x55,0x03,0x10,0x00,0x00,0x00,0x00,0xBB};
	data[3]=plate[0];
	data[4]=plate[1];
	data[5]=plate[2];
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	Send_ZigbeeData_To_Fifo(data,8);
	delay_ms(10);
	data[2]=0x11;
	data[3]=plate[3];
	data[4]=plate[4];
	data[5]=plate[5];
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	Send_ZigbeeData_To_Fifo(data,8);
	delay_ms(10);
}

/******************************************************
 * 函数名： Gate_Return  道闸状态回传函数
 * 参  数： 无  		
 * 返回值： 无
******************************************************/
void Gate_Return(void)
{
	uint8_t data[]={0x55,0x03,0x01,0x00,0x05,0x00,0x00,0xBB};
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	//return data;
}

/******************************************************
 * 函数名： Lamp  竞赛平台转向灯函数
 * 参  数： L=1=开，L=0=关 
		    R=1=开，R=0=关
 * 返回值： 无
******************************************************/
void Lamp(int L,int R)
{	
	if(L==1&&R==1){
		GPIO_ResetBits(GPIOH,GPIO_Pin_10);
		GPIO_ResetBits(GPIOH,GPIO_Pin_11);				
	}else if(L==1&&R==0){
		GPIO_ResetBits(GPIOH,GPIO_Pin_10);
		GPIO_SetBits(GPIOH,GPIO_Pin_11);
	}else if(L==0&&R==1){
		GPIO_SetBits(GPIOH,GPIO_Pin_10);
		GPIO_ResetBits(GPIOH,GPIO_Pin_11);
	}else if(L==0&&R==0){
		GPIO_SetBits(GPIOH,GPIO_Pin_10);
		GPIO_SetBits(GPIOH,GPIO_Pin_11);
	}

}

/******************************************************
 * 函数名： LED_WriteData  数据写入到数码管第一行或第二行函数
 * 参  数： swch=1=第一行，swch=2=第二行 
		    sj 要写入的数据
 * 返回值： 无
******************************************************/
void LED_WriteData(int swch,u8* sj)
{
	uint8_t data[]={0x55,0x04,0x00,0x00,0x00,0x00,0x00,0xBB};
	if(swch==1){
		data[2]=0x01;
		data[3]=sj[0]+sj[1];
		data[4]=sj[2]+sj[3];
		data[5]=sj[4]+sj[5];
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}else if(swch==2){
		data[2]=0x02;
		data[3]=sj[0]+sj[1];
		data[4]=sj[2]+sj[3];
		data[5]=sj[4]+sj[5];
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}
}

/******************************************************
 * 函数名： LED_Time  数码管计时函数
 * 参  数： swch=0；关闭
			swch=1；打开
			swch=2；清零
 * 返回值： 无
******************************************************/
void LED_Time(int swch)
{
	uint8_t data[]={0x55,0x04,0x03,0x00,0x00,0x00,0x00,0xBB};
	if(swch==0){
		data[3]=0x00;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}else if(swch==1){
		data[3]=0x01;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}else if(swch==2){
		data[3]=0x02;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
	}
}

/******************************************************
 * 函数名： LED_Dis  数码二排显示距离函数
 * 参  数： p 距离值
 * 返回值： 无
******************************************************/
void LED_Dis(uint16_t p)
{
	uint8_t data[]={0x55,0x04,0x04,0x00,0x00,0x00,0x00,0xBB};
	data[4]=p/100%10;
	data[5]=((p/10%10)<<4)+p%10;
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	Send_ZigbeeData_To_Fifo(data,8);
}

/******************************************************
 * 函数名： Buzzer  蜂鸣器开启与关闭函数
 * 参  数： swch SET RESET
 * 返回值： 无
******************************************************/
void Buzzer(uint8_t swch)
{
	if(swch==SET){
		Set_tba_Beep(SET);
	}else if(swch==RESET){
		Set_tba_Beep(RESET);
	}
}

/******************************************************
 * 函数名： Modify_Light  智能路灯档位调节函数
 * 参  数： End_Goal 目标档位
 * 返回值： 无
******************************************************/
void Modify_Light(u8 End_Goal)
{
	unsigned int thisCandela = 0; //当前光档位
	unsigned int afterCandela = 0;//调整后光档位
	do{
		thisCandela = Get_Bh_Value();
		Infrared_Send(H_1,4); //加一档
		delay_ms(500);delay_ms(500);
		afterCandela = Get_Bh_Value();
	}
	while(thisCandela < afterCandela);
	u8 temp = (End_Goal-1)%4;
	switch (temp)
	{
		case 0 : 
			break ;								
		case 1 : 
			Infrared_Send(H_1,4); 
			break ; 
		case 2 : 
			Infrared_Send(H_2,4); 
			break ;
		case 3 : 	
			Infrared_Send(H_3,4); 
			break ;
	}	
}

/******************************************************
 * 函数名： Stereo_Show  立体显示车牌和坐标函数
 * 参  数： plate:车牌数据
			xy	 :坐标数据
 * 返回值： 无
******************************************************/
void Stereo_Show(u8* plate,u8* xy)
{
	u8 data[]={0xFF,0x00,0x00,0x00,0x00,0x00};
	data[1]=0x20;
	data[2]=plate[0];
	data[3]=plate[1];
	data[4]=plate[2];
	data[5]=plate[3];
	Infrared_Send(data,6);
	delay_ms(100);
	data[1]=0x10;
	data[2]=plate[4];
	data[3]=plate[5];
	data[4]=xy[0];
	data[5]=xy[1];
	delay_ms(100);
	Infrared_Send(data,6);
}
/*void testC(int i){
	uint8_t data[8] = {0x55,0x04,0x02,0x00,0x00,0x00,0x00,0xBB};
	Ultrasonic_Ranging();//超声波测距，得到dis;
	//dis是执行超声波测距后计算出的距离，i是小车要移动到的目标距离
	if(dis<i)
	{//太近，缓缓后退
		while(dis==i){
			do{
				Car_Back(20,20);//小车后退函数
				delay_ms(10);	//延时防止指令对冲
				Ultrasonic_Ranging();//再次超声波测距，得到dis;
			
			}while(dis>=i);
		}
		data[3]=0x00;
		data[4]=dis/100%10;
		data[5]=((dis/10%10)*16+dis%10);
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
		delay_ms(500);
	}else if(dis==i)
	{
		data[3]=0x00;
		data[4]=dis/100%10;
		data[5]=((dis/10%10)*16+dis%10);
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
		delay_ms(500);
	}else if(dis>i)
	{//缓缓前进
		do{
			Car_Go(20,10);
			delay_ms(10);
			Ultrasonic_Ranging();
		}while(dis<=i);
		data[3]=0x00;
		data[4]=dis/100%10;
		data[5]=((dis/10%10)*16+dis%10);
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
		delay_ms(500);
	}
	
}*/

/******************************************************
 * 函数名： Ranging  超声波测距函数
 * 参  数： plate:车牌数据
			xy	 :坐标数据
 * 返回值： 无
******************************************************/
void Ranging(void)
{	
	uint8_t data[8] = {0x55,0x04,0x02,0x00,0x00,0x00,0x00,0xBB};
	int mp = 0;
	Ultrasonic_Ranging();
	while(1)
	{
		mp = (dis-300)*2.8;
		if(dis < 297)
		{
			Car_Back(30,-mp);
		}
		if(dis > 303)
		{
			Car_Go(30,mp);
		}
		if(dis > 297 && dis <303)
		{
			break;
		}
		Ultrasonic_Ranging();
		data[3]=0x00;
		data[4]=dis/100%10;
		data[5]=((dis/10%10)*16+dis%10);
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);
		delay_ms(300);	
	}
}

/******************************************************
 * 函数名： TFT_Plate  TFT显示车牌函数
 * 参  数： p:车牌数据	
 * 返回值： 无
******************************************************/
void TFT_Plate(u8* p)
{
	uint8_t data[]={0x55,0x0B,0x20,0x00,0x00,0x00,0x00,0xBB};
	data[3]=p[0];
	data[4]=p[1];
	data[5]=p[2];
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	delay_ms(10);
	Send_ZigbeeData_To_Fifo(data,8);
	delay_ms(150);
	data[2]=0x21;
	data[3]=p[3];
	data[4]=p[4];
	data[5]=p[5];
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	delay_ms(10);
	Send_ZigbeeData_To_Fifo(data,8);	
	delay_ms(10);
}

/******************************************************
 * 函数名： TFT_Time  TFT计时函数
 * 参  数： i:关闭=0，打开=1，清零=2
 * 返回值： 无
******************************************************/
void TFT_Time(int i)
{
	uint8_t data[]={0x55,0x0B,0x00,0x00,0x00,0x00,0x00,0xBB};
	data[2]=0x30;
	if(i==0)
	{
		data[3]=0x00;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);	
	}
	else if(i==1)
	{
		data[3]=0x01;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);	
	}
	else if(i==2)
	{
		data[3]=0x02;
		data[6]=(data[2]+data[3]+data[4]+data[5])%256;
		Send_ZigbeeData_To_Fifo(data,8);	
	}
}

/******************************************************
 * 函数名： TFT_HEX  TFT-HEX显示函数(16进制显示)
 * 参  数： p 数据
 * 返回值： 无
******************************************************/
void TFT_HEX(uint16_t* p)
{
	uint16_t data[]={0x55,0x0B,0x00,0x00,0x00,0x00,0x00,0xBB};
	
}

/******************************************************
 * 函数名： TFT_Dis  TFT距离显示函数
 * 参  数： p 距离
 * 返回值： 无
******************************************************/

void TFT_Dis(uint8_t* p)
{
	uint8_t data[]={0x55,0x0B,0x50,0x00,0x00,0x00,0x00,0xBB};
	data[3]=p[0];
	data[4]=p[1];
	data[5]=p[2];
	data[6]=(data[2]=data[3]=data[4]+data[5])%256;
	Send_ZigbeeData_To_Fifo(data,8);
}
							
/******************************************************
 * 函数名： Find_Card  寻找RFID卡片函数
 * 参  数： 无
 * 返回值： 无
******************************************************/
void FindCard(void)
{
	RFID_Task_Flag=SET;
	Car_Track(60);
	Car_Go(50,300);
	if(ReadCard()== 1)
	{
		Car_Go(50,50);
		if(ReadCard()== 1)
		{
			Car_Back(50,100);				
			if(ReadCard()== 1)
			{
				Car_Go(50,50);
			}
		}
	}
}

/******************************************************
 * 函数名： ReadCard  读取RFID卡片函数
 * 参  数： 无
 * 返回值： 无
******************************************************/
int ReadCard()
{
	char status = MI_ERR;
	uint8_t CT[2];									//卡类型
	uint8_t SN[4]; 									//卡号
	uint8_t KEY[6]={0xff,0xff,0xff,0xff,0xff,0xff}; //密钥
	//uint8_t s = 0x01;       						  	
	uint8_t RXRFIDH[8];
	#define  DATA_LEN    16                     	//定义数据字节长度	
	status = PcdRequest(PICC_REQALL,CT);		//寻卡
	if(status == MI_OK)							//寻卡成功
	{
		status=MI_ERR;
		status = PcdAnticoll(SN);				//防冲撞
		if(status == MI_OK)
		{
			status=MI_ERR;				
			status =PcdSelect(SN);				//选定此卡
			if(status == MI_OK)					//选定成功
			{
				status=MI_ERR;	
									/*A密钥 块地址 密钥	*/	
				status =PcdAuthState(0x60,0x03,KEY,SN);		//验证密钥
				if(status == MI_OK)					{
					status = MI_ERR;
								/*块地址 写入的参数*/
					status = PcdWrite(0x01,T_RFID);			//写卡
					if(status == MI_OK)						
					{
						status = MI_ERR;
						//SYN_TTS("写卡成功");
					}			/*块地址 读出的数据*/									
					status=PcdRead(0x01,RXRFID1);				//读卡
					if(status == MI_OK)
					{	//SYN_TTS("读卡成功");	
						//delay_ms(50);
						//读卡成功
						status = MI_ERR;
						//SYN_TTS("接收到的RFID已显示"); 
						
									
					}else
					{
							return 1;
					}						
					}else
					{
						return 1;
					}
				}else
				{
					return 1;
				}
			}else
			{
				return 1;
			}
		}else
		{
			return 1;
		}
	return 0;
}

/******************************************************
 * 函数名： Reverse_Track  反循迹函数
 * 参  数：  i=1   第一种反循迹
			i=2	  第二种反循迹
			i=3	  第三种反循迹
 * 返回值： 无
******************************************************/
void Reverse_Track(int i){
	if(i==1)
	{
		UnTrack_Task_Flag1 = SET;
		Car_Track(60);
		if(art==SET){
			Car_Go(50,70);
			//Car_Track(60);
		}
	}
	if(i==2)
	{
		UnTrack_Task_Flag1 = SET;
		Car_Track(60);
	}
	if(i==3)
	{
		UnTrack_Task_Flag1 = SET;
		Car_Track(60);
	}
}

/******************************************************
 * 函数名： QR_Code  接收二维码函数
 * 参  数： swch==1  发送二维码识别 ，swch==2 接收二维码数据
 * 返回值： 无
******************************************************/
void QR_Code(uint8_t swch)
{
	if(swch==1)
	{
		Principal_Tab[2]=0xC2;
		Send_WifiData_To_Fifo(Principal_Tab,12);
	}else if(swch==2)
	{
		for(int i = 0;i < 6; i ++)
		{
			Voice_Broadast("");
			//SYN_TTS("",Infrared_Tab[i]);
		}
	}
}

/******************************************************
 * 函数名： Plate_Code  接收车牌函数
 * 参  数： swch==1  发送车牌识别 ，swch==2 接收识别数据
 * 返回值： 无
******************************************************/
void Plate_Code(uint8_t swch)
{
	if(swch==1)
	{
		Principal_Tab[2]=0xC2;
		Send_WifiData_To_Fifo(Principal_Tab,12);
	}else if(2)
	{
		for(int i = 0;i < 6; i ++)
		{
			Voice_Broadast("");
			//SYN_TTS("",Infrared_Tab[i]);
		}
			
	}
	
}

/******************************************************
 * 函数名： Alertor  报警器报警函数
 * 参  数： swch==1  开启报警 ，swch==2 关闭报警
 * 返回值： 无
******************************************************/
void Alertor(uint8_t swch)
{
	if(swch == SET)
	{
		Infrared_Send(HW_K,6);
		//delay_ms(500);
	}else if(swch == RESET){
		Infrared_Send(HW_G,6);
	//delay_ms(500);
	}
}

/******************************************************
 * 函数名： Alertor_Code  修改报警器6位报警码函数
 * 参  数： p  6位报警码
 * 返回值： 无
******************************************************/
void Alertor_Code(u8* p)
{
	uint8_t data[]={0x55,0x07,0x00,0x00,0x00,0x00,0x00,0xBB};
	//前三
	data[2]=0x10;
	data[3]=p[0];
	data[4]=p[1];
	data[5]=p[2];
	data[6]=(data[2]=data[3]=data[4]+data[5])%256;
	delay_ms(50);
	Infrared_Send(data,8);
	delay_ms(50);
	//后三
	data[2]=0x11;
	data[3]=p[3];
	data[4]=p[4];
	data[5]=p[5];
	data[6]=(data[2]=data[3]=data[4]+data[5])%256;
	delay_ms(50);
	Infrared_Send(data,8);
	delay_ms(50);
}

/******************************************************
 * 函数名： Charge  开启无线充电站函数
 * 参  数： 无
 * 返回值： 无
******************************************************/
void Charge(void)
{
	uint8_t data[]={0x55,0x0A,0x01,0x01,0x00,0x00,0x00,0xBB};
	data[6]=(data[2]=data[3]=data[4]+data[5])%256;
	Send_ZigbeeData_To_Fifo(data,8);
}

/******************************************************
 * 函数名： Garage  设置立体车库层数函数
 * 参  数： floor 层数
 * 返回值： 无
******************************************************/
void Garage(u8 floor)
{
		int i = 0;
		u8 JXB[8] = {0x55,0x0D,0x01,0x00,0x00,0x00,0x00,0xbb};
		JXB[3] = floor;
		JXB[6]=(JXB[2]+JXB[3]+JXB[4]+JXB[5])%256;
		delay_us(10);
		Send_ZigbeeData_To_Fifo(JXB, 8);
		for(i = 0; i < 15; i++)
		{
			delay_ms(500); //车库升降延时
		}
}

/******************************************************
 * 函数名：	Voice_Broadast  语音播报指定文本信息
 * 参  数： *p  指向文本数据首地址
 * 返回值：	无
******************************************************/
void Voice_Broadast(char *p)
{
	
	uint8_t test1[13]={0xFD,0x00,0x06,0x01,0x03,0xD1,0x53,0x01,0x90};
			
	uint8_t YY_Init[5]={0xFD,0x00,0x00,0x01,0x01};//GBK
	uint16_t data_size=strlen(p);//返回长度
	YY_Init[1]=(data_size)/256;//高字节
	YY_Init[2]=(data_size)%256;//低字节
	
	Send_ZigbeeData_To_Fifo(YY_Init, 5);		// ZigBee数据发送
	Send_ZigbeeData_To_Fifo((u8 *)p, data_size);
}

/*void run()
{
	
	//双闪，蜂鸣器，led
	switch(make)
	{
		case 5:
			Lamp(1,1);
			delay_ms(50);
			Buzzer(SET);
			delay_ms(100);
			LED_Time(2);
			delay_ms(20);
			LED_Time(1);
			delay_ms(20);
			//发送车牌打开道闸
			Voice_Broadast("车牌");
			uint8_t plate[]={'A','2','1','4','8','1'};
			Gate_plate(plate);
			delay_ms(500);delay_ms(500);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_R(rl_sp);
			//语音：读取 三张 RFID卡
			Voice_Broadast("寻卡");
			FindCard();
			FindCard();
			Car_Go(g_sp,140);
			//右转
			Car_R(rl_sp);
			//车牌图片
			TFT_Plate(plate);
			Voice_Broadast("开始识别车牌");
			
			
			
			
			
			//图形识别
			uint8_t test8[13]={0xFD,0x00,0x06,0x01,0x03,0xFE,0x56,0x62,0x5F};
			Send_ZigbeeData_To_Fifo(test8, 13);
			delay_ms(350);
			uint8_t test9[13]={0xFD,0x00,0x06,0x01,0x03,0xC6,0x8B,0x2B,0x52};
			Send_ZigbeeData_To_Fifo(test9, 13);
			delay_ms(350);
			//完
			//左转
			Car_L(rl_sp,rl_mp);
			
			xk();
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_L(rl_sp,rl_mp);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_L(rl_sp,rl_mp);
			
			uint8_t test10[13]={0xFD,0x00,0x06,0x01,0x03,0xFB,0x8B,0xD6,0x53};
			Send_ZigbeeData_To_Fifo(test10, 13);
			delay_ms(350);
			uint8_t test11[13]={0xFD,0x00,0x06,0x01,0x03,0x52,0x00,0x46,0x00};
			Send_ZigbeeData_To_Fifo(test11, 13);
			delay_ms(350);
			uint8_t test12[13]={0xFD,0x00,0x06,0x01,0x03,0x49,0x00,0x44,0x00};
			Send_ZigbeeData_To_Fifo(test12, 13);
			delay_ms(350);
			uint8_t test13[13]={0xFD,0x00,0x06,0x01,0x03,0x61,0x53,0x47,0x72};
			Send_ZigbeeData_To_Fifo(test13, 13);
			delay_ms(350);
			
			xk();
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_L(rl_sp,rl_mp);
			//角度
			Car_Left_Nav(80,350);
			//发送车牌坐标
			uint8_t test14[13]={0xFD,0x00,0x06,0x01,0x03,0xD1,0x53,0x01,0x90};
			Send_ZigbeeData_To_Fifo(test14, 13);
			delay_ms(350);
			
			uint8_t test15[13]={0xFD,0x00,0x06,0x01,0x03,0x66,0x8F,0x4C,0x72};
			Send_ZigbeeData_To_Fifo(test15, 13);
			delay_ms(350);
			
			uint8_t test16[13]={0xFD,0x00,0x06,0x01,0x03,0x50,0x57,0x07,0x68};
			Send_ZigbeeData_To_Fifo(test16, 13);
			delay_ms(350);
			u8 p2[]={'D','4'};
			ltxs(plate,p2);
			Car_Right_Nav(80,400);
			Car_R(rl_sp,rl_mp);
			//etc
			ETC_Open_Flag=1;
			Car_Track(t_sp);
			//超声波
			csb();
			//二维码
			QR(1);
			delay_ms(100);
			QR(2);
			Car_R(rl_sp,rl_mp);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			//设置二档路灯
			uint8_t test17[13]={0xFD,0x00,0x06,0x01,0x03,0x8C,0x4E,0x63,0x68};
			Send_ZigbeeData_To_Fifo(test17, 13);
			delay_ms(350);
			uint8_t test18[13]={0xFD,0x00,0x06,0x01,0x03,0xEF,0x8D,0x6F,0x70};
			Send_ZigbeeData_To_Fifo(test18, 13);
			delay_ms(350);
			
			Modify_Light(2);
			Car_R(rl_sp,rl_mp);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_Right_Nav(80,350);
			uint8_t test19[13]={0xFD,0x00,0x06,0x01,0x03,0xA5,0x62,0x66,0x8B};
			Send_ZigbeeData_To_Fifo(test19, 13);
			delay_ms(350);
			//报警器
			
			fht(SET);
			delay_ms(500);delay_ms(500);
			fht(RESET);
			Car_Right_Nav(80,370);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_L(rl_sp,rl_mp);
			Car_Go(g_sp,g_mp);
			uint8_t test20[13]={0xFD,0x00,0x06,0x01,0x03,0xFB,0x8B,0xD6,0x53};
			Send_ZigbeeData_To_Fifo(test20, 13);
			delay_ms(350);
			uint8_t test21[13]={0xFD,0x00,0x06,0x01,0x03,0x52,0x00,0x46,0x00};
			Send_ZigbeeData_To_Fifo(test21, 13);
			delay_ms(350);
			uint8_t test22[13]={0xFD,0x00,0x06,0x01,0x03,0x49,0x00,0x44,0x00};
			Send_ZigbeeData_To_Fifo(test22, 13);
			delay_ms(350);
			uint8_t test23[13]={0xFD,0x00,0x06,0x01,0x03,0x61,0x53,0x47,0x72};
			Send_ZigbeeData_To_Fifo(test23, 13);
			delay_ms(350);
			xk();
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_L(rl_sp,rl_mp);
			Car_Back(80,2900);
			//车库
			uint8_t test24[13]={0xFD,0x00,0x06,0x01,0x03,0x8C,0x4E,0x42,0x5C};
			Send_ZigbeeData_To_Fifo(test24, 13);
			delay_ms(350);
			uint8_t test27[13]={0xFD,0x00,0x06,0x01,0x03,0x66,0x8F,0x93,0x5E};
			Send_ZigbeeData_To_Fifo(test27, 13);
			delay_ms(350);
			JXB(2);
			uint8_t test25[13]={0xFD,0x00,0x06,0x01,0x03,0xE0,0x65,0xBF,0x7E};
			Send_ZigbeeData_To_Fifo(test25, 13);
			delay_ms(350);
			uint8_t test28[13]={0xFD,0x00,0x06,0x01,0x03,0x45,0x51,0x35,0x75};
			Send_ZigbeeData_To_Fifo(test28, 13);
			delay_ms(350);
			cxf();
			uint8_t test26[13]={0xFD,0x00,0x06,0x01,0x03,0xD3,0x7E,0x5F,0x67};
			Send_ZigbeeData_To_Fifo(test26, 13);
			delay_ms(350);
			smg2(0);
			//充电
			//计时结束
			
			make=22;
			break;
		case 10:
			delay_ms(10);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_R(rl_sp,rl_mp);
			make=15;
			break;
		case 15:
			//寻卡
			xk();
			xk();
			Car_Go(g_sp,200);
			make=21;
			break;
		case 20:
			break;
		case 25:
			break;
		case 30:
			break;
		case 35:
			break;
		case 40:
			break;
		
	}
	*/

