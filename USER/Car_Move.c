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
									����������
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




															/*��·*/
/*********************************************************************************************************************************/

/*ǰ������*/
void Car_Go(int spend,uint16_t temp)
{
	Roadway_mp_syn();	//����ͬ��
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
/*���˺���*/
void Car_Back(int spend,uint16_t temp)
{
	Roadway_mp_syn();	//����ͬ��
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

/*ѭ������*/
void Car_Track(int spend)
{
	Roadway_mp_syn();	//����ͬ��
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

/*��ת����*/
void Car_L(int spend)
{
	Roadway_mp_syn();	//����ͬ��
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

/*��ת����*/
void Car_R(int spend)
{
	Roadway_mp_syn();	//����ͬ��
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

/*����ת����*/
void Car_Left_Nav(uint16_t time)
{	
	Control(-90,90);
	delay_ms(time);
	Control(0,0);
}

/*����ת����*/
void Car_Right_Nav(uint16_t time)
{	
	Control(90,-90);
	delay_ms(time);
	Control(0,0);
}
															/*����*/
/********************************************************************************************************************************/

/******************************************************
 * ��������	Gate  ���բ���Ϳ���ָ���
 * ��  ���� swch  SET==����RESET==��
 * ����ֵ��	��
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
 * ��������	Gate_plate  ���բ����6�ֽڳ������ݿ�����բ����
 * ��  ���� plate  		�����ı�����
 * ����ֵ�� ��
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
 * �������� Gate_Return  ��բ״̬�ش�����
 * ��  ���� ��  		
 * ����ֵ�� ��
******************************************************/
void Gate_Return(void)
{
	uint8_t data[]={0x55,0x03,0x01,0x00,0x05,0x00,0x00,0xBB};
	data[6]=(data[2]+data[3]+data[4]+data[5])%256;
	//return data;
}

/******************************************************
 * �������� Lamp  ����ƽ̨ת��ƺ���
 * ��  ���� L=1=����L=0=�� 
		    R=1=����R=0=��
 * ����ֵ�� ��
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
 * �������� LED_WriteData  ����д�뵽����ܵ�һ�л�ڶ��к���
 * ��  ���� swch=1=��һ�У�swch=2=�ڶ��� 
		    sj Ҫд�������
 * ����ֵ�� ��
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
 * �������� LED_Time  ����ܼ�ʱ����
 * ��  ���� swch=0���ر�
			swch=1����
			swch=2������
 * ����ֵ�� ��
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
 * �������� LED_Dis  ���������ʾ���뺯��
 * ��  ���� p ����ֵ
 * ����ֵ�� ��
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
 * �������� Buzzer  ������������رպ���
 * ��  ���� swch SET RESET
 * ����ֵ�� ��
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
 * �������� Modify_Light  ����·�Ƶ�λ���ں���
 * ��  ���� End_Goal Ŀ�굵λ
 * ����ֵ�� ��
******************************************************/
void Modify_Light(u8 End_Goal)
{
	unsigned int thisCandela = 0; //��ǰ�⵵λ
	unsigned int afterCandela = 0;//������⵵λ
	do{
		thisCandela = Get_Bh_Value();
		Infrared_Send(H_1,4); //��һ��
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
 * �������� Stereo_Show  ������ʾ���ƺ����꺯��
 * ��  ���� plate:��������
			xy	 :��������
 * ����ֵ�� ��
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
	Ultrasonic_Ranging();//��������࣬�õ�dis;
	//dis��ִ�г��������������ľ��룬i��С��Ҫ�ƶ�����Ŀ�����
	if(dis<i)
	{//̫������������
		while(dis==i){
			do{
				Car_Back(20,20);//С�����˺���
				delay_ms(10);	//��ʱ��ָֹ��Գ�
				Ultrasonic_Ranging();//�ٴγ�������࣬�õ�dis;
			
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
	{//����ǰ��
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
 * �������� Ranging  ��������ຯ��
 * ��  ���� plate:��������
			xy	 :��������
 * ����ֵ�� ��
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
 * �������� TFT_Plate  TFT��ʾ���ƺ���
 * ��  ���� p:��������	
 * ����ֵ�� ��
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
 * �������� TFT_Time  TFT��ʱ����
 * ��  ���� i:�ر�=0����=1������=2
 * ����ֵ�� ��
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
 * �������� TFT_HEX  TFT-HEX��ʾ����(16������ʾ)
 * ��  ���� p ����
 * ����ֵ�� ��
******************************************************/
void TFT_HEX(uint16_t* p)
{
	uint16_t data[]={0x55,0x0B,0x00,0x00,0x00,0x00,0x00,0xBB};
	
}

/******************************************************
 * �������� TFT_Dis  TFT������ʾ����
 * ��  ���� p ����
 * ����ֵ�� ��
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
 * �������� Find_Card  Ѱ��RFID��Ƭ����
 * ��  ���� ��
 * ����ֵ�� ��
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
 * �������� ReadCard  ��ȡRFID��Ƭ����
 * ��  ���� ��
 * ����ֵ�� ��
******************************************************/
int ReadCard()
{
	char status = MI_ERR;
	uint8_t CT[2];									//������
	uint8_t SN[4]; 									//����
	uint8_t KEY[6]={0xff,0xff,0xff,0xff,0xff,0xff}; //��Կ
	//uint8_t s = 0x01;       						  	
	uint8_t RXRFIDH[8];
	#define  DATA_LEN    16                     	//���������ֽڳ���	
	status = PcdRequest(PICC_REQALL,CT);		//Ѱ��
	if(status == MI_OK)							//Ѱ���ɹ�
	{
		status=MI_ERR;
		status = PcdAnticoll(SN);				//����ײ
		if(status == MI_OK)
		{
			status=MI_ERR;				
			status =PcdSelect(SN);				//ѡ���˿�
			if(status == MI_OK)					//ѡ���ɹ�
			{
				status=MI_ERR;	
									/*A��Կ ���ַ ��Կ	*/	
				status =PcdAuthState(0x60,0x03,KEY,SN);		//��֤��Կ
				if(status == MI_OK)					{
					status = MI_ERR;
								/*���ַ д��Ĳ���*/
					status = PcdWrite(0x01,T_RFID);			//д��
					if(status == MI_OK)						
					{
						status = MI_ERR;
						//SYN_TTS("д���ɹ�");
					}			/*���ַ ����������*/									
					status=PcdRead(0x01,RXRFID1);				//����
					if(status == MI_OK)
					{	//SYN_TTS("�����ɹ�");	
						//delay_ms(50);
						//�����ɹ�
						status = MI_ERR;
						//SYN_TTS("���յ���RFID����ʾ"); 
						
									
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
 * �������� Reverse_Track  ��ѭ������
 * ��  ����  i=1   ��һ�ַ�ѭ��
			i=2	  �ڶ��ַ�ѭ��
			i=3	  �����ַ�ѭ��
 * ����ֵ�� ��
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
 * �������� QR_Code  ���ն�ά�뺯��
 * ��  ���� swch==1  ���Ͷ�ά��ʶ�� ��swch==2 ���ն�ά������
 * ����ֵ�� ��
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
 * �������� Plate_Code  ���ճ��ƺ���
 * ��  ���� swch==1  ���ͳ���ʶ�� ��swch==2 ����ʶ������
 * ����ֵ�� ��
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
 * �������� Alertor  ��������������
 * ��  ���� swch==1  �������� ��swch==2 �رձ���
 * ����ֵ�� ��
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
 * �������� Alertor_Code  �޸ı�����6λ�����뺯��
 * ��  ���� p  6λ������
 * ����ֵ�� ��
******************************************************/
void Alertor_Code(u8* p)
{
	uint8_t data[]={0x55,0x07,0x00,0x00,0x00,0x00,0x00,0xBB};
	//ǰ��
	data[2]=0x10;
	data[3]=p[0];
	data[4]=p[1];
	data[5]=p[2];
	data[6]=(data[2]=data[3]=data[4]+data[5])%256;
	delay_ms(50);
	Infrared_Send(data,8);
	delay_ms(50);
	//����
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
 * �������� Charge  �������߳��վ����
 * ��  ���� ��
 * ����ֵ�� ��
******************************************************/
void Charge(void)
{
	uint8_t data[]={0x55,0x0A,0x01,0x01,0x00,0x00,0x00,0xBB};
	data[6]=(data[2]=data[3]=data[4]+data[5])%256;
	Send_ZigbeeData_To_Fifo(data,8);
}

/******************************************************
 * �������� Garage  �������峵���������
 * ��  ���� floor ����
 * ����ֵ�� ��
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
			delay_ms(500); //����������ʱ
		}
}

/******************************************************
 * ��������	Voice_Broadast  ��������ָ���ı���Ϣ
 * ��  ���� *p  ָ���ı������׵�ַ
 * ����ֵ��	��
******************************************************/
void Voice_Broadast(char *p)
{
	
	uint8_t test1[13]={0xFD,0x00,0x06,0x01,0x03,0xD1,0x53,0x01,0x90};
			
	uint8_t YY_Init[5]={0xFD,0x00,0x00,0x01,0x01};//GBK
	uint16_t data_size=strlen(p);//���س���
	YY_Init[1]=(data_size)/256;//���ֽ�
	YY_Init[2]=(data_size)%256;//���ֽ�
	
	Send_ZigbeeData_To_Fifo(YY_Init, 5);		// ZigBee���ݷ���
	Send_ZigbeeData_To_Fifo((u8 *)p, data_size);
}

/*void run()
{
	
	//˫������������led
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
			//���ͳ��ƴ򿪵�բ
			Voice_Broadast("����");
			uint8_t plate[]={'A','2','1','4','8','1'};
			Gate_plate(plate);
			delay_ms(500);delay_ms(500);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			Car_R(rl_sp);
			//��������ȡ ���� RFID��
			Voice_Broadast("Ѱ��");
			FindCard();
			FindCard();
			Car_Go(g_sp,140);
			//��ת
			Car_R(rl_sp);
			//����ͼƬ
			TFT_Plate(plate);
			Voice_Broadast("��ʼʶ����");
			
			
			
			
			
			//ͼ��ʶ��
			uint8_t test8[13]={0xFD,0x00,0x06,0x01,0x03,0xFE,0x56,0x62,0x5F};
			Send_ZigbeeData_To_Fifo(test8, 13);
			delay_ms(350);
			uint8_t test9[13]={0xFD,0x00,0x06,0x01,0x03,0xC6,0x8B,0x2B,0x52};
			Send_ZigbeeData_To_Fifo(test9, 13);
			delay_ms(350);
			//��
			//��ת
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
			//�Ƕ�
			Car_Left_Nav(80,350);
			//���ͳ�������
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
			//������
			csb();
			//��ά��
			QR(1);
			delay_ms(100);
			QR(2);
			Car_R(rl_sp,rl_mp);
			Car_Track(t_sp);
			Car_Go(g_sp,g_mp);
			//���ö���·��
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
			//������
			
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
			//����
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
			//���
			//��ʱ����
			
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
			//Ѱ��
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

