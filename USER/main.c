/**
�������ƣ�2018�����ۺϳ���			
�޸�ʱ�䣺2018.05.15
*/
#include <stdio.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "infrared.h"
#include "cba.h"
#include "ultrasonic.h"
#include "canp_hostcom.h"
#include "hard_can.h"
#include "bh1750.h"
#include "syn7318.h"
#include "power_check.h"
#include "can_user.h"
#include "data_base.h"
#include "roadway_check.h"
#include "tba.h"
#include "data_base.h"
#include "swopt_drv.h"
#include "uart_a72.h"
#include "Can_check.h"
#include "delay.h"
#include "can_user.h"
#include "Timer.h"
#include "Rc522.h"

//�µ���
#include "Car_move.h"


RCC_ClocksTypeDef RCC_Clocks;

uint16_t main_cont;

/**
�������ܣ�Ӳ����ʼ��
��    ������
�� �� ֵ����
*/
void Hardware_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);					//�жϷ���

	delay_init(168);
	
	Tba_Init();														//������ʼ��
	Infrared_Init();												//�����ʼ��
	Cba_Init();														//���İ��ʼ��
	Ultrasonic_Init();												//��������ʼ��
	Hard_Can_Init();												//CAN���߳�ʼ��
	BH1750_Configure();												//BH1750��ʼ������
	SYN7318_Init();													//����ʶ���ʼ��
	Electricity_Init();												//��������ʼ��

	UartA72_Init();
	Can_check_Init(83,7);											//CAN���߶�ʱ����ʼ��
	roadway_check_TimInit(167,1999);								//·�����
	Timer_Init(167,999);										    //��������ͨѶʱ��֡
	Readcard_daivce_Init();											//RFID��ʼ��
}


uint8_t open_road_buf[] = {0x55,0x03,0x01,0x01,0x00,0x00,0x02,0xBB};			//��բ����
uint8_t test_buf[] = {0xFD,0x00,0x06,0x01,0x01,0xC4,0xFA,0xBA,0xC3};			//�������������á�
uint8_t repo_buf[] = {0x03,0x05,0x14,0x45,0xDE,0x92};							//�򿪺��ⱨ��

	
/**
�������ܣ��������
��    ������
�� �� ֵ����
*/
void KEY_Check()
{
	if(S1 == 0)
	{
		delay_ms(10);
		if(S1 == 0)
		{
			//Auto_Move_Flag=1;
			//ETC_Open_Flag = 1;
			
			//make=5;
			//Car_g(100);
			//Reverse_Track(1);
			
			
			//uint8_t test1[13]={0xFD,0x00,0x06,0x01,0x03,0xD1,0x53,0x01,0x90};
			//Send_ZigbeeData_To_Fifo(test1, 13);
			//delay_ms(350);
			
			//uint8_t test2[13]={0xFD,0x00,0x06,0x01,0x03,0x66,0x8F,0x4C,0x72};
			//Send_ZigbeeData_To_Fifo(test2, 13);
			//Car_Go(50,250);
			//Car_Back(80,600);
			//Car_Track(80);
			//Car_Go(50,420);
			//Car_R(120,300);//ǰ������λ�ã�����ת����һ��
			//Gate(RESET);
			//uin
			//xk();
			//delay_ms(100);
			//Car_Go(50,300);
			//xk();
			//TTS();
			//uint8_t data[]={'A','2','H','4','8','1'};
			//Gate_plate(data);
			//zxd(0,0);
			//smg1(1,data);
			//smg2(2);
			//fmq(SET);
			//delay_ms(600);delay_ms(600);
			//fmq(RESET);
			//Ultrasonic_Ranging();
			//Modify_Light(3);
			//testC(300);
			//csb();
			//TFT_Plate(data);
			//TFT_js(1);
			//delay_ms(600);delay_ms(600);delay_ms(600);delay_ms(600);
			//TFT_js(0);
			//delay_ms(600);delay_ms(600);delay_ms(600);delay_ms(600);delay_ms(600);delay_ms(600);
			//TFT_js(2);
			//uint8_t p[]={1,2,3};
			//TFT_JL(p);
			//xk();
			//xk();
			//xk();
			//LED1 = !LED1;
			//while(!S1);
			//Send_ZigbeeData_To_Fifo(open_road_buf, 8);			//�򿪵�բ
			//Car_Go(70,50);
		}
	}
	if(S2 == 0)
	{
		delay_ms(10);
		if(S2 == 0)
		{
			//LED2 = !LED2;
			while(!S2);
			/*Infrared_Send(repo_buf,6);							//�򿪱�����
			Send_ZigbeeData_To_Fifo(test_buf, 13);				//��������
			*/
			Reverse_Track(1);
		}
	}
	if(S3 == 0)
	{
		delay_ms(10);
		if(S3 == 0)
		{
			//LED3 = !LED3;
			while(!S3);
			//SYN_7318_One_test(1,0);								//����ʶ�� 
			Reverse_Track(3);
		}
	}
	
	if(S4 == 0)
	{
		delay_ms(10);
		if(S4 == 0)
		{		
			//LED4 = !LED4;
			while(!S4);			
			//Read_Card();
			//Car_L(50,30);			
			//RFID����
			//Car_Left_Nav(80,350);
			//Car_Right_Nav(80,400);
			//uint8_t plate[]={'A','2','1','4','8','1'};
			//u8 p2[]={'D','4'};
			//ltxs(plate,p2);
			//JXB(3);
			Car_Track(100);
		}
	}
}

static uint32_t Power_check_times;		  //�����������
volatile static uint32_t LED_twinkle_times;		  //LED��˸����
static uint32_t WIFI_Upload_data_times;   //ͨ��Wifi�ϴ���������
static uint32_t RFID_Init_Check_times;

int main(void)
{	
	uint16_t Light_Value = 0;				//��ǿ��ֵ	
	uint16_t CodedDisk_Value = 0;			//����
	uint16_t Nav_Value = 0;					//�Ƕ�
	
	Hardware_Init();						//Ӳ����ʼ��
	
	LED_twinkle_times =  gt_get() + 50;     
	Power_check_times =  gt_get() + 200;
	WIFI_Upload_data_times = gt_get() + 200;
	RFID_Init_Check_times = gt_get()+200;
	
	Principal_Tab[0] = 0x55;
	Principal_Tab[1] = 0xAA;	
	
	Follower_Tab[0] = 0x55;
	Follower_Tab[1] = 0x02;
	
	Send_UpMotor(0 ,0);

	while(1)
	{
		KEY_Check();									//�������
		
		Can_WifiRx_Check();
		Can_ZigBeeRx_Check();
		
		while(Auto_Move_Flag){
				//run();
		}
		
		if(ETC_Open_Flag == 1)					
		{
			if(Stop_Flag == 0x06){ //������բ���
				Auto_Move_Flag = 1;
			}
		}
		
		#if 0
		if(gt_get_sub(LED_twinkle_times) == 0) 			
		{
			LED_twinkle_times =  gt_get() + 50;			//LED4״̬ȡ��
			LED4 = !LED4;
		} 
		#endif
		
		if(gt_get_sub(Power_check_times) == 0) 			
		{
			Power_check_times =  gt_get() + 200;		//��ص������
			Power_Check();		
		} 
		
		#if 1
		if(gt_get_sub(RFID_Init_Check_times) == 0) 			
		{
			RFID_Init_Check_times =  gt_get() + 200;	//RFID��ʼ�����
			if(Rc522_GetLinkFlag() == 0)					
			{
				Readcard_daivce_Init();
				//MP_SPK = !MP_SPK;
			} else {
				MP_SPK = 0;
				LED4 = !LED4;
				Rc522_LinkTest();
			}
		} 
		#endif

		if(gt_get_sub(WIFI_Upload_data_times) == 0)
		{
			WIFI_Upload_data_times =  gt_get() + 200;
		
			if(Host_AGV_Return_Flag == RESET)
			{
				Principal_Tab[2] = Stop_Flag;				//����״̬
				Principal_Tab[3] = Get_tba_phsis_value();	//����״ֵ̬����
				
				Ultrasonic_Ranging();						//����������						
				Principal_Tab[4]=dis%256;        
				Principal_Tab[5]=dis/256;
				
				Light_Value = Get_Bh_Value();				//��ǿ�ȴ�����	
				Principal_Tab[6]=Light_Value%256;	    	//��������
				Principal_Tab[7]=Light_Value/256;
				
				CodedDisk_Value = CanHost_Mp;				//����
				Principal_Tab[8]=CodedDisk_Value%256;	    	
				Principal_Tab[9]=CodedDisk_Value/256;
				
				Nav_Value = CanHost_Navig;					//�Ƕ�
				Principal_Tab[10]=Nav_Value%256;	    	
				Principal_Tab[11]=Nav_Value/256;
				
				Send_WifiData_To_Fifo(Principal_Tab,12);
				UartA72_TxClear();
				UartA72_TxAddStr(Principal_Tab,12);
				UartA72_TxStart();
			} else if((Host_AGV_Return_Flag == SET) && (AGV_data_Falg == SET)){
				
				UartA72_TxClear();
				UartA72_TxAddStr(Follower_Tab,50);
				UartA72_TxStart();
				Send_WifiData_To_Fifo(Follower_Tab,50);
				AGV_data_Falg = 0;
			}
		}
	}		
}

