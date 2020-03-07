# Car_Code  ![](https://img.shields.io/badge/license-GPL3.0-green) ![](https://img.shields.io/badge/download-65M-yellow) ![](https://img.shields.io/badge/Singlechip-arm%2Fstm32-blue) ![](https://img.shields.io/badge/compiler-Keil%20uVision5-orange)



参加嵌入式比赛时写的控制竞赛平台小车代码，仅仅是Keil的代码

实现代码在USER/Car_Move.c中

代码道路函数部分定义如下所示(USER/Car_Move.h):

```c
#ifndef _CAR_MOVE_H
#define _CAR_MOVE_H

extern u8 RFID_Task_Flag;
extern u8 RFID_Go_Flag;
extern u8 UnTrack_Task_Flag1;
extern u8 UnTrack_Task_Flag2;
extern u8 UnTrack_Task_Flag3;
extern u32 ETC_Open_Flag;
extern u32 Auto_Move_Flag;
extern u8 art;
//extern int make;
                                         
										/*道路*/
/**************************************************************************************/


/*前进函数*/
void Car_Go(int spend,uint16_t temp);
	
/*后退函数*/
void Car_Back(int spend,uint16_t temp);

/*循迹函数*/
void Car_Track(int spend);

/*左转函数*/
void Car_L(int spend);

/*右转函数*/
void Car_R(int spend);

/*左旋转函数*/
void Car_Left_Nav(uint16_t time);

/*右旋转函数*/
void Car_Right_Nav(uint16_t time);

                                         /*任务*/
/***************************************************************************************/

/******************************************************
 * 函数名：	Gate  向道闸发送开关指令函数
 * 参  数： swch  SET==开，RESET==关
 * 返回值：	无
******************************************************/
void Gate(uint8_t swch);

/******************************************************
 * 函数名：	Gate_plate  向道闸发送6位车牌数据开启道闸函数
 * 参  数： plate  		车牌文本数组
 * 返回值： 无
******************************************************/
void Gate_plate(u8* plate);

/******************************************************
 * 函数名： Gate_Return  道闸状态回传函数
 * 参  数： 无  		
 * 返回值： 无
******************************************************/
void Gate_Return(void);

/******************************************************
 * 函数名： Lamp  竞赛平台转向灯函数
 * 参  数： L=1=开，L=0=关 
		    R=1=开，R=0=关
 * 返回值： 无
******************************************************/
void Lamp(int L,int R);

/******************************************************
 * 函数名： LED_WriteData  数据写入到数码管第一行或第二行函数
 * 参  数： swch=1=第一行，swch=2=第二行 
		    sj 要写入的数据
 * 返回值： 无
******************************************************/
void LED_WriteData(int swch,u8* sj);

/******************************************************
 * 函数名： LED_Time  数码管计时函数
 * 参  数： swch=0；关闭
			swch=1；打开
			swch=2；清零
 * 返回值： 无
******************************************************/
void LED_Time(int swch);

/******************************************************
 * 函数名： LED_Dis  数码二排显示距离函数
 * 参  数： p 距离值
 * 返回值： 无
******************************************************/
void LED_Dis(uint16_t p);


/******************************************************
 * 函数名： Buzzer  蜂鸣器开启与关闭函数
 * 参  数： p 距离值
 * 返回值： 无
******************************************************/
void Buzzer(uint8_t swch);

/******************************************************
 * 函数名： Modify_Light  智能路灯档位调节函数
 * 参  数： End_Goal 目标档位
 * 返回值： 无
******************************************************/
void Modify_Light(u8 End_Goal);

/******************************************************
 * 函数名： Stereo_Show  立体显示车牌和坐标函数
 * 参  数： plate:车牌数据
			xy	 :坐标数据
 * 返回值： 无
******************************************************/
void Stereo_Show(u8* plate,u8* xy);

/******************************************************
 * 函数名： Ranging  超声波测距函数
 * 参  数： plate:车牌数据
			xy	 :坐标数据
 * 返回值： 无
******************************************************/
void Ranging(void);

/******************************************************
 * 函数名： TFT_Plate  TFT显示车牌函数
 * 参  数： p:车牌数据	
 * 返回值： 无
******************************************************/
void TFT_Plate(u8* p);

/******************************************************
 * 函数名： TFT_Time  TFT计时函数
 * 参  数： i:关闭=0，打开=1，清零=2
 * 返回值： 无
******************************************************/
void TFT_Time(int i);

/******************************************************
 * 函数名： TFT_HEX  TFT-HEX显示函数(16进制显示)
 * 参  数： p 数据
 * 返回值： 无
******************************************************/
void TFT_HEX(uint16_t* p);

/******************************************************
 * 函数名： TFT_Dis  TFT距离显示函数
 * 参  数： p 距离
 * 返回值： 无
******************************************************/
void TFT_Dis(uint8_t* p);

/******************************************************
 * 函数名： Find_Card  寻找RFID卡片函数
 * 参  数： 无
 * 返回值： 无
******************************************************/
void FindCard(void);

/******************************************************
 * 函数名： ReadCard  读取RFID卡片函数
 * 参  数： 无
 * 返回值： 无
******************************************************/
int ReadCard(void);

/******************************************************
 * 函数名： Reverse_Track  反循迹函数
 * 参  数：  i=1   第一种反循迹
			i=2	  第二种反循迹
			i=3	  第三种反循迹
 * 返回值： 无
******************************************************/
void Reverse_Track(int i);

/******************************************************
 * 函数名： QR_Code  接收二维码函数
 * 参  数： swch==1  发送二维码识别 ，swch==2 接收二维码数据
 * 返回值： 无
******************************************************/
void QR_Code(uint8_t swch);

/******************************************************
 * 函数名： Plate_Code  接收车牌函数
 * 参  数： swch==1  发送车牌识别 ，swch==2 接收识别数据
 * 返回值： 无
******************************************************/
void Plate_Code(uint8_t swch)

/******************************************************
 * 函数名： Alertor  报警器报警函数
 * 参  数： swch==1  开启报警 ，swch==2 关闭报警
 * 返回值： 无
******************************************************/
void Alertor(uint8_t swch);

/******************************************************
 * 函数名： Alertor_Code  修改报警器6位报警码函数
 * 参  数： p  6位报警码
 * 返回值： 无
******************************************************/
void Alertor_Code(u8* p);

/******************************************************
 * 函数名： Charge  开启无线充电站函数
 * 参  数： 无
 * 返回值： 无
******************************************************/
void Charge(void);

/******************************************************
 * 函数名： Garage  设置立体车库层数函数
 * 参  数： floor 层数
 * 返回值： 无
******************************************************/
void Garage(u8 floor);

/******************************************************
 * 函数名：	Voice_Broadast  语音播报指定文本信息
 * 参  数： *p  指向文本数据首地址
 * 返回值：	无
******************************************************/
void Voice_Broadast(char *p);


#endif/*_CAR_MOVE_H*/
```



