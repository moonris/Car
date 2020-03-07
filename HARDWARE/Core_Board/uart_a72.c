#include "stm32f4xx.h"
#include "uart_drv.h"
#include "can_user.h"
#include "data_channel.h"
#include "Timer.h"


#define __UART_A72_C__
#include "uart_a72.h"

//************************************************串口4********************************************//
void Uart_A72_PortInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_TypeDefStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA
							|RCC_AHB1Periph_GPIOD
							|RCC_AHB1Periph_GPIOI, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2
						  	|RCC_APB1Periph_UART4
							,ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
		
	//__________________________________________________________________________	

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOI, &GPIO_InitStructure);

	Hard_Uart_MurtSel(0);										//选择与A72相连的端口
	
	//__________________________________________________________________________	

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_InitStructure.USART_Mode =  USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	USART_Cmd(UART4, ENABLE);	

	NVIC_TypeDefStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_TypeDefStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_TypeDefStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_TypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TypeDefStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_Mode =  USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);
	
	NVIC_TypeDefStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_TypeDefStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_TypeDefStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_TypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TypeDefStructure);
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART2, ENABLE);	
	
}

/*
函数功能：串口2中断服务函数
参    数：无
返 回 值：无
**/
void USART2_IRQHandler(void)
{
/*	if(Wifi_Rx_flag == 0)
	{
		if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
		{
			Wifi_Rx_Buf[Wifi_Rx_num]= USART_ReceiveData(USART2);
			Wifi_Rx_num++;
			if(Wifi_Rx_num > WIFI_MAX_NUM)	
			{
				Wifi_Rx_num = 0;
				Wifi_Rx_flag = 1;
			}
		}
	}
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);*/
	
//	if(USART_GetITStatus(uart_prot_buf[p->uart_port], USART_IT_TXE) != RESET)
//	{
//		p->tx_curr++;
//		if(p->tx_curr < p->tx_leng)
//			uart_prot_buf[p->uart_port]->DR=p->tx_buf[p->tx_curr];
//		else
//			USART_ITConfig(uart_prot_buf[p->uart_port], USART_IT_TXE, DISABLE);
//			
//		USART_ClearITPendingBit(uart_prot_buf[p->uart_port], USART_IT_TXE);		
//	}
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		if(Wifi_Rx_flag == 0)
		{
			canu_wifi_rxtime = gt_get()+10;
			Wifi_Rx_num =0;
			Wifi_Rx_Buf[Wifi_Rx_num]= USART_ReceiveData(USART2);
			Wifi_Rx_flag = 1;
		}
		else if(Wifi_Rx_num < WIFI_MAX_NUM )	
		{
			Wifi_Rx_Buf[++Wifi_Rx_num]= USART_ReceiveData(USART2);	 
		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}

/*
函数功能：串口输出端口软件切换
参    数：c = 0 选择与A72相连的端口输出   c = 1 选择连接蓝牙模块端口输出
返 回 值：无
**/
void Hard_Uart_MurtSel(uint8_t c)
{
	if(c)
		GPIO_SetBits(GPIOI,GPIO_Pin_8);
	else
		GPIO_ResetBits(GPIOI,GPIO_Pin_8);
}


//************************************************串口2********************************************//
#define PUART_A72TX	(&(uart_struct[4]))

uint8_t Uart_A72_TxEmpty;


#define FIFOSIZE_A72_TX	50
uint8_t FifoBuf_A72_Tx[FIFOSIZE_A72_TX];

#define FIFOSIZE_A72_SRX 2
uint8_t FifuBuf_A72_SRx[FIFOSIZE_A72_SRX];


void UartA72_Init(void)
{
	PUART_A72TX->tx_buf = FifoBuf_A72_Tx;
	PUART_A72TX->tx_size = FIFOSIZE_A72_TX;
	PUART_A72TX->tx_leng =  0;
	PUART_A72TX->tx_curr = 0;
	PUART_A72TX->tx_overs = 0;

	PUART_A72TX->rx_buf = FifuBuf_A72_SRx;
	PUART_A72TX->rx_size = FIFOSIZE_A72_SRX;
	PUART_A72TX->rx_leng =  0;
	PUART_A72TX->rx_overs = 0;
	PUART_A72TX->rx_times = 0;
	
	PUART_A72TX->uart_port = 4;
	
	Uart_A72_TxEmpty = 0;
	
	Uart_A72_PortInit();
}

void UartA72_TxClear(void)
{
	UartTx_Clear(PUART_A72TX);
	Uart_A72_TxEmpty = 0;
}

void UartA72_TxAddChar(uint8_t d)
{
	UartTx_AddChar(d,PUART_A72TX);
	Uart_A72_TxEmpty = 1;
}

void UartA72_TxAddStr(uint8_t *p,uint32_t l)
{
	UartTx_AddBuf(p,l,PUART_A72TX);
	if(l)
		Uart_A72_TxEmpty = 1;
}

void UartA72_TxStart(void)
{
	UartTx_Start(PUART_A72TX);
}

void UART4_IRQHandler(void)
{
	Uart_Irq(PUART_A72TX);
}


void UartA72_TxWhileCheck(void)
{
//	if(Uart_A72_TxEmpty)
//	{
//		if(USART_GetFlagStatus(UART4,USART_FLAG_TXE) != RESET)
//		{
//			uint8_t temp;
//			if(FifoDrv_ReadOne(&Fifo_UartA72_Tx,&temp))
//			{
//				USART_SendData(UART4,temp);
//			}
//			else
//				Uart_A72_TxEmpty = 0;
//		}
//	}
}







//end file
