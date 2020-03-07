#ifndef __SYN7318_H
#define __SYN7318_H
#include "stm32f4xx.h"
#include "sys.h"

#define SYN7318RESET PBout(5)

void SYN7318_Init(void);
void SYN_TTS(uint8_t *Pst);
void SYN7318_Test( void);					// ������������
void SYN_7318_One_test(uint8_t mode,uint8_t num);		// mode ģʽ���� 1 ���ָ�� 0 ָ��ָ��
void Yu_Yin_Asr(void);						// ����ʶ������
	
#endif

