/*******************************************************************************
* �ļ����� : ʵ��CANͨѶ�����ú�ʹ�� 
* ���÷��� : ���������е���	CAN_Config(); ���ã�������ΪPA11��12ΪCAN�ӿ�
* ʹ�÷��� ����Ҫ�õ��ļ��м��� can.h ���� stm32f10x_it.c �м����Ӧ
*			 ���жϷ����� CAN_Receive(CAN_FIFO0, &tmp_CanRxMessage);
*			 ���յ����ݱ����� tmp_CanRxMessage �У��� NVIC_Configuration(); ��
*			 ���ú��ն����ȼ�ֱ�ӵ��� CAN_Send_Message(&tmp_TxMessage); 
*			 �� tmp_TxMessage �е����ݷ���ȥ��
*   ע��   ��extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
*			 extern TIM_OCInitTypeDef  TIM_OCInitStructure;
*            ���������ṹ�壬�������ʹ��TIM4����ʹ��TIM3ʱ��Ҫ�� extern ȥ��
*            ͬʱ���� int Change_PWM_Pulse(TIM_TypeDef* TIMx,int chanel,int x)��
*            ���������
*   ����   �� taoyeah 
*   ʱ��   ��2011��7��13��
*******************************************************************************/
#include "stm32f10x_lib.h"
CanRxMsg tmp_CanRxMessage;
CanTxMsg tmp_TxMessage;
/*-----------------------------------------------------------------------------
** ����ԭ�ͣ�void CAN_Configuration(void)
** �������ܣ�Configures the CAN.
** ��ڲ�������
** ���ڲ�������
** ��    ע����
-----------------------------------------------------------------------------*/
void CAN_Config(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

    // Configure CAN pin: RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Configure CAN pin: TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CAN register init
    CAN_DeInit();
    CAN_StructInit(&CAN_InitStructure);

    // CAN cell init
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    //CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
    CAN_InitStructure.CAN_Prescaler = 4;          //������ 72MHz/2=36MHz=PCLK1 / 4 => 9000KHz / (1+10+7) => 500KHz

    CAN_Init(&CAN_InitStructure);

    // CAN filter init
	//��׼֡�� 31~21 λ  ��11λ STID[10:0] 
    //��չ֡�� 20~3 λ   ��29λ EXID[17:0]+STID[10:0] 

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    //CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x00<<5;        // STD_ID<<5	��λҪһһ��Ӧ���ܹ�
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x00;			   //
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x00<<5;    // STD_ID<<5	��ӦλΪ1�Ĳ��ܹ�
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x00;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // CAN FIFO0 message pending interrupt enable
    CAN_ITConfig(CAN_IT_FMP0, ENABLE);
}

void CAN_Send_Message(CanTxMsg *temp_TXMsg)
{
    u32 i = 0;
    u8 TransmitMailbox = 0;
	//while
    TransmitMailbox = CAN_Transmit(temp_TXMsg);
    i = 0;
    while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK)&&i<=0xff )
    {
      i++;
    }
}
