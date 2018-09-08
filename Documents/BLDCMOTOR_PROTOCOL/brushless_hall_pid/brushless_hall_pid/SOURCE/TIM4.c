 #include "stm32f10x_lib.h"
 extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 extern TIM_OCInitTypeDef  TIM_OCInitStructure;
void TIM4_Configuration1(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //  TIM_OCInitTypeDef  TIM_OCInitStructure ;
    TIM_DeInit( TIM4);                              //��λTIM2��ʱ��

    /* TIM2 clock enable [TIM2��ʱ������]*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 2;          //       
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;    // 72M/(35999+1)/2 = 1kHz       
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // ʱ�ӷָ�  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //�����������ϼ���
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* Clear TIM2 update pending flag[���TIM2����жϱ�־] */
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);

    /* Enable TIM2 Update interrupt [TIM2����ж�����]*/
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);  

    /* TIM2 enable counter [����tim2����]*/
    TIM_Cmd(TIM4, ENABLE);
}
