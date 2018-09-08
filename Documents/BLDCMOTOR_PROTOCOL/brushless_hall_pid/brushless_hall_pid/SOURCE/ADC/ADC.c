/*******************************************************************************
* �ļ����� : ʵ��ADC���ú�ʹ�� 
* ���÷��� : ���������е���	DMA_Config();  ADC_Config(); ���á�
* ʹ�÷��� ����Ҫ�õ��ļ��м��� ADC.h ��ֱ�Ӷ�ȡ ADC_ConvertedValue ��Ϊת�������
*   ע��   ��DMA_Config(); Ҫ���� ADC_Config(); ǰ�棬����ת������λ�û���ҡ�
*   ����   �� taoyeah 
*   ʱ��   ��2011��7��14��
*******************************************************************************/
#include "stm32f10x_lib.h"
#define ADC1_DR_Address    ((u32)0x4001244C)
vu16 ADC_ConvertedValue[8];
/*******************************************************************************
* Function Name  : ADC_Configuration1
* Description    : Configures the ADC1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	
    GPIO_InitTypeDef GPIO_InitStructure;

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	/* Configure PA (ADC Channel11) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
								  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   // ��������ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // ɨ�跽ʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   // ����ת��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // �ⲿ������ֹ
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // �����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 8;              // ����ת����ͨ����
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* ADC1 regular channel14 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_71Cycles5);
    
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
    
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
     
    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
}
/***********************************************************************************/
/*******************************************************************************
* Function Name  : DMA_Configuration1
* Description    : Configures the DMA1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	  /* DMA channel1 configuration ----------------------------------------------*/
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;      // �����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue; // �ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;               // DMA ���䷽����
    DMA_InitStructure.DMA_BufferSize = 8;                            // ����DMA�ڴ���ʱ�������ĳ��� word
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // ����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;         // ����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // ���������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          // �ڴ������ֳ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;     // ����DMA�Ĵ���ģʽ���������ϵ�ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; // ����DMA�����ȼ���
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;        // ����DMA��2��memory�еı����������
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    /* Enable DMA channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}
