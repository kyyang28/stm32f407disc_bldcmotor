/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
#include "main.h"

extern CanRxMsg tmp_CanRxMessage;
extern CanTxMsg tmp_TxMessage;
 extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 extern TIM_OCInitTypeDef  TIM_OCInitStructure;
extern volatile struct {
		unsigned Key 	  :		1;
		unsigned CalSpeed : 	1;
		unsigned Sec      :		1;
		unsigned Fault 	  :		1;
		}Flags;
extern unsigned int T3Count;
extern unsigned int pwm;
u16 AD_value;
u16 Count;
u16 aaa;
bool LED_15;
bool Direction;
u16 Hall;
extern u16 motor_statue;
u16 My_PWM=100;
extern int state,state1,state2,state3,counter1,counter2,counter3,speed_1,aim_speed,check_run;
extern unsigned char LED_Code[16];
extern int LED_Dis;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void delay(void)
{
	int i;
	for(i=0;i<100;i++);	
}
/*******************************************************************************
* Function Name  : NMIException
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMIException(void)
{
}

/*******************************************************************************
* Function Name  : HardFaultException
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFaultException(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManageException
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManageException(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFaultException
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFaultException(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFaultException
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFaultException(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : DebugMonitor
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMonitor(void)
{
}

/*******************************************************************************
* Function Name  : SVCHandler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVCHandler(void)
{
}

/*******************************************************************************
* Function Name  : PendSVC
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSVC(void)
{
}

/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{ 
	int i;
	speed_1=counter1*25;
	state1++;
	state2++;
	counter1=0;
	if((state2%5)==1)
	GPIO_WriteBit(GPIOE, GPIO_Pin_4, (!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_4))<<4);
	//check_run=0;
}

/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : PVD_IRQHandler
* Description    : This function handles PVD interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PVD_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TAMPER_IRQHandler
* Description    : This function handles Tamper interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TAMPER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FLASH_IRQHandler
* Description    : This function handles Flash interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RCC_IRQHandler
* Description    : This function handles RCC interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Hall_SW(void)
{
	u32 i,j;
	motor_statue=1;
	switch(Hall)
	{
		case 1:
		/* Next step: Step 2 Configuration ---------------------------- */
		/* Channel3 configuration */
    	
	  	   //GPIO_SetBits(GPIOC, GPIO_Pin_9);	   //C=1  
		   TIM3->CCR4=0;
		   delay();
		   GPIO_ResetBits(GPIOC, GPIO_Pin_6); 
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_4);    //A=-1
		   delay();
		   //GPIO_ResetBits(GPIOC, GPIO_Pin_7);	
  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR2 = My_PWM;					  // 37.5%占空比
  
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_5);  //B=0
		   //GPIO_SetBits(GPIOC, GPIO_Pin_8);  
  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR3=0;

			break;
		case 4:
		/* Next step: Step 3 Configuration ---------------------------- */
    	/* Channel2 configuration */

	  	   //GPIO_SetBits(GPIOC, GPIO_Pin_8);   //B=1	 
  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR3=0;
		   delay();
		   GPIO_ResetBits(GPIOC, GPIO_Pin_5);	
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_6);    //C=-1
		   delay();
		   //GPIO_ResetBits(GPIOC, GPIO_Pin_9); 
  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR4= My_PWM;						  //12.5%占空比

	
		   GPIO_SetBits(GPIOC, GPIO_Pin_4);  //A=0
		   //GPIO_SetBits(GPIOC, GPIO_Pin_7); 
  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR2=0;
		   
			break;
		case 5:
		/* Next step: Step 4 Configuration ---------------------------- */
		/* Channel3 configuration */
		
	  	   //GPIO_SetBits(GPIOC, GPIO_Pin_8);	   //B=1	
  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR3=0;
		   delay();
		   GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_4);    //A=-1
		   delay();
		   //GPIO_ResetBits(GPIOC, GPIO_Pin_7);	   
  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   					  // 37.5%占空比
  TIM3->CCR2=My_PWM;
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_6);  //C=0
		   //GPIO_SetBits(GPIOC, GPIO_Pin_9); 
  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR4=0;
			break;

		case 2:
		/* Next step: Step 5 Configuration ---------------------------- */
		/* Channel3 configuration */
		
	  	   //GPIO_SetBits(GPIOC, GPIO_Pin_7);	   //A=1  
  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR2=0;
		   delay();
		   GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_5);    //B=-1
		   delay();
		   //GPIO_ResetBits(GPIOC, GPIO_Pin_8); 
  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  				   //25%占空比
  TIM3->CCR3=My_PWM;
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_6);  //C=0
		   //GPIO_SetBits(GPIOC, GPIO_Pin_9);  
  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR4=0;
		   
			break;

		case 3:
		/* Next step: Step 6 Configuration ---------------------------- */
		/* Channel3 configuration */
    	
	  	   //GPIO_SetBits(GPIOC, GPIO_Pin_9);		   //C=1 
  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR4=0;
		   delay();
		   GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_5);    //B=-1
		   delay();
		   //GPIO_ResetBits(GPIOC, GPIO_Pin_8);	  
  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  					   //25%占空比
 TIM3->CCR3=My_PWM;
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_4);  //A=0
		   //GPIO_SetBits(GPIOC, GPIO_Pin_7); 
  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR2=0;
			break;

		case 6:
		/* Next step: Step 1 Configuration ---------------------------- */
		/* Channel1 configuration */
	  	   	
		   //GPIO_SetBits(GPIOC, GPIO_Pin_7);	   //A=1  
  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR2=0;
		   delay();
		   GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_6);    //C=-1
		   delay();
		   //GPIO_ResetBits(GPIOC, GPIO_Pin_9);	 
  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM3->CCR4=My_PWM;
	
		   GPIO_SetBits(GPIOC, GPIO_Pin_5);  //B=0
		   //GPIO_SetBits(GPIOC, GPIO_Pin_8);  
  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM3->CCR3=0;
			break;


		default:
		/* Next step: Step 1 Configuration ---------------------------- */
		/* Channel1 configuration */
		break;
	}
}
void EXTI0_IRQHandler(void)
{	  
	Hall=GPIO_ReadInputData(GPIOD);
	Hall=Hall&0x0007;
	//Hall=Hall>>6;
	//DisplayNumber4(0,0,Hall);
	if(!Direction)Hall=7-Hall;
	//Hall=3;
	delay();
	Hall_SW();
	counter1++;
	check_run++;
	if(EXTI_GetITStatus(EXTI_Line0)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	
}

/*******************************************************************************
* Function Name  : EXTI1_IRQHandler
* Description    : This function handles External interrupt Line 1 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI1_IRQHandler(void)
{	   
	Hall=GPIO_ReadInputData(GPIOD);
	Hall=Hall&0x0007;
	//Hall=Hall>>6;
	//DisplayNumber4(0,0,Hall);
	if(!Direction)Hall=7-Hall;
	//Hall=3;
	delay();
	Hall_SW();
	counter1++;
	state3++;	
	if((state3%20)==1)
	GPIO_WriteBit(GPIOE, GPIO_Pin_5, (!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_5))<<5);
	
	if(EXTI_GetITStatus(EXTI_Line1)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

/*******************************************************************************
* Function Name  : EXTI2_IRQHandler
* Description    : This function handles External interrupt Line 2 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{    
	Hall=GPIO_ReadInputData(GPIOD);
	Hall=Hall&0x0007;
	//Hall=Hall>>6;
	//DisplayNumber4(0,0,Hall);
	if(!Direction)Hall=7-Hall;
	//Hall=3;
	delay();
	Hall_SW();
	counter1++;
	if(EXTI_GetITStatus(EXTI_Line2)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

/*******************************************************************************
* Function Name  : EXTI3_IRQHandler
* Description    : This function handles External interrupt Line 3 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    /* Toggle GPIO_LED pin 4 */
	//counter1++;
	state=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4);
	//GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3);
  }
}

/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External interrupt Line 4 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel3_IRQHandler
* Description    : This function handles DMA1 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel4_IRQHandler
* Description    : This function handles DMA1 Channel 4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel5_IRQHandler
* Description    : This function handles DMA1 Channel 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel6_IRQHandler
* Description    : This function handles DMA1 Channel 6 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel 7 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_2_IRQHandler(void)
{
	if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET)
	{
		aaa++;
		AD_value=ADC1->JDR1;//ADC1->DR;
		ADC1->SR = ~(u32)ADC_FLAG_JEOC;
	}
}

/*******************************************************************************
* Function Name  : USB_HP_CAN_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
	CAN_Receive(CAN_FIFO0, &tmp_CanRxMessage);
	aim_speed=tmp_CanRxMessage.Data[0]*1000;
}

/*******************************************************************************
* Function Name  : CAN_RX1_IRQHandler
* Description    : This function handles CAN RX1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_RX1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : CAN_SCE_IRQHandler
* Description    : This function handles CAN SCE interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_SCE_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{	
	int i;
	if(EXTI_GetITStatus(EXTI_Line5)!= RESET)
	{
		aim_speed+=200;
		if(aim_speed>10000)
			aim_speed=10000;
		for(i=0;i<200000;i++);
		EXTI_ClearITPendingBit(EXTI_Line5);
	}		 
	if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
	{
		for(i=0;i<200000;i++);
		aim_speed-=200;	
		if(aim_speed<0)
			aim_speed=0; 
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
}

/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : This function handles TIM1 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_TRG_COM_IRQHandler
* Description    : This function handles TIM1 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
	
	//AD_value = ADC_GetConversionValue(ADC1);
	//DisplayNumber4(0,0,AD_value);
	//TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 );
}

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int LED_dual=0,LED_bit=0;
void TIM2_IRQHandler(void)
{  

	int ReadValue;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		LED_bit++;
		switch(LED_bit)
		{
			case 1:	LED_dual= LED_Dis%10;  
					GPIO_ResetBits(GPIOB, GPIO_Pin_15);					
					GPIO_SetBits(GPIOB, GPIO_Pin_13);
					GPIO_SetBits(GPIOB, GPIO_Pin_14);
					GPIO_SetBits(GPIOB, GPIO_Pin_12);
					ReadValue = (GPIO_ReadOutputData(GPIOD)&0xff); 
					GPIO_Write(GPIOD, (ReadValue | (LED_Code[LED_dual]<<8))); 
					break;					 
			case 2:	LED_dual= (LED_Dis/10)%10;  
					GPIO_ResetBits(GPIOB, GPIO_Pin_14);					
					GPIO_SetBits(GPIOB, GPIO_Pin_12);
					GPIO_SetBits(GPIOB, GPIO_Pin_13);
					GPIO_SetBits(GPIOB, GPIO_Pin_15);
					ReadValue = (GPIO_ReadOutputData(GPIOD)&0xff); 
					GPIO_Write(GPIOD, (ReadValue | (LED_Code[LED_dual]<<8))); 
					break;
			case 3:	LED_dual= (LED_Dis/100)%10;  
					GPIO_ResetBits(GPIOB, GPIO_Pin_13);					
					GPIO_SetBits(GPIOB, GPIO_Pin_14);
					GPIO_SetBits(GPIOB, GPIO_Pin_12);
					GPIO_SetBits(GPIOB, GPIO_Pin_15);
					ReadValue = (GPIO_ReadOutputData(GPIOD)&0xff); 
					GPIO_Write(GPIOD, (ReadValue | (LED_Code[LED_dual]<<8))); 
					break;
			case 4:	LED_dual= (LED_Dis/1000)%10;  
					GPIO_ResetBits(GPIOB, GPIO_Pin_12);					
					GPIO_SetBits(GPIOB, GPIO_Pin_13);
					GPIO_SetBits(GPIOB, GPIO_Pin_14);
					GPIO_SetBits(GPIOB, GPIO_Pin_15);
					ReadValue = (GPIO_ReadOutputData(GPIOD)&0xff); 
					GPIO_Write(GPIOD, (ReadValue | (LED_Code[LED_dual]<<8))); 
					LED_bit=0;
					break;
					
		}
    }
}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{	   
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int Key_Test1=0,Key_Test2=0;
void TIM4_IRQHandler(void)
{	  
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2)==0)
		{
			 Key_Test1++;
			 if(Key_Test1>50)
			 {
			 	Key_Test1=0;
				aim_speed+=200;
				if(aim_speed>10000)
				aim_speed=10000;
			}
			 	
		} 
		
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
			Key_Test1=0;	
		//***********************************************	
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)==0)
		{
			 Key_Test2++;
			 if(Key_Test2>50)
			 {
			 	Key_Test2=0;
				aim_speed-=200;
				if(aim_speed<0)
				aim_speed=0;
			}
			 	
		} 
		
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
			Key_Test2=0;	
	}
}

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI2_IRQHandler
* Description    : This function handles SPI2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{  

}

/*******************************************************************************
* Function Name  : RTCAlarm_IRQHandler
* Description    : This function handles RTC Alarm interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USBWakeUp_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBWakeUp_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_BRK_IRQHandler
* Description    : This function handles TIM8 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_BRK_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_UP_IRQHandler
* Description    : This function handles TIM8 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_UP_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_TRG_COM_IRQHandler
* Description    : This function handles TIM8 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_CC_IRQHandler
* Description    : This function handles TIM8 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_CC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC3_IRQHandler
* Description    : This function handles ADC3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FSMC_IRQHandler
* Description    : This function handles FSMC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM5_IRQHandler
* Description    : This function handles TIM5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI3_IRQHandler
* Description    : This function handles SPI3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART4_IRQHandler
* Description    : This function handles UART4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART5_IRQHandler
* Description    : This function handles UART5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM6_IRQHandler
* Description    : This function handles TIM6 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM6_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : This function handles TIM7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel1_IRQHandler
* Description    : This function handles DMA2 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel2_IRQHandler
* Description    : This function handles DMA2 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel3_IRQHandler
* Description    : This function handles DMA2 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel4_5_IRQHandler
* Description    : This function handles DMA2 Channel 4 and DMA2 Channel 5
*                  interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel4_5_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
