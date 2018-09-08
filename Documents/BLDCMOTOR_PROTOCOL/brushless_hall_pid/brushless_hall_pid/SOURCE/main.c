/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "main.h"
#include "CAN\can.h"

#define ADC1_DR_Address    ((u32)0x4001244C)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static vu32 TimingDelay = 0;

volatile struct {
		unsigned Key 	  :		1;
		unsigned CalSpeed : 	1;
		unsigned Sec      :		1;
		unsigned Fault 	  :		1;
		}Flags;

unsigned int DesiredSpeed=500;
unsigned int ActualSpeed;
unsigned int pwm=500;
unsigned int T3Count;
unsigned int ActualSpeed5[3];
vu16 ADC_DMABUF;
unsigned int AveActualSpeed;
unsigned char AveNum;
unsigned char j;


float kp=0.50,ki=0.08,kd=0.0;
//float ABC[3]={0.8,0.08,0.0};
int ek=0,ek1=0,ek2=0;
float duk;
int du;
int ekSpeed=0;
unsigned char LED_Code[16]={0x5f,0x44,0x9d,0xd5,0xc6,0xd3,0xdb,0x45,0xdf,0xd7,0xcf,0xda,0x1b,0xdc,0x9b,0x8b};
int LED_Dis;

u16 motor_statue=0;
extern u16 My_PWM;
extern bool Direction; 
extern CanRxMsg tmp_CanRxMessage;
extern CanTxMsg tmp_TxMessage;
int state,state1,state2,state3,counter1,counter2,counter3,speed_1,aim_speed,check_run,speed_code;
short ADC_ConvertedValue[5]={0,0,0,0,0};

u8 *Row0[10]={"Set Speed:"};
u8 *Row1[10]={"Now Speed:"};
u8 *Clear[16]={"                "};

TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure; 
EXTI_InitTypeDef EXTI_InitStructure;
ErrorStatus HSEStartUpStatus;  

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void CalculateDC(int u,int y);
void TIM3_Configuration1(void);
void TIM2_Configuration1(void);
void TIM4_Configuration1(void);
void SysTick_Configuration(void);
void DMA_Configuration1(void);
void ADC_Configuration1(void);
int pid(int nonce,int aim);
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
 {
	#ifdef DEBUG
	debug();
	#endif
  	int i,j,k;
	/* System Clocks Configuration */
	RCC_Configuration();		// 初始化时钟
     
	/* NVIC configuration */
	NVIC_Configuration();		// 初始化中断
	CAN_Config();				// 初始化CAN
    DMA_Configuration1();		// DMA初始化
	ADC_Configuration1();		// ADC初始化
	
	TIM3_Configuration1();	  	// 定时器3初始化， 占空比，PWM
	TIM2_Configuration1();		// 定时器2初始化
	TIM4_Configuration1();		// 定时器4初始化

	
	
	GPIO_Configuration();  		// GPIO初始化
	SysTick_Configuration();	// 系统定时器初始化
	SysTick_CounterCmd(SysTick_Counter_Enable);   //使能系统定时器
	/* Configure the systick */    
	//SysTick_Config(); 

	/* Configure the GPIO ports */
	

	
		
	GPIO_SetBits(GPIOC, GPIO_Pin_11);		// 管脚 C11设置 1，使能 IR2136芯片
   	//DisplayNumber(5,0,325);	 
	aim_speed = 2000;					// 希望电机最终以2000速度转
										// PID调速例子:
										// 第一回合: speed = 0, aim_speed = 2000, 偏差值 = aim_speed - speed = 2000 - 0 = 2000
										// 第二回合: speed = 5, aim_speed = 2000, 偏差值 = aim_speed - speed = 2000 - 5 = 1995
										// ...
										// 第N回合: speed = 2000, aim_speed = 2000, 偏差值 = aim_speed - speed = 2000 - 2000 = 0
	//My_PWM=100;
		//EXTI_GenerateSWInterrupt(EXTI_Line1); 
		//for(i=0;i<500;i++);
		//EXTI_GenerateSWInterrupt(EXTI_Line1);	
		//for(i=0;i<500;i++);
		//EXTI_GenerateSWInterrupt(EXTI_Line1);
	while (1)
	{
		// 代码看不看得懂，代码含义能不能懂
		//motor_statue=0;
		//Direction=FALSE; 
		LED_Dis = speed_1;		// 把一个速度初始化到变量 LED_Dis
		if (speed_code != ((GPIO_ReadInputData(GPIOE)>>1)&0x07))	// speed_code调速编码 1,2,3,4,5
		{
			/* speed_code编码通过CAN总线发出去 */
			speed_code=(GPIO_ReadInputData(GPIOE)>>1)&0x07;
		 	tmp_TxMessage.StdId = 0x0f;
		  	tmp_TxMessage.RTR = CAN_RTR_DATA;
		  	tmp_TxMessage.IDE = CAN_ID_STD;
		  	tmp_TxMessage.DLC = 8;
  			tmp_TxMessage.Data[0] = speed_code;
			CAN_Send_Message(&tmp_TxMessage);

			//aim_speed=speed_code*1000;
		}

	  if(state1 > 5)
	  {	
		if(check_run<3&&aim_speed>300)
		{	GPIO_ResetBits(GPIOC, GPIO_Pin_11);
		for(i=0;i<5000;i++);
		My_PWM = 200;
		GPIO_SetBits(GPIOC, GPIO_Pin_11);
			EXTI_GenerateSWInterrupt(EXTI_Line0); 
			for(i=0;i<500;i++);
			EXTI_GenerateSWInterrupt(EXTI_Line1);	
			for(i=0;i<500;i++);
			EXTI_GenerateSWInterrupt(EXTI_Line2);
		}
		state1=0;
		check_run=0;
	}		

	for(i=0;i<100000;i++);			// 延迟一段时间

	// aim_speed = 2000
	// speed_1 = 0, 5, 10, 15, 20, 25, ...., 1995, 2000
	// 用PID控制算法输出无刷电机下一刻的速度，赋值给My_PWM变量
	// pid算法本身不能控制电机，换句话说，有其他的函数来控制电机转
	My_PWM += pid(speed_1, aim_speed) / ((speed_1/My_PWM)+1);

	// My_PWM有可能为负数，强行设置电机速度为0
	if(My_PWM < 0)
		My_PWM = 0;

	// My_PWM上限不能大于5000，强行设置电机速度为5000
	if(My_PWM > 5000)
	My_PWM = 5000;
	}
}

int pid(int nonce,int aim)
{
	static int ek_2=0;
	static int ek_1=0;
	static int ek=0;
//	int ec;	
	int uk; 
	
	ek=aim-nonce;
//	ec=ek/T;
	uk=kp*(ek-ek_1+ki*ek+kd*(ek-2*ek_1+ek_2));
	ek_2=ek_1;
	ek_1=ek;
	return (uk);
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    // RCC system reset(for debug purpose)
    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);

    // Wait till HSE is ready
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus == SUCCESS)
    {
        // Enable Prefetch Buffer
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        // Flash 2 wait state
        FLASH_SetLatency(FLASH_Latency_2);

        // HCLK = SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);

        // PCLK1 = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);

        // PLLCLK = 8MHz * 9 = 72 MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

        // Enable PLL
        RCC_PLLCmd(ENABLE);

        // Wait till PLL is ready
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        // Select PLL as system clock source
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // Wait till PLL is used as system clock source
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    {
        // If HSE fails to start-up, the application will have wrong clock configuration.
        // User can add here some code to deal with this error

        // Go to infinite loop
        while (1)
        {
        }
    }

#ifdef _GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#endif

#ifdef _GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif

#ifdef _GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
#endif

#ifdef _GPIOD
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
#endif

#ifdef _GPIOE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
#endif

#ifdef _GPIOF
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
#endif

#ifdef _GPIOG
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
#endif

#ifdef _AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef _CAN
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);
#endif

#ifdef _USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif

#ifdef _SPI1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
#endif

#ifdef _FSMC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
#endif

#ifdef _USART2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif

#ifdef _PWR
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
#endif

#ifdef _BKP
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
#endif

#ifdef _TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
#endif

#ifdef _DMA1
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

#ifdef _ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
#endif

}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	/*PE4，PE5，PE6为LED*/
  	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	
	 /*PD8-PD15 为数码管段选*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
	 							  
	/*PB12-PB15 为数码管位选*/
  	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	/*PE2,PE3为输入上拉 按键 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	 
  	/* 配置Hall接口IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/*霍尔信号线中断配置*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource2);
 
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	

	 /*PC4,PC5,PC6 为上半桥臂 PC11为驱动使能*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /*PC7,PC8,PC9 为下半桥臂，复用为PWM输出*/
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);
	  //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

}
 
//闭环计算子程序
void CalculateDC(int u,int y)
{
	ek=u-y;
	if(ek>1||ek<-1)
	{
		duk=kp*(ek-ek1)+ki*ek+kd*(ek+ek2-ek1*2);
		du=(int)duk;
		if(duk>1)duk=1;
		if(duk<-1)duk=-1;
		if(du>10)du=10;
		if(du<-5)du=-5;	
		pwm+=du;    
		if(pwm<60)
		{
			pwm=60;		
		}
		if(pwm>0x7FE)
		{
			//pwm=2398;
			pwm=0x7FE;	
		}
		//TIM1->CCR1 = pwm;
		//TIM1->CCR2 = pwm;
		//TIM1->CCR3 = pwm;
		ek2=ek1;
		ek1=ek;
	}
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configure the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  
	#ifdef  VECT_TAB_RAM  
		/* Set the Vector Table base location at 0x20000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
	/* Enable the EXTI9_5 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	  

	
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	  	
		   
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	  		

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	  	

	//NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQChannel;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);
	
	/* Configure and enable ADC interrupt */
	//NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);

	//Step3.使能TIM1的输出比较匹配中断
    //NVIC_InitStructure.NVIC_IRQChannel = TIM3_CC_IRQChannel;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*******************************************************************************
* Function Name  : SysTick_Config
* Description    : Configure a SysTick Base time to 10 ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Configuration(void)
{
    /* Select AHB clock(HCLK) as SysTick clock source */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    
    /* Set SysTick Priority to 3 */
    NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 5, 0);
    
    /* SysTick interrupt each 1ms with HCLK equal to 72MHz */
    SysTick_SetReload(900000);
    
    /* Enable the SysTick Interrupt */
    SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length (time base 10 ms).
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(u32 nCount)
{
  TimingDelay = nCount;

  /* Enable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Enable);
  
  while(TimingDelay != 0)
  {
  }

  /* Disable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Disable);

  /* Clear the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}

/*******************************************************************************
* Function Name  : Decrement_TimingDelay
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void Decrement_TimingDelay(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
