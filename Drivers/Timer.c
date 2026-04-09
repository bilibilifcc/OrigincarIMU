#include "stm32f10x.h"                  // Device header

void Timer2_Init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_InternalClockConfig(TIM2);	// 使用内部时钟
	
	TIM_TimeBaseInitTypeDef Tim_TimeBaseInitStructure;
	Tim_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			// 分频器一般为0
	Tim_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 计数方式，一般向上
	Tim_TimeBaseInitStructure.TIM_Period = 1000 - 1;								// 计数周期，计数到这个值时触发中断
	Tim_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;							// 预分频器			
	Tim_TimeBaseInitStructure.TIM_RepetitionCounter = 0;						// 高级定时器用，一般用写0即可
	TIM_TimeBaseInit(TIM2,&Tim_TimeBaseInitStructure);							// 初始化TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);												// 配置中断
	
	NVIC_InitTypeDef NVIC_InitTypedefStructure;
	NVIC_InitTypedefStructure.NVIC_IRQChannel = TIM2_IRQn;					// TIM2中断通道
	NVIC_InitTypedefStructure.NVIC_IRQChannelCmd = ENABLE;								// 开启中断
	NVIC_InitTypedefStructure.NVIC_IRQChannelPreemptionPriority = 2;	// 设置主优先级
	NVIC_InitTypedefStructure.NVIC_IRQChannelSubPriority = 1;				// 设置副优先级
	NVIC_Init(&NVIC_InitTypedefStructure);
	
}


void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
}