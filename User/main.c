#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "debug_uart.h"
#include "bsp_i2c.h"
#include "bmi088.h"
#include "delay.h"

#define CONFIG_DEBUG_MODE
#define TASK_NUM 		3

typedef struct {
	uint32_t period;		// ms
	uint32_t now_time;	// ms
	void (*task_func)(void);
} task_t;


void Scheduler_Run(task_t *, size_t task_num);		//
void Scheduler_Tick(task_t *, size_t task_num);

static uint32_t time_stamp = 0;		// ms
static uint8_t time_update = 0;

static BMI088_RawData raw;
static float acc_g[3], gyro_dps[3];

// 上电校准
#define CALI_SAMPLES 50
#define CALI_FREQ			100 // ms
static BMI088_RawData calibra[CALI_SAMPLES];
static BMI088_RawData bias;

void BMI088_Cali(void);
void BMI088_Rowpro(BMI088_RawData*);


void task_2hz(void);
void task_10hz(void);
void task_100hz(void);

task_t tasks[TASK_NUM] = {
	{500,0,task_2hz},
	{100,0,task_10hz},
	{10,0,task_100hz}
};

void task_2hz(void)
{
	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13))
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}else
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}
}
void task_10hz(void)
{
	BMI088_ReadAll(&raw);
	// BMI088_Rowpro(&raw);
	#ifdef CONFIG_DEBUG_MODE
	printf("RAW:%6d,%6d,%6d,%6d,%6d,%6d\r\n",
		raw.acc_x, raw.acc_y, raw.acc_z,
		raw.gyro_x, raw.gyro_y, raw.gyro_z);
	#endif
}
void task_100hz(void)
{}

int main(void)
{
	Delay_ms(50);
	Timer2_Init();
	LED_Init();
	BSP_I2C_Init();
	Debug_UART_Init(115200);
	
	if(!BMI088_Init())
	{
		#ifdef CONFIG_DEBUG_MODE
		printf("BMI088 Init Erorr!!");
		while(1);
		#endif
		
	}
	
	// BMI088_Cali();
	
	#ifdef CONFIG_DEBUG_MODE
	printf("BMI088 Init 0K!!");
	#endif	
	
	TIM_Cmd(TIM2, ENABLE); // 开启定时器
	while (1)
	{
		
		Scheduler_Run(tasks,TASK_NUM);
		
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == 1)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		Scheduler_Tick(tasks,TASK_NUM);
	}
}

void BMI088_Cali(void)
{
	int i = 0;
	BMI088_RawData sum;
	TIM_Cmd(TIM2, DISABLE); // 开启定时器
	
	for(i=0;i<CALI_SAMPLES;i++)
	{
		BMI088_ReadAll(&calibra[i]);
		
		if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13))
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		}else
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_13);
		}
		Delay_ms(CALI_FREQ);

	}
	
	
	for(i=0;i<CALI_SAMPLES;i++)
	{
		sum.acc_x += calibra[i].acc_x;
		sum.acc_y += calibra[i].acc_y;
		sum.acc_z += calibra[i].acc_z;
		
		sum.gyro_x += calibra[i].gyro_x;
		sum.gyro_y += calibra[i].gyro_y;
		sum.gyro_z += calibra[i].gyro_z;
	}
	
	bias.acc_x = sum.acc_x / CALI_SAMPLES;
	bias.acc_y = sum.acc_y / CALI_SAMPLES;
	bias.acc_z = sum.acc_z / CALI_SAMPLES;

	bias.gyro_x = sum.gyro_x / CALI_SAMPLES;
	bias.gyro_y = sum.gyro_y / CALI_SAMPLES;
	bias.gyro_z = sum.gyro_z / CALI_SAMPLES;
	
	TIM_Cmd(TIM2, ENABLE); // 开启定时器
}

void BMI088_Rowpro(BMI088_RawData* raw)
{
	raw->acc_x -= bias.acc_x;
	raw->acc_y -= bias.acc_y;
	raw->acc_z -= bias.acc_z;
	
	raw->gyro_x -= bias.gyro_x;
	raw->gyro_y -= bias.gyro_y;
	raw->gyro_z -= bias.gyro_z;
	
}

void Scheduler_Run(task_t *tasks, size_t task_num)
{
	size_t i;
	if(!task_num)
	{
		return;
	}
	
	for(i=0;i<task_num;i++)
	{
		if(tasks[i].now_time > tasks[i].period)
		{
			tasks[i].now_time = 0;
			tasks[i].task_func();	// 运行
		}
	}
	
}
void Scheduler_Tick(task_t *tasks, size_t task_num)	// 
{
	size_t i;
	if(!task_num)
	{
		return;
	}
	
	for(i=0;i<task_num;i++)
	{
		tasks[i].now_time++;
	}
}