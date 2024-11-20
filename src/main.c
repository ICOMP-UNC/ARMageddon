#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"


#define IN0_PIN 9    // Conectar IN0 del L298N al pin P0.9
#define IN1_PIN 8    // Conectar IN1 del L298N al pin P0.8
#define IN2_PIN 27   // Conectar IN2 del L298N al pin P0.27
#define IN4_PIN 28   // Conectar IN4 del L298N al pin P0.28

void configPin(void);
void configADC(void);
void configTimer0();
void configTimer1();
void ROTATE_COUNTERCLOCKWISE();
void ROTATE_CLOCKWISE();
void STOP_MOTOR();
void ROTATE_UP();
void ROTATE_DOWN();
void STOP_MOTOR_V();

__IO uint32_t ldr0_value;
__IO uint32_t ldr1_value;
__IO uint32_t ldr2_value;
__IO uint32_t ldr4_value;



int main(void){

	configPin();
	configADC();
	configTimer0();
	configTimer1();


	while(1){
	}

	return 0;
}

void configPin(void){
	//configuro los pines P0.8, P0.9 como salidas GPIO
	LPC_PINCON->PINSEL0 &= ~(0xF<<16);
	// Configuración de los pines P0.9, P0.8, P0.7 y P0.6 como salidas
	LPC_GPIO0->FIODIR |= (0b11<<8);

	//configuro los pines P0.27, P0.28 como salidas GPIO
	LPC_PINCON->PINSEL1 &= ~(0xF<<22);
	// Configuración de los pines P0.27, P0.28 como salidas
	LPC_GPIO0->FIODIR |= (0b11<<27);
}

void configADC(void){
	//conf los p0.23 y p024 como AD0 y AD1/
		PINSEL_CFG_Type pinsel0;
		pinsel0.OpenDrain = PINSEL_PINMODE_NORMAL;
		pinsel0.Pinmode = PINSEL_PINMODE_TRISTATE;
		pinsel0.Funcnum = PINSEL_FUNC_1;
		pinsel0.Portnum = PINSEL_PORT_0;
		pinsel0.Pinnum = PINSEL_PIN_23;
		PINSEL_ConfigPin(&pinsel0);
		pinsel0.Pinnum = PINSEL_PIN_24;
		PINSEL_ConfigPin(&pinsel0);
	//conf los p0.25 y p1.30 como AD2 y AD4/
		pinsel0.Pinnum = PINSEL_PIN_25;
		PINSEL_ConfigPin(&pinsel0);
		pinsel0.Portnum = PINSEL_PORT_1;
		pinsel0.Pinnum = PINSEL_PIN_30;
		PINSEL_ConfigPin(&pinsel0);

		ADC_Init(LPC_ADC, 200000);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_2, ENABLE);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_4, ENABLE);
		ADC_BurstCmd(LPC_ADC, 1);
		ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
	//	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
		ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, SET);
		ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, SET);
		ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, SET);
		ADC_IntConfig(LPC_ADC, ADC_ADINTEN4, SET);

		NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(){

	TIM_Cmd(LPC_TIM0, DISABLE);
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
	NVIC_DisableIRQ(TIMER0_IRQn);

	NVIC_DisableIRQ(ADC_IRQn);

	//preguntamos que canal causo la interrupcion
	if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)){
		ldr0_value = ADC_ChannelGetData(LPC_ADC, 0);
	}
	if(ADC_ChannelGetStatus(LPC_ADC, 1, ADC_DATA_DONE)){
		ldr1_value = ADC_ChannelGetData(LPC_ADC, 1);
	}
	if(ADC_ChannelGetStatus(LPC_ADC, 2, ADC_DATA_DONE)){
		ldr2_value = ADC_ChannelGetData(LPC_ADC, 2);
	}
	if(ADC_ChannelGetStatus(LPC_ADC, 4, ADC_DATA_DONE)){
		ldr4_value = ADC_ChannelGetData(LPC_ADC, 4);
	}

	if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE) && ADC_ChannelGetStatus(LPC_ADC, 1, ADC_DATA_DONE)){

			ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, RESET);
			ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, RESET);

			if(ldr0_value > ldr1_value){
				ROTATE_COUNTERCLOCKWISE();
			}
			else if (ldr0_value < ldr1_value){
				ROTATE_CLOCKWISE();
			}

			else {
				STOP_MOTOR();
			}

		}
	if(ADC_ChannelGetStatus(LPC_ADC, 2, ADC_DATA_DONE) && ADC_ChannelGetStatus(LPC_ADC, 4, ADC_DATA_DONE)){

			ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, RESET);
			ADC_IntConfig(LPC_ADC, ADC_ADINTEN4, RESET);

			if(ldr2_value > ldr4_value){
				ROTATE_UP();
			}
			else if (ldr2_value < ldr4_value){
				ROTATE_DOWN();
			}

			else {
				STOP_MOTOR_V();
			}

		}

	NVIC_EnableIRQ(TIMER0_IRQn);
	TIM_Cmd(LPC_TIM0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
}


void configTimer0(){
	TIM_TIMERCFG_Type     struct_config;
	TIM_MATCHCFG_Type     struct_match;

	struct_config.PrescaleOption   =  TIM_PRESCALE_USVAL;
	struct_config.PrescaleValue    = 1000;  //en microsegundos

	struct_match.MatchChannel        = 1;
	struct_match.IntOnMatch          = ENABLE;
	struct_match.ResetOnMatch        = ENABLE;
	struct_match.StopOnMatch         = DISABLE;
	struct_match.ExtMatchOutputType  = TIM_EXTMATCH_NOTHING;
	struct_match.MatchValue          = 10000;   // 100mseg se va a generar un match

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &struct_config);
	TIM_ConfigMatch(LPC_TIM0, &struct_match);

	TIM_Cmd(LPC_TIM0, ENABLE);


	NVIC_EnableIRQ(TIMER0_IRQn);
	return;
}


//voy a usar el timer1 como retardante/
void configTimer1(){
	TIM_TIMERCFG_Type     struct_config;
	TIM_MATCHCFG_Type     struct_match;

	struct_config.PrescaleOption   =  TIM_PRESCALE_USVAL;
	struct_config.PrescaleValue    = 1000;  //en microsegundos

	struct_match.MatchChannel        = 0;
	struct_match.IntOnMatch          = ENABLE;
	struct_match.ResetOnMatch        = ENABLE;
	struct_match.StopOnMatch         = DISABLE;
	struct_match.ExtMatchOutputType  = TIM_EXTMATCH_NOTHING;
	struct_match.MatchValue          = 10000;   // 100mseg se va a generar un match

	TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &struct_config);
	TIM_ConfigMatch(LPC_TIM1, &struct_match);

	TIM_Cmd(LPC_TIM1, ENABLE);
	NVIC_EnableIRQ(TIMER1_IRQn);
	return;
}

void TIMER1_IRQHandler(void){

	TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);

	return;
}


void TIMER0_IRQHandler(void){

	TIM_ClearIntPending(LPC_TIM0,TIM_MR0_INT);
//	ADC_StartCmd(LPC_ADC, ADC_START_NOW );

	return;
}


void ROTATE_UP(){
	LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN2 en HIGH
	LPC_GPIO0->FIOCLR = (1 << IN4_PIN);  // IN4 en LOW

	return;
}

void ROTATE_DOWN(){
	LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN2 en LOW
	LPC_GPIO0->FIOSET = (1 << IN4_PIN);  // IN4 en HIGH

	return;
}
void STOP_MOTOR_V(){
	LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN2 en HIGH
	LPC_GPIO0->FIOSET = (1 << IN4_PIN);  // IN4 en HIGH

	return;
}

void ROTATE_COUNTERCLOCKWISE(){
	LPC_GPIO0->FIOSET = (1 << IN0_PIN);  // IN0 en HIGH
	LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN1 en LOW

	return;
}

void ROTATE_CLOCKWISE(){
	LPC_GPIO0->FIOCLR = (1 << IN0_PIN);  // IN0 en LOW
	LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN1 en HIGH

	return;
}
void STOP_MOTOR(){
	LPC_GPIO0->FIOSET = (1 << IN0_PIN);  // IN0 en HIGH
	LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN1 en HIGH

	return;
}



