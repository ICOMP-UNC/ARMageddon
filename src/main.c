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

#define LDR0_CHANNEL 0  // Canal ADC0.0 (PIN 0.23)
#define LDR1_CHANNEL 1  // Canal ADC0.1 (PIN 0.24)
#define IN1_PIN 9    // Conectar IN1 del L298N al pin P0.9
#define IN2_PIN 8    // Conectar IN2 del L298N al pin P0.8

void configPin(void);
void configADC(void);
void configTimer0();
void configTimer1();
void ROTATE_COUNTERCLOCKWISE();
void ROTATE_CLOCKWISE();
void STOP_MOTOR();

__IO uint32_t ldr0_value;
__IO uint32_t ldr1_value;

uint8_t flag_counterclockwise=0;
uint8_t flag_clockwise=0;

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

		ADC_Init(LPC_ADC, 200000);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE);
		ADC_BurstCmd(LPC_ADC, 1);
		ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
	//	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
		ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, SET);
		ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, SET);

		NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(){

	TIM_Cmd(LPC_TIM0, DISABLE);
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
	NVIC_DisableIRQ(TIMER0_IRQn);

	NVIC_DisableIRQ(ADC_IRQn);

	if(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)){
		ldr0_value = ADC_ChannelGetData(LPC_ADC, 0);
	}
	if(ADC_ChannelGetStatus(LPC_ADC, 1, ADC_DATA_DONE)){
			ldr1_value = ADC_ChannelGetData(LPC_ADC, 1);
		}

	//preguntamos que canal causo la interrupcion
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


void ROTATE_COUNTERCLOCKWISE(){
	LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN1 en HIGH
	LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN2 en LOW

	//retardo
//	while(!TIM_GetIntStatus(LPC_TIM1,TIM_MR0_INT));
	flag_counterclockwise=1;
	flag_clockwise=0;
	return;
}

void ROTATE_CLOCKWISE(){
	LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN1 en LOW
	LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN2 en HIGH

	//retardo
//	while(!TIM_GetIntStatus(LPC_TIM1,TIM_MR0_INT));
	flag_counterclockwise=0;
	flag_clockwise=1;
	return;
}
void STOP_MOTOR(){
	LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN1 en HIGH
	LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN2 en HIGH
	return;
}
