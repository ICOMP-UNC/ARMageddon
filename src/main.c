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

#define EINT0  	(1<<0)

#define ENA_PIN (1 << 1)  // P2.1 para ENA (PWM1.2)

#define	time_manual			10
#define	time_automatico 	100
#define LED_PIN ((uint32_t)(1 << 22))
#define OUTPUT (uint8_t)	 1
#define INPUT (uint8_t) 	 0

#define DMA_SIZE 60
#define NUM_SINE_SAMPLE 60
#define SINE_FREQ_IN_HZ 50
#define PCLK_DAC_IN_MHZ 25

void configPin(void);
void configADC(void);
void configTimer0();
void configTimer1();
void confUart(void);
void ROTATE_COUNTERCLOCKWISE();
void ROTATE_CLOCKWISE();
void STOP_MOTOR();
void ROTATE_UP();
void ROTATE_DOWN();
void STOP_MOTOR_V();
void configEXT();
void motor_forward();
void motor_reverse();
void motor_stop();
void UART0_IRQHandler(void);
void UART_IntReceive(void);
void setupPWM(void);
void setMotorSpeed(uint8_t dutyCycle);
void config_SYSTICK();
void confDMA(void);
void confDac(void);

GPDMA_Channel_CFG_Type GPDMACfg;

uint32_t dac_sine_lut[NUM_SINE_SAMPLE];

__IO uint32_t ldr0_value;
__IO uint32_t ldr1_value;
__IO uint32_t ldr2_value;
__IO uint32_t ldr4_value;


uint8_t len = 0;
uint8_t info[1] = ""; // Tamaño 1 para recibir un solo byte

uint8_t flag_exti=0;
uint8_t motor;

void delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms * 10000; i++) {
        __NOP(); // No Operation: solo genera una espera
    }
}

uint8_t salto1[] = "\n";
uint8_t salto2[] = "\r";

int main(void){
	uint32_t i;
	uint32_t sin_0_to_90_16_samples[16]={0,1045,2079,3090,4067, 5000,5877,6691,7431,8090,8660,9135,9510,9781,9945,10000};

	configPin();
	configEXT();
	setupPWM();
	setMotorSpeed(60);
	config_SYSTICK();
	confDac();
	//Prepare DAC sine look up table
		for(i=0;i<NUM_SINE_SAMPLE;i++){
			if(i<=15){
				dac_sine_lut[i] = 512 + 512*sin_0_to_90_16_samples[i]/10000;
				if(i==15)
					dac_sine_lut[i]= 1023;}
			else if(i<=30){
				dac_sine_lut[i] = 512 + 512*sin_0_to_90_16_samples[30-i]/10000;}
			else if(i<=45){
				dac_sine_lut[i] = 512 - 512*sin_0_to_90_16_samples[i-30]/10000;}
			else{
				dac_sine_lut[i] = 512 - 512*sin_0_to_90_16_samples[60-i]/10000;}
		dac_sine_lut[i] = (dac_sine_lut[i]<<6);}
	confDMA();

	GPDMA_ChannelCmd(0, ENABLE);
	configADC();
	configTimer0();
	configTimer1();
	confUart();

//	motor_stop();     // Motor detenido

	while(1){

		if(flag_exti){
			config_SYSTICK();

			while(1) {
				        len = 0;
				        while(len == 0)
				            len = UART_Receive(LPC_UART0, info, sizeof(info), NONE_BLOCKING);

				        if(info[0] == 'w') {
				        	// Mueve el motor hacia arriba
				        	motor = 1;
				            motor_forward();
				            delay_ms(500);   // Espera 0.5 segundos
				            motor_stop();
				            info[0] = 'x';  // Respuesta con 'x'
				            UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
				        } else if(info[0] == 's') {
				            //Mueve el motor hacia abajo
				        	motor_reverse();
				        	delay_ms(500);   // Espera 0.5 segundos
				        	motor_stop();
				        	info[0] = 'l';  // Respuesta con 'l'
				        	UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
				        } else if(info[0] == 'a') {
				        	//Mueve el motor a la izquierda
				        	motor_reverse();
				        	delay_ms(500);   // Espera 0.5 segundos
				        	motor_stop();
				        	info[0] = 'z';  // Respuesta con 'z'
				        	UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
				        } else if(info[0] == 'd') {
				        	//Mueve el motor a la derecha
				        	motor = 0;
				        	motor_forward();
				        	delay_ms(500);   // Espera 0.5 segundos
				        	motor_stop();
				        	info[0] = 'y';  // Respuesta con 'y'
				        	UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
				        }else {
				        	motor_stop();     // Detiene el motor

				        	if(info[0] == 13) { // Si es Enter (ASCII 13)
				        		UART_Send(LPC_UART0, salto1, sizeof(salto1), BLOCKING);
				        		UART_Send(LPC_UART0, salto2, sizeof(salto2), BLOCKING);
				        	} else {
				        		UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
				        	}
				        }
				    }
		}
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

	PINSEL_CFG_Type led_pin_cfg; /* Create a variable to store the configuration of the pin */


	led_pin_cfg.Portnum = PINSEL_PORT_0;
	led_pin_cfg.Pinnum = PINSEL_PIN_22;
	led_pin_cfg.Funcnum = PINSEL_FUNC_0;
	led_pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;
    led_pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL;


    PINSEL_ConfigPin(&led_pin_cfg);


	 GPIO_SetDir(PINSEL_PORT_0, LED_PIN, OUTPUT);
	 PINSEL_CFG_Type PinCfg;
	//conf AOUT DAC
	 PinCfg.Funcnum = 2;
	 PinCfg.OpenDrain = 0;
	 PinCfg.Pinmode = 0;
	 PinCfg.Pinnum = 26;
	 PinCfg.Portnum = 0;
	 PINSEL_ConfigPin(&PinCfg);
}

void configEXT(){
	//configuro el p2.10 como EXT0
		PINSEL_CFG_Type pinsel0;
		pinsel0.OpenDrain = PINSEL_PINMODE_NORMAL;
		pinsel0.Pinmode = PINSEL_PINMODE_TRISTATE;
		pinsel0.Funcnum = PINSEL_FUNC_1;
		pinsel0.Portnum = PINSEL_PORT_2;
		pinsel0.Pinnum = PINSEL_PIN_10;
		PINSEL_ConfigPin(&pinsel0);

		LPC_SC->EXTMODE	 |= EINT0;
		LPC_SC->EXTPOLAR &= ~EINT0;
		LPC_SC->EXTINT |= EINT0;

		NVIC_EnableIRQ(EINT0_IRQn);
	//	NVIC_SetPriority(EINT0_IRQn, 2);

}

void EINT0_IRQHandler(void) {

	NVIC_DisableIRQ(ADC_IRQn);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, RESET);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, RESET);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, RESET);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN4, RESET);

	STOP_MOTOR();
	STOP_MOTOR_V();

	 flag_exti = 1;

	LPC_SC->EXTINT |= EINT0;
}

void confUart(void) {

	PINSEL_CFG_Type PinCfg;

	 // Configuración de pines para UART en P0.2 y P0.3
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 2;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 3;
    PINSEL_ConfigPin(&PinCfg);


    UART_CFG_Type UartCfgStruct;
    UART_FIFO_CFG_Type UartFifoCfgStruct;

    UartCfgStruct.Baud_rate = 9600; // Cambiado a 9600 baudios
    UartCfgStruct.Databits = UART_DATABIT_8;
    UartCfgStruct.Parity = UART_PARITY_NONE;
    UartCfgStruct.Stopbits = UART_STOPBIT_1;

    UART_Init(LPC_UART0, &UartCfgStruct); // Inicializa el periférico UART
    UART_FIFOConfigStructInit(&UartFifoCfgStruct); // Inicializa FIFO
    UART_FIFOConfig(LPC_UART0, &UartFifoCfgStruct);
    UART_TxCmd(LPC_UART0, ENABLE);
 //   NVIC_SetPriority(LPC_UART0, 1);

    return;
}

void UART0_IRQHandler(void) {
    uint32_t intsrc, tmp, tmp1;

    intsrc = UART_GetIntId(LPC_UART0); // Determina la fuente de interrupción
    tmp = intsrc & UART_IIR_INTID_MASK; // Evalúa si Transmit Holding está vacío

    if(tmp == UART_IIR_INTID_RLS) { // Evalúa line Status
        tmp1 = UART_GetLineStatus(LPC_UART0);
        tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
        if(tmp1)
            while(1){}; // Error de línea
    }
    if((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
        UART_IntReceive();

    return;
}

void UART_IntReceive(void) {
    UART_Receive(LPC_UART0, info, sizeof(info), NONE_BLOCKING);
    return;
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
	//	NVIC_SetPriority(ADC_IRQn, 5);
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

void motor_forward() {
	if(motor){	//Motor base - horizontal
		LPC_GPIO0->FIOSET = (1 << IN0_PIN);  // IN1 en HIGH
		LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN2 en LOW
	} else {	//Motor brazo - vertical
		LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN1 en HIGH
		LPC_GPIO0->FIOCLR = (1 << IN4_PIN);  // IN2 en LOW
	}

}

void motor_reverse() {
	if(motor){
		LPC_GPIO0->FIOCLR = (1 << IN0_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN2 en HIGH
	} else {
		LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOSET = (1 << IN4_PIN);  // IN2 en HIGH
	}
}

void motor_stop() {
	if(motor){
		LPC_GPIO0->FIOCLR = (1 << IN0_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN2 en LOW
	} else {
		LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOCLR = (1 << IN4_PIN);  // IN2 en LOW
	}

}

// Configuración de PWM para ENA
void setupPWM(void) {
    LPC_SC->PCONP |= (1 << 6);     // Habilitar el periférico PWM1
    LPC_SC->PCLKSEL0 |= (1 << 12); // Seleccionar el clock para PWM1

    LPC_PINCON->PINSEL4 |= (1 << 2); // Configurar P2.1 como PWM1.2

    LPC_PWM1->MR0 = 10000;         // Ciclo total del PWM (1000 -> 1kHz)
    LPC_PWM1->MR2 = 0;            // Ciclo de trabajo inicial al 0%
    LPC_PWM1->LER |= (1 << 0) | (1 << 2); // Actualizar MR0 y MR2

    LPC_PWM1->MCR = (1 << 1);     // Reset del contador en MR0
    LPC_PWM1->PCR |= (1 << 10);   // Habilitar PWM1.2
    LPC_PWM1->TCR = (1 << 0) | (1 << 3); // Habilitar el contador y el modo PWM
}

// Configuración de la velocidad del motor mediante PWM
void setMotorSpeed(uint8_t dutyCycle) {
     // Limitar el duty cycle al 100%
    LPC_PWM1->MR2 = (LPC_PWM1->MR0 * dutyCycle) / 100; // Calcular nuevo MR2
    LPC_PWM1->LER |= (1 << 2); // Actualizar MR2
}

void config_SYSTICK(){

	if((flag_exti=1)){

		SYSTICK_InternalInit(time_manual);

	}

	else{
		SYSTICK_InternalInit(time_automatico);
	}
		SYSTICK_Cmd(ENABLE);
		SYSTICK_IntCmd(ENABLE);
}

void SysTick_Handler(void)
{
    SYSTICK_ClearCounterFlag();

    if (GPIO_ReadValue(PINSEL_PORT_0) & LED_PIN)
    {
        GPIO_ClearValue(PINSEL_PORT_0, LED_PIN);
    }
    else
    {
        GPIO_SetValue(PINSEL_PORT_0, LED_PIN);
    }
}

void confDMA(void){
	GPDMA_LLI_Type DMA_LLI_Struct;

	DMA_LLI_Struct.SrcAddr= (uint32_t)dac_sine_lut;
	DMA_LLI_Struct.DstAddr= (uint32_t)&(LPC_DAC->DACR);
	DMA_LLI_Struct.NextLLI= (uint32_t)&DMA_LLI_Struct;
	DMA_LLI_Struct.Control= DMA_SIZE
												| (2<<18) //source width 32 bit
												| (2<<21) //dest. width 32 bit
												| (1<<26); //source increment;

	/* GPDMA block section -------------------------------------------- */
	/* Initialize GPDMA controller */
	GPDMA_Init();
	// Setup GPDMA channel --------------------------------
	// channel 0
	GPDMACfg.ChannelNum = 0;
	// Source memory
	GPDMACfg.SrcMemAddr = (uint32_t)(dac_sine_lut);
	// Destination memory - unused
	GPDMACfg.DstMemAddr = 0;
	// Transfer size
	GPDMACfg.TransferSize = DMA_SIZE;
	// Transfer width - unused
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	// Source connection - unused
	GPDMACfg.SrcConn = 0;
	// Destination connection
	GPDMACfg.DstConn = GPDMA_CONN_DAC;
	// Linker List Item - unused
	GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct;
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);
	return;}

void confDac(void){
	uint32_t tmp;
	DAC_CONVERTER_CFG_Type DAC_ConverterConfigStruct;
	DAC_ConverterConfigStruct.CNT_ENA = SET;
	DAC_ConverterConfigStruct.DMA_ENA = SET;

	DAC_Init(LPC_DAC);
	/* set time out for DAC*/
	tmp = (PCLK_DAC_IN_MHZ*1000000)/(SINE_FREQ_IN_HZ*NUM_SINE_SAMPLE);
	DAC_SetDMATimeOut(LPC_DAC,tmp);
	DAC_ConfigDAConverterControl(LPC_DAC, &DAC_ConverterConfigStruct);
	return;}
