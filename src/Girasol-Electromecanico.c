*
 * ELECTRONICA DIGITAL III 2024
 * Proyecto Final
 * @file Girasol-Electronico.c
 * @brief sistema embebido para el control de un dispositivo con dos grados de libertad rotacional,
 * equipado con cuatro sensores de luz en su punta, que permite posicionarse automáticamente para maximizar la
 * captación de luminosidad. El sistema también incluye un modo de operación manual, controlado por UART, que
 * permite el movimiento direccional mediante comandos de teclado. Este sistema utilizará varios periféricos
 * (UART, GPIO, SysTick, Timer, DMA, ADC y PWM) para realizar el procesamiento y control necesarios.
 * @version 1.0
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif


#include <cr_section_macros.h>

#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"

void configUART(void);


#define PORT_0 (uint8_t) 0
#define PORT_1 (uint8_t) 1

#define OUTPUT (uint8_t)	 1
#define INPUT (uint8_t) 	 0

#define	time_manual			10 
#define	time_automatico 	100 


// Pines para IN1 e IN2
#define IN1_PIN 9    // Conectar IN1 del L298N al pin P0.9
#define IN2_PIN 8    // Conectar IN2 del L298N al pin P0.8
#define IN3_PIN 7    // Conectar IN3 del L298N al pin P0.7
#define IN4_PIN 6    // Conectar IN4 del L298N al pin P0.6

/* Pin Definitions */
#define LED_PIN ((uint32_t)(1 << 22)) /* P0.22 connected to LED */

uint8_t		flag_manual=0;
uint8_t 	state = 0;
uint16_t 	ldr_0_val = 0;
uint16_t 	ldr_1_val = 0;



void config_pin_v();
void config_adc_v();
void init_ldr_v();
void ROTATE_COUNTERCLOCKWISE();
void ROTATE_CLOCKWISE();
void STOP_MOTOR();

void config_pins();
void config_SYSTICK();
void config_DAC();
void config_DMA();

#define DMA_SIZE 60
#define NUM_SINE_SAMPLE 60
#define NUM_TRIANGLE_SAMPLE 60
#define SINE_FREQ_IN_HZ 50
#define PCLK_DAC_IN_MHZ 25 //CCLK divided by 4


uint32_t dac_sine_lut[NUM_SINE_SAMPLE];
uint32_t triangle_wave_lut[NUM_TRIANGLE_SAMPLE];

int main(void) {

	init_ldr_v();
	config_SYSTICK();
	config_DAC();
	
	// defino variables auxiliares
	uint32_t i;   //contador

		
	uint32_t sin_0_to_90_16_samples[16]={  //cuarto de onda senoidal
				0,1045,2079,3090,4067,\
				5000,5877,6691,7431,8090,\
				8660,9135,9510,9781,9945,10000\
		};
	
		
		while(1){
			
		}


	return 0 ;
}

void init_ldr_v(){

	config_pin_v();
	config_adc_v();

}

void config_pin_v(){

	/conf los p0.23 y p024 como AD0 y AD1/
	PINSEL_CFG_Type pinsel0;
	pinsel0.OpenDrain = PINSEL_PINMODE_NORMAL;
	pinsel0.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinsel0.Funcnum = PINSEL_FUNC_1;
	pinsel0.Portnum = PINSEL_PORT_0;
	pinsel0.Pinnum = PINSEL_PIN_23;
	PINSEL_ConfigPin(&pinsel0);
	pinsel0.Pinnum = PINSEL_PIN_24;
	PINSEL_ConfigPin(&pinsel0);

}

void config_adc_v(){
	ADC_Init(LPC_ADC, 200000);

	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE);

	ADC_BurstCmd(LPC_ADC, 1);
	ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);

	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, SET);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, SET);

	NVIC_EnableIRQ(ADC_IRQn);
}


void ADC_IRQHandler(){

	if(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, 1) && ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_1, 1)){
		ldr_0_val = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
		ldr_1_val = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_1);


			if(ldr_0_val > ldr_1_val){
				ROTATE_COUNTERCLOCKWISE();

			}else if(ldr_1_val > ldr_0_val){
				ROTATE_CLOCKWISE();
			}


		else STOP_MOTOR();
	}

	return;


}

void ROTATE_COUNTERCLOCKWISE(){
	LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN1 en HIGH
	LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN2 en LOW
	return;
}

void ROTATE_CLOCKWISE(){
	LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN1 en LOW
	LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN2 en HIGH
	return;
}
void STOP_MOTOR(){
	LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN1 en LOW
	LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN2 en LOW
	return;
}

void config_pins(){
		PINSEL_CFG_Type led_pin_cfg; /* Create a variable to store the configuration of the pin */

	    /* We need to configure the struct with the desired configuration */
	    led_pin_cfg.Portnum = PINSEL_PORT_0;           /* The port number is 0 */
	    led_pin_cfg.Pinnum = PINSEL_PIN_22;            /* The pin number is 22 */
	    led_pin_cfg.Funcnum = PINSEL_FUNC_0;           /* The function number is 0 */
	    led_pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;   /* The pin mode is pull-up */
	    led_pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL; /* The pin is in the normal mode */

	    /* Configure the pin */
	    PINSEL_ConfigPin(&led_pin_cfg);

	    /* Set the pins as input or output */
	    GPIO_SetDir(PINSEL_PORT_0, LED_PIN, OUTPUT); /* Set the P0.22 pin as output */
}

void config_SYSTICK(){
	
	if(flag_manual==1){
		
		SYSTICK_InternalInit(time_manual);
		
	}
	
	else{
		SYSTICK_InternalInit(time_automatico);
	}
		SYSTICK_Cmd(Enable);
		SYSTICK_IntCmd(Enable);
}

void SysTick_Handler(void)
{
    SYSTICK_ClearCounterFlag(); /* Clear interrupt flag */

    if (GPIO_ReadValue(PINSEL_PORT_0) & LED_PIN)
    {
        GPIO_ClearValue(PINSEL_PORT_0, LED_PIN); /* Turn off LED */
    }
    else
    {
        GPIO_SetValue(PINSEL_PORT_0, LED_PIN); /* Turn on LED */
    }
}

void config_DMA(){
	
	 GPDMA_Init();
	 
	 GPDMA_Channel_CFG_Type		DMA;
	 DMA.ChannelNum = 0;
	 DMA.SrcMemAddr = 0;                        /* Source is peripheral (ADC) */
	 DMA.DstMemAddr = (uint32_t)adc_dma_buffer; /* Destination is memory buffer */
	 DMA.TransferSize = DMA_BUFFER_SIZE;        /* Number of transfers */
	 DMA.TransferWidth = 0;                     /* Width is not used for ADC */
	 DMA.TransferType = GPDMA_TRANSFERTYPE_P2M; /* Peripheral to memory */
	 DMA.SrcConn = GPDMA_CONN_ADC;              /* ADC is the source */
	 DMA.DstConn = 0;                           /* Memory as destination */
	 DMA.DMALLI = 0;                            /* No linked list */

	     GPDMA_Setup(&dma_config); /* Setup the DMA transfer */
	
}

void confDac(void){
	
	PINSEL_CFG_Type PinCfg;
		/*
		 * Init DAC pin connect
		 * AOUT on P0.26
		 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 26;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
		
	uint32_t tmp;

	DAC_CONVERTER_CFG_Type DAC_ConverterConfigStruct;
	DAC_ConverterConfigStruct.CNT_ENA = SET; //habilitamos el contador asocioado al dac
	DAC_ConverterConfigStruct.DMA_ENA = SET; //asociamos el dma al dac

	//inicializamos el DAC con un 0 en el BIAS osea un max de frecuencia de conversion de 1MHz
	DAC_Init(LPC_DAC);

	tmp = (PCLK_DAC_IN_MHZ*1000000)/(SINE_FREQ_IN_HZ*NUM_SINE_SAMPLE);

	DAC_SetDMATimeOut(LPC_DAC, tmp);
	//conf todo lo asociado al contador y al dma al dac
	DAC_ConfigDAConverterControl(LPC_DAC, &DAC_ConverterConfigStruct);
	
	if(flag_manual=1){
		//prepare DAC sine look up table
			for(i=0; i<NUM_SINE_SAMPLE;i++){
				if(i<=15){
					dac_sine_lut[i] = 512 + 512*sin_0_to_90_16_samples[i]/10000;
					if(i==15) dac_sine_lut[i] = 1023;
				}
				else if(i<=30)
				{
					dac_sine_lut[i] =  512 + 512*sin_0_to_90_16_samples[30-i]/10000;
				}
				else if(i<=45)
				{
					dac_sine_lut[i] = 512 - 512*sin_0_to_90_16_samples[i-30]/10000;
				}
				else
				{
					dac_sine_lut[i] = 512 - 512*sin_0_to_90_16_samples[60-i]/10000;
				}
				dac_sine_lut[i] =  (dac_sine_lut[i]<<6);
			}
		confDMA();
		//Enable GPDMA channel 0
		GPDMA_ChannelCmd(0, ENABLE);
	}
	
	else{
		//señal triangular
		// Generar la tabla de onda triangular para DAC
		for (i = 0; i < NUM_TRIANGLE_SAMPLE; i++) {
		    if (i < 15) {
		        // Primer cuarto: incremento lineal desde 512 hasta el pico (1023)
		        triangle_wave_lut[i] = 512 + (i * 34);  // 34 es el paso para alcanzar 1023 en 15 muestras
		    }
		    else if (i < 30) {
		        // Segundo cuarto: decremento lineal desde el pico hasta el centro (512)
		        triangle_wave_lut[i] = 1023 - ((i - 15) * 34);
		    }
		    else if (i < 45) {
		        // Tercer cuarto: decremento lineal desde el centro hasta el mínimo (0)
		        triangle_wave_lut[i] = 512 - ((i - 30) * 34);
		    }
		    else {
		        // Cuarto cuarto: incremento lineal desde el mínimo hasta el centro (512)
		        triangle_wave_lut[i] = (i - 45) * 34;
		    }

		    // Ajuste de desplazamiento para el DAC si es necesario (por ejemplo, <<6)
		    triangle_wave_lut[i] = (triangle_wave_lut[i] << 6);
		}
		
		confDMA();
		//Enable GPDMA channel 0
		GPDMA_ChannelCmd(0, ENABLE);
		
	}
	
	return;
}