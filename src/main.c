/**
 * @file main.c
 * @brief Laboratorio 3 (EDIII) "Gates and Hopes" - Control de puerta y sistema de monitoreo ambiental.
 *
 * Este programa implementa un sistema que controla una puerta y monitorea el ambiente a través de diversos sensores.
 * Se utiliza un ADC para leer datos de temperatura, movimiento y detección de incendios. Además, se controla un motor
 * para abrir y cerrar la puerta según las condiciones ambientales, activando una alarma cuando sea necesario.
 *
 * @version 1.0
 * @date 2024
 * @author [Tu nombre]
 *
 * @details
 * - Entrada de temperatura (LM35), sensor de movimiento (PIR) y detección de gases/incendios (MQ-2).
 * - Control de un motor mediante un DAC y GPIO para abrir/cerrar la puerta.
 * - Alarma activada en condiciones críticas (detección de incendio o temperatura baja).
 * - Control manual de la puerta mediante un botón externo.
 *
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_spi.h"
#include "lpc17xx_uart.h"
#endif

#define TEMP_MIN 124 // 10°C en LM35

#ifdef __USE_MCUEXPRESSO
#include <cr_section_macros.h> /* The cr_section_macros is specific to the MCUXpresso delivered toolchain */
#endif

// Declaración de funciones
/**
 * @brief Activa la alarma.
 *
 * Enciende el LED de alarma conectado al pin P0.22.
 */
void activarAlarma();

/**
 * @brief Cierra la puerta.
 *
 * Activa el motor para cerrar la puerta controlado por el pin P0.21 y espera hasta que la puerta esté completamente cerrada.
 */
void cerrarPuerta();

/**
 * @brief Abre la puerta.
 *
 * Activa el motor para abrir la puerta controlado por el pin P0.21 y utiliza el SysTick para detener el motor tras 5 segundos.
 */
void abrirPuerta();

int main(void) {

	//configGPIO();
	//configIntExt();
	configADC();
	configDAC();
	//configTimer();
	//configUart();
	//configDMA();
	//configSPI();
	NVIC_SetPriority(EINT0_IRQn,1);

	// Inicia la puerta abierta
	if(((LPC_GPIO0->FIOPIN) & (1 << 24)) != 0){
		LPC_GPIO0->FIOSET = (1);
	}

	while(1);

    return 0 ;
}

/* void configGPIO(void){

- P0.26 funcion aout salida (del DAC) a ULN2003 (Motor de ventilador): (Control del ventilador en base a la temperatura)

- P0.21 funcion gpio salida a ULN2003 (Control de motor de la puerta)
- P1.18 funcion gpio salida a LED de estado de la batería
- P0.22 funcion gpio salida a LED de alarma
- P0.17 funcion gpio (IntExt) entrada de (boton) Final de carrera (Detección de puerta cerrada)
	NVICEnable(EINT3_IRQn);

- Display SPI (OLED 128x64):
MOSI: P0.18
SCLK: P0.15
CS: P0.16

	return;
}*/

/**
 * @brief Manejador de interrupción externa para el final de carrera.
 *
 * Detecta cuando la puerta ha llegado al final de su carrera y detiene el motor.
 */
void EINT3_IRQHandler(void){
	if((LPC_GPIOINT->IO0IntStatR) & (1<<17)){			/* LLAVE Representacion de sensor de final de carrera de apertura */
		LPC_GPIO0->FIOCLR = 1;							/* Detiene el motor */
		LPC_GPIOINT-> IO0IntClr |= (1<<24);
	}
	return;
}

/* void configIntExt(void){

- P2.10 funcion EINT0 entrada de Botón (Control manual de la puerta)
	NVICEnable(EINT0_IRQn);
	return;
}*/

/**
 * @brief Manejador de interrupción externa para el botón de control manual.
 *
 * Alterna el estado de la puerta entre abierta y cerrada según la condición actual.
 */
void EINT0_IRQHandler(void){

	//Verifico con el final de carrera en que situacion esta la puerta
	if(((LPC_GPIO0->FIOPIN) & (1 << 17)) != 0){			/* Si esta cerrada */
		abrirPuerta();
	} else if (((LPC_GPIO0->FIOPIN)&(1 << 17)) == 0){ 	/* Si esta abierta */
		cerrarPuerta();
	}

	LPC_SC->EXTINT |= 1; //Limpia la bandera de interrupcion externa.
	return;
}

void activarAlarma(void){
	LPC_GPIO0->FIOSET = (1<<22);
	return;
}

void cerrarPuerta(void){
	LPC_GPIO0->FIOSET = (1<<21);					/* Activa el motor para cerrar */
	while(((LPC_GPIO0->FIOPIN)&(1<<21)) == 0);	    /* espero hasta que el sensor de carrera sea 0(es decir, esta cerrada)*/
	return;
}

void abrirPuerta(void){

	LPC_GPIO0->FIOSET = (1<<21);				/* Activa el motor para abrir */
	SysTick_Config(SystemCoreClock/100); 		/*Configuro el systick para apagar el motor cuando termine de abrir*/
	while(((LPC_GPIO0->FIOPIN)&(1<<21)) == 0);
	SYST_CSR &= ~(1 << 0);						// Desactivar SysTick: Limpia el bit ENABLE (bit 0)
	return;
}

void SysTick_Handler(void){
    static uint8_t clkDiv = 0;						/* Divisor del reloj para realizar operaciones cada 5 ciclos */

	//Si el divisor del reloj alcanza 5(s), apaga el motor de la puerta
    if(clkDiv == 5){
    	LPC_GPIO0->FIOCLR = (1<<21);				/* Apaga el motor para la apertura */
        clkDiv = 0;									/* Reinicia el divisor del reloj */
    } else {
        clkDiv++;									/* Incrementa el divisor del reloj */
    }

    SysTick->CTRL &= SysTick->CTRL; 				/* Limpia la bandera de interrupción */
}

/**
 * @brief Configuración de los canales del ADC.
 *
 * - P0.23: entrada de LM35 (sensor de temperatura)
 * - P0.24: entrada de PIR (sensor de movimiento)
 * - P0.25: entrada de MQ-2 (detección de incendios dentro del refugio)
 * - P0.02: entrada de MQ-2 (detección de gases fuera del refugio)
 */
void configADC(void){

	LPC_PINCON->PINSEL0  |= (1<<14); //P0.2 como ADC0.7
	LPC_PINCON->PINSEL1  |= ((0x2A)<<14); //P0.23 como ADC0.0, P0.24 como ADC0.1,P0.25 como ADC0.2

	LPC_PINCON->PINMODE0 |= (1<<5); //Ni pull up ni pull down en P0.2
	LPC_PINCON->PINMODE1 |= ((0x2A)<<14); //Ni pull up ni pull down en P0.23, P0.24, P0.25 Y el pin 23 del puerto 0.

	LPC_SC->PCONP |= (1 << 12);		 //Se da energia al ADC
	LPC_ADC->ADCR |= (1 << 21);		 //habilita el ADC

	LPC_SC->PCLKSEL0 |= (3<<24); 	 	//CCLK/8
	LPC_ADC->ADCR       &=~(255<<8);	 // CLKDIV=0; No hay division extra

	LPC_ADC->ADCR |= 0x87;   	 	//Se usa el canal 0,1,2,7 para convertir
	LPC_ADC->ADCR |= (1 << 16);  	 // Modo burst activado

	NVIC_EnableIRQ(ADC_IRQn);			 // Habilita interrupcion en NVIC.

	return;
}

/**
 * @brief Manejador de interrupción del ADC.
 *
 * Lee los valores de los sensores y activa la alarma o controla la puerta en consecuencia.
 */
void ADC_IRQHandler(void){
	uint16_t temp;

	//Sensor de temperatura (LM35)
	if(LPC_ADC->ADDR0 & (1<<31)){
		temp = ((LPC_ADC->ADDR0) >> 4) & (0xFFF);
		if(temp < TEMP_MIN){ //10° en el LM35 = (0.1/3.3)2^12(resolucion)
			activarAlarma();
			cerrarPuerta();
		}
	}

	//Sensor de movimiento (PIR)
	if(LPC_ADC->ADDR1 & (1<<31)){
		if((((LPC_ADC->ADDR1) >> 4) & (0xFFF)) > 0){ //apenas detecta mov se activa la alarma y el cierre de puerta
			activarAlarma();
			cerrarPuerta();
		}
	}

	//Sensor de Incendios dentro del refugio (MQ-2)
	if(LPC_ADC->ADDR2 & (1<<31)){
		if((((LPC_ADC->ADDR2) >> 4) & (0xFFF)) > 0){ //apenas detecta humo/flama se activa la alarma y el abre la puerta
			activarAlarma();
			abrirPuerta();
		}
	}

	//Sensor de Gases fuera del refugio (MQ-2)
	if(LPC_ADC->ADDR7 & (1<<31)){
		if((((LPC_ADC->ADDR7) >> 4) & (0xFFF)) > 0){ //apenas detecta que afuera no es bueno el aire se activa la alarma y el cierre de puerta
			activarAlarma();
			cerrarPuerta();
		}
	}
	return;
}

/**
 * @brief Configuración del DAC.
 *
 * Inicializa el DAC para controlar dispositivos como el motor de la puerta y el ventilador.
 */
void configDAC(void){
	DAC_Init(LPC_DAC);
	return;
}
