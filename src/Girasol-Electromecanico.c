/*
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

void configUART(void);
void configADC(void);
void configGPIO(void);
void configTimer(void);
void configDAC(void);

int main(void) {
	configUART();
	configADC();
	configGPIO();
	configTimer();
	configDAC();

    return 0 ;
}
