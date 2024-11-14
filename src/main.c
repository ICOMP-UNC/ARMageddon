/*
* Prueba de control con uart
*/
#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

// Pines para IN1 e IN2
#define IN1_PIN 9    // Conectar IN1 del L298N al pin P0.9
#define IN2_PIN 8    // Conectar IN2 del L298N al pin P0.8
#define IN3_PIN 7    // Conectar IN3 del L298N al pin P0.7
#define IN4_PIN 6    // Conectar IN4 del L298N al pin P0.6

uint8_t motor;

void delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms * 10000; i++) {
        __NOP(); // No Operation: solo genera una espera
    }
}

void motor_forward() {
	if(motor){	//Motor base - horizontal
		LPC_GPIO0->FIOSET = (1 << IN1_PIN);  // IN1 en HIGH
		LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN2 en LOW
	} else {	//Motor brazo - vertical
		LPC_GPIO0->FIOSET = (1 << IN3_PIN);  // IN1 en HIGH
		LPC_GPIO0->FIOCLR = (1 << IN4_PIN);  // IN2 en LOW
	}

}

void motor_reverse() {
	if(motor){
		LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOSET = (1 << IN2_PIN);  // IN2 en HIGH
	} else {
		LPC_GPIO0->FIOCLR = (1 << IN3_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOSET = (1 << IN4_PIN);  // IN2 en HIGH
	}
}

void motor_stop() {
	if(motor){
		LPC_GPIO0->FIOCLR = (1 << IN1_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOCLR = (1 << IN2_PIN);  // IN2 en LOW
	} else {
		LPC_GPIO0->FIOCLR = (1 << IN3_PIN);  // IN1 en LOW
		LPC_GPIO0->FIOCLR = (1 << IN4_PIN);  // IN2 en LOW
	}

}


void confPin(void);
void confUart(void);
void UART0_IRQHandler(void);
void UART_IntReceive(void);

uint8_t len = 0;
uint8_t info[1] = ""; // Tamaño 1 para recibir un solo byte

int main(void) {
    confPin();
    confUart();
    uint8_t salto1[] = "\n";
    uint8_t salto2[] = "\r";
    motor_stop();     // Motor detenido

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
    return 0;
}

void confPin(void) {
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

    // Configuración de los pines P0.9, P0.8, P0.7 y P0.6 como salida
    LPC_GPIO0->FIODIR |= (1 << IN1_PIN) | (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN);

    return;
}

void confUart(void) {
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

