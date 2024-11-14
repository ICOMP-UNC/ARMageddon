/*
* Prueba de control con uart
*/
#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

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
    // Apagar todos los LEDs
                        GPIO_SetValue(0, (1 << 22)); // Apagar LED rojo
                        GPIO_SetValue(3, (1 << 25)); // Apagar LED verde
                        GPIO_SetValue(3, (1 << 26)); // Apagar LED azul

    while(1) {
        len = 0;
        while(len == 0)
            len = UART_Receive(LPC_UART0, info, sizeof(info), NONE_BLOCKING);

        if(info[0] == 'w') {
                    // Encender LED rojo en P0.22
                    GPIO_ClearValue(0, (1 << 22));
                    // Apagar LEDs verde y azul
                    GPIO_SetValue(3, (1 << 25));
                    GPIO_SetValue(3, (1 << 26));
                    info[0] = 'x';  // Respuesta con 'x'
                    UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
                } else if(info[0] == 'd') { // 'g' para encender el LED verde
                    // Encender LED verde en P0.25
                    GPIO_ClearValue(3, (1 << 25));
                    // Apagar LEDs rojo y azul
                    GPIO_SetValue(0, (1 << 22));
                    GPIO_SetValue(3, (1 << 26));
                    info[0] = 'y';  // Respuesta con 'y'
                    UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
                } else if(info[0] == 'a') { // 'b' para encender el LED azul
                    // Encender LED azul en P0.26
                    GPIO_ClearValue(3, (1 << 26));
                    // Apagar LEDs rojo y verde
                    GPIO_SetValue(0, (1 << 22));
                    GPIO_SetValue(3, (1 << 25));
                    info[0] = 'z';  // Respuesta con 'z'
                    UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
                } else {
                    // Apagar todos los LEDs
                    GPIO_SetValue(0, (1 << 22)); // Apagar LED rojo
                    GPIO_SetValue(3, (1 << 25)); // Apagar LED verde
                    GPIO_SetValue(3, (1 << 26)); // Apagar LED azul

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

    // Configuración de P0.22 como GPIO de salida para el LED rojo
    GPIO_SetDir(0, (1 << 22), 1); // P0.22 como salida
    // Configuración de P0.25 como GPIO de salida para el LED verde
    GPIO_SetDir(3, (1 << 25), 1); // P3.25 como salida
    // Configuración de P0.26 como GPIO de salida para el LED azul
    GPIO_SetDir(3, (1 << 26), 1);

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