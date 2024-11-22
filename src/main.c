/**
 * @file main.c
 * @brief Sistema embebido para el control de un dispositivo con dos grados de libertad rotacional, equipado con
 * sensores de luz en su punta, que permite posicionarse automáticamente para maximizar la captación de luminosidad.
 * Incluye modo manual mediante UART.
 * @version 1.0
 * @date 2024
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

/** @defgroup GPIO_PINS Pines GPIO configurados para control del motor */
/**@{*/
#define IN0_PIN 9  /**< Pin P0.9 conectado a IN0 del L298N */
#define IN1_PIN 8  /**< Pin P0.8 conectado a IN1 del L298N */
#define IN2_PIN 27 /**< Pin P0.27 conectado a IN2 del L298N */
#define IN4_PIN 28 /**< Pin P0.28 conectado a IN4 del L298N */
/**@}*/

#define EINT0 (1 << 0) /**< Interrupción externa 0 */

/** @brief Pines y configuraciones PWM */
#define ENA_PIN (1 << 1) /**< P2.1 usado para ENA (PWM1.2) */

/** @brief Tiempos predefinidos */
#define time_manual     10  /**< Tiempo en modo manual */
#define time_automatico 100 /**< Tiempo en modo automático */

/** @brief Pines de LED */
#define LED_PIN ((uint32_t)(1 << 22)) /**< Pin P0.22 conectado al LED */

/** @brief Modos de configuración GPIO */
#define OUTPUT (uint8_t)1 /**< Modo de salida GPIO */
#define INPUT  (uint8_t)0 /**< Modo de entrada GPIO */

/** @brief Configuración para el DAC */
#define DMA_SIZE        60 /**< Tamaño del buffer de DMA */
#define NUM_SINE_SAMPLE 60 /**< Número de muestras de la señal senoidal */
#define SINE_FREQ_IN_HZ 50 /**< Frecuencia de la señal senoidal (Hz) */
#define PCLK_DAC_IN_MHZ 25 /**< Frecuencia del reloj del DAC (MHz) */

/**
 * @brief Configura los pines GPIO necesarios para el control del motor y el DAC.
 * Configura los pines como salidas o entradas según sea necesario.
 */
void configPin(void);

/**
 * @brief Configura el ADC para lectura de los sensores de luz.
 */
void configADC(void);

/**
 * @brief Configura el Timer 0 para funciones de temporización.
 */
void configTimer0(void);

/**
 * @brief Configura el Timer 1 para funciones de temporización.
 */
void configTimer1(void);

/**
 * @brief Configura la UART para la comunicación serial con el usuario.
 */
void confUart(void);

/**
 * @brief Rota el motor en sentido antihorario.
 */
void ROTATE_COUNTERCLOCKWISE(void);

/**
 * @brief Rota el motor en sentido horario.
 */
void ROTATE_CLOCKWISE(void);

/**
 * @brief Detiene el motor horizontal.
 */
void STOP_MOTOR(void);

/**
 * @brief Rota el motor hacia arriba.
 */
void ROTATE_UP(void);

/**
 * @brief Rota el motor hacia abajo.
 */
void ROTATE_DOWN(void);

/**
 * @brief Detiene el motor vertical.
 */
void STOP_MOTOR_V(void);

/**
 * @brief Configura las interrupciones externas.
 */
void configEXT(void);

/**
 * @brief Mueve el motor hacia adelante.
 */
void motor_forward(void);

/**
 * @brief Mueve el motor hacia atrás.
 */
void motor_reverse(void);

/**
 * @brief Detiene el motor.
 */
void motor_stop(void);

/**
 * @brief Manejador de interrupciones para la UART.
 */
void UART0_IRQHandler(void);

/**
 * @brief Configura el manejo de interrupciones de recepción UART.
 */
void UART_IntReceive(void);

/**
 * @brief Configura los parámetros del PWM.
 */
void setupPWM(void);

/**
 * @brief Establece la velocidad del motor mediante el ciclo útil del PWM.
 * @param dutyCycle Ciclo útil del PWM (en porcentaje).
 */
void setMotorSpeed(uint8_t dutyCycle);

/**
 * @brief Configura el temporizador del sistema (SysTick).
 */
void config_SYSTICK(void);

/**
 * @brief Configura el DMA para la transmisión de la señal senoidal.
 */
void confDMA(void);

/**
 * @brief Configura el DAC para generar la señal senoidal.
 */
void confDac(void);

/**
 * @brief Realiza una espera activa de una cantidad de milisegundos.
 * @param ms Tiempo de espera en milisegundos.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Configuración de la estructura del DMA.
 */
GPDMA_Channel_CFG_Type GPDMACfg; /**< Configuración del canal DMA */

/** @brief Look-up table para la señal senoidal generada por el DAC */
uint32_t dac_sine_lut[NUM_SINE_SAMPLE];

/** @brief Valores leídos desde los sensores de luz */
__IO uint32_t ldr0_value; /**< Valor del sensor LDR 0 */
__IO uint32_t ldr1_value; /**< Valor del sensor LDR 1 */
__IO uint32_t ldr2_value; /**< Valor del sensor LDR 2 */
__IO uint32_t ldr4_value; /**< Valor del sensor LDR 4 */

/** @brief Bandera de interrupción externa */
uint8_t flag_exti = 0;

/** @brief Estado actual del motor */
uint8_t motor;

/**
 * @var char salto1[]
 * @brief Cadena que contiene el carácter de nueva línea ('\n').
 */
char salto1[] = "\n";
/**
 * @var char salto2[]
 * @brief Cadena que contiene el carácter de retorno ('\r').
 */
char salto2[] = "\r";

/**
 * @brief Función principal del sistema.
 *
 * Realiza las siguientes tareas:
 * - Configuración inicial de periféricos (pines, PWM, SysTick, ADC, DAC, UART).
 * - Generación de tabla de seno para salida DAC.
 * - Bucle principal para control manual del sistema mediante UART.
 *
 * @return Siempre retorna 0.
 */

int main(void)
{
    uint32_t i;
    uint32_t sin_0_to_90_16_samples[16] = {
        0, 1045, 2079, 3090, 4067, 5000, 5877, 6691, 7431, 8090, 8660, 9135, 9510, 9781, 9945, 10000};

    configPin();
    configEXT();
    setupPWM();
    setMotorSpeed(60);
    config_SYSTICK();
    confDac();
    // Prepare DAC sine look up table
    for (i = 0; i < NUM_SINE_SAMPLE; i++)
    {
        if (i <= 15)
        {
            dac_sine_lut[i] = 512 + 512 * sin_0_to_90_16_samples[i] / 10000;
            if (i == 15)
                dac_sine_lut[i] = 1023;
        }
        else if (i <= 30)
        {
            dac_sine_lut[i] = 512 + 512 * sin_0_to_90_16_samples[30 - i] / 10000;
        }
        else if (i <= 45)
        {
            dac_sine_lut[i] = 512 - 512 * sin_0_to_90_16_samples[i - 30] / 10000;
        }
        else
        {
            dac_sine_lut[i] = 512 - 512 * sin_0_to_90_16_samples[60 - i] / 10000;
        }
        dac_sine_lut[i] = (dac_sine_lut[i] << 6);
    }
    confDMA();
    // Enable GPDMA channel 0
    GPDMA_ChannelCmd(0, ENABLE);
    configADC();
    configTimer0();
    configTimer1();
    confUart();

    //	motor_stop();     // Motor detenido

    while (1)
    {
    }

    return 0;
}

/**
 * @brief Configura los pines necesarios como GPIO y los inicializa.
 *
 * Configura los pines para el control de LEDs, DAC y motores. Incluye configuración de pines
 * específicos en los puertos P0.8, P0.9, P0.27, P0.28 y P0.22.
 */
void configPin(void)
{
    // configuro los pines P0.8, P0.9 como salidas GPIO
    LPC_PINCON->PINSEL0 &= ~(0xF << 16);
    // Configuración de los pines P0.9, P0.8, P0.7 y P0.6 como salidas
    LPC_GPIO0->FIODIR |= (0b11 << 8);

    // configuro los pines P0.27, P0.28 como salidas GPIO
    LPC_PINCON->PINSEL1 &= ~(0xF << 22);
    // Configuración de los pines P0.27, P0.28 como salidas
    LPC_GPIO0->FIODIR |= (0b11 << 27);

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
}

/**
 * @brief Configura las interrupciones externas.
 *
 * Configura las interrupciones externas requeridas para el sistema. Detalles específicos no están implementados.
 */
void configEXT()
{
    // configuro el p2.10 como EXT0
    PINSEL_CFG_Type pinsel0;
    pinsel0.OpenDrain = PINSEL_PINMODE_NORMAL;
    pinsel0.Pinmode = PINSEL_PINMODE_TRISTATE;
    pinsel0.Funcnum = PINSEL_FUNC_1;
    pinsel0.Portnum = PINSEL_PORT_2;
    pinsel0.Pinnum = PINSEL_PIN_10;
    PINSEL_ConfigPin(&pinsel0);

    LPC_SC->EXTMODE |= EINT0;   // EINT0 edge sensitive
    LPC_SC->EXTPOLAR &= ~EINT0; // EINT0 rising edge
    LPC_SC->EXTINT |= EINT0;    // EINT0 clear flag

    NVIC_EnableIRQ(EINT0_IRQn);
    //	NVIC_SetPriority(EINT0_IRQn, 2);
}
/**
 * @brief Manejador de interrupción para EINT0.
 */
void EINT0_IRQHandler(void)
{

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
/**
 * @brief Configura UART0 para comunicación serial.
 */
void confUart(void)
{

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

    UART_Init(LPC_UART0, &UartCfgStruct);          // Inicializa el periférico UART
    UART_FIFOConfigStructInit(&UartFifoCfgStruct); // Inicializa FIFO
    UART_FIFOConfig(LPC_UART0, &UartFifoCfgStruct);
    UART_TxCmd(LPC_UART0, ENABLE);
    //   NVIC_SetPriority(LPC_UART0, 1);

    return;
}

/**
 * @brief Manejador de interrupción para UART0.
 */

void UART0_IRQHandler(void)
{
    uint32_t intsrc, tmp, tmp1;

    intsrc = UART_GetIntId(LPC_UART0);  // Determina la fuente de interrupción
    tmp = intsrc & UART_IIR_INTID_MASK; // Evalúa si Transmit Holding está vacío

    if (tmp == UART_IIR_INTID_RLS)
    { // Evalúa line Status
        tmp1 = UART_GetLineStatus(LPC_UART0);
        tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
        if (tmp1)
            while (1)
            {
            }; // Error de línea
    }
    if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
        UART_IntReceive();
    len = 0;
    while (len == 0) len = UART_Receive(LPC_UART0, info, sizeof(info), NONE_BLOCKING);

    if (info[0] == 'w')
    {
        // Mueve el motor hacia arriba
        motor = 1;
        motor_forward();
        while (!TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT))
            ; // retardo
        motor_stop();
        info[0] = 'x'; // Respuesta con 'x'
        UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
    }
    else if (info[0] == 's')
    {
        // Mueve el motor hacia abajo
        motor_reverse();
        while (!TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT))
            ; // retardo
        motor_stop();
        info[0] = 'l'; // Respuesta con 'l'
        UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
    }
    else if (info[0] == 'a')
    {
        // Mueve el motor a la izquierda
        motor_reverse();
        while (!TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT))
            ; // retardo
        motor_stop();
        info[0] = 'z'; // Respuesta con 'z'
        UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
    }
    else if (info[0] == 'd')
    {
        // Mueve el motor a la derecha
        motor = 0;
        motor_forward();
        while (!TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT))
            ; // retardo
        motor_stop();
        info[0] = 'y'; // Respuesta con 'y'
        UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
    }
    else
    {
        motor_stop(); // Detiene el motor

        if (info[0] == 13)
        { // Si es Enter (ASCII 13)
            UART_Send(LPC_UART0, salto1, sizeof(salto1), BLOCKING);
            UART_Send(LPC_UART0, salto2, sizeof(salto2), BLOCKING);
        }
        else
        {
            UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
        }
    }
    return;
}

/**
 * @brief Recibe datos de UART en modo no bloqueante.
 */
void UART_IntReceive(void)
{
    UART_Receive(LPC_UART0, info, sizeof(info), NONE_BLOCKING);
    return;
}

/**
 * @brief Configura los pines y el periférico ADC.
 */
void configADC(void)
{
    // conf los p0.23 y p024 como AD0 y AD1/
    PINSEL_CFG_Type pinsel0;
    pinsel0.OpenDrain = PINSEL_PINMODE_NORMAL;
    pinsel0.Pinmode = PINSEL_PINMODE_TRISTATE;
    pinsel0.Funcnum = PINSEL_FUNC_1;
    pinsel0.Portnum = PINSEL_PORT_0;
    pinsel0.Pinnum = PINSEL_PIN_23;
    PINSEL_ConfigPin(&pinsel0);
    pinsel0.Pinnum = PINSEL_PIN_24;
    PINSEL_ConfigPin(&pinsel0);
    // conf los p0.25 y p1.30 como AD2 y AD4/
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

/**
 * @brief Manejador de interrupciones del ADC.
 *
 * Procesa las interrupciones generadas por los canales ADC y determina las acciones a realizar
 * con base en los valores de los sensores de luz (LDR).
 *
 * - Llama a funciones específicas para rotar motores en diferentes direcciones.
 * - Evalúa los valores en los canales 0, 1, 2 y 4.
 * - Controla la activación y desactivación de las interrupciones de ADC y TIMER0.
 */
void ADC_IRQHandler()
{

    TIM_Cmd(LPC_TIM0, DISABLE);
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
    NVIC_DisableIRQ(TIMER0_IRQn);

    NVIC_DisableIRQ(ADC_IRQn);

    // preguntamos que canal causo la interrupcion
    if (ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE))
    {
        ldr0_value = ADC_ChannelGetData(LPC_ADC, 0);
    }
    if (ADC_ChannelGetStatus(LPC_ADC, 1, ADC_DATA_DONE))
    {
        ldr1_value = ADC_ChannelGetData(LPC_ADC, 1);
    }
    if (ADC_ChannelGetStatus(LPC_ADC, 2, ADC_DATA_DONE))
    {
        ldr2_value = ADC_ChannelGetData(LPC_ADC, 2);
    }
    if (ADC_ChannelGetStatus(LPC_ADC, 4, ADC_DATA_DONE))
    {
        ldr4_value = ADC_ChannelGetData(LPC_ADC, 4);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE) && ADC_ChannelGetStatus(LPC_ADC, 1, ADC_DATA_DONE))
    {

        ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, RESET);
        ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, RESET);

        if (ldr0_value > ldr1_value)
        {
            ROTATE_COUNTERCLOCKWISE();
        }
        else if (ldr0_value < ldr1_value)
        {
            ROTATE_CLOCKWISE();
        }

        else
        {
            STOP_MOTOR();
        }
    }
    if (ADC_ChannelGetStatus(LPC_ADC, 2, ADC_DATA_DONE) && ADC_ChannelGetStatus(LPC_ADC, 4, ADC_DATA_DONE))
    {

        ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, RESET);
        ADC_IntConfig(LPC_ADC, ADC_ADINTEN4, RESET);

        if (ldr2_value > ldr4_value)
        {
            ROTATE_UP();
        }
        else if (ldr2_value < ldr4_value)
        {
            ROTATE_DOWN();
        }

        else
        {
            STOP_MOTOR_V();
        }
    }

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
    NVIC_EnableIRQ(ADC_IRQn);
}

/**
 * @brief Configura el temporizador 0 para generar interrupciones periódicas.
 *
 * Configura el temporizador con:
 * - Una frecuencia de preescala definida en microsegundos.
 * - Un valor de coincidencia que genera una interrupción cada 100 ms.
 *
 * @note Este temporizador se usa principalmente como referencia temporal para el control.
 */

void configTimer0()
{
    TIM_TIMERCFG_Type struct_config;
    TIM_MATCHCFG_Type struct_match;

    struct_config.PrescaleOption = TIM_PRESCALE_USVAL;
    struct_config.PrescaleValue = 1000; // en microsegundos

    struct_match.MatchChannel = 1;
    struct_match.IntOnMatch = ENABLE;
    struct_match.ResetOnMatch = ENABLE;
    struct_match.StopOnMatch = DISABLE;
    struct_match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    struct_match.MatchValue = 10000; // 100mseg se va a generar un match

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &struct_config);
    TIM_ConfigMatch(LPC_TIM0, &struct_match);

    TIM_Cmd(LPC_TIM0, ENABLE);

    NVIC_EnableIRQ(TIMER0_IRQn);
    return;
}

/**
 * @brief Configura el temporizador 1 como un generador de retardos.
 *
 * Define una frecuencia de preescala y un valor de coincidencia que genera una interrupción
 * cada 100 ms. Se utiliza como base para retardos temporales.
 */
void configTimer1()
{
    TIM_TIMERCFG_Type struct_config;
    TIM_MATCHCFG_Type struct_match;

    struct_config.PrescaleOption = TIM_PRESCALE_USVAL;
    struct_config.PrescaleValue = 1000; // en microsegundos

    struct_match.MatchChannel = 0;
    struct_match.IntOnMatch = ENABLE;
    struct_match.ResetOnMatch = ENABLE;
    struct_match.StopOnMatch = DISABLE;
    struct_match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    struct_match.MatchValue = 10000; // 100mseg se va a generar un match

    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &struct_config);
    TIM_ConfigMatch(LPC_TIM1, &struct_match);

    TIM_Cmd(LPC_TIM1, ENABLE);
    NVIC_EnableIRQ(TIMER1_IRQn);
    return;
}

/**
 * @brief Manejador de interrupciones del temporizador 1.
 *
 * Limpia la interrupción pendiente en el canal correspondiente. Se utiliza principalmente
 * para retardos.
 */
void TIMER1_IRQHandler(void)
{

    TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);

    return;
}

/**
 * @brief Manejador de interrupciones del temporizador 0.
 *
 * Limpia la interrupción pendiente en el canal correspondiente. Puede usarse para iniciar
 * conversiones ADC u otras acciones periódicas.
 */
void TIMER0_IRQHandler(void)
{

    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
    //	ADC_StartCmd(LPC_ADC, ADC_START_NOW );

    return;
}

/**
 * @brief Rota el motor vertical hacia arriba.
 *
 * Activa los pines necesarios para rotar el motor hacia arriba.
 */
void ROTATE_UP()
{
    LPC_GPIO0->FIOSET = (1 << IN2_PIN); // IN2 en HIGH
    LPC_GPIO0->FIOCLR = (1 << IN4_PIN); // IN4 en LOW

    return;
}

/**
 * @brief Rota el motor vertical hacia abajo.
 *
 * Activa los pines necesarios para rotar el motor hacia abajo.
 */
void ROTATE_DOWN()
{
    LPC_GPIO0->FIOCLR = (1 << IN2_PIN); // IN2 en LOW
    LPC_GPIO0->FIOSET = (1 << IN4_PIN); // IN4 en HIGH

    return;
}

/**
 * @brief Detiene el motor vertical.
 *
 * Detiene el motor vertical aplicando freno.
 */
void STOP_MOTOR_V()
{
    LPC_GPIO0->FIOSET = (1 << IN2_PIN); // IN2 en HIGH
    LPC_GPIO0->FIOSET = (1 << IN4_PIN); // IN4 en HIGH

    return;
}

/**
 * @brief Rota el motor en sentido antihorario.
 *
 * Activa los pines necesarios para rotar el motor hacia la izquierda.
 */
void ROTATE_COUNTERCLOCKWISE()
{
    LPC_GPIO0->FIOSET = (1 << IN0_PIN); // IN0 en HIGH
    LPC_GPIO0->FIOCLR = (1 << IN1_PIN); // IN1 en LOW

    return;
}

/**
 * @brief Rota el motor en sentido horario.
 *
 * Activa los pines necesarios para rotar el motor hacia la derecha.
 */
void ROTATE_CLOCKWISE()
{
    LPC_GPIO0->FIOCLR = (1 << IN0_PIN); // IN0 en LOW
    LPC_GPIO0->FIOSET = (1 << IN1_PIN); // IN1 en HIGH

    return;
}

/**
 * @brief Detiene el motor horizontal.
 *
 * Detiene el motor aplicando freno.
 */
void STOP_MOTOR()
{
    LPC_GPIO0->FIOSET = (1 << IN0_PIN); // IN0 en HIGH
    LPC_GPIO0->FIOSET = (1 << IN1_PIN); // IN1 en HIGH

    return;
}

/**
 * @brief Moves the motor forward depending on the motor type.
 */
void motor_forward()
{
    if (motor)
    {                                       // Motor base - horizontal
        LPC_GPIO0->FIOSET = (1 << IN0_PIN); // IN1 en HIGH
        LPC_GPIO0->FIOCLR = (1 << IN1_PIN); // IN2 en LOW
    }
    else
    {                                       // Motor brazo - vertical
        LPC_GPIO0->FIOSET = (1 << IN2_PIN); // IN1 en HIGH
        LPC_GPIO0->FIOCLR = (1 << IN4_PIN); // IN2 en LOW
    }
}
/**
 * @brief Moves the motor in reverse depending on the motor type.
 */

void motor_reverse()
{
    if (motor)
    {
        LPC_GPIO0->FIOCLR = (1 << IN0_PIN); // IN1 en LOW
        LPC_GPIO0->FIOSET = (1 << IN1_PIN); // IN2 en HIGH
    }
    else
    {
        LPC_GPIO0->FIOCLR = (1 << IN2_PIN); // IN1 en LOW
        LPC_GPIO0->FIOSET = (1 << IN4_PIN); // IN2 en HIGH
    }
}

/**
 * @brief Stops the motor depending on the motor type.
 */
void motor_stop()
{
    if (motor)
    {
        LPC_GPIO0->FIOCLR = (1 << IN0_PIN); // IN1 en LOW
        LPC_GPIO0->FIOCLR = (1 << IN1_PIN); // IN2 en LOW
    }
    else
    {
        LPC_GPIO0->FIOCLR = (1 << IN2_PIN); // IN1 en LOW
        LPC_GPIO0->FIOCLR = (1 << IN4_PIN); // IN2 en LOW
    }
}
/**
 * @brief Configures PWM for motor speed control on ENA pin.
 */
void setupPWM(void)
{
    LPC_SC->PCONP |= (1 << 6);     // Habilitar el periférico PWM1
    LPC_SC->PCLKSEL0 |= (1 << 12); // Seleccionar el clock para PWM1

    LPC_PINCON->PINSEL4 |= (1 << 2); // Configurar P2.1 como PWM1.2

    LPC_PWM1->MR0 = 10000;                // Ciclo total del PWM (1000 -> 1kHz)
    LPC_PWM1->MR2 = 0;                    // Ciclo de trabajo inicial al 0%
    LPC_PWM1->LER |= (1 << 0) | (1 << 2); // Actualizar MR0 y MR2

    LPC_PWM1->MCR = (1 << 1);            // Reset del contador en MR0
    LPC_PWM1->PCR |= (1 << 10);          // Habilitar PWM1.2
    LPC_PWM1->TCR = (1 << 0) | (1 << 3); // Habilitar el contador y el modo PWM
}

/**
 * @brief Sets motor speed via PWM by adjusting the duty cycle.
 * @param dutyCycle Percentage of the duty cycle (0-100).
 */
void setMotorSpeed(uint8_t dutyCycle)
{
    // if (dutyCycle > 100) dutyCycle = 100; // Limitar el duty cycle al 100%
    LPC_PWM1->MR2 = (LPC_PWM1->MR0 * dutyCycle) / 100; // Calcular nuevo MR2
    LPC_PWM1->LER |= (1 << 2);                         // Actualizar MR2
}
/**
 * @brief Configures SysTick for time-based operations based on mode.
 */
void config_SYSTICK()
{

    if ((flag_exti = 1))
    {

        SYSTICK_InternalInit(time_manual);
    }

    else
    {
        SYSTICK_InternalInit(time_automatico);
    }
    SYSTICK_Cmd(ENABLE);
    SYSTICK_IntCmd(ENABLE);
}

/**
 * @brief SysTick interrupt handler toggling an LED.
 */
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
    if (flag_exti)
    {
        config_SYSTICK();
    }
}

/**
 * @brief Configures DMA for data transfer.
 */
void confDMA(void)
{
    GPDMA_LLI_Type DMA_LLI_Struct;
    // Prepare DMA link list item structure
    DMA_LLI_Struct.SrcAddr = (uint32_t)dac_sine_lut;
    DMA_LLI_Struct.DstAddr = (uint32_t) & (LPC_DAC->DACR);
    DMA_LLI_Struct.NextLLI = (uint32_t)&DMA_LLI_Struct;
    DMA_LLI_Struct.Control = DMA_SIZE | (2 << 18) // source width 32 bit
                             | (2 << 21)          // dest. width 32 bit
                             | (1 << 26);         // source increment;

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
    return;
}

/**
 * @brief Configures DAC with DMA support.
 */
void confDac(void)
{
    uint32_t tmp;
    DAC_CONVERTER_CFG_Type DAC_ConverterConfigStruct;
    DAC_ConverterConfigStruct.CNT_ENA = SET;
    DAC_ConverterConfigStruct.DMA_ENA = SET;

    DAC_Init(LPC_DAC);
    /* set time out for DAC*/
    tmp = (PCLK_DAC_IN_MHZ * 1000000) / (SINE_FREQ_IN_HZ * NUM_SINE_SAMPLE);
    DAC_SetDMATimeOut(LPC_DAC, tmp);
    DAC_ConfigDAConverterControl(LPC_DAC, &DAC_ConverterConfigStruct);
    return;
}