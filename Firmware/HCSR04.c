/*******************************************************************************//**
 * @file    HCSR04.c
 * @brief   Driver del sensor ultrasónico HC-SR04 con captura del TIMER2.
 * @details Un pulso de ≥ 10 µs en TRIG (P0.5) hace que el sensor emita ultrasonido
 *          y mantenga ECHO en alto hasta oír el rebote. ECHO (5 V) entra por un
 *          divisor 1 kΩ/2 kΩ a P0.4 = CAP2.0: el TIMER2 cuenta µs y el hardware
 *          copia la cuenta en CR0 en cada flanco, sin depender de la demora del
 *          software. Ancho t [µs] → d = (t·343 + 1000)/2000 [mm]; válido de 116 a
 *          1500 µs (2 cm a 257 mm: fondo del tanque más margen). Si cambia
 *          H_SENSOR_MM (220), recalcular ANCHO_MAX_US ≈ (H_SENSOR_MM + 37)·2000/343.
 * @note    ESW.2.1.6
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "LPC17xx.h"
#include "HCSR04.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define TIMER2_PR           24U         // 25 MHz/(24 + 1) = 1 MHz → 1 µs por cuenta
#define TIMER2_PRIORIDAD    1U          // La más alta: identifica el flanco a tiempo
#define TRIG_US             12U         // Ancho del pulso de disparo (mínimo: 10 µs)
#define ANCHO_MIN_US        116U        // 2 cm: alcance mínimo del sensor
#define ANCHO_MAX_US        1500U       // ≈ 257 mm: fondo + 37 mm; más largo = sin eco válido
#define SONIDO_M_S          343U        // Velocidad del sonido a 20 °C
#define FACTOR_US_MM        2000U       // Ida y vuelta (2) y µm → mm (1000)
#define FALLAS_LIMITE       3U          // Disparos seguidos sin eco = falla

/*==============================================================================
 * DEFINICIÓN DE VARIABLES GLOBALES
 *============================================================================*/
static uint32_t tiempoSubida = 0U;              // CR0 en el flanco ascendente [µs]
static volatile uint16_t distanciaMm = 0U;      // Última distancia válida [mm]
static volatile bool nuevaMedicion = false;     // ISR → main: distancia sin leer
static volatile bool medicionPendiente = false; // Disparo todavía sin eco válido
static volatile uint8_t fallas = 0U;            // Disparos seguidos sin eco válido

/*==============================================================================
 * PROTOTIPOS DE FUNCIONES
 *============================================================================*/
static uint16_t HCSR04_AnchoADistancia(uint32_t anchoUs);
void TIMER2_IRQHandler(void);

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura TRIG como salida y el TIMER2 para capturar ECHO.
 * @details P0.5: GPIO (PINSEL0 bits 11:10 = 00) de salida en bajo. P0.4: CAP2.0
 *          (PINSEL0 bits 9:8 = 11) sin pull (PINMODE0 bits 9:8 = 10), porque el
 *          divisor fija el nivel. TIMER2: encendido (PCONP bit 22), PCLK = 25 MHz
 *          (PCLKSEL1 bits 13:12 = 00), modo temporizador (CTCR = 0), PR = 24.
 *          Con TC en reset (TCR bit 1) borra IR y programa CCR: captura en CR0
 *          por flanco ascendente (bit 0) y descendente (bit 1) con interrupción
 *          (bit 2). NVIC prioridad 1 y arranque (TCR bit 0).
 * @note    USW.2.1.6.1
 ********************************************************************************/
void HCSR04_Init(void)
{
    LPC_PINCON->PINSEL0 &= ~(3U << 10);
    LPC_GPIO0->FIOCLR = (1U << 5);
    LPC_GPIO0->FIODIR |= (1U << 5);
    LPC_PINCON->PINSEL0 |= (3U << 8);
    LPC_PINCON->PINMODE0 &= ~(3U << 8);
    LPC_PINCON->PINMODE0 |= (2U << 8);
    LPC_SC->PCONP |= (1U << 22);
    LPC_SC->PCLKSEL1 &= ~(3U << 12);
    LPC_TIM2->CTCR = 0U;
    LPC_TIM2->PR = TIMER2_PR;
    LPC_TIM2->TCR = (1U << 1);
    LPC_TIM2->IR = (0x3FU << 0);
    LPC_TIM2->CCR = (1U << 0) | (1U << 1) | (1U << 2);
    NVIC_SetPriority(TIMER2_IRQn, TIMER2_PRIORIDAD);
    NVIC_EnableIRQ(TIMER2_IRQn);
    LPC_TIM2->TCR = (1U << 0);
}

/*******************************************************************************//**
 * @brief   Inicia una medición generando el pulso de TRIG (cada 100 ms).
 * @details Si la medición anterior no terminó con eco válido suma una falla
 *          (hasta 3). Marca la medición como pendiente, sube TRIG (FIOSET),
 *          espera 12 cuentas de TC (12 µs) y lo baja (FIOCLR). La resta sin
 *          signo TC − inicio sigue siendo correcta si TC desborda.
 * @note    USW.2.1.6.2
 ********************************************************************************/
void HCSR04_Disparar(void)
{
    uint32_t inicio;

    if (medicionPendiente && (fallas < FALLAS_LIMITE)) {
        fallas++;
    }
    medicionPendiente = true;
    LPC_GPIO0->FIOSET = (1U << 5);
    inicio = LPC_TIM2->TC;
    while ((LPC_TIM2->TC - inicio) < TRIG_US) {
    }
    LPC_GPIO0->FIOCLR = (1U << 5);
}

/*******************************************************************************//**
 * @brief   Entrega la última distancia medida, si es nueva.
 * @details Lee el dato y baja la bandera con la IRQ del TIMER2 deshabilitada para
 *          que la ISR no escriba en el medio; una captura en ese lapso queda
 *          pendiente en el NVIC y no se pierde.
 * @param   distancia   Destino de la distancia [mm].
 * @return  true si hubo una medición nueva desde la llamada anterior.
 * @note    USW.2.1.6.3
 ********************************************************************************/
bool HCSR04_NuevaMedicion(uint16_t *distancia)
{
    bool hayNueva;

    NVIC_DisableIRQ(TIMER2_IRQn);
    hayNueva = nuevaMedicion;
    if (hayNueva) {
        *distancia = distanciaMm;
        nuevaMedicion = false;
    }
    NVIC_EnableIRQ(TIMER2_IRQn);
    return hayNueva;
}

/*******************************************************************************//**
 * @brief   Indica si el sensor está en falla.
 * @details Falla = 3 disparos seguidos sin eco válido (sensor desconectado, sin
 *          superficie que refleje o con un eco de más allá del fondo: sensor
 *          inclinado o descolocado); la borra la primera medición válida.
 * @return  true si hay falla.
 * @note    USW.2.1.6.4
 ********************************************************************************/
bool HCSR04_Falla(void)
{
    return (fallas >= FALLAS_LIMITE);
}

/*******************************************************************************//**
 * @brief   Convierte el ancho del pulso de ECHO en distancia.
 * @details d = t·v/2 con v = 343 m/s: d [mm] = t [µs]·343/2000, redondeado al
 *          mm más cercano (se suma la mitad del divisor antes de dividir). El
 *          producto máximo (1500·343 = 514 500) entra holgado en 32 bits.
 * @param   anchoUs     Ancho del pulso de ECHO [µs].
 * @return  Distancia [mm].
 * @note    USW.2.1.6.5
 ********************************************************************************/
static uint16_t HCSR04_AnchoADistancia(uint32_t anchoUs)
{
    return (uint16_t)(((anchoUs * SONIDO_M_S) + (FACTOR_US_MM / 2U)) / FACTOR_US_MM);
}

/*******************************************************************************//**
 * @brief   Rutina de servicio de interrupción del TIMER2 (captura CAP2.0).
 * @details Lee CR0 y borra la bandera (IR bit 4, se borra escribiendo 1). El
 *          flanco se identifica leyendo P0.4 en FIOPIN, que refleja el pin aun
 *          en función CAP2.0: alto → ascendente, se guarda el instante; bajo →
 *          descendente, ancho = CR0 − tiempoSubida. Si había disparo pendiente y
 *          el ancho es válido, publica la distancia y pone las fallas en 0.
 * @note    USW.2.1.6.6
 ********************************************************************************/
void TIMER2_IRQHandler(void)
{
    uint32_t captura = LPC_TIM2->CR0;
    uint32_t ancho;

    LPC_TIM2->IR = (1U << 4);
    if ((LPC_GPIO0->FIOPIN & (1U << 4)) != 0U) {
        tiempoSubida = captura;
    } else {
        ancho = captura - tiempoSubida;
        if (medicionPendiente && (ancho >= ANCHO_MIN_US) && (ancho <= ANCHO_MAX_US)) {
            distanciaMm = HCSR04_AnchoADistancia(ancho);
            nuevaMedicion = true;
            medicionPendiente = false;
            fallas = 0U;
        }
    }
}
