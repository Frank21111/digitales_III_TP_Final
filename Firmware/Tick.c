/*******************************************************************************//**
 * @file    Tick.c
 * @brief   Base de tiempo del sistema con SysTick.
 * @details SysTick es un contador descendente de 24 bits del Cortex-M3: cuenta de
 *          LOAD a 0 a 100 MHz, recarga y pide interrupción cada (LOAD + 1)/100 MHz
 *          = 10 ms. La ISR reparte el tiempo: HC-SR04 cada 100 ms (Ts del lazo),
 *          telemetría cada 500 ms y LM35 cada 1 s, sin depender del superlazo.
 * @note    ESW.2.1.5
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "LPC17xx.h"
#include "Tick.h"
#include "HCSR04.h"
#include "LM35.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define SYSTICK_LOAD        999999U     // 100 MHz · 10 ms − 1
#define SYSTICK_PRIORIDAD   2U          // Por debajo de TIMER2 (captura del eco)
#define TICKS_HCSR04        10U         // 10 · 10 ms = 100 ms
#define TICKS_TELEMETRIA    50U         // 50 · 10 ms = 500 ms
#define TICKS_CICLO         100U        // 100 · 10 ms = 1 s (LM35); múltiplo de 10 y 50

/*==============================================================================
 * DEFINICIÓN DE VARIABLES GLOBALES
 *============================================================================*/
static uint8_t ticks = 0U;                          // Cuenta 0..99 dentro del ciclo de 1 s
static volatile bool telemetriaPendiente = false;   // ISR → main: toca enviar telemetría

/*==============================================================================
 * PROTOTIPOS DE FUNCIONES
 *============================================================================*/
void SysTick_Handler(void);

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura SysTick para interrumpir cada 10 ms.
 * @details LOAD = 999 999, VAL = 0 (período completo desde el inicio), prioridad
 *          2 y CTRL: CLKSOURCE (bit 2, reloj del núcleo), TICKINT (bit 1, pide
 *          interrupción) y ENABLE (bit 0). Va última en main: arranca las
 *          mediciones.
 * @note    USW.2.1.5.1
 ********************************************************************************/
void Tick_Init(void)
{
    SysTick->LOAD = SYSTICK_LOAD;
    SysTick->VAL = 0U;
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORIDAD);
    SysTick->CTRL = (1U << 2) | (1U << 1) | (1U << 0);
}

/*******************************************************************************//**
 * @brief   Rutina de servicio de interrupción de SysTick (cada 10 ms).
 * @details Avanza la cuenta del ciclo de 1 s: al cerrarlo inicia el ADC, en los
 *          múltiplos de 10 dispara el HC-SR04 (≈ 12 µs) y en los de 50 pide
 *          telemetría. No hay bandera que borrar: el núcleo limpia el pedido.
 * @note    USW.2.1.5.2
 ********************************************************************************/
void SysTick_Handler(void)
{
    ticks++;
    if (ticks >= TICKS_CICLO) {
        ticks = 0U;
        LM35_IniciarConversion();
    }
    if ((ticks % TICKS_HCSR04) == 0U) {
        HCSR04_Disparar();
    }
    if ((ticks % TICKS_TELEMETRIA) == 0U) {
        telemetriaPendiente = true;
    }
}

/*******************************************************************************//**
 * @brief   Indica si corresponde enviar telemetría y consume el pedido.
 * @details Sólo escribe la bandera si la encontró en true: escribirla siempre
 *          podría borrar un pedido que la ISR levante entre lectura y escritura.
 * @return  true una vez cada 500 ms.
 * @note    USW.2.1.5.3
 ********************************************************************************/
bool Tick_TelemetriaPendiente(void)
{
    if (!telemetriaPendiente) {
        return false;
    }
    telemetriaPendiente = false;
    return true;
}
