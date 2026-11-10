/*******************************************************************************//**
 * @file    LM35.c
 * @brief   Driver del sensor de temperatura LM35 con el ADC (AD0.0).
 * @details El LM35 entrega 10 mV/°C. Su salida pasa por un RC 1 kΩ/1 µF a P0.23
 *          (AD0.0). El ADC de 12 bits lleva 0–3,3 V a 0–4095 cuentas: mV =
 *          cuentas·3300/4095; como 1 mV = 0,1 °C, la temperatura en décimas de °C
 *          vale lo mismo que la tensión en mV. Conversión por software cada 1 s
 *          con aviso de fin por interrupción.
 * @note    ESW.2.1.7
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "LPC17xx.h"
#include "LM35.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define ADC_CLKDIV          1U          // 25 MHz/(1 + 1) = 12,5 MHz (máximo: 13 MHz)
#define ADC_PRIORIDAD       3U          // Debajo de TIMER2 y SysTick
#define ADC_VREF_MV         3300U       // Referencia del ADC (nominal); cargar la tensión medida en J6-28
#define ADC_CUENTAS_MAX     4095U       // 12 bits: 2^12 − 1
#define LM35_MV_POR_C       10U         // Sensibilidad del LM35: 10 mV/°C
#define DECIMAS_POR_C       10U         // La temperatura se expresa en 0,1 °C

/*==============================================================================
 * DEFINICIÓN DE VARIABLES GLOBALES
 *============================================================================*/
static volatile uint16_t temperaturaDc = 0U;    // Última temperatura [0,1 °C]
static volatile bool nuevaTemperatura = false;  // ISR → main: temperatura sin leer

/*==============================================================================
 * PROTOTIPOS DE FUNCIONES
 *============================================================================*/
static uint16_t LM35_RawAMilivolt(uint16_t raw);
static uint16_t LM35_MilivoltATemperatura(uint16_t milivolt);
void ADC_IRQHandler(void);

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura P0.23 como AD0.0 y el ADC con interrupción del canal 0.
 * @details P0.23: AD0.0 (PINSEL1 bits 15:14 = 01) sin pull (PINMODE1 bits 15:14
 *          = 10), que alteraría la tensión. ADC: encendido (PCONP bit 12),
 *          PCLK = CCLK/4 = 25 MHz (PCLKSEL0 bits 25:24 = 00, valor de SystemInit)
 *          y CLKDIV = 1 → 12,5 MHz. ADCR: canal 0 (bit 0), CLKDIV (bits 15:8),
 *          BURST = 0 (a pedido), PDN = 1 (bit 21, operativo). ADINTEN bit 0: sólo
 *          el canal 0 interrumpe. NVIC prioridad 3.
 * @note    USW.2.1.7.1
 ********************************************************************************/
void LM35_Init(void)
{
    LPC_PINCON->PINSEL1 &= ~(3U << 14);
    LPC_PINCON->PINSEL1 |= (1U << 14);
    LPC_PINCON->PINMODE1 &= ~(3U << 14);
    LPC_PINCON->PINMODE1 |= (2U << 14);
    LPC_SC->PCONP |= (1U << 12);
    LPC_SC->PCLKSEL0 &= ~(3U << 24);
    LPC_ADC->ADCR = (1U << 0) | (ADC_CLKDIV << 8) | (1U << 21);
    LPC_ADC->ADINTEN = (1U << 0);
    NVIC_SetPriority(ADC_IRQn, ADC_PRIORIDAD);
    NVIC_EnableIRQ(ADC_IRQn);
}

/*******************************************************************************//**
 * @brief   Inicia una conversión del canal 0 (cada 1 s, desde SysTick).
 * @details START = 001 en ADCR bits 26:24 ("convertir ahora"). Tarda 65 ciclos
 *          del reloj del ADC (5,2 µs) y al terminar interrumpe.
 * @note    USW.2.1.7.2
 ********************************************************************************/
void LM35_IniciarConversion(void)
{
    LPC_ADC->ADCR &= ~(7U << 24);
    LPC_ADC->ADCR |= (1U << 24);
}

/*******************************************************************************//**
 * @brief   Entrega la última temperatura medida, si es nueva.
 * @details Lee el dato y baja la bandera con la IRQ del ADC deshabilitada para
 *          que la ISR no escriba en el medio.
 * @param   temperatura Destino de la temperatura [0,1 °C].
 * @return  true si hubo una medición nueva desde la llamada anterior.
 * @note    USW.2.1.7.3
 ********************************************************************************/
bool LM35_NuevaMedicion(uint16_t *temperatura)
{
    bool hayNueva;

    NVIC_DisableIRQ(ADC_IRQn);
    hayNueva = nuevaTemperatura;
    if (hayNueva) {
        *temperatura = temperaturaDc;
        nuevaTemperatura = false;
    }
    NVIC_EnableIRQ(ADC_IRQn);
    return hayNueva;
}

/*******************************************************************************//**
 * @brief   Convierte cuentas del ADC en tensión.
 * @details mV = cuentas·3300/4095 en 32 bits (producto máximo 13,5·10^6).
 * @param   raw     Resultado de 12 bits del ADC (0..4095).
 * @return  Tensión de entrada [mV].
 * @note    USW.2.1.7.4
 ********************************************************************************/
static uint16_t LM35_RawAMilivolt(uint16_t raw)
{
    return (uint16_t)(((uint32_t)raw * ADC_VREF_MV) / ADC_CUENTAS_MAX);
}

/*******************************************************************************//**
 * @brief   Convierte la tensión del LM35 en temperatura.
 * @details T [0,1 °C] = mV·10/(10 mV/°C), numéricamente igual a los mV
 *          (345 mV → 345 → 34,5 °C).
 * @param   milivolt    Tensión de salida del LM35 [mV].
 * @return  Temperatura [0,1 °C].
 * @note    USW.2.1.7.5
 ********************************************************************************/
static uint16_t LM35_MilivoltATemperatura(uint16_t milivolt)
{
    return (uint16_t)(((uint32_t)milivolt * DECIMAS_POR_C) / LM35_MV_POR_C);
}

/*******************************************************************************//**
 * @brief   Rutina de servicio de interrupción del ADC.
 * @details Lee ADDR0 (la lectura borra DONE y el pedido). Si DONE (bit 31) = 1,
 *          toma el resultado de los bits 15:4, lo pasa a mV y a 0,1 °C y avisa
 *          al superlazo.
 * @note    USW.2.1.7.6
 ********************************************************************************/
void ADC_IRQHandler(void)
{
    uint32_t dato = LPC_ADC->ADDR0;

    if ((dato & (1U << 31)) != 0U) {
        temperaturaDc = LM35_MilivoltATemperatura(LM35_RawAMilivolt((uint16_t)((dato >> 4) & 0xFFFU)));
        nuevaTemperatura = true;
    }
}
