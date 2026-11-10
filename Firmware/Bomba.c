/*******************************************************************************//**
 * @file    Bomba.c
 * @brief   Driver de la bomba con PWM1 (salida PWM1.1 en P2.0).
 * @details PWM: señal de frecuencia fija cuyo tiempo en alto (ciclo de trabajo)
 *          fija la tensión media en la bomba. P2.0 excita el PC817, que maneja el
 *          MOSFET IRF630N. El contador cuenta a 25 MHz y, en modo PWM, al igualar
 *          MR0 = 24 999 vuelve a 1 (UM10360, tabla 448): período de 24 999
 *          cuentas ≈ 1 ms (1000,04 Hz). La salida sube al inicio del período y
 *          baja al igualar MR1, así que MR1 = 250·duty (≈ 250 cuentas por cada
 *          1 %, error 0,004 %).
 * @note    ESW.2.1.8
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "LPC17xx.h"
#include "Bomba.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define PWM_MR0             24999U      // Modo PWM: TC cuenta 1..MR0 → 25 MHz/24 999 ≈ 1 kHz
#define PWM_CUENTAS_PCT     250U        // 250 cuentas por 1 % (24 999 por período: +0,004 %)
#define DUTY_MAX            100U        // Ciclo de trabajo máximo [%]

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura el PWM1 a 1 kHz con la salida PWM1.1 al 0 %.
 * @details PWM1 encendido (PCONP bit 6) con PCLK = 25 MHz (PCLKSEL0 bits 13:12 =
 *          00). Con el contador en reset (TCR bit 1): PR = 0, reinicio en MR0
 *          (MCR bit 1), MR0 = 24 999, MR1 = 0 y LER bits 0 y 1 para cargarlos.
 *          PCR bit 9 habilita PWM1.1; TCR: contador (bit 0) y modo PWM (bit 3),
 *          con MR0 ya cargado. Al final P2.0 pasa a PWM1.1 (PINSEL4 bits 1:0 =
 *          01) ya al 0 %: con MR1 = 0 la salida nunca sube.
 * @note    USW.2.1.8.1
 ********************************************************************************/
void Bomba_Init(void)
{
    LPC_SC->PCONP |= (1U << 6);
    LPC_SC->PCLKSEL0 &= ~(3U << 12);
    LPC_PWM1->TCR = (1U << 1);
    LPC_PWM1->PR = 0U;
    LPC_PWM1->MCR = (1U << 1);
    LPC_PWM1->MR0 = PWM_MR0;
    LPC_PWM1->MR1 = 0U;
    LPC_PWM1->LER = (1U << 0) | (1U << 1);
    LPC_PWM1->PCR = (1U << 9);
    LPC_PWM1->TCR = (1U << 0) | (1U << 3);
    LPC_PINCON->PINSEL4 &= ~(3U << 0);
    LPC_PINCON->PINSEL4 |= (1U << 0);
}

/*******************************************************************************//**
 * @brief   Fija el ciclo de trabajo de la bomba.
 * @details Satura en 100 % y escribe MR1 = 250·duty: con 0 la salida queda en
 *          bajo; con 100, MR1 = 25 000 > MR0 nunca se alcanza y queda en alto.
 *          LER bit 1 aplica el valor al comenzar el próximo período (sin pulsos
 *          deformados).
 * @param   porcentaje  Ciclo de trabajo pedido [%].
 * @note    USW.2.1.8.2
 ********************************************************************************/
void Bomba_SetDuty(uint8_t porcentaje)
{
    if (porcentaje > DUTY_MAX) {
        porcentaje = DUTY_MAX;
    }
    LPC_PWM1->MR1 = PWM_CUENTAS_PCT * porcentaje;
    LPC_PWM1->LER |= (1U << 1);
}
