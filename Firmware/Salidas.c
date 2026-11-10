/*******************************************************************************//**
 * @file    Salidas.c
 * @brief   Driver de las salidas digitales: ventilador, LED y buzzer.
 * @details GPIO de salida, 1 = encendido: P2.2 → PC817 → IRF630N → ventilador
 *          12 V; P0.22 → PN2222A → LED rojo (también el LED de la placa); P2.3 →
 *          PN2222A → buzzer. LED y buzzer forman la alarma. FIOSET/FIOCLR sólo
 *          afectan los bits escritos en 1: no hace falta leer-modificar-escribir.
 * @note    ESW.2.1.9
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "LPC17xx.h"
#include "Salidas.h"

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura ventilador, LED y buzzer como salidas apagadas.
 * @details GPIO (00) en PINSEL1 bits 13:12 (P0.22) y PINSEL4 bits 5:4 (P2.2) y
 *          7:6 (P2.3). Primero FIOCLR y después FIODIR: ningún pin presenta un 1
 *          transitorio. Es lo primero que ejecuta main.
 * @note    USW.2.1.9.1
 ********************************************************************************/
void Salidas_Init(void)
{
    LPC_PINCON->PINSEL1 &= ~(3U << 12);
    LPC_PINCON->PINSEL4 &= ~((3U << 4) | (3U << 6));
    LPC_GPIO0->FIOCLR = (1U << 22);
    LPC_GPIO2->FIOCLR = (1U << 2) | (1U << 3);
    LPC_GPIO0->FIODIR |= (1U << 22);
    LPC_GPIO2->FIODIR |= (1U << 2) | (1U << 3);
}

/*******************************************************************************//**
 * @brief   Enciende o apaga el ventilador.
 * @details Escribe el bit 2 de FIOSET o FIOCLR del puerto 2.
 * @param   encendido   true enciende, false apaga.
 * @note    USW.2.1.9.2
 ********************************************************************************/
void Ventilador_Set(bool encendido)
{
    if (encendido) {
        LPC_GPIO2->FIOSET = (1U << 2);
    } else {
        LPC_GPIO2->FIOCLR = (1U << 2);
    }
}

/*******************************************************************************//**
 * @brief   Enciende o apaga la alarma (LED y buzzer juntos).
 * @details Escribe P0.22 (LED) y P2.3 (buzzer) con FIOSET o FIOCLR.
 * @param   encendida   true enciende, false apaga.
 * @note    USW.2.1.9.3
 ********************************************************************************/
void Alarma_Set(bool encendida)
{
    if (encendida) {
        LPC_GPIO0->FIOSET = (1U << 22);
        LPC_GPIO2->FIOSET = (1U << 3);
    } else {
        LPC_GPIO0->FIOCLR = (1U << 22);
        LPC_GPIO2->FIOCLR = (1U << 3);
    }
}
