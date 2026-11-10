/*******************************************************************************//**
 * @file    Tick.h
 * @brief   Interfaz de la base de tiempo del sistema (SysTick, 10 ms).
 * @details Inicialización de SysTick y consulta del pedido de telemetría.
 * @note    ESW.2.1.5
 ********************************************************************************/
#ifndef TICK_H
#define TICK_H

#include <stdbool.h>

void Tick_Init(void);
bool Tick_TelemetriaPendiente(void);

#endif
