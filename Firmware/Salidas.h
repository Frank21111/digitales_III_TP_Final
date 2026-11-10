/*******************************************************************************//**
 * @file    Salidas.h
 * @brief   Interfaz del driver de las salidas digitales (ventilador y alarma).
 * @details P2.2 ventilador, P0.22 LED y P2.3 buzzer.
 * @note    ESW.2.1.9
 ********************************************************************************/
#ifndef SALIDAS_H
#define SALIDAS_H

#include <stdbool.h>

void Salidas_Init(void);
void Ventilador_Set(bool encendido);
void Alarma_Set(bool encendida);

#endif
