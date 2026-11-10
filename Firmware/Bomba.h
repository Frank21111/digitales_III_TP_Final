/*******************************************************************************//**
 * @file    Bomba.h
 * @brief   Interfaz del driver de la bomba (PWM1.1 en P2.0, 1 kHz).
 * @details Inicialización del PWM1 y fijación del ciclo de trabajo.
 * @note    ESW.2.1.8
 ********************************************************************************/
#ifndef BOMBA_H
#define BOMBA_H

#include <stdint.h>

void Bomba_Init(void);
void Bomba_SetDuty(uint8_t porcentaje);

#endif
