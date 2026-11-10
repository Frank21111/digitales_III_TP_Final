/*******************************************************************************//**
 * @file    HCSR04.h
 * @brief   Interfaz del driver del sensor ultrasónico HC-SR04.
 * @details TRIG en P0.5, ECHO en P0.4 (CAP2.0): disparo, distancia y falla.
 * @note    ESW.2.1.6
 ********************************************************************************/
#ifndef HCSR04_H
#define HCSR04_H

#include <stdint.h>
#include <stdbool.h>

void HCSR04_Init(void);
void HCSR04_Disparar(void);
bool HCSR04_NuevaMedicion(uint16_t *distancia);
bool HCSR04_Falla(void);

#endif
