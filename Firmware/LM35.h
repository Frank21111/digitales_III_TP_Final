/*******************************************************************************//**
 * @file    LM35.h
 * @brief   Interfaz del driver del sensor de temperatura LM35 (AD0.0).
 * @details Inicialización del ADC, inicio de conversión y lectura en 0,1 °C.
 * @note    ESW.2.1.7
 ********************************************************************************/
#ifndef LM35_H
#define LM35_H

#include <stdint.h>
#include <stdbool.h>

void LM35_Init(void);
void LM35_IniciarConversion(void);
bool LM35_NuevaMedicion(uint16_t *temperatura);

#endif
