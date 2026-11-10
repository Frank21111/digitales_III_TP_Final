/*******************************************************************************//**
 * @file    UART.h
 * @brief   Interfaz del driver de la UART0 (9600 baud, 8N1).
 * @details Inicialización, envío de texto y lectura de la última línea recibida.
 * @note    ESW.2.1.10
 ********************************************************************************/
#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

#define UART_LINEA_MAX  15U         // Caracteres útiles por línea recibida (más '\0')

void UART_Init(void);
void UART_EnviarTexto(const char *texto);
bool UART_LeerLinea(char *destino, uint8_t tam);

#endif
