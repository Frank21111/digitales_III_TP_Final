/*******************************************************************************//**
 * @file    Comm.h
 * @brief   Interfaz del protocolo ASCII entre la PC y el LPC1769.
 * @details PC → MCU: "SP=<0|30..160>\n" y "U=<0..100>\n". MCU → PC cada 500 ms:
 *          "L=..,SP=..,T=..,PWM=..,FAN=..,ALM=..,ERR=..,MODO=..\n".
 * @note    ESW.2.1.4
 ********************************************************************************/
#ifndef COMM_H
#define COMM_H

void Comm_ProcesarComandos(void);
void Comm_EnviarTelemetria(void);

#endif
