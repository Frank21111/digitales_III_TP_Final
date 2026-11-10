/*******************************************************************************//**
 * @file    PID.h
 * @brief   Interfaz del controlador PID discreto con anti-windup.
 * @details PID_t guarda ganancias, período, límites de la salida y el estado.
 * @note    ESW.2.1.3
 ********************************************************************************/
#ifndef PID_H
#define PID_H

typedef struct {
    float kp;                       // Ganancia proporcional [%/mm]
    float ki;                       // Ganancia integral [%/(mm·s)]
    float kd;                       // Ganancia derivativa [%·s/mm]
    float ts;                       // Período de muestreo [s]
    float integral;                 // Término integral acumulado [%]
    float errorPrevio;              // Error de la muestra anterior [mm]
    float salidaMin;                // Límite inferior de la salida [%]
    float salidaMax;                // Límite superior de la salida [%]
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float ts,
              float salidaMin, float salidaMax);
float PID_Compute(PID_t *pid, float setpoint, float medicion);
void PID_Reset(PID_t *pid, float valorIntegral);

#endif
