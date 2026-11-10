/*******************************************************************************//**
 * @file    Control.h
 * @brief   Interfaz de la lógica de control de nivel y de temperatura.
 * @details Parámetros del proceso y del PI, tipo Estado_t y funciones del ESW.
 * @note    ESW.2.1.2
 ********************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define H_SENSOR_MM     220U        // Distancia del HC-SR04 al fondo del tanque
#define NIVEL_MAX_MM    180U        // Corte de seguridad: bomba al 0 % en cualquier modo
#define SP_MIN_MM       30U         // Setpoint mínimo admitido (0 = bomba apagada)
#define SP_MAX_MM       160U        // Setpoint máximo admitido
#define PID_KP          1.51f       // Kc = τ/(K·λ) = 15,1 m⁻¹ = 1,51 %/mm (λ = 30 s)
#define PID_KI          0.0126f     // Kc/Ti = 1,51/120 s = 0,0126 %/(mm·s)
#define PID_KD          0.0f        // Control PI: sin término derivativo
#define PID_TS          0.1f        // Ts = 100 ms (un disparo del HC-SR04)
#define T_ON_DC         450U        // 45,0 °C: enciende ventilador, LED y buzzer
#define T_OFF_DC        400U        // 40,0 °C: los apaga (histéresis de 5 °C)

typedef struct {
    uint16_t nivel_mm;              // Nivel de agua medido [mm]
    uint16_t setpoint_mm;           // Nivel deseado [mm] (0 = bomba apagada)
    uint16_t temp_dC;               // Temperatura de la placa [0,1 °C]
    uint8_t  duty;                  // Ciclo de trabajo aplicado a la bomba [%]
    bool     ventilador;            // Ventilador encendido
    bool     alarma;                // LED y buzzer encendidos
    bool     falla;                 // Falla del sensor de nivel
    char     modo;                  // 'A' automático (PI) o 'M' manual
} Estado_t;

void Control_Init(void);
void Control_Nivel(uint16_t distancia);
void Control_FallaSensor(void);
void Control_Termico(uint16_t temperatura);
bool Control_SetSetpoint(uint16_t setpoint);
bool Control_SetManual(uint16_t porcentaje);
const Estado_t *Control_GetEstado(void);

#endif
