/*******************************************************************************//**
 * @file    Control.c
 * @brief   Lógica de control de nivel (PI) y de protección térmica.
 * @details Convierte distancia en nivel, decide el ciclo de trabajo de la bomba
 *          según el modo (AUTO con PI o MANUAL, usado para identificar la
 *          planta), aplica el corte por nivel máximo y la reacción ante falla del
 *          sensor, y gobierna ventilador y alarma con histéresis. Sólo lo usa el
 *          superlazo: ninguna ISR accede a estas variables.
 * @note    ESW.2.1.2
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "Control.h"
#include "PID.h"
#include "Bomba.h"
#include "Salidas.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define DUTY_MAX        100U        // Ciclo de trabajo máximo [%]
#define MODO_AUTO       'A'         // Lazo cerrado con el PI
#define MODO_MANUAL     'M'         // Lazo abierto (ensayo escalón)

/*==============================================================================
 * DEFINICIÓN DE VARIABLES GLOBALES
 *============================================================================*/
static Estado_t estado;             // Variables del proceso (las lee la telemetría)
static PID_t pid;                   // Controlador del nivel
static uint8_t dutyManual = 0U;     // Ciclo de trabajo pedido en modo manual [%]

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Inicializa el estado del proceso y el PI.
 * @details Modo AUTO con setpoint 0 (bomba apagada), sin falla ni alarma. PI con
 *          Kp, Ki, Kd y Ts de Control.h y salida limitada a 0–100 %.
 * @note    USW.2.1.2.1
 ********************************************************************************/
void Control_Init(void)
{
    estado = (Estado_t){ .modo = MODO_AUTO };
    dutyManual = 0U;
    PID_Init(&pid, PID_KP, PID_KI, PID_KD, PID_TS, 0.0f, (float)DUTY_MAX);
}

/*******************************************************************************//**
 * @brief   Ejecuta un paso del lazo de nivel con una distancia nueva (cada 100 ms).
 * @details Nivel = H_SENSOR_MM − distancia (mínimo 0); una medición válida borra
 *          la falla. Prioridad: nivel ≥ NIVEL_MAX_MM → bomba 0 % en cualquier
 *          modo; MANUAL → duty fijo; AUTO con setpoint 0 → bomba 0 % e integral
 *          0; si no, PI con la salida redondeada al entero más cercano.
 * @param   distancia   Distancia sensor-agua [mm].
 * @note    USW.2.1.2.2
 ********************************************************************************/
void Control_Nivel(uint16_t distancia)
{
    estado.nivel_mm = (distancia >= H_SENSOR_MM) ? 0U : (uint16_t)(H_SENSOR_MM - distancia);
    estado.falla = false;
    if (estado.nivel_mm >= NIVEL_MAX_MM) {
        estado.duty = 0U;
    } else if (estado.modo == MODO_MANUAL) {
        estado.duty = dutyManual;
    } else if (estado.setpoint_mm == 0U) {
        estado.duty = 0U;
        PID_Reset(&pid, 0.0f);
    } else {
        estado.duty = (uint8_t)(PID_Compute(&pid, (float)estado.setpoint_mm,
                                            (float)estado.nivel_mm) + 0.5f);
    }
    Bomba_SetDuty(estado.duty);
}

/*******************************************************************************//**
 * @brief   Lleva el sistema a estado seguro ante falla del sensor de nivel.
 * @details Marca la falla (ERR=1) y apaga la bomba. No toca el PI: mientras dura
 *          la falla no se llama a PID_Compute, así que la integral queda congelada
 *          en su último valor (≈ u0) y al volver la medición el lazo sigue desde
 *          ahí; si el nivel bajó mucho, la salida satura y la integración
 *          condicional evita el windup.
 * @note    USW.2.1.2.3
 ********************************************************************************/
void Control_FallaSensor(void)
{
    estado.falla = true;
    estado.duty = 0U;
    Bomba_SetDuty(0U);
}

/*******************************************************************************//**
 * @brief   Gobierna ventilador y alarma con histéresis de temperatura.
 * @details T ≥ 45,0 °C enciende, T ≤ 40,0 °C apaga y entre ambos mantiene el
 *          estado: la banda evita conmutaciones por ruido cerca de un umbral.
 * @param   temperatura Temperatura de la placa [0,1 °C].
 * @note    USW.2.1.2.4
 ********************************************************************************/
void Control_Termico(uint16_t temperatura)
{
    estado.temp_dC = temperatura;
    if (temperatura >= T_ON_DC) {
        estado.alarma = true;
    } else if (temperatura <= T_OFF_DC) {
        estado.alarma = false;
    }
    estado.ventilador = estado.alarma;
    Ventilador_Set(estado.ventilador);
    Alarma_Set(estado.alarma);
}

/*******************************************************************************//**
 * @brief   Fija el setpoint y pasa a modo AUTO.
 * @details Válido: 0 (bomba apagada) o SP_MIN_MM..SP_MAX_MM. Si venía de MANUAL
 *          carga la integral con el duty aplicado: si SP = nivel la bomba sigue
 *          igual (sin salto); si no, la primera salida suma Kp·(SP − nivel), la
 *          reacción proporcional a un cambio de setpoint.
 * @param   setpoint    Nivel deseado [mm].
 * @return  true si se aplicó; false si el valor es inválido.
 * @note    USW.2.1.2.5
 ********************************************************************************/
bool Control_SetSetpoint(uint16_t setpoint)
{
    if ((setpoint != 0U) && ((setpoint < SP_MIN_MM) || (setpoint > SP_MAX_MM))) {
        return false;
    }
    if (estado.modo == MODO_MANUAL) {
        PID_Reset(&pid, (float)estado.duty);
    }
    estado.setpoint_mm = setpoint;
    estado.modo = MODO_AUTO;
    return true;
}

/*******************************************************************************//**
 * @brief   Pasa a modo MANUAL con un ciclo de trabajo fijo (ensayo escalón).
 * @details Se aplica en la próxima medición y respeta el corte por nivel máximo
 *          y la falla del sensor.
 * @param   porcentaje  Ciclo de trabajo, 0..100 %.
 * @return  true si se aplicó; false si el valor es inválido.
 * @note    USW.2.1.2.6
 ********************************************************************************/
bool Control_SetManual(uint16_t porcentaje)
{
    if (porcentaje > DUTY_MAX) {
        return false;
    }
    dutyManual = (uint8_t)porcentaje;
    estado.modo = MODO_MANUAL;
    return true;
}

/*******************************************************************************//**
 * @brief   Da acceso de sólo lectura a las variables del proceso.
 * @details Lo usa Comm_EnviarTelemetria; el puntero a const impide modificarlas.
 * @return  Puntero constante al estado.
 * @note    USW.2.1.2.7
 ********************************************************************************/
const Estado_t *Control_GetEstado(void)
{
    return &estado;
}
