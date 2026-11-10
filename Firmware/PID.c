/*******************************************************************************//**
 * @file    PID.c
 * @brief   Controlador PID discreto con saturación y anti-windup.
 * @details u(k) = Kp·e(k) + I(k) + Kd·[e(k) − e(k−1)]/Ts, con I(k) = I(k−1) +
 *          Ki·Ts·e(k) (rectangular hacia atrás). La salida se limita y la
 *          integral sólo avanza si no saturó (integración condicional): evita el
 *          "windup", es decir, que la integral crezca con la bomba ya al 100 % y
 *          cause un sobrepico grande. Se usa con Kd = 0 (PI).
 * @note    ESW.2.1.3
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "PID.h"

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura un controlador y deja su estado en cero.
 * @details Copia ganancias, período y límites; integral y error previo en 0.
 * @param   pid         Controlador a configurar.
 * @param   kp          Ganancia proporcional.
 * @param   ki          Ganancia integral.
 * @param   kd          Ganancia derivativa.
 * @param   ts          Período de muestreo [s].
 * @param   salidaMin   Límite inferior de la salida.
 * @param   salidaMax   Límite superior de la salida.
 * @note    USW.2.1.3.1
 ********************************************************************************/
void PID_Init(PID_t *pid, float kp, float ki, float kd, float ts,
              float salidaMin, float salidaMax)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ts = ts;
    pid->salidaMin = salidaMin;
    pid->salidaMax = salidaMax;
    PID_Reset(pid, 0.0f);
}

/*******************************************************************************//**
 * @brief   Calcula la salida del controlador para una muestra.
 * @details e = setpoint − medición; suma P, I candidata y D. Si la suma excede
 *          un límite se recorta y la integral queda congelada; si no, se acepta
 *          la integral nueva. Debe llamarse una vez por período Ts.
 * @param   pid         Controlador.
 * @param   setpoint    Valor deseado.
 * @param   medicion    Valor medido.
 * @return  Salida limitada a [salidaMin, salidaMax].
 * @note    USW.2.1.3.2
 ********************************************************************************/
float PID_Compute(PID_t *pid, float setpoint, float medicion)
{
    float error = setpoint - medicion;
    float integralNueva = pid->integral + (pid->ki * pid->ts * error);
    float salida = (pid->kp * error) + integralNueva
                 + (pid->kd * (error - pid->errorPrevio) / pid->ts);

    if (salida > pid->salidaMax) {
        salida = pid->salidaMax;
    } else if (salida < pid->salidaMin) {
        salida = pid->salidaMin;
    } else {
        pid->integral = integralNueva;
    }
    pid->errorPrevio = error;
    return salida;
}

/*******************************************************************************//**
 * @brief   Reinicia el estado del controlador.
 * @details Fija la integral y borra el error previo. Con 0 se usa al apagar la
 *          bomba; con el duty actual, al pasar de MANUAL a AUTO (sin salto si el
 *          error es ≈ 0; si no, la salida suma Kp·e, como un cambio de setpoint).
 * @param   pid             Controlador.
 * @param   valorIntegral   Valor inicial del término integral.
 * @note    USW.2.1.3.3
 ********************************************************************************/
void PID_Reset(PID_t *pid, float valorIntegral)
{
    pid->integral = valorIntegral;
    pid->errorPrevio = 0.0f;
}
