/*******************************************************************************//**
 * @file    main.c
 * @brief   Aplicación principal del control automático de nivel de agua.
 * @details Inicializa los ESW en un orden seguro y ejecuta el superlazo, que sólo
 *          atiende banderas: la base de tiempo la fija SysTick (ESW.2.1.5), así
 *          el período de muestreo no depende de lo que tarde el superlazo.
 * @note    ESW.2.1.1
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "Salidas.h"
#include "Bomba.h"
#include "HCSR04.h"
#include "LM35.h"
#include "UART.h"
#include "Control.h"
#include "Comm.h"
#include "Tick.h"

/*==============================================================================
 * FUNCIÓN PRINCIPAL
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Punto de entrada: inicialización y superlazo.
 * @details El startup de MCUXpresso ya llamó a SystemInit (CCLK = 100 MHz, PCLK
 *          = 25 MHz). Orden: salidas en 0, bomba al 0 %, sensores, UART, control
 *          y por último SysTick, que arranca las mediciones. Superlazo: distancia
 *          nueva → Control_Nivel; falla del sensor → Control_FallaSensor;
 *          temperatura nueva → Control_Termico; comandos de la PC; telemetría.
 * @return  No retorna.
 * @note    USW.2.1.1.1
 ********************************************************************************/
int main(void)
{
    uint16_t distancia = 0U;
    uint16_t temperatura = 0U;

    Salidas_Init();
    Bomba_Init();
    HCSR04_Init();
    LM35_Init();
    UART_Init();
    Control_Init();
    Tick_Init();

    while (1) {
        if (HCSR04_NuevaMedicion(&distancia)) {
            Control_Nivel(distancia);
        }
        if (HCSR04_Falla()) {
            Control_FallaSensor();
        }
        if (LM35_NuevaMedicion(&temperatura)) {
            Control_Termico(temperatura);
        }
        Comm_ProcesarComandos();
        if (Tick_TelemetriaPendiente()) {
            Comm_EnviarTelemetria();
        }
    }
    return 0;
}
