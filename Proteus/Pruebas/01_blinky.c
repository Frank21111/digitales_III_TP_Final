/* Prueba 1: GPIO y SysTick. P0.22 (J6-24, también el LED de la placa) cambia cada 500 ms.
 * Esperado en Proteus: onda cuadrada de 1 Hz en J6-24. */
#include "LPC17xx.h"

static volatile uint32_t ms = 0;

void SysTick_Handler(void)
{
    ms++;
}

int main(void)
{
    uint32_t proximo = 500U;
    SystemCoreClockUpdate();
    LPC_PINCON->PINSEL1 &= ~(3U << 12);
    LPC_GPIO0->FIODIR |= (1U << 22);
    SysTick_Config(SystemCoreClock / 1000U);
    while (1) {
        if (ms >= proximo) {
            proximo += 500U;
            LPC_GPIO0->FIOPIN ^= (1U << 22);
        }
    }
}
