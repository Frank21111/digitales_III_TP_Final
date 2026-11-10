/* Prueba 4: PWM1.1 en P2.0 (J6-42) a 1 kHz. El ciclo de trabajo pasa por 0, 25, 50, 75 y 100 %
 * cada un segundo y se informa por la UART.
 * Esperado: en el osciloscopio, período de 1 ms y ancho alto de 0; 0,25; 0,5; 0,75 y 1 ms. */
#include "uart_min.h"

static volatile uint32_t ms = 0;

void SysTick_Handler(void)
{
    ms++;
}

int main(void)
{
    const uint32_t duty[5] = {0U, 25U, 50U, 75U, 100U};
    uint32_t i = 0U, proximo = 0U;
    SystemCoreClockUpdate();
    uart_init();
    LPC_SC->PCONP |= (1U << 6);
    LPC_SC->PCLKSEL0 &= ~(3U << 12);
    LPC_PINCON->PINSEL4 &= ~3U;
    LPC_PINCON->PINSEL4 |= 1U;
    LPC_PWM1->TCR = 2U;
    LPC_PWM1->PR = 0U;
    LPC_PWM1->MCR = 2U;
    LPC_PWM1->MR0 = 25000U;
    LPC_PWM1->MR1 = 0U;
    LPC_PWM1->LER = 3U;
    LPC_PWM1->PCR = (1U << 9);
    LPC_PWM1->TCR = 9U;
    SysTick_Config(SystemCoreClock / 1000U);
    while (1) {
        if (ms >= proximo) {
            proximo += 1000U;
            LPC_PWM1->MR1 = 250U * duty[i];
            LPC_PWM1->LER = 2U;
            uart_puts("duty=");
            uart_putu(duty[i]);
            uart_puts(" %\r\n");
            i = (i + 1U) % 5U;
        }
    }
}
