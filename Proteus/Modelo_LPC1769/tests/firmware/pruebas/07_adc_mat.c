/* Prueba 7: ADC disparado por hardware. TIMER0 cambia MAT0.1 cada 50 ms (EMR en modo toggle) y el ADC
 * convierte AD0.0 (P0.23, J6-15) en cada flanco ascendente de MAT0.1, es decir cada 100 ms.
 * La conversión se atiende por interrupción; ADINTEN queda en su valor de reset (bit 8, DONE global).
 * Esperado con 0,25 V en J6-15: "raw=310" diez veces por segundo. */
#include "uart_min.h"

static volatile uint32_t raw = 0;
static volatile uint8_t nuevo = 0;

void ADC_IRQHandler(void)
{
    uint32_t dr = LPC_ADC->ADGDR;
    raw = (dr >> 4) & 0xFFFU;
    nuevo = 1;
}

int main(void)
{
    uart_init();
    LPC_PINCON->PINSEL1 &= ~(3U << 14);
    LPC_PINCON->PINSEL1 |= (1U << 14);
    LPC_SC->PCONP |= (1U << 12) | (1U << 1);
    LPC_SC->PCLKSEL0 &= ~(3U << 2);
    LPC_TIM0->PR = 24U;
    LPC_TIM0->MR1 = 50000U - 1U;
    LPC_TIM0->MCR = (1U << 4);
    LPC_TIM0->EMR = (3U << 6);
    LPC_TIM0->TCR = 2U;
    LPC_TIM0->TCR = 1U;
    LPC_ADC->ADCR = (1U << 0) | (1U << 8) | (1U << 21) | (4U << 24);
    NVIC_EnableIRQ(ADC_IRQn);
    uart_puts("Prueba ADC por MAT0.1\r\n");
    while (1) {
        if (nuevo) {
            nuevo = 0;
            uart_puts("raw=");
            uart_putu(raw);
            uart_puts("\r\n");
        }
    }
}
