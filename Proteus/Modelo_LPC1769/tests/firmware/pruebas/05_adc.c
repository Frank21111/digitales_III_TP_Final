/* Prueba 5: ADC en AD0.0 (P0.23, J6-15). Convierte cada 200 ms por software, informa la cuenta y los mV,
 * y enciende P0.22 (J6-24) si la entrada supera 1,65 V.
 * Esperado con 0,25 V en J6-15: "raw=310 mV=249". */
#include "uart_min.h"

static volatile uint32_t ms = 0;

void SysTick_Handler(void)
{
    ms++;
}

int main(void)
{
    uint32_t proximo = 200U;
    SystemCoreClockUpdate();
    uart_init();
    LPC_PINCON->PINSEL1 &= ~((3U << 14) | (3U << 12));
    LPC_PINCON->PINSEL1 |= (1U << 14);
    LPC_PINCON->PINMODE1 &= ~(3U << 14);
    LPC_PINCON->PINMODE1 |= (2U << 14);
    LPC_GPIO0->FIODIR |= (1U << 22);
    LPC_SC->PCONP |= (1U << 12);
    LPC_ADC->ADCR = (1U << 0) | (1U << 8) | (1U << 21);
    SysTick_Config(SystemCoreClock / 1000U);
    while (1) {
        if (ms >= proximo) {
            uint32_t dr, raw, mv;
            proximo += 200U;
            LPC_ADC->ADCR |= (1U << 24);
            do {
                dr = LPC_ADC->ADDR0;
            } while ((dr & (1U << 31)) == 0U);
            LPC_ADC->ADCR &= ~(7U << 24);
            raw = (dr >> 4) & 0xFFFU;
            mv = raw * 3300U / 4095U;
            if (mv > 1650U) LPC_GPIO0->FIOSET = (1U << 22);
            else LPC_GPIO0->FIOCLR = (1U << 22);
            uart_puts("raw=");
            uart_putu(raw);
            uart_puts(" mV=");
            uart_putu(mv);
            uart_puts("\r\n");
        }
    }
}
