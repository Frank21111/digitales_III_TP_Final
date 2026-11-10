/* Prueba 6: interrupción externa EINT0 en P2.10 (J6-51), por flanco descendente.
 * Cada pulsación cambia P0.22 (J6-24) y se informa por la UART.
 * Esperado: con un pulsador de J6-51 a GND (pull-up interno), cada pulsación imprime "EINT0 n". */
#include "uart_min.h"

static volatile uint32_t cuenta = 0;
static volatile uint8_t nuevo = 0;

void EINT0_IRQHandler(void)
{
    LPC_SC->EXTINT = 1U;
    LPC_GPIO0->FIOPIN ^= (1U << 22);
    cuenta++;
    nuevo = 1;
}

int main(void)
{
    uart_init();
    LPC_PINCON->PINSEL1 &= ~(3U << 12);
    LPC_GPIO0->FIODIR |= (1U << 22);
    LPC_PINCON->PINSEL4 &= ~(3U << 20);
    LPC_PINCON->PINSEL4 |= (1U << 20);
    LPC_SC->EXTMODE |= 1U;
    LPC_SC->EXTPOLAR &= ~1U;
    LPC_SC->EXTINT = 1U;
    NVIC_EnableIRQ(EINT0_IRQn);
    uart_puts("Prueba EINT0\r\n");
    while (1) {
        if (nuevo) {
            nuevo = 0;
            uart_puts("EINT0 ");
            uart_putu(cuenta);
            uart_puts("\r\n");
        }
    }
}
