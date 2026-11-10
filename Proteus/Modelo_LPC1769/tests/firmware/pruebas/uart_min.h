/* UART0 mínima (P0.2 = TXD0 en J6-21, P0.3 = RXD0 en J6-22), 9600 8N1 con PCLK = 25 MHz.
 * Compartida por los firmwares de prueba del modelo de Proteus. */
#ifndef UART_MIN_H
#define UART_MIN_H
#include "LPC17xx.h"

static void uart_init(void)
{
    LPC_SC->PCONP |= (1U << 3);
    LPC_SC->PCLKSEL0 &= ~(3U << 6);
    LPC_PINCON->PINSEL0 &= ~((3U << 4) | (3U << 6));
    LPC_PINCON->PINSEL0 |= (1U << 4) | (1U << 6);
    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 163;
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 0x03;
    LPC_UART0->FCR = 0x07;
}

static void uart_putc(char c)
{
    while ((LPC_UART0->LSR & (1U << 5)) == 0U) {
    }
    LPC_UART0->THR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_putu(uint32_t v)
{
    char b[11];
    int n = 0;
    do { b[n++] = (char)('0' + v % 10U); v /= 10U; } while (v);
    while (n) uart_putc(b[--n]);
}
#endif
