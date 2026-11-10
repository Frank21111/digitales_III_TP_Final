/* Prueba 2: UART0 a 9600 8N1. Envía un saludo y devuelve cada carácter recibido;
 * al recibir Enter informa cuántas líneas lleva.
 * Esperado: en el Virtual Terminal aparece el saludo y el eco de lo que se tipea. */
#include "uart_min.h"

int main(void)
{
    uint32_t lineas = 0U;
    uart_init();
    uart_puts("LPC1769 en Proteus: UART0 OK\r\n");
    while (1) {
        if (LPC_UART0->LSR & 1U) {
            char c = (char)LPC_UART0->RBR;
            uart_putc(c);
            if (c == '\r' || c == '\n') {
                lineas++;
                uart_puts("\r\nlineas=");
                uart_putu(lineas);
                uart_puts("\r\n");
            }
        }
    }
}
