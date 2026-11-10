/* Prueba 3: TIMER2 con captura en ambos flancos (CAP2.0 = P0.4, J6-38) y disparo en P0.5 (J6-39).
 * Cada 100 ms genera un pulso de 12 us en P0.5 y mide el ancho del pulso que vuelve a P0.4.
 * Con el modelo HC-SR04 (TRIG a J6-39, ECHO a J6-38) informa el ancho y la distancia.
 * Esperado con DIST = 2,0 V: "ancho=1166 us d=200 mm". */
#include "uart_min.h"

static volatile uint32_t subida = 0, ancho = 0, ms = 0;
static volatile uint8_t nuevo = 0;

void SysTick_Handler(void)
{
    ms++;
}

void TIMER2_IRQHandler(void)
{
    uint32_t cr = LPC_TIM2->CR0;
    LPC_TIM2->IR = (1U << 4);
    if (LPC_GPIO0->FIOPIN & (1U << 4)) {
        subida = cr;
    } else {
        ancho = cr - subida;
        nuevo = 1;
    }
}

int main(void)
{
    uint32_t proximo = 100U;
    SystemCoreClockUpdate();
    uart_init();
    LPC_PINCON->PINSEL0 &= ~(3U << 10);
    LPC_GPIO0->FIODIR |= (1U << 5);
    LPC_PINCON->PINSEL0 |= (3U << 8);
    LPC_SC->PCONP |= (1U << 22);
    LPC_SC->PCLKSEL1 &= ~(3U << 12);
    LPC_TIM2->PR = 24U;
    LPC_TIM2->TCR = 2U;
    LPC_TIM2->CCR = 7U;
    LPC_TIM2->TCR = 1U;
    NVIC_EnableIRQ(TIMER2_IRQn);
    SysTick_Config(SystemCoreClock / 1000U);
    uart_puts("Prueba de captura TIMER2\r\n");
    while (1) {
        if (ms >= proximo) {
            uint32_t t0;
            proximo += 100U;
            LPC_GPIO0->FIOSET = (1U << 5);
            t0 = LPC_TIM2->TC;
            while ((LPC_TIM2->TC - t0) < 12U) {
            }
            LPC_GPIO0->FIOCLR = (1U << 5);
        }
        if (nuevo) {
            uint32_t a = ancho;
            nuevo = 0;
            uart_puts("ancho=");
            uart_putu(a);
            uart_puts(" us d=");
            uart_putu((a * 343U + 1000U) / 2000U);
            uart_puts(" mm\r\n");
        }
    }
}
