/* Probador: segundo LPC1769 simulado que estimula al dispositivo bajo prueba.
 * UART0 TX (P0.2) -> RX del DUT. P2.1/P2.2: DAC de temperatura. P2.3/P2.4: DAC de distancia.
 * P2.5: habilita ECHO. P2.6: pulsador (activo en bajo) hacia EINT0 del DUT. */
#include "LPC17xx.h"
static volatile uint32_t ms = 0;
void SysTick_Handler(void) { ms++; }
static void putc_(char c) { while ((LPC_UART0->LSR & (1U << 5)) == 0U) {} LPC_UART0->THR = (uint8_t)c; }
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void pin(uint32_t n, uint32_t v) { if (v) LPC_GPIO2->FIOSET = 1U << n; else LPC_GPIO2->FIOCLR = 1U << n; }
static void temp(uint32_t a, uint32_t b) { pin(1, a); pin(2, b); }
static void dist(uint32_t a, uint32_t b) { pin(3, a); pin(4, b); }
static void esperar(uint32_t t) { while (ms < t) {} }
/* Emulacion del HC-SR04: TRIG entra por P0.4 (CAP2.0), ECHO sale por P0.6 (GPIO). */
static volatile uint32_t eco_us = 1166, eco_on = 1, t_sube = 0, fase = 0;
void TIMER2_IRQHandler(void)
{
    uint32_t ir = LPC_TIM2->IR;
    if (ir & (1U << 4)) {
        uint32_t cr = LPC_TIM2->CR0;
        LPC_TIM2->IR = (1U << 4);
        if (LPC_GPIO0->FIOPIN & (1U << 4)) t_sube = cr;
        else if ((cr - t_sube) >= 10U && eco_on && fase == 0) { LPC_TIM2->MR0 = cr + 250U; fase = 1; LPC_TIM2->MCR = 1U; }
    }
    if (ir & 1U) {
        LPC_TIM2->IR = 1U;
        if (fase == 1) { LPC_GPIO0->FIOSET = (1U << 6); LPC_TIM2->MR0 += eco_us; fase = 2; }
        else if (fase == 2) { LPC_GPIO0->FIOCLR = (1U << 6); fase = 0; LPC_TIM2->MCR = 0U; }
    }
}
int main(void)
{
    LPC_SC->PCONP |= (1U << 3);
    LPC_SC->PCLKSEL0 &= ~(3U << 6);
    LPC_PINCON->PINSEL0 = (LPC_PINCON->PINSEL0 & ~((3U << 4) | (3U << 6))) | (1U << 4) | (1U << 6);
    LPC_UART0->LCR = 0x83; LPC_UART0->DLL = 163; LPC_UART0->DLM = 0; LPC_UART0->LCR = 0x03; LPC_UART0->FCR = 0x07;
    LPC_PINCON->PINSEL4 = 0;
    temp(0, 0); dist(1, 1); pin(5, 1); pin(6, 1);
    LPC_GPIO2->FIODIR |= 0x7EU;
    LPC_GPIO0->FIOCLR = (1U << 6); LPC_GPIO0->FIODIR |= (1U << 6);
    LPC_PINCON->PINSEL0 |= (3U << 8);
    LPC_SC->PCONP |= (1U << 22); LPC_SC->PCLKSEL1 &= ~(3U << 12);
    LPC_TIM2->PR = 24U; LPC_TIM2->TCR = 2U; LPC_TIM2->CCR = 7U; LPC_TIM2->TCR = 1U;
    NVIC_EnableIRQ(TIMER2_IRQn);
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);
#if ESC == 2            /* eco de UART */
    esperar(300); puts_("hola\r"); esperar(600); puts_("x\n");
#elif ESC == 3          /* solo sonar: 200 mm y a los 450 ms 85 mm */
    esperar(450); eco_us = 496;
#elif ESC == 6          /* pulsador EINT0: 3 pulsaciones */
    for (uint32_t i = 0; i < 3; i++) { esperar(200 + 200 * i); pin(6, 0); esperar(250 + 200 * i); pin(6, 1); }
#elif ESC == 8          /* barrido de los DAC para calibrar: cada combinación 600 ms */
    for (uint32_t i = 0; i < 4; i++) { temp(i & 1, i >> 1); dist(i & 1, i >> 1); esperar(600 * (i + 1)); }
#elif ESC == 9          /* protocolo del TPI (sección 6 del README) */
    esperar(2000); puts_("SP=120\n");
    esperar(4000); eco_us = 641; dist(1, 0);              /* 110 mm */
    esperar(6000); temp(1, 1);              /* 46 °C */
    esperar(8500); temp(1, 0);              /* 39 °C */
    esperar(11000); eco_us = 233; dist(0, 0);             /* 40 mm */
    esperar(13000); eco_on = 0; pin(5, 0);              /* ECHO desconectado */
    esperar(15000); puts_("TON=35\n");
    esperar(16000); puts_("TOFF=30\n");
    esperar(17000); puts_("TON=35\n");
    esperar(19000); eco_on = 1; pin(5, 1); puts_("TON=90\n"); puts_("TOFF=20\n"); puts_("SP=500\n"); puts_("basura\n");
    esperar(20000); eco_us = 1166; dist(1, 1); puts_("U=40\n");
    esperar(21500); puts_("SP=0\n");
#endif
    while (1) {}
}
