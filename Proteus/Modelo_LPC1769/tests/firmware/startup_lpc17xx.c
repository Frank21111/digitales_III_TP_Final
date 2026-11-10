/* Startup mínimo equivalente al cr_startup_lpc175x_6x.c de MCUXpresso:
 * tabla de vectores con handlers débiles, copia de .data, cero en .bss,
 * SystemInit() y main(). Sólo para compilar en Linux las pruebas del modelo. */
#include <stdint.h>

extern uint32_t _etext, _sdata, _edata, _sbss, _ebss, _vStackTop;
extern int main(void);
extern void SystemInit(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_ALIAS;
void HardFault_Handler(void) WEAK_ALIAS;
void MemManage_Handler(void) WEAK_ALIAS;
void BusFault_Handler(void) WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void) WEAK_ALIAS;
void DebugMon_Handler(void) WEAK_ALIAS;
void PendSV_Handler(void) WEAK_ALIAS;
void SysTick_Handler(void) WEAK_ALIAS;
void WDT_IRQHandler(void) WEAK_ALIAS;
void TIMER0_IRQHandler(void) WEAK_ALIAS;
void TIMER1_IRQHandler(void) WEAK_ALIAS;
void TIMER2_IRQHandler(void) WEAK_ALIAS;
void TIMER3_IRQHandler(void) WEAK_ALIAS;
void UART0_IRQHandler(void) WEAK_ALIAS;
void UART1_IRQHandler(void) WEAK_ALIAS;
void UART2_IRQHandler(void) WEAK_ALIAS;
void UART3_IRQHandler(void) WEAK_ALIAS;
void PWM1_IRQHandler(void) WEAK_ALIAS;
void I2C0_IRQHandler(void) WEAK_ALIAS;
void I2C1_IRQHandler(void) WEAK_ALIAS;
void I2C2_IRQHandler(void) WEAK_ALIAS;
void SPI_IRQHandler(void) WEAK_ALIAS;
void SSP0_IRQHandler(void) WEAK_ALIAS;
void SSP1_IRQHandler(void) WEAK_ALIAS;
void PLL0_IRQHandler(void) WEAK_ALIAS;
void RTC_IRQHandler(void) WEAK_ALIAS;
void EINT0_IRQHandler(void) WEAK_ALIAS;
void EINT1_IRQHandler(void) WEAK_ALIAS;
void EINT2_IRQHandler(void) WEAK_ALIAS;
void EINT3_IRQHandler(void) WEAK_ALIAS;
void ADC_IRQHandler(void) WEAK_ALIAS;
void BOD_IRQHandler(void) WEAK_ALIAS;
void USB_IRQHandler(void) WEAK_ALIAS;
void CAN_IRQHandler(void) WEAK_ALIAS;
void DMA_IRQHandler(void) WEAK_ALIAS;
void I2S_IRQHandler(void) WEAK_ALIAS;
void ENET_IRQHandler(void) WEAK_ALIAS;
void RIT_IRQHandler(void) WEAK_ALIAS;
void MCPWM_IRQHandler(void) WEAK_ALIAS;
void QEI_IRQHandler(void) WEAK_ALIAS;
void PLL1_IRQHandler(void) WEAK_ALIAS;
void USBActivity_IRQHandler(void) WEAK_ALIAS;
void CANActivity_IRQHandler(void) WEAK_ALIAS;

__attribute__((section(".isr_vector"), used))
void (*const g_pfnVectors[])(void) = {
    (void (*)(void))&_vStackTop, Reset_Handler, NMI_Handler, HardFault_Handler, MemManage_Handler,
    BusFault_Handler, UsageFault_Handler, 0, 0, 0, 0, SVC_Handler, DebugMon_Handler, 0, PendSV_Handler,
    SysTick_Handler, WDT_IRQHandler, TIMER0_IRQHandler, TIMER1_IRQHandler, TIMER2_IRQHandler,
    TIMER3_IRQHandler, UART0_IRQHandler, UART1_IRQHandler, UART2_IRQHandler, UART3_IRQHandler,
    PWM1_IRQHandler, I2C0_IRQHandler, I2C1_IRQHandler, I2C2_IRQHandler, SPI_IRQHandler, SSP0_IRQHandler,
    SSP1_IRQHandler, PLL0_IRQHandler, RTC_IRQHandler, EINT0_IRQHandler, EINT1_IRQHandler, EINT2_IRQHandler,
    EINT3_IRQHandler, ADC_IRQHandler, BOD_IRQHandler, USB_IRQHandler, CAN_IRQHandler, DMA_IRQHandler,
    I2S_IRQHandler, ENET_IRQHandler, RIT_IRQHandler, MCPWM_IRQHandler, QEI_IRQHandler, PLL1_IRQHandler,
    USBActivity_IRQHandler, CANActivity_IRQHandler,
};

void Reset_Handler(void)
{
    uint32_t *src = &_etext, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    SystemInit();
    main();
    while (1) {}
}

void Default_Handler(void)
{
    while (1) {}
}
