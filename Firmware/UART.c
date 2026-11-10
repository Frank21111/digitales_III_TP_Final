/*******************************************************************************//**
 * @file    UART.c
 * @brief   Driver de la UART0: 9600 baud, 8N1, recepción por interrupción.
 * @details P0.2 (TXD0) va al RX del CP2102 y P0.3 (RXD0) a su TX, con GND común.
 *          Cada carácter son 10 bits (arranque, 8 de datos, parada): ≈ 1 ms a
 *          9600 baud. La recepción arma una línea por interrupción y la entrega
 *          al superlazo al llegar '\n'; la transmisión es bloqueante.
 * @note    ESW.2.1.10
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "LPC17xx.h"
#include "UART.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define UART_DLL            163U        // 25 MHz/(16·163) = 9586 baud (−0,15 %)
#define UART_PRIORIDAD      4U          // La más baja: la PC no exige tiempo real

/*==============================================================================
 * DEFINICIÓN DE VARIABLES GLOBALES
 *============================================================================*/
static volatile char linea[UART_LINEA_MAX + 1U];   // Línea en armado o lista para leer
static uint8_t indice = 0U;                         // Próxima posición; 16 = descartar línea
static volatile bool lineaLista = false;            // ISR → main: línea completa sin leer

/*==============================================================================
 * PROTOTIPOS DE FUNCIONES
 *============================================================================*/
void UART0_IRQHandler(void);

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Configura la UART0 a 9600 baud 8N1 con interrupción de recepción.
 * @details UART0 encendida (PCONP bit 3), PCLK = 25 MHz (PCLKSEL0 bits 7:6 = 00),
 *          P0.2/P0.3 como TXD0/RXD0 (PINSEL0 bits 7:4 = 0101). LCR = 0x83 (8
 *          bits, DLAB = 1) da acceso a DLL = 163 y DLM = 0: 9586 baud, −0,15 %
 *          (tolerable hasta ±2 %). LCR = 0x03: 8N1 y DLAB = 0. FDR = 0x10 anula
 *          el divisor fraccional. FCR = 0x07 habilita y vacía las FIFO. IER bit 0:
 *          interrupción por dato recibido. NVIC prioridad 4.
 * @note    USW.2.1.10.1
 ********************************************************************************/
void UART_Init(void)
{
    LPC_SC->PCONP |= (1U << 3);
    LPC_SC->PCLKSEL0 &= ~(3U << 6);
    LPC_PINCON->PINSEL0 &= ~((3U << 4) | (3U << 6));
    LPC_PINCON->PINSEL0 |= (1U << 4) | (1U << 6);
    LPC_UART0->LCR = (3U << 0) | (1U << 7);
    LPC_UART0->DLL = UART_DLL;
    LPC_UART0->DLM = 0U;
    LPC_UART0->LCR = (3U << 0);
    LPC_UART0->FDR = (1U << 4);
    LPC_UART0->FCR = (1U << 0) | (1U << 1) | (1U << 2);
    LPC_UART0->IER = (1U << 0);
    NVIC_SetPriority(UART0_IRQn, UART_PRIORIDAD);
    NVIC_EnableIRQ(UART0_IRQn);
}

/*******************************************************************************//**
 * @brief   Envía un texto terminado en '\0' (bloqueante).
 * @details Por cada carácter espera THRE (LSR bit 5 = 1: THR vacío; con la FIFO
 *          habilitada, FIFO de transmisión vacía; el carácter anterior puede
 *          seguir saliendo por el registro de desplazamiento) y lo escribe en THR.
 *          Mientras espera, las interrupciones siguen activas.
 * @param   texto   Cadena a enviar.
 * @note    USW.2.1.10.2
 ********************************************************************************/
void UART_EnviarTexto(const char *texto)
{
    while (*texto != '\0') {
        while ((LPC_UART0->LSR & (1U << 5)) == 0U) {
        }
        LPC_UART0->THR = (uint8_t)*texto;
        texto++;
    }
}

/*******************************************************************************//**
 * @brief   Copia la última línea recibida, si hay una, y libera el buffer.
 * @details Con lineaLista en true la ISR no toca el buffer, así que se copia sin
 *          deshabilitar interrupciones; al bajar la bandera la ISR vuelve a
 *          aceptar caracteres.
 * @param   destino Buffer destino; recibe la línea sin '\n' y terminada en '\0'.
 * @param   tam     Tamaño de destino [bytes].
 * @return  true si había una línea completa.
 * @note    USW.2.1.10.3
 ********************************************************************************/
bool UART_LeerLinea(char *destino, uint8_t tam)
{
    uint8_t i = 0U;

    if (!lineaLista || (tam == 0U)) {
        return false;
    }
    while (((i + 1U) < tam) && (linea[i] != '\0')) {
        destino[i] = linea[i];
        i++;
    }
    destino[i] = '\0';
    lineaLista = false;
    return true;
}

/*******************************************************************************//**
 * @brief   Rutina de servicio de interrupción de la UART0 (recepción).
 * @details Mientras LSR bit 0 = 1 (dato disponible) lee RBR; vaciar la FIFO borra
 *          el pedido. '\r' se ignora; '\n' cierra la línea y levanta lineaLista.
 *          Una línea de más de 15 caracteres, que contiene un '\0' (por ejemplo,
 *          el carácter nulo que deja un break o un error de trama) o que llega
 *          con la anterior sin leer, se descarta completa hasta su '\n'
 *          (indice = 16).
 * @note    USW.2.1.10.4
 ********************************************************************************/
void UART0_IRQHandler(void)
{
    char dato;

    while ((LPC_UART0->LSR & (1U << 0)) != 0U) {
        dato = (char)LPC_UART0->RBR;
        if (dato == '\n') {
            if (!lineaLista && (indice <= UART_LINEA_MAX)) {
                linea[indice] = '\0';
                lineaLista = true;
            }
            indice = 0U;
        } else if (lineaLista || (dato == '\0')) {
            indice = UART_LINEA_MAX + 1U;
        } else if ((dato != '\r') && (indice <= UART_LINEA_MAX)) {
            linea[indice] = dato;
            indice++;
        }
    }
}
