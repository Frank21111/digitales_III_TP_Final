/*******************************************************************************//**
 * @file    Comm.c
 * @brief   Protocolo ASCII de comandos y telemetría sobre la UART0.
 * @details Interpreta las líneas recibidas ("SP=<n>": nivel deseado en mm;
 *          "U=<n>": modo manual con n % de bomba) y arma la telemetría con las
 *          variables de Control.c. Al ser texto legible, también se puede probar
 *          con un terminal serie. Número ↔ texto con código propio (sin printf).
 * @note    ESW.2.1.4
 ********************************************************************************/

/*==============================================================================
 * DIRECTIVAS DE INCLUSIÓN
 *============================================================================*/
#include "Comm.h"
#include "UART.h"
#include "Control.h"

/*==============================================================================
 * DEFINICIÓN DE CONSTANTES
 *============================================================================*/
#define COMM_LINEA_TAM      (UART_LINEA_MAX + 1U)   // Línea recibida más '\0'
#define COMM_TRAMA_TAM      64U                     // Campos numéricos (≤ 51) más '\0'
#define COMM_CIFRAS_MAX     5U                      // Cifras aceptadas en un valor
#define COMM_VALOR_MAX      65535U                  // Máximo representable en uint16_t

/*==============================================================================
 * DEFINICIÓN DE TIPOS DE DATOS
 *============================================================================*/
typedef enum {
    CMD_NINGUNO = 0,                // Línea vacía, desconocida o mal formada
    CMD_SETPOINT,                   // "SP=<n>": nivel deseado [mm]
    CMD_MANUAL                      // "U=<n>": ciclo de trabajo manual [%]
} Comando_t;

/*==============================================================================
 * PROTOTIPOS DE FUNCIONES
 *============================================================================*/
static Comando_t Comm_ParsearComando(const char *linea, uint16_t *valor);
static uint8_t Comm_AgregarCampo(char *trama, uint8_t pos, const char *etiqueta,
                                 uint16_t valor, bool conDecimal);

/*==============================================================================
 * DEFINICIONES DE FUNCIONES
 *============================================================================*/
/*******************************************************************************//**
 * @brief   Atiende el comando recibido por la UART, si hay uno.
 * @details Toma la línea con UART_LeerLinea, la interpreta y la aplica con
 *          Control_SetSetpoint o Control_SetManual. Un comando mal formado o
 *          fuera de rango se ignora: la PC lo nota en la telemetría.
 * @note    USW.2.1.4.1
 ********************************************************************************/
void Comm_ProcesarComandos(void)
{
    char linea[COMM_LINEA_TAM];
    uint16_t valor = 0U;
    Comando_t comando;

    if (!UART_LeerLinea(linea, COMM_LINEA_TAM)) {
        return;
    }
    comando = Comm_ParsearComando(linea, &valor);
    if (comando == CMD_SETPOINT) {
        (void)Control_SetSetpoint(valor);
    } else if (comando == CMD_MANUAL) {
        (void)Control_SetManual(valor);
    }
}

/*******************************************************************************//**
 * @brief   Envía una trama de telemetría con el estado del proceso.
 * @details Ejemplo: "L=105,SP=120,T=34.5,PWM=42,FAN=0,ALM=0,ERR=0,MODO=A\n". Los
 *          campos numéricos se arman con Comm_AgregarCampo (T con un decimal);
 *          MODO, que es un carácter, se envía después junto con el '\n'.
 *          A 9600 baud la trama demora ≈ 55 ms.
 * @note    USW.2.1.4.2
 ********************************************************************************/
void Comm_EnviarTelemetria(void)
{
    const Estado_t *estado = Control_GetEstado();
    char trama[COMM_TRAMA_TAM];
    uint8_t pos;

    pos = Comm_AgregarCampo(trama, 0U, "L=", estado->nivel_mm, false);
    pos = Comm_AgregarCampo(trama, pos, ",SP=", estado->setpoint_mm, false);
    pos = Comm_AgregarCampo(trama, pos, ",T=", estado->temp_dC, true);
    pos = Comm_AgregarCampo(trama, pos, ",PWM=", estado->duty, false);
    pos = Comm_AgregarCampo(trama, pos, ",FAN=", estado->ventilador, false);
    pos = Comm_AgregarCampo(trama, pos, ",ALM=", estado->alarma, false);
    pos = Comm_AgregarCampo(trama, pos, ",ERR=", estado->falla, false);
    trama[pos] = '\0';
    UART_EnviarTexto(trama);
    UART_EnviarTexto((estado->modo == 'M') ? ",MODO=M\n" : ",MODO=A\n");
}

/*******************************************************************************//**
 * @brief   Interpreta una línea de comando.
 * @details Acepta exactamente "SP=" o "U=" seguido de 1 a 5 cifras y nada más.
 *          El número se arma cifra a cifra: n = n·10 + (carácter − '0'). Los
 *          rangos válidos los controla Control.c.
 * @param   linea   Línea recibida, terminada en '\0' y sin '\n'.
 * @param   valor   Destino del número (sólo se escribe si la línea es válida).
 * @return  CMD_SETPOINT, CMD_MANUAL o CMD_NINGUNO.
 * @note    USW.2.1.4.3
 ********************************************************************************/
static Comando_t Comm_ParsearComando(const char *linea, uint16_t *valor)
{
    Comando_t tipo;
    uint32_t numero = 0U;
    uint8_t cifras = 0U;

    if ((linea[0] == 'S') && (linea[1] == 'P') && (linea[2] == '=')) {
        tipo = CMD_SETPOINT;
        linea += 3;
    } else if ((linea[0] == 'U') && (linea[1] == '=')) {
        tipo = CMD_MANUAL;
        linea += 2;
    } else {
        return CMD_NINGUNO;
    }
    while (*linea != '\0') {
        if ((*linea < '0') || (*linea > '9') || (cifras >= COMM_CIFRAS_MAX)) {
            return CMD_NINGUNO;
        }
        numero = (numero * 10U) + (uint32_t)(*linea - '0');
        cifras++;
        linea++;
    }
    if ((cifras == 0U) || (numero > COMM_VALOR_MAX)) {
        return CMD_NINGUNO;
    }
    *valor = (uint16_t)numero;
    return tipo;
}

/*******************************************************************************//**
 * @brief   Agrega una etiqueta y un número en texto a la trama.
 * @details Copia la etiqueta y convierte el número (utoa propio): obtiene las
 *          cifras de menor a mayor peso con % 10 y / 10 y las copia en orden
 *          inverso. Con conDecimal la primera cifra son las décimas, se inserta
 *          el punto y se garantiza una cifra entera (5 → "0.5", 345 → "34.5").
 * @param   trama       Buffer de la trama.
 * @param   pos         Posición donde se empieza a escribir.
 * @param   etiqueta    Texto previo al número, por ejemplo ",SP=".
 * @param   valor       Número a escribir (con decimal: en décimas).
 * @param   conDecimal  true para escribir valor/10 con un decimal.
 * @return  Posición siguiente al último carácter escrito.
 * @note    USW.2.1.4.4
 ********************************************************************************/
static uint8_t Comm_AgregarCampo(char *trama, uint8_t pos, const char *etiqueta,
                                 uint16_t valor, bool conDecimal)
{
    char cifras[6];
    uint8_t n = 0U;

    while (*etiqueta != '\0') {
        trama[pos++] = *etiqueta++;
    }
    do {
        cifras[n++] = (char)('0' + (valor % 10U));
        valor /= 10U;
        if (conDecimal && (n == 1U)) {
            cifras[n++] = '.';
        }
    } while ((valor != 0U) || (conDecimal && (n < 3U)));
    while (n > 0U) {
        trama[pos++] = cifras[--n];
    }
    return pos;
}
