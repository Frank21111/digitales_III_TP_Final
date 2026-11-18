# Simulación del TPI en Proteus con un modelo propio del LPC1769

Proteus no trae modelo de simulación del LPC1769 (ni de ningún LPC17xx). Esta carpeta contiene uno hecho para el TPI: una DLL de modelo VSM con un intérprete del Cortex-M3 y los periféricos del LPC1769 que usa la cátedra. El modelo ejecuta **el mismo `.axf` que genera MCUXpresso**, sin cambiar el firmware.

**Estado (2026-11-10):** probado sólo en Linux. El núcleo se comparó contra Unicorn: 24 000 bloques de instrucciones sin diferencias. El firmware del TPI y siete firmwares de prueba corren completos en un host que imita a Proteus. Tres revisiones independientes del código (núcleo, periféricos y adaptador) señalaron 34 defectos y riesgos: 29 están corregidos y los otros figuran en la sección 8. **Todavía no se ejecutó desde la interfaz de Proteus.** La sección 4 es la validación que falta, en orden; si algo falla, la ventana de registro del modelo dice qué pasó (sección 7).

**Validación en Windows (2026-11-16):** las DLL de `bin/x86` se cargaron en el simulador de Proteus 8.13 (`PROSPICE.EXE`, por consola, con netlists escritos a mano). Pasaron las siete pruebas de la sección 4 y los nueve pasos de la sección 6, con el HC-SR04 de esta carpeta, el paso analógico por defecto y las salidas medidas en los pines. Esa validación encontró y corrigió dos defectos: Proteus no autoriza dos DLL propias en la misma simulación (ahora `LPC1769.DLL` sirve a los dos modelos) y la CPU no arrancaba con un pin analógico conectado. Quedó sin validar: la interfaz gráfica, las etapas de potencia y el eco a través del divisor 1 kΩ / 2 kΩ (en los netlists hechos a mano una entrada digital no ve una red con resistencias analógicas).

| Carpeta / archivo | Contenido |
|---|---|
| `Modelo_LPC1769/bin/x64/LPC1769.DLL` | Modelo del LPC1769 y del HC-SR04 para Proteus de 64 bits |
| `Modelo_LPC1769/bin/x86/LPC1769.DLL` | Lo mismo para Proteus de 32 bits. Proteus 8.13 también es de 32 bits: usar esta |
| `Modelo_LPC1769/bin/*/HCSR04.DLL` | El HC-SR04 solo, para circuitos sin el LPC1769. No se puede combinar con `LPC1769.DLL` (ver 7.2) |
| `Modelo_LPC1769/pines_J6.csv` | Los 54 pines del conector J6 para crear el componente |
| `Pruebas/` | Siete firmwares de prueba (fuente `.c`, `.axf` y `.hex` listos) y el firmware del TPI compilado |
| `Modelo_LPC1769/src/` | Código del modelo (núcleo, periféricos, adaptador VSM) |
| `Modelo_LPC1769/tests/` | Pruebas de Linux: diferencial contra Unicorn, SoC, host que imita a Proteus |
| `Modelo_LPC1769/tests/proteus/` | Banco para Windows: corre las pruebas de las secciones 4 y 6 dentro del simulador de Proteus (`PROSPICE.EXE`), sin la interfaz |

---

## 1. Instalación

1. Ver si el Proteus es de 64 bits: en el Administrador de tareas, `PDS.EXE` sin «(32 bits)». Proteus 8.13 es de 32 bits.
2. Copiar `bin/x86/LPC1769.DLL` (o la de `bin/x64/` si el Proteus es de 64 bits) a la carpeta de modelos de Proteus. Para saber cuál es: **System → System Settings → Simulation Model and Module Folders**. Suele ser `C:\ProgramData\Labcenter Electronics\Proteus 8 Professional\MODELS` o `C:\Program Files (x86)\Labcenter Electronics\Proteus 8 Professional\DATA\MODELS`.
3. Reiniciar Proteus.

La DLL no necesita nada más: sólo usa bibliotecas de Windows 10/11.

## 2. Crear el componente LPCXPRESSO-1769

Se dibuja una vez y queda en una biblioteca del usuario. El mismo símbolo sirve para el esquemático del informe.

1. En el esquemático, dibujar un rectángulo con **2D Graphics Box**.
2. Con **Device Pins Mode**, poner los 54 pines de `pines_J6.csv`: pines 1 a 27 a la izquierda y 28 a 54 a la derecha, como en la placa. Para cada pin:
   - **Name**: la columna `nombre_en_Proteus` (`P0.4`, `P2.0`, `GND`, `VDD`…). El modelo reconoce los pines por este nombre; también acepta `P0[4]` y `P0_4`.
   - **Number**: la columna `pin_J6`.
   - **Electrical Type**: `IO` para los `P*.*`, `Power` para `GND` y `VDD`, `Input` para `RESET_N` y `Passive` para el resto.
3. Seleccionar todo y usar **Library → Make Device**:
   - Device Name `LPCXPRESSO-1769`, Reference Prefix `U`.
   - En **Component Properties & Definitions** cargar estas propiedades:

| Propiedad | Valor por defecto | Para qué sirve |
|---|---|---|
| `PRIMITIVE` | `MIXED,LPC1769` | Tipo de modelo. Si Proteus lo rechaza, usar `DIGITAL,LPC1769` (ver 7.2) |
| `MODDLL` | `LPC1769.DLL` | DLL del modelo |
| `PROGRAM` | (vacío) | Ruta al `.axf`, `.elf` o `.hex` |
| `XTAL` | `12MHz` | Cristal de la LPCXpresso; la CPU llega a 100 MHz con el `SystemInit` estándar |
| `VREF` | `3.3` | Tensión de referencia del ADC [V] |
| `QUANTUM` | `10u` | Cada cuánto se sincroniza la CPU con Proteus. Las salidas salen con este retardo fijo |
| `POPUP` | `1` | Abre la ventana de registro del modelo |
| `BREAKONFAULT` | `1` | Pausa la simulación si el firmware cae en HardFault |
| `ADC0` … `ADC7` | `0` | Tensión fija [V] de cada canal, sólo en modo `DIGITAL` |
| `ASTEP` | `1m` | Paso máximo del simulador analógico mientras hay pines analógicos: la tensión que lee el ADC tiene a lo sumo esta antigüedad |
| `AGMIN` | `1n` | Conductancia a masa de cada pin analógico, para que un AD0.x sin conectar no deje un nodo flotante. `0` la desactiva |

   - Guardar en una biblioteca nueva, por ejemplo `TPI`.

## 3. Crear el componente HCSR04

Proteus tampoco trae el HC-SR04. Su modelo está dentro de `LPC1769.DLL`, que crea un HC-SR04 cuando el nombre del dispositivo (Device Name) contiene `SR04`: responde a cada pulso de TRIG de al menos 10 µs con un ECHO de 2·d / 343 m/s, 250 µs después. Si d supera 4 m, devuelve un ECHO de 38 ms, como el módulo real sin obstáculo.

- Pines: `TRIG` (Input), `ECHO` (Output), `DIST` (Input), `VCC` (Power), `GND` (Power).
- Propiedades:

| Propiedad | Valor | Para qué sirve |
|---|---|---|
| `PRIMITIVE` | `MIXED,HCSR04` | (o `DIGITAL,HCSR04`) |
| `MODDLL` | `LPC1769.DLL` | La misma DLL del microcontrolador; el Device Name tiene que contener `SR04` (por ejemplo `HCSR04`) |
| `MMPERVOLT` | `100` | La tensión en `DIST` fija la distancia: 1 V = 100 mm |
| `DISTANCE` | `200` | Distancia en mm, sin unidad, si `DIST` no está conectado o el modelo es `DIGITAL` |
| `DELAY` | `250u` | Demora entre el fin de TRIG y el inicio de ECHO |
| `POPUP` | `0` | `1` muestra cada disparo en una ventana |

Para mover la distancia durante la simulación, conectar `DIST` al cursor de un `POT-HG` de 10 kΩ entre +5 V y GND: 0–5 V equivale a 0–500 mm.

## 4. Validación por etapas

Cada prueba agrega un periférico. Hay que pasar a la siguiente sólo si la anterior funcionó. Los `.axf` están en `Pruebas/`. Esperar a que la ventana «LPC1769 - registro del modelo» muestre `Programa cargado` y `CCLK = 100.000 MHz`.

| # | Firmware | Circuito mínimo | Resultado esperado |
|---|---|---|---|
| 1 | `01_blinky.axf` | Osciloscopio o LED con 330 Ω en J6-24 | Onda cuadrada de 1 Hz (500 ms arriba, 500 ms abajo) |
| 2 | `02_uart_eco.axf` | Virtual Terminal: RXD a J6-21, TXD a J6-22, 9600 8N1 | Muestra `LPC1769 en Proteus: UART0 OK`, devuelve lo que se tipea y `lineas=n` con Enter |
| 3 | `03_captura.axf` | HCSR04: TRIG a J6-39, ECHO a J6-38; `DISTANCE=200`; Virtual Terminal | `ancho=1166 us d=200 mm` cada 100 ms |
| 4 | `04_pwm.axf` | Osciloscopio en J6-42; Virtual Terminal | Período de 1 ms; ancho alto de 0; 0,25; 0,5; 0,75 y 1 ms, cambiando cada segundo |
| 5 | `05_adc.axf` | Fuente DC de 0,25 V (o el LM35) en J6-15; Virtual Terminal | `raw=310 mV=249`; con más de 1,65 V se enciende J6-24 |
| 6 | `06_eint.axf` | Pulsador (`BUTTON`) de J6-51 a GND; Virtual Terminal | Cada pulsación imprime `EINT0 n` y cambia J6-24 |
| 7 | `07_adc_mat.axf` | Fuente DC de 0,25 V en J6-15; Virtual Terminal | `raw=310` diez veces por segundo: el ADC convierte en cada flanco de MAT0.1 y avisa por interrupción |

Los mismos resultados se obtuvieron en Linux con el host de prueba (`tests/run_axf.cpp`).

**Qué reportar si una prueba falla:** captura de la ventana de registro del modelo, del Simulation Log de Proteus y versión de Proteus.

## 5. Circuito del TPI en Proteus

Usar el firmware propio compilado en MCUXpresso: `PROGRAM` = `<workspace>\<proyecto>\Debug\<proyecto>.axf`. También sirve `Pruebas/tpi.axf`, compilado de la carpeta `Firmware/` sin cambios.

| Bloque | Componentes de Proteus (Pick Devices) | Conexión |
|---|---|---|
| Microcontrolador | `LPCXPRESSO-1769` (sección 2) | J6-28 a +3V3, J6-1 y J6-54 a GND |
| Nivel (SSE.6.1) | `HCSR04` (sección 3), `RES` 1 kΩ y 2 kΩ, `POT-HG` 10 kΩ | TRIG a J6-39. ECHO → 1 kΩ → nodo → 2 kΩ → GND; nodo a J6-38. Cursor del pote a DIST |
| Temperatura (SSE.6.2) | `LM35`, `RES` 1 kΩ y 220 kΩ, `CAP` 1 µF | Salida → 1 kΩ → J6-15; 1 µF y 220 kΩ de J6-15 a GND |
| Bomba (SSE.4.1 + SSE.5.1) | `PC817`, `IRF630`, `RES` 390 Ω, 100 Ω, 10 kΩ, `1N5819`, `MOTOR-DC` | J6-42 → 390 Ω → ánodo del PC817; cátodo a GND. Colector a +12V; emisor → 100 Ω → gate; 10 kΩ gate–source. Source a GND_PWR; drain → motor → +5V_PWR; 1N5819 en antiparalelo con el motor |
| Ventilador (SSE.4.2 + SSE.5.2) | `PC817`, `IRF630`, mismas resistencias, `1N4007`, `MOTOR-DC` | Igual que la bomba desde J6-44; motor a +12V |
| Alarma (SSE.5.3) | 2 × `2N2222A`, `LED-RED`, `BUZZER`, `RES` 2,2 kΩ, 820 Ω, 270 Ω, 2 × 4,7 kΩ, `1N4148` | LED: J6-24 → 2,2 kΩ → base, 4,7 kΩ base–emisor, colector → LED + 270 Ω → +5V_CTRL. Buzzer: J6-45 → 820 Ω → base, 4,7 kΩ, colector → buzzer (con 1N4148) → +5V_CTRL |
| PC (SSE.7.1 + SSE.3.1) | `VIRTUAL TERMINAL` y/o `COMPIM` | J6-21 a RXD; J6-22 a TXD; 9600 8N1 |
| Alimentación | Terminales `POWER` con +3V3, +5V_CTRL, +5V_PWR, +12V (fuentes `DC` o `BATTERY`) | En la simulación, el LM2596 se reemplaza por una fuente de 5 V |

Detalles de la simulación:
- **Dos masas.** GND_PWR se dibuja con un terminal `GROUND` renombrado `GND_PWR`. Para que SPICE tenga camino de continua, se une a GND con una resistencia de 100 MΩ; en el informe se aclara que es un artificio de simulación.
- **`app_nivel.py` con la simulación.** Instalar com0com y crear el par COM5 ↔ COM6. `COMPIM` usa COM5 a 9600 y la aplicación abre COM6.
- **Nombres de los componentes.** Si `PC817`, `IRF630` o `2N2222A` no aparecen en Pick Devices, usar `OPTOCOUPLER-NPN`, `IRF540` y `2N2222`. No cambia el funcionamiento.

## 6. Protocolo de prueba del TPI

Valores obtenidos en Linux con el firmware de `Firmware/` sin modificar:

| Paso | Acción | Esperado |
|---|---|---|
| 1 | Arrancar; pote del HC-SR04 en 2,0 V (200 mm); LM35 en 25 °C | Telemetría cada 500 ms: `L=20,SP=0,T=24.9,TON=45,TOFF=40,PWM=0,FAN=0,ALM=0,ERR=0,MODO=A` |
| 2 | Osciloscopio en J6-39 | Pulso de TRIG de 11–12 µs cada 100 ms. Llega a 11 µs porque el bucle de espera arranca en cualquier punto de la cuenta de 1 µs del TIMER2; sigue por encima de los 10 µs que pide el sensor |
| 3 | Enviar `SP=120` | `SP=120` y `PWM=100`: el error de 100 mm satura el PI; 1 kHz en J6-42 con duty 100 % |
| 4 | Pote en 1,1 V (110 mm, nivel 110 mm) | `L=110` y `PWM=15` (1,51 %/mm × 10 mm); el duty sube lento por la integral |
| 5 | LM35 a 46 °C | `T=45.7,FAN=1,ALM=1` en menos de 1 s (el divisor 1 kΩ / 220 kΩ del filtro baja la lectura); motor del ventilador, LED y buzzer encendidos |
| 6 | LM35 a 39 °C | `FAN=0,ALM=0` (histéresis 45/40 °C) |
| 7 | Pote en 0,4 V (40 mm, nivel 180 mm) | `L=180` y `PWM=0` (corte de seguridad) |
| 8 | Desconectar ECHO | `ERR=1` y `PWM=0` después de 3 disparos sin eco |
| 9 | LM35 a 39 °C; enviar `TON=35`, después `TOFF=30` y `TON=35` | El primer `TON=35` se ignora porque no supera a TOFF=40. Con 35/30 la trama muestra `TON=35,TOFF=30` y en menos de 1 s `FAN=1,ALM=1`. Valores fuera de 30–80 se ignoran |

## 7. Diagnóstico

### 7.1 La ventana de registro

La ventana «LPC1769 - registro del modelo» se abre desde **Debug** con la simulación en pausa. Informa la carga del programa, cada cambio de CCLK y los errores típicos del firmware:
- acceso a un periférico con su bit de PCONP en 0;
- ADC con un canal cuyo pin no está configurado como AD0.x en PINSEL;
- reloj del ADC por encima de 13 MHz;
- overrun de la UART;
- periféricos no modelados (I2C, SPI, DAC, RIT, RTC, WDT…);
- HardFault con los registros y la dirección que falló.

### 7.2 Problemas posibles

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `mixed model HCSR04.DLL failed to authorize` (o `LPC1769.DLL`) y la simulación no arranca | Proteus 8.13 no autoriza una segunda DLL propia en la misma simulación: `authorize(0)` da falso | Usar `MODDLL=LPC1769.DLL` también en el HCSR04 (sección 3) |
| `External model DLL "LPC1769.DLL" not found` (o `HCSR04.DLL`) | Carpeta equivocada o DLL de otra arquitectura | Revisar la carpeta de la sección 1; probar las DLL de `x86` |
| Proteus rechaza `PRIMITIVE=MIXED` | La palabra clave del modelo mixto no es la que suponemos: no hay documentación pública | Usar `DIGITAL,LPC1769`. En Proteus 8.13, con `PRIMITIVE=DIGITAL` se crea igual el modelo mixto: el ADC lee la tensión del pin si está conectado y `ADC0`… `ADC7` si no. Avisar para corregirlo |
| `falta la propiedad PROGRAM` o `no se pudo abrir` | Ruta vacía o relativa | Usar ruta absoluta al `.axf` |
| Las salidas valen 5 V en vez de 3,3 V | La interfaz digital-analógica de Proteus no toma la tensión de VDD | Avisar; mientras tanto no afecta la lógica, sí el cálculo de las corrientes de base y LED |
| Simulación muy lenta | El intérprete corre a ~0,7 veces el tiempo real en una PC media, sin el resto del circuito | Subir `QUANTUM` a `50u` (las salidas salen 50 µs tarde) o simular tramos cortos |
| Pausa con «falla del procesador» | El firmware cayó en HardFault | Leer en la ventana de registro la dirección y el tipo de falla |

## 8. Limitaciones del modelo

- **Ciclos por instrucción.** Cada instrucción cuenta un ciclo, más dos en los saltos tomados. Los tiempos medidos con timers son exactos; los retardos por bucle de software no.
- **Pines analógicos.** En modo `MIXED`, AD0.0–AD0.5 (P0.23–P0.26, P1.30, P1.31) son entradas analógicas y no funcionan como GPIO. AD0.6 y AD0.7 comparten pin con la UART0 y quedan digitales: en ellos el ADC lee `ADC6`/`ADC7`.
- **Periféricos no modelados.** I2C, SSP/SPI, DAC, RIT, RTC, WDT, DMA, USB, CAN, Ethernet, MCPWM y QEI: leen 0 y la ventana de registro lo avisa.
- **Captura y PWM.** No están modelados el modo contador de los timers (CTCR) ni la captura del PWM1.
- **Arranque.** No se modela el bootloader ni el IAP: el programa arranca directo en el vector de reset.
- **Retornos de excepción mal armados.** No se verifican todas las condiciones de consistencia de un retorno de excepción (por ejemplo, volver a modo Thread con otras excepciones activas). Sólo afecta a firmware con errores.
- **Privilegios.** CONTROL.nPRIV se guarda pero no restringe nada: el código sin privilegios puede hacer lo mismo que el privilegiado. Sólo importa con un RTOS que separe privilegios.
- **Latch del PWM1.** Con el contador detenido, los registros de match se cargan directo, sin esperar a LER. El firmware que olvida LER funciona en la simulación y puede fallar en la placa.

## 9. Recompilar el modelo

En Linux:

```
cd Proteus/Modelo_LPC1769
./build.sh        # DLL x64 y x86, firmwares de prueba y pruebas de Linux
./build.sh test   # además corre todas las pruebas
```

Requiere llvm-mingw y el xPack de `arm-none-eabi-gcc` en `~/opt`; las rutas se cambian con las variables `MINGW` y `ARM`. `src/vsm/vsm.hpp` es una copia del encabezado de Labcenter publicada en GitHub (MockbaTheBorg/VSMModel), con el orden de las sobrecargas virtuales ajustado al de MSVC. La compatibilidad binaria con Proteus se verificó comparando las vtables y el código generado bajo los dos ABI (MSVC e Itanium). En Windows sirven el llvm-mingw para Windows (mismas órdenes de `build.sh`, desde Git Bash) y el `arm-none-eabi-gcc` de MCUXpresso; las pruebas de `tests/` también compilan con ese clang como `.exe`.

Para repetir la validación dentro de Proteus, en Windows y desde Git Bash:

```
cd Proteus/Modelo_LPC1769/tests/proteus
./correr.sh       # usa bin/x86; imprime PASA o FALLA por prueba y el total
```

Usa `PROSPICE.EXE` de la instalación de Proteus y el `arm-none-eabi-gcc` de MCUXpresso (variables `PROSPICE` y `TOOL`). Un segundo LPC1769 simulado (`probador.c`) envía los comandos y cambia la temperatura y la distancia.
