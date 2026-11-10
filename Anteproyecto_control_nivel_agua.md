# Anteproyecto — Sistema de control de nivel de agua y monitoreo térmico

## 1. Descripción del proyecto

Se propone desarrollar un sistema de control automático de nivel de agua en un recipiente de aproximadamente 2 litros. El usuario podrá configurar desde una PC el nivel deseado; el microcontrolador medirá el nivel actual y regulará una bomba mediante una acción de control PID.

Adicionalmente, se medirá la temperatura de la placa electrónica. Si se supera un umbral configurable, se activará una alarma sonora y visual mediante un buzzer y un LED, además de encender un ventilador para reducir la temperatura.

## 2. Arquitectura de hardware

- **Control:** microcontrolador LPC1769.
- **Adquisición:** sensor ultrasónico HC-SR04 para medir el nivel de agua y sensor de temperatura LM35 para monitorear la placa.
- **Actuación hidráulica:** mini bomba de agua sumergible, controlada mediante PWM y una etapa de potencia con MOSFET.
- **Protección:** optoacoplador PC817 entre la salida de control del microcontrolador y la etapa de potencia de la bomba.
- **Alarma térmica:** LED, buzzer y ventilador Cooler Master Hyper H411R.
- **Comunicación:** interfaz USB-UART para configurar el nivel deseado desde la PC y visualizar las mediciones.
- **Alimentación:** fuentes separadas para el microcontrolador y los actuadores, con aislamiento eléctrico entre ambas etapas donde corresponda.

## 3. Modelo matemático del sistema

Se considera un tanque de sección transversal constante, con entrada de agua mediante una bomba y salida por gravedad a través de un orificio.

**Variables:**

- \(h(t)\): nivel de agua.
- \(u(t)\): señal de control normalizada aplicada a la bomba.
- \(A\): área transversal del tanque.
- \(Q_{in}(t)\), \(Q_{out}(t)\): caudales de entrada y salida.

**Ecuación de balance de volumen:**

\[
A\frac{dh(t)}{dt}=Q_{in}(t)-Q_{out}(t)
\]

**Modelo de salida por gravedad:**

\[
Q_{out}(t)=C_d a_o\sqrt{2gh(t)}
\]

**Modelo aproximado de la bomba:**

\[
Q_{in}(t)=K_pu(t)
\]

**Modelo no lineal resultante:**

\[
\frac{dh(t)}{dt}=
\frac{K_p}{A}u(t)-
\frac{C_da_o\sqrt{2g}}{A}\sqrt{h(t)}
\]

Estos modelos servirán como base para obtener la función de transferencia de la planta mediante linealización alrededor de un punto de operación.

## 4. Diagrama básico

```text
 [PC]  <---->  [USB-UART]  <---->  [LPC1769]

 [Sensor de nivel]  ------->  [LPC1769]  ------->  [PC]

 [LPC1769]  --->  [PC817]  --->  [MOSFET]  --->  [Bomba]  --->  [Tanque]

 [LM35]  ------->  [LPC1769]  ------->  [LED + Buzzer + Ventilador]
```

## 5. Componentes principales

- LPC1769.
- Recipiente transparente de hasta 2 L y reservorio de agua.
- Mini bomba sumergible de 3–5 V.
- Sensor ultrasónico HC-SR04.
- Sensor de temperatura LM35.
- Optoacoplador PC817.
- MOSFET de canal N y diodo de protección para la bomba.
- Ventilador Cooler Master Hyper H411R.
- LED, buzzer y resistencias.
- Adaptador USB-UART TTL de 3,3 V.
- Fuentes de alimentación separadas para control y actuación.
- Mangueras, conexiones y válvula manual para regular la salida de agua.

**Ecuación de interés para la linealización:**

\[
A\frac{d\Delta h(t)}{dt}
=
K_p\Delta u(t)
-
\frac{C_da_o\sqrt{2g}}{2\sqrt{h_0}}\Delta h(t)
\]

Esta es la ecuación de interés para obtener la función de transferencia de la planta. Los parámetros deberán determinarse o medirse experimentalmente.

**Nota:** el Cooler Master Hyper H411R es un conjunto de refrigeración para CPU; para este proyecto se utilizaría su ventilador, verificando la tensión y la conexión de control.
