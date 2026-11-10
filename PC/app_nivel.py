#!/usr/bin/env python3
"""
@file     app_nivel.py
@brief    Aplicación de PC del sistema de control automático de nivel de agua (TPI ED III, G7).
@details  Se comunica con el LPC1769 por el conversor USB-UART CP2102 (9600 8N1): muestra la
          telemetría, grafica los últimos 120 s, envía el setpoint (modo AUTO) o un PWM fijo
          (modo MANUAL) y registra un CSV que procesa identificar.py. Tramas ASCII con fin LF:
            MCU -> PC cada 500 ms: L=105,SP=120,T=34.5,PWM=42,FAN=0,ALM=0,ERR=0,MODO=A
            PC -> MCU: SP=<0|30..160> (SP=0 apaga la bomba) y U=<0..100>
          Sin hilos: el puerto se lee sin bloquear cada 100 ms con after() de Tkinter.
          Uso: pip install -r requirements.txt   y luego   python app_nivel.py
@note     SSE.3.1
"""
import os
import re
import time
import tkinter as tk
from collections import deque
from contextlib import suppress
from datetime import datetime
from tkinter import ttk

import serial
from serial.tools import list_ports

BAUDIOS, PERIODO_MS, VENTANA_S, SILENCIO_S, MAX_TRAMA = 9600, 100, 120, 2.0, 200
PERIODO_TRAMA_S = 0.5  # el firmware envía una trama cada 500 ms (Tick.c, TICKS_TELEMETRIA)
CARPETA = os.path.dirname(os.path.abspath(__file__))
CABECERA_CSV = "t_s,L_mm,SP_mm,T_C,PWM_pct,FAN,ALM,ERR,MODO"
TRAMA = re.compile(r"L=([0-9]{1,3}),SP=([0-9]{1,3}),T=([0-9]{1,3}\.[0-9]),PWM=([0-9]{1,3}),"
                   r"FAN=([01]),ALM=([01]),ERR=([01]),MODO=([AM])")
MENSAJES = {"SP": "Setpoint inválido: ingrese 0 (detener) o un entero de 30 a 160 mm.",
            "U": "PWM manual inválido: ingrese un entero de 0 a 100 %."}
ESTADO = (("L", "Nivel [mm]", str), ("SP", "Setpoint [mm]", str),
          ("T", "Temperatura [°C]", lambda v: ("%.1f" % v).replace(".", ",")),
          ("PWM", "Bomba [% PWM]", str), ("FAN", "Ventilador", lambda v: "ON" if v else "OFF"),
          ("ALM", "Alarma", lambda v: "ON" if v else "OFF"),
          ("ERR", "Sensor", lambda v: "FALLA" if v else "OK"),
          ("MODO", "Modo", lambda v: "AUTO" if v == "A" else "MANUAL"))
GRAFICOS = (("Nivel y setpoint (últimos 120 s)", 250, 200, 50,
             (("L", "#1565c0", None, "Nivel [mm]"), ("SP", "#e65100", (6, 4), "Setpoint [mm]"))),
            ("PWM de la bomba [%]", 140, 100, 25, (("PWM", "#2e7d32", None, "PWM [%]"),)))
ROJO, GRILLA = "#c62828", "#dddddd"
FUENTE_EJE, NEGRITA = ("Helvetica", 8), ("Helvetica", 12, "bold")


def parse_telemetria(linea):
    """Convierte una trama MCU -> PC (str) en dict; None si está incompleta, mal formada o
    trae un valor que el firmware no puede enviar."""
    m = TRAMA.fullmatch(linea.strip())
    if m is None:
        return None
    v = m.groups()
    d = {"L": int(v[0]), "SP": int(v[1]), "T": float(v[2]), "PWM": int(v[3]),
         "FAN": int(v[4]), "ALM": int(v[5]), "ERR": int(v[6]), "MODO": v[7]}
    if (d["L"] > 220 or not (d["SP"] == 0 or 30 <= d["SP"] <= 160)
            or d["T"] > 330.0 or d["PWM"] > 100):
        return None
    return d


def comando(tipo, valor):
    """Arma un comando PC -> MCU (comando("SP", 120) -> b"SP=120\\n"); ValueError si es inválido."""
    texto = str(valor).strip()
    v = int(texto) if re.fullmatch(r"[0-9]{1,3}", texto) else -1
    if (tipo == "SP" and (v == 0 or 30 <= v <= 160)) or (tipo == "U" and 0 <= v <= 100):
        return ("%s=%d\n" % (tipo, v)).encode("ascii")
    raise ValueError(MENSAJES.get(tipo, "Comando desconocido: %r" % (tipo,)))


def crear_csv(carpeta, ahora):
    """Crea registro_AAAAMMDD_HHMMSS.csv con la cabecera, sin sobrescribir; lo devuelve abierto."""
    nombre = os.path.join(carpeta, ahora.strftime("registro_%Y%m%d_%H%M%S.csv"))
    archivo = open(nombre, "x", encoding="ascii", newline="")
    archivo.write(CABECERA_CSV + "\n")
    return archivo


def fila_csv(t_s, d):
    """Fila del CSV (con fin de línea) en el orden de CABECERA_CSV; t_s en segundos, 1 decimal."""
    return "%.1f,%d,%d,%.1f,%d,%d,%d,%d,%s\n" % (t_s, d["L"], d["SP"], d["T"], d["PWM"],
                                                 d["FAN"], d["ALM"], d["ERR"], d["MODO"])


class Enlace:
    """Puerto serie 9600 8N1 con lectura no bloqueante y separación de tramas. No depende de Tk."""

    def __init__(self):
        self.puerto, self.buf = None, b""

    def abrir(self, nombre):
        """Abre el puerto y descarta lo recibido antes; si falla levanta SerialException."""
        self.cerrar()
        self.puerto = serial.Serial(nombre, BAUDIOS, bytesize=8, parity="N", stopbits=1,
                                    timeout=0, write_timeout=1)
        self.puerto.reset_input_buffer()

    def cerrar(self):
        """Cierra el puerto ignorando errores (el cable USB pudo haberse desconectado)."""
        if self.puerto is not None:
            with suppress(OSError):
                self.puerto.close()
        self.puerto, self.buf = None, b""

    def leer_lineas(self):
        """Devuelve las líneas completas recibidas, sin bloquear; lo incompleto queda en el buffer.
        Si el puerto falla, pyserial levanta SerialException (subclase de OSError)."""
        if self.puerto is None:
            return []
        datos = self.buf + self.puerto.read(max(4096, self.puerto.in_waiting))
        *lineas, resto = datos.split(b"\n")
        self.buf = resto[-MAX_TRAMA:]
        return [x.decode("ascii", "replace").strip() for x in lineas]

    def enviar(self, datos):
        """Escribe un comando ya armado con comando()."""
        self.puerto.write(datos)


class App(tk.Tk):
    """Ventana principal: conexión, paneles Estado y Comandos, gráficos y barra de estado."""

    def __init__(self):
        super().__init__()
        self.title("Control de nivel — G7")
        self.enlace, self.historia, self.csv, self.t0_csv = Enlace(), deque(maxlen=600), None, 0.0
        self.t_dato, self.recibiendo = 0.0, None
        self.var_puerto, self.var_csv = tk.StringVar(), tk.BooleanVar()
        self._armar()
        self._listar_puertos()
        self.protocol("WM_DELETE_WINDOW", self._salir)
        self.after(PERIODO_MS, self._sondear)

    def _armar(self):
        """Crea los widgets: barra superior, Estado, Comandos, gráficos y barra de estado."""
        sup = ttk.Frame(self, padding=4)
        sup.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.combo = ttk.Combobox(sup, textvariable=self.var_puerto, width=42,
                                  postcommand=self._listar_puertos)
        self.boton = ttk.Button(sup, text="Conectar", command=self._conectar)
        casilla = ttk.Checkbutton(sup, text="Registrar CSV", variable=self.var_csv,
                                  command=self._alternar_csv)
        for widget in (ttk.Label(sup, text="Puerto"), self.combo, self.boton, casilla):
            widget.pack(side="left", padx=4)
        estado = ttk.LabelFrame(self, text="Estado", padding=8)
        estado.grid(row=1, column=0, sticky="new", padx=6)
        self.valores = {}
        for fila, (clave, texto, _) in enumerate(ESTADO):
            ttk.Label(estado, text=texto).grid(row=fila, column=0, sticky="w", pady=2)
            self.valores[clave] = ttk.Label(estado, text="—", width=7, anchor="e", font=NEGRITA)
            self.valores[clave].grid(row=fila, column=1, padx=(16, 0))
        cmd = ttk.LabelFrame(self, text="Comandos", padding=8)
        cmd.grid(row=2, column=0, sticky="new", padx=6)
        for fila, tipo, texto, texto_boton in ((0, "SP", "Setpoint [mm]", "Enviar SP"),
                                               (2, "U", "Manual [%]", "Aplicar U")):
            entrada, boton = ttk.Entry(cmd, width=6), ttk.Button(cmd, text=texto_boton)
            boton.configure(command=lambda t=tipo, e=entrada: self._enviar(t, e.get()))
            entrada.bind("<Return>", lambda evento, b=boton: b.invoke())
            ttk.Label(cmd, text=texto).grid(row=fila, column=0, sticky="w")
            entrada.grid(row=fila, column=1, padx=6)
            boton.grid(row=fila, column=2, sticky="ew")
        ttk.Button(cmd, text="Detener (SP=0)", command=lambda: self._enviar("SP", 0)).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=6)
        self.lienzos = []
        for fila, (titulo, alto, *_) in enumerate(GRAFICOS, start=1):
            marco = ttk.LabelFrame(self, text=titulo, padding=4)
            marco.grid(row=fila, column=1, sticky="nsew", padx=(0, 6), pady=(0, 6))
            lienzo = tk.Canvas(marco, width=520, height=alto, bg="white", highlightthickness=0)
            lienzo.pack(fill="both", expand=True)
            lienzo.bind("<Configure>", lambda evento: self._graficar())
            self.lienzos.append(lienzo)
        self.barra = ttk.Label(self, text="Elija el puerto y presione Conectar.", anchor="w",
                               relief="sunken", padding=(6, 2), width=1)
        self.barra.grid(row=3, column=0, columnspan=2, sticky="ew")
        self.columnconfigure(1, weight=1)
        self.rowconfigure((1, 2), weight=1)

    def _listar_puertos(self):
        """Actualiza la lista de puertos; si no hay uno elegido, prefiere el conversor CP210x."""
        puertos = sorted(list_ports.comports(), key=lambda p: p.device)
        opciones = ["%s — %s" % (p.device, p.description) for p in puertos]
        self.combo["values"] = opciones
        if opciones and not self.var_puerto.get():
            self.var_puerto.set(next((o for o in opciones if "CP210" in o.upper()), opciones[0]))

    def _estado(self, texto, error=False):
        """Muestra un mensaje con la hora en la barra inferior (en rojo si es un error)."""
        self.barra.configure(text=time.strftime("%H:%M:%S  ") + texto,
                             foreground=ROJO if error else "")

    def _conectar(self):
        """Botón Conectar/Desconectar: abre o cierra el puerto elegido."""
        if self.enlace.puerto is not None:
            return self._desconectar("Puerto cerrado.")
        nombre = self.var_puerto.get().split(" — ")[0].strip()
        try:
            self.enlace.abrir(nombre)
        except (OSError, ValueError) as e:
            return self._estado("No se pudo abrir el puerto '%s': %s" % (nombre, e), True)
        self.t_dato, self.recibiendo = time.monotonic(), None
        self.boton.configure(text="Desconectar")
        self._estado("Conectado a %s (9600 8N1). Esperando telemetría..." % nombre)

    def _desconectar(self, mensaje, error=False):
        """Cierra el puerto sin cerrar la aplicación y vacía el panel Estado."""
        self.enlace.cerrar()
        self.boton.configure(text="Conectar")
        for etiqueta in self.valores.values():
            etiqueta.configure(text="—", foreground="")
        self._estado(mensaje, error)

    def _enviar(self, tipo, valor):
        """Valida y envía un comando SP o U; los errores se informan en la barra de estado."""
        if self.enlace.puerto is None:
            return self._estado("Primero conecte el puerto.", True)
        try:
            datos = comando(tipo, valor)
            self.enlace.enviar(datos)
            self._estado("Enviado: " + datos.decode("ascii").strip())
        except ValueError as e:
            self._estado(str(e), True)
        except OSError as e:
            self._desconectar("Se perdió la conexión al enviar: %s" % e, True)

    def _alternar_csv(self):
        """Casilla Registrar CSV: crea registro_AAAAMMDD_HHMMSS.csv o cierra el registro actual."""
        if self.var_csv.get():
            try:
                self.csv, self.t0_csv = crear_csv(CARPETA, datetime.now()), time.monotonic()
                self._estado("Registrando en " + self.csv.name)
            except OSError as e:
                self.var_csv.set(False)
                self._estado("No se pudo crear el CSV: %s" % e, True)
        elif self.csv is not None:
            self.csv.close()
            self._estado("CSV guardado: " + self.csv.name)
            self.csv = None

    def _sondear(self):
        """Cada 100 ms: lee el puerto, procesa las tramas, registra el CSV y vigila el silencio."""
        self.after(PERIODO_MS, self._sondear)
        if self.enlace.puerto is None:
            return
        try:
            lineas = [x for x in self.enlace.leer_lineas() if x]
        except OSError as e:
            return self._desconectar("Conexión perdida (revise el cable USB): %s" % e, True)
        ahora = time.monotonic()
        for k, linea in enumerate(lineas):
            t = ahora - PERIODO_TRAMA_S * (len(lineas) - 1 - k)
            d = parse_telemetria(linea)
            if d is None:
                self._estado("Trama descartada (incompleta o mal formada): " + linea[:60], True)
                continue
            if not self.recibiendo:
                self._estado("Recibiendo telemetría.")
            self.t_dato, self.recibiendo = ahora, True
            self.historia.append((t, d))
            for clave, _, formato in ESTADO:
                color = ROJO if clave in ("ALM", "ERR") and d[clave] else ""
                self.valores[clave].configure(text=formato(d[clave]), foreground=color)
            if self.csv is not None and t >= self.t0_csv:
                self.csv.write(fila_csv(t - self.t0_csv, d))
                self.csv.flush()
        if lineas:
            self._graficar()
        elif self.recibiendo is not False and ahora - self.t_dato > SILENCIO_S:
            self.recibiendo = False
            self._estado("Sin telemetría hace más de 2 s: revise TX/RX cruzados, GND común, "
                         "9600 baudios y que el firmware esté corriendo.", True)

    def _graficar(self):
        """Redibuja cada gráfico: grilla, escalas, leyenda y una polilínea por serie."""
        t_ref = self.historia[-1][0] if self.historia else 0.0
        muestras = [(t - t_ref, d) for t, d in self.historia if t >= t_ref - VENTANA_S]
        for c, (_, _, ymax, paso, series) in zip(self.lienzos, GRAFICOS):
            c.delete("all")
            x0, y0, x1, y1 = 40, 24, c.winfo_width() - 14, c.winfo_height() - 20
            kx, ky = (x1 - x0) / VENTANA_S, (y1 - y0) / ymax
            for v in range(0, ymax + 1, paso):
                c.create_line(x0, y1 - ky * v, x1, y1 - ky * v, fill=GRILLA)
                c.create_text(x0 - 5, y1 - ky * v, text=v, anchor="e", font=FUENTE_EJE)
            for s in range(-VENTANA_S, 1, 30):
                c.create_line(x1 + kx * s, y0, x1 + kx * s, y1, fill=GRILLA)
                c.create_text(x1 + kx * s, y1 + 3, text="%d s" % s, anchor="n", font=FUENTE_EJE)
            for k, (clave, color, trazo, nombre) in enumerate(series):
                xl = x0 + 130 * k
                c.create_line(xl, 11, xl + 26, 11, fill=color, width=2, dash=trazo)
                c.create_text(xl + 32, 11, text=nombre, anchor="w", font=FUENTE_EJE)
                pts = [p for dt, d in muestras
                       for p in (x1 + kx * dt, y1 - ky * min(max(d[clave], 0), ymax))]
                if len(pts) >= 4:
                    c.create_line(*pts, fill=color, width=2, dash=trazo)

    def _salir(self):
        """Cierra el CSV y el puerto antes de cerrar la ventana."""
        if self.csv is not None:
            self.csv.close()
        self.enlace.cerrar()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
