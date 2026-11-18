"""
@file     test_identificar.py
@brief    Prueba de identificar.py con telemetría sintética de un tanque de primer orden.
@details  Genera el registro que guardaría un terminal serie durante un ensayo escalón (U=40 -> U=45)
          con K y tau conocidos, nivel entero en mm, ruido, líneas dañadas, eco local de los comandos y
          marcas de tiempo del terminal, y verifica que identificar.py recupere K y tau dentro del 2 %.
          Uso: python -m unittest test_identificar.py
@note     SSE.3.1
"""
import os
import random
import re
import subprocess
import sys
import tempfile
import unittest

AQUI = os.path.dirname(os.path.abspath(__file__))
K_MM_POR_PCT, TAU_S = 4.73, 214.0  # plan.md §5: K = 0,473 m por unidad de u, tau = 214 s


def trama(L, pwm, modo="M"):
    return "L=%d,SP=0,T=27.3,TON=45,TOFF=40,PWM=%d,FAN=0,ALM=0,ERR=0,MODO=%s" % (L, pwm, modo)


def registro_sintetico(ruta, danadas=True, marcas=False, semilla=1):
    """Escribe el registro: 60 s en AUTO, 300 s con U=40 estable, escalón a U=45 y 1200 s más."""
    rnd = random.Random(semilla)
    L0 = 100.0
    lineas = ["=~=~=~=~=~=~=~ PuTTY log 2026.11.11 10:00:00 =~=~=~=~=~=~=~", "PWM=0,FAN=0,ALM=0,ERR=0,MODO=A"]
    for _ in range(120):
        lineas.append(trama(round(L0 + rnd.gauss(0, 0.6)), 0, "A"))
    lineas.append("U=40")
    for _ in range(600):
        lineas.append(trama(round(L0 + rnd.gauss(0, 0.6)), 40))
    lineas.append("U=45")
    for k in range(2400):
        t = 0.5 * k
        L = L0 + K_MM_POR_PCT * 5 * (1 - 2.718281828459045 ** (-t / TAU_S))
        pwm = 40 if k == 0 else 45  # la primera trama tras U= puede salir con el PWM anterior
        lineas.append(trama(round(L + rnd.gauss(0, 0.6)), pwm))
    if danadas:
        for idx in rnd.sample(range(130, len(lineas) - 5), 40):
            if not lineas[idx].startswith(("U=", "SP=")):
                lineas[idx] = lineas[idx][: rnd.randrange(3, 40)] + "�"
    if marcas:
        lineas = ["[10:%02d:%02d.%03d] %s" % (i // 120 % 60, i // 2 % 60, 500 * (i % 2), x)
                  for i, x in enumerate(lineas)]
    lineas.append("L=10")  # el terminal se cerró en medio de una trama
    with open(ruta, "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(lineas) + "\n")


def identificar(ruta):
    r = subprocess.run([sys.executable, os.path.join(AQUI, "identificar.py"), ruta, "--lambda", "30"],
                       capture_output=True, text=True, check=True)
    m = re.search(r"Mínimos cuadrados:\s+K = ([0-9.]+) mm/%\s+tau = ([0-9.]+) s", r.stdout)
    return float(m.group(1)), float(m.group(2)), r.stdout


class TestIdentificar(unittest.TestCase):
    def _verificar(self, **kw):
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, "ensayo.txt")
            registro_sintetico(ruta, **kw)
            K, tau, salida = identificar(ruta)
        self.assertIn("PWM 40 % -> 45 %", salida)
        self.assertAlmostEqual(K, K_MM_POR_PCT, delta=0.02 * K_MM_POR_PCT, msg=salida)
        self.assertAlmostEqual(tau, TAU_S, delta=0.02 * TAU_S, msg=salida)
        self.assertIn("#define PID_KP", salida)

    def test_registro_limpio(self):
        self._verificar(danadas=False)

    def test_lineas_danadas_y_eco(self):
        self._verificar(danadas=True)

    def test_marcas_de_tiempo_del_terminal(self):
        self._verificar(danadas=True, marcas=True)

    def test_archivo_sin_tramas(self):
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, "vacio.txt")
            with open(ruta, "w", encoding="utf-8") as f:
                f.write("hola\nmundo\nfin\n")
            r = subprocess.run([sys.executable, os.path.join(AQUI, "identificar.py"), ruta],
                               capture_output=True, text=True)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("ninguna trama", r.stderr)


if __name__ == "__main__":
    unittest.main()
