"""
@file     identificar.py
@brief    Identificación de la planta de nivel (primer orden) y cálculo del PI a partir del CSV de app_nivel.py.
@details  Ensayo: en modo MANUAL (MODO = M), con el nivel estable, se aplica un escalón de PWM
          (por ejemplo U=40 -> U=45) y se registra hasta que el nivel se estabilice de nuevo
          (al menos 4 tau, unos 15 min). Se ajusta
              dL(t) = K * dU * (1 - exp(-t / tau))
          por el método del 63,2 % y por mínimos cuadrados (búsqueda de tau por sección áurea,
          L0 y K*dU lineales). Con el lambda pedido se calcula la sintonía lambda/SIMC:
              Kc = tau / (K * lambda)   Ti = min(tau, 4 * lambda)   Ki = Kc / Ti
          e imprime las líneas #define para Control.h (PID_KP en %/mm, PID_KI en %/(mm*s)).
          Sólo usa la biblioteca estándar de Python 3.
          Uso: python identificar.py registro_AAAAMMDD_HHMMSS.csv [--lambda 30] [--ventana 30]
@note     SSE.3.1
"""
import argparse
import csv
import math
import sys

CABECERA = ["t_s", "L_mm", "SP_mm", "T_C", "PWM_pct", "FAN", "ALM", "ERR", "MODO"]
NIVEL_MAX_MM = 180
TAU_MIN, TAU_MAX = 1.0, 5000.0


def leer_csv(ruta):
    """Lee el CSV de app_nivel.py y devuelve una lista de dict con t, L, pwm, err y modo."""
    with open(ruta, newline="", encoding="utf-8-sig") as f:
        lector = csv.reader(f)
        cabecera = [c.strip() for c in next(lector, [])]
        if cabecera != CABECERA:
            raise ValueError("Cabecera inesperada: %s (se esperaba %s)" % (",".join(cabecera), ",".join(CABECERA)))
        filas = []
        for c in lector:
            if len(c) != len(CABECERA):
                continue
            try:
                filas.append({"t": float(c[0]), "L": float(c[1]), "pwm": float(c[4]),
                              "err": int(c[7]), "modo": c[8].strip().upper()})
            except ValueError:
                continue
    return filas


def forzada(f):
    """Fila en la que el firmware puso la bomba al 0 % por su cuenta: falla del sensor (ERR = 1) o corte por nivel."""
    return f["pwm"] == 0 and (f["err"] == 1 or f["L"] >= NIVEL_MAX_MM)


def manual_valida(f):
    """Fila en modo M con el PWM que pidió el usuario (no forzada por el firmware)."""
    return f["modo"] == "M" and not forzada(f)


def detectar_escalon(filas, elegido=None, ventana=30.0):
    """Devuelve (i, inicio, fin): fila del escalón de PWM en modo M y límites del tramo manual que lo contiene.
    Sólo cuenta un cambio de PWM entre filas manual_valida con al menos `ventana` s de PWM fijo antes: así no se
    toman como escalón la falla del sensor, el corte de 180 mm ni la primera trama tras U=..., que puede salir
    con MODO = M y el PWM anterior (el duty cambia en el Control_Nivel siguiente)."""
    escalones = []
    for i in range(1, len(filas)):
        if manual_valida(filas[i - 1]) and manual_valida(filas[i]) and filas[i]["pwm"] != filas[i - 1]["pwm"]:
            inicio = i - 1
            while inicio > 0 and manual_valida(filas[inicio - 1]) and filas[inicio - 1]["pwm"] == filas[i - 1]["pwm"]:
                inicio -= 1
            if filas[i - 1]["t"] - filas[inicio]["t"] >= ventana:
                escalones.append((i, inicio))
    if not escalones:
        raise ValueError("No hay ningún cambio de PWM en modo MANUAL (MODO = M) con al menos %g s de PWM fijo antes "
                         "y sin falla del sensor ni corte de %d mm." % (ventana, NIVEL_MAX_MM))
    if elegido is None:
        i, inicio = max(escalones, key=lambda e: abs(filas[e[0]]["pwm"] - filas[e[0] - 1]["pwm"]))
    else:
        i, inicio = escalones[elegido]
    fin = i
    while fin + 1 < len(filas) and manual_valida(filas[fin + 1]) and filas[fin + 1]["pwm"] == filas[i]["pwm"]:
        fin += 1
    return i, inicio, fin


def promedio(valores):
    """Promedio aritmético de una lista no vacía."""
    return sum(valores) / len(valores)


def pendiente(t, L):
    """Pendiente de la recta de mínimos cuadrados de L contra t [mm/s]."""
    tm, Lm = promedio(t), promedio(L)
    stt = sum((x - tm) ** 2 for x in t)
    return 0.0 if stt == 0.0 else sum((x - tm) * (y - Lm) for x, y in zip(t, L)) / stt


def metodo_632(t, L, L0, Lf, media_s=10.0):
    """Instante en que el nivel suavizado (media móvil centrada de +-media_s) cruza L0 + 0,632 (Lf - L0).
    Con ruido puede cruzar varias veces: se promedia el primer y el último cruce (interpolados)."""
    objetivo = L0 + 0.632 * (Lf - L0)
    suave, j0, j1 = [], 0, 0
    for k in range(len(t)):
        while t[j0] < t[k] - media_s:
            j0 += 1
        while j1 + 1 < len(t) and t[j1 + 1] <= t[k] + media_s:
            j1 += 1
        suave.append(promedio(L[j0:j1 + 1]))
    cruces = []
    for k in range(1, len(t)):
        a, b = suave[k - 1] - objetivo, suave[k] - objetivo
        if t[k] > 0.0 and a * b <= 0.0 and a != b:
            cruces.append(t[k - 1] + a / (a - b) * (t[k] - t[k - 1]))
    return 0.5 * (cruces[0] + cruces[-1]) if cruces else float("nan")


def ajuste_lineal(tau, t, L):
    """Para un tau fijo resuelve por mínimos cuadrados L = L0 + A (1 - exp(-t/tau)) (t < 0 -> 0); devuelve (sse, L0, A)."""
    f = [0.0 if ti <= 0.0 else 1.0 - math.exp(-ti / tau) for ti in t]
    n, sf, sff = len(t), sum(f), sum(x * x for x in f)
    sy, sfy = sum(L), sum(x * y for x, y in zip(f, L))
    det = n * sff - sf * sf
    if det <= 0.0:
        return float("inf"), 0.0, 0.0
    A = (n * sfy - sf * sy) / det
    L0 = (sy - A * sf) / n
    sse = sum((y - L0 - A * x) ** 2 for x, y in zip(f, L))
    return sse, L0, A


def minimos_cuadrados(t, L, tau_min=TAU_MIN, tau_max=TAU_MAX):
    """Busca tau por sección áurea sobre log(tau); devuelve (tau, L0, A, rms)."""
    r = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = math.log(tau_min), math.log(tau_max)
    c, d = b - r * (b - a), a + r * (b - a)
    fc, fd = ajuste_lineal(math.exp(c), t, L)[0], ajuste_lineal(math.exp(d), t, L)[0]
    while b - a > 1e-5:
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - r * (b - a)
            fc = ajuste_lineal(math.exp(c), t, L)[0]
        else:
            a, c, fc = c, d, fd
            d = a + r * (b - a)
            fd = ajuste_lineal(math.exp(d), t, L)[0]
    tau = math.exp(0.5 * (a + b))
    sse, L0, A = ajuste_lineal(tau, t, L)
    return tau, L0, A, math.sqrt(sse / len(t))


def sintonia(K, tau, lam):
    """Sintonía lambda/SIMC para G(s) = K/(tau s + 1): devuelve (Kc [%/mm], Ti [s], Ki [%/(mm*s)])."""
    Kc = tau / (K * lam)
    Ti = min(tau, 4.0 * lam)
    return Kc, Ti, Kc / Ti


def main():
    """Lee el CSV, detecta el escalón, identifica K y tau, calcula el PI e imprime los #define."""
    ap = argparse.ArgumentParser(description="Identifica K y tau del tanque desde el CSV de app_nivel.py y calcula el PI.")
    ap.add_argument("csv", help="archivo registro_*.csv generado por app_nivel.py")
    ap.add_argument("--lambda", dest="lam", type=float, default=30.0, help="constante de tiempo deseada del lazo cerrado [s] (30)")
    ap.add_argument("--ventana", type=float, default=30.0, help="segundos promediados antes del escalón y al final (30)")
    ap.add_argument("--escalon", type=int, default=None, help="índice entre los escalones válidos (0 = el primero)")
    a = ap.parse_args()
    if not (math.isfinite(a.lam) and a.lam > 0.0):
        sys.exit("Error: --lambda debe ser un número positivo [s].")
    if not (math.isfinite(a.ventana) and a.ventana > 0.0):
        sys.exit("Error: --ventana debe ser un número positivo [s].")

    try:
        filas = leer_csv(a.csv)
        i, ini, fin = detectar_escalon(filas, a.escalon, a.ventana)
    except (OSError, ValueError, IndexError) as e:
        sys.exit("Error: %s" % e)

    t_esc = 0.5 * (filas[i - 1]["t"] + filas[i]["t"])
    u1, u2 = filas[i - 1]["pwm"], filas[i]["pwm"]
    dU = u2 - u1
    tramo = filas[ini:fin + 1]
    t = [f["t"] - t_esc for f in tramo]
    L = [f["L"] for f in tramo]
    antes = [f["L"] for f in tramo if -a.ventana <= f["t"] - t_esc < 0.0]
    final = [f["L"] for f in tramo if f["t"] - t_esc >= t[-1] - a.ventana]
    if len(antes) < 3 or len(final) < 3:
        sys.exit("Error: faltan muestras antes del escalón o al final del ensayo (revisar --ventana).")
    L0, Lf = promedio(antes), promedio(final)
    duracion = t[-1]

    print("Escalón en t = %.1f s: PWM %g %% -> %g %% (dU = %+g %%), tramo manual de %.0f s antes y %.0f s después."
          % (t_esc, u1, u2, dU, -t[0], duracion))
    print("Nivel antes = %.1f mm, nivel final = %.1f mm (dL = %+.1f mm)." % (L0, Lf, Lf - L0))
    previo = [(ti, Li) for ti, Li in zip(t, L) if -120.0 <= ti < 0.0]
    deriva = 60.0 * pendiente([p[0] for p in previo], [p[1] for p in previo]) if len(previo) >= 3 else 0.0
    if abs(deriva) > 0.5:
        print("Aviso: antes del escalón el nivel no estaba estable (deriva de %+.1f mm/min); repetir esperando a que se "
              "estabilice con el PWM manual fijo." % deriva)
    if fin + 1 < len(filas) and forzada(filas[fin + 1]) and filas[fin + 1]["L"] >= NIVEL_MAX_MM:
        sys.exit("Error: el nivel llegó al corte de seguridad (%d mm) a los %.0f s del escalón; el ensayo no es válido "
                 "(repetir con un escalón menor)." % (NIVEL_MAX_MM, filas[fin + 1]["t"] - t_esc))
    elif fin + 1 < len(filas) and forzada(filas[fin + 1]):
        print("Aviso: falla del sensor (ERR = 1) a los %.0f s del escalón; se usa el registro hasta ahí."
              % (filas[fin + 1]["t"] - t_esc))
    if duracion < a.ventana:
        sys.exit("Error: el tramo posterior al escalón dura %.0f s, menos que --ventana (%g s); revisar el ensayo."
                 % (duracion, a.ventana))
    if abs(dU) < 3:
        print("Aviso: el escalón es chico frente a la resolución del PWM (1 %); conviene dU >= 5 %.")

    tau63 = metodo_632(t, L, L0, Lf)
    K63 = (Lf - L0) / dU
    ajuste = [(ti, Li) for ti, Li in zip(t, L) if ti >= -a.ventana]
    tau, L0ls, Als, rms = minimos_cuadrados([p[0] for p in ajuste], [p[1] for p in ajuste])
    Kls = Als / dU
    print("")
    print("Método del 63,2 %%:   K = %.3f mm/%%   tau = %.1f s" % (K63, tau63))
    print("Mínimos cuadrados:   K = %.3f mm/%%   tau = %.1f s   (L0 = %.1f mm, residuo RMS = %.2f mm)" % (Kls, tau, L0ls, rms))
    if duracion < 4.0 * tau:
        print("Aviso: el registro posterior al escalón dura %.1f tau (< 4 tau): el valor final del método del 63,2 %% "
              "no es confiable; se usa el ajuste por mínimos cuadrados." % (duracion / tau))
    if not (Kls > 0.0 and 1.01 * TAU_MIN < tau < 0.99 * TAU_MAX):
        sys.exit("Error: el ajuste no dio K > 0 y tau dentro de la búsqueda (%g a %g s); revisar el ensayo o elegir "
                 "otro escalón con --escalon." % (TAU_MIN, TAU_MAX))

    Kc, Ti, Ki = sintonia(Kls, tau, a.lam)
    print("")
    print("Sintonía lambda/SIMC con lambda = %g s (sobre el ajuste por mínimos cuadrados):" % a.lam)
    print("  Kc = tau/(K lambda) = %.3f %%/mm   Ti = min(tau, 4 lambda) = %.1f s   Ki = Kc/Ti = %.5f %%/(mm s)" % (Kc, Ti, Ki))
    print("")
    print("/* Control.h: identificado de %s (K = %.3f mm/%%, tau = %.1f s, lambda = %g s) */" % (a.csv, Kls, tau, a.lam))
    print("#define PID_KP    %#.3gf" % Kc)
    print("#define PID_KI    %#.3gf" % Ki)


if __name__ == "__main__":
    main()
