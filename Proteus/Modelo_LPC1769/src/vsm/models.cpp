// Adaptador VSM: conecta el modelo del LPC1769 y el del HC-SR04 con el simulador de Proteus.
//
// Sincronización: la CPU corre en bloques de QUANTUM (10 us por defecto) dentro de callbacks de DSIM.
// Las entradas se aplican en su instante exacto (el SoC nunca va adelantado respecto de Proteus) y las
// salidas se publican con un retardo fijo de un QUANTUM, porque Proteus no acepta eventos en el pasado.
// Ninguna excepción de C++ sale de la DLL: cada punto de entrada la atrapa y la informa a Proteus.
#include "models.hpp"
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace lpcsim;

namespace vsmmodels {

static const ABSTIME PS_US = 1000000;

bool parse_port_pin(const char *name, int &port, int &bit) {
    if (!name) return false;
    const char *s = name;
    if ((s[0] == 'P' || s[0] == 'p') && (s[1] == 'I' || s[1] == 'i') && (s[2] == 'O' || s[2] == 'o')) s += 3;
    else if (s[0] == 'P' || s[0] == 'p') s += 1;
    else return false;
    if (!isdigit((unsigned char)s[0])) return false;
    int p = s[0] - '0';
    s++;
    if (*s != '.' && *s != '_' && *s != '[') return false;
    s++;
    if (!isdigit((unsigned char)*s)) return false;
    int b = 0;
    while (isdigit((unsigned char)*s)) { b = b * 10 + (*s - '0'); s++; }
    if (*s == ']') s++;
    if (*s != 0 && *s != '/' && *s != ' ') return false;
    if (p > 4 || b > 31) return false;
    port = p;
    bit = b;
    return true;
}

std::string prop_str(IINSTANCE *inst, const char *name, const char *def) {
    CHAR *v = inst->getstrval((CHAR *)name, nullptr);
    if (!v || !*v) return def ? def : "";
    return v;
}

static bool ieq(const char *a, const char *b) {
    for (; *a && *b; a++, b++)
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    return *a == 0 && *b == 0;
}

double parse_num(const std::string &str, double def, const char *unit, bool &bad) {
    bad = false;
    if (str.empty()) return def;
    const char *s = str.c_str();
    char *end = nullptr;
    double v = strtod(s, &end);
    if (end == s) { bad = true; return def; }
    while (*end == ' ') end++;
    if (*end == 0) return v;
    if (unit && ieq(end, unit)) return v;
    double mult = 1;
    const char *rest = end;
    if (strncmp(end, "meg", 3) == 0 || strncmp(end, "MEG", 3) == 0) { mult = 1e6; rest = end + 3; }
    else {
        switch (*end) {
        case 'p': mult = 1e-12; break;
        case 'n': mult = 1e-9; break;
        case 'u': case 'U': mult = 1e-6; break;
        case 'm': mult = 1e-3; break;
        case 'k': case 'K': mult = 1e3; break;
        case 'M': mult = 1e6; break;
        case 'G': mult = 1e9; break;
        default: bad = true; return v;
        }
        rest = end + 1;
    }
    if (*rest != 0 && !(unit && ieq(rest, unit))) bad = true;
    return v * mult;
}

void Logger::attach(IINSTANCE *i, const char *caption, POPUPID id, bool want_popup) {
    inst = i;
    popup = nullptr;
    if (!want_popup) return;
    CREATEPOPUPSTRUCT cps;
    memset(&cps, 0, sizeof(cps));
    cps.id = id;
    cps.type = PWT_DEBUG;
    cps.caption = (CHAR *)caption;
    cps.width = 640;
    cps.height = 240;
    cps.flags = (DWORD)(PWF_VISIBLE | PWF_SIZEABLE);
    popup = (IDEBUGPOPUP *)inst->createpopup(&cps);
}

void Logger::print(const std::string &msg) {
    for (size_t pos = 0; pos < msg.size() || pos == 0; pos += 160) {
        std::string part = msg.substr(pos, 160);
        if (popup) popup->print((CHAR *)"%s\n", part.c_str());
        if (inst) inst->log((CHAR *)"%s", part.c_str());
        if (msg.empty()) break;
    }
}

void Logger::log_only(const std::string &msg) {
    if (inst) inst->log((CHAR *)"%s", msg.substr(0, 200).c_str());
}

// ============================================================ LPC1769

LpcModel::LpcModel(bool m, const char *dev) : mixed(m), device(dev ? dev : "") {
    for (int p = 0; p < 5; p++) for (int b = 0; b < 32; b++) { dindex[p][b] = -1; aindex[p][b] = -1; }
}

LpcModel::~LpcModel() {}

bool LpcModel::is_analog_pin(int port, int bit) const {
    return (port == 0 && bit >= 23 && bit <= 26) || (port == 1 && (bit == 30 || bit == 31));
}

void LpcModel::report(const char *where) {
    running = false;
    chain++;
    if (inst) inst->error((CHAR *)"LPC1769: error interno del modelo en %s; la CPU se detuvo", (CHAR *)where);
}

double LpcModel::num_prop(const char *name, double def, const char *unit) {
    bool bad = false;
    std::string s = prop_str(inst, name, "");
    double v = parse_num(s, def, unit, bad);
    if (bad) {
        char b[200];
        snprintf(b, sizeof(b), "Propiedad %s=%s: no se entiende la unidad; se usa %g", name, s.c_str(), v);
        logger.print(b);
        inst->warning((CHAR *)"%s", b);
    }
    return v;
}

INT LpcModel::isanalog(CHAR *pinname) {
    int p, b;
    if (!mixed || !parse_port_pin(pinname, p, b) || !is_analog_pin(p, b)) return FALSE;
    analog_names.push_back(pinname);
    return TRUE;
}

INT LpcModel::isdigital(CHAR *pinname) {
    int p, b;
    if (parse_port_pin(pinname, p, b)) {
        if (mixed && is_analog_pin(p, b)) return FALSE;
        digital_names.push_back(pinname);
        return TRUE;
    }
    std::string n = pinname ? pinname : "";
    if (n == "RESET_N" || n == "RESET" || n == "$RESET$" || n == "NRESET") { digital_names.push_back(n); return TRUE; }
    return FALSE;
}

void LpcModel::init_common(IINSTANCE *i) {
    if (inst == i && soc) return;
    inst = i;
    logger.attach(inst, "LPC1769 - registro del modelo", 0x1769, prop_str(inst, "POPUP", "1") != "0");
    if (!soc) soc.reset(new Lpc1769(this));
    soc->set_xtal(num_prop("XTAL", 12e6, "Hz"));
    soc->set_vref(num_prop("VREF", 3.3, "V"));
    double q = num_prop("QUANTUM", 10e-6, "s");
    if (q < 1e-7) q = 1e-7;
    if (q > 1e-3) q = 1e-3;
    quantum = (int64_t)(q * 1e12);
    analog_step = num_prop("ASTEP", 1e-3, "s");
    break_on_fault = prop_str(inst, "BREAKONFAULT", "1") != "0";
    char key[8];
    for (int ch = 0; ch < 8; ch++) {
        snprintf(key, sizeof(key), "ADC%d", ch);
        adc_fallback[ch] = num_prop(key, 0.0, "V");
    }
    char b[200];
    snprintf(b, sizeof(b), "Modelo LPC1769 %s (dispositivo \"%s\"), XTAL = %.3f MHz, quantum = %.1f us",
             mixed ? "mixto: AD0.0-AD0.5 leen la tensión del circuito" : "digital: el ADC usa las propiedades ADC0..ADC7",
             device.c_str(), num_prop("XTAL", 12e6, "Hz") / 1e6, q * 1e6);
    logger.print(b);
}

VOID LpcModel::setup(IINSTANCE *i, ISPICECKT *ckt) {
    try {
        init_common(i);
        spice = ckt;
        apins.clear();
        for (int p = 0; p < 5; p++) for (int b = 0; b < 32; b++) aindex[p][b] = -1;
        std::vector<std::string> names = analog_names;
        if (names.empty()) {
            const int cand[6][2] = {{0, 23}, {0, 24}, {0, 25}, {0, 26}, {1, 30}, {1, 31}};
            char n[16];
            for (auto &c : cand) {
                snprintf(n, sizeof(n), "P%d.%d", c[0], c[1]); names.push_back(n);
                snprintf(n, sizeof(n), "P%d[%d]", c[0], c[1]); names.push_back(n);
                snprintf(n, sizeof(n), "P%d_%d", c[0], c[1]); names.push_back(n);
            }
        }
        double agmin = num_prop("AGMIN", 1e-9, "S");
        for (auto &n : names) {
            int p, b;
            if (!parse_port_pin(n.c_str(), p, b) || aindex[p][b] >= 0) continue;
            SPICENODE node = inst->getspicenode((CHAR *)n.c_str(), FALSE);
            if (node <= 0) continue;
            APin r;
            r.name = n; r.port = p; r.bit = b; r.node = node;
            if (agmin > 0) r.smp = spice->allocsmp(node, node);
            aindex[p][b] = (int)apins.size();
            apins.push_back(r);
        }
    } catch (...) { report("setup (SPICE)"); }
}

void LpcModel::collect_pins() {
    std::vector<std::string> names = digital_names;
    if (names.empty()) {
        char n[16];
        for (int p = 0; p < 5; p++)
            for (int b = 0; b < 32; b++) {
                if (mixed && is_analog_pin(p, b)) continue;
                snprintf(n, sizeof(n), "P%d.%d", p, b); names.push_back(n);
                snprintf(n, sizeof(n), "P%d[%d]", p, b); names.push_back(n);
                snprintf(n, sizeof(n), "P%d_%d", p, b); names.push_back(n);
            }
        names.push_back("RESET_N");
        names.push_back("RESET");
    }
    for (auto &n : names) {
        int p, b;
        if (!parse_port_pin(n.c_str(), p, b)) {
            if (!reset_pin) {
                reset_pin = inst->getdsimpin((CHAR *)n.c_str(), FALSE);
                if (reset_pin) reset_level = !islow(reset_pin->istate());
            }
            continue;
        }
        if (dindex[p][b] >= 0) continue;
        IDSIMPIN *pin = inst->getdsimpin((CHAR *)n.c_str(), FALSE);
        if (!pin) continue;
        DPin r;
        r.name = n; r.port = p; r.bit = b; r.pin = pin;
        r.level = ishigh(pin->istate());
        dindex[p][b] = (int)dpins.size();
        dpins.push_back(r);
    }
    for (auto &r : dpins) soc->set_pin_present(r.port, r.bit, true, r.level);
    char b[120];
    snprintf(b, sizeof(b), "Pines conectados al modelo: %d digitales, %d analógicos", (int)dpins.size(), (int)apins.size());
    logger.print(b);
    if (dpins.empty())
        inst->warning((CHAR *)"LPC1769: ningún pin digital reconocido; los nombres deben ser P0.4, P0[4] o P0_4 (ver pines_J6.csv)");
}

VOID LpcModel::setup(IINSTANCE *i, IDSIMCKT *ckt) {
    try {
        init_common(i);
        dsim = ckt;
        dpins.clear();
        for (int p = 0; p < 5; p++) for (int b = 0; b < 32; b++) dindex[p][b] = -1;
        boot_outputs.clear();
        dsim_ready = false;
        callback_seen = false;
        stop_logged = false;
        reset_pin = nullptr;
        reset_level = true;
        running = false;
        loaded = false;
        chain++;
        now_t = 0;
        collect_pins();
        std::string prog = prop_str(inst, "PROGRAM", "");
        if (prog.empty()) {
            logger.print("Falta la propiedad PROGRAM: la CPU no arranca");
            inst->error((CHAR *)"LPC1769: falta la propiedad PROGRAM (ruta al .axf, .elf o .hex de MCUXpresso)");
            return;
        }
        std::string err;
        if (!soc->load_program(prog, err)) {
            logger.print("Error al cargar " + prog + ": " + err);
            inst->error((CHAR *)"LPC1769: %s", (CHAR *)err.c_str());
            return;
        }
        loaded = true;
        logger.print("Programa cargado: " + prog);
        soc->reset(0);
        if (reset_pin && !reset_level) soc->hold_reset(0, true);
        running = true;
        start_chain(0);
    } catch (...) { report("setup (DSIM)"); }
}

void LpcModel::start_chain(ABSTIME time) {
    chain++;
    dsim->setcallback(time + quantum, static_cast<IDSIMMODEL *>(this), chain);
}

STATE LpcModel::to_state(Drive d) {
    switch (d) {
    case Drive::STRONG_HIGH: return SHI;
    case Drive::STRONG_LOW: return SLO;
    case Drive::WEAK_HIGH: return WHI;
    case Drive::WEAK_LOW: return WLO;
    default: return FLT;
    }
}

void LpcModel::pin_output(int64_t t_ps, int port, int bit, Drive d) {
    if (aindex[port][bit] >= 0 && (d == Drive::STRONG_HIGH || d == Drive::STRONG_LOW)) {
        char key[16];
        snprintf(key, sizeof(key), "an%d.%d", port, bit);
        if (warned.insert(key).second) {
            char b[200];
            snprintf(b, sizeof(b), "P%d.%d está conectado como entrada analógica (modelo mixto): el firmware lo usa como salida y en la simulación no se ve", port, bit);
            logger.print(b);
        }
    }
    int idx = dindex[port][bit];
    if (idx < 0) return;
    DPin &r = dpins[idx];
    if (!r.pin) return;
    if (!dsim_ready) { boot_outputs.push_back({idx, d}); return; }
    ABSTIME tt = (ABSTIME)t_ps + quantum;
    if (tt <= now_t) tt = now_t + 1;
    STATE st = to_state(d);
    r.sched.push_back({tt, st});
    r.pin->drivestate(tt, st);
}

void LpcModel::flush_boot(ABSTIME time) {
    dsim_ready = true;
    for (auto &o : boot_outputs) {
        DPin &r = dpins[o.first];
        STATE st = to_state(o.second);
        r.sched.push_back({time + 1, st});
        r.pin->drivestate(time + 1, st);
    }
    boot_outputs.clear();
}

double LpcModel::analog_input(int port, int bit) {
    int idx = aindex[port][bit];
    if (idx >= 0) return apins[idx].volts;
    static const int pins_ch[8][2] = {{0, 23}, {0, 24}, {0, 25}, {0, 26}, {1, 30}, {1, 31}, {0, 3}, {0, 2}};
    for (int ch = 0; ch < 8; ch++)
        if (pins_ch[ch][0] == port && pins_ch[ch][1] == bit) return adc_fallback[ch];
    return 0.0;
}

void LpcModel::soc_log(const std::string &msg) {
    char b[48];
    snprintf(b, sizeof(b), "[%.6f s] ", (double)now_t * 1e-12);
    logger.print(b + msg);
    if (break_on_fault && dsim && inst && (msg.rfind("LOCKUP", 0) == 0 || msg.rfind("Escala a HardFault", 0) == 0))
        dsim->suspend(inst, (CHAR *)"LPC1769: falla del procesador (ver la ventana de registro del modelo)");
}

VOID LpcModel::runctrl(RUNMODES mode) {
    try {
        if (mode == RM_STOP && soc && running && !stop_logged) {
            stop_logged = true;
            char b[200];
            snprintf(b, sizeof(b), "LPC1769: fin de la simulación a %.3f s, %.1f M ciclos de CPU, CCLK = %.3f MHz",
                     (double)now_t * 1e-12, (double)soc->instructions() / 1e6, soc->cclk() / 1e6);
            logger.log_only(b);
        }
    } catch (...) {}
}

VOID LpcModel::actuate(REALTIME, ACTIVESTATE) {}
BOOL LpcModel::indicate(REALTIME, ACTIVEDATA *) { return FALSE; }

VOID LpcModel::dcload(REALTIME, SPICEMODES, DOUBLE *, DOUBLE *) {
    for (auto &r : apins)
        if (r.smp) *r.smp += 1e-9;
}

VOID LpcModel::acload(SPICEFREQ, DOUBLE *, DOUBLE *) {}

VOID LpcModel::trunc(REALTIME, REALTIME *newtimestep) {
    if (!apins.empty() && newtimestep && analog_step > 0 && *newtimestep > analog_step) *newtimestep = analog_step;
}

VOID LpcModel::accept(REALTIME, DOUBLE *rhs) {
    if (!rhs) return;
    for (auto &r : apins) r.volts = rhs[r.node];
}

VOID LpcModel::simulate(ABSTIME time, DSIMMODES) {
    try {
        if (!soc || !dsim) return;
        if (time > now_t) now_t = time;
        if (!dsim_ready) flush_boot(time);
        if (running && !callback_seen && time >= 3 * quantum) {
            logger.print("El primer callback no llegó: se reinicia la cadena de ejecución");
            callback_seen = true;
            start_chain(time);
        }
        for (auto &r : dpins) {
            if (!r.pin) continue;
            while (!r.sched.empty() && r.sched.front().first <= time) { r.own = r.sched.front().second; r.sched.pop_front(); }
            bool lvl = ishigh(r.pin->istate());
            if (lvl == r.level) continue;
            r.level = lvl;
            if (r.own == SHI || r.own == SLO) {
                soc->set_pin_present(r.port, r.bit, true, lvl);
                continue;
            }
            soc->input_edge(time, r.port, r.bit, lvl);
        }
        if (reset_pin) {
            bool lvl = !islow(reset_pin->istate());
            if (lvl != reset_level) {
                reset_level = lvl;
                if (running) soc->run_until(time);
                soc->hold_reset(time, !lvl);
            }
        }
    } catch (...) { report("simulate"); }
}

VOID LpcModel::callback(ABSTIME time, EVENTID id) {
    try {
        if (id != chain || !soc || !running) return;
        callback_seen = true;
        now_t = time;
        if (!dsim_ready) flush_boot(time);
        soc->run_until(time);
        dsim->setcallback(time + quantum, static_cast<IDSIMMODEL *>(this), chain);
    } catch (...) { report("callback"); }
}

// ============================================================ HC-SR04

SonarModel::SonarModel(bool m) : mixed(m) {}

INT SonarModel::isanalog(CHAR *pinname) {
    return (mixed && pinname && strcmp(pinname, "DIST") == 0) ? TRUE : FALSE;
}

INT SonarModel::isdigital(CHAR *pinname) {
    if (!pinname) return FALSE;
    return (strcmp(pinname, "TRIG") == 0 || strcmp(pinname, "ECHO") == 0) ? TRUE : FALSE;
}

VOID SonarModel::setup(IINSTANCE *i, ISPICECKT *ckt) {
    try {
        inst = i;
        dist_node = inst->getspicenode((CHAR *)"DIST", FALSE);
        dist_smp = (dist_node > 0 && ckt) ? ckt->allocsmp(dist_node, dist_node) : nullptr;
    } catch (...) { if (inst) inst->error((CHAR *)"HC-SR04: error interno en setup (SPICE)"); }
}

VOID SonarModel::setup(IINSTANCE *i, IDSIMCKT *ckt) {
    try {
        inst = i;
        dsim = ckt;
        started = false;
        busy_until = 0;
        trig_rise = -1;
        logger.attach(inst, "HC-SR04", 0x5204, prop_str(inst, "POPUP", "0") != "0");
        trig = inst->getdsimpin((CHAR *)"TRIG", TRUE);
        echo = inst->getdsimpin((CHAR *)"ECHO", TRUE);
        bool bad = false;
        std::string s = prop_str(inst, "DISTANCE", "200");
        distance_mm = parse_num(s, 200.0, "mm", bad);
        if (bad) inst->warning((CHAR *)"HC-SR04: DISTANCE se da en mm, sin unidad (por ejemplo 200)");
        mm_per_volt = parse_num(prop_str(inst, "MMPERVOLT", "100"), 100.0, nullptr, bad);
        delay_s = parse_num(prop_str(inst, "DELAY", "250u"), 250e-6, "s", bad);
        if (trig) trig_level = ishigh(trig->istate());
        if (!trig || !echo) inst->error((CHAR *)"HC-SR04: faltan los pines TRIG y ECHO en el símbolo");
    } catch (...) { if (inst) inst->error((CHAR *)"HC-SR04: error interno en setup (DSIM)"); }
}

double SonarModel::current_mm() const {
    if (mixed && dist_node > 0 && dist_sampled) return dist_volts * mm_per_volt;
    return distance_mm;
}

VOID SonarModel::simulate(ABSTIME time, DSIMMODES) {
    try {
        if (!trig || !echo) return;
        if (!started) { echo->drivestate(time + 1, SLO); started = true; }
        bool lvl = ishigh(trig->istate());
        if (lvl == trig_level) return;
        trig_level = lvl;
        if (lvl) { trig_rise = time; return; }
        if (trig_rise < 0) return;
        ABSTIME width = time - trig_rise;
        trig_rise = -1;
        if (width < 10 * PS_US || time < busy_until) return;
        double mm = current_mm();
        double secs;
        if (mm > 4000.0) secs = 38e-3;
        else {
            if (mm < 20.0) mm = 20.0;
            secs = 2.0 * mm / 1000.0 / 343.0;
        }
        ABSTIME t0 = time + (ABSTIME)(delay_s * 1e12);
        ABSTIME t1 = t0 + (ABSTIME)(secs * 1e12);
        echo->drivestate(t0, SHI);
        echo->drivestate(t1, SLO);
        busy_until = t1;
        char b[120];
        snprintf(b, sizeof(b), "Disparo a %.6f s: distancia %.1f mm, eco de %.1f us", (double)time * 1e-12, mm, secs * 1e6);
        logger.print(b);
    } catch (...) { if (inst) inst->error((CHAR *)"HC-SR04: error interno en simulate"); trig = nullptr; }
}

VOID SonarModel::callback(ABSTIME, EVENTID) {}
VOID SonarModel::runctrl(RUNMODES) {}
VOID SonarModel::actuate(REALTIME, ACTIVESTATE) {}
BOOL SonarModel::indicate(REALTIME, ACTIVEDATA *) { return FALSE; }
VOID SonarModel::dcload(REALTIME, SPICEMODES, DOUBLE *, DOUBLE *) { if (dist_smp) *dist_smp += 1e-9; }
VOID SonarModel::acload(SPICEFREQ, DOUBLE *, DOUBLE *) {}
VOID SonarModel::trunc(REALTIME, REALTIME *) {}
VOID SonarModel::accept(REALTIME, DOUBLE *rhs) {
    if (dist_node > 0 && rhs) {
        double v = rhs[dist_node];
        dist_volts = v < 0 ? 0 : v;
        dist_sampled = true;
    }
}

} // namespace vsmmodels
