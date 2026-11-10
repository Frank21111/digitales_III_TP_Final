// Modelo del LPC1769. Referencia de registros: UM10360 rev. 4.1.
#include "lpc1769.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>

namespace lpcsim {

static const int64_t NEVER_T = INT64_MAX;

// ============================================================ Counter

uint32_t Counter::tc_at(int64_t t) const {
    if (!enabled) return tc_base;
    if (t < base_t) return hold_val;
    int64_t n = (int64_t)((double)(t - base_t) / ps_per_pclk + 1e-9);
    uint64_t total = pc_base + (uint64_t)n;
    return tc_base + (uint32_t)(total / ((uint64_t)pr + 1));
}

uint32_t Counter::pc_at(int64_t t) const {
    if (!enabled || t < base_t) return (uint32_t)pc_base;
    int64_t n = (int64_t)((double)(t - base_t) / ps_per_pclk + 1e-9);
    uint64_t total = pc_base + (uint64_t)n;
    return (uint32_t)(total % ((uint64_t)pr + 1));
}

void Counter::rebase(int64_t t) {
    if (!enabled || t <= base_t) return;
    int64_t n = (int64_t)((double)(t - base_t) / ps_per_pclk + 1e-9);
    if (n <= 0) return;
    uint64_t total = pc_base + (uint64_t)n;
    tc_base += (uint32_t)(total / ((uint64_t)pr + 1));
    pc_base = total % ((uint64_t)pr + 1);
    base_t += (int64_t)((double)n * ps_per_pclk + 0.5);
    fresh = false;
}

int64_t Counter::time_tc_reaches(uint32_t v) const {
    if (!enabled) return NEVER_T;
    if (v == tc_base && fresh) return base_t;
    uint64_t k = (uint32_t)(v - tc_base);
    if (k == 0) k = 1ull << 32;
    double cycles = (double)k * ((double)pr + 1.0) - (double)pc_base;
    return base_t + (int64_t)std::ceil(cycles * ps_per_pclk - 1e-6);
}

// ============================================================ construcción, carga, reset

Lpc1769::Lpc1769(SocHost *h)
    : host(h), cpu(this, 35), flash(512 * 1024, 0xFF), sram0(32 * 1024, 0), sram1(32 * 1024, 0), bootrom(8 * 1024, 0) {
    cpu.add_region(0x00000000, (uint32_t)flash.size(), flash.data(), false);
    cpu.add_region(0x10000000, (uint32_t)sram0.size(), sram0.data(), true);
    cpu.add_region(0x2007C000, (uint32_t)sram1.size(), sram1.data(), true);
    cpu.add_region(0x1FFF0000, (uint32_t)bootrom.size(), bootrom.data(), false);
    for (int i = 0; i < 4; i++) { tim[i].idx = i; uart[i].idx = i; }
    for (int p = 0; p < 5; p++) for (int b = 0; b < 32; b++) drive[p][b] = Drive::WEAK_HIGH;
}

Lpc1769::~Lpc1769() {}

static bool read_file(const std::string &path, std::vector<uint8_t> &out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

bool Lpc1769::load_program(const std::string &path, std::string &err) {
    std::vector<uint8_t> d;
    if (!read_file(path, d)) { err = "no se pudo abrir el archivo de programa: " + path; return false; }
    std::fill(flash.begin(), flash.end(), 0xFF);
    auto put = [&](uint32_t addr, const uint8_t *src, size_t n) -> bool {
        for (size_t i = 0; i < n; i++) {
            uint32_t a = addr + (uint32_t)i;
            if (a < flash.size()) flash[a] = src[i];
            else if (a >= 0x10000000u && a < 0x10000000u + sram0.size()) sram0[a - 0x10000000u] = src[i];
            else if (a >= 0x2007C000u && a < 0x2007C000u + sram1.size()) sram1[a - 0x2007C000u] = src[i];
            else return false;
        }
        return true;
    };
    if (d.size() >= 52 && d[0] == 0x7F && d[1] == 'E' && d[2] == 'L' && d[3] == 'F') {
        if (d[4] != 1 || d[5] != 1) { err = "ELF no es de 32 bits little-endian"; return false; }
        auto u16 = [&](size_t o) { return (uint32_t)d[o] | ((uint32_t)d[o + 1] << 8); };
        auto u32 = [&](size_t o) { return u16(o) | (u16(o + 2) << 16); };
        if (u16(18) != 40) { err = "el ELF no es para ARM"; return false; }
        uint32_t phoff = u32(28), phentsize = u16(42), phnum = u16(44);
        int loaded = 0;
        for (uint32_t i = 0; i < phnum; i++) {
            size_t o = phoff + (size_t)i * phentsize;
            if (o + 32 > d.size()) break;
            if (u32(o) != 1) continue;
            uint32_t off = u32(o + 4), paddr = u32(o + 12), filesz = u32(o + 16);
            if (filesz == 0) continue;
            if ((size_t)off + filesz > d.size()) { err = "ELF truncado"; return false; }
            if (!put(paddr, &d[off], filesz)) {
                char b[120];
                snprintf(b, sizeof(b), "el segmento en 0x%08X no cae en la Flash ni en la RAM del LPC1769", paddr);
                err = b;
                return false;
            }
            loaded++;
        }
        if (!loaded) { err = "el ELF no tiene segmentos cargables"; return false; }
        return true;
    }
    if (!d.empty() && d[0] == ':') {
        uint32_t upper = 0;
        size_t i = 0;
        auto hexv = [&](size_t pos, int n, bool &ok) -> uint32_t {
            uint32_t v = 0;
            for (int k = 0; k < n; k++) {
                if (pos + k >= d.size()) { ok = false; return 0; }
                char c = (char)d[pos + k];
                v <<= 4;
                if (c >= '0' && c <= '9') v |= (uint32_t)(c - '0');
                else if (c >= 'A' && c <= 'F') v |= (uint32_t)(c - 'A' + 10);
                else if (c >= 'a' && c <= 'f') v |= (uint32_t)(c - 'a' + 10);
                else { ok = false; return 0; }
            }
            return v;
        };
        while (i < d.size()) {
            if (d[i] != ':') { i++; continue; }
            bool ok = true;
            uint32_t n = hexv(i + 1, 2, ok), addr = hexv(i + 3, 4, ok), type = hexv(i + 7, 2, ok);
            if (!ok) { err = "Intel HEX mal formado"; return false; }
            std::vector<uint8_t> bytes(n);
            for (uint32_t k = 0; k < n; k++) bytes[k] = (uint8_t)hexv(i + 9 + 2 * k, 2, ok);
            if (!ok) { err = "Intel HEX mal formado"; return false; }
            if (type == 0) {
                if (!put(upper + addr, bytes.data(), n)) { err = "dato del HEX fuera de la memoria"; return false; }
            } else if (type == 1) break;
            else if (type == 2 && n == 2) upper = (((uint32_t)bytes[0] << 8) | bytes[1]) << 4;
            else if (type == 4 && n == 2) upper = (((uint32_t)bytes[0] << 8) | bytes[1]) << 16;
            i += 11 + 2 * n;
        }
        return true;
    }
    if (d.size() > flash.size()) { err = "el binario no entra en la Flash"; return false; }
    put(0, d.data(), d.size());
    return true;
}

void Lpc1769::reset_peripherals() {
    flashcfg = 0x303A; pll0con = pll0cfg = pll0con_act = pll0cfg_act = pll0feed = 0;
    pll1con = pll1cfg = pll1con_act = pll1cfg_act = pll1feed = 0;
    cclkcfg = usbclkcfg = clksrcsel = scs = 0;
    pconp = 0x042887DE;
    pclksel0 = pclksel1 = extint = extmode = extpolar = pcon = clkoutcfg = 0;
    memset(pinsel, 0, sizeof(pinsel));
    memset(pinmode, 0, sizeof(pinmode));
    memset(pinmode_od, 0, sizeof(pinmode_od));
    memset(fiodir, 0, sizeof(fiodir));
    memset(fiomask, 0, sizeof(fiomask));
    memset(fioout, 0, sizeof(fioout));
    memset(io_int_en_r, 0, sizeof(io_int_en_r));
    memset(io_int_en_f, 0, sizeof(io_int_en_f));
    memset(io_int_stat_r, 0, sizeof(io_int_stat_r));
    memset(io_int_stat_f, 0, sizeof(io_int_stat_f));
    for (int i = 0; i < 4; i++) { int idx = tim[i].idx; tim[i] = Timer(); tim[i].idx = idx; }
    pwm = Pwm();
    adc = Adc();
    for (int i = 0; i < 4; i++) { Uart u; u.idx = i; uart[i] = u; }
    st_ctrl = st_load = st_val0 = 0; st_base = 0; st_next = NEVER_T; st_countflag = false;
}

void Lpc1769::reset(int64_t t) {
    reset_peripherals();
    cpu.reset();
    cclk_hz = 4e6;
    ps_per_cycle = 1e12 / cclk_hz;
    seg_t0 = t;
    seg_c0 = cpu.cycles;
    st_base = t;
    rebase_peripherals(t);
    for (int p = 0; p < 5; p++)
        for (int b = 0; b < 32; b++) {
            drive[p][b] = compute_drive(p, b);
            host->pin_output(t, p, b, drive[p][b]);
        }
}

void Lpc1769::hold_reset(int64_t t, bool held) {
    if (held == in_reset) return;
    reset(t);
    in_reset = held;
    cpu_log(held ? "RESET en bajo: CPU detenida, pines como entradas con pull-up"
                 : "RESET liberado: arranca el programa");
}

void Lpc1769::set_pin_present(int port, int bit, bool pres, bool level) {
    if (pres) present[port] |= 1u << bit; else present[port] &= ~(1u << bit);
    if (level) in_level[port] |= 1u << bit; else in_level[port] &= ~(1u << bit);
}

int64_t Lpc1769::now() const {
    return seg_t0 + (int64_t)((double)(cpu.cycles - seg_c0) * ps_per_cycle);
}

void Lpc1769::cpu_log(const std::string &msg) { host->soc_log(msg); }

void Lpc1769::warn_once(const std::string &key, const std::string &msg) {
    if (logged.insert(key).second) host->soc_log(msg);
}

// ============================================================ relojes

double Lpc1769::pclk(int sel_reg, int field) const {
    uint32_t reg = sel_reg == 0 ? pclksel0 : pclksel1;
    switch ((reg >> (2 * field)) & 3) {
    case 0: return cclk_hz / 4;
    case 1: return cclk_hz;
    case 2: return cclk_hz / 2;
    default: return cclk_hz / 8;
    }
}

void Lpc1769::update_cclk() {
    double fin;
    switch (clksrcsel & 3) {
    case 1:
        if (!(scs & 0x20)) {
            warn_once("osc", "CLKSRCSEL elige el oscilador principal pero SCS.OSCEN está en 0: se sigue con el IRC de 4 MHz");
            fin = 4e6;
        } else fin = xtal_hz;
        break;
    case 2: fin = 32768.0; break;
    default: fin = 4e6; break;
    }
    double f;
    if ((pll0con_act & 3) == 3) {
        double m = (double)((pll0cfg_act & 0x7FFF) + 1), n = (double)(((pll0cfg_act >> 16) & 0xFF) + 1);
        double fcco = 2.0 * m * fin / n;
        if (fcco < 275e6 || fcco > 550e6) {
            char b[160];
            snprintf(b, sizeof(b), "PLL0: Fcco = %.1f MHz fuera del rango 275–550 MHz (M=%.0f, N=%.0f)", fcco / 1e6, m, n);
            warn_once("fcco", b);
        }
        f = fcco / (double)((cclkcfg & 0xFF) + 1);
    } else {
        f = fin / (double)((cclkcfg & 0xFF) + 1);
    }
    if (f > 120e6 + 1) warn_once("cclk", "CCLK por encima de 120 MHz: el LPC1769 real no lo soporta");
    if (std::fabs(f - cclk_hz) > 0.5) set_cclk(f);
}

void Lpc1769::set_cclk(double hz) {
    int64_t t = now();
    for (int i = 0; i < 4; i++) tim[i].c.rebase(t);
    pwm.c.rebase(t);
    st_val0 = systick_val(t);
    st_base = t;
    seg_t0 = t;
    seg_c0 = cpu.cycles;
    cclk_hz = hz;
    ps_per_cycle = 1e12 / hz;
    char b[80];
    snprintf(b, sizeof(b), "CCLK = %.3f MHz", hz / 1e6);
    host->soc_log(b);
    rebase_peripherals(t);
    cpu.stop_request = true;
}

void Lpc1769::rebase_peripherals(int64_t t) {
    static const int tim_sel[4][2] = {{0, 1}, {0, 2}, {1, 6}, {1, 7}};
    for (int i = 0; i < 4; i++) {
        tim[i].c.rebase(t);
        tim[i].c.ps_per_pclk = 1e12 / pclk(tim_sel[i][0], tim_sel[i][1]);
        timer_resched(tim[i], t);
    }
    pwm.c.rebase(t);
    pwm.c.ps_per_pclk = 1e12 / pclk(0, 6);
    pwm_resched(t);
    systick_resched(t);
}

// ============================================================ pines

PinFunc Lpc1769::func_of(int port, int bit) const {
    uint32_t sel = (pinsel[port * 2 + (bit >> 4)] >> ((bit & 15) * 2)) & 3;
    if (sel == 0) return PF_GPIO;
    switch ((port << 5) | bit) {
    case (0 << 5) | 0: return sel == 2 ? PF_TXD3 : PF_OTHER;
    case (0 << 5) | 1: return sel == 2 ? PF_RXD3 : PF_OTHER;
    case (0 << 5) | 2: return sel == 1 ? PF_TXD0 : sel == 2 ? PF_AD7 : PF_OTHER;
    case (0 << 5) | 3: return sel == 1 ? PF_RXD0 : sel == 2 ? PF_AD6 : PF_OTHER;
    case (0 << 5) | 4: return sel == 3 ? PF_CAP20 : PF_OTHER;
    case (0 << 5) | 5: return sel == 3 ? PF_CAP21 : PF_OTHER;
    case (0 << 5) | 6: return sel == 3 ? PF_MAT20 : PF_OTHER;
    case (0 << 5) | 7: return sel == 3 ? PF_MAT21 : PF_OTHER;
    case (0 << 5) | 8: return sel == 3 ? PF_MAT22 : PF_OTHER;
    case (0 << 5) | 9: return sel == 3 ? PF_MAT23 : PF_OTHER;
    case (0 << 5) | 10: return sel == 1 ? PF_TXD2 : sel == 3 ? PF_MAT30 : PF_OTHER;
    case (0 << 5) | 11: return sel == 1 ? PF_RXD2 : sel == 3 ? PF_MAT31 : PF_OTHER;
    case (0 << 5) | 15: return sel == 1 ? PF_TXD1 : PF_OTHER;
    case (0 << 5) | 16: return sel == 1 ? PF_RXD1 : PF_OTHER;
    case (0 << 5) | 23: return sel == 1 ? PF_AD0 : sel == 3 ? PF_CAP30 : PF_OTHER;
    case (0 << 5) | 24: return sel == 1 ? PF_AD1 : sel == 3 ? PF_CAP31 : PF_OTHER;
    case (0 << 5) | 25: return sel == 1 ? PF_AD2 : sel == 3 ? PF_TXD3 : PF_OTHER;
    case (0 << 5) | 26: return sel == 1 ? PF_AD3 : sel == 2 ? PF_ANALOG : sel == 3 ? PF_RXD3 : PF_OTHER;
    case (1 << 5) | 18: return sel == 2 ? PF_PWM1 : sel == 3 ? PF_CAP10 : PF_OTHER;
    case (1 << 5) | 19: return sel == 3 ? PF_CAP11 : PF_OTHER;
    case (1 << 5) | 20: return sel == 2 ? PF_PWM2 : PF_OTHER;
    case (1 << 5) | 21: return sel == 2 ? PF_PWM3 : PF_OTHER;
    case (1 << 5) | 22: return sel == 3 ? PF_MAT10 : PF_OTHER;
    case (1 << 5) | 23: return sel == 2 ? PF_PWM4 : PF_OTHER;
    case (1 << 5) | 24: return sel == 2 ? PF_PWM5 : PF_OTHER;
    case (1 << 5) | 25: return sel == 3 ? PF_MAT11 : PF_OTHER;
    case (1 << 5) | 26: return sel == 2 ? PF_PWM6 : sel == 3 ? PF_CAP00 : PF_OTHER;
    case (1 << 5) | 27: return sel == 3 ? PF_CAP01 : PF_OTHER;
    case (1 << 5) | 28: return sel == 3 ? PF_MAT00 : PF_OTHER;
    case (1 << 5) | 29: return sel == 3 ? PF_MAT01 : PF_OTHER;
    case (1 << 5) | 30: return sel == 3 ? PF_AD4 : PF_OTHER;
    case (1 << 5) | 31: return sel == 3 ? PF_AD5 : PF_OTHER;
    case (2 << 5) | 0: return sel == 1 ? PF_PWM1 : sel == 2 ? PF_TXD1 : PF_OTHER;
    case (2 << 5) | 1: return sel == 1 ? PF_PWM2 : sel == 2 ? PF_RXD1 : PF_OTHER;
    case (2 << 5) | 2: return sel == 1 ? PF_PWM3 : PF_OTHER;
    case (2 << 5) | 3: return sel == 1 ? PF_PWM4 : PF_OTHER;
    case (2 << 5) | 4: return sel == 1 ? PF_PWM5 : PF_OTHER;
    case (2 << 5) | 5: return sel == 1 ? PF_PWM6 : PF_OTHER;
    case (2 << 5) | 8: return sel == 2 ? PF_TXD2 : PF_OTHER;
    case (2 << 5) | 9: return sel == 2 ? PF_RXD2 : PF_OTHER;
    case (2 << 5) | 10: return sel == 1 ? PF_EINT0 : PF_OTHER;
    case (2 << 5) | 11: return sel == 1 ? PF_EINT1 : PF_OTHER;
    case (2 << 5) | 12: return sel == 1 ? PF_EINT2 : PF_OTHER;
    case (2 << 5) | 13: return sel == 1 ? PF_EINT3 : PF_OTHER;
    case (3 << 5) | 25: return sel == 2 ? PF_MAT00 : sel == 3 ? PF_PWM2 : PF_OTHER;
    case (3 << 5) | 26: return sel == 2 ? PF_MAT01 : sel == 3 ? PF_PWM3 : PF_OTHER;
    case (4 << 5) | 28: return sel == 2 ? PF_MAT20 : sel == 3 ? PF_TXD3 : PF_OTHER;
    case (4 << 5) | 29: return sel == 2 ? PF_MAT21 : sel == 3 ? PF_RXD3 : PF_OTHER;
    default: return PF_OTHER;
    }
}

Drive Lpc1769::pull_drive(int port, int bit) const {
    if (port == 0 && (bit == 27 || bit == 28)) return Drive::FLOAT;
    uint32_t mode = (pinmode[port * 2 + (bit >> 4)] >> ((bit & 15) * 2)) & 3;
    switch (mode) {
    case 0: return Drive::WEAK_HIGH;
    case 1: return (in_level[port] >> bit) & 1 ? Drive::WEAK_HIGH : Drive::WEAK_LOW;
    case 2: return Drive::FLOAT;
    default: return Drive::WEAK_LOW;
    }
}

Drive Lpc1769::compute_drive(int port, int bit) const {
    PinFunc f = func_of(port, bit);
    auto strong = [](bool v) { return v ? Drive::STRONG_HIGH : Drive::STRONG_LOW; };
    bool od = ((pinmode_od[port] >> bit) & 1) || (port == 0 && (bit == 27 || bit == 28));
    auto out = [&](bool v) { return (od && v) ? pull_drive(port, bit) : strong(v); };
    switch (f) {
    case PF_GPIO:
        if ((fiodir[port] >> bit) & 1) return out((fioout[port] >> bit) & 1);
        return pull_drive(port, bit);
    case PF_TXD0: case PF_TXD1: case PF_TXD2: case PF_TXD3:
        return out(uart[f - PF_TXD0].txd);
    case PF_PWM1: case PF_PWM2: case PF_PWM3: case PF_PWM4: case PF_PWM5: case PF_PWM6: {
        int ch = f - PF_PWM1 + 1;
        bool en = (pwm.pcr >> (8 + ch)) & 1;
        return out(en && pwm.out[ch]);
    }
    case PF_MAT00: case PF_MAT01: return out((tim[0].emr >> (f - PF_MAT00)) & 1);
    case PF_MAT10: case PF_MAT11: return out((tim[1].emr >> (f - PF_MAT10)) & 1);
    case PF_MAT20: case PF_MAT21: case PF_MAT22: case PF_MAT23: return out((tim[2].emr >> (f - PF_MAT20)) & 1);
    case PF_MAT30: case PF_MAT31: return out((tim[3].emr >> (f - PF_MAT30)) & 1);
    case PF_ANALOG: case PF_AD0: case PF_AD1: case PF_AD2: case PF_AD3: case PF_AD4: case PF_AD5: case PF_AD6: case PF_AD7:
        return Drive::FLOAT;
    default:
        return pull_drive(port, bit);
    }
}

void Lpc1769::update_pin(int port, int bit, int64_t t) {
    Drive d = compute_drive(port, bit);
    if (d != drive[port][bit]) {
        drive[port][bit] = d;
        host->pin_output(t, port, bit, d);
    }
}

void Lpc1769::update_port(int port, int64_t t) {
    for (int b = 0; b < 32; b++) update_pin(port, b, t);
}

void Lpc1769::update_all_pins(int64_t t) {
    for (int p = 0; p < 5; p++) update_port(p, t);
}

void Lpc1769::update_func_pins(PinFunc f, int64_t t) {
    for (int p = 0; p < 5; p++)
        for (int b = 0; b < 32; b++)
            if (func_of(p, b) == f) update_pin(p, b, t);
}

bool Lpc1769::pin_level(int port, int bit) const {
    Drive d = drive[port][bit];
    if (d == Drive::STRONG_HIGH) return true;
    if (d == Drive::STRONG_LOW) return false;
    if ((present[port] >> bit) & 1) return (in_level[port] >> bit) & 1;
    return d == Drive::WEAK_HIGH;
}

uint32_t Lpc1769::fiopin_read(int port) const {
    uint32_t v = 0;
    for (int b = 0; b < 32; b++) if (pin_level(port, b)) v |= 1u << b;
    return v;
}

// ============================================================ interrupciones

void Lpc1769::irq(int n) { cpu.set_pending(16 + n); }

bool Lpc1769::irq_level(int n) {
    switch (n) {
    case 1: case 2: case 3: case 4: return (tim[n - 1].ir & 0x3F) != 0;
    case 5: case 6: case 7: case 8: return (uart_iir(uart[n - 5]) & 1) == 0;
    case 9: return (pwm.ir & 0x73F) != 0;
    case 18: case 19: case 20: return (extint >> (n - 18)) & 1;
    case 21: return (extint & 8) || io_int_stat_r[0] || io_int_stat_f[0] || io_int_stat_r[2] || io_int_stat_f[2];
    case 22: {
        uint32_t done = 0;
        for (int i = 0; i < 8; i++) if (adc.dr[i] >> 31) done |= 1u << i;
        return (done & adc.inten & 0xFF) || ((adc.inten & 0x100) && (adc.gdr >> 31));
    }
    default: return false;
    }
}

void Lpc1769::cpu_irq_recheck(uint64_t mask) {
    for (int n = 0; n < 35; n++)
        if (((mask >> n) & 1) && irq_level(n)) cpu.set_pending(16 + n);
}

void Lpc1769::eint_recheck() {
    for (int n = 0; n < 4; n++) {
        if ((extmode >> n) & 1) continue;
        for (int p = 0; p < 5; p++)
            for (int b = 0; b < 32; b++)
                if (func_of(p, b) == (PinFunc)(PF_EINT0 + n)) eint_input(n, pin_level(p, b), false, false);
    }
}

void Lpc1769::adc_hw_trigger(uint32_t code, bool rising, int64_t t) {
    if (!((pconp >> 12) & 1) || !(adc.cr & (1u << 21)) || (adc.cr & (1u << 16))) return;
    if (((adc.cr >> 24) & 7) != code) return;
    bool falling_edge = (adc.cr >> 27) & 1;
    if (rising == !falling_edge) adc_start(t, false);
}

void Lpc1769::cpu_exception_return(int exc) {
    if (exc >= 16 && irq_level(exc - 16)) cpu.set_pending(exc);
}

void Lpc1769::eint_input(int n, bool level, bool rise, bool fall) {
    bool edge_mode = (extmode >> n) & 1;
    bool pol = (extpolar >> n) & 1;
    bool fire = edge_mode ? (pol ? rise : fall) : (level == pol);
    if (fire) { extint |= 1u << n; irq(18 + n); }
}

bool Lpc1769::powered(uint32_t bit, const char *name) {
    if ((pconp >> bit) & 1) return true;
    warn_once(std::string("pconp") + name,
              std::string("Acceso a ") + name + " con su bit de PCONP en 0: el periférico está apagado (lectura 0, escritura ignorada)");
    return false;
}

// ============================================================ TIMER0-3

void Lpc1769::timer_resched(Timer &tm, int64_t) {
    int64_t best = NEVER_T;
    for (int x = 0; x < 4; x++) {
        uint32_t act = (tm.mcr >> (3 * x)) & 7, emc = (tm.emr >> (4 + 2 * x)) & 3;
        if (!act && !emc) continue;
        int64_t tx = tm.c.time_tc_reaches(tm.mr[x]);
        if (tx < best) best = tx;
    }
    tm.next_t = best;
}

void Lpc1769::timer_event(Timer &tm, int64_t t) {
    uint32_t tc = tm.c.tc_at(t);
    bool reset = false, stop = false, emr_changed = false;
    uint32_t old_ir = tm.ir;
    uint32_t emr_before = tm.emr;
    for (int x = 0; x < 4; x++) {
        uint32_t act = (tm.mcr >> (3 * x)) & 7, emc = (tm.emr >> (4 + 2 * x)) & 3;
        if (!act && !emc) continue;
        if (tm.mr[x] != tc || tm.c.time_tc_reaches(tm.mr[x]) > t) continue;
        if (act & 1) tm.ir |= 1u << x;
        if (act & 2) reset = true;
        if (act & 4) stop = true;
        if (emc) {
            uint32_t bit = 1u << x, before = tm.emr & bit;
            if (emc == 1) tm.emr &= ~bit;
            else if (emc == 2) tm.emr |= bit;
            else tm.emr ^= bit;
            if ((tm.emr & bit) != before) emr_changed = true;
        }
    }
    tm.c.base_t = t;
    tm.c.tc_base = tc;
    tm.c.pc_base = 0;
    tm.c.fresh = false;
    if (reset) {
        tm.c.hold_val = tc;
        tm.c.base_t = t + tm.c.tick_ps();
        tm.c.tc_base = 0;
        tm.c.fresh = true;
    }
    if (stop) {
        tm.tcr &= ~1u;
        if (reset) { tm.c.tc_base = 0; tm.c.base_t = t; tm.c.fresh = false; }
        tm.c.enabled = false;
    }
    if (tm.ir != old_ir) irq(1 + tm.idx);
    if (emr_changed && tm.idx <= 1) {
        static const int src[2][4] = {{-1, 4, -1, 5}, {6, 7, -1, -1}};
        for (int x = 0; x < 4; x++) {
            int code = src[tm.idx][x];
            if (code < 0) continue;
            bool now_hi = (tm.emr >> x) & 1;
            bool was_hi = (emr_before >> x) & 1;
            if (now_hi != was_hi) adc_hw_trigger((uint32_t)code, now_hi, t);
        }
    }
    if (emr_changed) {
        static const PinFunc base[4] = {PF_MAT00, PF_MAT10, PF_MAT20, PF_MAT30};
        static const int count[4] = {2, 2, 4, 2};
        for (int k = 0; k < count[tm.idx]; k++) update_func_pins((PinFunc)(base[tm.idx] + k), t);
    }
    timer_resched(tm, t);
}

void Lpc1769::timer_capture(Timer &tm, int ch, bool rising, int64_t t) {
    uint32_t cfg = (tm.ccr >> (3 * ch)) & 7;
    if ((rising && (cfg & 1)) || (!rising && (cfg & 2))) {
        tm.cr[ch] = tm.c.tc_at(t);
        if (cfg & 4) { tm.ir |= 1u << (4 + ch); irq(1 + tm.idx); }
    }
}

uint32_t Lpc1769::timer_read(Timer &tm, uint32_t off, int64_t t) {
    switch (off) {
    case 0x00: return tm.ir;
    case 0x04: return tm.tcr;
    case 0x08: return tm.c.tc_at(t);
    case 0x0C: return tm.c.pr;
    case 0x10: return tm.c.pc_at(t);
    case 0x14: return tm.mcr;
    case 0x18: case 0x1C: case 0x20: case 0x24: return tm.mr[(off - 0x18) / 4];
    case 0x28: return tm.ccr;
    case 0x2C: return tm.cr[0];
    case 0x30: return tm.cr[1];
    case 0x3C: return tm.emr;
    case 0x70: return tm.ctcr;
    default: return 0;
    }
}

void Lpc1769::timer_write(Timer &tm, uint32_t off, uint32_t v, int64_t t) {
    tm.c.rebase(t);
    switch (off) {
    case 0x00: tm.ir &= ~(v & 0x3F); break;
    case 0x04: {
        bool was = tm.c.enabled;
        tm.tcr = v & 3;
        if (v & 2) { tm.c.tc_base = 0; tm.c.pc_base = 0; tm.c.base_t = t; tm.c.fresh = false; tm.c.reset_on_enable = false; }
        bool en = (v & 1) && !(v & 2);
        if (en && !was) {
            if (tm.c.reset_on_enable) {
                tm.c.hold_val = tm.c.tc_base;
                tm.c.tc_base = 0;
                tm.c.pc_base = 0;
                tm.c.base_t = t + tm.c.tick_ps();
                tm.c.fresh = true;
                tm.c.reset_on_enable = false;
            } else { tm.c.base_t = t; tm.c.fresh = false; }
        }
        if (!en && was) {
            if (t < tm.c.base_t) { tm.c.reset_on_enable = true; tm.c.tc_base = tm.c.hold_val; tm.c.pc_base = 0; tm.c.fresh = false; }
            else { tm.c.tc_base = tm.c.tc_at(t); tm.c.pc_base = tm.c.pc_at(t); }
            tm.c.base_t = t;
        }
        tm.c.enabled = en;
        break;
    }
    case 0x08: tm.c.tc_base = v; tm.c.base_t = t; tm.c.fresh = false; break;
    case 0x0C: tm.c.pr = v; break;
    case 0x10: tm.c.pc_base = v; tm.c.base_t = t; break;
    case 0x14: tm.mcr = v & 0xFFF; break;
    case 0x18: case 0x1C: case 0x20: case 0x24: tm.mr[(off - 0x18) / 4] = v; break;
    case 0x28: tm.ccr = v & 0x3F; break;
    case 0x3C: {
        tm.emr = v & 0xFFF;
        static const PinFunc base[4] = {PF_MAT00, PF_MAT10, PF_MAT20, PF_MAT30};
        static const int count[4] = {2, 2, 4, 2};
        for (int k = 0; k < count[tm.idx]; k++) update_func_pins((PinFunc)(base[tm.idx] + k), t);
        break;
    }
    case 0x70:
        tm.ctcr = v & 0xF;
        if (tm.ctcr & 3) warn_once("ctcr", "TIMER en modo contador (CTCR != 0): no está modelado, cuenta con PCLK");
        break;
    default: break;
    }
    timer_resched(tm, t);
}

// ============================================================ PWM1

void Lpc1769::pwm_set_out(int ch, bool v, int64_t t) {
    if (pwm.out[ch] == v) return;
    pwm.out[ch] = v;
    update_func_pins((PinFunc)(PF_PWM1 + ch - 1), t);
}

void Lpc1769::pwm_cycle_start(int64_t t) {
    for (int k = 0; k < 7; k++)
        if (pwm.ler & (1u << k)) pwm.mr[k] = pwm.mr_sh[k];
    pwm.ler = 0;
    if (!(pwm.tcr & 8)) return;
    for (int n = 1; n <= 6; n++) {
        bool dbl = n >= 2 && ((pwm.pcr >> n) & 1);
        if (!dbl) pwm_set_out(n, pwm.mr[n] != 0, t);
    }
}

void Lpc1769::pwm_resched(int64_t) {
    int64_t best = NEVER_T;
    bool pwm_mode = (pwm.tcr & 8) != 0;
    for (int k = 0; k < 7; k++) {
        uint32_t act = (pwm.mcr >> (3 * k)) & 7;
        if (!act && !pwm_mode) continue;
        int64_t tx = pwm.c.time_tc_reaches(pwm.mr[k]);
        if (tx < best) best = tx;
    }
    if (pwm.c.enabled && pwm.c.fresh && pwm.c.tc_base == 0 && pwm.c.base_t < best) best = pwm.c.base_t;
    pwm.next_t = best;
}

void Lpc1769::pwm_event(int64_t t) {
    uint32_t tc = pwm.c.tc_at(t);
    bool pwm_mode = (pwm.tcr & 8) != 0;
    if (pwm.c.fresh && t >= pwm.c.base_t && tc == 0) pwm_cycle_start(t);
    bool reset = false, stop = false;
    uint32_t old_ir = pwm.ir;
    for (int k = 0; k < 7; k++) {
        if (pwm.mr[k] != tc || pwm.c.time_tc_reaches(pwm.mr[k]) > t) continue;
        uint32_t act = (pwm.mcr >> (3 * k)) & 7;
        if (act & 1) pwm.ir |= (k < 4) ? (1u << k) : (1u << (k + 4));
        if (act & 2) reset = true;
        if (act & 4) stop = true;
        if (pwm_mode) {
            if (k >= 1) pwm_set_out(k, false, t);
            if (k >= 1 && k + 1 <= 6 && ((pwm.pcr >> (k + 1)) & 1)) pwm_set_out(k + 1, true, t);
        }
    }
    pwm.c.base_t = t;
    pwm.c.tc_base = tc;
    pwm.c.pc_base = 0;
    pwm.c.fresh = false;
    if (reset) {
        pwm.c.hold_val = tc;
        pwm.c.base_t = t + pwm.c.tick_ps();
        pwm.c.tc_base = 0;
        pwm.c.fresh = true;
    }
    if (stop) { pwm.tcr &= ~1u; pwm.c.enabled = false; }
    if (pwm.ir != old_ir) irq(9);
    pwm_resched(t);
}

uint32_t Lpc1769::pwm_read(uint32_t off, int64_t t) {
    switch (off) {
    case 0x00: return pwm.ir;
    case 0x04: return pwm.tcr;
    case 0x08: return pwm.c.tc_at(t);
    case 0x0C: return pwm.c.pr;
    case 0x10: return pwm.c.pc_at(t);
    case 0x14: return pwm.mcr;
    case 0x18: case 0x1C: case 0x20: case 0x24: return pwm.mr_sh[(off - 0x18) / 4];
    case 0x28: return pwm.ccr;
    case 0x2C: case 0x30: case 0x34: case 0x38: return pwm.cr[(off - 0x2C) / 4];
    case 0x40: case 0x44: case 0x48: return pwm.mr_sh[4 + (off - 0x40) / 4];
    case 0x4C: return pwm.pcr;
    case 0x50: return pwm.ler;
    case 0x70: return pwm.ctcr;
    default: return 0;
    }
}

void Lpc1769::pwm_write(uint32_t off, uint32_t v, int64_t t) {
    pwm.c.rebase(t);
    bool pwm_mode = (pwm.tcr & 8) != 0;
    auto set_mr = [&](int k) {
        pwm.mr_sh[k] = v;
        if (!pwm_mode || !pwm.c.enabled) pwm.mr[k] = v;
    };
    switch (off) {
    case 0x00: pwm.ir &= ~(v & 0x73F); break;
    case 0x04: {
        bool was = pwm.c.enabled;
        pwm.tcr = v & 0xB;
        if (v & 2) { pwm.c.tc_base = 0; pwm.c.pc_base = 0; pwm.c.base_t = t; pwm.c.fresh = false; }
        bool en = (v & 1) && !(v & 2);
        if (en && !was && pwm.c.reset_on_enable) {
            pwm.c.reset_on_enable = false;
            pwm.c.tc_base = 0;
            pwm.c.pc_base = 0;
        }
        if (en && !was) {
            pwm.c.base_t = t;
            pwm.c.fresh = false;
            for (int k = 0; k < 7; k++) pwm.mr[k] = pwm.mr_sh[k];
            pwm.ler = 0;
            pwm.c.enabled = true;
            if (pwm.tcr & 8) pwm_cycle_start(t);
        }
        if (!en && was) {
            if (t < pwm.c.base_t) { pwm.c.reset_on_enable = true; pwm.c.tc_base = pwm.c.hold_val; pwm.c.pc_base = 0; pwm.c.fresh = false; }
            else { pwm.c.tc_base = pwm.c.tc_at(t); pwm.c.pc_base = pwm.c.pc_at(t); }
            pwm.c.base_t = t;
        }
        pwm.c.enabled = en;
        if (!(pwm.tcr & 8)) for (int n = 1; n <= 6; n++) pwm_set_out(n, false, t);
        break;
    }
    case 0x08: pwm.c.tc_base = v; pwm.c.base_t = t; pwm.c.fresh = false; break;
    case 0x0C: pwm.c.pr = v; break;
    case 0x10: pwm.c.pc_base = v; pwm.c.base_t = t; break;
    case 0x14: pwm.mcr = v & 0x1FFFFF; break;
    case 0x18: case 0x1C: case 0x20: case 0x24: set_mr((int)(off - 0x18) / 4); break;
    case 0x28: pwm.ccr = v & 0xFFF; break;
    case 0x40: case 0x44: case 0x48: set_mr(4 + (int)(off - 0x40) / 4); break;
    case 0x4C:
        pwm.pcr = v & 0x7E7C;
        for (int n = 1; n <= 6; n++) update_func_pins((PinFunc)(PF_PWM1 + n - 1), t);
        break;
    case 0x50:
        pwm.ler = v & 0x7F;
        if (!pwm.c.enabled) { for (int k = 0; k < 7; k++) if (pwm.ler & (1u << k)) pwm.mr[k] = pwm.mr_sh[k]; pwm.ler = 0; }
        break;
    case 0x70: pwm.ctcr = v & 0xF; if (v & 3) warn_once("pwmctcr", "PWM1 en modo contador: no está modelado"); break;
    default: break;
    }
    pwm_resched(t);
}

// ============================================================ ADC

void Lpc1769::adc_start(int64_t t, bool from_burst) {
    uint32_t sel = adc.cr & 0xFF;
    if (!sel) { warn_once("adcsel", "ADC: START sin ningún canal en SEL"); adc.done_t = NEVER_T; return; }
    int ch;
    if (from_burst && adc.ch >= 0) {
        ch = -1;
        for (int k = 1; k <= 8; k++) { int c = (adc.ch + k) & 7; if (sel & (1u << c)) { ch = c; break; } }
    } else {
        ch = __builtin_ctz(sel);
        if (!(adc.cr & (1u << 16)) && (sel & (sel - 1)))
            warn_once("adcmulti", "ADC: con START por software hay que seleccionar un solo canal en SEL; se convierte el más bajo");
    }
    double padc = pclk(0, 12);
    double fadc = padc / (double)(((adc.cr >> 8) & 0xFF) + 1);
    if (fadc > 13e6 + 1) {
        char b[140];
        snprintf(b, sizeof(b), "ADC: reloj de %.2f MHz, por encima del máximo de 13 MHz (CLKDIV demasiado chico)", fadc / 1e6);
        warn_once("adcclk", b);
    }
    adc.ch = ch;
    adc.done_t = t + (int64_t)(65.0 * 1e12 / fadc);
}

void Lpc1769::adc_event(int64_t t) {
    static const int pins[8][2] = {{0, 23}, {0, 24}, {0, 25}, {0, 26}, {1, 30}, {1, 31}, {0, 3}, {0, 2}};
    int ch = adc.ch;
    int port = pins[ch][0], bit = pins[ch][1];
    double v = 0;
    if (func_of(port, bit) != (PinFunc)(PF_AD0 + ch)) {
        char b[160];
        snprintf(b, sizeof(b), "ADC: se convirtió AD0.%d pero P%d.%d no está configurado como AD0.%d en PINSEL (resultado 0)", ch, port, bit, ch);
        warn_once("adcpin" + std::to_string(ch), b);
    } else {
        v = host->analog_input(port, bit);
    }
    long r = std::lround(v / vref * 4095.0);
    if (r < 0) r = 0;
    if (r > 4095) r = 4095;
    uint32_t ovr = (adc.dr[ch] >> 31) & 1;
    adc.dr[ch] = ((uint32_t)r << 4) | (1u << 31) | (ovr << 30);
    adc.gdr = ((uint32_t)r << 4) | ((uint32_t)ch << 24) | (1u << 31) | (((adc.gdr >> 31) & 1) << 30);
    adc_update_irq();
    if ((adc.cr & (1u << 16)) && (adc.cr & (1u << 21))) adc_start(t, true);
    else adc.done_t = NEVER_T;
}

void Lpc1769::adc_update_irq() { if (irq_level(22)) irq(22); }

// ============================================================ UART

double Lpc1769::uart_bit_ps(const Uart &u) const {
    static const int sel[4][2] = {{0, 3}, {0, 4}, {1, 8}, {1, 9}};
    double pc = pclk(sel[u.idx][0], sel[u.idx][1]);
    double dl = (double)(u.dlm * 256 + u.dll);
    if (dl < 1) dl = 1;
    double divadd = (double)(u.fdr & 0xF), mul = (double)((u.fdr >> 4) & 0xF);
    double frac = (divadd > 0 && mul > 0) ? (1.0 + divadd / mul) : 1.0;
    double baud = pc / (16.0 * dl * frac);
    return 1e12 / baud;
}

int Lpc1769::uart_frame_bits(const Uart &u) const {
    int data = 5 + (int)(u.lcr & 3);
    int par = (u.lcr & 8) ? 1 : 0;
    int stop = (u.lcr & 4) ? 2 : 1;
    return 1 + data + par + stop;
}

uint32_t Lpc1769::uart_iir(Uart &u) {
    int trig[4] = {1, 4, 8, 14};
    int level = u.fifo_en ? trig[(u.fcr >> 6) & 3] : 1;
    uint32_t id = 1;
    if ((u.ier & 4) && (u.lsr_err & 0x1E)) id = 0x6;
    else if ((u.ier & 1) && u.rx_count >= level) id = 0x4;
    else if ((u.ier & 1) && u.cti && u.rx_count > 0) id = 0xC;
    else if ((u.ier & 2) && u.thre_int) id = 0x2;
    return id | (u.fifo_en ? 0xC0 : 0);
}

void Lpc1769::uart_update_irq(Uart &u) {
    if ((uart_iir(u) & 1) == 0) irq(5 + u.idx);
}

void Lpc1769::uart_tx_start(Uart &u, int64_t t) {
    if (u.tx_busy || u.tx_count == 0 || !(u.ter & 0x80)) return;
    uint8_t byte = u.tx_fifo[u.tx_head];
    u.tx_head = (u.tx_head + 1) & 15;
    u.tx_count--;
    int data = 5 + (int)(u.lcr & 3);
    uint32_t frame = 0;
    int n = 0;
    frame |= 0u << n++;
    int ones = 0;
    for (int i = 0; i < data; i++) { uint32_t b = (byte >> i) & 1; ones += (int)b; frame |= b << n++; }
    if (u.lcr & 8) {
        uint32_t p;
        switch ((u.lcr >> 4) & 3) {
        case 0: p = (ones & 1) ? 0 : 1; break;
        case 1: p = (ones & 1) ? 1 : 0; break;
        case 2: p = 1; break;
        default: p = 0; break;
        }
        frame |= p << n++;
    }
    int stop = (u.lcr & 4) ? 2 : 1;
    for (int i = 0; i < stop; i++) frame |= 1u << n++;
    u.tx_frame = frame;
    u.tx_bits = n;
    u.tx_pos = 0;
    u.tx_busy = true;
    u.txd = (u.lcr & 0x40) ? false : (frame & 1);
    update_func_pins((PinFunc)(PF_TXD0 + u.idx), t);
    u.tx_t = t + (int64_t)uart_bit_ps(u);
    if (u.tx_count == 0) { u.thre_int = true; uart_update_irq(u); }
}

void Lpc1769::uart_event(Uart &u, int64_t t) {
    if (u.tx_t <= t) {
        u.tx_pos++;
        if (u.tx_pos < u.tx_bits) {
            u.txd = (u.lcr & 0x40) ? false : ((u.tx_frame >> u.tx_pos) & 1);
            update_func_pins((PinFunc)(PF_TXD0 + u.idx), t);
            u.tx_t += (int64_t)uart_bit_ps(u);
        } else {
            u.tx_busy = false;
            u.tx_t = NEVER_T;
            u.txd = !(u.lcr & 0x40);
            update_func_pins((PinFunc)(PF_TXD0 + u.idx), t);
            uart_tx_start(u, t);
        }
    }
    if (u.rx_t <= t) {
        bool lvl = u.rxd_level;
        int data = 5 + (int)(u.lcr & 3);
        int par = (u.lcr & 8) ? 1 : 0;
        double bit = uart_bit_ps(u);
        if (u.rx_pos == 0) {
            if (lvl) { u.rx_busy = false; u.rx_t = NEVER_T; }
            else { u.rx_frame = 0; u.rx_pos = 1; u.rx_t = u.rx_start + (int64_t)(1.5 * bit); }
        } else if (u.rx_pos <= data + par) {
            if (lvl) u.rx_frame |= 1u << (u.rx_pos - 1);
            u.rx_pos++;
            u.rx_t = u.rx_start + (int64_t)((u.rx_pos + 0.5) * bit);
        } else {
            uint32_t byte = u.rx_frame & ((1u << data) - 1);
            if (par) {
                int ones = __builtin_popcount(byte);
                uint32_t pbit = (u.rx_frame >> data) & 1, exp;
                switch ((u.lcr >> 4) & 3) {
                case 0: exp = (ones & 1) ? 0 : 1; break;
                case 1: exp = (ones & 1) ? 1 : 0; break;
                case 2: exp = 1; break;
                default: exp = 0; break;
                }
                if (pbit != exp) u.lsr_err |= 0x4;
            }
            if (!lvl) {
                u.lsr_err |= 0x8;
                if (u.rx_frame == 0) u.lsr_err |= 0x10;
            }
            int cap = u.fifo_en ? 16 : 1;
            if (u.rx_count < cap) {
                u.rx_fifo[(u.rx_head + u.rx_count) & 15] = (uint8_t)byte;
                u.rx_count++;
            } else {
                u.lsr_err |= 0x2;
                warn_once("uartovr" + std::to_string(u.idx), "UART" + std::to_string(u.idx) + ": overrun, el firmware no leyó a tiempo el RBR");
            }
            u.cti = false;
            u.cti_t = t + (int64_t)(4.0 * uart_frame_bits(u) * bit);
            u.rx_busy = false;
            u.rx_t = NEVER_T;
            uart_update_irq(u);
            if (!u.rxd_level) uart_rx_edge(u, false, t);
        }
    }
    if (u.cti_t <= t) {
        u.cti_t = NEVER_T;
        if (u.rx_count > 0) { u.cti = true; uart_update_irq(u); }
    }
}

void Lpc1769::uart_rx_edge(Uart &u, bool level, int64_t t) {
    u.rxd_level = level;
    if (!u.rx_busy && !level) {
        u.rx_busy = true;
        u.rx_start = t;
        u.rx_pos = 0;
        u.rx_t = t + (int64_t)(0.5 * uart_bit_ps(u));
    }
}

uint32_t Lpc1769::uart_read(Uart &u, uint32_t off, int64_t t) {
    bool dlab = (u.lcr & 0x80) != 0;
    switch (off) {
    case 0x00: {
        if (dlab) return u.dll;
        if (u.rx_count == 0) return 0;
        uint8_t b = u.rx_fifo[u.rx_head];
        u.rx_head = (u.rx_head + 1) & 15;
        u.rx_count--;
        u.cti = false;
        if (u.rx_count > 0) u.cti_t = t + (int64_t)(4.0 * uart_frame_bits(u) * uart_bit_ps(u));
        else u.cti_t = NEVER_T;
        cpu.stop_request = true;
        return b;
    }
    case 0x04: return dlab ? u.dlm : u.ier;
    case 0x08: {
        uint32_t id = uart_iir(u);
        if ((id & 0xF) == 0x2) u.thre_int = false;
        return id;
    }
    case 0x0C: return u.lcr;
    case 0x14: {
        uint32_t v = 0;
        if (u.rx_count) v |= 1;
        v |= u.lsr_err & 0x1E;
        if (u.tx_count == 0) v |= 0x20;
        if (u.tx_count == 0 && !u.tx_busy) v |= 0x40;
        if (u.lsr_err & 0x1C) v |= 0x80;
        u.lsr_err = 0;
        return v;
    }
    case 0x1C: return u.scr;
    case 0x20: return u.acr;
    case 0x24: return u.icr;
    case 0x28: return u.fdr;
    case 0x30: return u.ter;
    default: return 0;
    }
}

void Lpc1769::uart_write(Uart &u, uint32_t off, uint32_t v, int64_t t) {
    bool dlab = (u.lcr & 0x80) != 0;
    switch (off) {
    case 0x00:
        if (dlab) { u.dll = v & 0xFF; break; }
        {
            int cap = u.fifo_en ? 16 : 1;
            if (u.tx_count < cap) {
                u.tx_fifo[(u.tx_head + u.tx_count) & 15] = (uint8_t)v;
                u.tx_count++;
            } else {
                warn_once("uarttxovf" + std::to_string(u.idx), "UART" + std::to_string(u.idx) + ": se escribió THR con la FIFO de transmisión llena (dato perdido)");
            }
            u.thre_int = false;
            uart_tx_start(u, t);
        }
        break;
    case 0x04:
        if (dlab) { u.dlm = v & 0xFF; break; }
        {
            bool thre_on = !(u.ier & 2) && (v & 2);
            u.ier = v & 0x307;
            if (thre_on && u.tx_count == 0) u.thre_int = true;
        }
        break;
    case 0x08:
        u.fcr = v & 0xC7;
        u.fifo_en = v & 1;
        if (v & 2) { u.rx_count = 0; u.rx_head = 0; u.cti = false; }
        if (v & 4) { u.tx_count = 0; u.tx_head = 0; }
        if (!(v & 1)) warn_once("uartfifo" + std::to_string(u.idx), "UART" + std::to_string(u.idx) + ": FCR con FIFO deshabilitada; el LPC17xx necesita FCR bit 0 en 1 para operar normalmente");
        break;
    case 0x0C: u.lcr = v & 0xFF; break;
    case 0x1C: u.scr = v & 0xFF; break;
    case 0x20: u.acr = v; break;
    case 0x24: u.icr = v; break;
    case 0x28: u.fdr = v & 0xFF; break;
    case 0x30: u.ter = v & 0x80; uart_tx_start(u, t); break;
    default: break;
    }
    uart_update_irq(u);
}

// ============================================================ SysTick

uint32_t Lpc1769::systick_val(int64_t t) const {
    if (!(st_ctrl & 1)) return st_val0;
    int64_t n = (int64_t)((double)(t - st_base) / ps_per_cycle + 1e-9);
    if (n <= (int64_t)st_val0) return st_val0 - (uint32_t)n;
    if (st_load == 0) return 0;
    uint64_t m = (uint64_t)(n - (int64_t)st_val0 - 1);
    return st_load - (uint32_t)(m % ((uint64_t)st_load + 1));
}

void Lpc1769::systick_resched(int64_t t) {
    if (!(st_ctrl & 1) || st_load == 0) { st_next = NEVER_T; return; }
    st_val0 = systick_val(t);
    st_base = t;
    uint64_t cycles = st_val0 >= 1 ? st_val0 : (uint64_t)st_load + 1;
    st_next = t + (int64_t)std::ceil((double)cycles * ps_per_cycle - 1e-6);
}

void Lpc1769::systick_event(int64_t t) {
    st_countflag = true;
    if (st_ctrl & 2) cpu.set_pending(Cpu::EXC_SYSTICK);
    st_val0 = 0;
    st_base = t;
    if (st_load == 0) { st_next = NEVER_T; return; }
    st_next = t + (int64_t)std::ceil(((double)st_load + 1.0) * ps_per_cycle - 1e-6);
}

// ============================================================ entradas

void Lpc1769::input_edge(int64_t t, int port, int bit, bool level) {
    Edge e{t, port, bit, level};
    if (inputs.empty() || inputs.back().t <= t) inputs.push_back(e);
    else {
        auto it = std::upper_bound(inputs.begin(), inputs.end(), e, [](const Edge &a, const Edge &b) { return a.t < b.t; });
        inputs.insert(it, e);
    }
}

void Lpc1769::process_input(const Edge &e) {
    uint32_t m = 1u << e.bit;
    bool old = (in_level[e.port] & m) != 0;
    if (e.level) in_level[e.port] |= m; else in_level[e.port] &= ~m;
    if (old == e.level) return;
    bool rise = e.level, fall = !e.level;
    PinFunc f = func_of(e.port, e.bit);
    switch (f) {
    case PF_CAP00: case PF_CAP01: timer_capture(tim[0], f - PF_CAP00, rise, e.t); break;
    case PF_CAP10: case PF_CAP11: timer_capture(tim[1], f - PF_CAP10, rise, e.t); break;
    case PF_CAP20: case PF_CAP21: timer_capture(tim[2], f - PF_CAP20, rise, e.t); break;
    case PF_CAP30: case PF_CAP31: timer_capture(tim[3], f - PF_CAP30, rise, e.t); break;
    case PF_RXD0: case PF_RXD1: case PF_RXD2: case PF_RXD3: uart_rx_edge(uart[f - PF_RXD0], e.level, e.t); break;
    case PF_EINT0: case PF_EINT1: case PF_EINT2: case PF_EINT3: eint_input(f - PF_EINT0, e.level, rise, fall); break;
    default: break;
    }
    if (e.port == 2 && e.bit == 10) adc_hw_trigger(2, rise, e.t);
    if (e.port == 1 && e.bit == 27) adc_hw_trigger(3, rise, e.t);
    if (e.port == 0 || e.port == 2) {
        bool any = false;
        if (rise && (io_int_en_r[e.port] & m)) { io_int_stat_r[e.port] |= m; any = true; }
        if (fall && (io_int_en_f[e.port] & m)) { io_int_stat_f[e.port] |= m; any = true; }
        if (any) irq(21);
    }
    if (((pinmode[e.port * 2 + (e.bit >> 4)] >> ((e.bit & 15) * 2)) & 3) == 1) update_pin(e.port, e.bit, e.t);
}

// ============================================================ planificación

int64_t Lpc1769::next_event_time() const {
    int64_t t = st_next;
    for (int i = 0; i < 4; i++) {
        t = std::min(t, tim[i].next_t);
        t = std::min(t, uart[i].tx_t);
        t = std::min(t, uart[i].rx_t);
        t = std::min(t, uart[i].cti_t);
    }
    t = std::min(t, pwm.next_t);
    t = std::min(t, adc.done_t);
    if (!inputs.empty()) t = std::min(t, inputs.front().t);
    return t;
}

void Lpc1769::process_due(int64_t t) {
    for (int guard = 0; guard < 100000; guard++) {
        int64_t te = next_event_time();
        if (te > t) return;
        if (!inputs.empty() && inputs.front().t == te) {
            Edge e = inputs.front();
            inputs.pop_front();
            process_input(e);
            continue;
        }
        if (st_next == te) { systick_event(te); continue; }
        bool done = false;
        for (int i = 0; i < 4 && !done; i++) {
            if (tim[i].next_t == te) { timer_event(tim[i], te); done = true; }
            else if (uart[i].tx_t == te || uart[i].rx_t == te || uart[i].cti_t == te) { uart_event(uart[i], te); done = true; }
        }
        if (done) continue;
        if (pwm.next_t == te) { pwm_event(te); continue; }
        if (adc.done_t == te) { adc_event(te); continue; }
        return;
    }
    warn_once("evloop", "Demasiados eventos en el mismo instante: posible configuración degenerada de un timer (MR = 0 con reset)");
}

void Lpc1769::run_until(int64_t t_end) {
    for (int guard = 0; guard < 10000000; guard++) {
        if (reset_requested) { reset_requested = false; reset(now()); }
        int64_t t = now();
        process_due(t);
        if (t >= t_end) return;
        int64_t tn = std::min(next_event_time(), t_end);
        if (tn <= t) tn = t + 1;
        if (cpu.sleeping && cpu.can_take_exception()) cpu.sleeping = false;
        uint64_t budget = (uint64_t)std::ceil((double)(tn - t) / ps_per_cycle - 1e-9);
        if (budget == 0) budget = 1;
        if (in_reset || cpu.lockup || cpu.sleeping) { cpu.cycles += budget; continue; }
        cpu.run(budget);
    }
}

// ============================================================ bus de periféricos

static uint32_t lane_mask(uint32_t addr, int size) {
    uint32_t m = size == 4 ? 0xFFFFFFFFu : size == 2 ? 0xFFFFu : 0xFFu;
    return m << ((addr & 3) * 8);
}

bool Lpc1769::io_read(uint32_t addr, int size, uint32_t &value) {
    int64_t t = now();
    uint32_t shift = (addr & 3) * 8;
    uint32_t mask = size == 4 ? 0xFFFFFFFFu : size == 2 ? 0xFFFFu : 0xFFu;
    uint32_t v = 0;
    if (addr >= 0x2009C000u && addr < 0x2009C0A0u) {
        int port = (int)((addr - 0x2009C000u) >> 5);
        uint32_t reg = (addr - 0x2009C000u) & 0x1C;
        switch (reg) {
        case 0x00: v = fiodir[port]; break;
        case 0x10: v = fiomask[port]; break;
        case 0x14: v = fiopin_read(port) & ~fiomask[port]; break;
        case 0x18: v = fioout[port] & ~fiomask[port]; break;
        default: v = 0; break;
        }
        value = (v >> shift) & mask;
        return true;
    }
    if (addr >= 0x40000000u && addr < 0x40100000u) {
        int slot = (int)((addr - 0x40000000u) >> 14);
        uint32_t off = (addr & 0x3FFF) & ~3u;
        switch (slot) {
        case 1: case 2: case 36: case 37: {
            int i = slot == 1 ? 0 : slot == 2 ? 1 : slot == 36 ? 2 : 3;
            static const uint32_t bits[4] = {1, 2, 22, 23};
            static const char *names[4] = {"TIMER0", "TIMER1", "TIMER2", "TIMER3"};
            v = powered(bits[i], names[i]) ? timer_read(tim[i], off, t) : 0;
            break;
        }
        case 3: case 4: case 38: case 39: {
            int i = slot == 3 ? 0 : slot == 4 ? 1 : slot == 38 ? 2 : 3;
            static const uint32_t bits[4] = {3, 4, 24, 25};
            static const char *names[4] = {"UART0", "UART1", "UART2", "UART3"};
            v = powered(bits[i], names[i]) ? uart_read(uart[i], off, t) : 0;
            break;
        }
        case 6: v = powered(6, "PWM1") ? pwm_read(off, t) : 0; break;
        case 10: {
            switch (off) {
            case 0x80: v = ((io_int_stat_r[0] | io_int_stat_f[0]) ? 1 : 0) | ((io_int_stat_r[2] | io_int_stat_f[2]) ? 4 : 0); break;
            case 0x84: v = io_int_stat_r[0]; break;
            case 0x88: v = io_int_stat_f[0]; break;
            case 0x90: v = io_int_en_r[0]; break;
            case 0x94: v = io_int_en_f[0]; break;
            case 0xA4: v = io_int_stat_r[2]; break;
            case 0xA8: v = io_int_stat_f[2]; break;
            case 0xB0: v = io_int_en_r[2]; break;
            case 0xB4: v = io_int_en_f[2]; break;
            default: v = 0; break;
            }
            break;
        }
        case 11:
            if (off <= 0x28) v = pinsel[off / 4];
            else if (off >= 0x40 && off <= 0x64) v = pinmode[(off - 0x40) / 4];
            else if (off >= 0x68 && off <= 0x78) v = pinmode_od[(off - 0x68) / 4];
            break;
        case 13:
            if (!powered(12, "ADC")) { v = 0; break; }
            switch (off) {
            case 0x00: v = adc.cr; break;
            case 0x04: v = adc.gdr; adc.gdr &= ~(3u << 30); adc_update_irq(); cpu.stop_request = true; break;
            case 0x0C: v = adc.inten; break;
            case 0x30: {
                uint32_t done = 0, ovr = 0;
                for (int i = 0; i < 8; i++) { done |= ((adc.dr[i] >> 31) & 1) << i; ovr |= ((adc.dr[i] >> 30) & 1) << i; }
                v = done | (ovr << 8) | (irq_level(22) ? (1u << 16) : 0);
                break;
            }
            case 0x34: v = adc.trm; break;
            default:
                if (off >= 0x10 && off <= 0x2C) {
                    int ch = (int)(off - 0x10) / 4;
                    v = adc.dr[ch];
                    adc.dr[ch] &= ~(3u << 30);
                    cpu.stop_request = true;
                }
                break;
            }
            break;
        case 63:
            switch (off) {
            case 0x000: v = flashcfg; break;
            case 0x080: v = pll0con; break;
            case 0x084: v = pll0cfg; break;
            case 0x088: v = (pll0cfg_act & 0x7FFF) | (((pll0cfg_act >> 16) & 0xFF) << 16) | ((pll0con_act & 1) << 24) |
                             (((pll0con_act >> 1) & 1) << 25) | ((pll0con_act & 1) << 26); break;
            case 0x0A0: v = pll1con; break;
            case 0x0A4: v = pll1cfg; break;
            case 0x0A8: v = (pll1cfg_act & 0x7F) | ((pll1con_act & 1) << 8) | (((pll1con_act >> 1) & 1) << 9) | ((pll1con_act & 1) << 10); break;
            case 0x0C0: v = pcon; break;
            case 0x0C4: v = pconp; break;
            case 0x104: v = cclkcfg; break;
            case 0x108: v = usbclkcfg; break;
            case 0x10C: v = clksrcsel; break;
            case 0x140: v = extint; break;
            case 0x148: v = extmode; break;
            case 0x14C: v = extpolar; break;
            case 0x180: v = 1; break;
            case 0x1A0: v = (scs & 0x30) | ((scs & 0x20) ? 0x40 : 0); break;
            case 0x1A8: v = pclksel0; break;
            case 0x1AC: v = pclksel1; break;
            case 0x1C8: v = clkoutcfg; break;
            default: v = 0; break;
            }
            break;
        case 5: case 19: case 20: case 21: case 22: case 24: case 25: case 26: case 27:
        case 28: case 29: case 30: case 31: case 32: case 33: case 41: case 43: case 45: case 48: case 49: case 50: case 51:
        case 52: case 53: case 54: case 55: case 56: case 57: case 58: case 59: case 60: case 61: case 62:
            return false;
        default: {
            static const char *names[64] = {"WDT", 0, 0, 0, 0, 0, 0, "I2C0", "SPI", "RTC", 0, 0, "SSP1", 0, "CAN", "CAN", "CAN", "CAN", "CAN", 0,
                                            0, 0, 0, "I2C1", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "SSP0", "DAC", 0, 0, 0, 0, "I2C2", 0,
                                            "I2S", 0, "RIT", 0, "MCPWM", "QEI"};
            const char *n = (slot < 64 && names[slot]) ? names[slot] : "periférico";
            warn_once(std::string("unm") + n, std::string(n) + " no está modelado: las lecturas devuelven 0");
            v = 0;
            break;
        }
        }
        value = (v >> shift) & mask;
        return true;
    }
    if (addr >= 0x50000000u && addr < 0x50200000u) {
        warn_once("ahb", "Periféricos AHB (Ethernet, USB, GPDMA) no modelados: las lecturas devuelven 0");
        value = 0;
        return true;
    }
    if (addr >= 0xE000E010u && addr < 0xE000E020u) {
        uint32_t off = (addr - 0xE000E010u) & ~3u;
        switch (off) {
        case 0x0: v = st_ctrl | (st_countflag ? 0x10000u : 0); st_countflag = false; break;
        case 0x4: v = st_load; break;
        case 0x8: v = systick_val(t); break;
        default: v = 0x000F423F; break;
        }
        value = (v >> shift) & mask;
        return true;
    }
    if (addr >= 0xE0000000u && addr < 0xE0100000u) { value = 0; return true; }
    return false;
}

bool Lpc1769::io_write(uint32_t addr, int size, uint32_t value) {
    int64_t t = now();
    uint32_t shift = (addr & 3) * 8;
    uint32_t v = value << shift;
    cpu.stop_request = true;
    if (addr >= 0x2009C000u && addr < 0x2009C0A0u) {
        int port = (int)((addr - 0x2009C000u) >> 5);
        uint32_t reg = (addr - 0x2009C000u) & 0x1C;
        uint32_t lane = lane_mask(addr, size);
        uint32_t wm = lane & ~fiomask[port];
        switch (reg) {
        case 0x00: fiodir[port] = (fiodir[port] & ~lane) | (v & lane); break;
        case 0x10: fiomask[port] = (fiomask[port] & ~lane) | (v & lane); break;
        case 0x14: fioout[port] = (fioout[port] & ~wm) | (v & wm); break;
        case 0x18: fioout[port] |= v & wm; break;
        case 0x1C: fioout[port] &= ~(v & wm); break;
        default: break;
        }
        update_port(port, t);
        return true;
    }
    if (addr >= 0x40000000u && addr < 0x40100000u) {
        int slot = (int)((addr - 0x40000000u) >> 14);
        uint32_t off = (addr & 0x3FFF) & ~3u;
        switch (slot) {
        case 1: case 2: case 36: case 37: {
            int i = slot == 1 ? 0 : slot == 2 ? 1 : slot == 36 ? 2 : 3;
            static const uint32_t bits[4] = {1, 2, 22, 23};
            static const char *names[4] = {"TIMER0", "TIMER1", "TIMER2", "TIMER3"};
            if (powered(bits[i], names[i])) timer_write(tim[i], off, v, t);
            return true;
        }
        case 3: case 4: case 38: case 39: {
            int i = slot == 3 ? 0 : slot == 4 ? 1 : slot == 38 ? 2 : 3;
            static const uint32_t bits[4] = {3, 4, 24, 25};
            static const char *names[4] = {"UART0", "UART1", "UART2", "UART3"};
            if (powered(bits[i], names[i])) uart_write(uart[i], off, v, t);
            return true;
        }
        case 6: if (powered(6, "PWM1")) pwm_write(off, v, t); return true;
        case 10:
            switch (off) {
            case 0x8C: io_int_stat_r[0] &= ~v; io_int_stat_f[0] &= ~v; break;
            case 0x90: io_int_en_r[0] = v; break;
            case 0x94: io_int_en_f[0] = v; break;
            case 0xAC: io_int_stat_r[2] &= ~v; io_int_stat_f[2] &= ~v; break;
            case 0xB0: io_int_en_r[2] = v; break;
            case 0xB4: io_int_en_f[2] = v; break;
            default: break;
            }
            return true;
        case 11: {
            if (off <= 0x28) pinsel[off / 4] = v;
            else if (off >= 0x40 && off <= 0x64) pinmode[(off - 0x40) / 4] = v;
            else if (off >= 0x68 && off <= 0x78) pinmode_od[(off - 0x68) / 4] = v;
            for (int i = 0; i < 4; i++) {
                for (int p = 0; p < 5; p++)
                    for (int b = 0; b < 32; b++)
                        if (func_of(p, b) == (PinFunc)(PF_RXD0 + i) && !uart[i].rx_busy) uart[i].rxd_level = pin_level(p, b);
            }
            update_all_pins(t);
            eint_recheck();
            return true;
        }
        case 13:
            if (!powered(12, "ADC")) return true;
            switch (off) {
            case 0x00: {
                adc.cr = v & 0x0F21FFFF;
                adc.gdr &= ~(3u << 30);
                adc_update_irq();
                if (!(v & (1u << 21))) { adc.done_t = NEVER_T; break; }
                uint32_t start = (v >> 24) & 7;
                if (v & (1u << 16)) { if (start) warn_once("adcburst", "ADC: con BURST=1 el campo START debe ser 000"); if (adc.done_t == NEVER_T) adc_start(t, false); }
                else if (start == 1) adc_start(t, false);
                else if (start == 3) warn_once("adccap", "ADC: START por CAP0.1 depende del flanco en P1.27"); 
                break;
            }
            case 0x0C: adc.inten = v & 0x1FF; adc_update_irq(); break;
            case 0x34: adc.trm = v; break;
            default: break;
            }
            return true;
        case 63:
            switch (off) {
            case 0x000: flashcfg = v; break;
            case 0x080: pll0con = v & 3; break;
            case 0x084: pll0cfg = v & 0x00FF7FFF; break;
            case 0x08C:
                if ((v & 0xFF) == 0xAA) pll0feed = 1;
                else if ((v & 0xFF) == 0x55 && pll0feed == 1) {
                    pll0feed = 0;
                    pll0con_act = pll0con;
                    pll0cfg_act = pll0cfg;
                    update_cclk();
                } else pll0feed = 0;
                break;
            case 0x0A0: pll1con = v & 3; break;
            case 0x0A4: pll1cfg = v & 0x7F; break;
            case 0x0AC:
                if ((v & 0xFF) == 0xAA) pll1feed = 1;
                else if ((v & 0xFF) == 0x55 && pll1feed == 1) { pll1feed = 0; pll1con_act = pll1con; pll1cfg_act = pll1cfg; }
                else pll1feed = 0;
                break;
            case 0x0C0: pcon = v; break;
            case 0x0C4: pconp = v; break;
            case 0x104: cclkcfg = v & 0xFF; update_cclk(); break;
            case 0x108: usbclkcfg = v & 0xF; break;
            case 0x10C: clksrcsel = v & 3; update_cclk(); break;
            case 0x140:
                extint &= ~(v & 0xF);
                for (int n = 0; n < 4; n++) {
                    if (((extmode >> n) & 1) == 0) {
                        for (int p = 0; p < 5; p++) for (int b = 0; b < 32; b++)
                            if (func_of(p, b) == (PinFunc)(PF_EINT0 + n)) eint_input(n, pin_level(p, b), false, false);
                    }
                }
                break;
            case 0x148: extmode = v & 0xF; eint_recheck(); break;
            case 0x14C: extpolar = v & 0xF; eint_recheck(); break;
            case 0x1A0: scs = v & 0x30; break;
            case 0x1A8: { int64_t tt = t; for (int i = 0; i < 4; i++) tim[i].c.rebase(tt); pwm.c.rebase(tt); pclksel0 = v; rebase_peripherals(tt); break; }
            case 0x1AC: { int64_t tt = t; for (int i = 0; i < 4; i++) tim[i].c.rebase(tt); pwm.c.rebase(tt); pclksel1 = v; rebase_peripherals(tt); break; }
            case 0x1C8: clkoutcfg = v; break;
            default: break;
            }
            return true;
        case 5: case 19: case 20: case 21: case 22: case 24: case 25: case 26: case 27:
        case 28: case 29: case 30: case 31: case 32: case 33: case 41: case 43: case 45: case 48: case 49: case 50: case 51:
        case 52: case 53: case 54: case 55: case 56: case 57: case 58: case 59: case 60: case 61: case 62:
            return false;
        default: {
            static const char *names[64] = {"WDT", 0, 0, 0, 0, 0, 0, "I2C0", "SPI", "RTC", 0, 0, "SSP1", 0, "CAN", "CAN", "CAN", "CAN", "CAN", 0,
                                            0, 0, 0, "I2C1", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "SSP0", "DAC", 0, 0, 0, 0, "I2C2", 0,
                                            "I2S", 0, "RIT", 0, "MCPWM", "QEI"};
            const char *n = (slot < 64 && names[slot]) ? names[slot] : "periférico";
            warn_once(std::string("unmw") + n, std::string(n) + " no está modelado: las escrituras se ignoran");
            return true;
        }
        }
    }
    if (addr >= 0x50000000u && addr < 0x50200000u) {
        warn_once("ahbw", "Periféricos AHB (Ethernet, USB, GPDMA) no modelados: las escrituras se ignoran");
        return true;
    }
    if (addr >= 0xE000E010u && addr < 0xE000E020u) {
        uint32_t off = (addr - 0xE000E010u) & ~3u;
        switch (off) {
        case 0x0: {
            st_val0 = systick_val(t);
            st_base = t;
            st_ctrl = v & 7;
            if ((st_ctrl & 1) && !(st_ctrl & 4))
                warn_once("stclk", "SysTick con CLKSOURCE=0 (reloj externo STCLK): no modelado, se usa CCLK");
            systick_resched(t);
            break;
        }
        case 0x4: st_val0 = systick_val(t); st_base = t; st_load = v & 0xFFFFFF; systick_resched(t); break;
        case 0x8: st_val0 = 0; st_base = t; st_countflag = false; systick_resched(t); break;
        default: break;
        }
        return true;
    }
    if (addr >= 0xE0000000u && addr < 0xE0100000u) return true;
    return false;
}

} // namespace lpcsim
