// Prueba del SoC con el firmware del TPI sin modificar.
// Emula el HC-SR04, el LM35 y la PC (UART) alrededor del modelo y verifica
// la telemetría, el PWM de la bomba y las salidas de la alarma.
#include "../src/core/lpc1769.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace lpcsim;

static const int64_t US = 1000000;
static const int64_t MS = 1000 * US;
static const int64_t SEC = 1000 * MS;

struct Host : SocHost {
    Lpc1769 *soc = nullptr;
    double lm35_v = 0.25;
    bool echo_on = true;
    int64_t echo_width = 1166 * US;
    int64_t trig_rise = -1;
    int trig_count = 0;
    std::vector<int64_t> trig_widths;
    std::vector<std::pair<int64_t, bool>> txd, pwm;
    std::map<int, bool> level;
    std::vector<std::string> logs;

    static bool hi(Drive d) { return d == Drive::STRONG_HIGH || d == Drive::WEAK_HIGH; }
    void pin_output(int64_t t, int port, int bit, Drive d) override {
        bool l = hi(d);
        level[port * 32 + bit] = l;
        if (port == 0 && bit == 2) txd.push_back({t, l});
        if (port == 2 && bit == 0) pwm.push_back({t, l});
        if (port == 0 && bit == 5) {
            if (d == Drive::STRONG_HIGH) trig_rise = t;
            else if (d != Drive::STRONG_LOW) trig_rise = -1;
            else if (trig_rise >= 0) {
                trig_widths.push_back(t - trig_rise);
                trig_count++;
                if (echo_on && t - trig_rise >= 10 * US) {
                    soc->input_edge(t + 200 * US, 0, 4, true);
                    soc->input_edge(t + 200 * US + echo_width, 0, 4, false);
                }
                trig_rise = -1;
            }
        }
    }
    double analog_input(int port, int bit) override { return (port == 0 && bit == 23) ? lm35_v : 0.0; }
    void soc_log(const std::string &m) override { logs.push_back(m); printf("  [modelo] %s\n", m.c_str()); }

    void send_line(int64_t t, const std::string &s) {
        int64_t bit = (int64_t)(1e12 / 9600.0);
        for (char ch : s) {
            uint8_t c = (uint8_t)ch;
            soc->input_edge(t, 0, 3, false);
            bool prev = false;
            for (int i = 0; i < 8; i++) {
                bool b = (c >> i) & 1;
                if (b != prev) soc->input_edge(t + (i + 1) * bit, 0, 3, b);
                prev = b;
            }
            if (!prev) soc->input_edge(t + 9 * bit, 0, 3, true);
            t += 10 * bit;
        }
    }

    bool lvl_at(const std::vector<std::pair<int64_t, bool>> &e, int64_t t, bool def) const {
        bool v = def;
        for (auto &x : e) { if (x.first > t) break; v = x.second; }
        return v;
    }
    std::string decode_uart(int64_t from, int64_t to) const {
        double bit = 1e12 / (25e6 / (16.0 * 163));
        std::string out;
        size_t i = 0;
        while (i < txd.size()) {
            if (txd[i].first < from || txd[i].second) { i++; continue; }
            int64_t t0 = txd[i].first;
            if (t0 > to) break;
            uint8_t c = 0;
            for (int k = 0; k < 8; k++)
                if (lvl_at(txd, t0 + (int64_t)((k + 1.5) * bit), true)) c |= 1 << k;
            out += (char)c;
            int64_t tend = t0 + (int64_t)(9.6 * bit);
            while (i < txd.size() && txd[i].first < tend) i++;
        }
        return out;
    }
    double duty(int64_t from, int64_t to) const {
        int64_t high = 0;
        bool v = lvl_at(pwm, from, false);
        int64_t t = from;
        for (auto &x : pwm) {
            if (x.first <= from) continue;
            if (x.first >= to) break;
            if (v) high += x.first - t;
            t = x.first; v = x.second;
        }
        if (v) high += to - t;
        return 100.0 * (double)high / (double)(to - from);
    }
    double period_us(int64_t from, int64_t to) const {
        std::vector<int64_t> rises;
        for (auto &x : pwm) if (x.first >= from && x.first < to && x.second) rises.push_back(x.first);
        if (rises.size() < 2) return 0;
        return (double)(rises.back() - rises.front()) / (double)(rises.size() - 1) / 1e6;
    }
};

static int fails = 0;
static void check(bool ok, const std::string &what) {
    printf("%s %s\n", ok ? "  OK   " : "  FALLA", what.c_str());
    if (!ok) fails++;
}

static std::string last_line(const std::string &s) {
    size_t e = s.rfind('\n');
    if (e == std::string::npos) return "";
    size_t b = s.rfind('\n', e - 1);
    return s.substr(b == std::string::npos ? 0 : b + 1, e - (b == std::string::npos ? 0 : b + 1));
}

int main(int argc, char **argv) {
    const char *prog = argc > 1 ? argv[1] : "tests/firmware/build/tpi.axf";
    Host host;
    Lpc1769 soc(&host);
    host.soc = &soc;
    std::string err;
    if (!soc.load_program(prog, err)) { printf("carga: %s\n", err.c_str()); return 2; }
    soc.set_pin_present(0, 4, true, false);
    soc.set_pin_present(0, 3, true, true);
    soc.reset(0);
    auto w0 = std::chrono::steady_clock::now();

    printf("Etapa 1: nivel 20 mm (eco de 1166 us), LM35 a 0,25 V, SP = 0\n");
    soc.run_until(2 * SEC);
    std::string tel = host.decode_uart(0, 2 * SEC);
    std::string l1 = last_line(tel);
    printf("  telemetría: %s\n", l1.c_str());
    check(soc.cclk() > 99e6 && soc.cclk() < 101e6, "SystemInit deja CCLK en 100 MHz");
    check(l1.rfind("L=20,SP=0,T=24.9,PWM=0,FAN=0,ALM=0,ERR=0,MODO=A", 0) == 0, "trama de telemetría esperada");
    int lines = 0; for (char c : tel) if (c == '\n') lines++;
    check(lines >= 3 && lines <= 4, "telemetría cada 500 ms (" + std::to_string(lines) + " tramas en 2 s)");
    double tw = 0; for (auto w : host.trig_widths) tw += (double)w; tw /= (double)host.trig_widths.size() * 1e6;
    check(tw >= 11.0 && tw < 13.0, "pulso de TRIG de 11-12 us (el bucle arranca en cualquier fase del tick de 1 us) (medido " + std::to_string(tw) + " us)");
    check(host.trig_count >= 19 && host.trig_count <= 20, "un disparo cada 100 ms (" + std::to_string(host.trig_count) + " en 2 s)");

    printf("Etapa 2: la PC envía SP=120\n");
    host.send_line(2 * SEC, "SP=120\n");
    soc.run_until(4 * SEC);
    l1 = last_line(host.decode_uart(2 * SEC, 4 * SEC));
    printf("  telemetría: %s\n", l1.c_str());
    check(l1.find("SP=120") != std::string::npos && l1.find("PWM=100") != std::string::npos, "SP aceptado y bomba al 100 % (error de 100 mm satura el PI)");
    double d = host.duty(3 * SEC, 4 * SEC);
    check(d > 99.5, "duty medido en P2.0 = " + std::to_string(d) + " %");

    printf("Etapa 3: nivel 110 mm (eco de 641 us): el PI regula\n");
    host.echo_width = 641 * US;
    soc.run_until(6 * SEC);
    l1 = last_line(host.decode_uart(5 * SEC, 6 * SEC));
    printf("  telemetría: %s\n", l1.c_str());
    d = host.duty(5500 * MS, 6 * SEC);
    double per = host.period_us(5500 * MS, 6 * SEC);
    check(l1.find("L=110") != std::string::npos, "nivel 110 mm");
    check(d > 10 && d < 40, "duty parcial medido = " + std::to_string(d) + " %");
    check(per > 999 && per < 1001, "período del PWM = " + std::to_string(per) + " us (1 kHz)");

    printf("Etapa 4: LM35 a 0,46 V (46 °C)\n");
    host.lm35_v = 0.46;
    soc.run_until(7500 * MS);
    l1 = last_line(host.decode_uart(7 * SEC, 7500 * MS));
    printf("  telemetría: %s\n", l1.c_str());
    check(l1.find("T=46.0,") != std::string::npos && l1.find("FAN=1,ALM=1") != std::string::npos, "alarma encendida");
    check(host.level[2 * 32 + 2] && host.level[0 * 32 + 22] && host.level[2 * 32 + 3], "P2.2, P0.22 y P2.3 en alto");

    printf("Etapa 5: LM35 a 0,39 V (39 °C)\n");
    host.lm35_v = 0.39;
    soc.run_until(9 * SEC);
    l1 = last_line(host.decode_uart(8500 * MS, 9 * SEC));
    printf("  telemetría: %s\n", l1.c_str());
    check(l1.find("FAN=0,ALM=0") != std::string::npos, "alarma apagada por histéresis");
    check(!host.level[2 * 32 + 2] && !host.level[0 * 32 + 22] && !host.level[2 * 32 + 3], "P2.2, P0.22 y P2.3 en bajo");

    printf("Etapa 6: nivel 180 mm (eco de 233 us): corte de seguridad\n");
    host.echo_width = 233 * US;
    soc.run_until(10 * SEC);
    l1 = last_line(host.decode_uart(9500 * MS, 10 * SEC));
    printf("  telemetría: %s\n", l1.c_str());
    check(l1.find("L=180") != std::string::npos && l1.find("PWM=0,") != std::string::npos, "bomba en 0 % con nivel >= 180 mm");

    printf("Etapa 7: sin eco\n");
    host.echo_width = 641 * US;
    soc.run_until(10500 * MS);
    host.echo_on = false;
    soc.run_until(12 * SEC);
    l1 = last_line(host.decode_uart(11500 * MS, 12 * SEC));
    printf("  telemetría: %s\n", l1.c_str());
    check(l1.find("PWM=0,") != std::string::npos && l1.find("ERR=1") != std::string::npos, "falla de sensor: ERR=1 y bomba en 0 %");
    check(host.duty(11500 * MS, 12 * SEC) < 0.01, "P2.0 en bajo durante la falla");

    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    printf("\n12 s simulados en %.2f s reales (%.2fx tiempo real), %.0f M ciclos de CPU\n", wall, 12.0 / wall, (double)soc.instructions() / 1e6);
    printf("%s: %d fallas\n", fails ? "RESULTADO" : "RESULTADO", fails);
    return fails ? 1 : 0;
}
