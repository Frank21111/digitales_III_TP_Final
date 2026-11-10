// Ejecuta un .axf en el modelo del SoC con estímulos simples y muestra lo que sale por los pines.
// Uso: run_axf <prog.axf> <segundos> [--adc V] [--echo us] [--uart texto] [--eint]
#include "../src/core/lpc1769.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>
using namespace lpcsim;
static const int64_t US = 1000000, MS = 1000 * US, SEC = 1000 * MS;

struct Host : SocHost {
    Lpc1769 *soc = nullptr;
    double adc = 0;
    int64_t echo = -1;
    int64_t trig_rise = -1;
    std::map<int, std::vector<std::pair<int64_t, bool>>> edges;
    void pin_output(int64_t t, int port, int bit, Drive d) override {
        bool l = d == Drive::STRONG_HIGH || d == Drive::WEAK_HIGH;
        auto &e = edges[port * 32 + bit];
        if (e.empty() || e.back().second != l) e.push_back({t, l});
        if (port == 0 && bit == 5 && echo > 0) {
            if (l) trig_rise = t;
            else if (trig_rise >= 0) {
                soc->input_edge(t + 250 * US, 0, 4, true);
                soc->input_edge(t + 250 * US + echo, 0, 4, false);
                trig_rise = -1;
            }
        }
    }
    double analog_input(int, int) override { return adc; }
    void soc_log(const std::string &m) override { printf("  [modelo] %s\n", m.c_str()); }
};

static bool lvl(const std::vector<std::pair<int64_t, bool>> &e, int64_t t, bool d) {
    for (auto &x : e) { if (x.first > t) break; d = x.second; }
    return d;
}

int main(int argc, char **argv) {
    if (argc < 3) { printf("uso: run_axf prog.axf segundos [--adc V] [--echo us] [--uart txt] [--eint]\n"); return 2; }
    Host h;
    Lpc1769 soc(&h);
    h.soc = &soc;
    std::string err, uart;
    bool eint = false;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--adc")) h.adc = atof(argv[++i]);
        else if (!strcmp(argv[i], "--echo")) h.echo = (int64_t)(atof(argv[++i]) * US);
        else if (!strcmp(argv[i], "--uart")) uart = argv[++i];
        else if (!strcmp(argv[i], "--eint")) eint = true;
    }
    if (!soc.load_program(argv[1], err)) { printf("%s\n", err.c_str()); return 2; }
    soc.set_pin_present(0, 4, true, false);
    soc.set_pin_present(0, 3, true, true);
    soc.set_pin_present(2, 10, true, true);
    soc.reset(0);
    int64_t tend = (int64_t)(atof(argv[2]) * SEC);
    if (!uart.empty()) {
        int64_t t = 500 * MS, bit = (int64_t)(1e12 / 9600);
        for (char c : uart) {
            if (c == '|') c = '\r';
            soc.input_edge(t, 0, 3, false);
            for (int k = 0; k < 8; k++) soc.input_edge(t + (k + 1) * bit, 0, 3, (c >> k) & 1);
            soc.input_edge(t + 9 * bit, 0, 3, true);
            t += 10 * bit;
        }
    }
    if (eint) for (int k = 1; k <= 3; k++) { soc.input_edge(k * 300 * MS, 2, 10, false); soc.input_edge(k * 300 * MS + 20 * MS, 2, 10, true); }
    soc.run_until(tend);
    auto &tx = h.edges[2];
    double bit = 1e12 / (25e6 / (16.0 * 163));
    std::string out;
    for (size_t i = 0; i < tx.size();) {
        if (tx[i].second) { i++; continue; }
        int64_t t0 = tx[i].first; int c = 0;
        for (int k = 0; k < 8; k++) if (lvl(tx, t0 + (int64_t)((k + 1.5) * bit), true)) c |= 1 << k;
        out += (char)c;
        while (i < tx.size() && tx[i].first < t0 + (int64_t)(9.6 * bit)) i++;
    }
    printf("UART0 TX:\n%s\n", out.c_str());
    for (auto &kv : h.edges) {
        if (kv.first == 2) continue;
        int n = 0; for (auto &x : kv.second) if (x.first > 0) n++;
        if (n < 2) continue;
        printf("P%d.%d: %d flancos", kv.first / 32, kv.first % 32, n);
        if (kv.first == 64) {
            for (int s = 0; s < (int)(tend / SEC); s++) {
                int64_t a = s * SEC + 200 * MS, b = (s + 1) * SEC - 100 * MS, hi = 0, t = a;
                bool v = lvl(kv.second, a, false);
                for (auto &x : kv.second) { if (x.first <= a) continue; if (x.first >= b) break; if (v) hi += x.first - t; t = x.first; v = x.second; }
                if (v) hi += b - t;
                printf(" | s%d duty=%.1f%%", s, 100.0 * (double)hi / (double)(b - a));
            }
        } else {
            std::vector<int64_t> r; for (auto &x : kv.second) if (x.second && x.first > 0) r.push_back(x.first);
            if (r.size() > 1) printf(", período medio %.3f ms", (double)(r.back() - r.front()) / (double)(r.size() - 1) / 1e9);
        }
        printf("\n");
    }
    return 0;
}
