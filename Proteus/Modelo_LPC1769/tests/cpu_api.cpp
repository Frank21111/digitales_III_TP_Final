// API C mínima del núcleo para las pruebas diferenciales (ctypes desde Python).
#include "../src/core/cpu.hpp"
#include <cstring>
#include <cstdio>

using namespace lpcsim;

namespace {
struct NullIo : CpuIo {
    bool io_read(uint32_t, int, uint32_t &v) override { v = 0; return false; }
    bool io_write(uint32_t, int, uint32_t) override { return false; }
    void cpu_log(const std::string &m) override { fprintf(stderr, "[cpu] %s\n", m.c_str()); }
};
struct Ctx {
    NullIo io;
    Cpu cpu{&io};
};
}

extern "C" {
void *t_new(uint8_t *code, uint32_t code_size, uint8_t *ram, uint32_t ram_size) {
    Ctx *c = new Ctx;
    c->cpu.add_region(0x00000000, code_size, code, false);
    c->cpu.add_region(0x20000000, ram_size, ram, true);
    return c;
}
void t_free(void *h) { delete (Ctx *)h; }
void t_set(void *h, const uint32_t *regs, uint32_t apsr) {
    Cpu &c = ((Ctx *)h)->cpu;
    for (int i = 0; i < 15; i++) c.r[i] = regs[i];
    c.msp = regs[13];
    c.pc = regs[15] & ~1u;
    c.flagN = (apsr >> 31) & 1; c.flagZ = (apsr >> 30) & 1; c.flagC = (apsr >> 29) & 1;
    c.flagV = (apsr >> 28) & 1; c.flagQ = (apsr >> 27) & 1;
    c.itstate = 0;
    c.lockup = false;
}
// Ejecuta hasta que PC == stop o se agotan max_steps. Devuelve los pasos.
int t_run_until(void *h, uint32_t stop, int max_steps) {
    Cpu &c = ((Ctx *)h)->cpu;
    int n = 0;
    while (c.pc != stop && n < max_steps && !c.lockup) { c.run(1); n++; }
    return n;
}
void t_get(void *h, uint32_t *regs, uint32_t *apsr) {
    Cpu &c = ((Ctx *)h)->cpu;
    for (int i = 0; i < 15; i++) regs[i] = c.r[i];
    regs[15] = c.pc;
    *apsr = c.xpsr() & 0xF8000000u;
}
int t_lockup(void *h) { return ((Ctx *)h)->cpu.lockup ? 1 : 0; }
}
