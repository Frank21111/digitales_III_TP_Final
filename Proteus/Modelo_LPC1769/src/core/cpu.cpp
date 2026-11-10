// Intérprete ARMv7-M. Referencia: ARMv7-M Architecture Reference Manual (DDI 0403E).
#include "cpu.hpp"
#include <cstdio>
#include <cstring>

namespace lpcsim {

static inline uint32_t sext(uint32_t v, int bits) {
    uint32_t m = 1u << (bits - 1);
    v &= (bits == 32) ? 0xFFFFFFFFu : ((1u << bits) - 1);
    return (v ^ m) - m;
}
static inline uint32_t ror32(uint32_t v, int n) {
    n &= 31;
    return n ? ((v >> n) | (v << (32 - n))) : v;
}

Cpu::Cpu(CpuIo *io_, int nirq) : io(io_), num_irqs(nirq), nregions(0) {
    reset();
}

void Cpu::add_region(uint32_t base, uint32_t size, uint8_t *mem, bool writable) {
    if (nregions < 8) regions[nregions++] = Region{base, size, mem, writable};
}

void Cpu::reset() {
    memset(r, 0, sizeof(r));
    flagN = flagZ = flagC = flagV = flagQ = false;
    itstate = 0;
    ipsr = 0;
    primask = faultmask = basepri = control = 0;
    msp = psp = 0;
    cycles = 0;
    stop_request = false;
    sleeping = false;
    lockup = false;
    event_register = false;
    vtor = 0;
    memset(prio, 0, sizeof(prio));
    pend_mask = act_mask = en_mask = 0;
    fetch_mem = nullptr; fetch_base = 0; fetch_size = 0;
    prigroup = 0;
    shcsr = 0;
    ccr = 0x200;
    scr = 0;
    cfsr = hfsr = mmfar = bfar = 0;
    nvic_dirty = true;
    cached_pending = 0;
    cached_exec = 256;
    handler_mode = false;
    excl_monitor = false;
    excl_addr = 0;
    branched = false;
    next_pc = 0;
    pc = 0;
    cur_instr_addr = 0;
    if (nregions > 0) {
        try {
            uint32_t sp0 = bus_read(0, 4);
            uint32_t pc0 = bus_read(4, 4);
            msp = sp0 & ~3u;
            r[13] = msp;
            r[14] = 0xFFFFFFFF;
            pc = pc0 & ~1u;
            if ((pc0 & 1) == 0) log("Vector de reset sin bit Thumb (bit 0 en 0): el programa no es Thumb");
        } catch (CpuFault &) {
            log("No se pudo leer la tabla de vectores en 0x00000000");
            lockup = true;
        }
    }
}

uint32_t Cpu::xpsr() const {
    uint32_t v = 0;
    if (flagN) v |= 1u << 31;
    if (flagZ) v |= 1u << 30;
    if (flagC) v |= 1u << 29;
    if (flagV) v |= 1u << 28;
    if (flagQ) v |= 1u << 27;
    v |= (itstate & 3u) << 25;
    v |= ((itstate >> 2) & 0x3Fu) << 10;
    v |= 1u << 24;
    v |= ipsr & 0x1FF;
    return v;
}

std::string Cpu::state_dump() const {
    char buf[512];
    snprintf(buf, sizeof(buf),
             "PC=%08X R0=%08X R1=%08X R2=%08X R3=%08X R4=%08X R5=%08X R6=%08X R7=%08X "
             "R8=%08X R9=%08X R10=%08X R11=%08X R12=%08X SP=%08X LR=%08X xPSR=%08X CFSR=%08X HFSR=%08X",
             pc, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14],
             xpsr(), cfsr, hfsr);
    return buf;
}

// ---------------------------------------------------------------- memoria

uint32_t Cpu::bus_read(uint32_t a, int size) {
    for (int i = 0; i < nregions; i++) {
        const Region &rg = regions[i];
        uint32_t off = a - rg.base;
        if (off < rg.size && rg.size - off >= (uint32_t)size) {
            const uint8_t *p = rg.mem + off;
            if (size == 4) { uint32_t v; memcpy(&v, p, 4); return v; }
            if (size == 2) { uint16_t v; memcpy(&v, p, 2); return v; }
            return *p;
        }
    }
    if (a >= 0xE000E000u && a < 0xE000F000u && !(a >= 0xE000E010u && a < 0xE000E020u)) {
        uint32_t v = 0;
        scs_read(a, size, v);
        return v;
    }
    if ((a & 0xFE000000u) == 0x22000000u || (a & 0xFE000000u) == 0x42000000u) {
        uint32_t base = (a & 0xF0000000u) | ((a & 0x01FFFFFFu) >> 5);
        uint32_t bit = (a >> 2) & 7;
        uint32_t word_addr = base & ~3u;
        uint32_t bitpos = ((base & 3u) * 8) + bit;
        uint32_t w = bus_read(word_addr, 4);
        return (w >> bitpos) & 1;
    }
    uint32_t v = 0;
    if (!io->io_read(a, size, v)) throw CpuFault{F_BUS_DATA, a};
    return v;
}

void Cpu::bus_write(uint32_t a, int size, uint32_t v) {
    for (int i = 0; i < nregions; i++) {
        const Region &rg = regions[i];
        uint32_t off = a - rg.base;
        if (off < rg.size && rg.size - off >= (uint32_t)size) {
            if (!rg.writable) throw CpuFault{F_BUS_DATA, a};
            uint8_t *p = rg.mem + off;
            if (size == 4) memcpy(p, &v, 4);
            else if (size == 2) { uint16_t h = (uint16_t)v; memcpy(p, &h, 2); }
            else *p = (uint8_t)v;
            return;
        }
    }
    if (a >= 0xE000E000u && a < 0xE000F000u && !(a >= 0xE000E010u && a < 0xE000E020u)) {
        scs_write(a, size, v);
        return;
    }
    if ((a & 0xFE000000u) == 0x22000000u || (a & 0xFE000000u) == 0x42000000u) {
        uint32_t base = (a & 0xF0000000u) | ((a & 0x01FFFFFFu) >> 5);
        uint32_t bit = (a >> 2) & 7;
        uint32_t word_addr = base & ~3u;
        uint32_t bitpos = ((base & 3u) * 8) + bit;
        uint32_t w = bus_read(word_addr, 4);
        if (v & 1) w |= 1u << bitpos; else w &= ~(1u << bitpos);
        bus_write(word_addr, 4, w);
        return;
    }
    if (!io->io_write(a, size, v)) throw CpuFault{F_BUS_DATA, a};
}

uint32_t Cpu::read_mem(uint32_t addr, int size, bool aligned_required) {
    if (size > 1 && (addr & (size - 1))) {
        if (aligned_required || (ccr & 0x8)) throw CpuFault{F_UNALIGNED, addr};
        uint32_t v = 0;
        for (int i = 0; i < size; i++) v |= bus_read(addr + i, 1) << (8 * i);
        return v;
    }
    return bus_read(addr, size);
}

void Cpu::write_mem(uint32_t addr, int size, uint32_t v, bool aligned_required) {
    if (size > 1 && (addr & (size - 1))) {
        if (aligned_required || (ccr & 0x8)) throw CpuFault{F_UNALIGNED, addr};
        for (int i = 0; i < size; i++) bus_write(addr + i, 1, (v >> (8 * i)) & 0xFF);
        return;
    }
    bus_write(addr, size, v);
}

uint32_t Cpu::mem_read(uint32_t addr, int size) { return bus_read(addr, size); }
void Cpu::mem_write(uint32_t addr, int size, uint32_t v) { bus_write(addr, size, v); }

uint16_t Cpu::fetch16(uint32_t addr) {
    uint32_t off = addr - fetch_base;
    uint16_t v;
    if (off + 1 < fetch_size) { memcpy(&v, fetch_mem + off, 2); return v; }
    for (int i = 0; i < nregions; i++) {
        const Region &rg = regions[i];
        uint32_t o = addr - rg.base;
        if (o + 1 < rg.size) {
            fetch_base = rg.base; fetch_size = rg.size; fetch_mem = rg.mem;
            memcpy(&v, rg.mem + o, 2);
            return v;
        }
    }
    throw CpuFault{F_BUS_FETCH, addr};
}

// ---------------------------------------------------------------- utilidades ALU

uint32_t Cpu::add_with_carry(uint32_t x, uint32_t y, bool cin, bool &cout, bool &ovf) const {
    uint64_t us = (uint64_t)x + (uint64_t)y + (cin ? 1 : 0);
    int64_t ss = (int64_t)(int32_t)x + (int64_t)(int32_t)y + (cin ? 1 : 0);
    uint32_t res = (uint32_t)us;
    cout = (us >> 32) != 0;
    ovf = ss != (int64_t)(int32_t)res;
    return res;
}

// type: 0 LSL, 1 LSR, 2 ASR, 3 ROR, 4 RRX
uint32_t Cpu::shift_c(uint32_t value, int type, int amount, bool cin, bool &cout) const {
    if (type == 4) {
        cout = value & 1;
        return (cin ? 0x80000000u : 0) | (value >> 1);
    }
    if (amount == 0) { cout = cin; return value; }
    switch (type) {
    case 0:
        if (amount < 32) { cout = (value >> (32 - amount)) & 1; return value << amount; }
        cout = (amount == 32) ? (value & 1) : 0;
        return 0;
    case 1:
        if (amount < 32) { cout = (value >> (amount - 1)) & 1; return value >> amount; }
        cout = (amount == 32) ? (value >> 31) : 0;
        return 0;
    case 2:
        if (amount < 32) { cout = (value >> (amount - 1)) & 1; return (uint32_t)((int32_t)value >> amount); }
        cout = value >> 31;
        return (value & 0x80000000u) ? 0xFFFFFFFFu : 0;
    default: {
        uint32_t res = ror32(value, amount & 31);
        cout = res >> 31;
        return res;
    }
    }
}

uint32_t Cpu::shift(uint32_t value, int type, int amount, bool cin) const {
    bool c;
    return shift_c(value, type, amount, cin, c);
}

uint32_t Cpu::thumb_expand_imm_c(uint32_t imm12, bool cin, bool &cout) const {
    uint32_t imm8 = imm12 & 0xFF;
    if ((imm12 >> 10) == 0) {
        cout = cin;
        switch ((imm12 >> 8) & 3) {
        case 0: return imm8;
        case 1: return (imm8 << 16) | imm8;
        case 2: return (imm8 << 24) | (imm8 << 8);
        default: return imm8 * 0x01010101u;
        }
    }
    uint32_t unrot = 0x80 | (imm12 & 0x7F);
    uint32_t res = ror32(unrot, (imm12 >> 7) & 0x1F);
    cout = res >> 31;
    return res;
}

bool Cpu::cond_pass(uint32_t cond) const {
    switch (cond & 0xF) {
    case 0: return flagZ;
    case 1: return !flagZ;
    case 2: return flagC;
    case 3: return !flagC;
    case 4: return flagN;
    case 5: return !flagN;
    case 6: return flagV;
    case 7: return !flagV;
    case 8: return flagC && !flagZ;
    case 9: return !flagC || flagZ;
    case 10: return flagN == flagV;
    case 11: return flagN != flagV;
    case 12: return !flagZ && (flagN == flagV);
    case 13: return flagZ || (flagN != flagV);
    default: return true;
    }
}

void Cpu::it_advance() {
    if ((itstate & 7) == 0) itstate = 0;
    else itstate = (itstate & 0xE0) | ((itstate << 1) & 0x1F);
}

void Cpu::branch_to(uint32_t addr) {
    next_pc = addr & ~1u;
    branched = true;
}

void Cpu::set_reg(int n, uint32_t v) {
    if (n == 15) branch_to(v);
    else if (n == 13) r[13] = v & ~3u;
    else r[n] = v;
}

void Cpu::bx_write_pc(uint32_t addr) {
    if (handler_mode && (addr & 0xF0000000u) == 0xF0000000u) {
        exception_return(addr);
        return;
    }
    if ((addr & 1) == 0) {
        throw CpuFault{F_INVSTATE, addr};
    }
    branch_to(addr);
}

void Cpu::undefined() { throw CpuFault{F_UNDEF, cur_instr_addr}; }

// ---------------------------------------------------------------- bucle principal

uint64_t Cpu::run(uint64_t budget) {
    uint64_t start = cycles;
    stop_request = false;
    while (!lockup && !stop_request) {
        if (nvic_dirty) {
            cached_pending = highest_pending();
            cached_exec = exec_priority();
            nvic_dirty = false;
        }
        if (cached_pending) {
            int e = cached_pending;
            if (group_prio(exc_prio(e)) < cached_exec) {
                sleeping = false;
                take_exception(e, pc);
                cycles += 12;
                if (cycles - start >= budget) break;
                continue;
            }
        }
        if (sleeping) break;
        cycles += (uint64_t)step();
        if (cycles - start >= budget) break;
    }
    return cycles - start;
}

int Cpu::step() {
    cur_instr_addr = pc;
    int cyc = 1;
    try {
        uint32_t hw1 = fetch16(pc);
        bool is32 = (hw1 & 0xE000) == 0xE000 && (hw1 & 0x1800) != 0;
        uint32_t hw2 = is32 ? fetch16(pc + 2) : 0;
        r[15] = pc + 4;
        next_pc = pc + (is32 ? 4 : 2);
        branched = false;
        exc_returned = false;
        bool was_it = in_it();
        bool exec = !was_it || cond_pass(itstate >> 4);
        if (exec) {
            if (is32) exec32(hw1, hw2);
            else exec16(hw1);
        }
        if (was_it && !exc_returned) it_advance();
        if (branched) cyc += 2;
        pc = next_pc;
    } catch (CpuFault &f) {
        pc = cur_instr_addr;
        raise_fault(f.kind, f.addr);
    }
    return cyc;
}

// ---------------------------------------------------------------- 16 bits

void Cpu::exec16(uint32_t hw) {
    uint32_t op5 = hw >> 11;
    bool sf = !in_it();
    bool c, v;
    switch (op5) {
    case 0: case 1: case 2: {
        int imm5 = (hw >> 6) & 0x1F, rm = (hw >> 3) & 7, rd = hw & 7;
        int amt = imm5;
        if (op5 != 0 && imm5 == 0) amt = 32;
        uint32_t res = shift_c(r[rm], op5, amt, flagC, c);
        r[rd] = res;
        if (sf) { set_nz(res); flagC = c; }
        return;
    }
    case 3: {
        int rd = hw & 7, rn = (hw >> 3) & 7;
        uint32_t opnd = (hw & 0x400) ? ((hw >> 6) & 7) : r[(hw >> 6) & 7];
        uint32_t res = (hw & 0x200) ? add_with_carry(r[rn], ~opnd, true, c, v)
                                    : add_with_carry(r[rn], opnd, false, c, v);
        r[rd] = res;
        if (sf) { set_nz(res); flagC = c; flagV = v; }
        return;
    }
    case 4: {
        int rd = (hw >> 8) & 7;
        uint32_t res = hw & 0xFF;
        r[rd] = res;
        if (sf) set_nz(res);
        return;
    }
    case 5: {
        int rn = (hw >> 8) & 7;
        uint32_t res = add_with_carry(r[rn], ~(hw & 0xFF), true, c, v);
        set_nz(res); flagC = c; flagV = v;
        return;
    }
    case 6: case 7: {
        int rdn = (hw >> 8) & 7;
        uint32_t imm = hw & 0xFF;
        uint32_t res = (op5 == 7) ? add_with_carry(r[rdn], ~imm, true, c, v) : add_with_carry(r[rdn], imm, false, c, v);
        r[rdn] = res;
        if (sf) { set_nz(res); flagC = c; flagV = v; }
        return;
    }
    case 8: {
        if ((hw & 0x400) == 0) {
            int op = (hw >> 6) & 0xF, rm = (hw >> 3) & 7, rdn = hw & 7;
            uint32_t a = r[rdn], b = r[rm], res = 0;
            switch (op) {
            case 0: res = a & b; r[rdn] = res; if (sf) set_nz(res); break;
            case 1: res = a ^ b; r[rdn] = res; if (sf) set_nz(res); break;
            case 2: case 3: case 4: case 7: {
                int type = (op == 2) ? 0 : (op == 3) ? 1 : (op == 4) ? 2 : 3;
                res = shift_c(a, type, b & 0xFF, flagC, c);
                r[rdn] = res;
                if (sf) { set_nz(res); flagC = c; }
                break;
            }
            case 5: res = add_with_carry(a, b, flagC, c, v); r[rdn] = res; if (sf) { set_nz(res); flagC = c; flagV = v; } break;
            case 6: res = add_with_carry(a, ~b, flagC, c, v); r[rdn] = res; if (sf) { set_nz(res); flagC = c; flagV = v; } break;
            case 8: res = a & b; set_nz(res); break;
            case 9: res = add_with_carry(~b, 0, true, c, v); r[rdn] = res; if (sf) { set_nz(res); flagC = c; flagV = v; } break;
            case 10: res = add_with_carry(a, ~b, true, c, v); set_nz(res); flagC = c; flagV = v; break;
            case 11: res = add_with_carry(a, b, false, c, v); set_nz(res); flagC = c; flagV = v; break;
            case 12: res = a | b; r[rdn] = res; if (sf) set_nz(res); break;
            case 13: res = a * b; r[rdn] = res; if (sf) set_nz(res); break;
            case 14: res = a & ~b; r[rdn] = res; if (sf) set_nz(res); break;
            case 15: res = ~b; r[rdn] = res; if (sf) set_nz(res); break;
            }
            return;
        }
        int op = (hw >> 8) & 3;
        int rm = (hw >> 3) & 0xF;
        int rdn = ((hw >> 4) & 8) | (hw & 7);
        if (op == 0) {
            uint32_t res = r[rdn] + r[rm];
            set_reg(rdn, res);
        } else if (op == 1) {
            uint32_t res = add_with_carry(r[rdn], ~r[rm], true, c, v);
            set_nz(res); flagC = c; flagV = v;
        } else if (op == 2) {
            set_reg(rdn, r[rm]);
        } else {
            uint32_t target = r[rm];
            if (hw & 0x80) r[14] = next_pc | 1;
            bx_write_pc(target);
        }
        return;
    }
    case 9: {
        int rt = (hw >> 8) & 7;
        uint32_t addr = (r[15] & ~3u) + ((hw & 0xFF) << 2);
        r[rt] = read_mem(addr, 4);
        return;
    }
    case 10: case 11: {
        int opb = (hw >> 9) & 7, rm = (hw >> 6) & 7, rn = (hw >> 3) & 7, rt = hw & 7;
        uint32_t addr = r[rn] + r[rm];
        switch (opb) {
        case 0: write_mem(addr, 4, r[rt]); break;
        case 1: write_mem(addr, 2, r[rt]); break;
        case 2: write_mem(addr, 1, r[rt]); break;
        case 3: r[rt] = sext(read_mem(addr, 1), 8); break;
        case 4: r[rt] = read_mem(addr, 4); break;
        case 5: r[rt] = read_mem(addr, 2); break;
        case 6: r[rt] = read_mem(addr, 1); break;
        case 7: r[rt] = sext(read_mem(addr, 2), 16); break;
        }
        return;
    }
    case 12: case 13: case 14: case 15: case 16: case 17: {
        int imm5 = (hw >> 6) & 0x1F, rn = (hw >> 3) & 7, rt = hw & 7;
        int size = (op5 <= 13) ? 4 : (op5 <= 15) ? 1 : 2;
        bool load = (op5 & 1) != 0;
        uint32_t addr = r[rn] + (uint32_t)(imm5 * size);
        if (load) r[rt] = read_mem(addr, size);
        else write_mem(addr, size, r[rt]);
        return;
    }
    case 18: case 19: {
        int rt = (hw >> 8) & 7;
        uint32_t addr = r[13] + ((hw & 0xFF) << 2);
        if (op5 == 19) r[rt] = read_mem(addr, 4);
        else write_mem(addr, 4, r[rt]);
        return;
    }
    case 20: {
        int rd = (hw >> 8) & 7;
        r[rd] = (r[15] & ~3u) + ((hw & 0xFF) << 2);
        return;
    }
    case 21: {
        int rd = (hw >> 8) & 7;
        r[rd] = r[13] + ((hw & 0xFF) << 2);
        return;
    }
    case 22: case 23: {
        if ((hw & 0xFF00) == 0xB000) {
            uint32_t imm = (hw & 0x7F) << 2;
            if (hw & 0x80) r[13] -= imm; else r[13] += imm;
            return;
        }
        if ((hw & 0xF500) == 0xB100) {
            int rn = hw & 7;
            uint32_t imm = (((hw >> 9) & 1) << 6) | (((hw >> 3) & 0x1F) << 1);
            bool nonzero = (hw & 0x800) != 0;
            if ((r[rn] != 0) == nonzero) branch_to(r[15] + imm);
            return;
        }
        if ((hw & 0xFF00) == 0xB200) {
            int rm = (hw >> 3) & 7, rd = hw & 7;
            switch ((hw >> 6) & 3) {
            case 0: r[rd] = sext(r[rm], 16); break;
            case 1: r[rd] = sext(r[rm], 8); break;
            case 2: r[rd] = r[rm] & 0xFFFF; break;
            case 3: r[rd] = r[rm] & 0xFF; break;
            }
            return;
        }
        if ((hw & 0xFE00) == 0xB400) {
            uint32_t list = (hw & 0xFF) | ((hw & 0x100) ? 0x4000 : 0);
            int n = __builtin_popcount(list);
            uint32_t addr = r[13] - 4 * n;
            uint32_t a = addr;
            for (int i = 0; i < 15; i++)
                if (list & (1u << i)) { write_mem(a, 4, r[i], true); a += 4; }
            r[13] = addr;
            return;
        }
        if ((hw & 0xFFE8) == 0xB660) {
            bool disable = (hw & 0x10) != 0;
            if (hw & 2) primask = disable ? 1 : 0;
            if (hw & 1) { if (!disable) faultmask = 0; else if (exec_priority() > -1) faultmask = 1; }
            nvic_dirty = true;
            return;
        }
        if ((hw & 0xFF00) == 0xBA00) {
            int rm = (hw >> 3) & 7, rd = hw & 7;
            uint32_t x = r[rm];
            switch ((hw >> 6) & 3) {
            case 0: r[rd] = __builtin_bswap32(x); return;
            case 1: r[rd] = ((x & 0x00FF00FFu) << 8) | ((x & 0xFF00FF00u) >> 8); return;
            case 3: r[rd] = sext(((x & 0xFF) << 8) | ((x >> 8) & 0xFF), 16); return;
            default: undefined();
            }
            return;
        }
        if ((hw & 0xFE00) == 0xBC00) {
            uint32_t list = (hw & 0xFF) | ((hw & 0x100) ? 0x8000 : 0);
            uint32_t a = r[13];
            uint32_t newpc = 0;
            int n = __builtin_popcount(list);
            for (int i = 0; i < 16; i++)
                if (list & (1u << i)) {
                    uint32_t val = read_mem(a, 4, true);
                    a += 4;
                    if (i == 15) newpc = val; else r[i] = val;
                }
            r[13] = r[13] + 4 * n;
            if (list & 0x8000) bx_write_pc(newpc);
            return;
        }
        if ((hw & 0xFF00) == 0xBE00) {
            char buf[96];
            snprintf(buf, sizeof(buf), "BKPT #%u en 0x%08X: sin depurador, se trata como HardFault", hw & 0xFF, cur_instr_addr);
            log(buf);
            throw CpuFault{F_BKPT, cur_instr_addr};
        }
        if ((hw & 0xFF00) == 0xBF00) {
            if (hw & 0xF) { itstate = hw & 0xFF; return; }
            switch ((hw >> 4) & 0xF) {
            case 0: case 1: return;
            case 2:
                if (event_register) event_register = false;
                else { sleeping = true; }
                return;
            case 3: sleeping = true; return;
            case 4: event_register = true; return;
            default: return;
            }
        }
        undefined();
        return;
    }
    case 24: case 25: {
        int rn = (hw >> 8) & 7;
        uint32_t list = hw & 0xFF;
        uint32_t a = r[rn];
        int n = __builtin_popcount(list);
        if (op5 == 24) {
            for (int i = 0; i < 8; i++)
                if (list & (1u << i)) { write_mem(a, 4, r[i], true); a += 4; }
            r[rn] = r[rn] + 4 * n;
        } else {
            uint32_t base = r[rn];
            for (int i = 0; i < 8; i++)
                if (list & (1u << i)) { r[i] = read_mem(a, 4, true); a += 4; }
            if (!(list & (1u << rn))) r[rn] = base + 4 * n;
        }
        return;
    }
    case 26: case 27: {
        uint32_t cond = (hw >> 8) & 0xF;
        if (cond == 0xE) undefined();
        if (cond == 0xF) {
            int e = EXC_SVCALL;
            int ep = exec_priority();
            if (ep <= -1) {
                lockup = true;
                log("LOCKUP: SVC dentro de HardFault/NMI o con FAULTMASK activo. " + state_dump());
                return;
            }
            if (group_prio(exc_prio(e)) >= ep) {
                hfsr |= 1u << 30;
                pend_mask |= 1ull << (EXC_HARDFAULT);
            } else {
                pend_mask |= 1ull << (e);
            }
            nvic_dirty = true;
            return;
        }
        if (cond_pass(cond)) branch_to(r[15] + sext((hw & 0xFF) << 1, 9));
        return;
    }
    case 28:
        branch_to(r[15] + sext((hw & 0x7FF) << 1, 12));
        return;
    default:
        undefined();
    }
}

// ---------------------------------------------------------------- 32 bits

void Cpu::exec32(uint32_t hw1, uint32_t hw2) {
    uint32_t op1 = (hw1 >> 11) & 3;
    uint32_t op2 = (hw1 >> 4) & 0x7F;
    if (op1 == 1) {
        if ((op2 & 0x64) == 0x00) exec32_ldst_multiple(hw1, hw2);
        else if ((op2 & 0x64) == 0x04) exec32_ldst_dual_excl(hw1, hw2);
        else if ((op2 & 0x60) == 0x20) exec32_dp_shifted(hw1, hw2);
        else throw CpuFault{F_NOCP, cur_instr_addr};
    } else if (op1 == 2) {
        if (hw2 & 0x8000) exec32_branch_misc(hw1, hw2);
        else if ((op2 & 0x20) == 0) exec32_dp_modimm(hw1, hw2);
        else exec32_dp_plainimm(hw1, hw2);
    } else {
        if ((op2 & 0x71) == 0x00 || ((op2 & 0x61) == 0x01 && (op2 & 0x06) != 0x06)) exec32_ldst_single(hw1, hw2);
        else if ((op2 & 0x70) == 0x20) exec32_dp_register(hw1, hw2);
        else if ((op2 & 0x78) == 0x30) exec32_mul(hw1, hw2);
        else if ((op2 & 0x78) == 0x38) exec32_longmul(hw1, hw2);
        else if (op2 & 0x40) throw CpuFault{F_NOCP, cur_instr_addr};
        else undefined();
    }
}

void Cpu::exec32_dp_shifted(uint32_t hw1, uint32_t hw2) {
    int op = (hw1 >> 5) & 0xF;
    bool s = (hw1 >> 4) & 1;
    int rn = hw1 & 0xF, rd = (hw2 >> 8) & 0xF, rm = hw2 & 0xF;
    int imm5 = (((hw2 >> 12) & 7) << 2) | ((hw2 >> 6) & 3);
    int type = (hw2 >> 4) & 3;
    int amt = imm5;
    int stype = type;
    if (type == 1 || type == 2) { if (imm5 == 0) amt = 32; }
    else if (type == 3 && imm5 == 0) { stype = 4; amt = 1; }
    bool sc;
    uint32_t shifted = shift_c(r[rm], stype, amt, flagC, sc);
    uint32_t a = r[rn], res;
    bool c, v;
    switch (op) {
    case 0:
        res = a & shifted;
        if (rd == 15 && s) { set_nz(res); flagC = sc; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = sc; }
        return;
    case 1:
        res = a & ~shifted; set_reg(rd, res); if (s) { set_nz(res); flagC = sc; }
        return;
    case 2:
        res = (rn == 15) ? shifted : (a | shifted);
        set_reg(rd, res); if (s) { set_nz(res); flagC = sc; }
        return;
    case 3:
        res = (rn == 15) ? ~shifted : (a | ~shifted);
        set_reg(rd, res); if (s) { set_nz(res); flagC = sc; }
        return;
    case 4:
        res = a ^ shifted;
        if (rd == 15 && s) { set_nz(res); flagC = sc; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = sc; }
        return;
    case 8:
        res = add_with_carry(a, shifted, false, c, v);
        if (rd == 15 && s) { set_nz(res); flagC = c; flagV = v; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 10:
        res = add_with_carry(a, shifted, flagC, c, v);
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 11:
        res = add_with_carry(a, ~shifted, flagC, c, v);
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 13:
        res = add_with_carry(a, ~shifted, true, c, v);
        if (rd == 15 && s) { set_nz(res); flagC = c; flagV = v; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 14:
        res = add_with_carry(~a, shifted, true, c, v);
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    default:
        undefined();
    }
}

void Cpu::exec32_dp_modimm(uint32_t hw1, uint32_t hw2) {
    int op = (hw1 >> 5) & 0xF;
    bool s = (hw1 >> 4) & 1;
    int rn = hw1 & 0xF, rd = (hw2 >> 8) & 0xF;
    uint32_t imm12 = (((hw1 >> 10) & 1) << 11) | (((hw2 >> 12) & 7) << 8) | (hw2 & 0xFF);
    bool ic;
    uint32_t imm = thumb_expand_imm_c(imm12, flagC, ic);
    uint32_t a = r[rn], res;
    bool c, v;
    switch (op) {
    case 0:
        res = a & imm;
        if (rd == 15 && s) { set_nz(res); flagC = ic; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = ic; }
        return;
    case 1:
        res = a & ~imm; set_reg(rd, res); if (s) { set_nz(res); flagC = ic; }
        return;
    case 2:
        res = (rn == 15) ? imm : (a | imm);
        set_reg(rd, res); if (s) { set_nz(res); flagC = ic; }
        return;
    case 3:
        res = (rn == 15) ? ~imm : (a | ~imm);
        set_reg(rd, res); if (s) { set_nz(res); flagC = ic; }
        return;
    case 4:
        res = a ^ imm;
        if (rd == 15 && s) { set_nz(res); flagC = ic; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = ic; }
        return;
    case 8:
        res = add_with_carry(a, imm, false, c, v);
        if (rd == 15 && s) { set_nz(res); flagC = c; flagV = v; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 10:
        res = add_with_carry(a, imm, flagC, c, v);
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 11:
        res = add_with_carry(a, ~imm, flagC, c, v);
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 13:
        res = add_with_carry(a, ~imm, true, c, v);
        if (rd == 15 && s) { set_nz(res); flagC = c; flagV = v; return; }
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    case 14:
        res = add_with_carry(~a, imm, true, c, v);
        set_reg(rd, res); if (s) { set_nz(res); flagC = c; flagV = v; }
        return;
    default:
        undefined();
    }
}

void Cpu::exec32_dp_plainimm(uint32_t hw1, uint32_t hw2) {
    int op = (hw1 >> 4) & 0x1F;
    int rn = hw1 & 0xF, rd = (hw2 >> 8) & 0xF;
    uint32_t i = (hw1 >> 10) & 1, imm3 = (hw2 >> 12) & 7, imm8 = hw2 & 0xFF;
    uint32_t imm12 = (i << 11) | (imm3 << 8) | imm8;
    uint32_t imm2 = (hw2 >> 6) & 3;
    uint32_t lsbimm = (imm3 << 2) | imm2;
    switch (op) {
    case 0x00:
        set_reg(rd, (rn == 15) ? ((r[15] & ~3u) + imm12) : (r[rn] + imm12));
        return;
    case 0x0A:
        set_reg(rd, (rn == 15) ? ((r[15] & ~3u) - imm12) : (r[rn] - imm12));
        return;
    case 0x04:
        set_reg(rd, ((hw1 & 0xF) << 12) | imm12);
        return;
    case 0x0C:
        set_reg(rd, (((hw1 & 0xF) << 12 | imm12) << 16) | (r[rd] & 0xFFFF));
        return;
    case 0x10: case 0x12: case 0x18: case 0x1A: {
        bool sh = (op & 2) != 0;
        if (sh && lsbimm == 0) undefined();
        int64_t operand = sh ? (int64_t)(int32_t)r[rn] >> lsbimm : (int64_t)(int32_t)(r[rn] << lsbimm);
        bool sat = false;
        int64_t res;
        if (op < 0x18) {
            int n = (hw2 & 0x1F) + 1;
            int64_t mx = (1LL << (n - 1)) - 1, mn = -(1LL << (n - 1));
            if (operand > mx) { res = mx; sat = true; }
            else if (operand < mn) { res = mn; sat = true; }
            else res = operand;
        } else {
            int n = hw2 & 0x1F;
            int64_t mx = (1LL << n) - 1;
            if (operand > mx) { res = mx; sat = true; }
            else if (operand < 0) { res = 0; sat = true; }
            else res = operand;
        }
        set_reg(rd, (uint32_t)res);
        if (sat) flagQ = true;
        return;
    }
    case 0x14: case 0x1C: {
        int lsb = (int)lsbimm, w = (int)(hw2 & 0x1F) + 1;
        if (lsb + w > 32) undefined();
        uint32_t field = (w == 32) ? r[rn] : ((r[rn] >> lsb) & ((1u << w) - 1));
        set_reg(rd, (op == 0x14) ? sext(field, w) : field);
        return;
    }
    case 0x16: {
        int lsb = (int)lsbimm, msb = (int)(hw2 & 0x1F);
        if (msb < lsb) undefined();
        int w = msb - lsb + 1;
        uint32_t mask = ((w == 32) ? 0xFFFFFFFFu : ((1u << w) - 1)) << lsb;
        uint32_t src = (rn == 15) ? 0 : (r[rn] << lsb);
        set_reg(rd, (r[rd] & ~mask) | (src & mask));
        return;
    }
    default:
        undefined();
    }
}

void Cpu::exec32_branch_misc(uint32_t hw1, uint32_t hw2) {
    uint32_t op = (hw1 >> 4) & 0x7F;
    uint32_t op1 = (hw2 >> 12) & 7;
    uint32_t S = (hw1 >> 10) & 1, J1 = (hw2 >> 13) & 1, J2 = (hw2 >> 11) & 1;
    if ((op1 & 5) == 0) {
        if ((op & 0x38) != 0x38) {
            uint32_t cond = (hw1 >> 6) & 0xF;
            uint32_t imm = (S << 20) | (J2 << 19) | (J1 << 18) | ((hw1 & 0x3F) << 12) | ((hw2 & 0x7FF) << 1);
            if (cond_pass(cond)) branch_to(r[15] + sext(imm, 21));
            return;
        }
        if ((op & 0x7E) == 0x38) {
            uint32_t sysm = hw2 & 0xFF, val = r[hw1 & 0xF];
            uint32_t mask = (hw2 >> 10) & 3;
            switch (sysm >> 3) {
            case 0:
                if ((sysm & 4) == 0 && (mask & 2)) {
                    flagN = (val >> 31) & 1; flagZ = (val >> 30) & 1; flagC = (val >> 29) & 1;
                    flagV = (val >> 28) & 1; flagQ = (val >> 27) & 1;
                }
                return;
            case 1:
                if (sysm == 8) { if (using_psp()) msp = val & ~3u; else r[13] = val & ~3u; }
                else if (sysm == 9) { if (using_psp()) r[13] = val & ~3u; else psp = val & ~3u; }
                return;
            case 2:
                if (sysm == 16) primask = val & 1;
                else if (sysm == 17) basepri = val & 0xF8;
                else if (sysm == 18) { uint32_t nv = val & 0xF8; if (nv != 0 && (nv < basepri || basepri == 0)) basepri = nv; }
                else if (sysm == 19) { if (exec_priority() > -1) faultmask = val & 1; }
                else if (sysm == 20) {
                    uint32_t nc = val & 3;
                    if (handler_mode) nc = (nc & 1) | (control & 2);
                    bool before = using_psp();
                    control = nc;
                    bool after = using_psp();
                    if (before != after) {
                        if (before) { psp = r[13]; r[13] = msp; } else { msp = r[13]; r[13] = psp; }
                    }
                }
                nvic_dirty = true;
                return;
            default:
                return;
            }
        }
        if (op == 0x3A) {
            switch (hw2 & 0xFF) {
            case 2:
                if (event_register) event_register = false;
                else { sleeping = true; }
                return;
            case 3: sleeping = true; return;
            case 4: event_register = true; return;
            default: return;
            }
        }
        if (op == 0x3B) {
            if (((hw2 >> 4) & 0xF) == 2) excl_monitor = false;
            return;
        }
        if ((op & 0x7E) == 0x3E) {
            int rd = (hw2 >> 8) & 0xF;
            uint32_t sysm = hw2 & 0xFF, val = 0;
            switch (sysm >> 3) {
            case 0:
                if (sysm & 1) val |= ipsr & 0x1FF;
                if ((sysm & 4) == 0) val |= xpsr() & 0xF8000000u;
                break;
            case 1:
                if (sysm == 8) val = using_psp() ? msp : r[13];
                else if (sysm == 9) val = using_psp() ? r[13] : psp;
                break;
            case 2:
                if (sysm == 16) val = primask;
                else if (sysm == 17 || sysm == 18) val = basepri;
                else if (sysm == 19) val = faultmask;
                else if (sysm == 20) val = control;
                break;
            }
            set_reg(rd, val);
            return;
        }
        undefined();
    }
    uint32_t imm = (S << 24) | ((~(J1 ^ S) & 1) << 23) | ((~(J2 ^ S) & 1) << 22) | ((hw1 & 0x3FF) << 12) | ((hw2 & 0x7FF) << 1);
    uint32_t off = sext(imm, 25);
    if ((op1 & 5) == 1) { branch_to(r[15] + off); return; }
    if ((op1 & 5) == 5) { r[14] = next_pc | 1; branch_to(r[15] + off); return; }
    undefined();
}

void Cpu::exec32_ldst_multiple(uint32_t hw1, uint32_t hw2) {
    int op = (hw1 >> 7) & 3;
    bool w = (hw1 >> 5) & 1, load = (hw1 >> 4) & 1;
    int rn = hw1 & 0xF;
    uint32_t list = hw2 & 0xFFFF;
    if (op == 0 || op == 3) undefined();
    int n = __builtin_popcount(list);
    uint32_t base = r[rn];
    uint32_t a = (op == 1) ? base : base - 4 * n;
    uint32_t wb = (op == 1) ? base + 4 * n : base - 4 * n;
    if (load) {
        uint32_t newpc = 0;
        for (int i = 0; i < 16; i++)
            if (list & (1u << i)) {
                uint32_t val = read_mem(a, 4, true);
                a += 4;
                if (i == 15) newpc = val; else r[i] = val;
            }
        if (w && !(list & (1u << rn))) set_reg(rn, wb);
        if (list & 0x8000) bx_write_pc(newpc);
    } else {
        for (int i = 0; i < 15; i++)
            if (list & (1u << i)) { write_mem(a, 4, r[i], true); a += 4; }
        if (w) set_reg(rn, wb);
    }
}

void Cpu::exec32_ldst_dual_excl(uint32_t hw1, uint32_t hw2) {
    int op1 = (hw1 >> 7) & 3, op2 = (hw1 >> 4) & 3, op3 = (hw2 >> 4) & 0xF;
    int rn = hw1 & 0xF, rt = (hw2 >> 12) & 0xF, rd8 = (hw2 >> 8) & 0xF;
    if (op1 == 0 && op2 == 0) {
        uint32_t addr = r[rn] + ((hw2 & 0xFF) << 2);
        if (excl_monitor && excl_addr == addr) { write_mem(addr, 4, r[rt], true); r[rd8] = 0; }
        else r[rd8] = 1;
        excl_monitor = false;
        return;
    }
    if (op1 == 0 && op2 == 1) {
        uint32_t addr = r[rn] + ((hw2 & 0xFF) << 2);
        r[rt] = read_mem(addr, 4, true);
        excl_monitor = true; excl_addr = addr;
        return;
    }
    if (((op1 & 2) == 0 && op2 == 2) || ((op1 & 2) && (op2 & 1) == 0) ||
        ((op1 & 2) == 0 && op2 == 3) || ((op1 & 2) && (op2 & 1))) {
        bool load = (op2 & 1) != 0;
        bool p = (hw1 >> 8) & 1, u = (hw1 >> 7) & 1, w = (hw1 >> 5) & 1;
        uint32_t imm = (hw2 & 0xFF) << 2;
        uint32_t base = (rn == 15) ? (r[15] & ~3u) : r[rn];
        uint32_t off = u ? base + imm : base - imm;
        uint32_t addr = p ? off : base;
        if (load) {
            uint32_t v1 = read_mem(addr, 4, true), v2 = read_mem(addr + 4, 4, true);
            r[rt] = v1; r[rd8] = v2;
        } else {
            write_mem(addr, 4, r[rt], true);
            write_mem(addr + 4, 4, r[rd8], true);
        }
        if (w) set_reg(rn, off);
        return;
    }
    if (op1 == 1 && op2 == 0) {
        int rd = hw2 & 0xF;
        uint32_t addr = r[rn];
        int size = (op3 == 4) ? 1 : (op3 == 5) ? 2 : 0;
        if (!size) undefined();
        if (excl_monitor && excl_addr == addr) { write_mem(addr, size, r[rt], true); r[rd] = 0; }
        else r[rd] = 1;
        excl_monitor = false;
        return;
    }
    if (op1 == 1 && op2 == 1) {
        int rm = hw2 & 0xF;
        uint32_t base = (rn == 15) ? r[15] : r[rn];
        if (op3 == 0 || op3 == 1) {
            uint32_t half = (op3 == 0) ? read_mem(base + r[rm], 1) : read_mem(base + (r[rm] << 1), 2);
            branch_to(r[15] + 2 * half);
            return;
        }
        if (op3 == 4 || op3 == 5) {
            int size = (op3 == 4) ? 1 : 2;
            r[rt] = read_mem(r[rn], size, true);
            excl_monitor = true; excl_addr = r[rn];
            return;
        }
    }
    undefined();
}

void Cpu::exec32_ldst_single(uint32_t hw1, uint32_t hw2) {
    bool sgn = (hw1 >> 8) & 1;
    bool u12 = (hw1 >> 7) & 1;
    int sz = (hw1 >> 5) & 3;
    bool load = (hw1 >> 4) & 1;
    int rn = hw1 & 0xF, rt = (hw2 >> 12) & 0xF;
    int size = (sz == 0) ? 1 : (sz == 1) ? 2 : 4;
    if (sz == 3) undefined();
    if (!load && (sgn || rn == 15)) undefined();
    uint32_t addr;
    bool wb = false;
    uint32_t wbval = 0;
    if (rn == 15) {
        uint32_t base = r[15] & ~3u;
        addr = u12 ? base + (hw2 & 0xFFF) : base - (hw2 & 0xFFF);
    } else if (u12) {
        addr = r[rn] + (hw2 & 0xFFF);
    } else if (hw2 & 0x800) {
        bool p = (hw2 >> 10) & 1, u = (hw2 >> 9) & 1, w = (hw2 >> 8) & 1;
        if (!p && !w) undefined();
        uint32_t imm = hw2 & 0xFF;
        uint32_t off = u ? r[rn] + imm : r[rn] - imm;
        addr = p ? off : r[rn];
        if (w) { wb = true; wbval = off; }
    } else if (((hw2 >> 6) & 0x3F) == 0) {
        addr = r[rn] + (r[hw2 & 0xF] << ((hw2 >> 4) & 3));
    } else {
        undefined();
        return;
    }
    if (load) {
        if (rt == 15 && size != 4) { if (wb) set_reg(rn, wbval); return; }
        uint32_t v = read_mem(addr, size);
        if (sgn) v = sext(v, size * 8);
        if (wb) set_reg(rn, wbval);
        if (rt == 15) bx_write_pc(v);
        else set_reg(rt, v);
    } else {
        write_mem(addr, size, r[rt]);
        if (wb) set_reg(rn, wbval);
    }
}

void Cpu::exec32_dp_register(uint32_t hw1, uint32_t hw2) {
    int op1 = (hw1 >> 4) & 0xF, op2 = (hw2 >> 4) & 0xF;
    int rn = hw1 & 0xF, rd = (hw2 >> 8) & 0xF, rm = hw2 & 0xF;
    if ((hw2 & 0xF000) != 0xF000) undefined();
    if ((op1 & 8) == 0 && op2 == 0) {
        bool s = op1 & 1;
        bool c;
        uint32_t res = shift_c(r[rn], (op1 >> 1) & 3, r[rm] & 0xFF, flagC, c);
        set_reg(rd, res);
        if (s) { set_nz(res); flagC = c; }
        return;
    }
    if ((op1 & 8) == 0 && (op2 & 8)) {
        uint32_t rotated = ror32(r[rm], ((hw2 >> 4) & 3) * 8);
        uint32_t ext;
        switch (op1) {
        case 0: ext = sext(rotated, 16); break;
        case 1: ext = rotated & 0xFFFF; break;
        case 4: ext = sext(rotated, 8); break;
        case 5: ext = rotated & 0xFF; break;
        default: undefined(); return;
        }
        set_reg(rd, (rn == 15) ? ext : r[rn] + ext);
        return;
    }
    if ((op1 & 0xC) == 0x8 && (op2 & 0xC) == 0x8) {
        int o1 = op1 & 3, o2 = op2 & 3;
        uint32_t x = r[rm];
        if (o1 == 1) {
            switch (o2) {
            case 0: set_reg(rd, __builtin_bswap32(x)); return;
            case 1: set_reg(rd, ((x & 0x00FF00FFu) << 8) | ((x & 0xFF00FF00u) >> 8)); return;
            case 2: {
                uint32_t res = 0;
                for (int i = 0; i < 32; i++) if (x & (1u << i)) res |= 1u << (31 - i);
                set_reg(rd, res);
                return;
            }
            case 3: set_reg(rd, sext(((x & 0xFF) << 8) | ((x >> 8) & 0xFF), 16)); return;
            }
        }
        if (o1 == 3 && o2 == 0) { set_reg(rd, x ? __builtin_clz(x) : 32); return; }
    }
    undefined();
}

void Cpu::exec32_mul(uint32_t hw1, uint32_t hw2) {
    int op1 = (hw1 >> 4) & 7, op2 = (hw2 >> 4) & 3;
    int rn = hw1 & 0xF, ra = (hw2 >> 12) & 0xF, rd = (hw2 >> 8) & 0xF, rm = hw2 & 0xF;
    if (op1 == 0 && op2 == 0) {
        uint32_t res = r[rn] * r[rm];
        if (ra != 15) res += r[ra];
        set_reg(rd, res);
        return;
    }
    if (op1 == 0 && op2 == 1) { set_reg(rd, r[ra] - r[rn] * r[rm]); return; }
    undefined();
}

void Cpu::exec32_longmul(uint32_t hw1, uint32_t hw2) {
    int op1 = (hw1 >> 4) & 7, op2 = (hw2 >> 4) & 0xF;
    int rn = hw1 & 0xF, rdlo = (hw2 >> 12) & 0xF, rdhi = (hw2 >> 8) & 0xF, rm = hw2 & 0xF;
    switch (op1) {
    case 0:
        if (op2 == 0) { int64_t p = (int64_t)(int32_t)r[rn] * (int64_t)(int32_t)r[rm]; r[rdlo] = (uint32_t)p; r[rdhi] = (uint32_t)((uint64_t)p >> 32); return; }
        break;
    case 1:
        if (op2 == 0xF) {
            int32_t a = (int32_t)r[rn], b = (int32_t)r[rm];
            uint32_t res;
            if (b == 0) { if (ccr & 0x10) throw CpuFault{F_DIVBYZERO, cur_instr_addr}; res = 0; }
            else if (a == INT32_MIN && b == -1) res = (uint32_t)INT32_MIN;
            else res = (uint32_t)(a / b);
            set_reg(rdhi, res);
            cycles += 4;
            return;
        }
        break;
    case 2:
        if (op2 == 0) { uint64_t p = (uint64_t)r[rn] * r[rm]; r[rdlo] = (uint32_t)p; r[rdhi] = (uint32_t)(p >> 32); return; }
        break;
    case 3:
        if (op2 == 0xF) {
            uint32_t b = r[rm], res;
            if (b == 0) { if (ccr & 0x10) throw CpuFault{F_DIVBYZERO, cur_instr_addr}; res = 0; }
            else res = r[rn] / b;
            set_reg(rdhi, res);
            cycles += 4;
            return;
        }
        break;
    case 4:
        if (op2 == 0) {
            int64_t acc = (int64_t)(((uint64_t)r[rdhi] << 32) | r[rdlo]);
            int64_t p = (int64_t)(int32_t)r[rn] * (int64_t)(int32_t)r[rm] + acc;
            r[rdlo] = (uint32_t)p; r[rdhi] = (uint32_t)((uint64_t)p >> 32);
            return;
        }
        break;
    case 6:
        if (op2 == 0) {
            uint64_t acc = ((uint64_t)r[rdhi] << 32) | r[rdlo];
            uint64_t p = (uint64_t)r[rn] * r[rm] + acc;
            r[rdlo] = (uint32_t)p; r[rdhi] = (uint32_t)(p >> 32);
            return;
        }
        break;
    }
    undefined();
}

// ---------------------------------------------------------------- excepciones y NVIC

int Cpu::exc_prio(int exc) const {
    if (exc == EXC_RESET) return -3;
    if (exc == EXC_NMI) return -2;
    if (exc == EXC_HARDFAULT) return -1;
    return prio[exc];
}

int Cpu::group_prio(int p) const {
    if (p < 0) return p;
    int shift = (int)prigroup + 1;
    return (p >> shift) << shift;
}

int Cpu::exec_priority() {
    int highest = 256;
    uint64_t a = act_mask & ~3ull;
    while (a) {
        int e = __builtin_ctzll(a);
        a &= a - 1;
        int g = group_prio(exc_prio(e));
        if (g < highest) highest = g;
    }
    int boosted = 256;
    if (basepri & 0xF8) boosted = group_prio((int)(basepri & 0xF8));
    if (primask & 1) boosted = 0;
    if (faultmask & 1) boosted = -1;
    return boosted < highest ? boosted : highest;
}

int Cpu::highest_pending() {
    uint64_t c = pend_mask & (en_mask | 0xFFFFull) & ~3ull;
    int best = 0, bestp = 1000;
    while (c) {
        int e = __builtin_ctzll(c);
        c &= c - 1;
        int p = exc_prio(e);
        if (p < bestp) { bestp = p; best = e; }
    }
    return best;
}

bool Cpu::can_take_exception() {
    if (nvic_dirty) { cached_pending = highest_pending(); cached_exec = exec_priority(); nvic_dirty = false; }
    return cached_pending != 0;
}

void Cpu::set_pending(int exc) {
    if (exc <= 0 || exc >= MAX_EXC) return;
    if (!((pend_mask >> (exc)) & 1)) { pend_mask |= 1ull << (exc); nvic_dirty = true; }
    if (exc >= EXC_IRQ0 && ((en_mask >> (exc)) & 1)) sleeping = false;
    else if (exc < EXC_IRQ0) sleeping = false;
}

void Cpu::clear_pending(int exc) {
    if (exc <= 0 || exc >= MAX_EXC) return;
    if (((pend_mask >> (exc)) & 1)) { pend_mask &= ~(1ull << (exc)); nvic_dirty = true; }
}

void Cpu::raise_fault(int kind, uint32_t addr) {
    int exc;
    const char *name;
    switch (kind) {
    case F_BUS_DATA: exc = EXC_BUSFAULT; cfsr |= (1u << 9) | (1u << 15); bfar = addr; name = "BusFault (acceso a una dirección sin memoria ni periférico)"; break;
    case F_BUS_FETCH: exc = EXC_BUSFAULT; cfsr |= 1u << 8; name = "BusFault (ejecución fuera de la memoria de programa)"; break;
    case F_UNDEF: exc = EXC_USAGEFAULT; cfsr |= 1u << 16; name = "UsageFault (instrucción no definida o no soportada por el Cortex-M3)"; break;
    case F_INVSTATE: exc = EXC_USAGEFAULT; cfsr |= 1u << 17; name = "UsageFault (salto a dirección sin bit Thumb)"; break;
    case F_INVPC: exc = EXC_USAGEFAULT; cfsr |= 1u << 18; name = "UsageFault (EXC_RETURN inválido)"; break;
    case F_UNALIGNED: exc = EXC_USAGEFAULT; cfsr |= 1u << 24; name = "UsageFault (acceso desalineado)"; break;
    case F_DIVBYZERO: exc = EXC_USAGEFAULT; cfsr |= 1u << 25; name = "UsageFault (división por cero)"; break;
    case F_BKPT: exc = EXC_HARDFAULT; hfsr |= 1u << 31; name = "HardFault (BKPT sin depurador)"; break;
    default: exc = EXC_USAGEFAULT; cfsr |= 1u << 19; name = "UsageFault (coprocesador no disponible)"; break;
    }
    char buf[200];
    snprintf(buf, sizeof(buf), "%s en PC=0x%08X, dirección 0x%08X", name, cur_instr_addr, addr);
    log(buf);
    bool enabled_fault = (exc == EXC_BUSFAULT && (shcsr & (1u << 17))) || (exc == EXC_USAGEFAULT && (shcsr & (1u << 18)));
    int ep = exec_priority();
    if (exc == EXC_HARDFAULT && ep <= -1) {
        lockup = true;
        log("LOCKUP: falla dentro de HardFault/NMI. " + state_dump());
        return;
    }
    if (exc != EXC_HARDFAULT && (!enabled_fault || group_prio(exc_prio(exc)) >= ep)) {
        if (ep <= -1) {
            lockup = true;
            log("LOCKUP: falla dentro de HardFault/NMI. " + state_dump());
            return;
        }
        hfsr |= 1u << 30;
        exc = EXC_HARDFAULT;
        log("Escala a HardFault. " + state_dump());
    }
    pend_mask |= 1ull << (exc);
    nvic_dirty = true;
}

void Cpu::take_exception(int exc, uint32_t return_addr) {
    uint32_t forcealign = (ccr >> 9) & 1;
    uint32_t sp = r[13];
    uint32_t frameptralign = (sp >> 2) & 1 & forcealign;
    uint32_t frameptr = (sp - 0x20) & ~(forcealign << 2);
    uint32_t psr = (xpsr() & ~(1u << 9)) | (frameptralign << 9);
    try {
        write_mem(frameptr + 0x00, 4, r[0]);
        write_mem(frameptr + 0x04, 4, r[1]);
        write_mem(frameptr + 0x08, 4, r[2]);
        write_mem(frameptr + 0x0C, 4, r[3]);
        write_mem(frameptr + 0x10, 4, r[12]);
        write_mem(frameptr + 0x14, 4, r[14]);
        write_mem(frameptr + 0x18, 4, return_addr);
        write_mem(frameptr + 0x1C, 4, psr);
    } catch (CpuFault &) {
        lockup = true;
        log("LOCKUP: la pila no apunta a RAM al entrar a una excepción. " + state_dump());
        return;
    }
    r[13] = frameptr;
    if (handler_mode) r[14] = 0xFFFFFFF1;
    else if (control & 2) r[14] = 0xFFFFFFFD;
    else r[14] = 0xFFFFFFF9;
    bool was_psp = using_psp();
    handler_mode = true;
    control &= ~2u;
    if (was_psp) { psp = r[13]; r[13] = msp; }
    uint32_t vec;
    try {
        vec = bus_read(vtor + 4 * (uint32_t)exc, 4);
    } catch (CpuFault &) {
        lockup = true;
        log("LOCKUP: no se pudo leer el vector de la excepción");
        return;
    }
    if ((vec & 1) == 0) {
        char buf[120];
        snprintf(buf, sizeof(buf), "Vector de la excepción %d sin bit Thumb (0x%08X): falta el handler en la tabla de vectores", exc, vec);
        log(buf);
    }
    pend_mask &= ~(1ull << (exc));
    act_mask |= 1ull << (exc);
    ipsr = (uint32_t)exc;
    event_register = true;
    itstate = 0;
    pc = vec & ~1u;
    excl_monitor = false;
    nvic_dirty = true;
}

void Cpu::exception_return(uint32_t exc_return) {
    int returning = (int)ipsr;
    uint32_t low = exc_return & 0xF;
    if (low != 1 && low != 9 && low != 0xD) throw CpuFault{F_INVPC, exc_return};
    if (returning > 0 && returning < MAX_EXC) act_mask &= ~(1ull << (returning));
    if (returning != EXC_NMI) faultmask = 0;
    if (low == 1) {
        handler_mode = true;
    } else {
        handler_mode = false;
        if (low == 0xD) { control |= 2u; msp = r[13]; r[13] = psp; }
        else control &= ~2u;
    }
    uint32_t fp = r[13];
    uint32_t psr;
    r[0] = read_mem(fp + 0x00, 4);
    r[1] = read_mem(fp + 0x04, 4);
    r[2] = read_mem(fp + 0x08, 4);
    r[3] = read_mem(fp + 0x0C, 4);
    r[12] = read_mem(fp + 0x10, 4);
    r[14] = read_mem(fp + 0x14, 4);
    uint32_t ret = read_mem(fp + 0x18, 4);
    psr = read_mem(fp + 0x1C, 4);
    uint32_t spadj = ((ccr >> 9) & 1) && ((psr >> 9) & 1) ? 4 : 0;
    r[13] = fp + 0x20 + spadj;
    flagN = (psr >> 31) & 1; flagZ = (psr >> 30) & 1; flagC = (psr >> 29) & 1; flagV = (psr >> 28) & 1; flagQ = (psr >> 27) & 1;
    itstate = ((psr >> 25) & 3) | (((psr >> 10) & 0x3F) << 2);
    ipsr = handler_mode ? (psr & 0x1FF) : 0;
    exc_returned = true;
    event_register = true;
    next_pc = ret & ~1u;
    branched = true;
    excl_monitor = false;
    nvic_dirty = true;
    if (!handler_mode && (scr & 2)) sleeping = true;
    io->cpu_exception_return(returning);
}

bool Cpu::scs_read(uint32_t addr, int size, uint32_t &v) {
    uint32_t off = addr - 0xE000E000u;
    uint32_t aligned = off & ~3u;
    uint32_t w = 0;
    if (aligned >= 0x400 && aligned < 0x4F0) {
        int base = 16 + (int)(aligned - 0x400);
        for (int i = 0; i < 4; i++) if (base + i < MAX_EXC) w |= (uint32_t)prio[base + i] << (8 * i);
    } else if (aligned >= 0xD18 && aligned < 0xD24) {
        int base = 4 + (int)(aligned - 0xD18);
        for (int i = 0; i < 4; i++) w |= (uint32_t)prio[base + i] << (8 * i);
    } else if (aligned >= 0x100 && aligned < 0x380) {
        int group = (int)((aligned & 0x7F) >> 2);
        int kind = (int)((aligned - 0x100) >> 7);
        for (int b = 0; b < 32; b++) {
            int e = 16 + group * 32 + b;
            if (e >= 16 + num_irqs) break;
            bool bit = false;
            if (kind == 0 || kind == 1) bit = ((en_mask >> (e)) & 1);
            else if (kind == 2 || kind == 3) bit = ((pend_mask >> (e)) & 1);
            else if (kind == 4) bit = ((act_mask >> (e)) & 1);
            if (bit) w |= 1u << b;
        }
    } else {
        switch (aligned) {
        case 0x004: w = (uint32_t)((num_irqs + 31) / 32 - 1); break;
        case 0xD00: w = 0x412FC230; break;
        case 0xD04: {
            w = ipsr & 0x1FF;
            int hp = highest_pending();
            w |= (uint32_t)(hp & 0x1FF) << 12;
            bool isr = false;
            for (int e = 16; e < 16 + num_irqs; e++) if (((pend_mask >> (e)) & 1)) { isr = true; break; }
            if (isr) w |= 1u << 22;
            int nact = 0;
            for (int e = 2; e < 16 + num_irqs; e++) if (((act_mask >> (e)) & 1)) nact++;
            if (nact <= 1) w |= 1u << 11;
            if (((pend_mask >> (EXC_SYSTICK)) & 1)) w |= 1u << 26;
            if (((pend_mask >> (EXC_PENDSV)) & 1)) w |= 1u << 28;
            if (((pend_mask >> (EXC_NMI)) & 1)) w |= 1u << 31;
            break;
        }
        case 0xD08: w = vtor; break;
        case 0xD0C: w = 0xFA050000u | (prigroup << 8); break;
        case 0xD10: w = scr; break;
        case 0xD14: w = ccr; break;
        case 0xD24: {
            w = shcsr & 0x70000;
            if (((act_mask >> (EXC_MEMMANAGE)) & 1)) w |= 1;
            if (((act_mask >> (EXC_BUSFAULT)) & 1)) w |= 2;
            if (((act_mask >> (EXC_USAGEFAULT)) & 1)) w |= 8;
            if (((act_mask >> (EXC_SVCALL)) & 1)) w |= 1u << 7;
            if (((act_mask >> (EXC_DEBUGMON)) & 1)) w |= 1u << 8;
            if (((act_mask >> (EXC_PENDSV)) & 1)) w |= 1u << 10;
            if (((act_mask >> (EXC_SYSTICK)) & 1)) w |= 1u << 11;
            if (((pend_mask >> (EXC_USAGEFAULT)) & 1)) w |= 1u << 12;
            if (((pend_mask >> (EXC_MEMMANAGE)) & 1)) w |= 1u << 13;
            if (((pend_mask >> (EXC_BUSFAULT)) & 1)) w |= 1u << 14;
            if (((pend_mask >> (EXC_SVCALL)) & 1)) w |= 1u << 15;
            break;
        }
        case 0xD28: w = cfsr; break;
        case 0xD2C: w = hfsr; break;
        case 0xD34: w = mmfar; break;
        case 0xD38: w = bfar; break;
        default: w = 0; break;
        }
    }
    uint32_t sh = (off & 3) * 8;
    v = w >> sh;
    if (size == 1) v &= 0xFF;
    else if (size == 2) v &= 0xFFFF;
    return true;
}

bool Cpu::scs_write(uint32_t addr, int size, uint32_t v) {
    uint32_t off = addr - 0xE000E000u;
    if ((off >= 0x400 && off < 0x4F0) || (off >= 0xD18 && off < 0xD24)) {
        int base = (off >= 0xD18) ? 4 + (int)(off - 0xD18) : 16 + (int)(off - 0x400);
        for (int i = 0; i < size; i++) {
            int e = base + i;
            if (e >= 4 && e < MAX_EXC) prio[e] = (uint8_t)((v >> (8 * i)) & 0xF8);
        }
        nvic_dirty = true;
        return true;
    }
    uint32_t aligned = off & ~3u;
    uint32_t w = v << ((off & 3) * 8);
    uint32_t wm = w;
    if (size < 4 && (aligned == 0xD08 || aligned == 0xD0C || aligned == 0xD10 || aligned == 0xD14 || aligned == 0xD24)) {
        uint32_t lane = (size == 2 ? 0xFFFFu : 0xFFu) << ((off & 3) * 8);
        uint32_t cur = 0;
        scs_read(0xE000E000u + aligned, 4, cur);
        wm = (cur & ~lane) | (w & lane);
    }
    if (aligned >= 0x100 && aligned < 0x300) {
        int group = (int)((aligned & 0x7F) >> 2);
        int kind = (int)((aligned - 0x100) >> 7);
        if (group > 1) return true;
        uint64_t valid = (num_irqs >= 64) ? ~0ull : ((1ull << num_irqs) - 1);
        uint64_t bits = (((uint64_t)w << (32 * group)) & valid) << 16;
        switch (kind) {
        case 0: en_mask |= bits; break;
        case 1: en_mask &= ~bits; break;
        case 2: pend_mask |= bits; break;
        default: pend_mask &= ~bits; break;
        }
        nvic_dirty = true;
        if ((kind == 0 || kind == 3) && bits) io->cpu_irq_recheck(bits >> 16);
        return true;
    }
    switch (aligned) {
    case 0xD04:
        if (w & (1u << 31)) pend_mask |= 1ull << (EXC_NMI);
        if (w & (1u << 28)) pend_mask |= 1ull << (EXC_PENDSV);
        if (w & (1u << 27)) pend_mask &= ~(1ull << (EXC_PENDSV));
        if (w & (1u << 26)) pend_mask |= 1ull << (EXC_SYSTICK);
        if (w & (1u << 25)) pend_mask &= ~(1ull << (EXC_SYSTICK));
        nvic_dirty = true;
        break;
    case 0xD08: vtor = wm & 0x3FFFFF80u; break;
    case 0xD0C:
        if ((wm >> 16) == 0x05FA) {
            prigroup = (wm >> 8) & 7;
            if (wm & 4) { log("SYSRESETREQ: el firmware pidió un reset"); io->cpu_reset_request(); }
            nvic_dirty = true;
        }
        break;
    case 0xD10: scr = wm & 0x16; break;
    case 0xD14: ccr = wm & 0x31B; break;
    case 0xD24: shcsr = wm & 0x70000; nvic_dirty = true; break;
    case 0xD28: cfsr &= ~w; break;
    case 0xD2C: hfsr &= ~w; break;
    case 0xF00: {
        int e = 16 + (int)(w & 0x1FF);
        if (e < 16 + num_irqs) { pend_mask |= 1ull << (e); nvic_dirty = true; }
        break;
    }
    default: break;
    }
    return true;
}

} // namespace lpcsim
