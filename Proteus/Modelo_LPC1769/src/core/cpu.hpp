// Intérprete ARMv7-M (Cortex-M3): Thumb/Thumb-2 entero, excepciones, NVIC y SCB.
// El SysTick y el resto de los periféricos viven en el SoC (CpuIo).
#pragma once
#include <cstdint>
#include <string>

namespace lpcsim {

class CpuIo {
public:
    virtual ~CpuIo() {}
    virtual bool io_read(uint32_t addr, int size, uint32_t &value) = 0;
    virtual bool io_write(uint32_t addr, int size, uint32_t value) = 0;
    virtual void cpu_log(const std::string &msg) = 0;
    virtual void cpu_reset_request() {}
    virtual void cpu_exception_return(int exc) { (void)exc; }
    virtual void cpu_irq_recheck(uint64_t irq_mask) { (void)irq_mask; }
};

struct CpuFault {
    int kind;
    uint32_t addr;
};

class Cpu {
public:
    enum Exc { EXC_RESET = 1, EXC_NMI = 2, EXC_HARDFAULT = 3, EXC_MEMMANAGE = 4, EXC_BUSFAULT = 5,
               EXC_USAGEFAULT = 6, EXC_SVCALL = 11, EXC_DEBUGMON = 12, EXC_PENDSV = 14, EXC_SYSTICK = 15,
               EXC_IRQ0 = 16 };
    enum FaultKind { F_BUS_DATA, F_BUS_FETCH, F_UNDEF, F_INVSTATE, F_INVPC, F_UNALIGNED, F_DIVBYZERO, F_NOCP, F_BKPT };
    static const int MAX_EXC = 64;

    explicit Cpu(CpuIo *io, int num_irqs = 35);

    void add_region(uint32_t base, uint32_t size, uint8_t *mem, bool writable);
    void reset();
    // Ejecuta instrucciones hasta consumir 'budget' ciclos, o hasta stop_request / sueño / lockup.
    uint64_t run(uint64_t budget);

    // Estado visible
    uint32_t r[16];
    uint32_t pc;
    bool flagN, flagZ, flagC, flagV, flagQ;
    uint32_t itstate;
    uint32_t ipsr;
    uint32_t primask, faultmask, basepri, control;
    uint32_t msp, psp;
    uint64_t cycles;
    bool stop_request;
    bool sleeping;
    bool lockup;
    bool event_register;
    uint32_t vtor;

    // NVIC / SCB
    void set_pending(int exc);
    void clear_pending(int exc);
    bool scs_read(uint32_t addr, int size, uint32_t &v);
    bool scs_write(uint32_t addr, int size, uint32_t v);
    bool can_take_exception();   // hay una excepción pendiente que despierta/interrumpe

    uint32_t xpsr() const;
    std::string state_dump() const;

    // Accesos a memoria del punto de vista del núcleo (usados por el SoC para el bit-band)
    uint32_t mem_read(uint32_t addr, int size);
    void mem_write(uint32_t addr, int size, uint32_t v);

private:
    CpuIo *io;
    int num_irqs;
    struct Region { uint32_t base, size; uint8_t *mem; bool writable; };
    Region regions[8];
    int nregions;

    // NVIC
    uint8_t prio[MAX_EXC];
    uint64_t pend_mask, act_mask, en_mask;
    const uint8_t *fetch_mem = nullptr;
    uint32_t fetch_base = 0, fetch_size = 0;
    uint32_t prigroup;
    uint32_t shcsr;
    uint32_t ccr;
    uint32_t scr;
    uint32_t cfsr, hfsr, mmfar, bfar;
    bool nvic_dirty;
    int cached_pending;
    int cached_exec;
    bool handler_mode;

    uint32_t next_pc;
    bool branched;
    bool exc_returned = false;
    uint32_t cur_instr_addr;
    bool excl_monitor;
    uint32_t excl_addr;

    // núcleo
    int step();
    void exec16(uint32_t hw);
    void exec32(uint32_t hw1, uint32_t hw2);
    void exec32_dp_shifted(uint32_t hw1, uint32_t hw2);
    void exec32_dp_modimm(uint32_t hw1, uint32_t hw2);
    void exec32_dp_plainimm(uint32_t hw1, uint32_t hw2);
    void exec32_branch_misc(uint32_t hw1, uint32_t hw2);
    void exec32_ldst_multiple(uint32_t hw1, uint32_t hw2);
    void exec32_ldst_dual_excl(uint32_t hw1, uint32_t hw2);
    void exec32_ldst_single(uint32_t hw1, uint32_t hw2);
    void exec32_dp_register(uint32_t hw1, uint32_t hw2);
    void exec32_mul(uint32_t hw1, uint32_t hw2);
    void exec32_longmul(uint32_t hw1, uint32_t hw2);

    bool cond_pass(uint32_t cond) const;
    bool in_it() const { return (itstate & 0xF) != 0; }
    bool last_in_it() const { return (itstate & 0xF) == 0x8; }
    void it_advance();

    uint32_t reg(int n) const { return r[n]; }
    void set_reg(int n, uint32_t v);         // n==15 → BranchWritePC (ALUWritePC)
    void branch_to(uint32_t addr);           // BranchWritePC
    void bx_write_pc(uint32_t addr);         // BXWritePC / LoadWritePC
    void set_sp(uint32_t v) { r[13] = v & ~3u; }

    uint32_t add_with_carry(uint32_t x, uint32_t y, bool carry_in, bool &carry_out, bool &overflow) const;
    uint32_t shift_c(uint32_t value, int type, int amount, bool carry_in, bool &carry_out) const;
    uint32_t shift(uint32_t value, int type, int amount, bool carry_in) const;
    uint32_t thumb_expand_imm_c(uint32_t imm12, bool carry_in, bool &carry_out) const;
    void set_nz(uint32_t v) { flagN = (v >> 31) != 0; flagZ = v == 0; }

    uint16_t fetch16(uint32_t addr);
    uint32_t read_mem(uint32_t addr, int size, bool aligned_required = false);
    void write_mem(uint32_t addr, int size, uint32_t v, bool aligned_required = false);
    uint32_t bus_read(uint32_t addr, int size);
    void bus_write(uint32_t addr, int size, uint32_t v);

    // excepciones
    void take_exception(int exc, uint32_t return_addr);
    void exception_return(uint32_t exc_return);
    void raise_fault(int kind, uint32_t addr);
    int exec_priority();
    int group_prio(int p) const;
    int exc_prio(int exc) const;
    int highest_pending();
    void select_sp_process(bool use_psp);
    bool using_psp() const { return !handler_mode && (control & 2); }
    void undefined();
    void log(const std::string &s) { io->cpu_log(s); }
};

} // namespace lpcsim
