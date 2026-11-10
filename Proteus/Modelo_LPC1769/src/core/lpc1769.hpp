// Modelo del microcontrolador LPC1769: memoria, relojes, pines y periféricos.
// Referencia: UM10360 (LPC176x/5x User Manual), rev. 4.1.
// El tiempo se mide en picosegundos (la misma unidad que DSIM de Proteus).
#pragma once
#include "cpu.hpp"
#include <cstdint>
#include <deque>
#include <set>
#include <string>
#include <vector>

namespace lpcsim {

enum class Drive : uint8_t { FLOAT, WEAK_LOW, WEAK_HIGH, STRONG_LOW, STRONG_HIGH };

class SocHost {
public:
    virtual ~SocHost() {}
    virtual void pin_output(int64_t t_ps, int port, int bit, Drive d) = 0;
    virtual double analog_input(int port, int bit) = 0;
    virtual void soc_log(const std::string &msg) = 0;
};

enum PinFunc : uint8_t {
    PF_GPIO, PF_OTHER, PF_ANALOG,
    PF_TXD0, PF_TXD1, PF_TXD2, PF_TXD3,
    PF_RXD0, PF_RXD1, PF_RXD2, PF_RXD3,
    PF_PWM1, PF_PWM2, PF_PWM3, PF_PWM4, PF_PWM5, PF_PWM6,
    PF_CAP00, PF_CAP01, PF_CAP10, PF_CAP11, PF_CAP20, PF_CAP21, PF_CAP30, PF_CAP31,
    PF_MAT00, PF_MAT01, PF_MAT10, PF_MAT11, PF_MAT20, PF_MAT21, PF_MAT22, PF_MAT23, PF_MAT30, PF_MAT31,
    PF_AD0, PF_AD1, PF_AD2, PF_AD3, PF_AD4, PF_AD5, PF_AD6, PF_AD7,
    PF_EINT0, PF_EINT1, PF_EINT2, PF_EINT3
};

// Contador con preescala (base de TIMER0-3 y PWM1), calculado en función del tiempo.
struct Counter {
    double ps_per_pclk = 40000.0;
    uint32_t pr = 0;
    bool enabled = false;
    int64_t base_t = 0;
    uint32_t tc_base = 0;
    uint64_t pc_base = 0;
    bool fresh = false;      // en base_t el TC acaba de pasar a tc_base (por reset en match)
    bool reset_on_enable = false;  // se deshabilitó con un reset por match pendiente
    uint32_t hold_val = 0;   // valor del TC antes de base_t (cuando base_t está en el futuro)
    uint32_t tc_at(int64_t t) const;
    uint32_t pc_at(int64_t t) const;
    void rebase(int64_t t);
    int64_t time_tc_reaches(uint32_t v) const;
    int64_t tick_ps() const { return (int64_t)((pr + 1.0) * ps_per_pclk + 0.5); }
};

class Lpc1769 : public CpuIo {
public:
    static const int64_t NEVER = INT64_MAX;

    explicit Lpc1769(SocHost *host);
    ~Lpc1769() override;

    void set_xtal(double hz) { xtal_hz = hz; }
    void set_vref(double v) { vref = v; }
    bool load_program(const std::string &path, std::string &err);
    void reset(int64_t t_ps);
    void run_until(int64_t t_ps);
    void input_edge(int64_t t_ps, int port, int bit, bool level);
    void set_pin_present(int port, int bit, bool present, bool level);
    void hold_reset(int64_t t_ps, bool held);
    int64_t now() const;
    Drive pin_drive(int port, int bit) const { return drive[port][bit]; }
    double cclk() const { return cclk_hz; }
    Cpu &core() { return cpu; }
    uint64_t instructions() const { return cpu.cycles; }

    // CpuIo
    bool io_read(uint32_t addr, int size, uint32_t &value) override;
    bool io_write(uint32_t addr, int size, uint32_t value) override;
    void cpu_log(const std::string &msg) override;
    void cpu_reset_request() override { reset_requested = true; }
    void cpu_exception_return(int exc) override;
    void cpu_irq_recheck(uint64_t irq_mask) override;

private:
    SocHost *host;
    Cpu cpu;
    std::vector<uint8_t> flash, sram0, sram1, bootrom;
    double xtal_hz = 12e6;
    double vref = 3.3;
    double cclk_hz = 4e6;
    double ps_per_cycle = 250000.0;
    int64_t seg_t0 = 0;
    uint64_t seg_c0 = 0;
    bool in_reset = false;
    bool reset_requested = false;
    std::set<std::string> logged;

    // ---- System control
    uint32_t flashcfg = 0x303A, pll0con = 0, pll0cfg = 0, pll0con_act = 0, pll0cfg_act = 0, pll0feed = 0;
    uint32_t pll1con = 0, pll1cfg = 0, pll1con_act = 0, pll1cfg_act = 0, pll1feed = 0;
    uint32_t cclkcfg = 0, usbclkcfg = 0, clksrcsel = 0, scs = 0, pconp = 0x042887DE;
    uint32_t pclksel0 = 0, pclksel1 = 0, extint = 0, extmode = 0, extpolar = 0, pcon = 0, clkoutcfg = 0;
    void update_cclk();
    void set_cclk(double hz);

    // ---- Pines
    uint32_t pinsel[11] = {0};
    uint32_t pinmode[10] = {0};
    uint32_t pinmode_od[5] = {0};
    uint32_t fiodir[5] = {0}, fiomask[5] = {0}, fioout[5] = {0};
    uint32_t in_level[5] = {0};
    uint32_t present[5] = {0};
    Drive drive[5][32];
    PinFunc func_of(int port, int bit) const;
    Drive pull_drive(int port, int bit) const;
    Drive compute_drive(int port, int bit) const;
    void update_pin(int port, int bit, int64_t t);
    void update_port(int port, int64_t t);
    void update_all_pins(int64_t t);
    void update_func_pins(PinFunc f, int64_t t);
    bool pin_level(int port, int bit) const;
    uint32_t fiopin_read(int port) const;

    // ---- GPIO interrupts / EINT
    uint32_t io_int_en_r[3] = {0}, io_int_en_f[3] = {0}, io_int_stat_r[3] = {0}, io_int_stat_f[3] = {0};
    void eint_input(int n, bool level, bool edge_rise, bool edge_fall);
    void eint_recheck();
    void adc_hw_trigger(uint32_t code, bool rising, int64_t t);

    // ---- Timers
    struct Timer {
        int idx;
        uint32_t ir = 0, tcr = 0, mcr = 0, mr[4] = {0}, ccr = 0, cr[2] = {0}, emr = 0, ctcr = 0;
        Counter c;
        int64_t next_t = NEVER;
    } tim[4];
    void timer_resched(Timer &tm, int64_t t);
    void timer_event(Timer &tm, int64_t t);
    void timer_capture(Timer &tm, int ch, bool rising, int64_t t);
    void timer_write(Timer &tm, uint32_t off, uint32_t v, int64_t t);
    uint32_t timer_read(Timer &tm, uint32_t off, int64_t t);

    // ---- PWM1
    struct Pwm {
        uint32_t ir = 0, tcr = 0, mcr = 0, mr[7] = {0}, mr_sh[7] = {0}, ccr = 0, cr[4] = {0}, pcr = 0, ler = 0, ctcr = 0;
        Counter c;
        bool out[7] = {false};
        int64_t next_t = NEVER;
        int64_t cycle_t = NEVER;
    } pwm;
    void pwm_resched(int64_t t);
    void pwm_event(int64_t t);
    void pwm_cycle_start(int64_t t);
    void pwm_set_out(int ch, bool v, int64_t t);
    void pwm_write(uint32_t off, uint32_t v, int64_t t);
    uint32_t pwm_read(uint32_t off, int64_t t);

    // ---- ADC
    struct Adc {
        uint32_t cr = 0x1, gdr = 0, inten = 0x100, dr[8] = {0}, stat = 0, trm = 0x0F00;
        int ch = -1;
        int64_t done_t = NEVER;
    } adc;
    void adc_start(int64_t t, bool from_burst);
    void adc_event(int64_t t);
    void adc_update_irq();

    // ---- UART
    struct Uart {
        int idx;
        uint8_t rx_fifo[16]; int rx_head = 0, rx_count = 0;
        uint8_t tx_fifo[16]; int tx_head = 0, tx_count = 0;
        uint32_t ier = 0, lcr = 0, lsr_err = 0, scr = 0, acr = 0, icr = 0, fdr = 0x10, ter = 0x80;
        uint32_t dll = 1, dlm = 0, fcr = 0;
        bool fifo_en = false;
        bool thre_int = false;
        bool txd = true;
        bool tx_busy = false;
        uint32_t tx_frame = 0; int tx_bits = 0; int tx_pos = 0;
        int64_t tx_t = NEVER;
        bool rx_busy = false; int64_t rx_start = 0; int rx_pos = 0; uint32_t rx_frame = 0; int rx_bits = 0;
        int64_t rx_t = NEVER;
        int64_t cti_t = NEVER;
        bool cti = false;
        bool rxd_level = true;
    } uart[4];
    double uart_bit_ps(const Uart &u) const;
    int uart_frame_bits(const Uart &u) const;
    void uart_write(Uart &u, uint32_t off, uint32_t v, int64_t t);
    uint32_t uart_read(Uart &u, uint32_t off, int64_t t);
    void uart_tx_start(Uart &u, int64_t t);
    void uart_event(Uart &u, int64_t t);
    void uart_rx_edge(Uart &u, bool level, int64_t t);
    void uart_update_irq(Uart &u);
    uint32_t uart_iir(Uart &u);

    // ---- SysTick
    uint32_t st_ctrl = 0, st_load = 0, st_val0 = 0;
    int64_t st_base = 0;
    int64_t st_next = NEVER;
    bool st_countflag = false;
    uint32_t systick_val(int64_t t) const;
    void systick_resched(int64_t t);
    void systick_event(int64_t t);

    // ---- Entradas
    struct Edge { int64_t t; int port, bit; bool level; };
    std::deque<Edge> inputs;
    void process_input(const Edge &e);

    // ---- Planificación
    int64_t next_event_time() const;
    void process_due(int64_t t);
    void irq(int n);
    bool irq_level(int n);
    void warn_once(const std::string &key, const std::string &msg);
    bool powered(uint32_t pconp_bit, const char *name);
    double pclk(int sel_reg, int field) const;
    void rebase_peripherals(int64_t t);
    void reset_peripherals();
};

} // namespace lpcsim
