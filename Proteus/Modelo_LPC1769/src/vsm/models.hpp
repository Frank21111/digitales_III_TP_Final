// Modelos VSM: LPC1769 (LPC1769.DLL) y HC-SR04 (HCSR04.DLL), en variante mixta o digital.
#pragma once
#include "vsm.hpp"
#include "../core/lpc1769.hpp"
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace vsmmodels {

bool parse_port_pin(const char *name, int &port, int &bit);
std::string prop_str(IINSTANCE *inst, const char *name, const char *def);
// Número con prefijo SI opcional (p n u m k M meg G). 'unit' (p. ej. "Hz") se acepta y se ignora.
// Si queda texto sin interpretar, 'bad' se pone en true.
double parse_num(const std::string &s, double def, const char *unit, bool &bad);

class Logger {
public:
    void attach(IINSTANCE *inst, const char *caption, POPUPID id, bool popup);
    void print(const std::string &msg);
    void log_only(const std::string &msg);
private:
    IINSTANCE *inst = nullptr;
    IDEBUGPOPUP *popup = nullptr;
};

class LpcModel final : public IMIXEDMODEL, public lpcsim::SocHost {
public:
    LpcModel(bool mixed, const char *device);
    ~LpcModel();

    INT isanalog(CHAR *pinname) override;
    INT isdigital(CHAR *pinname) override;
    VOID runctrl(RUNMODES mode) override;
    VOID actuate(REALTIME time, ACTIVESTATE newstate) override;
    BOOL indicate(REALTIME time, ACTIVEDATA *newstate) override;
    VOID setup(IINSTANCE *inst, ISPICECKT *spice) override;
    VOID dcload(REALTIME time, SPICEMODES mode, DOUBLE *oldrhs, DOUBLE *newrhs) override;
    VOID acload(SPICEFREQ omega, DOUBLE *rhs, DOUBLE *irhs) override;
    VOID trunc(REALTIME time, REALTIME *newtimestep) override;
    VOID accept(REALTIME time, DOUBLE *rhs) override;
    VOID setup(IINSTANCE *inst, IDSIMCKT *dsim) override;
    VOID simulate(ABSTIME time, DSIMMODES mode) override;
    VOID callback(ABSTIME time, EVENTID eventid) override;

    void pin_output(int64_t t_ps, int port, int bit, lpcsim::Drive d) override;
    double analog_input(int port, int bit) override;
    void soc_log(const std::string &msg) override;

private:
    struct DPin {
        std::string name;
        int port = -1, bit = -1;
        IDSIMPIN *pin = nullptr;
        bool level = false;
        std::deque<std::pair<ABSTIME, STATE>> sched;  // salidas propias ya programadas
        STATE own = FLT;                              // última salida propia vigente
    };
    struct APin {
        std::string name;
        int port = -1, bit = -1;
        SPICENODE node = 0;
        DOUBLE *smp = nullptr;
        double volts = 0;
    };
    bool mixed;
    std::string device;
    IINSTANCE *inst = nullptr;
    IDSIMCKT *dsim = nullptr;
    ISPICECKT *spice = nullptr;
    std::unique_ptr<lpcsim::Lpc1769> soc;
    std::vector<DPin> dpins;
    std::vector<APin> apins;
    std::vector<std::string> digital_names, analog_names;
    int dindex[5][32];
    int aindex[5][32];
    IDSIMPIN *reset_pin = nullptr;
    bool reset_level = true;
    double adc_fallback[8] = {0};
    double analog_step = 1e-3;
    int64_t quantum = 10000000;
    int64_t now_t = 0;
    EVENTID chain = 1;
    bool callback_seen = false;
    bool loaded = false;
    bool running = false;
    bool stop_logged = false;
    bool break_on_fault = true;
    bool dsim_ready = false;
    std::vector<std::pair<int, lpcsim::Drive>> boot_outputs;
    std::set<std::string> warned;
    Logger logger;

    void init_common(IINSTANCE *inst);
    double num_prop(const char *name, double def, const char *unit);
    bool is_analog_pin(int port, int bit) const;
    void collect_pins();
    void flush_boot(ABSTIME time);
    void start_chain(ABSTIME time);
    void report(const char *where);
    static STATE to_state(lpcsim::Drive d);
};

class SonarModel final : public IMIXEDMODEL {
public:
    explicit SonarModel(bool mixed);
    INT isanalog(CHAR *pinname) override;
    INT isdigital(CHAR *pinname) override;
    VOID runctrl(RUNMODES mode) override;
    VOID actuate(REALTIME time, ACTIVESTATE newstate) override;
    BOOL indicate(REALTIME time, ACTIVEDATA *newstate) override;
    VOID setup(IINSTANCE *inst, ISPICECKT *spice) override;
    VOID dcload(REALTIME time, SPICEMODES mode, DOUBLE *oldrhs, DOUBLE *newrhs) override;
    VOID acload(SPICEFREQ omega, DOUBLE *rhs, DOUBLE *irhs) override;
    VOID trunc(REALTIME time, REALTIME *newtimestep) override;
    VOID accept(REALTIME time, DOUBLE *rhs) override;
    VOID setup(IINSTANCE *inst, IDSIMCKT *dsim) override;
    VOID simulate(ABSTIME time, DSIMMODES mode) override;
    VOID callback(ABSTIME time, EVENTID eventid) override;

private:
    bool mixed;
    IINSTANCE *inst = nullptr;
    IDSIMCKT *dsim = nullptr;
    IDSIMPIN *trig = nullptr, *echo = nullptr;
    SPICENODE dist_node = 0;
    DOUBLE *dist_smp = nullptr;
    double dist_volts = 0;
    bool dist_sampled = false;
    double mm_per_volt = 100;
    double distance_mm = 200;
    double delay_s = 250e-6;
    bool trig_level = false;
    ABSTIME trig_rise = -1;
    ABSTIME busy_until = 0;
    bool started = false;
    Logger logger;
    double current_mm() const;
};

} // namespace vsmmodels
