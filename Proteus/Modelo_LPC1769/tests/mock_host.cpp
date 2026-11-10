// Host de simulación que imita a Proteus (interfaces de vsm.hpp) para probar en Linux
// el adaptador VSM y el modelo del HC-SR04 junto con el firmware del TPI sin modificar.
//
// Circuito simulado:
//   U1 = LPC1769 (modelo mixto)    U2 = HC-SR04 (modelo mixto)
//   P0.5 ── TRIG        ECHO ── P0.4        P0.23 = tensión del LM35 (nodo analógico)
//   DIST = tensión que fija la distancia (100 mm/V)
//   P0.3 ← PC (UART)    P0.2 → PC          P2.0 = PWM    P2.2/P0.22/P2.3 = alarma
#include "../src/vsm/vsm.hpp"
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "../src/vsm/models.hpp"
extern "C" IMIXEDMODEL *createmixedmodel(CHAR *device, ILICENCESERVER *ils);
extern "C" VOID deletemixedmodel(IMIXEDMODEL *model);
extern "C" IDSIMMODEL *createdsimmodel(CHAR *device, ILICENCESERVER *ils);
extern "C" VOID deletedsimmodel(IDSIMMODEL *model);

static const ABSTIME US = 1000000, MS = 1000 * US, SEC = 1000 * MS;

struct Sim;
struct Net;

struct Event {
    ABSTIME t;
    uint64_t seq;
    std::function<void()> fn;
    bool operator>(const Event &o) const { return t != o.t ? t > o.t : seq > o.seq; }
};

struct Sim {
    ABSTIME now = 0;
    uint64_t seq = 0;
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> q;
    bool suspended = false;
    void at(ABSTIME t, std::function<void()> fn) { q.push(Event{t, seq++, std::move(fn)}); }
    void run_until(ABSTIME t) {
        while (!q.empty() && q.top().t <= t && !suspended) {
            Event e = q.top();
            q.pop();
            if (e.t < now) { printf("ERROR: evento en el pasado (%lld < %lld)\n", (long long)e.t, (long long)now); }
            now = e.t;
            e.fn();
        }
        if (!suspended) now = t;
    }
};

static int strength_of(STATE s) { return s & SS_MASK; }

struct Pin;
struct Model;

struct Net {
    std::string name;
    std::vector<Pin *> pins;
    STATE ext = FLT;
    STATE state = FLT;
    std::vector<std::pair<ABSTIME, bool>> history;
    void resolve(Sim &sim);
};

struct Model {
    IDSIMMODEL *dsim = nullptr;
};

struct Pin : IDSIMPIN2 {
    Sim *sim;
    Net *net;
    Model *owner;
    STATE drv = FLT;
    Pin(Sim *s, Net *n, Model *o) : sim(s), net(n), owner(o) { n->pins.push_back(this); }
    BOOL invert() override { return FALSE; }
    STATE istate() override { return net->state; }
    BOOL issteady() override { return TRUE; }
    INT activity() override { return 0; }
    BOOL isactive() override { return ishigh(net->state); }
    BOOL isinactive() override { return islow(net->state); }
    BOOL isposedge() override { return FALSE; }
    BOOL isnegedge() override { return FALSE; }
    BOOL isedge() override { return FALSE; }
    VOID setstate(STATE) override { printf("ERROR: el modelo llamó a setstate (sobrecarga riesgosa)\n"); }
    EVENT *setstate(ABSTIME, RELTIME, STATE) override { printf("ERROR: el modelo llamó a setstate (sobrecarga riesgosa)\n"); return nullptr; }
    EVENT *setstate(ABSTIME, RELTIME, RELTIME, RELTIME, STATE) override { printf("ERROR: el modelo llamó a setstate\n"); return nullptr; }
    VOID sethandler(IDSIMMODEL *, PINHANDLERFN) override {}
    DSIMNODE getnode() override { return net; }
    STATE getstate() override { return net->state; }
    VOID settiming(RELTIME, RELTIME, RELTIME) override {}
    VOID setstates(STATE, STATE, STATE) override {}
    EVENT *drivebool(ABSTIME t, BOOL f) override { return drivestate(t, f ? SHI : SLO); }
    EVENT *drivestate(ABSTIME t, STATE s) override {
        if (t < sim->now) printf("ERROR: drivestate en el pasado en %s (%lld < %lld)\n", net->name.c_str(), (long long)t, (long long)sim->now);
        sim->at(t, [this, s]() { drv = s; net->resolve(*sim); });
        return nullptr;
    }
    EVENT *drivetristate(ABSTIME t) override { return drivestate(t, FLT); }
};

void Net::resolve(Sim &sim) {
    STATE best = ext;
    for (auto *p : pins)
        if (strength_of(p->drv) > strength_of(best)) best = p->drv;
    if (best == state) return;
    bool oldhi = ishigh(state);
    state = best;
    if (ishigh(state) != oldhi || history.empty()) history.push_back({sim.now, (bool)ishigh(state)});
    std::vector<Model *> notified;
    for (auto *p : pins) {
        bool seen = false;
        for (auto *m : notified) if (m == p->owner) seen = true;
        if (seen || !p->owner || !p->owner->dsim) continue;
        notified.push_back(p->owner);
        p->owner->dsim->simulate(sim.now, DSIMNORMAL);
    }
}

struct Popup : IDEBUGPOPUP {
    std::string tag;
    VOID print(CHAR *msg, ...) override {
        va_list ap; va_start(ap, msg);
        char b[512]; vsnprintf(b, sizeof(b), msg, ap); va_end(ap);
        printf("  [%s] %s", tag.c_str(), b);
    }
    VOID dump(const BYTE *, UINT, UINT) override {}
};

struct Instance : IINSTANCE {
    std::string ident;
    std::map<std::string, std::string> props;
    std::map<std::string, Pin *> dpins;
    std::map<std::string, SPICENODE> anodes;
    Popup popup;
    Sim *sim;
    std::vector<std::string> errors;
    CHAR *id() override { return (CHAR *)ident.c_str(); }
    CHAR *value() override { return (CHAR *)""; }
    CHAR *getstrval(CHAR *name, CHAR *defval) override {
        auto it = props.find(name);
        return it == props.end() ? defval : (CHAR *)it->second.c_str();
    }
    VOID getnumval(DOUBLE *r, CHAR *name, DOUBLE defval) override { auto it = props.find(name); *r = it == props.end() ? defval : atof(it->second.c_str()); }
    BOOL getboolval(CHAR *, BOOL d) override { return d; }
    DWORD gethexval(CHAR *, DWORD d) override { return d; }
    LONG getinitval(CHAR *, LONG d) override { return d; }
    RELTIME getdelay(CHAR *, RELTIME d) override { return d; }
    IACTIVEMODEL *getactivemodel() override { return nullptr; }
    IINSTANCE *getinterfacemodel() override { return nullptr; }
    BOOL getmoddata(BYTE **, DWORD *) override { return FALSE; }
    SPICENODE getspicenode(CHAR *name, BOOL) override { auto it = anodes.find(name); return it == anodes.end() ? 0 : it->second; }
    IDSIMPIN *getdsimpin(CHAR *name, BOOL required) override {
        auto it = dpins.find(name);
        if (it == dpins.end()) { if (required) printf("ERROR: pin requerido %s no existe\n", name); return nullptr; }
        return it->second;
    }
    VOID log(CHAR *, ...) override {}
    VOID warning(CHAR *msg, ...) override { va_list ap; va_start(ap, msg); char b[512]; vsnprintf(b, sizeof(b), msg, ap); va_end(ap); printf("  WARNING %s\n", b); }
    VOID error(CHAR *msg, ...) override { va_list ap; va_start(ap, msg); char b[512]; vsnprintf(b, sizeof(b), msg, ap); va_end(ap); printf("  ERROR %s\n", b); errors.push_back(b); }
    VOID fatal(CHAR *msg, ...) override { va_list ap; va_start(ap, msg); char b[512]; vsnprintf(b, sizeof(b), msg, ap); va_end(ap); printf("  FATAL %s\n", b); errors.push_back(b); }
    BOOL message(CHAR *, ...) override { return TRUE; }
    IPOPUP *createpopup(CREATEPOPUPSTRUCT *cps) override { popup.tag = cps->caption; return &popup; }
    VOID deletepopup(POPUPID) override {}
    BOOL setvdmhlr(class ICPU *) override { return FALSE; }
    BOOL loadmemory(CHAR *, VOID *, UINT, UINT, UINT) override { return FALSE; }
    IBUSPIN *getbuspin(CHAR *, IDSIMPIN **, UINT) override { return nullptr; }
    IBUSPIN *getbuspin(CHAR *, UINT, UINT, BOOL) override { return nullptr; }
};

struct DsimCkt : IDSIMCKT {
    Sim *sim;
    VOID sysvar(DOUBLE *r, DSIMVARS) override { *r = 0; }
    EVENT *setcallback(ABSTIME t, IDSIMMODEL *m, EVENTID id) override {
        if (t < sim->now) printf("ERROR: setcallback en el pasado\n");
        sim->at(t, [m, t, id]() { m->callback(t, id); });
        return nullptr;
    }
    BOOL cancelcallback(EVENT *, IDSIMMODEL *) override { return FALSE; }
    VOID setbreak(ABSTIME) override {}
    VOID suspend(IINSTANCE *, CHAR *msg) override { printf("  SUSPEND: %s\n", msg); sim->suspended = true; }
    EVENT *setcallbackex(ABSTIME, IDSIMMODEL *, CALLBACKHANDLERFN, EVENTID) override { return nullptr; }
    DSIMNODE newnode(CHAR *, CHAR *) override { return nullptr; }
    IDSIMPIN *newpin(IINSTANCE *, DSIMNODE, CHAR *, DWORD) override { return nullptr; }
    EVENT *setclockcallback(ABSTIME, RELTIME, IDSIMMODEL *, CALLBACKHANDLERFN, EVENTID) override { return nullptr; }
};

struct SpiceCkt : ISPICECKT {
    double dummy = 0;
    BOOL ismode(SPICEMODES) override { return FALSE; }
    VOID sysvar(DOUBLE *r, SPICEVARS) override { *r = 0; }
    DOUBLE &statevar(INT, INT) override { return dummy; }
    DOUBLE &rhs(SPICENODE) override { return dummy; }
    DOUBLE &rhsold(SPICENODE) override { return dummy; }
    DOUBLE &irhs(SPICENODE) override { return dummy; }
    DOUBLE &irhsold(SPICENODE) override { return dummy; }
    SPICENODE getnode(CHAR *) override { return 0; }
    SPICENODE newvoltnode(CHAR *, CHAR *) override { return 0; }
    SPICENODE newcurnode(CHAR *, CHAR *) override { return 0; }
    VOID delnode(SPICENODE) override {}
    DOUBLE *allocsmp(SPICENODE, SPICENODE) override { return &dummy; }
    BOOL setbreak(REALTIME) override { return TRUE; }
    VOID suspend(IINSTANCE *, CHAR *) override {}
    INT allocvars(INT) override { return 0; }
    VOID integrate(DOUBLE *, DOUBLE *, DOUBLE, INT) override {}
    VOID truncerr(INT, DOUBLE *) override {}
};

struct Licence : ILICENCESERVER {
    BOOL authorize(DWORD, DWORD) override { return TRUE; }
    BOOL authorizeex(DWORD, DWORD, DWORD) override { return TRUE; }
    DWORD getid() override { return 0; }
    CHAR *getname() override { return (CHAR *)"mock"; }
    CHAR *getcompany() override { return (CHAR *)"mock"; }
    CHAR *getexpirydate() override { return (CHAR *)""; }
};

// ------------------------------------------------------------ banco de pruebas

static bool lvl_at(const std::vector<std::pair<ABSTIME, bool>> &h, ABSTIME t, bool def) {
    bool v = def;
    for (auto &x : h) { if (x.first > t) break; v = x.second; }
    return v;
}

static std::string decode_uart(const Net &n, ABSTIME from, ABSTIME to, double baud) {
    double bit = 1e12 / baud;
    std::string out;
    size_t i = 0;
    auto &h = n.history;
    while (i < h.size()) {
        if (h[i].first < from || h[i].second) { i++; continue; }
        ABSTIME t0 = h[i].first;
        if (t0 > to) break;
        int c = 0;
        for (int k = 0; k < 8; k++) if (lvl_at(h, t0 + (ABSTIME)((k + 1.5) * bit), true)) c |= 1 << k;
        out += (char)c;
        ABSTIME tend = t0 + (ABSTIME)(9.6 * bit);
        while (i < h.size() && h[i].first < tend) i++;
    }
    return out;
}

static double duty(const Net &n, ABSTIME from, ABSTIME to) {
    ABSTIME high = 0, t = from;
    bool v = lvl_at(n.history, from, false);
    for (auto &x : n.history) {
        if (x.first <= from) continue;
        if (x.first >= to) break;
        if (v) high += x.first - t;
        t = x.first; v = x.second;
    }
    if (v) high += to - t;
    return 100.0 * (double)high / (double)(to - from);
}

static std::string last_line(const std::string &s) {
    size_t e = s.rfind('\n');
    if (e == std::string::npos) return "";
    size_t b = s.rfind('\n', e - 1);
    size_t st = (b == std::string::npos) ? 0 : b + 1;
    return s.substr(st, e - st);
}

static int fails = 0;
static void check(bool ok, const std::string &w) { printf("%s %s\n", ok ? "  OK   " : "  FALLA", w.c_str()); if (!ok) fails++; }

int main(int argc, char **argv) {
    std::string prog = argc > 1 ? argv[1] : "tests/firmware/build/tpi.axf";
    bool digital_only = argc > 2 && std::string(argv[2]) == "digital";
    Sim sim;
    Licence lic;
    DsimCkt dckt; dckt.sim = &sim;
    SpiceCkt sckt;

    std::map<std::string, std::unique_ptr<Net>> nets;
    auto net = [&](const std::string &n) -> Net * { auto &p = nets[n]; if (!p) { p.reset(new Net); p->name = n; } return p.get(); };

    // --- U1: LPC1769
    Model m1;
    Instance u1; u1.ident = "U1"; u1.sim = &sim;
    u1.props["PROGRAM"] = prog;
    u1.props["XTAL"] = "12MHz";
    u1.props["QUANTUM"] = "10u";
    u1.props["ADC0"] = "0.25";
    IMIXEDMODEL *lpc_mixed = nullptr;
    IDSIMMODEL *lpc_d;
    if (digital_only) lpc_d = createdsimmodel((CHAR *)"LPC1769", &lic);
    else { lpc_mixed = createmixedmodel((CHAR *)"LPC1769", &lic); lpc_d = lpc_mixed; }
    m1.dsim = lpc_d;
    const char *u1pins[] = {"P0.2", "P0.3", "P0.4", "P0.5", "P0.22", "P2.0", "P2.2", "P2.3", "P0.23", "P0.9", "P2.13"};
    std::vector<std::string> analog_u1;
    for (auto *pn : u1pins) {
        bool isA = lpc_mixed ? static_cast<ISPICEMODEL *>(lpc_mixed)->isanalog((CHAR *)pn) : FALSE;
        bool isD = lpc_d->isdigital((CHAR *)pn);
        if (isA) analog_u1.push_back(pn);
        if (isD) u1.dpins[pn] = new Pin(&sim, net(std::string("N_") + pn), &m1);
        if (isA && isD) printf("  aviso: %s analógico y digital a la vez\n", pn);
    }
    check(analog_u1.size() == (digital_only ? 0u : 1u) && (analog_u1.empty() || analog_u1[0] == "P0.23"), "P0.23 se declara analógico (sólo en modo mixto)");
    if (!digital_only) u1.anodes["P0.23"] = 1;

    // --- U2: HC-SR04
    Model m2;
    Instance u2; u2.ident = "U2"; u2.sim = &sim;
    u2.props["MMPERVOLT"] = "100";
    u2.props["DISTANCE"] = "200";
    IMIXEDMODEL *sonar_mixed = digital_only ? nullptr : static_cast<IMIXEDMODEL *>(new vsmmodels::SonarModel(true));
    IDSIMMODEL *sonar_d = digital_only ? static_cast<IDSIMMODEL *>(new vsmmodels::SonarModel(false)) : sonar_mixed;
    m2.dsim = sonar_d;
    sonar_d->isdigital((CHAR *)"TRIG");
    sonar_d->isdigital((CHAR *)"ECHO");
    if (sonar_mixed) static_cast<ISPICEMODEL *>(sonar_mixed)->isanalog((CHAR *)"DIST");
    u2.dpins["TRIG"] = new Pin(&sim, net("N_P0.5"), &m2);
    u2.dpins["ECHO"] = new Pin(&sim, net("N_P0.4"), &m2);
    if (!digital_only) u2.anodes["DIST"] = 2;

    // Tensiones analógicas: nodo 1 = LM35, nodo 2 = DIST
    double rhs[3] = {0, 0.25, 2.0};
    net("N_P0.3")->ext = WHI;
    net("N_P0.3")->state = WHI;
    auto analog_tick = [&]() {
        if (lpc_mixed) static_cast<ISPICEMODEL *>(lpc_mixed)->accept(realtime(sim.now), rhs);
        if (sonar_mixed) static_cast<ISPICEMODEL *>(sonar_mixed)->accept(realtime(sim.now), rhs);
    };

    if (lpc_mixed) static_cast<ISPICEMODEL *>(lpc_mixed)->setup(&u1, &sckt);
    if (sonar_mixed) static_cast<ISPICEMODEL *>(sonar_mixed)->setup(&u2, &sckt);
    lpc_d->setup(&u1, &dckt);
    sonar_d->setup(&u2, &dckt);
    analog_tick();
    lpc_d->simulate(0, DSIMBOOT);
    sonar_d->simulate(0, DSIMBOOT);
    lpc_d->runctrl(RM_START);

    std::function<void()> tick;
    tick = [&]() { analog_tick(); sim.at(sim.now + MS, tick); };
    sim.at(MS, tick);

    auto send_line = [&](ABSTIME t, const std::string &s) {
        double bit = 1e12 / 9600.0;
        Net *n = net("N_P0.3");
        for (char ch : s) {
            unsigned c = (unsigned char)ch;
            for (int k = 0; k < 10; k++) {
                bool b = k == 0 ? false : k == 9 ? true : ((c >> (k - 1)) & 1);
                sim.at(t + (ABSTIME)(k * bit), [n, b, &sim]() { n->ext = b ? IHI : ILO; n->resolve(sim); });
            }
            t += (ABSTIME)(10 * bit);
        }
    };

    auto w0 = std::chrono::steady_clock::now();
    double baud = 25e6 / (16.0 * 163);
    Net *txd = net("N_P0.2"), *pwm = net("N_P2.0");

    printf("Etapa 1: DIST = 2,0 V (200 mm -> nivel 20 mm), LM35 0,25 V\n");
    sim.run_until(2 * SEC);
    std::string l = last_line(decode_uart(*txd, 0, 2 * SEC, baud));
    printf("  telemetría: %s\n", l.c_str());
    if (digital_only) check(l.rfind("L=", 0) == 0 && l.find("T=24.9") != std::string::npos, "modo digital: T sale de la propiedad ADC0 = 0,25 V");
    else check(l == "L=20,SP=0,T=24.9,PWM=0,FAN=0,ALM=0,ERR=0,MODO=A", "trama esperada a través de Proteus simulado");

    printf("Etapa 2: SP=120 por la UART\n");
    send_line(2 * SEC, "SP=120\n");
    sim.run_until(4 * SEC);
    l = last_line(decode_uart(*txd, 2 * SEC, 4 * SEC, baud));
    printf("  telemetría: %s\n", l.c_str());
    check(l.find("SP=120") != std::string::npos && l.find("PWM=100") != std::string::npos, "SP aceptado, PWM al 100 %");
    check(duty(*pwm, 3 * SEC, 4 * SEC) > 99.5, "duty en la red de P2.0");

    if (!digital_only) {
        printf("Etapa 3: DIST = 1,1 V (110 mm -> nivel 110 mm)\n");
        rhs[2] = 1.1;
        sim.run_until(6 * SEC);
        l = last_line(decode_uart(*txd, 5 * SEC, 6 * SEC, baud));
        printf("  telemetría: %s\n", l.c_str());
        double d = duty(*pwm, 5500 * MS, 6 * SEC);
        check(l.find("L=110") != std::string::npos && d > 10 && d < 40, "nivel 110 mm y duty parcial " + std::to_string(d) + " %");

        printf("Etapa 4: LM35 0,46 V\n");
        rhs[1] = 0.46;
        sim.run_until(7600 * MS);
        l = last_line(decode_uart(*txd, 7 * SEC, 7600 * MS, baud));
        printf("  telemetría: %s\n", l.c_str());
        check(l.find("FAN=1,ALM=1") != std::string::npos, "alarma encendida");
        check(ishigh(net("N_P2.2")->state) && ishigh(net("N_P0.22")->state) && ishigh(net("N_P2.3")->state), "redes del ventilador, LED y buzzer en alto");
    }

    printf("Etapa 5: pines sin usar con pull-up interno\n");
    check(net("N_P0.9")->state == WHI && net("N_P2.13")->state == WHI, "P0.9 y P2.13 quedan en WHI (pull-up por defecto)");

    lpc_d->runctrl(RM_STOP);
    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    printf("\n%.1f s simulados en %.2f s reales (%.2fx), errores del host: %zu\n", realtime(sim.now), wall, realtime(sim.now) / wall, u1.errors.size() + u2.errors.size());
    if (lpc_mixed) deletemixedmodel(lpc_mixed); else deletedsimmodel(lpc_d);
    delete dynamic_cast<vsmmodels::SonarModel *>(sonar_d);
    printf("RESULTADO: %d fallas\n", fails);
    return fails ? 1 : 0;
}
