// Exportaciones de LPC1769.DLL. Sirve a los dos modelos: si el nombre del dispositivo contiene "SR04"
// crea el HC-SR04 y, si no, el LPC1769. Así el circuito del TPI carga una sola DLL propia: Proteus 8.13
// no autoriza una segunda DLL de terceros en la misma simulación.
#include "models.hpp"
#include <cctype>
#include <new>
#include <string>

#if defined(_WIN32)
#define VSM_EXPORT extern "C" __declspec(dllexport)
#else
#define VSM_EXPORT extern "C" __attribute__((visibility("default")))
#endif

using vsmmodels::LpcModel;
using vsmmodels::SonarModel;

static bool is_sonar(const CHAR *device) {
    std::string d = device ? device : "";
    for (auto &c : d) c = (char)toupper((unsigned char)c);
    return d.find("SR04") != std::string::npos;
}

template <class T> static void destroy(T *model) {
    if (auto *lpc = dynamic_cast<LpcModel *>(model)) delete lpc;
    else delete dynamic_cast<SonarModel *>(model);
}

VSM_EXPORT IDSIMMODEL *createdsimmodel(CHAR *device, ILICENCESERVER *ils) {
    if (ils && !ils->authorize(0)) return nullptr;
    try {
        if (is_sonar(device)) return static_cast<IDSIMMODEL *>(new SonarModel(false));
        return static_cast<IDSIMMODEL *>(new LpcModel(false, device));
    } catch (...) { return nullptr; }
}

VSM_EXPORT VOID deletedsimmodel(IDSIMMODEL *model) {
    destroy(model);
}

VSM_EXPORT IMIXEDMODEL *createmixedmodel(CHAR *device, ILICENCESERVER *ils) {
    if (ils && !ils->authorize(0)) return nullptr;
    try {
        if (is_sonar(device)) return static_cast<IMIXEDMODEL *>(new SonarModel(true));
        return static_cast<IMIXEDMODEL *>(new LpcModel(true, device));
    } catch (...) { return nullptr; }
}

VSM_EXPORT VOID deletemixedmodel(IMIXEDMODEL *model) {
    destroy(model);
}
