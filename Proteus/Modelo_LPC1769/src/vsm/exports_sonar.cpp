// Exportaciones de HCSR04.DLL: cualquier dispositivo que la use es un HC-SR04.
#include "models.hpp"
#include <new>

#if defined(_WIN32)
#define VSM_EXPORT extern "C" __declspec(dllexport)
#else
#define VSM_EXPORT extern "C" __attribute__((visibility("default")))
#endif

using vsmmodels::SonarModel;

VSM_EXPORT IDSIMMODEL *createdsimmodel(CHAR *, ILICENCESERVER *ils) {
    if (ils && !ils->authorize(0)) return nullptr;
    try { return static_cast<IDSIMMODEL *>(new SonarModel(false)); } catch (...) { return nullptr; }
}

VSM_EXPORT VOID deletedsimmodel(IDSIMMODEL *model) {
    delete dynamic_cast<SonarModel *>(model);
}

VSM_EXPORT IMIXEDMODEL *createmixedmodel(CHAR *, ILICENCESERVER *ils) {
    if (ils && !ils->authorize(0)) return nullptr;
    try { return static_cast<IMIXEDMODEL *>(new SonarModel(true)); } catch (...) { return nullptr; }
}

VSM_EXPORT VOID deletemixedmodel(IMIXEDMODEL *model) {
    delete dynamic_cast<SonarModel *>(model);
}
