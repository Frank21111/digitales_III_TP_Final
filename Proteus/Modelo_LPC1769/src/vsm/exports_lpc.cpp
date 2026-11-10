// Exportaciones de LPC1769.DLL: cualquier dispositivo que la use es un LPC1769.
#include "models.hpp"
#include <new>

#if defined(_WIN32)
#define VSM_EXPORT extern "C" __declspec(dllexport)
#else
#define VSM_EXPORT extern "C" __attribute__((visibility("default")))
#endif

using vsmmodels::LpcModel;

VSM_EXPORT IDSIMMODEL *createdsimmodel(CHAR *device, ILICENCESERVER *ils) {
    if (ils && !ils->authorize(0)) return nullptr;
    try { return static_cast<IDSIMMODEL *>(new LpcModel(false, device)); } catch (...) { return nullptr; }
}

VSM_EXPORT VOID deletedsimmodel(IDSIMMODEL *model) {
    delete dynamic_cast<LpcModel *>(model);
}

VSM_EXPORT IMIXEDMODEL *createmixedmodel(CHAR *device, ILICENCESERVER *ils) {
    if (ils && !ils->authorize(0)) return nullptr;
    try { return static_cast<IMIXEDMODEL *>(new LpcModel(true, device)); } catch (...) { return nullptr; }
}

VSM_EXPORT VOID deletemixedmodel(IMIXEDMODEL *model) {
    delete dynamic_cast<LpcModel *>(model);
}
