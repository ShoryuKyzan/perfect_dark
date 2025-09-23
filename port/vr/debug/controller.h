#ifndef VR_DEBUG_CONTROLLER_H
#define VR_DEBUG_CONTROLLER_H

#include "openvr_mingw.hpp"
#include "../include/Matrices.h"

#ifdef __cplusplus
extern "C" {
#endif

void controllerDebugInit(vr::IVRSystem *pHmd);
void controllerDebugCleanup();
void controllerDebugRender(float projectionMatrix[4][4]);

#ifdef __cplusplus
}
#endif

#endif