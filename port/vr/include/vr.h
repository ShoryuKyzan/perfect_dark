#ifndef VR_H
#define VR_H
#include "Matrices.h"
#include "openvr_mingw.hpp"
#include <PR/ultratypes.h>

#ifdef __cplusplus
extern "C" {
#endif

extern bool vrEnabled;
extern uint32_t vrRenderWidth;
extern uint32_t vrRenderHeight;
Matrix4 vrSteamVRMtx44ToMat4(const struct vr::HmdMatrix44_t &mtx);
Matrix4 vrSteamVRMtx34ToMat4(const struct vr::HmdMatrix34_t &matPose);
bool vrInit();
void vrShutdown();
void vrTick();
void vrLogSubmitResult(vr::EVRCompositorError error, u8 eye);
void vrMat4ToFloat44(float m[4][4], const Matrix4 &mat4);
extern "C" void vrGetCurrentProjectionMtx(float dest[4][4], vr::Hmd_Eye nEye);

#ifdef __cplusplus
}
#endif
#endif
