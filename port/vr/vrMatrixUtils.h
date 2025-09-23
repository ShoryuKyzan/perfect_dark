#ifndef VR_MATRIX_UTILS_H
#define VR_MATRIX_UTILS_H

#include "openvr_mingw.hpp"
#include "include/Matrices.h"


#ifdef __cplusplus
extern "C" {
#endif

Matrix4 vrSteamVRMtx34ToMat4(const vr::HmdMatrix34_t &matPose);

#ifdef __cplusplus
}
#endif


#endif // VR_MATRIX_UTILS_H