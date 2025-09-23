#include "vrMatrixUtils.h"
#include "include/Matrices.h"
#include "openvr_mingw.hpp"
//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
extern "C" Matrix4 vrSteamVRMtx34ToMat4(const vr::HmdMatrix34_t &matPose)
{
    Matrix4 matrixObj(
        matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
        matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
        matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
        matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f);
    return matrixObj;
}
