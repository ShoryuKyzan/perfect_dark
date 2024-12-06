#include <PR/ultratypes.h>
#include "include/Matrices.h"
#include "openvr_mingw.hpp"
#include "system.h"
#include "include/vr.h"

// VR
// external
bool vrEnabled;
uint32_t vrRenderWidth;
uint32_t vrRenderHeight;
// internal
vr::IVRSystem *m_pHMD;
vr::TrackedDevicePose_t vrTrackedDevicePoses[vr::k_unMaxTrackedDeviceCount];
Matrix4 mat4DevicePoseList[vr::k_unMaxTrackedDeviceCount];
Matrix4 mat4HMDPose;
Matrix4 mat4VRProjectionLeft;
Matrix4 mat4VRProjectionRight;
Matrix4 mat4VREyePosLeft;
Matrix4 mat4VREyePosRight;
Matrix4 mat4Camera;
Vector3 vecHMDPositionLast;
Vector3 vecHMDRotationLast;
Vector3 vecHMDPositionDiff;
Vector3 vecHMDRotationDiff;
float fNearClip = 0.1f;
float fFarClip = 9000.0f;
Matrix4 mat4ControllerPoseLeft;
Matrix4 mat4ControllerPoseRight;
bool controllerConnectedLeft;
bool controllerConnectedRight;

// Add this helper function at the top of the file
Vector3 getRotationFromMatrix(const Matrix4& mat) {
    Vector3 rot;
    // Extract Euler angles from rotation matrix
    // Assuming Y-up coordinate system
    rot.y = atan2(mat[2], mat[10]); // Yaw around Y axis
    rot.x = atan2(-mat[6], sqrt(mat[2]*mat[2] + mat[10]*mat[10])); // Pitch around X
    rot.z = atan2(mat[4], mat[5]); // Roll around Z
    return rot;
}

extern "C" Matrix4 vrSteamVRMtx44ToMat4(const vr::HmdMatrix44_t &mtx)
{
    Matrix4 matrixObj(
        mtx.m[0][0], mtx.m[1][0], mtx.m[2][0], mtx.m[3][0],
        mtx.m[0][1], mtx.m[1][1], mtx.m[2][1], mtx.m[3][1],
        mtx.m[0][2], mtx.m[1][2], mtx.m[2][2], mtx.m[3][2],
        mtx.m[0][3], mtx.m[1][3], mtx.m[2][3], mtx.m[3][3]);
    return matrixObj;
}
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


extern "C" void vrMat4ToFloat44(float m[4][4], const Matrix4 &mat4)
{
    float tmp[4][4];

    tmp[0][0] = mat4[0];
    tmp[0][1] = mat4[1];
    tmp[0][2] = mat4[2];
    tmp[0][3] = mat4[3];
    tmp[1][0] = mat4[4];
    tmp[1][1] = mat4[5];
    tmp[1][2] = mat4[6];
    tmp[1][3] = mat4[7];
    tmp[2][0] = mat4[8];
    tmp[2][1] = mat4[9];
    tmp[2][2] = mat4[10];
    tmp[2][3] = mat4[11],
    tmp[3][0] = mat4[12];
    tmp[3][1] = mat4[13];
    tmp[3][2] = mat4[14];
    tmp[3][3] = mat4[15];
    memcpy(m, tmp, sizeof(tmp));
}

extern "C" void vrFloat44ToMat4(Matrix4 &destMat4, float srcM[4][4])
{
    static float tmp[16];
    tmp[0] = srcM[0][0];
    tmp[1] = srcM[0][1];
    tmp[2] = srcM[0][2];
    tmp[3] = srcM[0][3];
    tmp[4] = srcM[1][0];
    tmp[5] = srcM[1][1];
    tmp[6] = srcM[1][2];
    tmp[7] = srcM[1][3];
    tmp[8] = srcM[2][0];
    tmp[9] = srcM[2][1];
    tmp[10] = srcM[2][2];
    tmp[11] = srcM[2][3];
    tmp[12] = srcM[3][0];
    tmp[13] = srcM[3][1];
    tmp[14] = srcM[3][2];
    tmp[15] = srcM[3][3];
    destMat4.set(tmp);
}


extern "C" bool vrInit()
{

    // Loading the SteamVR Runtime
    vr::EVRInitError eError = vr::VRInitError_None;
    m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
    char buf[1024];

    if (eError != vr::VRInitError_None)
    {
        m_pHMD = NULL;
        sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        sysLogPrintf(LOG_ERROR, buf);
        return false;
    }

    sysLogPrintf(LOG_NOTE, "vr init success");

    if (!vr::VRCompositor())
    {
        sysLogPrintf(LOG_ERROR, "vr compositor failed to init");
        vrEnabled = false;
        return false;
    }
    sysLogPrintf(LOG_NOTE, "vr compositor init success");

    // vr init
    m_pHMD->GetRecommendedRenderTargetSize(&vrRenderWidth, &vrRenderHeight);

    sprintf_s(buf, sizeof(buf), "vr got recommended sizes w %d h %d", vrRenderWidth, vrRenderHeight);
    sysLogPrintf(LOG_NOTE, buf);

    sysLogPrintf(LOG_NOTE, "vr initial waitgetposes");

    // Check if system is still connected
    bool connected = m_pHMD->IsTrackedDeviceConnected(vr::k_unTrackedDeviceIndex_Hmd);
    if (!connected)
    {
        sysLogPrintf(LOG_NOTE, "vr hmd not connected");
        return false;
    }
    else
    {
        sysLogPrintf(LOG_NOTE, "vr hmd connected");
    }
    vr::VRCompositor()->WaitGetPoses(vrTrackedDevicePoses, vr::k_unMaxTrackedDeviceCount, NULL, 0);

    sysLogPrintf(LOG_NOTE, "vr get eye proj/position matrices");
    vr::HmdMatrix44_t projMat = m_pHMD->GetProjectionMatrix(vr::Eye_Left, fNearClip, fFarClip);
    mat4VRProjectionLeft = vrSteamVRMtx44ToMat4(m_pHMD->GetProjectionMatrix(vr::Eye_Left, fNearClip, fFarClip));
    mat4VRProjectionRight = vrSteamVRMtx44ToMat4(m_pHMD->GetProjectionMatrix(vr::Eye_Right, fNearClip, fFarClip));

    sysLogPrintf(LOG_NOTE, "vr get eye position matrices");
    mat4VREyePosLeft = vrSteamVRMtx34ToMat4(m_pHMD->GetEyeToHeadTransform(vr::Eye_Left));
    mat4VREyePosLeft.invert();
    mat4VREyePosRight = vrSteamVRMtx34ToMat4(m_pHMD->GetEyeToHeadTransform(vr::Eye_Right));
    mat4VREyePosRight.invert();


    // init input bindings
    vr::VRInput()->SetActionManifestPath("pdvr_actions.json");


    vrEnabled = true;

    // after this it will do the framebuffer creation
    return true;
}


extern "C" void vrShutdown()
{
    if (m_pHMD && vrEnabled)
    {
        vr::VR_Shutdown();
        m_pHMD = NULL;
        vrEnabled = false;
        sysLogPrintf(LOG_NOTE, "vr shutdown");
    }
}

extern "C" void vrTick()
{
    static bool firstTick = false;
    Vector3 vecHMDPosNext;
    if (!m_pHMD)
        return;

    vr::VRCompositor()->WaitGetPoses(vrTrackedDevicePoses, vr::k_unMaxTrackedDeviceCount, NULL, 0);

    for (unsigned int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
    {
        if (vrTrackedDevicePoses[nDevice].bPoseIsValid)
        {
            mat4DevicePoseList[nDevice] = vrSteamVRMtx34ToMat4(vrTrackedDevicePoses[nDevice].mDeviceToAbsoluteTracking);
        }
    }

    if (vrTrackedDevicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
        mat4HMDPose = mat4DevicePoseList[vr::k_unTrackedDeviceIndex_Hmd];
        mat4HMDPose.invert();

        Vector3 vecHMDRotNext = getRotationFromMatrix(mat4HMDPose);
        
        if(firstTick) {
            vecHMDPositionLast.x = 0;
            vecHMDPositionLast.y = 0;
            vecHMDPositionLast.z = 0;
            vecHMDRotationLast = vecHMDRotNext;
            firstTick = false;
        } else {
            vecHMDPosNext = mat4HMDPose * vecHMDPosNext;
            vecHMDPositionDiff = vecHMDPositionLast - vecHMDPosNext;
            vecHMDRotationDiff = vecHMDRotationLast - vecHMDRotNext;
            vecHMDRotationLast = vecHMDRotNext;
        }
    }

    // Track controller poses
    if (vrTrackedDevicePoses[vr::k_unTrackedDeviceIndex_Controller0].bPoseIsValid) {
        mat4ControllerPoseLeft = vrSteamVRMtx34ToMat4(vrTrackedDevicePoses[vr::k_unTrackedDeviceIndex_Controller0].mDeviceToAbsoluteTracking);
        controllerConnectedLeft = true;
    } else {
        controllerConnectedLeft = false;
    }

    if (vrTrackedDevicePoses[vr::k_unTrackedDeviceIndex_Controller1].bPoseIsValid) {
        mat4ControllerPoseRight = vrSteamVRMtx34ToMat4(vrTrackedDevicePoses[vr::k_unTrackedDeviceIndex_Controller1].mDeviceToAbsoluteTracking);
        controllerConnectedRight = true;
    } else {
        controllerConnectedRight = false;
    }
}

extern "C" void vrGetHMDMovementDiff(float coord[3]){
    coord[0] = vecHMDPositionDiff.x;
    coord[1] = vecHMDPositionDiff.y;
    coord[2] = vecHMDPositionDiff.z;
}

extern "C" void vrGetHMDRotationDiff(float coord[3]){
    coord[0] = vecHMDRotationDiff.x;
    coord[1] = vecHMDRotationDiff.y;
    coord[2] = vecHMDRotationDiff.z;
}

extern "C" void vrLogSubmitResult(vr::EVRCompositorError error, u8 eye)
{
    char eyeName[64];
    sprintf_s(eyeName, sizeof(eyeName), eye == 0 ? "left" : "right");
    if (error != vr::VRCompositorError_None)
    {
        // Handle error
        switch (error)
        {
        case vr::VRCompositorError_RequestFailed:
            sysLogPrintf(LOG_ERROR, "%s Eye: Submit failed: RequestFailed", eyeName);
            break;
        case vr::VRCompositorError_InvalidTexture:
            sysLogPrintf(LOG_ERROR, "%s Eye: Submit failed: InvalidTexture", eyeName);
            break;
        case vr::VRCompositorError_IsNotSceneApplication:
            sysLogPrintf(LOG_ERROR, "%s Eye: Submit failed: IsNotSceneApplication", eyeName);
            break;
        // Add other error cases as needed
        default:
            sysLogPrintf(LOG_ERROR, "%s Eye: Submit failed with error: %d", eyeName, (int)error);
            break;
        }
    }
    else
    {
        sysLogPrintf(LOG_NOTE, "%s Eye: successfully submitted texture", eyeName);
    }
}


///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


extern "C" void vrGetCurrentProjectionMtx(float dest[4][4], vr::Hmd_Eye nEye)
{
    Matrix4 matMVP;
    if (nEye == vr::Eye_Left)
    {
        matMVP = mat4VRProjectionLeft * mat4VREyePosLeft;
    }
    else if (nEye == vr::Eye_Right)
    {
        matMVP = mat4VRProjectionRight * mat4VREyePosRight;
    }
    vrMat4ToFloat44(dest, matMVP);
}

extern "C" void vrSetCameraMtx(float matrix[4][4])
{
    vrFloat44ToMat4(mat4Camera, matrix);
}

extern "C" bool vrGetLeftControllerMatrix(float matrix[4][4]) {
    if (!controllerConnectedLeft) {
        return false;
    }
    vrMat4ToFloat44(matrix, mat4ControllerPoseLeft);
    return true;
}

extern "C" bool vrGetRightControllerMatrix(float matrix[4][4]) {
    if (!controllerConnectedRight) {
        return false;
    }
    vrMat4ToFloat44(matrix, mat4ControllerPoseRight);
    return true;
}