#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include "system.h"
#include "openvr_mingw.hpp"
#include "../include/Matrices.h"
#include "../vrMatrixUtils.h"
#include "CGLRenderModel.h"

std::vector< CGLRenderModel * > m_vecRenderModels;
GLuint m_unRenderModelProgramID;
GLint m_nRenderModelMatrixLocation;
bool showController[2] = { true, true }; // Left and Right hand controllers
std::string modelNames[2] = { "", "" };
CGLRenderModel *renderModels[2] = { nullptr, nullptr }; // Left and Right hand models
Matrix4 mat4Pose[2];

enum EHand
{
	Left = 0,
	Right = 1,
};


//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
	if( unRequiredBufferLen == 0 )
		return "";

	char *pchBuffer = new char[ unRequiredBufferLen ];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
	std::string sResult = pchBuffer;
	delete [] pchBuffer;
	return sResult;
}




GLuint CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader )
{
	GLuint unProgramID = glCreateProgram();

	GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource( nSceneVertexShader, 1, &pchVertexShader, NULL);
	glCompileShader( nSceneVertexShader );

	GLint vShaderCompiled = GL_FALSE;
	glGetShaderiv( nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
	if ( vShaderCompiled != GL_TRUE)
	{
		sysLogPrintf(LOG_ERROR, "%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
		glDeleteProgram( unProgramID );
		glDeleteShader( nSceneVertexShader );
		return 0;
	}
	glAttachShader( unProgramID, nSceneVertexShader);
	glDeleteShader( nSceneVertexShader ); // the program hangs onto this once it's attached

	GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource( nSceneFragmentShader, 1, &pchFragmentShader, NULL);
	glCompileShader( nSceneFragmentShader );

	GLint fShaderCompiled = GL_FALSE;
	glGetShaderiv( nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
	if (fShaderCompiled != GL_TRUE)
	{
		sysLogPrintf(LOG_ERROR, "%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader );
		glDeleteProgram( unProgramID );
		glDeleteShader( nSceneFragmentShader );
		return 0;	
	}

	glAttachShader( unProgramID, nSceneFragmentShader );
	glDeleteShader( nSceneFragmentShader ); // the program hangs onto this once it's attached

	glLinkProgram( unProgramID );

	GLint programSuccess = GL_TRUE;
	glGetProgramiv( unProgramID, GL_LINK_STATUS, &programSuccess);
	if ( programSuccess != GL_TRUE )
	{
		sysLogPrintf(LOG_ERROR, "%s - Error linking program %d!\n", pchShaderName, unProgramID);
		glDeleteProgram( unProgramID );
		return 0;
	}

	glUseProgram( unProgramID );
	glUseProgram( 0 );

	return unProgramID;
}


///load model
//-----------------------------------------------------------------------------
// Purpose: Finds a render model we've already loaded or loads a new one
//-----------------------------------------------------------------------------
CGLRenderModel* FindOrLoadRenderModel( const char *pchRenderModelName )
{
	CGLRenderModel *pRenderModel = NULL;
	for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )
	{
		if( !stricmp( (*i)->GetName().c_str(), pchRenderModelName ) )
		{
			pRenderModel = *i;
			break;
		}
	}

	// load the model if we didn't find one
	if( !pRenderModel )
	{
		vr::RenderModel_t *pModel;
		vr::EVRRenderModelError error;
		while ( 1 )
		{
			error = vr::VRRenderModels()->LoadRenderModel_Async( pchRenderModelName, &pModel );
			if ( error != vr::VRRenderModelError_Loading )
				break;

			sysSleep(1000);
		}

		if ( error != vr::VRRenderModelError_None )
		{
			sysLogPrintf(LOG_ERROR, "Unable to load render model %s - %s", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
			return NULL; // move on to the next tracked device
		}

		vr::RenderModel_TextureMap_t *pTexture;
		while ( 1 )
		{
			error = vr::VRRenderModels()->LoadTexture_Async( pModel->diffuseTextureId, &pTexture );
			if ( error != vr::VRRenderModelError_Loading )
				break;

			sysSleep(1000);
		}

		if ( error != vr::VRRenderModelError_None )
		{
			sysLogPrintf(LOG_ERROR, "Unable to load render texture id:%d for render model %s", pModel->diffuseTextureId, pchRenderModelName );
			vr::VRRenderModels()->FreeRenderModel( pModel );
			return NULL; // move on to the next tracked device
		}

		pRenderModel = new CGLRenderModel( pchRenderModelName );
		if ( !pRenderModel->BInit( *pModel, *pTexture ) )
		{
			sysLogPrintf(LOG_ERROR, "Unable to create GL model from render model %s", pchRenderModelName );
			delete pRenderModel;
			pRenderModel = NULL;
		}
		else
		{
			m_vecRenderModels.push_back( pRenderModel );
		}
		vr::VRRenderModels()->FreeRenderModel( pModel );
		vr::VRRenderModels()->FreeTexture( pTexture );
	}
	return pRenderModel;
}


extern "C" void controllerDebugInit(vr::IVRSystem *pHmd)
{

	m_unRenderModelProgramID = CompileGLShader( 
		"render model",

		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3NormalIn;\n"
		"layout(location = 2) in vec2 v2TexCoordsIn;\n"
		"out vec2 v2TexCoord;\n"
		"void main()\n"
		"{\n"
		"	v2TexCoord = v2TexCoordsIn;\n"
		"	gl_Position = matrix * vec4(position.xyz, 1);\n"
		"}\n",

		//fragment shader
		"#version 410 core\n"
		"uniform sampler2D diffuse;\n"
		"in vec2 v2TexCoord;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture( diffuse, v2TexCoord);\n"
		"}\n"

		);
	m_nRenderModelMatrixLocation = glGetUniformLocation( m_unRenderModelProgramID, "matrix" );
	if( m_nRenderModelMatrixLocation == -1 )
	{
		sysLogPrintf(LOG_ERROR, "Unable to find matrix uniform in render model shader");
		return;
	}


	///model
	for ( EHand eHand = Left; eHand <= Right; ((int&)eHand)++ )
	{
		vr::InputPoseActionData_t poseData;
		vr::VRActionHandle_t actionHandle;
		if ( vr::VRInput()->GetPoseActionDataForNextFrame( actionHandle, vr::TrackingUniverseStanding, &poseData, sizeof( poseData ), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None
			&& poseData.bActive && poseData.pose.bPoseIsValid )
		{
			mat4Pose[eHand] = vrSteamVRMtx34ToMat4( poseData.pose.mDeviceToAbsoluteTracking );

			vr::InputOriginInfo_t originInfo;
			if ( vr::VRInput()->GetOriginTrackedDeviceInfo( poseData.activeOrigin, &originInfo, sizeof( originInfo ) ) == vr::VRInputError_None 
				&& originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid )
			{
				std::string sRenderModelName = GetTrackedDeviceString( pHmd, originInfo.trackedDeviceIndex, vr::Prop_RenderModelName_String );
				// if modelname has changed, change the model...
				if ( sRenderModelName != modelNames[eHand] )
				{
					renderModels[eHand] = FindOrLoadRenderModel( sRenderModelName.c_str() );
					modelNames[eHand] = sRenderModelName;
				}
			}
		}else{
			showController[eHand] = false;
		}
	}


}


extern "C" void controllerDebugCleanup() {
	for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )
	{
		delete (*i);
	}
	m_vecRenderModels.clear();
}


extern "C" void controllerDebugRender(float projectionMatrix[4][4]) {
	Matrix4 pMat = Matrix4(projectionMatrix);
	/// render
	// ----- Render Model rendering -----
	glUseProgram( m_unRenderModelProgramID );

	for ( EHand eHand = Left; eHand <= Right; ((int&)eHand)++ )
	{
		if ( !showController[eHand] || !renderModels[eHand] )
			continue;

			
		const Matrix4 & matDeviceToTracking = mat4Pose[eHand];
		Matrix4 matMVP = pMat * matDeviceToTracking;
		glUniformMatrix4fv( m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get() );

		renderModels[eHand]->Draw();
	}

}

