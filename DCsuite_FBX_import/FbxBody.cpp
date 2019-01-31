#include <Util/BasicLibrary/BasicLibraries.h>
#include <Body/JointStruct/MotionGenerator.h>
#include <Editor/FbxBody/FbxBody.h>

#include <fstream>
#include <sstream>
#include "../ImportedBody/glslShader_body.h"

using namespace std;

namespace dcsbodyspace {

FbxBody::FbxBody()
: BasicBody()
{
	mNumMesh = 0;
	mNumAnimation= 0;

	mHasMesh = false;
	mHasMaterial = false;
	mHasAnimation = false;
	mHasTexture = false;

	mShader = new GLSLProgram(panel_vertexshader,panel_fragmentshader);
	if(mShader && mShader->getProgId()==0) {
		delete mShader;
		mShader=NULL;
	}
	mShowShadow = false;
}

FbxBody::~FbxBody()
{
	for (int i = 0; i < mNumMesh; i++)
	{
		delete mMeshes[i];
		delete mMeshesNextFrame[i];
		
		mMeshes[i] = NULL;
		mMeshesNextFrame[i] = NULL;
	}
	mMeshes.clear();
	mMeshesNextFrame.clear();

	if(mShader)				delete mShader;					mShader=NULL;
	delete  gSceneContext;
}

void FbxBody::update() {
	return;
}

bool FbxBody::load(const std::wstring &wfilename)
{	
	string filename = StringLibrary::wstring2string(wfilename);
	const __wchar_t * FbxSdkwstring = reinterpret_cast<const __wchar_t *>(wfilename.c_str());
	char * FbxSdkstring;
	fbxsdk::FbxWCToUTF8(FbxSdkwstring,FbxSdkstring);
	bodyname = (filename);
	mWFileName = wfilename;
	mFileName = filename;
	gSceneContext = new FBXLoader(FbxSdkstring);
	if (! gSceneContext->LoadFile())
	{
		std::cout<< "CANNOT load FBX with FBXSDK"<<std::endl;
		return false;
	}
	std::cout<< " load FBX with FBXSDK DONE"<<std::endl;
	gSceneContext->SetCurrentAnimStack(0); //Animation Array Selection.. default = 0
	frame_max = gSceneContext->mAnimTotalFrame;
	mCurrenFrame = 0;

	gSceneContext->Draw( mCurrenFrame , CURRENT_FRAME ); // frame , draw boolean
	
	std::cout<< " Initialize collider"<<std::endl;
	InitializeForCollider();	//just pointing DCmesh
	UpdateForCollider();		//just pointing DCmesh
	std::cout<< " collider update done"<<std::endl;

	return true;
}

void FbxBody::InitializeForCollider()
{
	mNumMesh = gSceneContext->fbxMeshVec.size();
	mMeshes.resize(gSceneContext->fbxMeshVec.size());
	mMeshesNextFrame.resize(gSceneContext->fbxMeshVec.size());

	for (int i = 0; i < gSceneContext->fbxMeshVec.size(); i++)
	{
		const VBOMesh * lMeshCache = static_cast<const VBOMesh *>(gSceneContext->fbxMeshVec[i]->GetUserDataPtr());
		mMeshes[i] = lMeshCache->GetColliderMesh();
		mMeshesNextFrame[i] = lMeshCache->GetColliderMesh_nf();
	}  
}

void FbxBody::UpdateForCollider()
{
	for (int i = 0; i < gSceneContext->fbxMeshVec.size(); i++)
	{
		const VBOMesh * lMeshCache = static_cast<const VBOMesh *>(gSceneContext->fbxMeshVec[i]->GetUserDataPtr());
		mMeshes[i] = lMeshCache->mColliderDCMesh;
		mMeshesNextFrame[i] = lMeshCache->mColliderDCMesh_nf;;
	}
}

void FbxBody::DrawFbxMesh(){
	glEnable(GL_TEXTURE_2D);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	gSceneContext->Draw( mCurrenFrame , CURRENT_FRAME );
	gSceneContext->Draw(mCurrenFrame + 1 , NEXT_FRAME);
	glPopAttrib();
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);	
	UpdateForCollider();
#if 0
	mShader = NULL;
	for (int i = 0; i < mNumMesh; i++)
	{
		glPushMatrix();

		glEnable(GL_TEXTURE_2D);

		if (mShader) {
			mShader->enable();
			mShader->setUniform1i("istex", 1);
		}
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		mMeshesNextFrame[i]->GlDrawTexturedMesh(mShader);

		//colliderMesh[i]->GlDrawSimpleMesh();
		glPopAttrib();
		glPopMatrix();

		if (mShader)
			mShader->disable();

		glDisable(GL_TEXTURE_2D);	
		glPopMatrix();
	}
#endif
}

bool FbxBody::set_frame(int frame){
	 mCurrenFrame = frame;
	
	return true;
}
}