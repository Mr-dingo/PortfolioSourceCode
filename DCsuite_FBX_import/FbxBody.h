#ifndef _dcsbodyspace_FBXSTRUCT_H_
#define _dcsbodyspace_FBXSTRUCT_H_

#include <Editor/BasicBody/BasicBody.h>
#include <Editor/FbxBody/FbxLoader.h>
#include <DCBody_DLL.h>
///////////////////// Implemented by Luis Lee <Begin>/////////////////////////
namespace dcsbodyspace {

using std::string;
using std::vector;

//class DCBODYDLL FbxBody : public BasicBody 

class DCBODYDLL FbxBody : public MotionController, public BasicBody {
	
public:
	FbxBody();
	~FbxBody();
	void DrawFbxMesh();
	void init();
	bool load(const std::wstring&);
	void update();
	bool setFrame(int i );
	wstring GetWFileName(){return mWFileName;}

	//mesh
	vector<MeshStruct *>			mMeshes;
	vector<MeshStruct *>			mMeshesNextFrame;
	int mNumMesh;
	int mNumAnimation;
	//mShader
	GLSLProgram *	mShader;
	GLuint			mShadowmapID;
	bool			mShowShadow;
	FBXLoader * gSceneContext;
	// vert norm index data init
	void		InitializeForCollider();
	// normal and vertex position data
	void		UpdateForCollider();		
	
private:
	bool mHasTexture;
	bool mHasAnimation;
	bool mHasMaterial;
	bool mHasMesh;
	int mCurrenFrame;
	wstring mWFileName;
	string mFileName;
};

}//namespace dcsbodyspace
#endif
///////////////////// Implemented by Luis Lee <End>/////////////////////////
