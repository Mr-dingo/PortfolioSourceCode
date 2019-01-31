#pragma once

#pragma warning(disable: 4819)


#ifndef _dcsbodyspace_FBXLOADER_H_
#define _dcsbodyspace_FBXLOADER_H_

#include <Util/OBJStruct/TextureMap_DCBody.h>
#include <Editor/BasicBody/BasicBody.h>
#include <DCBody_DLL.h>
#include <fbxsdk.h>


namespace dcsbodyspace {

using std::string;
using std::vector;

class DCBODYDLL FBXLoader
{
public:
	const char * mFileName;
	mutable FbxString WindowMessage;

	mutable FbxTime mFrameTime, mStart, mStop, mCurrentTime;
	mutable FbxTime mCache_Start, mCache_Stop;
	int mAnimTotalFrame;

	FbxManager * SdkManager;
	FbxScene * Scene;
	FbxImporter * Importer;
	FbxAnimLayer * CurrentAnimLayer;
	FbxNode * SelectedNode;
	///////////////////// Implemented by Luis Lee <Begin>/////////////////////////
	int mPoseIndex;
	bool mSupportVBO;

	FbxArray<FbxString*> mAnimStackNameArray;

	vector< FbxMesh * > fbxMeshVec;
	vector<MeshStruct *> colliderMesh;
	vector<MeshStruct *> colliderMesh_nf;
	vector<wstring>		texturePathList;

	FBXLoader(const char * pFileName);
	~FBXLoader(void);
	void SetSelectedNode(FbxNode * pSelectedNode);
	void InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene);
	bool LoadTextureFromFile(const FbxString & pFilePath, unsigned int & pTextureObject);
	void LoadCacheRecursive(FbxNode * pNode, FbxAnimLayer * pAnimLayer, bool pSupportVBO);
	void UnloadCacheRecursive(FbxNode * pNode); 
	void UnloadCacheRecursive(FbxScene * pScene);
	bool LoadFile();
	void FillPoseArray(FbxScene* pScene, FbxArray<FbxPose*>& pPoseArray);
	void PreparePointCacheData(FbxScene* pScene, FbxTime &pCache_Start, FbxTime &pCache_Stop);
	bool Draw(int frame,const int iOption); //iOption : current / next frame ���
	void DrawNodeRecursive(FbxNode* pNode, FbxTime& pTime, FbxAnimLayer* pAnimLayer,
		FbxAMatrix& pParentGlobalPosition, FbxPose* pPose ,const int iOption);
	void LoadCacheRecursive(FbxScene * pScene, FbxAnimLayer * pAnimLayer, const char * pFbxFileName, bool pSupportVBO);
	bool SetCurrentAnimStack(int pIndex);
	void DrawNode(FbxNode* pNode, FbxTime& pTime,FbxAnimLayer* pAnimLayer,FbxAMatrix& pParentGlobalPosition,
		FbxAMatrix& pGlobalPosition,FbxPose* pPose ,const int iOption);
	void DrawSkeleton(FbxNode* pNode, FbxAMatrix& pParentGlobalPosition, FbxAMatrix& pGlobalPosition);
	void DrawMesh(FbxNode* pNode, FbxTime& pTime, FbxAnimLayer* pAnimLayer,
		FbxAMatrix& pGlobalPosition, FbxPose* pPose ,const int iOption);
	void UpdateMeshVert(FbxNode* pNode, FbxTime& pTime, FbxAnimLayer* pAnimLayer,
		FbxAMatrix& pGlobalPosition, FbxPose* pPose , FbxVector4* target_vert_array 
		, const int iOption);
	///////////////////// Implemented by Luis Lee <End>/////////////////////////


	void OnTimerClick() const;
	FbxAMatrix GetGlobalPosition(FbxNode* pNode, const FbxTime& pTime, FbxPose* pPose = NULL, FbxAMatrix* pParentGlobalPosition = NULL);
	FbxAMatrix GetPoseMatrix(FbxPose* pPose, int pNodeIndex);
	FbxAMatrix GetGeometry(FbxNode* pNode);
	void GlDrawLimbNode(FbxAMatrix& pGlobalBasePosition, FbxAMatrix& pGlobalEndPosition);
	void ReadVertexCacheData(FbxMesh* pMesh, FbxTime& pTime, FbxVector4* pVertexArray);
	void ComputeShapeDeformation(FbxMesh* pMesh, FbxTime& pTime, FbxAnimLayer * pAnimLayer, FbxVector4* pVertexArray);
	void ComputeSkinDeformation(FbxAMatrix& pGlobalPosition, FbxMesh* pMesh,  FbxTime& pTime, FbxVector4* pVertexArray, FbxPose* pPose);
	void ComputeLinearDeformation(FbxAMatrix& pGlobalPosition, FbxMesh* pMesh, FbxTime& pTime, FbxVector4* pVertexArray, FbxPose* pPose);
	void ComputeClusterDeformation(FbxAMatrix& pGlobalPosition, 
		FbxMesh* pMesh,FbxCluster* pCluster, FbxAMatrix& pVertexTransformMatrix,FbxTime pTime, FbxPose* pPose);
	void MatrixScale(FbxAMatrix& pMatrix, double pValue);
	void MatrixAddToDiagonal(FbxAMatrix& pMatrix, double pValue);
	void MatrixAdd(FbxAMatrix& pDstMatrix, FbxAMatrix& pSrcMatrix);
	void ComputeDualQuaternionDeformation(FbxAMatrix& pGlobalPosition, 
		FbxMesh* pMesh, FbxTime& pTime,FbxVector4* pVertexArray,FbxPose* pPose);
	void DestroySdkObjects(FbxManager* pManager, bool pExitStatus);
	const FbxTime GetFrameTime() const { return mFrameTime; }

};


const int TRIANGLE_VERTEX_COUNT = 3;

// Four floats for every position.
const int VERTEX_STRIDE = 4;
// Three floats for every normal.
const int NORMAL_STRIDE = 3;
// Two floats for every UV.
const int UV_STRIDE = 2;

const int CURRENT_FRAME = 0;
const int NEXT_FRAME = 1;
// Save mesh vertices, normals, UVs and indices in GPU with OpenGL Vertex Buffer Objects
class DCBODYDLL VBOMesh
{
public:
	VBOMesh();
	~VBOMesh();
	///////////////////// Implemented by Luis Lee <Begin>/////////////////////////
	void SetColliderIndex(unsigned int * mIndices ) const; 
	void SetColliderPosNorm(float * mVertices , const int iOption)  const;
	bool Initialize(FbxMesh * pMesh);
	void UpdateVertexPosition(const FbxMesh * pMesh, FbxAMatrix& pGlobalPosition, const FbxVector4 * pVertices , const int iOption) const;
	// Bind buffers, set vertex arrays, turn on lighting and texture.
	void BeginDraw() const;
	// Draw all the faces with specific material with given shading mode.
	void Draw(int pMaterialIndex) const;
	// Unbind buffers, reset vertex arrays, turn off lighting and texture.
	void EndDraw() const;
	// Get the count of material groups
	int GetSubMeshCount() const { return mSubMeshes.GetCount(); }
	void GetColliderMesh() const { return mColliderDCMesh;}
	void GetColliderMesh_nf() const { return mColliderDCMesh_nf;}
	
	///////////////////// Implemented by Luis Lee <End>/////////////////////////
private:
	FbxVector4 * mNextVertexArray;
	MeshStruct * mColliderDCMesh;
	MeshStruct * mColliderDCMesh_nf;
	enum
	{
		VERTEX_VBO,
		NORMAL_VBO,
		UV_VBO,
		INDEX_VBO,
		VBO_COUNT,
	};
	// For every material, record the offsets in every VBO and triangle counts
	struct SubMesh
	{
		SubMesh() : IndexOffset(0), TriangleCount(0) {}

		int IndexOffset;
		int TriangleCount;
	};
	GLuint mVBONames[VBO_COUNT];
	FbxArray<SubMesh*> mSubMeshes;
	bool mHasNormal;
	bool mHasUV;
	bool mAllByControlPoint; // Save data in VBO by control point or by polygon vertex.
	int mNumVert;	//For allbyControlpoint
	int mNumPolyIndex;	//For allbyControlpoint

};
// Cache for FBX material
class DCBODYDLL MaterialCache
{
public:
	MaterialCache();
	~MaterialCache();
	bool Initialize(const FbxSurfaceMaterial * pMaterial);
	void SetCurrentMaterial() const;
	bool HasTexture() const { return mDiffuse.mTextureName != 0; }
	static void SetDefaultMaterial();
	FbxDouble3 GetMaterialProperty(const FbxSurfaceMaterial * pMaterial,
		const char * pPropertyName,
		const char * pFactorPropertyName,
		GLuint & pTextureName)
	{
		FbxDouble3 lResult(0, 0, 0);
		const FbxProperty lProperty = pMaterial->FindProperty(pPropertyName);
		const FbxProperty lFactorProperty = pMaterial->FindProperty(pFactorPropertyName);
		if (lProperty.IsValid() && lFactorProperty.IsValid())
		{
			lResult = lProperty.Get<FbxDouble3>();
			double lFactor = lFactorProperty.Get<FbxDouble>();
			if (lFactor != 1)
			{
				lResult[0] *= lFactor;
				lResult[1] *= lFactor;
				lResult[2] *= lFactor;
			}
		}

		if (lProperty.IsValid())
		{
			const int lTextureCount = lProperty.GetSrcObjectCount<FbxFileTexture>();
			if (lTextureCount)
			{
				const FbxFileTexture* lTexture = lProperty.GetSrcObject<FbxFileTexture>();
				if (lTexture && lTexture->GetUserDataPtr())
				{
					pTextureName = *(static_cast<GLuint *>(lTexture->GetUserDataPtr()));
				}
			}
		}

		return lResult;
	}
private:
	struct ColorChannel
	{
		ColorChannel() : mTextureName(0)
		{
			mColor[0] = 0.0f;
			mColor[1] = 0.0f;
			mColor[2] = 0.0f;
			mColor[3] = 1.0f;
		}

		GLuint mTextureName;
		GLfloat mColor[4];
	};
	ColorChannel mEmissive;
	ColorChannel mAmbient;
	ColorChannel mDiffuse;
	ColorChannel mSpecular;
	GLfloat mShinness;
};


// Property cache, value and animation curve.
struct DCBODYDLL PropertyChannel
{
	PropertyChannel() : mAnimCurve(NULL), mValue(0.0f) {}
	// Query the channel value at specific time.
	GLfloat Get(const FbxTime & pTime) const
	{
		if (mAnimCurve)
		{
			return mAnimCurve->Evaluate(pTime);
		}
		else
		{
			return mValue;
		}
	}

	FbxAnimCurve * mAnimCurve;
	GLfloat mValue;
};

DCBODYDLL const wchar_t * FbxStringtoDCWC(fbxsdk::FbxString inStr);


}
#endif