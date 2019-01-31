#include <Editor/GenerationRule/GenerationRule.h>
#include <Editor/SetBody/SetBody.h>
#include <math.h>


using namespace std;
using namespace dcsbodyspace;

SetBodyGenRules::SetBodyGenRules()
	:BodyGenRules()
{
	mFatness = 0.0f;
	mHeight = 0.0f;
}

SetBodyGenRules::~SetBodyGenRules()
{
}
void SetBodyGenRules::initialize()
{
}
void SetBodyGenRules::load(bool isMale)
{
}

void SetBodyGenRules::update()
{
	int fatnessIndex = floor(mFatness);
	int heightIndex = floor(mHeight);

	fatnessIndex = clamp(fatnessIndex,0,1);
	heightIndex = clamp(heightIndex,0,1);
	int nearFatIndex = clamp((int)(mFatness + 0.5),0,2);
	int nearHeightIndex = clamp((int)(mHeight + 0.5),0,2);

	double fatnessWeight = mFatness - fatnessIndex;
	double heightWeight = mHeight - heightIndex;

	SetBody* _body = (SetBody*)body;


	ImportedBody* b00 = _body->setBodies[3*heightIndex+fatnessIndex];
	ImportedBody* b10 = _body->setBodies[3*heightIndex+fatnessIndex+1];
	ImportedBody* b01 = _body->setBodies[3*(heightIndex+1)+fatnessIndex];
	ImportedBody* b11 = _body->setBodies[3*(heightIndex+1)+fatnessIndex+1];

	ImportedBody* b_temp = _body->setBodies[3*(nearHeightIndex)+nearFatIndex];
	_body->getBody()->riggingObject->setSkeleton(*b_temp->riggingObject->getSkeleton());
	_body->getBody()->riggingObject->setSkeletonBase(*b_temp->riggingObject->getSkeletonBase());

	
	for(int i=0; i < _body->getBody()->riggingObject->getSkeletonBase()->m_pBoneList.Size(); i++) {
		string boneName = _body->getBody()->riggingObject->getSkeletonBase()->m_pBoneList[i].name;
		Bone* bone=_body->getBody()->riggingObject->getSkeleton()->getBone(boneName);
		Bone* boneBase=_body->getBody()->riggingObject->getSkeletonBase()->getBone(boneName);
		Bone* bone_0_0=b00->riggingObject->getSkeletonBase()->getBone(boneName);
		Bone* bone_1_0=b10->riggingObject->getSkeletonBase()->getBone(boneName);
		Bone* bone_0_1=b01->riggingObject->getSkeletonBase()->getBone(boneName);
		Bone* bone_1_1=b11->riggingObject->getSkeletonBase()->getBone(boneName);
		Bone* boneHead=_body->getBody()->head->riggingObject->getSkeleton()->getBone(boneName);
		Bone* boneHeadBase=_body->getBody()->head->riggingObject->getSkeletonBase()->getBone(boneName);

		float length = (1-heightWeight)*(fatnessWeight*bone_1_0->length + (1 - fatnessWeight)*bone_0_0->length) + heightWeight*(fatnessWeight*bone_1_1->length + (1 - fatnessWeight)*bone_0_1->length);
		bone->length = length;
		bone->length_ori = length;
		boneBase->length = length;
		boneBase->length_ori = length;
		boneHead->length = length;
		boneHead->length_ori = length;
		boneHeadBase->length = length;
		boneHeadBase->length_ori = length;

	}
	for(int i=0; i < _body->getBody()->riggingObject->getSkeletonBase()->m_pBoneList.Size(); i++) {
		string boneName = _body->getBody()->riggingObject->getSkeletonBase()->m_pBoneList[i].name;
		Vector3f t00 = b00->riggingObject->getSkeletonBase()->getBone(boneName)->t;
		Vector3f t01 = b01->riggingObject->getSkeletonBase()->getBone(boneName)->t;
		Vector3f t10 = b10->riggingObject->getSkeletonBase()->getBone(boneName)->t;
		Vector3f t11 = b11->riggingObject->getSkeletonBase()->getBone(boneName)->t;

		Vector3f r00 = b00->riggingObject->getSkeletonBase()->getBone(boneName)->r;
		Vector3f r01 = b01->riggingObject->getSkeletonBase()->getBone(boneName)->r;
		Vector3f r10 = b10->riggingObject->getSkeletonBase()->getBone(boneName)->r;
		Vector3f r11 = b11->riggingObject->getSkeletonBase()->getBone(boneName)->r;

		float tl00 = b00->riggingObject->getSkeletonBase()->getBone(boneName)->tl;
		float tl01 = b01->riggingObject->getSkeletonBase()->getBone(boneName)->tl;
		float tl10 = b10->riggingObject->getSkeletonBase()->getBone(boneName)->tl;
		float tl11 = b11->riggingObject->getSkeletonBase()->getBone(boneName)->tl;

		Vector3f dir00 = b00->riggingObject->getSkeletonBase()->getBone(boneName)->dirLocal;
		Vector3f dir01 = b01->riggingObject->getSkeletonBase()->getBone(boneName)->dirLocal;
		Vector3f dir10 = b10->riggingObject->getSkeletonBase()->getBone(boneName)->dirLocal;
		Vector3f dir11 = b11->riggingObject->getSkeletonBase()->getBone(boneName)->dirLocal;

		Quaternion<double> q00 = Quaternion<double>(b00->riggingObject->getSkeletonBase()->getBone(boneName)->rotation_Parent_Current);
		Quaternion<double> q01 = Quaternion<double>(b01->riggingObject->getSkeletonBase()->getBone(boneName)->rotation_Parent_Current);
		Quaternion<double> q10 = Quaternion<double>(b10->riggingObject->getSkeletonBase()->getBone(boneName)->rotation_Parent_Current);
		Quaternion<double> q11 = Quaternion<double>(b11->riggingObject->getSkeletonBase()->getBone(boneName)->rotation_Parent_Current);

		Vector3f temp = (1-(float)heightWeight)*((float)fatnessWeight*t10 + (1 - (float)fatnessWeight)*t00) + (float)heightWeight*((float)fatnessWeight*t11 + (1 - (float)fatnessWeight)*t01);
		Vector3f rTemp = (1-(float)heightWeight)*((float)fatnessWeight*r10 + (1 - (float)fatnessWeight)*r00) + (float)heightWeight*((float)fatnessWeight*r11 + (1 - (float)fatnessWeight)*r01);
		float tlTemp = (1-(float)heightWeight)*((float)fatnessWeight*tl10 + (1 - (float)fatnessWeight)*tl00) + (float)heightWeight*((float)fatnessWeight*tl11 + (1 - (float)fatnessWeight)*tl01);
		Vector3f dirTemp = (1-(float)heightWeight)*((float)fatnessWeight*dir10 + (1 - (float)fatnessWeight)*dir00) + (float)heightWeight*((float)fatnessWeight*dir11 + (1 - (float)fatnessWeight)*dir01);
		dirTemp.normalize();
		Matrix4<double> matTemp = slerp(slerp(q00,q10,fatnessWeight),slerp(q01,q11,fatnessWeight),heightWeight).getMatrix4();

		_body->getBody()->riggingObject->getSkeletonBase()->getBone(boneName)->t= temp;
		_body->getBody()->head->riggingObject->getSkeletonBase()->getBone(boneName)->t= temp;
		_body->getBody()->riggingObject->getSkeletonBase()->getBone(boneName)->r= rTemp;
		_body->getBody()->head->riggingObject->getSkeletonBase()->getBone(boneName)->r= rTemp;
		_body->getBody()->riggingObject->getSkeletonBase()->getBone(boneName)->tl= tlTemp;
		_body->getBody()->head->riggingObject->getSkeletonBase()->getBone(boneName)->tl= tlTemp;
		_body->getBody()->riggingObject->getSkeletonBase()->getBone(boneName)->dirLocal= dirTemp;
		_body->getBody()->head->riggingObject->getSkeletonBase()->getBone(boneName)->dirLocal= dirTemp;
		_body->getBody()->riggingObject->getSkeletonBase()->getBone(boneName)->rotation_Parent_Current= matTemp;
		_body->getBody()->head->riggingObject->getSkeletonBase()->getBone(boneName)->rotation_Parent_Current= matTemp;

		_body->getBody()->riggingObject->getSkeleton()->getBone(boneName)->t= temp;
		_body->getBody()->head->riggingObject->getSkeleton()->getBone(boneName)->t= temp;
		_body->getBody()->riggingObject->getSkeleton()->getBone(boneName)->r= rTemp;
		_body->getBody()->head->riggingObject->getSkeleton()->getBone(boneName)->r= rTemp;
		_body->getBody()->riggingObject->getSkeleton()->getBone(boneName)->tl= tlTemp;
		_body->getBody()->head->riggingObject->getSkeleton()->getBone(boneName)->tl= tlTemp;
		_body->getBody()->riggingObject->getSkeleton()->getBone(boneName)->dirLocal= dirTemp;
		_body->getBody()->head->riggingObject->getSkeleton()->getBone(boneName)->dirLocal= dirTemp;
		_body->getBody()->riggingObject->getSkeleton()->getBone(boneName)->rotation_Parent_Current= matTemp;
		_body->getBody()->head->riggingObject->getSkeleton()->getBone(boneName)->rotation_Parent_Current= matTemp;
		
	}

	_body->getBody()->riggingObject->getSkeleton()->computeTransformEachBone();
	_body->getBody()->riggingObject->getSkeletonBase()->computeTransformEachBone();
	_body->getBody()->head->riggingObject->getSkeleton()->computeTransformEachBone();
	_body->getBody()->head->riggingObject->getSkeletonBase()->computeTransformEachBone();



	interpolationFatness(_body->getBody()->riggingObject,b00->riggingObject,b01->riggingObject,b10->riggingObject,b11->riggingObject);
	interpolationFatness(_body->getBody()->head->riggingObject,b00->head->riggingObject,b01->head->riggingObject,b10->head->riggingObject,b11->head->riggingObject);
	interpolationFatness(_body->getBody()->Body_LFoot->riggingObject,b00->Body_LFoot->riggingObject,b01->Body_LFoot->riggingObject,b10->Body_LFoot->riggingObject,b11->Body_LFoot->riggingObject);
	interpolationFatness(_body->getBody()->Body_RFoot->riggingObject,b00->Body_RFoot->riggingObject,b01->Body_RFoot->riggingObject,b10->Body_RFoot->riggingObject,b11->Body_RFoot->riggingObject);
	interpolationFatness(_body->getBody()->Body_LHand->riggingObject,b00->Body_LHand->riggingObject,b01->Body_LHand->riggingObject,b10->Body_LHand->riggingObject,b11->Body_LHand->riggingObject);
	interpolationFatness(_body->getBody()->Body_RHand->riggingObject,b00->Body_RHand->riggingObject,b01->Body_RHand->riggingObject,b10->Body_RHand->riggingObject,b11->Body_RHand->riggingObject);
	interpolationFatness(_body->getBody()->Body_LShoes->riggingObject,b00->Body_LShoes->riggingObject,b01->Body_LShoes->riggingObject,b10->Body_LShoes->riggingObject,b11->Body_LShoes->riggingObject);
	interpolationFatness(_body->getBody()->Body_RShoes->riggingObject,b00->Body_RShoes->riggingObject,b01->Body_RShoes->riggingObject,b10->Body_RShoes->riggingObject,b11->Body_RShoes->riggingObject);

	interpolationFatness(_body->getBody()->Body_LFoot_Cap->riggingObject,b00->Body_LFoot_Cap->riggingObject,b01->Body_LFoot_Cap->riggingObject,b10->Body_LFoot_Cap->riggingObject,b11->Body_LFoot_Cap->riggingObject);
	interpolationFatness(_body->getBody()->Body_RFoot_Cap->riggingObject,b00->Body_RFoot_Cap->riggingObject,b01->Body_RFoot_Cap->riggingObject,b10->Body_RFoot_Cap->riggingObject,b11->Body_RFoot_Cap->riggingObject);
	interpolationFatness(_body->getBody()->Body_LHand_Cap->riggingObject,b00->Body_LHand_Cap->riggingObject,b01->Body_LHand_Cap->riggingObject,b10->Body_LHand_Cap->riggingObject,b11->Body_LHand_Cap->riggingObject);
	interpolationFatness(_body->getBody()->Body_RHand_Cap->riggingObject,b00->Body_RHand_Cap->riggingObject,b01->Body_RHand_Cap->riggingObject,b10->Body_RHand_Cap->riggingObject,b11->Body_RHand_Cap->riggingObject);

	interpolationFatness(_body->getBody()->head->body_other_nodes->other_nodes[0]->riggingObject,b00->head->body_other_nodes->other_nodes[0]->riggingObject,b01->head->body_other_nodes->other_nodes[0]->riggingObject,b10->head->body_other_nodes->other_nodes[0]->riggingObject,b11->head->body_other_nodes->other_nodes[0]->riggingObject);

	for(int i=0; i<body->getBody()->head->body_hair->body_accessories.size(); i++) {
		vector<Vector3<float> > a00 = b00->head->body_hair->body_accessories[i]->get_OBJStruct().posVerts;
		vector<Vector3<float> > a01 = b01->head->body_hair->body_accessories[i]->get_OBJStruct().posVerts;
		vector<Vector3<float> > a10 = b10->head->body_hair->body_accessories[i]->get_OBJStruct().posVerts;
		vector<Vector3<float> > a11 = b11->head->body_hair->body_accessories[i]->get_OBJStruct().posVerts;
		for(int j=0; j<a00.size(); j++) {
			Vector3<float> temp = (1-(float)heightWeight)*((float)fatnessWeight*a10[j] + (1 - (float)fatnessWeight)*a00[j]) + (float)heightWeight*((float)fatnessWeight*a11[j] + (1 - (float)fatnessWeight)*a01[j]);
			body->getBody()->head->body_hair->body_accessories[i]->obj.posVerts[j] = temp;
		}
	}
	for(int i=0; i<body->getBody()->head->body_accessories->body_accessories.size(); i++) {
		vector<Vector3<float> > a00 = b00->head->body_accessories->body_accessories[i]->get_OBJStruct().posVerts;
		vector<Vector3<float> > a01 = b01->head->body_accessories->body_accessories[i]->get_OBJStruct().posVerts;
		vector<Vector3<float> > a10 = b10->head->body_accessories->body_accessories[i]->get_OBJStruct().posVerts;
		vector<Vector3<float> > a11 = b11->head->body_accessories->body_accessories[i]->get_OBJStruct().posVerts;
		for(int j=0; j<a00.size(); j++) {
			Vector3<float> temp = (1-(float)heightWeight)*((float)fatnessWeight*a10[j] + (1 - (float)fatnessWeight)*a00[j]) + (float)heightWeight*((float)fatnessWeight*a11[j] + (1 - (float)fatnessWeight)*a01[j]);
			body->getBody()->head->body_accessories->body_accessories[i]->obj.posVerts[j] = temp;
		}
	}
}

void SetBodyGenRules::interpolationFatness(RiggingObject_Base* rigBase,RiggingObject_Base* rig_0_0,RiggingObject_Base* rig_0_1,RiggingObject_Base* rig_1_0,RiggingObject_Base* rig_1_1) {

	int fatnessIndex = floor(mFatness);
	int heightIndex = floor(mHeight);

	fatnessIndex = clamp(fatnessIndex,0,1);
	heightIndex = clamp(heightIndex,0,1);

	double fatnessWeight = mFatness - fatnessIndex;
	double heightWeight = mHeight - heightIndex;

	ArrayN<Vector3r,0> vBody = rigBase->get_posVert_base();
	ArrayN<std::vector<real>> wBody = rigBase->get_weights();
	ArrayN<std::vector<real>> newWeight;
	ArrayN<std::vector<unsigned int>> vidxBody = rigBase->get_vertIdx();
	ArrayN<Vector3r,0> v00=rig_0_0->get_posVert_base();
	ArrayN<std::vector<real>> w00=rig_0_0->get_weights();
	ArrayN<std::vector<unsigned int>> vidx00 = rig_0_0->get_vertIdx();
	ArrayN<Vector3r,0> v10=rig_1_0->get_posVert_base();
	ArrayN<std::vector<real>> w10=rig_1_0->get_weights();
	ArrayN<std::vector<unsigned int>> vidx10 = rig_1_0->get_vertIdx();
	ArrayN<Vector3r,0> v01=rig_0_1->get_posVert_base();
	ArrayN<std::vector<real>> w01=rig_0_1->get_weights();
	ArrayN<std::vector<unsigned int>> vidx01 = rig_0_1->get_vertIdx();
	ArrayN<Vector3r,0> v11=rig_1_1->get_posVert_base();
	ArrayN<std::vector<real>> w11=rig_1_1->get_weights();
	ArrayN<std::vector<unsigned int>> vidx11 = rig_1_1->get_vertIdx();

	
	for(int i=0; i<vidxBody.Size(); i++) {
		vidxBody[i].clear();
		for(int j=0;j<vBody.Size(); j++) {
			vidxBody[i].push_back(j);
		}
	}
	
	for(int i=0; i<wBody.Size(); i++) {
		wBody[i].resize(vBody.Size());
	}
	double heightWeightClamp = heightWeight;
	double fatnessWeightClamp = fatnessWeight;
	for(int i=0; i < rigBase->getSkeleton()->m_pBoneList.Size(); i++) {
		for(int j=0; j<wBody[i].size(); j++) {
			wBody[i][j] = 0;
		}
		string boneName = rigBase->getSkeleton()->m_pBoneList[i].name;
		Bone* bone_0_0=rig_0_0->getSkeletonBase()->getBone(boneName);
		Bone* bone_1_0=rig_1_0->getSkeletonBase()->getBone(boneName);
		Bone* bone_0_1=rig_0_1->getSkeletonBase()->getBone(boneName);
		Bone* bone_1_1=rig_1_1->getSkeletonBase()->getBone(boneName);
		vector<unsigned int> vidx=rig_0_0->get_vertIdx()[bone_0_0->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = (1-heightWeightClamp)*(1 - fatnessWeightClamp)*w00[bone_0_0->idx][j];
			wBody[i][vidx[j]] += weight;
		}
		vidx=rig_0_1->get_vertIdx()[bone_0_1->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = heightWeightClamp*(1 - fatnessWeightClamp)*w01[bone_0_1->idx][j];
			wBody[i][vidx[j]] += weight;
		}
		vidx=rig_1_0->get_vertIdx()[bone_1_0->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = (1-heightWeightClamp)*fatnessWeightClamp*w10[bone_1_0->idx][j];
			wBody[i][vidx[j]] += weight;
		}
		vidx=rig_1_1->get_vertIdx()[bone_1_1->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = heightWeightClamp*fatnessWeightClamp*w11[bone_1_1->idx][j];
			wBody[i][vidx[j]] += weight;
		}
	}

	newWeight.Resize(wBody.Size());
	for(int i=0; i < rigBase->getSkeleton()->m_pBoneList.Size(); i++) {
		vidxBody[i].clear();
		for(int j=0; j<wBody[i].size(); j++) {
			if(wBody[i][j]>1e-6) {
				vidxBody[i].push_back(j);
				newWeight[i].push_back(wBody[i][j]);
			}
		}
	}

	ArrayNr weightSum(vBody.Size());
	weightSum.MemsetZero();
	for(int i=0;i<rigBase->getSkeleton()->m_pBoneList.Size();i++) {
		for(size_t j=0;j<vidxBody[i].size();++j) {
			weightSum[vidxBody[i][j]] += newWeight[i][j];
		}
	}

	for(int i=0;i<rigBase->getSkeleton()->m_pBoneList.Size();i++) {
		for(size_t j=0;j<vidxBody[i].size();++j) 
			newWeight[i][j] /= weightSum[vidxBody[i][j]];
	}

	rigBase->set_vertIdx(vidxBody);
	rigBase->set_weights(newWeight);
	wBody = rigBase->get_weights();
	for(int i=0; i<vBody.Size(); i++)
		vBody[i].set(0,0,0);

	for(int i=0; i < rigBase->getSkeleton()->m_pBoneList.Size(); i++) {
		string boneName = rigBase->getSkeleton()->m_pBoneList[i].name;
		Bone* bone=rigBase->getSkeletonBase()->getBone(boneName);
		Bone* bone_0_0=rig_0_0->getSkeletonBase()->getBone(boneName);
		Bone* bone_1_0=rig_1_0->getSkeletonBase()->getBone(boneName);
		Bone* bone_0_1=rig_0_1->getSkeletonBase()->getBone(boneName);
		Bone* bone_1_1=rig_1_1->getSkeletonBase()->getBone(boneName);

		vector<unsigned int> vidx=rigBase->get_vertIdx()[bone->idx];
		for(int j=0;j<(int)vidx.size();j++){
			Vector3r local_0_0,local_1_0,local_0_1,local_1_1;
			Vector3r vertex00=v00[vidx[j]]-bone_0_0->origin;
			Vector3r vertex10=v10[vidx[j]]-bone_1_0->origin;
			Vector3r vertex01=v01[vidx[j]]-bone_0_1->origin;
			Vector3r vertex11=v11[vidx[j]]-bone_1_1->origin;

			local_0_0=bone_0_0->transform_Current.getRotationMatrix().getInverse()*vertex00;
			local_1_0=bone_1_0->transform_Current.getRotationMatrix().getInverse()*vertex10;
			local_0_1=bone_0_1->transform_Current.getRotationMatrix().getInverse()*vertex01;
			local_1_1=bone_1_1->transform_Current.getRotationMatrix().getInverse()*vertex11;

			Vector3r temp = (1-heightWeight)*(fatnessWeight*local_1_0 + (1 - fatnessWeight)*local_0_0) + heightWeight*(fatnessWeight*local_1_1 + (1 - fatnessWeight)*local_0_1);
			Vector3r bone_temp = (1-heightWeight)*(fatnessWeight*bone_1_0->origin + (1 - fatnessWeight)*bone_0_0->origin) + heightWeight*(fatnessWeight*bone_1_1->origin + (1 - fatnessWeight)*bone_0_1->origin);
			
			temp = bone->transform_Current.getRotationMatrix()*temp;
			temp = temp+bone_temp;
			vBody[vidx[j]]+=wBody[bone->idx][j]*temp;
		}
	}
	rigBase->setposvert(vBody);

}

void SetBodyGenRules::interpolationFatnessForHands(RiggingObject_Base* rigBase,RiggingObject_Base* rig_0_0,RiggingObject_Base* rig_0_1,RiggingObject_Base* rig_1_0,RiggingObject_Base* rig_1_1) {

	int fatnessIndex = floor(mFatness);
	int heightIndex = floor(mHeight);

	fatnessIndex = clamp(fatnessIndex,0,1);
	heightIndex = clamp(heightIndex,0,1);

	double fatnessWeight = mFatness - fatnessIndex;
	double heightWeight = mHeight - heightIndex;

	ArrayN<Vector3r,0> vBody = rigBase->get_posVert_base();
	ArrayN<std::vector<real>> wBody = rigBase->get_weights();
	ArrayN<std::vector<real>> newWeight;
	ArrayN<std::vector<unsigned int>> vidxBody = rigBase->get_vertIdx();
	ArrayN<Vector3r,0> v00=rig_0_0->get_posVert_base();
	ArrayN<std::vector<real>> w00=rig_0_0->get_weights();
	ArrayN<std::vector<unsigned int>> vidx00 = rig_0_0->get_vertIdx();
	ArrayN<Vector3r,0> v10=rig_1_0->get_posVert_base();
	ArrayN<std::vector<real>> w10=rig_1_0->get_weights();
	ArrayN<std::vector<unsigned int>> vidx10 = rig_1_0->get_vertIdx();
	ArrayN<Vector3r,0> v01=rig_0_1->get_posVert_base();
	ArrayN<std::vector<real>> w01=rig_0_1->get_weights();
	ArrayN<std::vector<unsigned int>> vidx01 = rig_0_1->get_vertIdx();
	ArrayN<Vector3r,0> v11=rig_1_1->get_posVert_base();
	ArrayN<std::vector<real>> w11=rig_1_1->get_weights();
	ArrayN<std::vector<unsigned int>> vidx11 = rig_1_1->get_vertIdx();

	
	for(int i=0; i<vidxBody.Size(); i++) {
		vidxBody[i].clear();
		for(int j=0;j<vBody.Size(); j++) {
			vidxBody[i].push_back(j);
		}
	}
	
	for(int i=0; i<wBody.Size(); i++) {
		wBody[i].resize(vBody.Size());
	}
	//double heightWeightClamp = clamp(heightWeight,0.0,2.0);
	//double fatnessWeightClamp = clamp(fatnessWeight,0.0,2.0);
	double heightWeightClamp = heightWeight;
	double fatnessWeightClamp = fatnessWeight;
	for(int i=0; i < rigBase->getSkeleton()->m_pBoneList.Size(); i++) {
		for(int j=0; j<wBody[i].size(); j++) {
			wBody[i][j] = 0;
		}
		Bone* bone_0_0=rig_0_0->getSkeletonBase()->getBone(i);
		Bone* bone_1_0=rig_1_0->getSkeletonBase()->getBone(i);
		Bone* bone_0_1=rig_0_1->getSkeletonBase()->getBone(i);
		Bone* bone_1_1=rig_1_1->getSkeletonBase()->getBone(i);
		vector<unsigned int> vidx=rig_0_0->get_vertIdx()[bone_0_0->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = (1-heightWeightClamp)*(1 - fatnessWeightClamp)*w00[bone_0_0->idx][j];
			wBody[i][vidx[j]] += weight;
		}
		vidx=rig_0_1->get_vertIdx()[bone_0_1->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = heightWeightClamp*(1 - fatnessWeightClamp)*w01[bone_0_1->idx][j];
			wBody[i][vidx[j]] += weight;
		}
		vidx=rig_1_0->get_vertIdx()[bone_1_0->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = (1-heightWeightClamp)*fatnessWeightClamp*w10[bone_1_0->idx][j];
			wBody[i][vidx[j]] += weight;
		}
		vidx=rig_1_1->get_vertIdx()[bone_1_1->idx];
		for(int j=0;j<(int)vidx.size();j++){
			double weight = heightWeightClamp*fatnessWeightClamp*w11[bone_1_1->idx][j];
			wBody[i][vidx[j]] += weight;
		}
	}

	newWeight.Resize(wBody.Size());
	for(int i=0; i < rigBase->getSkeleton()->m_pBoneList.Size(); i++) {
		vidxBody[i].clear();
		for(int j=0; j<wBody[i].size(); j++) {
			if(wBody[i][j]>1e-6) {
				vidxBody[i].push_back(j);
				newWeight[i].push_back(wBody[i][j]);
			}
		}
	}

	ArrayNr weightSum(vBody.Size());
	weightSum.MemsetZero();
	for(int i=0;i<rigBase->getSkeleton()->m_pBoneList.Size();i++) {
		for(size_t j=0;j<vidxBody[i].size();++j) {
			weightSum[vidxBody[i][j]] += newWeight[i][j];
		}
	}

	for(int i=0;i<rigBase->getSkeleton()->m_pBoneList.Size();i++) {
		for(size_t j=0;j<vidxBody[i].size();++j) 
			newWeight[i][j] /= weightSum[vidxBody[i][j]];
	}

	rigBase->set_vertIdx(vidxBody);
	rigBase->set_weights(newWeight);
	wBody = rigBase->get_weights();
	for(int i=0; i<vBody.Size(); i++)
		vBody[i].set(0,0,0);



		Bone* bone=rigBase->getSkeletonBase()->getBone("R_hand");
		Bone* bone_0_0=rig_0_0->getSkeletonBase()->getBone("R_hand");
		Bone* bone_1_0=rig_1_0->getSkeletonBase()->getBone("R_hand");
		Bone* bone_0_1=rig_0_1->getSkeletonBase()->getBone("R_hand");
		Bone* bone_1_1=rig_1_1->getSkeletonBase()->getBone("R_hand");


	for(int j=0;j<(int)vBody.Size();j++){
		Vector3r local_0_0,local_1_0,local_0_1,local_1_1;
		Vector3r vertex00=v00[j]-bone_0_0->origin;
		Vector3r vertex10=v10[j]-bone_1_0->origin;
		Vector3r vertex01=v01[j]-bone_0_1->origin;
		Vector3r vertex11=v11[j]-bone_1_1->origin;

		local_0_0=bone_0_0->transform_Current.getRotationMatrix().getInverse()*vertex00;
		local_1_0=bone_1_0->transform_Current.getRotationMatrix().getInverse()*vertex10;
		local_0_1=bone_0_1->transform_Current.getRotationMatrix().getInverse()*vertex01;
		local_1_1=bone_1_1->transform_Current.getRotationMatrix().getInverse()*vertex11;

		Vector3r temp = (1-heightWeight)*(fatnessWeight*local_1_0 + (1 - fatnessWeight)*local_0_0) + heightWeight*(fatnessWeight*local_1_1 + (1 - fatnessWeight)*local_0_1);
		Vector3r bone_temp = (1-heightWeight)*(fatnessWeight*bone_1_0->origin + (1 - fatnessWeight)*bone_0_0->origin) + heightWeight*(fatnessWeight*bone_1_1->origin + (1 - fatnessWeight)*bone_0_1->origin);
			
		temp = bone->transform_Current.getRotationMatrix()*temp;
		temp = temp+bone_temp;
		vBody[j]+=temp;
	}


	rigBase->setposvert(vBody);

}