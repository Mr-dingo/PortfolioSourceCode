#include <Editor/GenerationRule/GenerationRuleAtom.h>
#include <Util/Utils/CLString.h>
#include <Body/JointStruct/Bone.h>
#include <random>
using namespace std;
using namespace dcsbodyspace;

void LengthRuleAtom::fit(){
	if(name == "Waist_Back_Length")
		deform_value = fit_deform_value/100.f;
	else
		deform_value += fit_deform_value/110.f;
	update();
}

void LengthRuleAtom::update() {
	for(int i=0;i<(int)gen_bone_list.size();i++){
		Bone* bone=body->getBody()->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		Bone* bone1=NULL;
		if(body->getBody()->head)
			bone1=body->getBody()->head->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		int idx=bone->idx;
		RotationMatrix<double> RM;
		
		if(deform_type!=GEN_LENGTH){
			if(deform_type==GEN_ROTATION_X)
				RM.set(Vector3<double>::Vector3(1,0,0), deform_value * gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Y)
				RM.set(Vector3<double>::Vector3(0,1,0), deform_value * gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Z)
				RM.set(Vector3<double>::Vector3(0,0,1), deform_value * gen_bone_list_value[i]);
			Matrix4<double> M(RM);
			bone->rotation_Parent_Current=bone->rotation_Parent_Current*M;
			if(bone1)
				bone1->rotation_Parent_Current=bone1->rotation_Parent_Current*M;
		}
		else{
			if(this->idx==7){
				bone->length=bone->length_ori*(1+deform_value);
				if(bone1)
					bone1->length=bone1->length_ori*(1+deform_value);
				if(i!=0){
					real x=dot(bone->dirGlobal.normalize(),Vector3r(1,0,0));
					RM.set(Vector3<double>::Vector3(0,0,1), 1.5*deform_value*gen_bone_list_value[i]);
					Matrix4<double> M(RM);
					bone->dirLocal=RM*dir_local[bone->idx];
					if(bone1)
						bone1->dirLocal=RM*dir_local[bone->idx];
				}
			}
			else{
				bone->length=bone->length*(1+deform_value);
				if(bone1)
					bone1->length=bone1->length*(1+deform_value);
			}
		}
	}
}

HeightRuleAtom::HeightRuleAtom() {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	gen_bone_list_RM.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
}

HeightRuleAtom::HeightRuleAtom(xmlNodePtr node) {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	gen_bone_list_RM.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
	ImportXmlNode(node);
}

HeightRuleAtom::~HeightRuleAtom() {
}

void HeightRuleAtom::fit(){
	deform_value += fit_deform_value/100.f;
	update();
}

void HeightRuleAtom::update() {
	for(int i=0;i<(int)gen_bone_list.size();i++){
		Bone* bone=body->getBody()->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		Bone* bone1=NULL;
		if(body->getBody()->head)
			bone1=body->getBody()->head->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		int idx=bone->idx;
		RotationMatrix<double> RM;
	
		if(deform_type!=GEN_LENGTH){
			if(deform_type==GEN_ROTATION_X)
				RM.set(Vector3<double>::Vector3(1,0,0), (deform_value)*gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Y)
				RM.set(Vector3<double>::Vector3(0,1,0), (deform_value)*gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Z)
				RM.set(Vector3<double>::Vector3(0,0,1), (deform_value)*gen_bone_list_value[i]);
			Matrix4<double> M(RM);
			bone->rotation_Parent_Current=bone->rotation_Parent_Current*M;
			if (bone1)
				bone1->rotation_Parent_Current=bone1->rotation_Parent_Current*M;
		}
		else {
			if (this->idx == 0) {
				bone->length = bone->length_ori * (1 + deform_value * gen_bone_list_value[i]);
				if (bone1)
					bone1->length = bone1->length_ori * (1 + deform_value * gen_bone_list_value[i]);
			}
			else {
				bone->length = bone->length * (1 + deform_value * gen_bone_list_value[i]);
				if (bone1)
					bone1->length = bone1->length * (1 + deform_value * gen_bone_list_value[i]);
			}
		}
	}
}

BreadthRuleAtom::BreadthRuleAtom() {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	gen_bone_list_RM.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
}

BreadthRuleAtom::BreadthRuleAtom(xmlNodePtr node) {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	gen_bone_list_RM.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
	ImportXmlNode(node);
}

BreadthRuleAtom::~BreadthRuleAtom() {
}
void BreadthRuleAtom::fit(){
	deform_value += fit_deform_value/120.f;
	update();
}
void BreadthRuleAtom::update() {
	for(int i=0;i<(int)gen_bone_list.size();i++){
		Bone* bone=body->getBody()->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		Bone* bone1=NULL;
		if(body->getBody()->head)
			bone1=body->getBody()->head->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		int idx=bone->idx;
		RotationMatrix<double> RM;
		
		if(deform_type!=GEN_LENGTH){
			if(deform_type==GEN_ROTATION_X)
				RM.set(Vector3<double>::Vector3(1,0,0), (deform_value)*gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Y)
				RM.set(Vector3<double>::Vector3(0,1,0), (deform_value)*gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Z)
				RM.set(Vector3<double>::Vector3(0,0,1), (deform_value)*gen_bone_list_value[i]);
			Matrix4<double> M(RM);
			bone->rotation_Parent_Current=bone->rotation_Parent_Current*M;
			if(bone1)
				bone1->rotation_Parent_Current=bone1->rotation_Parent_Current*M;
		}
		else{
			bone->length=bone->length*(1+deform_value);
			if(bone1)
				bone1->length=bone1->length*(1+deform_value);
		}
	}
}

DepthRuleAtom::DepthRuleAtom() {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	gen_bone_list_RM.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
}

DepthRuleAtom::DepthRuleAtom(xmlNodePtr node) {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	gen_bone_list_RM.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
	ImportXmlNode(node);
}

DepthRuleAtom::~DepthRuleAtom() {
}

void DepthRuleAtom::update() {
	for(int i=0;i<(int)gen_bone_list.size();i++){
		Bone* bone=body->getBody()->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		Bone* bone1=NULL;
		if(body->getBody()->head)
			bone1=body->getBody()->head->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
		int idx=bone->idx;
		RotationMatrix<double> RM;
		
		if (deform_type!=GEN_LENGTH) {
			if(deform_type==GEN_ROTATION_X)
				RM.set(Vector3<double>::Vector3(1,0,0), (deform_value)*gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Y)
				RM.set(Vector3<double>::Vector3(0,1,0), (deform_value)*gen_bone_list_value[i]);
			else if(deform_type==GEN_ROTATION_Z)
				RM.set(Vector3<double>::Vector3(0,0,1), (deform_value)*gen_bone_list_value[i]);
			Matrix4<double> M(RM);
			bone->rotation_Parent_Current=bone->rotation_Parent_Current*M;
			if (bone1)
				bone1->rotation_Parent_Current=bone1->rotation_Parent_Current*M;
		}
		else {
			bone->length=bone->length*(1+deform_value);
			if(bone1)
				bone1->length=bone1->length*(1+deform_value);
		}
	}
}

CircumRuleAtom::CircumRuleAtom() {
	gen_bone_list.clear();
	gen_bone_list_value.clear();
	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
	manCustomWeightBust.clear();
}

CircumRuleAtom::CircumRuleAtom(xmlNodePtr node) {
	gen_bone_list.clear();
	gen_bone_list_value.clear();

	deform_value=0; old_deform_value=0;
	original_measure=0; deform_percentage=0;
	ImportXmlNode(node);
}

CircumRuleAtom::~CircumRuleAtom() {
}
void CircumRuleAtom::fit(){
	if(name == "Thigh_Circum") 
		fit_deform_value *=0.7f;
	deform_percentage += fit_deform_value;
	update();
}
void CircumRuleAtom::update() {
	int iter = 0;
	ArrayN<Vector3r,0> a=body->getBody()->riggingObject->get_posVert_base();
	ArrayN<std::vector<real>> w=body->getBody()->riggingObject->get_weights();
	ArrayN<Vector3r,0> a1;
	ArrayN<std::vector<real>> w1;
	if(body->getBody()->head) {
		a1=body->getBody()->head->riggingObject->get_posVert_base();
		w1=body->getBody()->head->riggingObject->get_weights();
	}

	deform_value=(deform_percentage/60.f);
	while (true) {
		for (int i = 0; i < (int)gen_bone_list.size(); i++) {
			
			Bone* bone=body->getBody()->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
			if(weight_type==0){
				vector<unsigned int> vidx=body->getBody()->riggingObject->get_vertIdx()[bone->idx];
				for(int j=0;j<(int)vidx.size();j++){
					Vector3r tmp;
					Vector3r v=(*original_vert)[vidx[j]]-bone->origin;
					tmp=bone->transform_Current.getRotationMatrix().getInverse()*v;
					tmp.data[0]=0;
					tmp=bone->transform_Current.getRotationMatrix()*tmp;
					a[vidx[j]]+=w[bone->idx][j]*deform_value*tmp;
				}
			}
			else if( weight_type == -1) {
				if(name == "Bust_Circum"){
				for(int j=0;j<manCustomWeightBust[i].Size();j++){
					
					Vector3r tmp;
					Vector3r v=(*original_vert)[j]-bone->origin;
					tmp=bone->transform_Current.getRotationMatrix().getInverse()*v;
					tmp.data[0]=0;
					tmp=bone->transform_Current.getRotationMatrix()*tmp;
					a[j]+=manCustomWeightBust[i][j]*deform_value*tmp;
				}
				}
				else if(name == "Waist_Circum"){
					for(int j=0;j<manCustomWeightWaist[i].Size();j++){
					
					Vector3r tmp;
					Vector3r v=(*original_vert)[j]-bone->origin;
					tmp=bone->transform_Current.getRotationMatrix().getInverse()*v;
					tmp.data[0]=0;
					tmp=bone->transform_Current.getRotationMatrix()*tmp;
					a[j]+=manCustomWeightWaist[i][j]*deform_value*tmp;
					}
				}
				else if(name =="Knee_Circum"){
					for(int j=0;j<man_custom_weight_knee[i].Size();j++){
					
					Vector3r tmp;
					Vector3r v=(*original_vert)[j]-bone->origin;
					tmp=bone->transform_Current.getRotationMatrix().getInverse()*v;
					tmp.data[0]=0;
					tmp=bone->transform_Current.getRotationMatrix()*tmp;
					a[j]+=man_custom_weight_knee[i][j]*deform_value*tmp;

					}
				}
				else {

				}				

			}
			else{
				for(int j=0;j<custom_weight[i].Size();j++){
					Vector3r tmp;
					Vector3r v=(*original_vert)[j]-bone->origin;
					tmp=bone->transform_Current.getRotationMatrix().getInverse()*v;
					tmp.data[0]=0;
					tmp=bone->transform_Current.getRotationMatrix()*tmp;
					a[j]+=custom_weight[i][j]*deform_value*tmp;
				}
			}
		}
		body->getBody()->riggingObject->setposvert(a);	

		if(body->getBody()->head)  {
			for(int i=0;i<(int)gen_bone_list.size();i++){

				Bone* bone=body->getBody()->head->riggingObject->getSkeleton()->getBone(gen_bone_list[i]);
				vector<unsigned int> vidx=body->getBody()->head->riggingObject->get_vertIdx()[bone->idx];
				for(int j=0;j<(int)vidx.size();j++){
					Vector3r tmp;
					Vector3r v=(*original_vert_head)[vidx[j]]-bone->origin;
					tmp=bone->transform_Current.getRotationMatrix().getInverse()*v;
					tmp.data[0]=0;
					tmp=bone->transform_Current.getRotationMatrix()*tmp;
					a1[vidx[j]]+=w1[bone->idx][j]*deform_value*tmp;
				}
			}
			body->getBody()->head->riggingObject->setposvert(a1);
		}

		iter++;
		if (iter > 0)
			break;
	}
}