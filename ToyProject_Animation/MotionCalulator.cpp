/*
 * MotionCalulator.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: luislee
 */


#include "../include/MotionCalulator.h"
namespace MotionCalulator{

void smoothMotionTransition(MotionBody & b1, MotionBody& b2){
	int b1Step = b1.editPoint[1];
	int b2Step = b2.editPoint[0];
	TransitionDetails::alignMotion(b1,b1Step,b2,b2Step);
	TransitionDetails::warpMotion(b1,b1Step,b2,b2Step);
}

void TransitionDetails::alignMotion(MotionBody & b1,int b1Step, MotionBody& b2,int b2Step){
	motionVec beforeMotionVector(b1.rootPos[b1Step],b1.motionZXY[b1Step],b1.numChannel);
	motionVec afterMotionVector(b2.rootPos[b2Step],b2.motionZXY[b2Step],b2.numChannel);
	Vector3f rootPosDiff = (beforeMotionVector.rootPosition - afterMotionVector.rootPosition);
	Quaternionf rootOriDiff = afterMotionVector.rootOrientation.inverse()*beforeMotionVector.rootOrientation;
	//**********************ALIGNMENT***********************//
	glm::vec3 initRootPos = b2.rootPos[b2Step];
	glm::vec3 diffRootPos = EigenGlmConverter::E2g(rootPosDiff);
	for(int i=b2Step;i<b2.editPoint[1]+1;i++){
		Vector3f localPos = (EigenGlmConverter::g2E(b2.rootPos[i]-initRootPos));
		Vector3f newPos = rootOriDiff._transformVector(localPos)+EigenGlmConverter::g2E(b1.rootPos[b1Step]);
		newPos(1) = b2.rootPos[i][1];
		b2.rootPos[i] = EigenGlmConverter::E2g(newPos);
		Vector3f resultZXY = (EigenGlmConverter::glmEuler2EigenQuat(b2.motionZXY[i][0])*rootOriDiff)
								.toRotationMatrix().eulerAngles(2,0,1);
		resultZXY = resultZXY*RADIAN;
		b2.motionZXY[i][0] = EigenGlmConverter::E2g(resultZXY);
	}

}

void TransitionDetails::warpMotion(MotionBody & b1,int b1Step, MotionBody& b2,int b2Step){
	motionVec beforeMotionVector(b1.rootPos[b1Step],b1.motionZXY[b1Step],b1.numChannel);
	motionVec afterMotionVector(b2.rootPos[b2Step],b2.motionZXY[b2Step],b2.numChannel);
	std::vector<Quaternionf> nonRootDiff;
	for(int i=0;i<afterMotionVector.nonRoot.size();i++){
		nonRootDiff.push_back(afterMotionVector.nonRoot[i].inverse()*beforeMotionVector.nonRoot[i]);
	}
	for(int i=b2Step;i<b2Step+40;i++){
		for(int j=0;j<nonRootDiff.size();j++){
			Quaternionf unit;
			unit = unit.Identity();
			Quaternionf result = nonRootDiff[j].slerp(1-calculateWeight(i,b2Step,b2.total_frame_num),unit);
			Quaternionf q = (EigenGlmConverter::glmEuler2EigenQuat(b2.motionZXY[i][j+1])*result);
			Vector3f warpZXY =q.toRotationMatrix().eulerAngles(2,0,1);
			warpZXY = warpZXY*RADIAN;
			b2.motionZXY[i][j+1] = EigenGlmConverter::E2g(warpZXY);
		}
	}
}


inline float TransitionDetails::calculateWeight(int current,int start,int end){
	//linear
	float x = float(current-start);
	float x_range = float(end-start);
	float y = 1-(x/40);
	if(y<0) y=0;
	if(y>1) y=1;

	return y;
}


void TransitionDetails::alignMotionForBlending(MotionBody & b1,int b1Step, MotionBody& b2,int b2Step){
	motionVec beforeMotionVector(b1.rootPos[b1Step],b1.motionZXY[b1Step],b1.numChannel);
	motionVec afterMotionVector(b2.rootPosori[b2Step],b2.motionZXYori[b2Step],b2.numChannel);
	Vector3f rootPosDiff = (beforeMotionVector.rootPosition - afterMotionVector.rootPosition);
	//rootPosDiff = motionB.rootOrientation._transformVector(vec_diff);
	Quaternionf rootOriDiff = afterMotionVector.rootOrientation.inverse()*beforeMotionVector.rootOrientation;
	//**********************ALIGNMENT***********************//
	glm::vec3 initRootPos = b2.rootPosori[b2Step];
	glm::vec3 diffRootPos = EigenGlmConverter::E2g(rootPosDiff);
	for(int i=b2Step;i<b2.editPoint[1]+1;i++){
		Vector3f localPos = (EigenGlmConverter::g2E(b2.rootPosori[i]-initRootPos));
		Vector3f newPos = rootOriDiff._transformVector(localPos)+EigenGlmConverter::g2E(b1.rootPos[b1Step]);
		newPos(1) = b2.rootPosori[i][1];
		b2.rootPos[i] = EigenGlmConverter::E2g(newPos);
		Quaternionf unit;
		Matrix3f m_ = rootOriDiff.toRotationMatrix();
		m_.col(1) = Vector3f(0,1,0);
		unit = Quaternionf(m_);
		Quaternionf rootResult = rootOriDiff.slerp(1-calculateWeight(i,b2Step,b2.total_frame_num),unit);
		Quaternionf rootQuat =(EigenGlmConverter::glmEuler2EigenQuat(b2.motionZXYori[i][0])*rootResult);
		Vector3f resultZXY = rootQuat.toRotationMatrix().eulerAngles(2,0,1) * RADIAN;
		b2.motionZXY[i][0] = EigenGlmConverter::E2g(resultZXY);
	}
}

};
using std::cout;
using std::endl;
namespace MotionCalulator {

MotionBody blendMotion(MotionBody b1_ori,MotionBody b2_ori,double w1){
	BlendingDetails::Data detailData;
	MotionBody resultMotion , b1, b2; // Motion_ZXY 가 포인터라서 연결을 끊는 방법 구현 : pointer 하나 더만들ㅇ서 그거로 교체한다.
	b1=b1_ori; b2 = b2_ori; resultMotion = b1;
	b1.break_chain();
	b2.break_chain();
	resultMotion.break_chain();
	//
	BlendingDetails::matching(b1,b2,detailData);
	BlendingDetails::align_traj(b1,b2,detailData);
	//b1, b2 변경됨
	int b1_clipSize = b1.editPoint[1]-b1.editPoint[0]+1;
	int b2_clipSize = b2.editPoint[1]-b2.editPoint[0]+1;
	//진짜 Blending step//
	static int past = -1;
	for (int i=0;i<detailData.corres.size();i++){
		for(int j=0;j<b1.numChannel;j++){
			Quaternionf b1_quat = EigenGlmConverter::glmEuler2EigenQuat(b1.motionZXY[detailData.corres[i].first][j]) ;
			Quaternionf b2_quat = EigenGlmConverter::glmEuler2EigenQuat(b2.motionZXY[detailData.corres[i].second][j]);
			Quaternionf resultQuat = b1_quat.slerp(1-w1,b2_quat);
			Vector3f v = resultQuat.toRotationMatrix().eulerAngles(2,0,1) * RADIAN;
			Quaternionf compare = EigenGlmConverter::glmEuler2EigenQuat(EigenGlmConverter::E2g(v));
			resultMotion.motionZXY[detailData.corres[i].first][j] = EigenGlmConverter::E2g(v);
			resultMotion.motionZXYori[detailData.corres[i].first][j] = EigenGlmConverter::E2g(v);
			past = detailData.corres[i].first;

		}
	}
	for (int i=detailData.corres.size()-1;i>0;i--){
		Quaternionf b1_quat = EigenGlmConverter::glmEuler2EigenQuat(b1.motionZXY[detailData.corres[i].first][0]) ;
		Quaternionf b2_quat = EigenGlmConverter::glmEuler2EigenQuat(b2.motionZXY[detailData.corres[i].second][0]);
		Vector3f b1_rootPos =EigenGlmConverter::g2E(b1.rootPos[detailData.corres[i].first]);
		Vector3f b1_rootPosnext = EigenGlmConverter::g2E(b1.rootPos[detailData.corres[i-1].first]);
		Vector3f b2_rootPos = EigenGlmConverter::g2E(b2.rootPos[detailData.corres[i].second]);
		Vector3f b2_rootPosnext = EigenGlmConverter::g2E(b2.rootPos[detailData.corres[i-1].second]);
		Quaternionf resultRootOrien = EigenGlmConverter::glmEuler2EigenQuat(resultMotion.motionZXY[detailData.corres[i].first][0]);
		Vector3f b1_diff = b1_rootPosnext-b1_rootPos;
		Vector3f b2_diff = b2_rootPosnext-b2_rootPos;
		Vector3f b1_diffLocal = b1_quat.inverse()._transformVector(b1_diff);
		Vector3f b2_diffLocal = b2_quat.inverse()._transformVector(b2_diff);
		float b1_length = b1_diffLocal.norm();
		float b2_length = b2_diffLocal.norm();
		if(b1_diff.norm() == 0) {b1_diffLocal = Eigen::Vector3f(0,0,0);}
		if(b2_diff.norm() == 0) {b2_diffLocal = Eigen::Vector3f(0,0,0);}
		float Result_length = (b1_length*w1+b2_length*(1-w1));
		Vector3f resultVec = b1_diffLocal*w1 +b2_diffLocal*(1-w1) ;
		resultVec.normalize();
		resultVec = resultVec * Result_length;
		resultVec = resultRootOrien._transformVector(resultVec);
		resultMotion.rootPos[detailData.corres[i-1].first] = resultMotion.rootPos[detailData.corres[i].first] + EigenGlmConverter::E2g(resultVec);
		resultMotion.rootPosori[detailData.corres[i-1].first] = resultMotion.rootPos[detailData.corres[i].first] ;
	}

	return resultMotion;
}
namespace BlendingDetails{
inline double distance(glm::vec3* b1, glm::vec3* b2,int size){
	double result =0;
	for(int i=1;i<size; i++){//root ori 는 제외
		Quaternionf b1_quat = EigenGlmConverter::glmEuler2EigenQuat(b1[i]);
		Quaternionf b2_quat = EigenGlmConverter::glmEuler2EigenQuat(b2[i]);
		double angular = ((b1_quat.inverse()*b2_quat).w());
		angular = glm::acos(angular)*2;
		result += angular*angular;
	}
	return result;
}

double Dynamic(int i, int j,Data& d){
	if(i==0&&j==0) { 	d.M_(0,0) = d.distanceMat(0,0); return d.M_(0,0); 					}
	if(i==1&&j==0) {	d.M_(i,j) = d.distanceMat(i,j)+d.distanceMat(0,0); return d.M_(i,j);	}
	if(i==0&&j==1) {	d.M_(i,j) = d.distanceMat(i,j)+d.distanceMat(0,0); return d.M_(i,j);	}
	if(i==0&&j>0)  {	d.M_(i,j) = d.distanceMat(i,j)+Dynamic(i,j-1,d); return d.M_(i,j);		}
	if(j==0&&i>0)  {	d.M_(i,j) = d.distanceMat(i,j)+Dynamic(i-1,j,d); return d.M_(i,j);		}
	double a,b,c,min1,min2;
	if(d.M_(i,j) <0){
		a = Dynamic(i-1,j,d);
		b = Dynamic(i,j-1,d);
		c = Dynamic(i-1,j-1,d);
		d.M_(i-1,j) = a;
		d.M_(i,j-1) = b;
		d.M_(i-1,j-1) =c;
	if(a>b)		{ 	min1 = b;	}
	else 		{	min1 = a;	}
	if(min1 >c) {	min2 = c;	}
	else		{	min2 = min1;}
	double result =  min2 + d.distanceMat(i,j);
	d.M_(i,j) = result;
	}
	return d.M_(i,j);
}
void makePair(int i, int j,Data& d){

	d.corres.push_back(std::make_pair(i+d.m1_startFrame,j+d.m1_startFrame));
	if(i==0&&j==0)return;
	else if(i==0&&j!=0)makePair(0,j-1,d);
	else if(i!=0&&j==0)makePair(i-1,0,d);
	else{
		double a,b,c;
		a= d.M_(i-1,j);
		b= d.M_(i,j-1);
		c= d.M_(i-1,j-1);
		if	   (a<=b&& a<=c)makePair(i-1,j,d);
		else if(b<=a&&b<=c)makePair(i,j-1,d);
		else if(c<=b&&c<=a)makePair(i-1,j-1,d);
		else{
			cout<<"Error!"<<endl;
		}
	}
}
void matching(MotionBody& b1,MotionBody& b2,Data& d){
	d.m1_startFrame = b1.editPoint[0];
	d.m1_startFrame = b2.editPoint[0];
	int b1_size = b1.editPoint[1]-b1.editPoint[0]+1;
	int b2_size = b2.editPoint[1]-b2.editPoint[0]+1;
	d.distanceMat.resize(b1_size,b2_size);
	d.M_.resize(b1_size,b2_size);
	d.M_.setConstant(-1);
	int channelSize = b1.numChannel;
	for(int i=b1.editPoint[0] ;i<b1.editPoint[1]+1; i++){
		for(int j=b2.editPoint[0] ; j<b2.editPoint[1]+1; j++){
			d.distanceMat(i-b1.editPoint[0],j-b2.editPoint[0])
					= distance(b1.motionZXYori[i],b2.motionZXYori[j],channelSize);
		}
	}
	Dynamic(b1_size-1,b2_size-1,d);
	makePair(b1_size-1,b2_size-1,d);

}

void align_traj(MotionBody &b1_ori,MotionBody &b2_ori,Data& d){//start frame 저장 후 call
	d.m1_startFrame = b1_ori.editPoint[0];
	d.m1_startFrame = b2_ori.editPoint[0];
	MotionCalulator::TransitionDetails::alignMotionForBlending(b1_ori,d.m1_startFrame,b2_ori,d.m1_startFrame);
	b1_ori.resetMotion();
}

}	//MotionCalculator::Blending Details namespace
}	//MotionCalculator Details namespace

namespace EigenGlmConverter{
	glm::vec3 E2g(Vector3f v){
		glm::vec3 result;
		result[0] = v(0);	result[1] = v(1);	result[2] = v(2);
		return result;
	}
	Vector3f g2E(glm::vec3 v){Vector3f result;
	result(0) = v[0];result(1) = v[1];result(2)=v[2];
	return result;
	}
	Quaternionf glmEuler2EigenQuat(glm::vec3 ZXY){
		Matrix3f m;
		m = AngleAxisf(ZXY[0]/RADIAN, Vector3f::UnitZ())
			*AngleAxisf(ZXY[1]/RADIAN, Vector3f::UnitX())
			*AngleAxisf(ZXY[2]/RADIAN, Vector3f::UnitY());
		return Quaternionf(m);
	}
};
