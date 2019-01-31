/*
 * MotionCalulator.h
 *
 *  Created on: Nov 18, 2016
 *      Author: luislee
 */

#ifndef ANIMATION_PRO_MOTIONCALC_H_
#define ANIMATION_PRO_MOTIONCALC_H_
#include "Eigen/Dense"
#include "glm/common.hpp"
#include "MotionBody.h"
#include <vector>
using namespace Eigen;
using std::vector;
using std::pair;
#define RADIAN 57.2957

namespace EigenGlmConverter{
	glm::vec3 E2g(Vector3f v);
	Vector3f g2E(glm::vec3 v);
	Quaternionf glmEuler2EigenQuat(glm::vec3 ZXY);
	
};

class motionVec{
public:
	motionVec(){}
	motionVec(glm::vec3 root, glm::vec3* motion, int size){
		rootPosition(0) = root[0];
		rootPosition(1) = root[1];
		rootPosition(2) = root[2];
		rootOrientation = EigenGlmConverter::glmEuler2EigenQuat(motion[0]);//rootOrientation
		for(int i=1;i<size;i++){
			nonRoot.push_back(EigenGlmConverter::glmEuler2EigenQuat(motion[i]));
		}
	}
	Vector3f rootPosition;
	Quaternionf rootOrientation;
	std::vector<Quaternionf > nonRoot;
};

namespace MotionCalulator{
	void smoothMotionTransition(MotionBody & b1, MotionBody& b2);
	namespace TransitionDetails{
		void warpMotion(MotionBody & b1,int b1Step, MotionBody& b2,int b2Step);
		void alignMotion(MotionBody & b1,int b1Step, MotionBody& b2,int b2Step);
		void alignMotionForBlending(MotionBody & b1,int b1Step, MotionBody& b2,int b2Step);
		float calculateWeight(int current,int start, int end);//inline
	}
	MotionBody blendMotion(MotionBody b1_ori,MotionBody b2_ori,double w1);
	namespace BlendingDetails{
		typedef struct BlendingData{
			vector<pair<int,int> > corres;
			vector<int> m1_pair_Index;
			vector<int> m1_pair_Index;
			MatrixXf distanceMat;
			MatrixXf M_;
			int m1_startFrame;
			int m1_startFrame;
		} Data;
		double distance(glm::vec3* B1, glm::vec3* B2,int size);//inline
		void matching(MotionBody& B1, MotionBody& B2, Data& d);
		double Dynamic(int i, int j ,Data& d);
		void makePair(int i, int j, Data& d);
		void alignTraj(MotionBody &b1_ori,MotionBody &b2_ori, Data& d);
	}
};

#endif /* ANIMATION_PRO_MOTIONCALC_H_ */
