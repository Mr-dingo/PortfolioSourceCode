/*
 * picking.cpp
 *
 *  Created on: Mar 27, 2017
 *      Author: luislee
 */

#include "../include/picking.h"


using namespace Eigen;


void Picking::hitUpdate(){

	GLdouble *projMat = new GLdouble[16];
	GLdouble *viewMat = new GLdouble[16];
	GLint *identity = new GLint[4];
	GLdouble xx,yy,zz;
	glGetDoublev(GL_PROJECTION_MATRIX, projMat);
	glGetDoublev(GL_MODELVIEW_MATRIX, viewMat);
	glGetIntegerv(GL_VIEWPORT,identity);
	Eigen::Matrix4d projection = GL2Eigen(projMat);
	Eigen::Matrix4d modelview = GL2Eigen(viewMat);
	if(pickButton){
		modelviewMat = modelview;
		projectionMat = projection;
		pickCal();
		hitCheck();
	}
	else{
		for (int i = 0; i < item.size(); i++) {
				if(pickButton == false){
				item[i]->picked = false;	moving = false;
			}
		}
	}
}
void Picking::getMouse(Vector2d viewport,Vector3d mousePos){
	normalizedPos = Vector3d(mousePos(0)/viewport(0)*2-1,-(mousePos(1)/viewport(1)*2-1),0);
}
void Picking::pickCal(){
	Matrix4d invProjMat = projectionMat.inverse();
	Matrix4d invModelViewjMat = modelviewMat.inverse();
	Vector4d tempNear = (invProjMat*Vector4d(normalizedPos(0),normalizedPos(1),0,1));
	Vector4d tempFar = (invProjMat*Vector4d(normalizedPos(0),normalizedPos(1),1,1));

	tempNear = tempNear/tempNear(3);
	tempFar = tempFar/tempFar(3);
	tempNear = invModelViewjMat*tempNear;
	tempFar = invModelViewjMat*tempFar;
	nearPoint = Vector3d(tempNear(0),tempNear(1),tempNear(2));
	farPoint = Vector3d(tempFar(0),tempFar(1),tempFar(2));
}

struct sort_pred {
    bool operator()(const std::pair<int,int> &left, const std::pair<int,int> &right) {
        return left.second < right.second;
    }
};

void Picking::hitCheck(){
	//farPoint과 nearPoint를 잇는 라인의 vector
	Vector3d nearToFarLine = farPoint-nearPoint;

	for (int i = 0; i < item.size(); i++) {
		if(item[i]->picked == true) return;
	}

	for (int i = 0; i < item.size(); ++i) {
		item[i]->picked = false;
	}
	std::vector<std::pair<int,float> > prior;
	if(pickButton){
		for (int i = 0; i < item.size(); i++) {

			Vector4d temp = item[i]->getOrientation().block<1,4>(3,0);
			Vector3d v_center2near = Vector3d(temp(0),temp(1),temp(2))-nearPoint;
			double distance = (nearToFarLine.cross(v_center2near)).norm();
			distance = distance/nearToFarLine.norm();
			if(distance < item[i]->radius){
				Vector4d z_ = modelviewMat*temp;
				z_ = z_/z_(3);
				z_ = projectionMat*z_;
				std::pair<int,float> p;
				p.first = i; p.second = z_(2);
				prior.push_back(p);
			}
		}
	}
	//priority check ( near object selected )
	if(prior.size()>0){
		std::sort(prior.begin(), prior.end(), [](const std::pair<int,int> &left, const std::pair<int,int> &right)->bool{
        		return left.second < right.second;
				}
    	 	);
		item[prior[0].first]->picked = true; moving = true;
	}
	prior.clear();
}
void Picking::moveObject(Vector3d mouseMove){
	//프로젝션된 후의 z 컴포넌트를 찾자.
	for (int i = 0; i < item.size(); ++i) {
		if(item[i]->picked == true){
			Vector4d originalPos = item[i]->getOrientation().block<1,4>(3,0);
			Vector4d projectionPlane = item[i]->getOrientation().block<1,4>(3,0);
			projectionPlane = projectionMat*modelviewMat*projectionPlane;
			projectionPlane += Vector4d(mouseMove(0),mouseMove(1),0,0)*projectionPlane(3);
			projectionPlane = modelviewMat.inverse()*projectionMat.inverse()*projectionPlane;
			moveDirection = projectionPlane - originalPos;
			Vector3d dxyz = Vector3d(moveDirection(0),moveDirection(1),moveDirection(2));
			item[i]->moveGeometry(dxyz);
			return;
		}
	}
}


Matrix4d GL2Eigen(GLdouble *mat){
	Matrix4d m;
	m.col(0) = Vector4d(mat[0],mat[1],mat[2],mat[3]);
	m.col(1) = Vector4d(mat[4],mat[5],mat[6],mat[7]);
	m.col(2) = Vector4d(mat[8],mat[9],mat[10],mat[11]);
	m.col(3) = Vector4d(mat[12],mat[13],mat[14],mat[15]);
	return m;
}
GLdouble* Eigen2GL(Matrix4d mat){
	GLdouble* m;
	m = new GLdouble[16];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			m[4*i+j] = mat(i,j);
		}
	}
	return m;
}









