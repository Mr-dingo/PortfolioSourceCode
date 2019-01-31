/*
 * picking.h
 *
 *  Created on: Mar 27, 2017
 *      Author: luislee
 */

#include "../external/GL/freeglut.h"
#include "../external/Eigen/Eigen"
#include "Geometry.h"
#include <iostream>
#include <vector>

Eigen::Matrix4d GL2Eigen(GLdouble* mat);
GLdouble* Eigen2GL(Eigen::Matrix4d mat);


class Picking {
public:
	void getMouse(Eigen::Vector2d viewport,Eigen::Vector3d mousePos);
	void hitUpdate();
	void pickCal();
	void hitCheck();
	void moveObject(Eigen::Vector3d mouseMove);
private:
	Eigen::Vector3d normalizedPos;
	Eigen::Vector3d worldPos;
	Eigen::Matrix4d projectionMat;
	Eigen::Matrix4d modelviewMat;
	Eigen::Vector3d nearPoint;
	Eigen::Vector3d farPoint;
	bool pickButton;
	bool moving;
	std::vector<Geometry*> item;
	Eigen::Vector4d moveDirection;
};

