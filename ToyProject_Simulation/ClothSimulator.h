#pragma once

#include <Math/matrix.h>
#include <list>
#include "TriangularMesh.h"
#include "layout/ParameterWindow.h"
#include "Math/performance.h"

class GLWidget;

class ClothSimulator{
public:
	ClothSimulator();
	ClothSimulator(TriangularMesh* clmesh);
	virtual ~ClothSimulator();

	void initWithClothMesh(TriangularMesh* clmesh);
	void reset();
	void updateNextPos();
	void toggleFixedConstraint(int index);
	void addCollider(TriangularMesh* tm);

	friend GLWidget;
private:
	//calculate all kind of force & its jacobian
	void calForceJacobian();

	void calGravity();
	void calTensile();
	void calShear();
	void calBending();
	void calDamping();
	
	// System Solver
	void MPCG(MatrixS<Matrix3d>& _A, vector<Vector3d>& _x, vector<Vector3d>& _b,
												int maxItrNums, double tol);	
	// Constraint filtering
	vector<Vector3d> filtering(vector<Vector3d>& input);

	// Time Integrator
	void implicitEuler();
	void explicitEuler();
	//not yet implemented
	void rungekutta4();

	void updateFacetNormals();
	void updateVertexNormals();

private:
	// Original Mesh
	TriangularMesh*				clothMesh;
	vector<TriangularMesh*>		colliders;
	// Geometry
	unsigned int				nV;
	unsigned int				nE;
	unsigned int				nT;
	unsigned int				nB;

	vector<Vector3i>			triangles;					
	vector<Vector2i>			edges;			
	vector<Vector3i>			trianglesEdges;
	vector<int>					bendingElementIdx;			

	vector<Vector2d>			restPos;	
	vector<Vector3i>			restTri;	
	vector<double>				restLengths;				
	vector<double>				restTriangleAreas;

	vector<Vector3d>			vertexNormals;
	vector<Vector3d>			facetNormals;

	vector<vector<int> >		vertexIncidentFaces;

	// Physical Datas
	vector<double>				mass;	
	vector<Vector3d>			pos;	
	vector<Vector3d>			prevPos;		
	vector<Vector3d>			vel;	
	vector<Vector3d>			prevVel;
	vector<Vector3d>			deltaVel;
	vector<Vector3d>			force;	
	list<int>					fixedConstraintVertices;

	MatrixS<Matrix3d> matrixA;
	MatrixS<Matrix3d> forceJacobSparse;
	MatrixS<Matrix3d> dampingSparse;

	vector<Vector3d> bVec;
	int maxItrNums;
	double tol;
};
