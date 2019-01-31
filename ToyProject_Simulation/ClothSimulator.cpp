#include "ClothSimulator.h"

ClothSimulator::ClothSimulator(){
	clothMesh = NULL;
	nV = nE = nT = nB = 0;
}

ClothSimulator::ClothSimulator(TriangularMesh* clmesh):clothMesh(clmesh){
	initWithClothMesh(clmesh);
	
	MatrixN<double> m_sparsity(nV , nV);
	MatrixSd ____temp;
	int non_zero = 0;
	m_sparsity.zero();
	for (int i = 0; i < nT; i++)
	{
		Vector3i curr_tri = triangles[i];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m_sparsity.set( curr_tri[i] , curr_tri[j] , 1);	
			}
		}
	}
	____temp.setMatrixN(m_sparsity);
	matrixA.setDim(____temp.n , ____temp.numNonZeros);
	matrixA.setSparsity(____temp);
	matrixA.zero();
	forceJacobSparse.setDim(____temp.n , ____temp.numNonZeros);
	forceJacobSparse.setSparsity(____temp);
	forceJacobSparse.zero();
	dampingSparse.setDim(____temp.n , ____temp.numNonZeros);
	dampingSparse.setSparsity(____temp);
	dampingSparse.zero();

	deltaVel.resize(nV);
	bVec.resize(nV);
	for (int i = 0; i < nV; i++)
	{
		deltaVel[i] = 0.0;
	}
}

ClothSimulator::~ClothSimulator(){

}

void ClothSimulator::updateNextPos(){
	Performance pfForce("Calculate force jacobian");
	pfForce.Start();
	calForceJacobian();
	pfForce.End();

	//explicitEuler();
	implicitEuler();


	Performance pfUpdateNormals("Update Normals");
	pfUpdateNormals.Start();
	updateFacetNormals();
	updateVertexNormals();
	pfUpdateNormals.End();
}

void ClothSimulator::calForceJacobian(){
	if(nV == 0) return;

	//set matrixA �� sparsity
	memset(&(force[0]), 0 ,sizeof(double)*nV*3);
	memset(&(deltaVel[0]), 0 ,sizeof(double)*nV*3);

	matrixA.zero();
	forceJacobSparse.zero();
	dampingSparse.zero();
	for (int i = 0; i < nV; i++)
	{
		Matrix3d mass_mat;
		mass_mat.Identity();
		mass_mat *= mass[i];
		matrixA.setValueInc(i,i , mass_mat);
	}
	////////////////////////////////////////////////////////////////
	// calculate Force & Jacobian 
	// Tensile Force & Jacobian
	calDamping();
	calTensile();
	calShear();
	// Bending Force & Jacobian
	calBending();
	// Gravity
	calGravity();

	
}

void ClothSimulator::calGravity(){
	Vector3d gravity(0,ParameterWindow::getInstance().getGravity(),0);

	// calculate gravity force
	for(int i=0 ; i<nV; i++) { 
		force[i] += mass[i]*gravity;
	}
}

void ClothSimulator::toggleFixedConstraint(int index){
	list<int>::iterator itr = fixedConstraintVertices.begin();
	for(;itr != fixedConstraintVertices.end(); itr++){
		if(*itr < index)
			continue;
		else if(*itr > index){
			fixedConstraintVertices.insert(itr, index);
			return;
		}else if(index == *itr){
			fixedConstraintVertices.erase(itr);
			return;
		}
	}
		
	fixedConstraintVertices.push_back(index);
}

void ClothSimulator::addCollider(TriangularMesh* tm){
	colliders.push_back(tm);
}
