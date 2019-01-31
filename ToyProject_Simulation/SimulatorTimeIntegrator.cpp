#include "ClothSimulator.h"

void ClothSimulator::explicitEuler(){
	double dts = ParameterWindow::getInstance().getTimestep();

	dts = 0.0005;
	// Store currents velocity and position
	for(int i=0; i<nV ; i++) {
		prevPos[i] = pos[i];
		prevVel[i] = vel[i];
	}

	// velocity update
	for(int i=0; i<nV; i++) {
		if(mass[i]==0) continue;
		
		double invMass = 1.0f/mass[i];
		vel[i] = prevVel[i] + dts*invMass*force[i];
	}

	// position update
	for(int i=0; i<nV; i++) {
		pos[i] = prevPos[i] + dts*vel[i];
	}


	list<int>::iterator itr;
	for(itr = fixedConstraintVertices.begin(); itr!= fixedConstraintVertices.end(); itr++){
		int index = *itr;
		pos[index] = prevPos[index];
		vel[index] = 0.0;
	}

}

void ClothSimulator::implicitEuler(){
	double _h = ParameterWindow::getInstance().getTimestep();
	
	
	//setting bVec
	mul(forceJacobSparse , vel , bVec);
	for (int i = 0; i < nV; i++)
	{
		bVec[i] += force[i];
		bVec[i] *= _h;
	}

	MPCG(matrixA, deltaVel, bVec, 3*nV, 0.0001);

	// Store currents velocity and position
	for(int i=0; i<nV ; i++) {
		prevPos[i] = pos[i];
		prevVel[i] = vel[i];
	}

	// velocity update
	for(int i=0; i<nV; i++) {
		vel[i] = vel[i] + deltaVel[i];
	}

	// position update
	for(int i=0; i<nV; i++) {
		pos[i] = pos[i] + _h*vel[i];
	}


	//list<int>::iterator itr;
	//for(itr = fixedConstraintVertices.begin(); itr!= fixedConstraintVertices.end(); itr++){
	//	int index = *itr;
	//	pos[index] = prevPos[index];
	//	vel[index] = 0.0;
	//}


}

void ClothSimulator::rungekutta4(){

}
