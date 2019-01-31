  #include "ClothSimulator.h"
//Because of Complicate formula, use snake case for index 
//( when variable contain number : snake_case)
// Filtered Conjugate Gradient Method
void ClothSimulator::MPCG(MatrixS<Matrix3d>& _A, vector<Vector3d>& deltaVel, vector<Vector3d>& _b, int maxItrNums, double tol){

	bool symetric = false;

	double h_ = ParameterWindow::getInstance().getTimestep();		//time step h
	int vecSize = _b.size();
	
	MatrixS<Matrix3d> preconditioner(_A);
	MatrixS<Matrix3d> preconditionerInv(_A);

	preconditioner.zero();
	preconditionerInv.zero();
	for (int i = 0; i < _A.n; i++)
	{
		Matrix3d comp_ii = _A.get(i,i);
		Vector3d diagonal_part(comp_ii(0,0) , comp_ii(1,1) , comp_ii(2,2));
		
		comp_ii.zero();
		comp_ii.data[0] = diagonal_part(0);
		comp_ii.data[4] = diagonal_part(1);
		comp_ii.data[8] = diagonal_part(2);
		preconditioner.setValue(i,i , comp_ii);

		comp_ii.data[0] = 1.0/diagonal_part(0);
		comp_ii.data[4] = 1.0/diagonal_part(1);
		comp_ii.data[8] = 1.0/diagonal_part(2);
		preconditionerInv.setValue(i,i,comp_ii);

	}
	
	
	/////////////////////////3��° ����////////////////
	double error_0;
	vector<Vector3d> errorTemp;
	errorTemp.resize(_b.size());
	mul(preconditionerInv , filtering(_b) , errorTemp);
	error_0 = dot(filtering(_b) , errorTemp);

	/////////////////////////4��° ����////////////////
	vector<Vector3d> residual;		residual.resize(vecSize);
	vector<Vector3d> multAx;		multAx.resize(vecSize);
	
	mul(_A , deltaVel  , multAx);
	for (int i = 0; i < vecSize; i++)
	{
		residual[i] = (_b[i] - multAx[i]);
		
	}
	residual = std::move(filtering(residual));

	//////////////////5��° ����/////////////////////////
	vector<Vector3d> _c;
	_c.resize(vecSize);
	for (int i = 0; i < vecSize; i++)
	{
		_c[i] = (residual[i]);
	}
	mul(preconditionerInv , _c , _c );
	_c = std::move(filtering(_c));

	//////////////////6��° ����/////////////////////////
	double errorNew = dot(residual , _c );
	double errorOld = 0;

	vector<Vector3d> _q,_s;
	_q.resize(nV);
	_s.resize(nV);


	bool success = false;
	int iter_num = 0;
	//repeat
	for (int iter = 0; iter < maxItrNums; iter++)
	{
		iter_num++;
		if(abs( errorNew )< abs (error_0 * tol * tol )){
			success = true;
			break;
		}
		
		mul(_A , _c , _q);								//8
		_q = std::move(filtering(_q));

		double _alpha = errorNew / dot(_c , _q);		//9

		for ( int i = 0; i < deltaVel.size(); i++){
			deltaVel[i] += _alpha * _c[i];				//10
		}
		for ( int i = 0; i < deltaVel.size(); i++){
			residual[i] -= _alpha * _q[i];				//11
		}
		
		mul( preconditionerInv , residual , _s);		//12
		
		errorOld = errorNew;							//13

		errorNew = dot(residual , _s);					//14

		for (int i = 0; i < _c.size(); i++)
		{ 
			_c[i] = _s[i] + (errorNew/errorOld)*_c[i];	//15
		}
		//filtering _c
		_c = std::move(filtering(_c));
	}

	cout<<"iteration = "<<iter_num<<endl;

}


vector<Vector3d> ClothSimulator::filtering(vector<Vector3d>& input){
	vector<Vector3d> result;
	for (int i = 0; i < input.size(); i++)
	{
		result.push_back(input[i]);
	}

	list<int>::iterator itr;
	for(itr = fixedConstraintVertices.begin(); itr!= fixedConstraintVertices.end(); itr++){
		int index = *itr;
		result[index] = Vector3d(0,0,0);
	}

	return result;

}

void ClothSimulator::calDamping(){
   //Only Air drag damping
	double dts = ParameterWindow::getInstance().getTimestep();
	double coefficient = ParameterWindow::getInstance().getDampingStiffness()*1e-4;
	if(ParameterWindow::getInstance().getDampingOnOff()){

	double l;
	cout << "Damping Check"<<endl;
		for(int i=0; i<nV; i++){
			Vector3d n = Vector3d(0.0);
			double area=0;
			for(int j=0; j<vertexIncidentFaces[i].size(); j++){
			area += restTriangleAreas[vertexIncidentFaces[i][j]]/3.0;
			n += facetNormals[vertexIncidentFaces[i][j]];
			}
			n /= n.norm();
			Vector3d unitVel = vel[i] / vel[i].norm();
			if((dot(n,vel[i])*n).norm() > 1e-4){
				force[i] += -coefficient *area* dot(n,vel[i])*n;
				Matrix3d nnMat = Matrix3d(n,Vector3d(),Vector3d()).transp();
				nnMat *= Matrix3d(n,Vector3d(),Vector3d());
				matrixA.setValueInc(i,i,-dts*coefficient *area* nnMat);
			}

	   }
   }

}