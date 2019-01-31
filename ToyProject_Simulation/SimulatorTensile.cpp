#include "ClothSimulator.h"
//Because of Complicate formula, use snake case for index 
//( when variable contain number : snake_case)
void ClothSimulator::calTensile() {
	
	double k = ParameterWindow::getInstance().getStretchStiffness();
	double dts = ParameterWindow::getInstance().getTimestep();

	for (int i = 0; i < nT; i++)
	{
		Vector3i currTri = triangles[i];
		double restArea = restTriangleAreas[i];
		
		// currTri[0] ����
		Vector3d v0 = pos[ currTri[0] ];
		Vector3d v1 = pos[ currTri[1] ];
		Vector3d v2 = pos[ currTri[2] ];

		Vector2d uv0 = restPos[ currTri[0] ];
		Vector2d uv1 = restPos[ currTri[1] ];
		Vector2d uv2 = restPos[ currTri[2] ];

		Vector2d dUv_1 = uv1 - uv0;
		Vector2d dUv_2 = uv2 - uv0;
		
		Vector3d v1_0 = v1-v0;
		Vector3d v2_0 = v2-v0;

		Vector3d w_u = v1_0*dUv_2[1] - v2_0*dUv_1[1];
		Vector3d w_v = (-1.0)*v1_0*dUv_2[0] + v2_0*dUv_1[0];

		double denominator = dUv_1[0]*dUv_2[1] - dUv_2[0]*dUv_1[1];

		w_u /= denominator;
		w_v /= denominator;

		double wux_0 =( dUv_1[1] - dUv_2[1] )/denominator;
		double wux_1 =( dUv_2[1] )/denominator;
		double wux_2 =( -dUv_1[1] )/denominator;

		double wvx_0 =( dUv_2[0] - dUv_1[0] )/denominator;
		double wvx_1 =( -dUv_2[0] )/denominator;
		double wvx_2 =( dUv_1[0] )/denominator;

		Vector3d unitWu = w_u/(w_u.norm());
		Vector3d unitWv = w_v/(w_v.norm());

		MatrixN<double> m0(3,2);
		m0(0,0) = wux_0 * restArea * unitWu[0];
		m0(1,0) = wux_0 * restArea * unitWu[1];
		m0(2,0) = wux_0 * restArea * unitWu[2];
		m0(0,1) = wvx_0 * restArea * unitWv[0];
		m0(1,1) = wvx_0 * restArea * unitWv[1];
		m0(2,1) = wvx_0 * restArea * unitWv[2];

		MatrixN<double> m1(3,2);
		m1(0,0) = wux_1 * restArea * unitWu[0];
		m1(1,0) = wux_1 * restArea * unitWu[1];
		m1(2,0) = wux_1 * restArea * unitWu[2];
		m1(0,1) = wvx_1 * restArea * unitWv[0];
		m1(1,1) = wvx_1 * restArea * unitWv[1];
		m1(2,1) = wvx_1 * restArea * unitWv[2];

		MatrixN<double> m2(3,2);
		m2(0,0) = wux_2 * restArea * unitWu[0];
		m2(1,0) = wux_2 * restArea * unitWu[1];
		m2(2,0) = wux_2 * restArea * unitWu[2];
		m2(0,1) = wvx_2 * restArea * unitWv[0];
		m2(1,1) = wvx_2 * restArea * unitWv[1];
		m2(2,1) = wvx_2 * restArea * unitWv[2];
		
		VectorN<double> stretchCondition(2); 
		stretchCondition(0) = restArea* ((w_u.norm())- 1.0);
		stretchCondition(1) = restArea* ((w_v.norm())- 1.0);

		VectorN<double> force_0 = m0*stretchCondition;
		VectorN<double> force_1 = m1*stretchCondition;
		VectorN<double> force_2 = m2*stretchCondition;

		force[ currTri[0] ] -= k* Vector3d(force_0(0) , force_0(1) , force_0(2));
		force[ currTri[1] ] -= k* Vector3d(force_1(0) , force_1(1) , force_1(2));
		force[ currTri[2] ] -= k* Vector3d(force_2(0) , force_2(1) , force_2(2));


		////////////////////////////////calculate force jacob////////////////////////////////

		vector<MatrixNd> dCdX;
		dCdX.push_back( m0 );
		dCdX.push_back( m1 );
		dCdX.push_back( m2 );

		MatrixNd mTrans_0(2,3);		
		MatrixNd mTrans_1(2,3);		
		MatrixNd mTrans_2(2,3);		

		for (int i = 0; i < m0.nRows; i++)
		{
			for (int j = 0; j < m0.nCols; j++)
			{
				mTrans_0(j,i) = m0(i,j);
				mTrans_1(j,i) = m1(i,j);
				mTrans_2(j,i) = m2(i,j);
			}
		}

		vector<double > dWv_dX;
		dWv_dX.push_back( wvx_0 );
		dWv_dX.push_back( wvx_1 );
		dWv_dX.push_back( wvx_2 );
		vector<double > dWu_dX;
		dWu_dX.push_back( wux_0 );
		dWu_dX.push_back( wux_1 );
		dWu_dX.push_back( wux_2 );
		
		MatrixNd unitWuMat(3,1) , unitWuTrans(1,3),unitWvMat(3,1),unitWvTrans(1,3);
		unitWuMat(0,0)  = unitWu(0);	unitWuTrans(0,0) = unitWu(0);				
		unitWvMat(0,0) = unitWv(0);	unitWvTrans(0,0) = unitWv(0);
		unitWuMat(1,0)  = unitWu(1);	unitWuTrans(0,1) = unitWu(1);				
		unitWvMat(1,0) = unitWv(1);	unitWvTrans(0,1) = unitWv(1);
		unitWuMat(2,0)  = unitWu(2);	unitWuTrans(0,2) = unitWu(2);				
		unitWvMat(2,0) = unitWv(2);	unitWvTrans(0,2) = unitWv(2);

		MatrixNd uBack(3,3) ,	vBack(3,3) , iMat(3,3);
		iMat.identity();

		mul ( uBack ,unitWuMat , unitWuTrans);
		mul ( vBack ,unitWvMat , unitWvTrans);
		uBack *= -1.0;		vBack *= -1.0;
		uBack += iMat;	vBack += iMat;

		Matrix3d backPart_u(uBack.val), backPart_v(vBack.val);





		MatrixNd _front_0_0(3,3);
		MatrixNd _front_1_0(3,3);
		MatrixNd _front_2_0(3,3);
		MatrixNd _front_1_1(3,3);
		MatrixNd _front_1_2(3,3);
		MatrixNd _front_2_2(3,3);

		mul(_front_0_0 , m0 , mTrans_0);
		mul(_front_1_0 , m1 , mTrans_0);
		mul(_front_2_0 , m2 , mTrans_0);
		mul(_front_1_1 , m1 , mTrans_1);
		mul(_front_1_2 , m1 , mTrans_2);
		mul(_front_2_2 , m2 , mTrans_2);

		Matrix3d m0_3d(0.0);
		Matrix3d m1_3d(0.0);
		Matrix3d m2_3d(0.0);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				m0_3d.data[ i*3 + j] = m0(i,j);
				m1_3d.data[ i*3 + j] = m1(i,j);
				m2_3d.data[ i*3 + j] = m2(i,j);	
			}
		}

		Matrix3d front_0_0(m0_3d);  front_0_0 *= m0_3d.transp();
		Matrix3d front_1_0(m1_3d);  front_1_0 *= m0_3d.transp();
		Matrix3d front_2_0(m2_3d);  front_2_0 *= m0_3d.transp();
		Matrix3d front_1_1(m1_3d);  front_1_1 *= m1_3d.transp();
		Matrix3d front_1_2(m1_3d);  front_1_2 *= m2_3d.transp();
		Matrix3d front_2_2(m2_3d);  front_2_2 *= m2_3d.transp();

		
		

		double m0_0_coeff_u = restArea / w_u.norm() * wux_0 *wux_0 ;
		double m0_0_coeff_v = restArea / w_v.norm() * wvx_0 *wvx_0 ;

		double m1_0_coeff_u = restArea / w_u.norm() * wux_1 *wux_0 ;
		double m1_0_coeff_v = restArea / w_v.norm() * wvx_1 *wvx_0 ;

		double m2_0_coeff_u = restArea / w_u.norm() * wux_2 *wux_0 ;
		double m2_0_coeff_v = restArea / w_v.norm() * wvx_2 *wvx_0 ;

		double m1_1_coeff_u = restArea / w_u.norm() * wux_1 *wux_1 ;
		double m1_1_coeff_v = restArea / w_v.norm() * wvx_1 *wvx_1 ;

		double m1_2_coeff_u = restArea / w_u.norm() * wux_1 *wux_2 ;
		double m1_2_coeff_v = restArea / w_v.norm() * wvx_1 *wvx_2 ;

		double m2_2_coeff_u = restArea / w_u.norm() * wux_2 *wux_2 ;
		double m2_2_coeff_v = restArea / w_v.norm() * wvx_2 *wvx_2 ;


		Matrix3d back_0_0 = (backPart_u * m0_0_coeff_u * stretchCondition(0) + backPart_v * m0_0_coeff_v * stretchCondition(1) );
		Matrix3d back_1_0 = (backPart_u * m1_0_coeff_u * stretchCondition(0) + backPart_v * m1_0_coeff_v * stretchCondition(1) );
		Matrix3d back_2_0 = (backPart_u * m2_0_coeff_u * stretchCondition(0) + backPart_v * m2_0_coeff_v * stretchCondition(1) );
		Matrix3d back_1_1 = (backPart_u * m1_1_coeff_u * stretchCondition(0) + backPart_v * m1_1_coeff_v * stretchCondition(1) );
		Matrix3d back_1_2 = (backPart_u * m1_2_coeff_u * stretchCondition(0) + backPart_v * m1_2_coeff_v * stretchCondition(1) );
		Matrix3d back_2_2 = (backPart_u * m2_2_coeff_u * stretchCondition(0) + backPart_v * m2_2_coeff_v * stretchCondition(1) );



		Matrix3d jacob_0_0 = -k * (front_0_0 + back_0_0);
		Matrix3d jacob_1_0 = -k * (front_1_0 + back_1_0);
		Matrix3d jacob_2_0 = -k * (front_2_0 + back_2_0);
		Matrix3d jacob_1_1 = -k * (front_1_1 + back_1_1);
		Matrix3d jacob_1_2 = -k * (front_1_2 + back_1_2);
		Matrix3d jacob_2_2 = -k * (front_2_2 + back_2_2);




		matrixA.setValueInc(currTri[0] , currTri[0] ,-1.0* dts * dts * jacob_0_0);
		matrixA.setValueInc(currTri[0] , currTri[1] ,-1.0* dts * dts * jacob_1_0.transp());
		matrixA.setValueInc(currTri[0] , currTri[2] ,-1.0* dts * dts * jacob_2_0.transp());
		matrixA.setValueInc(currTri[1] , currTri[0] ,-1.0* dts * dts * jacob_1_0);
		matrixA.setValueInc(currTri[1] , currTri[1] ,-1.0* dts * dts * jacob_1_1);
		matrixA.setValueInc(currTri[1] , currTri[2] ,-1.0* dts * dts * jacob_1_2);
		matrixA.setValueInc(currTri[2] , currTri[0] ,-1.0* dts * dts * jacob_2_0);
		matrixA.setValueInc(currTri[2] , currTri[1] ,-1.0* dts * dts * jacob_1_2.transp());
		matrixA.setValueInc(currTri[2] , currTri[2] ,-1.0* dts * dts * jacob_2_2);

		
		forceJacobSparse.setValueInc(currTri[0] , currTri[0] , dts * jacob_0_0);
		forceJacobSparse.setValueInc(currTri[0] , currTri[1] , dts * jacob_1_0.transp());
		forceJacobSparse.setValueInc(currTri[0] , currTri[2] , dts * jacob_2_0.transp());
		forceJacobSparse.setValueInc(currTri[1] , currTri[0] , dts * jacob_1_0);
		forceJacobSparse.setValueInc(currTri[1] , currTri[1] , dts * jacob_1_1);
		forceJacobSparse.setValueInc(currTri[1] , currTri[2] , dts * jacob_1_2);
		forceJacobSparse.setValueInc(currTri[2] , currTri[0] , dts * jacob_2_0);
		forceJacobSparse.setValueInc(currTri[2] , currTri[1] , dts * jacob_1_2.transp());
		forceJacobSparse.setValueInc(currTri[2] , currTri[2] , dts * jacob_2_2)
		///////////////////////////////////////////DAMPING/////////////////////////////////
		if(ParameterWindow::getInstance().getDampingOnOff())
		{
			double damp_d = ParameterWindow::getInstance().getDampingStiffness();


			Vector3d damp_force_0 = front_0_0 * vel[currTri[0]];
			Vector3d damp_force_1 = front_1_1 * vel[currTri[1]];
			Vector3d damp_force_2 = front_2_2 * vel[currTri[2]];

			
			Vector3d C_dot_0 = m0_3d.transp() * vel[currTri[0]];
			Vector3d C_dot_1 = m1_3d.transp() * vel[currTri[1]];
			Vector3d C_dot_2 = m2_3d.transp() * vel[currTri[2]];

			Matrix3d d_jacob_0_0 = -k * ( backPart_u * m0_0_coeff_u * C_dot_0(0) + backPart_v * m0_0_coeff_v * C_dot_0(1) );
			Matrix3d d_jacob_1_0 = -k * ( backPart_u * m1_0_coeff_u * C_dot_1(0) + backPart_v * m1_0_coeff_v * C_dot_1(1) );
			Matrix3d d_jacob_2_0 = -k * ( backPart_u * m2_0_coeff_u * C_dot_2(0) + backPart_v * m2_0_coeff_v * C_dot_2(1) );
			Matrix3d d_jacob_1_1 = -k * ( backPart_u * m1_1_coeff_u * C_dot_1(0) + backPart_v * m1_1_coeff_v * C_dot_1(1) );
			Matrix3d d_jacob_1_2 = -k * ( backPart_u * m1_2_coeff_u * C_dot_1(0) + backPart_v * m1_2_coeff_v * C_dot_1(1) );
			Matrix3d d_jacob_2_2 = -k * ( backPart_u * m2_2_coeff_u * C_dot_2(0) + backPart_v * m2_2_coeff_v * C_dot_2(1) );

			Matrix3d dv_jacob_0_0 = -k * ( front_0_0 );
			Matrix3d dv_jacob_1_0 = -k * ( front_1_0 );
			Matrix3d dv_jacob_2_0 = -k * ( front_2_0 );
			Matrix3d dv_jacob_1_1 = -k * ( front_1_1 );
			Matrix3d dv_jacob_1_2 = -k * ( front_1_2 );
			Matrix3d dv_jacob_2_2 = -k * ( front_2_2 );


		}







	}

}













void ClothSimulator::calShear(){
	double dts = ParameterWindow::getInstance().getTimestep();
	double k = ParameterWindow::getInstance().getShearStiffness();
	k = 3.0;
	for (int i = 0; i < nT; i++)
	{
		Vector3i currTri = triangles[i];
		double restArea = restTriangleAreas[i];
		
		// currTri[0] ����
		Vector3d v0 = pos[ currTri[0] ];
		Vector3d v1 = pos[ currTri[1] ];
		Vector3d v2 = pos[ currTri[2] ];

		Vector2d uv0 = restPos[ currTri[0] ];
		Vector2d uv1 = restPos[ currTri[1] ];
		Vector2d uv2 = restPos[ currTri[2] ];

		Vector2d dUv_1 = uv1 - uv0;
		Vector2d dUv_2 = uv2 - uv0;
		
		Vector3d v1_0 = v1-v0;
		Vector3d v2_0 = v2-v0;

		Vector3d w_u = v1_0*dUv_2[1] - v2_0*dUv_1[1];
		Vector3d w_v = (-1.0)*v1_0*dUv_2[0] + v2_0*dUv_1[0];

		double denominator = dUv_1[0]*dUv_2[1] - dUv_2[0]*dUv_1[1];

		w_u /= denominator;
		w_v /= denominator;
		//same avobe

		double wux_0 =( dUv_1[1] - dUv_2[1] )/denominator;
		double wux_1 =( dUv_2[1] )/denominator;
		double wux_2 =( -dUv_1[1] )/denominator;

		double wvx_0 =( dUv_2[0] - dUv_1[0] )/denominator;
		double wvx_1 =( -dUv_2[0] )/denominator;
		double wvx_2 =( dUv_1[0] )/denominator;

		double shearCondition = restArea * w_u * w_v;

		Vector3d force_0 = shearCondition * ( ( restArea * wux_0 * w_v ) + ( restArea * wvx_0 * w_u ) );
		Vector3d force_1 = shearCondition * ( ( restArea * wux_1 * w_v ) + ( restArea * wvx_1 * w_u ) );
		Vector3d force_2 = shearCondition * ( ( restArea * wux_2 * w_v ) + ( restArea * wvx_2 * w_u ) );
		


		force[ currTri[0] ] -= k* Vector3d(force_0(0) , force_0(1) , force_0(2));
		force[ currTri[1] ] -= k* Vector3d(force_1(0) , force_1(1) , force_1(2));
		force[ currTri[2] ] -= k* Vector3d(force_2(0) , force_2(1) , force_2(2));





		Vector3d C_x_0 = ( restArea * wux_0 * w_v ) + ( restArea * wvx_0 * w_u );
		Vector3d C_x_1 = ( restArea * wux_1 * w_v ) + ( restArea * wvx_1 * w_u );
		Vector3d C_x_2 = ( restArea * wux_2 * w_v ) + ( restArea * wvx_2 * w_u );

		Matrix3d m0_3d(0.0);
		Matrix3d m1_3d(0.0);
		Matrix3d m2_3d(0.0);

		for (int i = 0; i < 3; i++)
		{
			m0_3d.data[i*3] = C_x_0[i];
			m1_3d.data[i*3] = C_x_1[i];
			m2_3d.data[i*3] = C_x_2[i];
		}
		
		Matrix3d front_0_0(m0_3d);  front_0_0 *= m0_3d.transp();
		Matrix3d front_1_0(m1_3d);  front_1_0 *= m0_3d.transp();
		Matrix3d front_2_0(m2_3d);  front_2_0 *= m0_3d.transp();
		Matrix3d front_1_1(m1_3d);  front_1_1 *= m1_3d.transp();
		Matrix3d front_1_2(m1_3d);  front_1_2 *= m2_3d.transp();
		Matrix3d front_2_2(m2_3d);  front_2_2 *= m2_3d.transp();

		double backScalar_0_0 = restArea * wux_0 * wvx_0 + restArea * wux_0 * wvx_0;
		double backScalar_1_0 = restArea * wux_1 * wvx_0 + restArea * wux_0 * wvx_1;
		double backScalar_2_0 = restArea * wux_2 * wvx_0 + restArea * wux_0 * wvx_2;
		double backScalar_1_1 = restArea * wux_1 * wvx_1 + restArea * wux_1 * wvx_1;
		double backScalar_1_2 = restArea * wux_1 * wvx_2 + restArea * wux_2 * wvx_1;
		double backScalar_2_2 = restArea * wux_2 * wvx_2 + restArea * wux_2 * wvx_2;


		Matrix3d back_0_0 ;
		Matrix3d back_1_0 ;
		Matrix3d back_2_0 ;
		Matrix3d back_1_1 ;
		Matrix3d back_1_2 ;
		Matrix3d back_2_2 ;

		back_0_0.Identity();
		back_1_0.Identity();
		back_2_0.Identity();
		back_1_1.Identity();
		back_1_2.Identity();
		back_2_2.Identity();

		Matrix3d jacob_0_0 = -k * (front_0_0 + shearCondition* backScalar_0_0 * back_0_0);
		Matrix3d jacob_1_0 = -k * (front_1_0 + shearCondition* backScalar_1_0 * back_1_0);
		Matrix3d jacob_2_0 = -k * (front_2_0 + shearCondition* backScalar_2_0 * back_2_0);
		Matrix3d jacob_1_1 = -k * (front_1_1 + shearCondition* backScalar_1_1 * back_1_1);
		Matrix3d jacob_1_2 = -k * (front_1_2 + shearCondition* backScalar_1_2 * back_1_2);
		Matrix3d jacob_2_2 = -k * (front_2_2 + shearCondition* backScalar_2_2 * back_2_2);



		matrixA.setValueInc(currTri[0] , currTri[0] ,-1.0* dts * dts * jacob_0_0);
		matrixA.setValueInc(currTri[0] , currTri[1] ,-1.0* dts * dts * jacob_1_0.transp());
		matrixA.setValueInc(currTri[0] , currTri[2] ,-1.0* dts * dts * jacob_2_0.transp());
		matrixA.setValueInc(currTri[1] , currTri[0] ,-1.0* dts * dts * jacob_1_0);
		matrixA.setValueInc(currTri[1] , currTri[1] ,-1.0* dts * dts * jacob_1_1);
		matrixA.setValueInc(currTri[1] , currTri[2] ,-1.0* dts * dts * jacob_1_2);
		matrixA.setValueInc(currTri[2] , currTri[0] ,-1.0* dts * dts * jacob_2_0);
		matrixA.setValueInc(currTri[2] , currTri[1] ,-1.0* dts * dts * jacob_1_2.transp());
		matrixA.setValueInc(currTri[2] , currTri[2] ,-1.0* dts * dts * jacob_2_2);

		
		forceJacobSparse.setValueInc(currTri[0] , currTri[0] , dts * jacob_0_0);
		forceJacobSparse.setValueInc(currTri[0] , currTri[1] , dts * jacob_1_0.transp());
		forceJacobSparse.setValueInc(currTri[0] , currTri[2] , dts * jacob_2_0.transp());
		forceJacobSparse.setValueInc(currTri[1] , currTri[0] , dts * jacob_1_0);
		forceJacobSparse.setValueInc(currTri[1] , currTri[1] , dts * jacob_1_1);
		forceJacobSparse.setValueInc(currTri[1] , currTri[2] , dts * jacob_1_2);
		forceJacobSparse.setValueInc(currTri[2] , currTri[0] , dts * jacob_2_0);
		forceJacobSparse.setValueInc(currTri[2] , currTri[1] , dts * jacob_1_2.transp());
		forceJacobSparse.setValueInc(currTri[2] , currTri[2] , dts * jacob_2_2);




	}

	

}