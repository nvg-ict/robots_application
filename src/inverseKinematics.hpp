/*
 * inverseKinematics.hpp
 *
 *  Created on: Mar 17, 2017
 *      Author: nico
 */

#ifndef INVERSEKINEMATICS_HPP_
#define INVERSEKINEMATICS_HPP_

#include <cmath>

#include "Matrix.hpp"

//const int SIZE1 = 10;
//const int SIZE2 = 10;
//const int SIZE3 = 10;

/*

Matrix<1,2,double> calculateJacobi(Matrix<1,3>& PHI);

void getConfiguration()
{
	//Matrix<1,2> e = {{0,0}};
	Matrix<1,2, double> g = {{10,10}};
	//while(e!=g)
	//while(true)
	//{

		Matrix<1,3> phi= {{90,90,90}};
	//bereken J(e,PHI) voor huidige phi
		Matrix<1,2,double> e = calculateJacobi(phi);
		std::cout<<"e:\n"<<e<<std::endl;


	//bereken J-

		//Matrix<1,2,double> eI = e.inverse();

		//std::cout<<"eI:\n"<<e.inverse()<<std::endl;


	//bereken d_e = B(g-e)
		double Beta = 0.1;
		int d_e = Beta*(g-e);

	//bereken d_PHI = J-*d_e
		Matrix<1,3,double> d_PHI = J_I*d_e;

	//bereken phi = phi + d_PHI
		phi = phi + d_PHI;

	//bereken forward kinematics nieuwe e
	//}
}

Matrix<1,2,double> calculateJacobi(Matrix<1,3>& PHI)
{

		double X = 0;
		double Y = 0;

		X = SIZE1 * std::cos(PHI[0][0] * PI /180.0)+ SIZE2 * std::cos((PHI[0][0] + PHI[0][1]) * PI /180.0)+ SIZE3 * std::cos((PHI[0][0] + PHI[0][1] + PHI[0][3]) * PI /180.0);
		Y = SIZE1 * std::sin(PHI[0][0] * PI /180.0)+ SIZE2 * std::sin((PHI[0][0] + PHI[0][1]) * PI /180.0)+ SIZE3 * std::sin((PHI[0][0] + PHI[0][1] + PHI[0][3]) * PI /180.0);

		Matrix<1,2,double> e = {{X,Y}};

		return e;
}
*/

#endif /* INVERSEKINEMATICS_HPP_ */
