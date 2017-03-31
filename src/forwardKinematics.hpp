/*
 * forwardKinematics.hpp
 *
 *  Created on: Mar 17, 2017
 *      Author: nico
 */

#ifndef FORWARDKINEMATICS_HPP_
#define FORWARDKINEMATICS_HPP_

#include <utility>
#include <cmath>

const double SIZE1 = 14.605;
const double SIZE2 = 18.733;
const double SIZE3 = 8.573;

const double PI = std::acos(-1);

inline std::pair<double,double> calcPosition(double anglePhi1,double anglePhi2, double anglePhi3,double posX = 0, double posY = 0)
{
	double X = 0;
	double Y = 0;

	X = posX + SIZE1 * std::sin(anglePhi1 * PI /180.0)+ SIZE2 * std::sin((anglePhi1 + anglePhi2) * PI /180.0)+ SIZE3 * std::sin((anglePhi1 + anglePhi2 + anglePhi3) * PI /180.0);
	Y = posY + SIZE1 * std::cos(anglePhi1 * PI /180.0)+ SIZE2 * std::cos((anglePhi1 + anglePhi2) * PI /180.0)+ SIZE3 * std::cos((anglePhi1 + anglePhi2 + anglePhi3) * PI /180.0);

	X = std::round(X * 100) / 100;
	Y = std::round(Y * 100) / 100;

	return std::make_pair(X,Y);
}



#endif /* FORWARDKINEMATICS_HPP_ */
