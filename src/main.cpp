/*
 * main.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: nico
 */

#include <cmath>

#include <iostream>

//#include "forwardKinematics.hpp"
//#include "inverseKinematics.hpp"

//#include "Matrix.hpp"

#include "AStar.hpp"

int main(int argc, char **argv) {

	//std::pair<double,double> a = calcPosition(20,20,20);
	//std::cout<<a.first<<","<<a.second<<std::endl;

	try {
		PathAlgorithm::AStar a;

		std::cout << "Let's try A*" << std::endl;
		PathAlgorithm::Path p = a.search(PathAlgorithm::Vertex(10,10,10),Point(18,22));

		std::cout << p.size() << std::endl;
		std::cout << "Done trying A*" << std::endl;

		for (const auto& i : p)
		{
			std::cout << i << std::endl;
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
	}
	return 0;

}
