/*
 * main.cpp
 *
 *  Created on: 21 feb. 2017
 *      Author: maurice
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <sstream>

#include "multipicture.h"
#include "SearchQuery.h"
#include "Configuration.h"
#include "webcam.h"

std::string pictureWindow = "Picture";

std::string input, aColour, aShape;


int main(int argc, char **argv) {

	cv::Mat picture_raw, show ,dst;
	cv::namedWindow(pictureWindow, CV_WINDOW_AUTOSIZE);
	Configuration c;

	picture_raw = getCameraFeed(0);

	if(argc > 1)
	{
		Configuration d(argv[1], picture_raw);
		cv::imshow(pictureWindow, picture_raw);
	}
	else
	{
		while(true){

			std::cout<< "vul eerst een kleur in daarna een figuur"<<std::endl;
			std::getline(std::cin, input);
			picture_raw = getCameraFeed(0);
			std::stringstream  ss(input);

			ss >> aColour >> aShape;
			std::cout << "vorm: " << aShape << " kleur:" << aColour << ": "<< std::endl;
			SearchQuery(picture_raw, dst, colourfinder::colour(c.findColour(aColour)),shapeFinder::shape(c.findShape(aShape)));

			combineScreens(dst, picture_raw,show);
			cv::imshow(pictureWindow, show);
			cv::waitKey(10);
		}
	}



}

