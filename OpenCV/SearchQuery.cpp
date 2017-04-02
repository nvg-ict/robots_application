

/* * SearchQuery.cpp
 *
 *  Created on: 15 mrt. 2017
 *      Author: maurice*/



#include "SearchQuery.h"



SearchQuery::SearchQuery(cv::Mat& source, cv::Mat& output, colourfinder::colour  aColour, shapeFinder::shape aShape)
{

	clock_t start = clock ();

	cv::Mat picure_raw, picture_filter,picture_mask, picture_hsv, picture_blur, show;
	cv::cvtColor(source, picture_hsv, cv::COLOR_BGR2HSV);
	cv::GaussianBlur(picture_hsv, picture_blur, cv::Size(3,3),0,0);
	colourfinder::filterOnColour(picture_blur, picture_filter,picture_mask, aColour);
	if(!shapeFinder::findShapes(picture_filter, output, picture_mask, aShape))
			{
			std::cout << "Not found" << std::endl;
			};

	std::cout<< clock() - start<< std::endl;
}

SearchQuery::~SearchQuery() {
	// TODO Auto-generated destructor stub
}






