/* * SearchQuery.h
 *
 *  Created on: 15 mrt. 2017
 *      Author: maurice*/


#ifndef SEARCHQUERY_H_
#define SEARCHQUERY_H_

#include "kleurenfilter.h"
#include "figurenfilter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>




class SearchQuery {
public:
	SearchQuery(cv::Mat& source ,cv::Mat& output,  colourfinder::colour  aColour, shapeFinder::shape aShape);
	virtual ~SearchQuery();

	uint64_t rdtsc();

};




#endif  // SEARCHQUERY_H_

