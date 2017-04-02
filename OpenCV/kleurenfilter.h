/*
 * kleurenfilter.h
 *
 *  Created on: 21 feb. 2017
 *      Author: maurice
 */
#pragma once
#ifndef KLEURENFILTER_H_
#define KLEURENFILTER_H_

#include <opencv2/opencv.hpp>
namespace colourfinder{


enum colour {ALL, YELLOW, RED, GREEN, BLUE, BLACK, WHITE, WOOD};

void filterOnColour(cv::Mat& input, cv::Mat& output, cv::Mat& mask_Output, colour aColour);

}

#endif /* KLEURENFILTER_H_ */
