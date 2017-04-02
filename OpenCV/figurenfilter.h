/*
 * figurenfilter.hpp
 *
 *  Created on: 7 mrt. 2017
 *      Author: maurice
 */
#pragma once
#ifndef FIGURENFILTER_HPP_
#define FIGURENFILTER_HPP_


#include <iomanip>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

namespace shapeFinder
{



enum shape{CIRCLE, SEMI_CIRCLE, SQUARE, RECTANGLE, TRIANGLE};
bool findShapes(cv::Mat& input, cv::Mat& output, cv::Mat& Mask, shape aShape);
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
cv::Mat erosionKernel(int a_size, int erosion_elem);
void printFigureInfo(std::vector<cv::Point>& contour);
}

#endif /* FIGURENFILTER_HPP_ */
