/*
 * multipicture.hpp
 *
 *  Created on: 25 feb. 2017
 *      Author: maurice
 */

#ifndef MULTIPICTURE_HPP_
#define MULTIPICTURE_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void combineScreens(cv::Mat& screen1, cv::Mat& screen2, cv::Mat& screen3, cv::Mat& screen4, cv::Mat& output){
	cv::Mat multiple1, multiple2;

	cv::hconcat(screen1, screen2, multiple1);
	cv::hconcat(screen3, screen4, multiple2);
	cv::vconcat(multiple1, multiple2, output);
}

void combineScreens(cv::Mat& screen1, cv::Mat& screen2, cv::Mat& screen3, cv::Mat& output){
	cv::Mat multiple1, multiple2, black;

	black = cv::Mat::zeros( screen3.size(), CV_8UC3 );

	cv::hconcat(screen1, screen2, multiple1);
	cv::hconcat(screen3, black, multiple2);
	cv::vconcat(multiple1, multiple2, output);
}

void combineScreens(cv::Mat& screen1, cv::Mat& screen2, cv::Mat& output){
	cv::Mat multiple1, multiple2;

	cv::hconcat(screen1, screen2, output);

}

#endif /* MULTIPICTURE_HPP_ */
