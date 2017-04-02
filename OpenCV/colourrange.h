/*
 * colourrange.h
 *
 *  Created on: 23 feb. 2017
 *      Author: maurice
 */

#ifndef COLOURRANGE_H_
#define COLOURRANGE_H_
#include <opencv2/opencv.hpp>

class Colour_range {
public:
	Colour_range(int l_hue, int l_sat, int l_val, int h_hue, int h_sat, int h_val);
	virtual ~Colour_range();

	cv::Scalar low()
	{
		return lowRange;
	}

	cv::Scalar high()
	{
		return highRange;
	}

	std::string toString()
	{
		return std::to_string(lowRange.val[0]) + std::to_string(lowRange.val[1]) +std::to_string(lowRange.val[2])
		+"|" + std::to_string(highRange.val[0]) + std::to_string(highRange.val[1]) + std::to_string(highRange.val[2]);
	}

private:
	cv::Scalar lowRange;
	cv::Scalar highRange;
};

#endif /* COLOURRANGE_H_ */
