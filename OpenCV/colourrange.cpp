/*
 * colourrange.cpp
 *
 *  Created on: 23 feb. 2017
 *      Author: maurice
 */

#include "colourrange.h"

Colour_range::Colour_range(int l_hue, int l_sat, int l_val, int h_hue,
		int h_sat, int h_val)
:lowRange(cv::Scalar(l_hue,l_sat,l_val)), highRange(cv::Scalar(h_hue, h_sat, h_val))
{
}

Colour_range::~Colour_range() {
	// TODO Auto-generated destructor stub
}

