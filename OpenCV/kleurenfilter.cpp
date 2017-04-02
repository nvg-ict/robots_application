/*
 * kleurenfilter.cpp
 *
 *  Created on: 16 mrt. 2017
 *      Author: maurice
 */
#include "kleurenfilter.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "colourrange.h"

Colour_range colour_filter_low(0,0,0,255,255,255);
Colour_range colour_filter_high(0,0,0,255,255,255);

Colour_range Geel      (20,78,20,    37,255,255);
Colour_range Alle      (0,0,0, 180,255,255);
Colour_range Rood_laag (0,50,50,     10,255,255);
Colour_range Rood_hoog (160,50,50,   180,255,255);
Colour_range Groen     (36,70,30,  78,255,255);
Colour_range Blauw     (78,92,18,    133,253,250);
Colour_range Zwart_laag(0,0,0,       180,255,50);
Colour_range Zwart_hoog(0,205,0,     180,255,58);
Colour_range Wit_laag  (0,0,156,     180,61,255);
Colour_range Wit_hoog  (0,0,142,     180,29,255);
Colour_range Hout 	   (13,60,50,    30,250,255);

void colourfinder::filterOnColour(cv::Mat& input, cv::Mat& output, cv::Mat& mask_Output, colourfinder::colour aColour)
{
	switch (aColour) {
		case colourfinder::ALL:
			colour_filter_low = Alle;
			colour_filter_high = Alle;
			break;
		case colourfinder::YELLOW:
			colour_filter_low = Geel;
			colour_filter_high = Geel;
			break;
		case colourfinder::RED:
			colour_filter_low = Rood_laag;
			colour_filter_high = Rood_hoog;
			break;
		case colourfinder::GREEN:
			colour_filter_low = Groen;
			colour_filter_high = Groen;
			break;
		case colourfinder::BLUE:
			colour_filter_low = Blauw;
			colour_filter_high = Blauw;
			break;
		case colourfinder::BLACK:
			colour_filter_low = Zwart_laag;
			colour_filter_high = Zwart_hoog;
			break;
		case colourfinder::WHITE:
			colour_filter_low = Wit_laag;
			colour_filter_high = Wit_hoog;
			break;
		case colourfinder::WOOD:
			colour_filter_low = Hout;
			colour_filter_high = Hout;
			break;
		default:
			break;
	}

	cv::Mat mask1, mask2, mask_combi ,desta, dstb;
	cv::inRange(input,colour_filter_low.low(), colour_filter_low.high(), mask1);
	cv::inRange(input,colour_filter_high.low(), colour_filter_high.high(), mask2);

	mask_Output = mask1 | mask2;

	desta = cv::Scalar::all(0);
	input.copyTo(dstb,mask_Output);
	cv::cvtColor(dstb, output, cv::COLOR_HSV2BGR);

}


