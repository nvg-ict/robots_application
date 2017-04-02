/*
 * figurenfilter.cpp
 *
 *  Created on: 16 mrt. 2017
 *      Author: maurice
 */

#include "figurenfilter.h"


bool shapeFinder::findShapes(cv::Mat& input, cv::Mat& output , cv::Mat& Mask, shapeFinder::shape aShape)
	{
	//Apply blur to smooth edges and use adaptive thresholding
	 unsigned long hits = 0;
	 cv::Mat input_grey, dst;
	 cv::cvtColor(input, input_grey, cv::COLOR_BGR2GRAY);
	 cv::GaussianBlur(input_grey,input_grey,cv::Size(7,5),0);
	 adaptiveThreshold(input_grey, input_grey,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,151,10);
	 Mask = input_grey & Mask;


	 std::vector<std::vector<cv::Point> > contours;
	 std::vector<cv::Vec4i> hierarchy;

	 //finding all contours in the image
	 cv::findContours( Mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	    for( unsigned int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through each contour.
	    {

	    	std::vector<cv::Point> approx;

	    	cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.03, true);



	    	if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
	    	{
	    				continue;
	    	}



			std::vector<double> cos;
			for (unsigned int j = 2; j < approx.size()+1; j++)
			{
				cos.push_back(angle(approx[j%approx.size()], approx[j-2], approx[j-1]));
			}

			// Sort ascending the cosine values
			std::sort(cos.begin(), cos.end());

			double mincos = cos.front();
			double maxcos = cos.back();



	    	if (aShape == TRIANGLE && approx.size() == 3)
	    			{
	    				++hits;
	    				printFigureInfo(contours[i]);
	    				setLabel(input, "TRI", contours[i]);    // Triangles
	    			}


	    	else if (approx.size() >= 4 && approx.size() <= 6)
	    			{
	    				cv::RotatedRect rotated = cv::minAreaRect(contours[i]);

						cv::Point2f vertices[4];
						rotated.points(vertices);

						double length = cv::norm(vertices[0] - vertices[1]);//length of rotated box
						double width = cv::norm(vertices[1] - vertices[2]);//width of rotated box
						double rotatedarea = length * width;
						double contourarea = cv::contourArea(contours[i]);


	    				if (approx.size() == 4 && mincos >= -0.15 && maxcos <= 0.3)
	    				{
	    					if (aShape == SQUARE && std::abs(1 - ((length / width))) <= 0.15  ){
	    					setLabel(input, "VIER", contours[i]);
	    					++hits;
	    					printFigureInfo(contours[i]);
	    					}
	    					else if(aShape == RECTANGLE)
	    					{
	    					setLabel(input, "RECT", contours[i]);
	    					++hits;
	    					printFigureInfo(contours[i]);
	    					}
	    				}


	    				//the contourarea of a semicircel relative to a squarebox area around it is PI/4. Check if it is in this range
	    				else if(aShape == SEMI_CIRCLE && contourarea/rotatedarea > CV_PI/4 - 0.06 && contourarea/rotatedarea < CV_PI/4)
	    				{
	    					for (int i = 0; i < 4; i++)//draw a rotated square around semi circels
	    					{
	    					    cv::line(input, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
	    					}

	    					setLabel(input, "half", contours[i]);
	    					++hits;
	    					printFigureInfo(contours[i]);
	    				}
	    			}
	    		else
	    			{
	    				// Detect and label circles
	    				cv::Rect r = cv::boundingRect(contours[i]);
	    				//if the contour has almost the same height as width and the contour area is almost equal to a perfect circle than I assume it is a circle
	    				if (aShape == CIRCLE && std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&   std::abs(1 - (cv::contourArea(contours[i]) / (CV_PI * std::pow(r.width / 2, 2)))) <= 0.1)
	    				{
	    					setLabel(input, "CIR", contours[i]);
	    					++hits;
	    					printFigureInfo(contours[i]);
	    				}
	    			}


	    }
	    output = input;
	    if(hits < 1)
	    {
	    	return false;
	    }
	    else{

	    	return true;
	    }


}

/**
 * Helper function to display text in the center of a contour
 */
void shapeFinder::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

void shapeFinder::printFigureInfo(std::vector<cv::Point>& contour)
{
	cv::Moments mom = cv::moments(contour, false);
	cv::Point2d center(mom.m10/mom.m00, mom.m01/mom.m00);

	std::cout<<center<<std::endl;

}

static double shapeFinder::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
