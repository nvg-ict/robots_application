/*
 * webcam.hpp
 *
 *  Created on: 16 mrt. 2017
 *      Author: maurice
 */

#ifndef WEBCAM_H_
#define WEBCAM_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

cv::Mat getCameraFeed(int index)
{
	cv::Mat output;
	cv::VideoCapture cap(index);

		for (int x = 0; x < 10; ++x) {//collect 30 frames because the webcam has to initialize
			cap >> output;

			cv::waitKey(30);
		}
	return output;
}


#endif /* WEBCAM_H_ */
