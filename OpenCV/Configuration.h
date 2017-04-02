
#ifndef CONFIG_H_
#define CONFIG_H_

#include <iostream>
#include <sstream>
#include <istream>
#include <fstream>
#include "opencv2/opencv.hpp"



#include <string>

class Configuration
{
public:
	/**
	 * @brief This is the constructor for a configuration file. You call this function by giving the filename to it. It should say Configuration c("yourconfigfile.ini)".
	 * It opens the *.ini file and puts the variables into a Map. The configuration string needs to be the file to be opened.
	 */
	Configuration(const std::string& aConfiguration, cv::Mat& input);
	Configuration(){};
	virtual ~Configuration();

	/**
		 * @brief compares given string to multiple options specified in the assignment.
		 * 			if not recognized it trows an runtime error.
		 * @param aShape string thats being compared
		 * @return enumeration of shape
		 */
		int findShape(const std::string& aShape);
		/**
		 * @brief compares given string to multiple options specified in the assignment.
		 * 			if not recognized it trows an runtime error.
		 * @param aColour string thats being compared
		 * @return enumeration of colour
		 */
		int findColour(const std::string& aColour);

private:
	/**
	 * @brief fileName filename to be opened
	 */
	std::string fileName;

	unsigned long currentLine = 0;
};

#endif /* CONFIG_H_ */
