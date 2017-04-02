
#include "Configuration.h"
#include "SearchQuery.h"

Configuration::Configuration(const std::string& aConfiguration, cv::Mat& input) :
		fileName(aConfiguration) {
	cv::Mat output;
	std::string readline;
	std::ifstream configfile(fileName);
	if (configfile.is_open()) {
		while (std::getline(configfile, readline)) {
			++currentLine;
			std::stringstream ss(readline);
			std::string vorm;
			std::string kleur;
			while (ss) {
				if (getline(ss, vorm, ' '))
					break;
			}

			while (ss) {
				if (getline(ss, kleur, '#'))
					break;
			}

			try {
				std::cout << "vorm: " << vorm << " kleur: " << kleur << std::endl;
				SearchQuery search(input, output,colourfinder::colour(findColour(kleur)),
						shapeFinder::shape(findShape(vorm)));
			} catch (std::runtime_error &e) {
				std::cout << e.what() << std::endl;
			}

		}
		configfile.close();
	}

}

Configuration::~Configuration() {}

//circel”,”halve circel”,”vierkant”,”rechthoek” of ”driehoek”
int Configuration::findShape(const std::string& aShape) {
	if (aShape == "cirkel")
		return shapeFinder::CIRCLE;
	if (aShape == "halve_cirkel")
		return shapeFinder::SEMI_CIRCLE;
	if (aShape == "vierkant")
		return shapeFinder::SQUARE;
	if (aShape == "rechthoek")
		return shapeFinder::RECTANGLE;
	if (aShape == "driehoek")
		return shapeFinder::TRIANGLE;
	else {
		std::stringstream errMsg;
		errMsg << "Could not recognize shape '" << aShape << "' on line "
				<< currentLine;
		throw std::runtime_error(errMsg.str().c_str());
	}
}

int Configuration::findColour(const std::string& aColour) {
	if (aColour == "rood")
		return colourfinder::RED;
	if (aColour == "groen")
		return colourfinder::GREEN;
	if (aColour == "blauw")
		return colourfinder::BLUE;
	if (aColour == "zwart")
		return colourfinder::BLACK;
	if (aColour == "wit")
		return colourfinder::WHITE;
	if (aColour == "hout")
		return colourfinder::WOOD;
	if (aColour == "alles")
		return colourfinder::ALL;
	else {
		std::stringstream errMsg;
		errMsg << "Could not recognize colour '" << aColour << "' on line "
				<< currentLine;
		throw std::runtime_error(errMsg.str().c_str());
	}
}
