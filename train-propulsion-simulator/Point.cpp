//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdlib.h>
#include "Point.h"
#include "Function.h"
#include "InputFileReader.h"


Point::Point() {
	this->x = 0.0;
	this->y = 0.0;
}


Point::Point(double x, double y) {
	this->x = x;
	this->y = y;
}


Point::~Point() {}


void Point::copy(Point* point) {
	x = point->x;
	y = point->y;
}


std::string Point::load(std::string str, Function* function) {
	// Check number of commas
	size_t nc = std::count(str.begin(), str.end(), ',');
	if (nc != 1) {
		std::string return_string = std::string("Point with invalid number of commas.  Valid point has one comma.");
		return return_string;
	}
	// Check number of numbers (should be two numbers -- x and y coordinates)
	std::vector<std::string> stringSegmentList = InputFileReader::split_string(str, ',');
	if (stringSegmentList.size() < 2) {
		return "Invalid point consisting of only one number.";
	}
	else if (stringSegmentList.size() > 2) {
		return "Invalid point consisting of more than two numbers.";
	}
	for (size_t i = 0; i < stringSegmentList.size(); i++) {
		if (function->stepFunctionBool == false) {
			if (i == 0) {
				try {
					x = std::stod(stringSegmentList[i]);
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Point with invalid number for independent component.");
				}
			}
			else {
				try {
					y = std::stod(stringSegmentList[i]);
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Point with invalid number for dependent component.");
				}
			}
		}
		else {
			if (i == 0) {
				try {
					x = std::stod(stringSegmentList[i]);
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Point with invalid number for independent component.");
				}
			}
			else {
				try {
					y = std::stoi(stringSegmentList[i]);
					double y_check = std::stod(stringSegmentList[i]);
					if (y != y_check) {
						return std::string("Point with invalid integer for dependent component.");
					}
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Point with invalid integer for dependent component.");
				}
			}
		}
	}
	return InputFileReader::VALID_INPUT_STRING;
}


void Point::console() {
	std::cout << x << "," << y;
}

