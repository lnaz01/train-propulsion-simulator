//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Interval.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "UserDefinedRRComponent.h"


Interval::Interval(int intervalIndex) {
	this->intervalIndex = intervalIndex;
	interpPointsCalcBool = false;
}


Interval::~Interval() {
	// Delete points
	for (size_t i = 0; i < points.size(); i++) {
		delete points[i];
	}
	// Delete interpolated points variable 'interpPoints', if necessary
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < (numInterpPointsPlot + 2); i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
}


void Interval::interpolateMult(double miny, double maxy) {
	if ((points.size()) >= 1 && (interpPointsCalcBool == false)) {
		// Allocate space on heap for interval interpolated points
		int xdSize = numInterpPointsPlot + 2;
		int interpPointsLen2 = 2;
		interpPoints = new double* [xdSize];
		for (int i = 0; i < xdSize; i++) {
			interpPoints[i] = new double[interpPointsLen2];
		}
		// Calculate independent variable values for interval interpolated points
		double del = (points[points.size() - 1]->x - points[0]->x) / (numInterpPointsPlot + 1);
		double* xd = new double[xdSize];
		xd[0] = points[0]->x;
		for (int i = 1; i < (xdSize - 1); i++) {
			xd[i] = points[0]->x + (i * del);
		}
		xd[xdSize - 1] = points[points.size() - 1]->x;
		for (int i = 0; i < xdSize; i++) {
			interpPoints[i][0] = xd[i];
		}
		// Calculate dependent variable values for interval interpolated points
		for (int i = 0; i < xdSize; i++) {
			interpPoints[i][1] = interpolate(xd[i], miny, maxy);
		}
		// Set interpolated points calculated boolean to true
		interpPointsCalcBool = true;
		// Delete 'xd' array
		delete[] xd;
	}
}


void Interval::copy(Interval* interval) {
	zap();
	intervalIndex = interval->intervalIndex;
	for (size_t i = 0; i < interval->points.size(); i++) {
		points.push_back(new Point);
		points[i]->copy(interval->points[i]);
	}
}


std::string Interval::load(std::string str, UserDefinedRRComponent* udrrc, Function* function) {
	// Interval index
	intervalIndex = function->intervals.size() - 1;
	// Split string based on semi-colon ';'
	std::vector<std::string> substr = InputFileReader::split_string(str, ';');
	// Check for proper use of semicolons
	size_t nsc;  // number of semicolons
	nsc = count(str.begin(), str.end(), ';');
	if (nsc != (substr.size() - 1)) {
		return std::string("Invalid interval with missing or misplaced semicolon.");
	}
	// Check for valid points
	for (size_t i = 0; i < substr.size(); i++) {
		add_point(new Point);
		std::string valpt = points[points.size() - 1]->load(substr[i], function);
		if (valpt.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
			return valpt;
		}
	}
	// Check values of point independent variables
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i]->x < udrrc->PVIMIN_US[function->functionIndex]) {
			if (udrrc->PVISTR_US[function->functionIndex].compare("") == 0) {
				return std::string("At least one independent value less than minimum allowable threshold of ") +
					std::to_string(udrrc->PVIMIN_US[function->functionIndex]) + std::string(".");
			}
			else {
				return std::string("At least one independent value less than minimum allowable threshold of ") +
					std::to_string(udrrc->PVIMIN_US[function->functionIndex]) + std::string(" ") +
					std::string(udrrc->PVISTR_US[function->functionIndex]) + std::string(".");
			}
		}
		else if (points[i]->x > udrrc->PVIMAX_US[function->functionIndex]) {
			if (udrrc->PVISTR_US[function->functionIndex].compare("") == 0) {
				return std::string("At least one independent value greater than maximum allowable threshold of ") +
					std::to_string(udrrc->PVIMAX_US[function->functionIndex]) + std::string(".");
			}
			else {
				return std::string("At least one independent value greater than maximum allowable threshold of ") +
					std::to_string(udrrc->PVIMAX_US[function->functionIndex]) + std::string(" ") +
					std::string(udrrc->PVISTR_US[function->functionIndex]) + std::string(".");
			}
		}
	}
	// Check values of point independent variables are increasing
	for (size_t i = 0; i < (points.size() - 1); i++) {
		if (points[i + 1]->x <= points[i]->x) {
			return std::string("Independent values not increasing.");
		}
	}
	// Check values of point dependent variables
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i]->y < udrrc->PVDMIN_US[function->functionIndex]) {
			if (udrrc->PVDSTR_US[function->functionIndex].compare("") == 0) {
				return std::string("At least one dependent value less than minimum threshold of ") +
					std::to_string(udrrc->PVDMIN_US[function->functionIndex]) + std::string(".");
			}
			else {
				return std::string("At least one dependent value less than minimum threshold of ") +
					std::to_string(udrrc->PVDMIN_US[function->functionIndex]) + std::string(" ") +
					std::string(udrrc->PVDSTR_US[function->functionIndex]) + std::string(".");
			}
		}
		else if (points[i]->y > udrrc->PVDMAX_US[function->functionIndex]) {
			if (udrrc->PVDSTR_US[function->functionIndex].compare("") == 0) {
				return std::string("At least one dependent value greater than maximum threshold of ") +
					std::to_string(udrrc->PVDMAX_US[function->functionIndex]) + std::string(".");
			}
			else {
				return std::string("At least one dependent value greater than maximum threshold of ") +
					std::to_string(udrrc->PVDMAX_US[function->functionIndex]) + std::string(" ") +
					std::string(udrrc->PVDSTR_US[function->functionIndex]) + std::string(".");
			}
		}
	}
	// Check specific requirements for smooth interval and step interval
	std::string _valil = _load(str, udrrc, function);
	if (_valil.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
		return _valil;
	}
	return InputFileReader::VALID_INPUT_STRING;
}


void Interval::console() {
	for (size_t i = 0; i < points.size(); i++) {
		points[i]->console();
		if (i < (points.size() - 1)) {
			std::cout << "; ";
		}
		else {
			std::cout << std::endl;
		}
	}
}

