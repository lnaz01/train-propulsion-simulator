//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "IntervalStep.h"
#include "InputFileReader_Simulation.h"
#include "LocomotiveOperator.h"
#include "Track.h"


IntervalStep::IntervalStep(int intervalIndex) : Interval(intervalIndex) {
	maxNumberOfPoints = 2;
	numInterpPointsPlot = 10;
}


IntervalStep::~IntervalStep() {}


void IntervalStep::zap() {
	// Delete points
	for (size_t i = 0; i < points.size(); i++) {
		delete points[i];
	}
	// Clear points vector
	points.clear();
	// Delete interpolated points variable 'interpPoints', if necessary
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < (numInterpPointsPlot + 2); i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
	// Reset 'interPointsCalcBool' variable to false
	interpPointsCalcBool = false;
}


void IntervalStep::add_point(Point* point) {
	points.push_back(point);
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < (numInterpPointsPlot + 2); i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
	interpPointsCalcBool = false;
}


double IntervalStep::interpolate(double xpt, double miny, double maxy) {
	double r = std::numeric_limits<double>::max();
	for (size_t i = 0; i < points.size() - 1; i++) {
		if ((xpt >= points[i]->x) && (xpt <= points[i + 1]->x)) {
			r = points[i]->y;
		}
	}
	return r;
}


std::string IntervalStep::_load(std::string str, UserDefinedRRComponent* udrrc, Function* function) {
	// Check for proper number of points
	std::vector<std::string> substr = InputFileReader::split_string(str, ';');
	if ((substr.size() != MIN_NUMBER_OF_POINTS) && (substr.size() != maxNumberOfPoints)) {
		// Note that MIN_NUMBER_OF_POINTS is equal to maxNumberOfPoints for a step interval, so only one of the variables can be used in return statement below
		return std::string("Invalid interval.  Number of points in step interval must be equal to ") + std::to_string(MIN_NUMBER_OF_POINTS) + std::string(".");
	}
	// Check value of independent variable for first point of first interval for locomotive operator
	if (udrrc->componentType == 4) {
		if (intervalIndex == 0) {
			if (((LocomotiveOperator*)udrrc)->DISTANCE_VS_TIME_INDICATOR == 0) {
				if (points[0]->x != udrrc->inputFileReader_Simulation->userDefinedTracks[0]->MAXD0_US[0]) {
					return std::string("Independent value of first point of first interval for locomotive operator must be equal to ") +
						std::to_string(udrrc->inputFileReader_Simulation->userDefinedTracks[0]->MAXD0_US[0]) + std::string(" ") +
						std::string(udrrc->inputFileReader_Simulation->userDefinedTracks[0]->PVISTR_US[0]) + std::string(".");
				}
			}
			else if (((LocomotiveOperator*)udrrc)->DISTANCE_VS_TIME_INDICATOR == 1) {
				if (points[0]->x != 0.0) {
					return std::string("Independent value of first point of first interval for locomotive operator must be equal to ") +
						std::string("0.0 ") + std::string(udrrc->inputFileReader_Simulation->userDefinedTracks[0]->PVISTR_US[0]) +
						std::string(".");
				}
			}
		}
	}
	// Check that values of dependent variable are equal for both points
	if (points[0]->y != points[1]->y) {
		return std::string("The dependent variable component of both points in a step function must be equal.");
	}
	return InputFileReader::VALID_INPUT_STRING;
}

