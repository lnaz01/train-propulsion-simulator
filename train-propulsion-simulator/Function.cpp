//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "Function.h"
#include "Car.h"
#include "Coupler.h"
#include "InputFileReader.h"
#include "IntervalStep.h"
#include "IntervalSmooth.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Track.h"


const std::string Function::START_FUNCTION_STRING = "Function_";


const std::string Function::END_FUNCTION_STRING = "_Function";


Function::Function(UserDefinedRRComponent* userDefinedRRComponent, int functionIndex) {
	this->functionIndex = functionIndex;
	// Determine 'stepFunctionBool' value
	if (userDefinedRRComponent->componentType == 4) {
		if ((functionIndex == 2) || (functionIndex == 3)) {
			stepFunctionBool = false;
		}
		else {
			stepFunctionBool = true;
		}
	}
	else {
		stepFunctionBool = false;
	}
	// Determine 'maxNumberOfPointsPerInterval' value
	if (userDefinedRRComponent->componentType == 1) {
		maxNumberOfPointsPerInterval = 2;
	}
	else if (userDefinedRRComponent->componentType == 4) {
		maxNumberOfPointsPerInterval = 2;
	}
	else if (userDefinedRRComponent->componentType == 0) {
		if (functionIndex == 0) {
			maxNumberOfPointsPerInterval = IntervalSmooth::ABSOLUTE_MAX_NUMBER_OF_POINTS;
		}
		else {
			maxNumberOfPointsPerInterval = 2;
		}
	}
	else {
		maxNumberOfPointsPerInterval = IntervalSmooth::ABSOLUTE_MAX_NUMBER_OF_POINTS;
	}
	interpPointsCalcBool = false;
}


Function::Function(bool stepFunctionBool, int maxNumberOfPointsPerInterval) {
	functionIndex = 0;
	this->stepFunctionBool = stepFunctionBool;
	this->maxNumberOfPointsPerInterval = maxNumberOfPointsPerInterval;
	if (stepFunctionBool == true) {
		this->maxNumberOfPointsPerInterval = 2;
	}
	else {
		this->maxNumberOfPointsPerInterval = maxNumberOfPointsPerInterval;
	}
	interpPointsCalcBool = false;
}


Function::~Function() {
	// Delete intervals
	for (size_t i = 0; i < intervals.size(); i++) {
		delete intervals[i];
	}
	// Delete interpolated points variable 'interpPoints', if necessary
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < numInterpPointsPlot; i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
}


void Function::add_interval() {
	int intervalIndex = (int)intervals.size();
	if (stepFunctionBool == true) {
		intervals.push_back(new IntervalStep(intervalIndex));
	}
	else {
		intervals.push_back(new IntervalSmooth(intervalIndex, maxNumberOfPointsPerInterval));
	}
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < numInterpPointsPlot; i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
	interpPointsCalcBool = false;
}


void Function::piecewiseLinear(double* x, double* y, int xySize) {
	for (int i = 0; i < (xySize - 1); i++) {
		if (stepFunctionBool == true) {
			intervals.push_back(new IntervalStep(intervals.size()));
		}
		else {
			intervals.push_back(new IntervalSmooth(intervals.size(),
				maxNumberOfPointsPerInterval));
		}
		intervals[i]->points.push_back(new Point);
		int npml = intervals[i]->points.size();
		intervals[i]->points[npml - 1]->x = x[i];
		intervals[i]->points[npml - 1]->y = y[i];
		intervals[i]->points.push_back(new Point);
		npml = intervals[i]->points.size();
		intervals[i]->points[npml - 1]->x = x[i + 1];
		if (stepFunctionBool == true) {
			intervals[i]->points[npml - 1]->y = y[i];
		}
		else {
			intervals[i]->points[npml - 1]->y = y[i + 1];
		}
	}
}


double Function::interpolate(double xpt, double miny, double maxy) {
	double r = 0.0;
	int nsiml = intervals.size();
	int npml = intervals[nsiml - 1]->points.size();
	if (xpt < intervals[0]->points[0]->x) {
		r = intervals[0]->points[0]->y;
	}
	else if (xpt > intervals[nsiml - 1]->points[npml - 1]->x) {
		r = intervals[nsiml - 1]->points[npml - 1]->y;
	}
	else {
		for (int i = 0; i < nsiml; i++) {
			npml = intervals[i]->points.size();
			if ((xpt >= intervals[i]->points[0]->x) && (xpt <= intervals[i]->points[npml - 1]->x)) {
				r = intervals[i]->interpolate(xpt, miny, maxy);
			}
		}
	}
	return r;
}


void Function::interpolateMult(double miny, double maxy) {
	if ((intervals.size()) >= 1 && (interpPointsCalcBool == false)) {
		// Calculate interpolated points for each interval
		for (size_t i = 0; i < intervals.size(); i++) {
			intervals[i]->interpolateMult(miny, maxy);
		}
		// Allocate space on heap for function interpolated points
		numInterpPointsPlot = (intervals[0]->numInterpPointsPlot + 2) * intervals.size();
		int interpPointsLen2 = 2;
		interpPoints = new double* [numInterpPointsPlot];
		for (int i = 0; i < numInterpPointsPlot; i++) {
			interpPoints[i] = new double[interpPointsLen2];
		}
		// Calculate interpolated points for function
		for (size_t i = 0; i < intervals.size(); i++) {
			for (int j = 0; j < (intervals[i]->numInterpPointsPlot + 2); j++) {
				// Independent variable
				double indvar = intervals[i]->interpPoints[j][0];
				interpPoints[((intervals[i]->numInterpPointsPlot + 2) * i) + j][0] = indvar;
				// Dependent variable
				double depvar = intervals[i]->interpPoints[j][1];
				interpPoints[((intervals[i]->numInterpPointsPlot + 2) * i) + j][1] = depvar;
			}
		}
		// Set interpolated points calculated boolean to true
		interpPointsCalcBool = true;
	}
}


void Function::copy(Function* function) {
	zap();
	stepFunctionBool = function->stepFunctionBool;
	functionIndex = function->functionIndex;
	for (size_t i = 0; i < function->intervals.size(); i++) {
		if (stepFunctionBool == true) {
			intervals.push_back(new IntervalStep(i));
		}
		else {
			intervals.push_back(new IntervalSmooth(i, function->maxNumberOfPointsPerInterval));
		}
		intervals[i]->copy(function->intervals[i]);
	}
}


void Function::zap() {
	// Delete intervals
	for (size_t i = 0; i < intervals.size(); i++) {
		delete intervals[i];
	}
	// Clear intervals vector
	intervals.clear();
	// Delete interpolated points variable 'interpPoints', if necessary
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < numInterpPointsPlot; i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
	// Reset 'interPointsCalcBool' variable to false
	interpPointsCalcBool = false;
}


std::string Function::load(InputFileReader* inputFileReader, UserDefinedRRComponent* udrrc, int functionIndex) {
	// Function index
	this->functionIndex = functionIndex;
	// Set step function boolean
	stepFunctionBool = udrrc->PVDINT[functionIndex];
	// Load intervals
	do {
		inputFileReader->nextLine();
		if ((inputFileReader->currentLine.compare("") != 0) && (inputFileReader->currentLine[0] != InputFileReader::COMMENT_CHARACTER) && (inputFileReader->currentLine.compare(END_FUNCTION_STRING) != 0)) {
			add_interval();
			if (intervals.size() > MAX_NUM_INTERVALS) {
				return std::string("Maximum of ") + std::to_string(MAX_NUM_INTERVALS) + std::string(" intervals allowed per function.  Number of ") +
					std::string("intervals exceeds maximum number of intervals allowed.");
			}
			std::string intervalstr = intervals[intervals.size() - 1]->load(inputFileReader->currentLine, udrrc, this);
			if (intervalstr.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
				return intervalstr;
			}
			// Check that function does not have 'gaps'
			if (intervals.size() > 1) {
				int npts0 = intervals[intervals.size() - 2]->points.size();
				double x0_f = intervals[intervals.size() - 2]->points[npts0 - 1]->x;
				double x1_i = intervals[intervals.size() - 1]->points[0]->x;
				if (x0_f != x1_i) {
					return std::string("Independent variable value of last point of preceding interval must be ") +
						std::string("equal to independent variable value of first point of current interval.");
				}
			}
			// Check that function is continuous if function is not step function ('stepFunctionBool' is false)
			if (stepFunctionBool == false) {
				if (intervals.size() > 1) {
					int npts0 = intervals[intervals.size() - 2]->points.size();
					double y0_f = intervals[intervals.size() - 2]->points[npts0 - 1]->y;
					double y1_i = intervals[intervals.size() - 1]->points[0]->y;
					if (y0_f != y1_i) {
						return std::string("Function must be continuous.  Dependent variable value of last point of preceding interval must be equal ") +
							std::string("to dependent variable value of first point of current interval.");
					}
				}
			}
		}
	} while (inputFileReader->currentLine.compare(END_FUNCTION_STRING) != 0);
	// Check that there is at least one interval defined
	if (intervals.size() == 0) {
		return std::string("At least one interval must be defined per function.");
	}
	// Check for valid last point of last interval for track functions
	if (udrrc->componentType == 0) {
		int nils1 = intervals.size();
		int npts1 = intervals[intervals.size() - 1]->points.size();
		if (intervals[nils1 - 1]->points[npts1 - 1]->x < udrrc->PVIMINMAX_US[functionIndex]) {
			return std::string("Last point of last interval for track functions must have independent value greater than or equal to ") +
				std::to_string(udrrc->PVIMINMAX_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVISTR_US[functionIndex]) +
				std::string(" and less than or equal to ") + std::to_string(udrrc->PVIMAX_US[functionIndex]) + std::string(" ") +
				std::string(udrrc->PVISTR_US[functionIndex]) + std::string(".");
		}
	}
	// Check for valid last point of last interval for coupler functions
	else if (udrrc->componentType == 1) {
		int nils1 = intervals.size();
		int npts1 = intervals[intervals.size() - 1]->points.size();
		if (intervals[nils1 - 1]->points[npts1 - 1]->x < udrrc->PVIMINMAX_US[functionIndex]) {
			return std::string("Last point of last interval for coupler loading and unloading functions must have independent value greater than or ") +
				std::string("equal to ") + std::to_string(udrrc->PVIMINMAX_US[functionIndex]) + std::string(" ") +
				std::string(udrrc->PVISTR_US[functionIndex]) + std::string(" and less than or equal to ") +
				std::to_string(udrrc->PVIMAX_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVISTR_US[functionIndex]) + std::string(".");
		}
		if (intervals[nils1 - 1]->points[npts1 - 1]->y < ((Coupler*)udrrc)->MINFF_US[functionIndex]) {
			if (functionIndex == 0) {
				return std::string("Last point of last interval for coupler loading function must have dependent value greater than or equal to ") +
					std::to_string(((Coupler*)udrrc)->MINFF_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVDSTR_US[functionIndex]) +
					std::string(" and less than or equal to ") + std::to_string(udrrc->PVDMAX_US[functionIndex]) + std::string(" ") +
					std::string(udrrc->PVDSTR_US[functionIndex]) + std::string(".");
			}
			else {
				return std::string("Last point of last interval for coupler unloading function must have dependent value greater than or equal to ") +
					std::to_string(((Coupler*)udrrc)->MINFF_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVDSTR_US[functionIndex]) +
					std::string(" and less than or equal to ") + std::to_string(udrrc->PVDMAX_US[functionIndex]) + std::string(" ") +
					std::string(udrrc->PVDSTR_US[functionIndex]) + std::string(".");
			}
		}
	}
	// Check for valid last point of last interval for car functions
	else if (udrrc->componentType == 2) {
		int nils1 = intervals.size();
		int npts1 = intervals[intervals.size() - 1]->points.size();
		if (intervals[nils1 - 1]->points[npts1 - 1]->x < udrrc->PVIMINMAX_US[functionIndex]) {
			return std::string("Last point of last interval for car function ") + std::to_string(functionIndex + 1) + std::string(" must have independent value ") +
				std::string("greater than or equal to ") + std::to_string(udrrc->PVIMINMAX_US[functionIndex]) + std::string(" ") +
				std::string(udrrc->PVISTR_US[functionIndex]) + std::string(" and less than or equal to ") +
				std::to_string(udrrc->PVIMAX_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVISTR_US[functionIndex]) + std::string(".");
		}
	}
	// Check for valid last point of last interval for locomotive functions
	else if (udrrc->componentType == 3) {
		int nils1 = intervals.size();
		int npts1 = intervals[intervals.size() - 1]->points.size();
		if (intervals[nils1 - 1]->points[npts1 - 1]->x < udrrc->PVIMINMAX_US[functionIndex]) {
			return std::string("Last point of last interval for locomotive function ") + std::to_string(functionIndex + 1) + std::string(" must have independent ") +
				std::string("value greater than or equal to ") + std::to_string(udrrc->PVIMINMAX_US[functionIndex]) + std::string(" ") +
				std::string(udrrc->PVISTR_US[functionIndex]) + std::string(" and less than or equal to ") +
				std::to_string(udrrc->PVIMAX_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVISTR_US[functionIndex]) + std::string(".");
		}
	}
	// Check for valid last point of last interval for locomotive operator functions
	else if (udrrc->componentType == 4) {
		if (((LocomotiveOperator*)udrrc)->DISTANCE_VS_TIME_INDICATOR == 0) {
			int nils1 = intervals.size();
			int npts1 = intervals[intervals.size() - 1]->points.size();
			if (intervals[nils1 - 1]->points[npts1 - 1]->x != udrrc->PVIMAX_US[functionIndex]) {
				return std::string("Last point of last interval for locomotive operator function ") + std::to_string(functionIndex + 1) + std::string(" must have ") +
					std::string("independent value equal to track length (") + std::to_string(udrrc->PVIMAX_US[functionIndex]) + std::string(" ") +
					std::string(udrrc->PVISTR_US[functionIndex]) + std::string(").");
			}
		}
		else if (((LocomotiveOperator*)udrrc)->DISTANCE_VS_TIME_INDICATOR == 1) {
			int nils1 = intervals.size();
			int npts1 = intervals[intervals.size() - 1]->points.size();
			if (intervals[nils1 - 1]->points[npts1 - 1]->x != udrrc->PVIMAX_US[functionIndex]) {
				return std::string("Last point of last interval for locomotive operator function ") + std::to_string(functionIndex + 1) + std::string(" must have ") +
					std::string("independent value equal to maximum allowable simulation time (") +
					std::to_string(udrrc->PVIMAX_US[functionIndex]) + std::string(" ") + std::string(udrrc->PVISTR_US[functionIndex]) +
					std::string(").");
			}
		}
	}
	return InputFileReader::VALID_INPUT_STRING;
}


void Function::console() {
	for (size_t i = 0; i < intervals.size(); i++) {
		intervals[i]->console();
	}
}

