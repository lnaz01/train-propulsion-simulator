//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "IntervalSmooth.h"
#include "Coupler.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "LinearSystem.h"
#include "LocomotiveOperator.h"
#include "Track.h"


IntervalSmooth::IntervalSmooth(int intervalIndex, int maxNumberOfPoints) : Interval(intervalIndex) {
	linearSystem = new LinearSystem();
	zvcb = false;
	this->maxNumberOfPoints = maxNumberOfPoints;
	numInterpPointsPlot = 100;
}


IntervalSmooth::~IntervalSmooth() {
	delete linearSystem;
	if (zvcb == true) {
		delete[] z;
		delete[] h;
	}
}


void IntervalSmooth::zap() {
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
	// Delete 'z' and 'h' arrays, if necessary
	if (zvcb == true) {
		delete[] z;
		delete[] h;
	}
	// Reset 'zvcb' variable to false
	zvcb = false;
}


void IntervalSmooth::add_point(Point* point) {
	points.push_back(point);
	if (interpPointsCalcBool == true) {
		for (int i = 0; i < (numInterpPointsPlot + 2); i++) {
			delete[] interpPoints[i];
		}
		delete[] interpPoints;
	}
	interpPointsCalcBool = false;
	if (zvcb == true) {
		delete[] z;
		delete[] h;
	}
	zvcb = false;
}


double IntervalSmooth::interpolate(double xpt, double miny, double maxy) {
	if (zvcb == false) {
		calc_z();
		zvcb = true;
	}
	double r = std::numeric_limits<double>::max();
	for (size_t i = 0; i < (points.size() - 1); i++) {
		if ((xpt >= points[i]->x) && (xpt <= points[i + 1]->x)) {
			double term1 = (z[i] / (6.0 * h[i + 1])) * pow((points[i + 1]->x - xpt), 3.0);
			double term2 = (z[i + 1] / (6.0 * h[i + 1])) * pow((xpt - points[i]->x), 3.0);
			double term3 = ((points[i + 1]->y / h[i + 1]) - ((z[i + 1] * h[i + 1]) / 6.0)) * (xpt - points[i]->x);
			double term4 = ((points[i]->y / h[i + 1]) - ((z[i] * h[i + 1]) / 6.0)) * (points[i + 1]->x - xpt);
			r = term1 + term2 + term3 + term4;
		}
	}
	if (r < miny) {
		r = miny;
	}
	else if (r > maxy) {
		r = maxy;
	}
	return r;
}


std::string IntervalSmooth::_load(std::string str, UserDefinedRRComponent* udrrc, Function* function) {
	// Check for proper number of points
	std::vector<std::string> substr = InputFileReader::split_string(str, ';');
	if (((int)substr.size() < MIN_NUMBER_OF_POINTS) || ((int)substr.size() > maxNumberOfPoints)) {
		if (MIN_NUMBER_OF_POINTS == maxNumberOfPoints) {
			return std::string("Invalid interval.  Number of points in interval must be equal to ") + std::to_string(MIN_NUMBER_OF_POINTS) + std::string(".");
		}
		else {
			return std::string("Invalid interval.  Number of points in interval must be greater than or equal to ") + std::to_string(MIN_NUMBER_OF_POINTS) +
				std::string(" ") + std::string("and less than or equal to ") + std::to_string(maxNumberOfPoints) + std::string(".");
		}
	}
	// Check value of independent variable for first point of first interval for coupler
	if (udrrc->componentType == 1) {
		if ((intervalIndex == 0) && (points[0]->x > ((Coupler*)udrrc)->MAXD0_US[function->functionIndex])) {
			return std::string("Independent value of first point of first interval for coupler must be less than ") +
				std::to_string(((Coupler*)udrrc)->MAXD0_US[function->functionIndex]) + std::string(" ") +
				std::string(udrrc->PVISTR_US[function->functionIndex]) + std::string(" and greater than ") +
				std::to_string(udrrc->PVIMIN_US[function->functionIndex]) + std::string(" ") +
				std::string(udrrc->PVISTR_US[function->functionIndex]) + std::string(".");
		}
	}
	// Check value of dependent variables for first point of first interval for coupler
	if (udrrc->componentType == 1) {
		if ((intervalIndex == 0) && (points[0]->y > ((Coupler*)udrrc)->MAXF0_US[function->functionIndex])) {
			return std::string("Dependent value of first point of first interval for coupler must be less than ") +
				std::to_string(((Coupler*)udrrc)->MAXF0_US[function->functionIndex]) + std::string(" ") +
				std::string(udrrc->PVDSTR_US[function->functionIndex]) + std::string(" and greater than ") +
				std::to_string(udrrc->PVDMIN_US[function->functionIndex]) + std::string(" ") +
				std::string(udrrc->PVDSTR_US[function->functionIndex]) + std::string(".");
		}
	}
	// Check values of dependent variables for coupler
	if (udrrc->componentType == 1) {
		for (size_t i = 0; i < points.size(); i++) {
			if ((i > 0) && (points[i]->y <= points[i - 1]->y)) {
				return std::string("Dependent components in coupler loading and unloading functions must be increasing.");
			}
			if (i > 0) {
				double slp = (points[i]->y - points[i - 1]->y) / (points[i]->x - points[i - 1]->x);
				if ((slp < ((Coupler*)udrrc)->MINS_US) || (slp > ((Coupler*)udrrc)->MAXS_US)) {
					return std::string("Coupler loading and unloading functions must have slope greater than ") +
						std::to_string(((Coupler*)udrrc)->MINS_US) + std::string(" kips per inch and less than ") +
						std::to_string(((Coupler*)udrrc)->MAXS_US) + std::string(" kips per inch.");
				}
			}
		}
	}
	// Check value of independent variable for first point of first interval for track
	if (udrrc->componentType == 0) {
		if ((intervalIndex == 0) && (points[0]->x != ((Track*)udrrc)->MAXD0_US[function->functionIndex])) {
			return std::string("Independent value of first point of first interval for track must be equal to ") +
				std::to_string(((Track*)udrrc)->MAXD0_US[function->functionIndex]) + " " +
				std::string(udrrc->PVISTR_US[function->functionIndex]) + std::string(".");
		}
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
						std::string("0.0 ") + std::string(udrrc->inputFileReader_Simulation->userDefinedTracks[0]->PVISTR_US[0]) + std::string(".");
				}
			}
		}
	}
	return InputFileReader::VALID_INPUT_STRING;
}


void IntervalSmooth::calc_z() {
	calc_h();
	// Forcing vector
	double* d = new double[points.size()];
	for (size_t i = 0; i < points.size(); i++) {
		if (i == 0) {
			double yprime = (points[1]->y - points[0]->y) / h[1];
			d[i] = ((6.0 / h[1]) * (points[1]->y - points[0]->y)) - (6.0 * yprime);
		}
		else if (i == (points.size() - 1)) {
			double yprime = (points[points.size() - 1]->y - points[points.size() - 2]->y) / h[points.size() - 1];
			d[points.size() - 1] = (6.0 * yprime) - ((6.0 / h[points.size() - 1]) * (points[points.size() - 1]->y - points[points.size() - 2]->y));
		}
		else {
			d[i] = ((6.0 / h[i + 1]) *
				(points[i + 1]->y - points[i]->y)) - ((6.0 / h[i]) * (points[i]->y - points[i - 1]->y));
		}
	}
	// System matrix
	double** a = new double* [points.size()];
	for (size_t i = 0; i < points.size(); i++) {
		a[i] = new double[points.size()];
	}
	for (size_t i = 0; i < points.size(); i++) {
		for (size_t j = 0; j < points.size(); j++) {
			if (i == j) {
				if (i == (points.size() - 1)) {
					a[i][j] = 2.0 * h[i];
				}
				else {
					a[i][j] = 2.0 * (h[i] + h[i + 1]);
				}
			}
			else if (j == (i + 1)) {
				a[i][j] = h[j];
			}
			else if (i == (j + 1)) {
				a[i][j] = h[i];
			}
			else {
				a[i][j] = 0.0;
			}
		}
	}
	linearSystem->gaussElimination(a, d, points.size());
	z = new double[points.size()];
	for (size_t i = 0; i < points.size(); i++) {
		z[i] = linearSystem->stateSpaceVector[i];
	}
	// Delete system matrix 'a'
	for (size_t i = 0; i < points.size(); i++) {
		delete[] a[i];
	}
	delete[] a;
	// Delete forcing vector 'd'
	delete[] d;
}


void IntervalSmooth::calc_h() {
	h = new double[points.size()];
	h[0] = 0.0;
	for (size_t i = 1; i < points.size(); i++) {
		h[i] = points[i]->x - points[i - 1]->x;
	}
}

