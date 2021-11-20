//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "Track.h"
#include "Function.h"
#include "IntervalSmooth.h"


const double Track::TrackGaugeSI = UnitConverter::in_To_M(56.5);


Track::Track(InputFileReader_Simulation* inputFileReader_Simulation) : UserDefinedRRComponent(inputFileReader_Simulation, 0, 3) {
	componentType = 0;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables.push_back(new Function(this, i));
	}
	slopeEquationCoefficientsCalculatedBool = false;
	START_UDRRC_STRING = "Track_";
	END_UDRRC_STRING = "_Track";
	// User-inputted physical variables
	// 0	-->	Grade (percent)
	// 1	-->	Track curvature (degrees)
	// 2	-->	Superelevation (inches)
	// Physical variable independent component US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	PVISTR_US[0] = "feet";
	PVISTR_US[1] = "feet";
	PVISTR_US[2] = "feet";
	// Physical variable dependent component US units
	// Index 0		-->	grade (percent)
	// Index 1		-->	curvature (degrees)
	// Index 2		-->	superelevation (inches)
	PVDSTR_US[0] = "percent";
	PVDSTR_US[1] = "degrees of curvature";
	PVDSTR_US[2] = "inches of supervelevation";
	// Physical variable dependent component integer boolean 
	// ('true' for integer; 'false' for double)
	// Index 0		-->	grade
	// Index 1		-->	curvature
	// Index 2		-->	superelevation
	PVDINT[0] = false;
	PVDINT[1] = false;
	PVDINT[2] = false;
	// Physical variable independent component minimum threshold values in US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	for (int i = 0; i < physicalVariablesSize; i++) {
		PVIMIN_US[i] = 0.0;
	}
	// Physical variable independent component minimum threshold values in SI units
	// Index 0		-->	Longitudinal position (meters)
	// Index 1		--> Longitudinal position (meters)
	// Index 2		-->	Longitudinal position (meters)
	for (int i = 0; i < physicalVariablesSize; i++) {
		PVIMIN_SI[i] = UnitConverter::ft_To_M(PVIMIN_US[i]);
	}
	// Physical variable independent component minimum maximum threshold values in US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	for (int i = 0; i < physicalVariablesSize; i++) {
		PVIMINMAX_US[i] = Track::MINTL;
	}
	// Physical variable independent component maximum threshold values in US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	for (int i = 0; i < physicalVariablesSize; i++) {
		PVIMAX_US[i] = Track::MAXTL;
	}
	// Physical variable independent component maximum threshold values in SI units
	// Index 0		-->	Longitudinal position (meters)
	// Index 1		--> Longitudinal position (meters)
	// Index 2		-->	Longitudinal position (meters)
	for (int i = 0; i < physicalVariablesSize; i++) {
		PVIMAX_SI[i] = UnitConverter::ft_To_M(PVIMAX_US[i]);
	}
	// Physical variable dependent component minimum threshold values in US units
	// Index 0		-->	grade (percent)
	// Index 1		-->	curvature (degrees)
	// Index 2		-->	superelevation (inches)
	PVDMIN_US[0] = -5.0;
	PVDMIN_US[1] = -10.0;
	PVDMIN_US[2] = -5.0;
	// Physical variable dependent component minimum threshold values in SI units
	// Index 0		-->	grade (newtons)
	// Index 1		-->	curvature (meters^-1)
	// Index 2		-->	superelevation (radians)
	PVDMIN_SI[0] = UnitConverter::pct_To_Rad(PVDMIN_US[0]);
	PVDMIN_SI[1] = UnitConverter::trkCurv_To_GCurv(PVDMIN_US[1]);
	PVDMIN_SI[2] = UnitConverter::inSup_To_Rad(PVDMIN_US[2]);
	// Physical variable dependent component maximum threshold values in US units
	// Index 0		-->	grade (percent)
	// Index 1		-->	curvature (degrees)
	// Index 2		-->	superelevation (inches)
	PVDMAX_US[0] = 5.0;
	PVDMAX_US[1] = 10.0;
	PVDMAX_US[2] = 5.0;
	// Physical variable dependent component maximum threshold values in SI units
	// Index 0		-->	grade (newtons)
	// Index 1		-->	curvature (meters^-1)
	// Index 2		-->	superelevation (radians)
	PVDMAX_SI[0] = UnitConverter::pct_To_Rad(PVDMAX_US[0]);
	PVDMAX_SI[1] = UnitConverter::trkCurv_To_GCurv(PVDMAX_US[1]);
	PVDMAX_SI[2] = UnitConverter::inSup_To_Rad(PVDMAX_US[2]);
}


Track::~Track() {}


void Track::calc_trackLength() {
	int nils = physicalVariables[0]->intervals.size();
	int npts = physicalVariables[0]->intervals[nils - 1]->points.size();
	trackLength = physicalVariables[0]->intervals[nils - 1]->points[npts - 1]->x;
}


double Track::angle(double longPosition) {
	if (slopeEquationCoefficientsCalculatedBool == false) {
		slopeEquationCoefficients();
	}
	double ypnt = std::numeric_limits<double>::max();
	for (size_t i = 0; i < physicalVariables[1]->intervals.size(); i++) {
		double x0 = physicalVariables[1]->intervals[i]->points[0]->x;
		double xf = physicalVariables[1]->intervals[i]->points[1]->x;
		if ((longPosition >= x0) && (longPosition <= xf)) {
			ypnt = (coef2[i] * pow(longPosition, 2.0)) + (coef1[i] * longPosition) + coef0[i];
			break;
		}
	}
	return ypnt;
}


void Track::convertToSI() {
	// Physical variable 0 -- grade
	for (size_t i = 0; i < physicalVariables[0]->intervals.size(); i++) {
		for (size_t j = 0; j < physicalVariables[0]->intervals[i]->points.size(); j++) {
			double x_us = physicalVariables[0]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[0]->intervals[i]->points[j]->y;
			double x_si = UnitConverter::ft_To_M(x_us);
			double y_si = UnitConverter::pct_To_Rad(y_us);
			physicalVariables[0]->intervals[i]->points[j]->x = x_si;
			physicalVariables[0]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[0]->intervals[i])->zvcb = false;
	}
	// Physical variable 1 -- curvature
	for (size_t i = 0; i < physicalVariables[1]->intervals.size(); i++) {
		for (size_t j = 0; j < physicalVariables[1]->intervals[i]->points.size(); j++) {
			double x_us = physicalVariables[1]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[1]->intervals[i]->points[j]->y;
			double x_si = UnitConverter::ft_To_M(x_us);
			double y_si = UnitConverter::trkCurv_To_GCurv(y_us);
			physicalVariables[1]->intervals[i]->points[j]->x = x_si;
			physicalVariables[1]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[1]->intervals[i])->zvcb = false;
	}
	// Physical variable 2 -- superelevation
	for (size_t i = 0; i < physicalVariables[2]->intervals.size(); i++) {
		for (size_t j = 0; j < physicalVariables[2]->intervals[i]->points.size(); j++) {
			double x_us = physicalVariables[2]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[2]->intervals[i]->points[j]->y;
			double x_si = UnitConverter::ft_To_M(x_us);
			double y_si = UnitConverter::inSup_To_Rad(y_us);
			physicalVariables[2]->intervals[i]->points[j]->x = x_si;
			physicalVariables[2]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[2]->intervals[i])->zvcb = false;
	}
}


void Track::loadPhysicalConstantAlternateNames() {}


void Track::slopeEquationCoefficients() {
	slopeEquationCoefficientsCalculatedBool = true;
	for (size_t i = 0; i < physicalVariables[1]->intervals.size(); i++) {
		double x0 = physicalVariables[1]->intervals[i]->points[0]->x;
		double xf = physicalVariables[1]->intervals[i]->points[1]->x;
		double y0 = physicalVariables[1]->intervals[i]->points[0]->y;
		double yf = physicalVariables[1]->intervals[i]->points[1]->y;
		double m = (yf - y0) / (xf - x0);
		coef2.push_back(0.5 * m);
		coef1.push_back(-(x0 * m) + y0);
		coef0.push_back(std::numeric_limits<double>::max());
		if (i == 0) {
			coef0[coef0.size() - 1] =
				0.0
				- (coef2[coef2.size() - 1] * pow(x0, 2.0))
				- (coef1[coef1.size() - 1] * x0);
		}
		else {
			double pifv;  // final value of previous interval
			pifv =
				(coef2[coef2.size() - 2] * pow(x0, 2.0))
				+ (coef1[coef1.size() - 2] * x0)
				+ coef0[coef0.size() - 2];
			coef0[coef0.size() - 1] =
				pifv
				- (coef2[coef2.size() - 1] * pow(x0, 2.0))
				- (coef1[coef1.size() - 1] * x0);
		}
	}
}

