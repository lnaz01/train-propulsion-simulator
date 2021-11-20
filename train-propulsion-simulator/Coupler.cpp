//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "Coupler.h"
#include "CouplingSystem.h"
#include "Function.h"
#include "IntervalSmooth.h"
#include "RailVehicle.h"


Coupler::Coupler(InputFileReader_Simulation* ifrsim) : UserDefinedRRComponent(ifrsim, 0, 1) {
	componentType = 1;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables.push_back(new Function(this, i));
	}
	// User-inputted physical variables
	// 0	-->	Force-deflection curve
	//			X axis	-->	Displacement (inches)
	//			Y axis	-->	Force (kips)
	START_UDRRC_STRING = "Coupler_";
	END_UDRRC_STRING = "_Coupler";
	// Physical variable independent component US units
	// Index 0		-->	Displacement (inches)
	PVISTR_US[0] = "inches";
	// Physical variable dependent component US units
	// Index 0		-->	Loading force (kips)
	PVDSTR_US[0] = "kips";
	// Physical variable dependent component integer boolean 
	// ('true' for integer; 'false' for double)
	// Index 0		-->	Loading force
	PVDINT[0] = false;
	// Physical variable independent component minimum threshold values in US units
	// Index 0		-->	Displacement (inches)
	PVIMIN_US[0] = -5.5;
	// Physical variable independent component minimum threshold values in SI units
	// Index 0		-->	Displacement (meters)
	PVIMIN_SI[0] = UnitConverter::in_To_M(PVIMIN_US[0]);
	// Physical variable independent component minimum maximum threshold values in US units
	// Index 0		-->	Displacement (inches)
	PVIMINMAX_US[0] = 3.5;
	// Physical variable independent component maximum threshold values in US units
	// Index 0		-->	Displacement (inches)
	PVIMAX_US[0] = 5.5;
	// Physical variable independent component maximum threshold values in SI units
	// Index 0		-->	Displacement (meters)
	PVIMAX_SI[0] = UnitConverter::in_To_M(PVIMAX_US[0]);
	// Physical variable dependent component minimum threshold values in US units
	// Index 0		-->	Loading force (kips)
	PVDMIN_US[0] = -550.0;
	// Physical variable dependent component minimum threshold values in SI units
	// Index 0		-->	Loading force (newtons)
	PVDMIN_SI[0] = UnitConverter::kip_To_N(PVDMIN_US[0]);
	// Physical variable dependent component maximum threshold values in US units
	// Index 0		-->	Loading force (kips)
	PVDMAX_US[0] = 550.0;
	// Physical variable dependent component maximum threshold values in SI units
	// Index 0		-->	Loading force (newtons)
	PVDMAX_SI[0] = UnitConverter::kip_To_N(PVDMAX_US[0]);
}


Coupler::~Coupler() {}


void Coupler::convertToSI() {
	// Load physical constants alternative names
	loadPhysicalConstantAlternateNames();
	// Physical variable 0 -- Force-deflection curve
	for (size_t i = 0; i < physicalVariables[0]->intervals.size(); i++) {
		for (size_t j = 0; j < physicalVariables[0]->intervals[i]->points.size();
			j++) {
			double x_us = physicalVariables[0]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[0]->intervals[i]->points[j]->y;
			double x_si = UnitConverter::in_To_M(x_us);
			double y_si = UnitConverter::kip_To_N(y_us);
			physicalVariables[0]->intervals[i]->points[j]->x = x_si;
			physicalVariables[0]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[0]->intervals[i])->zvcb = false;
	}
}


void Coupler::loadPhysicalConstantAlternateNames() {}

