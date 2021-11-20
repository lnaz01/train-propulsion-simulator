//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "LocomotiveOperator.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "IntervalSmooth.h"
#include "IntervalStep.h"
#include "Simulation.h"
#include "Track.h"
#include "TrainConsist.h"
#include "UnitConverter.h"


const double LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE = UnitConverter::psi_To_Pa(105.0);


const double LocomotiveOperator::EMERGENCY_BRAKING_THRESHOLD = UnitConverter::psi_To_Pa(15.1);


const double LocomotiveOperator::MINBPR = UnitConverter::psi_To_Pa(5.0);


LocomotiveOperator::LocomotiveOperator(InputFileReader_Simulation* ifrsim) : UserDefinedRRComponent(ifrsim, 1, 4) {
	componentType = 4;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables.push_back(new Function(this, i));
	}
	// User-inputted physical constants
	// Index 0		-->	Distance based ('0') or time based ('1')
	// User-inputted physical variables
	// Index 0		-->	Automatic air brake setting (psi)
	// Index 1		-->	Independent air brake setting (psi)
	// Index 2		-->	Throttle setting (unitless)
	// Index 3		-->	Dynamic brake setting (unitless)
	// Physical constant 0 -- Distance-based ('0') versus time-based ('1')
	double pcs_us;
	double pcs_si;
	pcs_us = physicalConstants[0];
	pcs_si = pcs_us;
	physicalConstants[0] = pcs_si;
	START_UDRRC_STRING = "LocomotiveOperator_";
	END_UDRRC_STRING = "_LocomotiveOperator";
	// Physical constants US units
	// Index 0		-->	Distance-based versus time-based indicator
	PCSTR_US[0] = "";
	// Physical constants integer boolean
	// Index 0		-->	Distance-based versus time-based indicator
	PCINT[0] = true;
	// Physical constant minimum threshold values in US units
	// Index 0		-->	Distance-based versus time-based indicator
	PCMIN_US[0] = 0.0;
	// Physical constant minimum threshold values in SI units
	// Index 0		-->	Distance-based versus time-based indicator
	PCMIN_SI[0] = PCMIN_US[0];
	// Physical constant maximum threshold values in US units
	// Index 0		-->	Distance-based versus time-based indicator
	PCMAX_US[0] = 1.0;
	// Physical constant maximum threshold values in SI units
	// Index 0		-->	Distance-based versus time-based indicator
	PCMAX_SI[0] = PCMAX_US[0];
}


LocomotiveOperator::~LocomotiveOperator() {}


void LocomotiveOperator::convertToSI() {
	// Load physical constants alternative names
	loadPhysicalConstantAlternateNames();
	// Physical variable 0 (automatic brake)
	for (size_t i = 0; i <= (physicalVariables[0]->intervals.size() - 1); i++) {
		for (size_t j = 0; j <= (physicalVariables[0]->intervals[i]->points.size() - 1); j++) {
			double x_us = physicalVariables[0]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[0]->intervals[i]->points[j]->y;
			double x_si;
			if (DISTANCE_VS_TIME_INDICATOR == 0) {
				x_si = UnitConverter::ft_To_M(x_us);
			}
			else {
				x_si = x_us;
			}
			double y_si = UnitConverter::psi_To_Pa(y_us);
			physicalVariables[0]->intervals[i]->points[j]->x = x_si;
			physicalVariables[0]->intervals[i]->points[j]->y = y_si;
		}
	}
	// Physical variable 1 (independent brake)
	for (size_t i = 0; i <= (physicalVariables[1]->intervals.size() - 1); i++) {
		for (size_t j = 0; j <= (physicalVariables[1]->intervals[i]->points.size() - 1); j++) {
			double x_us = physicalVariables[1]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[1]->intervals[i]->points[j]->y;
			double x_si;
			if (DISTANCE_VS_TIME_INDICATOR == 0) {
				x_si = UnitConverter::ft_To_M(x_us);
			}
			else {
				x_si = x_us;
			}
			double y_si = UnitConverter::psi_To_Pa(y_us);
			physicalVariables[1]->intervals[i]->points[j]->x = x_si;
			physicalVariables[1]->intervals[i]->points[j]->y = y_si;
		}
	}
	// Physical variable 2 (throttle setting)
	for (size_t i = 0; i <= (physicalVariables[2]->intervals.size() - 1); i++) {
		for (size_t j = 0; j <= (physicalVariables[2]->intervals[i]->points.size() - 1); j++) {
			double x_us = physicalVariables[2]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[2]->intervals[i]->points[j]->y;
			double x_si;
			if (DISTANCE_VS_TIME_INDICATOR == 0) {
				x_si = UnitConverter::ft_To_M(x_us);
			}
			else {
				x_si = x_us;
			}
			double y_si = y_us;
			physicalVariables[2]->intervals[i]->points[j]->x = x_si;
			physicalVariables[2]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[2]->intervals[i])->zvcb = false;
	}
	// Physical variable 3 (dynamic brake setting)
	for (size_t i = 0; i <= (physicalVariables[3]->intervals.size() - 1); i++) {
		for (size_t j = 0; j <= (physicalVariables[3]->intervals[i]->points.size() - 1); j++) {
			double x_us = physicalVariables[3]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[3]->intervals[i]->points[j]->y;
			double x_si;
			if (DISTANCE_VS_TIME_INDICATOR == 0) {
				x_si = UnitConverter::ft_To_M(x_us);
			}
			else {
				x_si = x_us;
			}
			double y_si = y_us;
			physicalVariables[3]->intervals[i]->points[j]->x = x_si;
			physicalVariables[3]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[3]->intervals[i])->zvcb = false;
	}
}


void LocomotiveOperator::definePhysicalVariableLimits() {
	// Track
	double t0_us = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[0]->intervals[0]->points[0]->x;
	int niml = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[0]->intervals.size();
	int npml = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[0]->intervals[niml - 1]->points.size();
	double tf_us = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[0]->intervals[niml - 1]->points[npml - 1]->x;
	// Physical variable independent component US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	// Index 3		--> Longitudinal position (feet)
	if (DISTANCE_VS_TIME_INDICATOR == 0) {
		PVISTR_US[0] = "feet";
		PVISTR_US[1] = "feet";
		PVISTR_US[2] = "feet";
		PVISTR_US[3] = "feet";
	}
	else {
		PVISTR_US[0] = "seconds";
		PVISTR_US[1] = "seconds";
		PVISTR_US[2] = "seconds";
		PVISTR_US[3] = "seconds";
	}
	// Physical variable dependent component US units
	// Index 0		-->	Automatic air brake setting (psi)
	// Index 1		-->	Independent air brake setting (psi)
	// Index 2		-->	Throttle setting (unitless)
	// Index 3		-->	Dynamic brake setting (unitless)
	PVDSTR_US[0] = "pounds per square inch";
	PVDSTR_US[1] = "pounds per square inch";
	PVDSTR_US[2] = "";
	PVDSTR_US[3] = "";
	// Physical variable dependent component integer boolean ('true' for integer; 'false' for double)
	// Index 0		-->	Automatic air brake setting
	// Index 1		-->	Independent air brake setting
	// Index 2		-->	Throttle setting
	// Index 3		-->	Dynamic brake setting
	PVDINT[0] = true;
	PVDINT[1] = true;
	PVDINT[2] = false;
	PVDINT[3] = false;
	// Physical variable independent component minimum threshold values in US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	// Index 3		-->	Longitudinal position (feet)
	if (DISTANCE_VS_TIME_INDICATOR == 0) {
		PVIMIN_US[0] = t0_us;
		PVIMIN_US[1] = t0_us;
		PVIMIN_US[2] = t0_us;
		PVIMIN_US[3] = t0_us;
	}
	else {
		PVIMIN_US[0] = 0.0;
		PVIMIN_US[1] = 0.0;
		PVIMIN_US[2] = 0.0;
		PVIMIN_US[3] = 0.0;
	}
	// Physical variable independent component minimum threshold values in SI units
	// Index 0		-->	Longitudinal position (meters)
	// Index 1		--> Longitudinal position (meters)
	// Index 2		-->	Longitudinal position (meters)
	// Index 3		-->	Longitudinal position (meters)
	if (DISTANCE_VS_TIME_INDICATOR == 0) {
		PVIMIN_SI[0] = UnitConverter::ft_To_M(PVIMIN_US[0]);
		PVIMIN_SI[1] = UnitConverter::ft_To_M(PVIMIN_US[1]);
		PVIMIN_SI[2] = UnitConverter::ft_To_M(PVIMIN_US[2]);
		PVIMIN_SI[3] = UnitConverter::ft_To_M(PVIMIN_US[3]);
	}
	else {
		PVIMIN_SI[0] = PVIMIN_US[0];
		PVIMIN_SI[1] = PVIMIN_US[1];
		PVIMIN_SI[2] = PVIMIN_US[2];
		PVIMIN_SI[3] = PVIMIN_US[3];
	}
	// Physical variable independent component maximum threshold values in US units
	// Index 0		-->	Longitudinal position (feet)
	// Index 1		--> Longitudinal position (feet)
	// Index 2		-->	Longitudinal position (feet)
	// Index 3		-->	Longitudinal position (feet)
	if (DISTANCE_VS_TIME_INDICATOR == 0) {
		PVIMAX_US[0] = tf_us;
		PVIMAX_US[1] = tf_us;
		PVIMAX_US[2] = tf_us;
		PVIMAX_US[3] = tf_us;
	}
	else {
		PVIMAX_US[0] = Simulation::MAX_NUMBER_OF_SIMULATED_SECONDS;
		PVIMAX_US[1] = Simulation::MAX_NUMBER_OF_SIMULATED_SECONDS;
		PVIMAX_US[2] = Simulation::MAX_NUMBER_OF_SIMULATED_SECONDS;
		PVIMAX_US[3] = Simulation::MAX_NUMBER_OF_SIMULATED_SECONDS;
	}
	// Physical variable independent component maximum threshold values in SI units
	// Index 0		-->	Longitudinal position (meters)
	// Index 1		--> Longitudinal position (meters)
	// Index 2		-->	Longitudinal position (meters)
	// Index 3		-->	Longitudinal position (meters)
	if (DISTANCE_VS_TIME_INDICATOR == 0) {
		PVIMAX_SI[0] = UnitConverter::ft_To_M(PVIMAX_US[0]);
		PVIMAX_SI[1] = UnitConverter::ft_To_M(PVIMAX_US[1]);
		PVIMAX_SI[2] = UnitConverter::ft_To_M(PVIMAX_US[2]);
		PVIMAX_SI[3] = UnitConverter::ft_To_M(PVIMAX_US[3]);
	}
	else {
		PVIMAX_SI[0] = PVIMAX_US[0];
		PVIMAX_SI[1] = PVIMAX_US[1];
		PVIMAX_SI[2] = PVIMAX_US[2];
		PVIMAX_SI[3] = PVIMAX_US[3];
	}
	// Physical variable dependent component minimum threshold values in US units
	// Index 0		-->	Automatic air brake setting (psi)
	// Index 1		-->	Independent air brake setting (psi)
	// Index 2		-->	Throttle setting (unitless)
	// Index 3		-->	Dynamic brake setting (unitless)
	PVDMIN_US[0] = round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE));  // 15 psi
	PVDMIN_US[1] = round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE));  // 15 psi
	PVDMIN_US[2] = 0.0;
	PVDMIN_US[3] = 0.0;
	// Physical variable dependent component minimum threshold values in SI units
	// Index 0		-->	Automatic air brake setting (pascals)
	// Index 1		-->	Independent air brake setting (pascals)
	// Index 2		-->	Throttle setting (unitless)
	// Index 3		-->	Dynamic brake setting (unitless)
	PVDMIN_SI[0] = UnitConverter::psi_To_Pa(PVDMIN_US[0]);
	PVDMIN_SI[1] = UnitConverter::psi_To_Pa(PVDMIN_US[1]);
	PVDMIN_SI[2] = PVDMIN_US[2];
	PVDMIN_SI[3] = PVDMIN_US[3];
	// Physical variable dependent component maximum threshold values in US units
	// Index 0		-->	Automatic air brake setting (psi)
	// Index 1		-->	Independent air brake setting (psi)
	// Index 2		-->	Throttle setting (unitless)
	// Index 3		-->	Dynamic brake setting (unitless)
	PVDMAX_US[0] = round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE));
	PVDMAX_US[1] = round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE));
	PVDMAX_US[2] = 1.0;
	PVDMAX_US[3] = 1.0;
	// Physical variable dependent component maximum threshold values in SI units
	// Index 0		-->	Automatic air brake setting (pascals)
	// Index 1		-->	Independent air brake setting (pascals)
	// Index 2		-->	Throttle notch (unitless)
	// Index 3		-->	Dynamic brake notch (unitless)
	PVDMAX_SI[0] = UnitConverter::psi_To_Pa(PVDMAX_US[0]);
	PVDMAX_SI[1] = UnitConverter::psi_To_Pa(PVDMAX_US[1]);
	PVDMAX_SI[2] = PVDMAX_US[2];
	PVDMAX_SI[3] = PVDMAX_US[3];
}


void LocomotiveOperator::loadPhysicalConstantAlternateNames() {
	DISTANCE_VS_TIME_INDICATOR = (int)physicalConstants[0];
}

