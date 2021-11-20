//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "ControlValve_Car.h"
#include "AuxiliaryReservoir.h"
#include "BrakeCylinder.h"
#include "Car.h"
#include "EmergencyReservoir.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "LocomotiveOperator.h"
#include "TrainConsist.h"


const double ControlValve_Car::AMAX_ar_bp = M_PI * pow((UnitConverter::in_To_M(0.125) / 2.0), 2.0);


const double ControlValve_Car::AMAX_er_bp = M_PI * pow((UnitConverter::in_To_M(0.125) / 2.0), 2.0);


const double ControlValve_Car::AMAX_bp_atm = M_PI * pow((UnitConverter::in_To_M(0.25) / 2.0), 2.0);


const double ControlValve_Car::AMAX_bc_ar = M_PI * pow((UnitConverter::in_To_M(0.125) / 2.0), 2.0);


const double ControlValve_Car::AMAX_bc_er = M_PI * pow((UnitConverter::in_To_M(0.125) / 2.0), 2.0);


const double ControlValve_Car::AMAX_bc_atm = M_PI * pow((UnitConverter::in_To_M(0.25) / 2.0), 2.0);


ControlValve_Car::ControlValve_Car(Car* car) : ControlValve() {
	this->car = car;
	currentOperatingMode = 0;
	int brakeCylinderPressureFunction_NumberDataPoints = 5;
	brakeCylinderPressureFunction_Domain = new double[brakeCylinderPressureFunction_NumberDataPoints];
	brakeCylinderPressureFunction_Domain[0] = TrainConsist::ATMOSPHERIC_PRESSURE;
	brakeCylinderPressureFunction_Domain[1] = car->brakeCylinder->maxPressureForFullServiceBraking - EBAMIT - UnitConverter::psi_To_Pa(1.0);
	brakeCylinderPressureFunction_Domain[2] = car->brakeCylinder->maxPressureForFullServiceBraking - EBAMIT;
	brakeCylinderPressureFunction_Domain[3] = car->brakeCylinder->maxPressureForFullServiceBraking;
	brakeCylinderPressureFunction_Domain[4] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
	brakeCylinderPressureFunction_Range = new double[brakeCylinderPressureFunction_NumberDataPoints];
	brakeCylinderPressureFunction_Range[0] = car->brakeCylinder->maxPressureForEmergencyBraking;
	brakeCylinderPressureFunction_Range[1] = car->brakeCylinder->maxPressureForEmergencyBraking;
	brakeCylinderPressureFunction_Range[2] = car->brakeCylinder->maxPressureForFullServiceBraking;
	brakeCylinderPressureFunction_Range[3] = car->brakeCylinder->maxPressureForFullServiceBraking;
	brakeCylinderPressureFunction_Range[4] = TrainConsist::ATMOSPHERIC_PRESSURE;
	calc_brakeCylinderPressureFunction(brakeCylinderPressureFunction_NumberDataPoints);
}


ControlValve_Car::~ControlValve_Car() {
	delete[] brakeCylinderPressureFunction_Domain;
	delete[] brakeCylinderPressureFunction_Range;
	delete brakeCylinderPressureFunction;
}


void ControlValve_Car::calc_currentOperatingMode() {
	if (currentOperatingMode != 3) {
		double input_signal;  // input signal
		input_signal = car->brakePipeAirPressure;
		double output_signal_1;  // output signal 1
		output_signal_1 = car->auxiliaryReservoir->pressure;
		double output_signal_2;  // output signal 2
		output_signal_2 = car->emergencyReservoir->pressure;
		if ((output_signal_1 - input_signal) > EBAMIT) {
			currentOperatingMode = 3;
			orificeBool_ar_bp = false;
			orificeBool_er_bp = false;
			orificeBool_bp_atm = true;
			orificeBool_bc_ar = true;
			orificeBool_bc_er = true;
			orificeBool_bc_atm = false;
		}
		else if ((output_signal_1 - input_signal) > SBAMIT) {
			currentOperatingMode = 1;
			orificeBool_ar_bp = false;
			orificeBool_er_bp = false;
			orificeBool_bp_atm = false;
			orificeBool_bc_ar = true;
			orificeBool_bc_er = false;
			orificeBool_bc_atm = false;
		}
		else if (((input_signal - output_signal_1) > SBRMIT) || ((input_signal - output_signal_2) > SBRMIT)) {
			currentOperatingMode = 2;
			orificeBool_ar_bp = true;
			orificeBool_er_bp = true;
			orificeBool_bp_atm = false;
			orificeBool_bc_ar = false;
			orificeBool_bc_er = false;
			orificeBool_bc_atm = true;
		}
		else if (((std::abs(output_signal_1 - input_signal) > LMIT) || (std::abs(output_signal_2 - input_signal) > LMIT)) &&
			(input_signal > (LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE - LMIT)) &&
			(input_signal < (LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE + LMIT))) {
			currentOperatingMode = 2;
			orificeBool_ar_bp = true;
			orificeBool_er_bp = true;
			orificeBool_bp_atm = false;
			orificeBool_bc_ar = false;
			orificeBool_bc_er = false;
			orificeBool_bc_atm = true;
		}
		else if (std::abs(output_signal_1 - input_signal) < LMIT) {
			currentOperatingMode = 0;
			orificeBool_ar_bp = false;
			orificeBool_er_bp = false;
			orificeBool_bp_atm = false;
			orificeBool_bc_ar = false;
			orificeBool_bc_er = false;
			orificeBool_bc_atm = false;
		}
		else {
			// do not change operating mode
		}
	}
	else {
		orificeBool_ar_bp = false;
		orificeBool_er_bp = false;
		orificeBool_bp_atm = true;
		orificeBool_bc_ar = true;
		orificeBool_bc_er = true;
		orificeBool_bc_atm = false;
	}
}


void ControlValve_Car::calc_mdot() {
	calc_mdot_ar_bp();
	calc_mdot_er_bp();
	calc_mdot_bp_atm();
	calc_mdot_bc_ar();
	calc_mdot_bc_er();
	calc_mdot_bc_atm();
	calc_mdot_ar();
	calc_mdot_er();
	calc_mdot_bc();
	calc_mdot_bp();
}


void ControlValve_Car::calc_mdot_ar() {
	mdot_ar = mdot_ar_bp - mdot_bc_ar;
}


void ControlValve_Car::calc_mdot_er() {
	mdot_er = mdot_er_bp - mdot_bc_er;
}


void ControlValve_Car::calc_mdot_bc() {
	mdot_bc = mdot_bc_ar + mdot_bc_er + mdot_bc_atm;
}


void ControlValve_Car::calc_mdot_bp() {
	mdot_bp = mdot_bp_atm - mdot_ar_bp - mdot_er_bp;
}


void ControlValve_Car::calc_mdot_ar_bp() {
	double airTemperature = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature;
	double orificeArea;
	if (orificeBool_ar_bp == true) {
		orificeArea = AMAX_ar_bp;
	}
	else {
		orificeArea = 0.0;
	}
	mdot_ar_bp = TrainConsist::massFlowRate(car->auxiliaryReservoir->pressure,
		car->brakePipeAirPressure,
		orificeArea, airTemperature);
}


void ControlValve_Car::calc_mdot_er_bp() {
	double airTemperature = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature;
	double orificeArea;
	if (orificeBool_er_bp == true) {
		orificeArea = AMAX_er_bp;
	}
	else {
		orificeArea = 0.0;
	}
	mdot_er_bp = TrainConsist::massFlowRate(car->emergencyReservoir->pressure,
		car->brakePipeAirPressure,
		orificeArea, airTemperature);
}


void ControlValve_Car::calc_mdot_bp_atm() {
	double airTemperature = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature;
	double orificeArea;
	if (orificeBool_bp_atm == true) {
		orificeArea = AMAX_bp_atm;
	}
	else {
		orificeArea = 0.0;
	}
	mdot_bp_atm = TrainConsist::massFlowRate(car->brakePipeAirPressure,
		TrainConsist::ATMOSPHERIC_PRESSURE,
		orificeArea, airTemperature);
}


void ControlValve_Car::calc_mdot_bc_ar() {
	double airTemperature = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature;
	double orificeArea;
	if (orificeBool_bc_ar == true) {
		orificeArea = AMAX_bc_ar;
	}
	else {
		orificeArea = 0.0;
	}
	mdot_bc_ar = TrainConsist::massFlowRate(car->brakeCylinder->pressure,
		car->auxiliaryReservoir->pressure,
		orificeArea, airTemperature);
}


void ControlValve_Car::calc_mdot_bc_er() {
	double airTemperature = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature;
	double orificeArea;
	if (orificeBool_bc_er == true) {
		orificeArea = AMAX_bc_er;
	}
	else {
		orificeArea = 0.0;
	}
	mdot_bc_er = TrainConsist::massFlowRate(car->brakeCylinder->pressure,
		car->emergencyReservoir->pressure,
		orificeArea, airTemperature);
}


void ControlValve_Car::calc_mdot_bc_atm() {
	double airTemperature = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature;
	double orificeArea;
	if (orificeBool_bc_atm == true) {
		orificeArea = AMAX_bc_atm;
	}
	else {
		orificeArea = 0.0;
	}
	mdot_bc_atm = TrainConsist::massFlowRate(car->brakeCylinder->pressure,
		TrainConsist::ATMOSPHERIC_PRESSURE,
		orificeArea, airTemperature);
}

