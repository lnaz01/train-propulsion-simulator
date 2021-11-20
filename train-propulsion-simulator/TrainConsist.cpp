//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "TrainConsist.h"
#include "BrakePipe_Cumulative.h"
#include "BrakePipe_FiniteElement.h"
#include "Car.h"
#include "Coupler.h"
#include "CouplingSystem.h"
#include "Function.h"
#include "Interval.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "RailVehicle.h"
#include "UnitConverter.h"


const double TrainConsist::ATMOSPHERIC_PRESSURE = UnitConverter::psi_To_Pa(15.0);


TrainConsist::TrainConsist(InputFileReader_Simulation* ifrsim) : UserDefinedRRComponent(ifrsim, 2, 0) {
	componentType = 5;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables.push_back(new Function(this, i));
	}
	railVehicleTypesDefinedBool = false;
	// User-inputted physical constants
	// 0	-->	Air temperature (fahrenheit)
	// 1	-->	End-of-train device capability
	START_UDRRC_STRING = "TrainConsist_";
	END_UDRRC_STRING = "_TrainConsist";
	// Physical constants US units
	// Index 0	-->	Air temperature (fahrenheit)
	// Index 1	-->	One-way ('1') or two-way ('2') end-of-train device
	PCSTR_US[0] = "degrees fahrenheit";
	PCSTR_US[1] = "";
	// Physical constants integer boolean
	// Index 0	-->	Air temperature
	// Index 1	-->	One-way ('1') or two-way ('2') end-of-train device
	PCINT[0] = false;
	PCINT[1] = true;
	// Physical constant minimum threshold values in US units
	// Index 0	-->	Air temperature (fahrenheit)
	// Index 1	-->	One-way ('1') or two-way ('2') end-of-train device
	PCMIN_US[0] = -40.0;
	PCMIN_US[1] = 1.0;
	// Physical constant minimum threshold values in SI units
	// Index 0	-->	Air temperature (kelvin)
	// Index 1	-->	One-way ('1') or two-way ('2') end-of-train device
	PCMIN_SI[0] = UnitConverter::f_To_K(PCMIN_US[0]);
	PCMIN_SI[1] = PCMIN_US[1];
	// Physical constant maximum threshold values in US units
	// Index 0	-->	Air temperature (fahrenheit)
	// Index 1	-->	One-way ('1') or two-way ('2') end-of-train device
	PCMAX_US[0] = 140.0;
	PCMAX_US[1] = 2.0;
	// Physical constant maximum threshold values in SI units
	// Index 0	-->	Air temperature (kelvin)
	// Index 1	-->	One-way ('1') or two-way ('2') end-of-train device
	PCMAX_SI[0] = UnitConverter::f_To_K(PCMAX_US[0]);
	PCMAX_SI[1] = PCMAX_US[1];
}


TrainConsist::~TrainConsist() {
	for (size_t i = 0; i < railVehicles.size(); i++) {
		delete railVehicles[i];
	}
	for (size_t i = 0; i < couplingSystems.size(); i++) {
		delete couplingSystems[i];
	}
	for (size_t i = 0; i < brakePipes_Cumulative.size(); i++) {
		delete brakePipes_Cumulative[i];
	}
	if (railVehicleTypesDefinedBool == true) {
		delete[] railVehicleTypes;
	}
}


double TrainConsist::massFlowRate(double Pexp, double Pup, double restrictionArea, double airTemperature) {
	double rEXP = Pexp / Pup;
	double mdot = 0.6 * restrictionArea * Pup * pow(std::abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemperature), 0.5);
	if (Pexp < Pup) {
		return mdot;
	}
	else if (Pexp > Pup) {
		return -mdot;
	}
	else {
		return 0.0;
	}
}


void TrainConsist::calc_railVehicleTypes() {
	if (railVehicleTypesDefinedBool == true) {
		delete[] railVehicleTypes;
	}
	else {
		railVehicleTypesDefinedBool = true;
	}
	railVehicleTypes = new int[railVehicles.size()];
	for (size_t i = 0; i < railVehicles.size(); i++) {
		if (railVehicles[i]->componentType == 2) {
			railVehicleTypes[i] = 0;
		}
		else {
			railVehicleTypes[i] = 1;
		}
	}
}


void TrainConsist::calc_locationOnTrack() {
	locationOnTrack = railVehicles[0]->ssv[0];
}


void TrainConsist::add_railVehicle(RailVehicle* railVehicle) {
	railVehicles.push_back(railVehicle);
	railVehicles[railVehicles.size() - 1]->positionInTrainConsist = railVehicles.size() - 1;
	railVehicles[railVehicles.size() - 1]->couplers[0]->railVehicle = railVehicles[railVehicles.size() - 1];
	railVehicles[railVehicles.size() - 1]->couplers[1]->railVehicle = railVehicles[railVehicles.size() - 1];
	if (railVehicles.size() > 1) {
		couplingSystems.push_back(new CouplingSystem(railVehicles[railVehicles.size() - 2]->couplers[1], railVehicles[railVehicles.size() - 1]->couplers[0]));
		railVehicles[railVehicles.size() - 2]->couplers[1]->couplingSystem = couplingSystems[couplingSystems.size() - 1];
		railVehicles[railVehicles.size() - 1]->couplers[0]->couplingSystem = couplingSystems[couplingSystems.size() - 1];
	}
}


void TrainConsist::calc_locomotives() {
	locomotives.clear();
	for (size_t i = 0; i <= (railVehicles.size() - 1); i++) {
		if (railVehicleTypes[i] == 1) {
			locomotives.push_back((Locomotive*)railVehicles[i]);
		}
	}
}


void TrainConsist::calc_brakePipes_Cumulative() {
	brakePipes_Cumulative.clear();
	bool cumulativeBrakePipeConstructionStarted = false;
	for (size_t i = 0; i < railVehicles.size(); i++) {
		if (i < railVehicles.size() - 1) {
			if (railVehicleTypes[i] == 1) {
				if (cumulativeBrakePipeConstructionStarted == true) {
					brakePipes_Cumulative[brakePipes_Cumulative.size() - 1]->railVehicles.push_back(railVehicles[i]);
					cumulativeBrakePipeConstructionStarted = false;
				}
				if (railVehicleTypes[i + 1] == 1) {
					// do nothing
				}
				else {
					brakePipes_Cumulative.push_back(new BrakePipe_Cumulative(this));
					brakePipes_Cumulative[brakePipes_Cumulative.size() - 1]->railVehicles.push_back(railVehicles[i]);
					cumulativeBrakePipeConstructionStarted = true;
				}
			}
			else {
				if (cumulativeBrakePipeConstructionStarted == false) {
					brakePipes_Cumulative.push_back(new BrakePipe_Cumulative(this));
					brakePipes_Cumulative[brakePipes_Cumulative.size() - 1]->railVehicles.push_back(railVehicles[i]);
					cumulativeBrakePipeConstructionStarted = true;
				}
				else {
					brakePipes_Cumulative[brakePipes_Cumulative.size() - 1]->railVehicles.push_back(railVehicles[i]);
				}
			}
		}
		else {
			if (cumulativeBrakePipeConstructionStarted == true) {
				brakePipes_Cumulative[brakePipes_Cumulative.size() - 1]->railVehicles.push_back(railVehicles[i]);
			}
			else {
				// do nothing
			}
		}
	}
}


void TrainConsist::calculateInitialPressuresAndMValuesForBPFENodes() {
	for (size_t i = 0; i < brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			int numRailVehicles = brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->railVehicles.size();
			int railVehicleIndices[BrakePipe_FiniteElement::NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
			railVehicleIndices[0] = 0;
			railVehicleIndices[1] = numRailVehicles - 1;
			for (int k = 0; k < BrakePipe_FiniteElement::NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; k++) {
				if (brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->railVehicleTypes[railVehicleIndices[k]] == 1) {
					double currentAutomaticBrakeValveSetting = ((Locomotive*)brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->railVehicles[railVehicleIndices[k]])->currentAutomaticBrakeValveSetting;
					brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->nodePressures[k] = currentAutomaticBrakeValveSetting;
					brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->nodeMValues[k] = 0.0;
				}
				else {
					double brakePipePressure = brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->railVehicles[railVehicleIndices[k]]->brakePipeAirPressure;
					brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->nodePressures[k] = brakePipePressure;
					brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->nodeMValues[k] = 0.0;
				}
			}
		}
	}
}


void TrainConsist::calc_airViscosity() {
	// (-50 F) <= airtemp < (-20 F)
	if ((airTemperature >= UnitConverter::f_To_K(-50.0)) && (airTemperature < UnitConverter::f_To_K(-20.0))) {
		airViscosity = ((14.80 * pow(10.0, -6.0)) + (15.68 * pow(10.0, -6.0))) / 2.0;
	}
	// (-20 F) <= airtemp < (0 F)
	else if ((airTemperature >= UnitConverter::f_To_K(-20.0)) && (airTemperature < UnitConverter::f_To_K(0.0))) {
		airViscosity = ((15.68 * pow(10.0, -6.0)) + (3.38 * pow(16.25, -6.0))) / 2.0;
	}
	// (0 F) <= airtemp < (10 F)
	else if ((airTemperature >= UnitConverter::f_To_K(0.0)) && (airTemperature < UnitConverter::f_To_K(10.0))) {
		airViscosity = ((16.25 * pow(10.0, -6.0)) + (16.54 * pow(10.0, -6.0))) / 2.0;
	}
	// (10 F) <= airtemp < (20 F)
	else if ((airTemperature >= UnitConverter::f_To_K(10.0)) && (airTemperature < UnitConverter::f_To_K(20.0))) {
		airViscosity = ((16.54 * pow(10.0, -6.0)) + (16.82 * pow(10.0, -6.0))) / 2.0;
	}
	// (20 F) <= airtemp < (30 F)
	else if ((airTemperature >= UnitConverter::f_To_K(20.0)) && (airTemperature < UnitConverter::f_To_K(30.0))) {
		airViscosity = ((16.82 * pow(10.0, -6.0)) + (17.10 * pow(10.0, -6.0))) / 2.0;
	}
	// (30 F) <= airtemp < (40 F)
	else if ((airTemperature >= UnitConverter::f_To_K(30.0)) && (airTemperature < UnitConverter::f_To_K(40.0))) {
		airViscosity = ((17.10 * pow(10.0, -6.0)) + (17.37 * pow(10.0, -6.0))) / 2.0;
	}
	// (40 F) <= airtemp < (50 F)
	else if ((airTemperature >= UnitConverter::f_To_K(40.0)) && (airTemperature < UnitConverter::f_To_K(50.0))) {
		airViscosity = ((17.37 * pow(10.0, -6.0)) + (17.64 * pow(10.0, -6.0))) / 2.0;
	}
	// (50 F) <= airtemp < (60 F)
	else if ((airTemperature >= UnitConverter::f_To_K(50.0)) && (airTemperature < UnitConverter::f_To_K(60.0))) {
		airViscosity = ((17.64 * pow(10.0, -6.0)) + (17.91 * pow(10.0, -6.0))) / 2.0;
	}
	// (60 F) <= airtemp < (70 F)
	else if ((airTemperature >= UnitConverter::f_To_K(60.0)) && (airTemperature < UnitConverter::f_To_K(70.0))) {
		airViscosity = ((17.91 * pow(10.0, -6.0)) + (18.18 * pow(10.0, -6.0))) / 2.0;
	}
	// (70 F) <= airtemp < (80 F)
	else if ((airTemperature >= UnitConverter::f_To_K(70.0)) && (airTemperature < UnitConverter::f_To_K(80.0))) {
		airViscosity = ((18.18 * pow(10.0, -6.0)) + (18.45 * pow(10.0, -6.0))) / 2.0;
	}
	// (80 F) <= airtemp < (100 F)
	else if ((airTemperature >= UnitConverter::f_To_K(80.0)) && (airTemperature < UnitConverter::f_To_K(100.0))) {
		airViscosity = ((18.45 * pow(10.0, -6.0)) + (18.97 * pow(10.0, -6.0))) / 2.0;
	}
	// (100 F) <= airtemp < (120 F)
	else if ((airTemperature >= UnitConverter::f_To_K(100.0)) && (airTemperature < UnitConverter::f_To_K(120.0))) {
		airViscosity = ((18.97 * pow(10.0, -6.0)) + (19.48 * pow(10.0, -6.0))) / 2.0;
	}
	// (120 F) <= airtemp <= (140 F)
	else if ((airTemperature >= UnitConverter::f_To_K(120.0)) && (airTemperature <= UnitConverter::f_To_K(140.0))) {
		airViscosity = ((19.48 * pow(10.0, -6.0)) + (19.99 * pow(10.0, -6.0))) / 2.0;
	}
}


void TrainConsist::configureCouplingSystems() {
	for (size_t i = 0; i < couplingSystems.size(); i++) {
		// Damping
		couplingSystems[i]->calc_effectiveDampingConstant();
		// Unstressed center-to-center distance
		couplingSystems[i]->calc_centerToCenterDistance_Unstressed();
		// Initial coupler displacements
		couplingSystems[i]->initialCouplerDisplacements();
	}
}


void TrainConsist::convertToSI() {
	// Physical constants
	double pcs_us;
	double pcs_si;
	// Physical constant 0 -- air temperature
	pcs_us = physicalConstants[0];
	pcs_si = UnitConverter::f_To_K(pcs_us);
	physicalConstants[0] = pcs_si;
	// Physical constant 1 -- end-of-train device (one-way vs two-way)
	pcs_us = physicalConstants[1];
	pcs_si = pcs_us;
	physicalConstants[1] = pcs_si;
	// Load physical constants alternative names
	loadPhysicalConstantAlternateNames();
	// Rail vehicles
	for (size_t i = 0; i <= (railVehicles.size() - 1); i++) {
		if (railVehicleTypes[i] == 0) {
			((Car*)railVehicles[i])->convertToSI();
		}
		else {
			((Locomotive*)railVehicles[i])->convertToSI();
			((Locomotive*)railVehicles[i])->locomotiveOperator->convertToSI();
		}
	}
}


void TrainConsist::loadPhysicalConstantAlternateNames() {
	airTemperature = physicalConstants[0];
	eotDeviceCapability = (int)physicalConstants[1];
}

