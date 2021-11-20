//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "BrakePipe_FiniteElement.h"
#include "BrakePipe_Cumulative.h"
#include "Car.h"
#include "ControlValve_Car.h"
#include "TrainConsist.h"
#include "UnitConverter.h"


const double BrakePipe_FiniteElement::DIAMETER = UnitConverter::in_To_M(1.25);


BrakePipe_FiniteElement::BrakePipe_FiniteElement(BrakePipe_Cumulative* brakePipe_Cumulative, int brakePipeFEIndex) {
	this->brakePipe_Cumulative = brakePipe_Cumulative;
	this->brakePipeFEIndex = brakePipeFEIndex;
	railVehicleTypesDefinedBool = false;
	railVehicleXLocationsDefinedBool = false;
	nodeDiameters = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeAreas = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodePressures = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeMValues = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeDensities = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeVelocities = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeCValues = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeReynoldsNumbers = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
	nodeWallFrictionFactors = new double[NUMBER_OF_NODES_PER_BRAKE_PIPE_FE];
}


BrakePipe_FiniteElement::~BrakePipe_FiniteElement() {
	delete[] nodeDiameters;
	delete[] nodeAreas;
	delete[] nodePressures;
	delete[] nodeMValues;
	delete[] nodeDensities;
	delete[] nodeVelocities;
	delete[] nodeCValues;
	delete[] nodeReynoldsNumbers;
	delete[] nodeWallFrictionFactors;
	if (railVehicleTypesDefinedBool == true) {
		delete[] railVehicleTypes;
	}
	if (railVehicleXLocationsDefinedBool == true) {
		delete[] railVehicleXLocations;
	}
}


void BrakePipe_FiniteElement::calc_leakage() {
	double mdot_bp = 0.0;
	for (size_t k = 0; k < railVehicles.size(); k++) {
		if (railVehicleTypes[k] == 0) {
			mdot_bp = mdot_bp + ((ControlValve_Car*)((Car*)railVehicles[k])->controlValve)->mdot_bp;
		}
	}
	leakage = mdot_bp;
}


void BrakePipe_FiniteElement::calc_railVehicleTypes() {
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


void BrakePipe_FiniteElement::calc_brakePipeFELength() {
	double xCumulativeDistance = 0.0;
	for (size_t i = 0; i < railVehicles.size(); i++) {
		if (i == 0) {
			if (railVehicleTypes[i] == 0) {
				xCumulativeDistance = railVehicles[i]->brakePipeLength;
			}
			else {
				xCumulativeDistance = railVehicles[i]->brakePipeLength / 2.0;
			}
		}
		else if (i == railVehicles.size() - 1) {
			if (railVehicleTypes[i] == 0) {
				xCumulativeDistance = xCumulativeDistance + railVehicles[i]->brakePipeLength;
			}
			else {
				xCumulativeDistance = xCumulativeDistance + (railVehicles[i]->brakePipeLength / 2.0);
			}
		}
		else {
			xCumulativeDistance = xCumulativeDistance + railVehicles[i]->brakePipeLength;
		}
	}
	brakePipeFELength = xCumulativeDistance;
}


void BrakePipe_FiniteElement::calc_railVehicleXLocations() {
	if (railVehicleXLocationsDefinedBool == true) {
		delete[] railVehicleXLocations;
	}
	else {
		railVehicleXLocationsDefinedBool = true;
	}
	railVehicleXLocations = new double[railVehicles.size()];
	double xCumulativeDistance = 0.0;
	for (size_t i = 0; i < railVehicles.size(); i++) {
		if (i == 0) {
			if (railVehicleTypes[i] == 0) {
				xCumulativeDistance = railVehicles[i]->brakePipeLength / 2.0;
			}
			else {
				xCumulativeDistance = 0.0;
			}
			railVehicleXLocations[i] = xCumulativeDistance;
			xCumulativeDistance = xCumulativeDistance + (railVehicles[i]->brakePipeLength / 2.0);
		}
		else if (i == railVehicles.size() - 1) {
			if (railVehicleTypes[i] == 1) {
				xCumulativeDistance = brakePipeFELength;
			}
			else {
				xCumulativeDistance = xCumulativeDistance + (railVehicles[i]->brakePipeLength / 2.0);
			}
			railVehicleXLocations[i] = xCumulativeDistance;
		}
		else {
			xCumulativeDistance = xCumulativeDistance + (railVehicles[i]->brakePipeLength / 2.0);
			railVehicleXLocations[i] = xCumulativeDistance;
			xCumulativeDistance = xCumulativeDistance + (railVehicles[i]->brakePipeLength / 2.0);
		}
	}
}


void BrakePipe_FiniteElement::calc_nodeVelocities() {
	for (int i = 0; i < NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; i++) {
		if (nodeDensities[i] != 0.0) {
			nodeVelocities[i] = nodeMValues[i] / nodeDensities[i];
		}
		else {
			nodeVelocities[i] = 0.0;
		}
	}
}


void BrakePipe_FiniteElement::calc_nodeDensities() {
	for (int i = 0; i < NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; i++) {
		nodeDensities[i] = nodePressures[i] / (TrainConsist::GAS_CONSTANT_AIR * brakePipe_Cumulative->trainConsist->airTemperature);
	}
}


void BrakePipe_FiniteElement::calc_nodeAreas() {
	for (int i = 0; i < NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; i++) {
		nodeAreas[i] = M_PI * pow((nodeDiameters[i] / 2.0), 2.0);
	}
}


void BrakePipe_FiniteElement::calc_nodeCValues() {
	for (int i = 0; i < NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; i++) {
		if (nodeVelocities[i] == 0.0) {
			nodeCValues[i] = 0.0;
		}
		else {
			nodeCValues[i] = nodeWallFrictionFactors[i] * ((nodeDensities[i] * pow(nodeVelocities[i], 2.0)) / 8.0) * (nodeVelocities[i] / std::abs(nodeVelocities[i])) * M_PI * effDiameter;
		}
	}
}


void BrakePipe_FiniteElement::calc_effCValue() {
	effCValue = (nodeCValues[0] + nodeCValues[1]) / 2.0;
}


void BrakePipe_FiniteElement::calc_effDiameter() {
	effDiameter = (2.0 * nodeDiameters[0] * nodeDiameters[1]) / (nodeDiameters[0] + nodeDiameters[1]);
}


void BrakePipe_FiniteElement::calc_effArea() {
	effArea = (2.0 * nodeAreas[0] * nodeAreas[1]) / (nodeAreas[0] + nodeAreas[1]);
}


void BrakePipe_FiniteElement::calc_nodeReynoldsNumbers() {
	for (int i = 0; i < NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; i++) {
		nodeReynoldsNumbers[i] = abs((nodeDensities[i] * std::abs(nodeVelocities[i]) * nodeDiameters[i]) / brakePipe_Cumulative->trainConsist->airViscosity);
		if (nodeReynoldsNumbers[i] < 0.1) {
			nodeReynoldsNumbers[i] = 0.1;
		}
	}
}


void BrakePipe_FiniteElement::calc_nodeWallFrictionFactors() {
	for (int i = 0; i < NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; i++) {
		double a = 0.0;
		double b = 0.0;
		if ((nodeReynoldsNumbers[i] >= 0.0) && (nodeReynoldsNumbers[i] <= 2000.0)) {
			a = 64.00;
			b = -1.00;
		}
		else if ((nodeReynoldsNumbers[i] > 2000.0) && (nodeReynoldsNumbers[i] <= 4000.0)) {
			a = 0.000137;
			b = 0.717;
		}
		else if ((nodeReynoldsNumbers[i] > 4000.0) && (nodeReynoldsNumbers[i] <= 40000.0)) {
			a = 0.13977;
			b = -0.11781;
		}
		else {
			a = 0.04;
			b = 0.0;
		}
		nodeWallFrictionFactors[i] = a * pow(nodeReynoldsNumbers[i], b);
	}
}


void BrakePipe_FiniteElement::calculateBrakePipePressureForRailVehicles() {
	for (size_t i = 0; i < railVehicles.size(); i++) {
		railVehicles[i]->brakePipeAirPressure = calculateInterpolatedValue(railVehicleXLocations[i], true);
	}
}


double BrakePipe_FiniteElement::calculateInterpolatedValue(double x, bool pressureBool) {
	double X1 = 0;
	double X2 = brakePipeFELength;
	double S1 = (X2 - x) / brakePipeFELength;
	double S2 = (x - X1) / brakePipeFELength;
	double interpolatedValue;
	if (pressureBool == true) {
		interpolatedValue = (nodePressures[0] * S1) + (nodePressures[1] * S2);
	}
	else {
		interpolatedValue = (nodeMValues[0] * S1) + (nodeMValues[1] * S2);
	}
	return interpolatedValue;
}

