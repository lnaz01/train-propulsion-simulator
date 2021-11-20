//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "BrakePipe_Cumulative.h"
#include "BrakePipe_FiniteElement.h"
#include "Car.h"
#include "EndOfTrainDevice.h"
#include "InputFileReader_Simulation.h"
#include "LinearSystem.h"
#include "Locomotive.h"
#include "Simulation.h"
#include "TrainConsist.h"


BrakePipe_Cumulative::BrakePipe_Cumulative(TrainConsist* trainConsist) {
	this->trainConsist = trainConsist;
	railVehicleTypesDefinedBool = false;
	linearSystem = new LinearSystem();
	systemMatrixAndForcingVectorBoolean = false;
}


BrakePipe_Cumulative::~BrakePipe_Cumulative() {
	for (size_t i = 0; i < brakePipe_FiniteElements.size(); i++) {
		delete brakePipe_FiniteElements[i];
	}
	if (systemMatrixAndForcingVectorBoolean == true) {
		for (int i = 0; i < systemMatrixDim; i++) {
			delete[] systemMatrix[i];
		}
		delete[] systemMatrix;
		delete[] forcingVector;
	}
	if (railVehicleTypesDefinedBool == true) {
		delete[] railVehicleTypes;
	}
}


void BrakePipe_Cumulative::calc_railVehicleTypes() {
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


void BrakePipe_Cumulative::initializeSystemMatrixAndForcingVector() {
	// Allocate space for global system matrix
	systemMatrixDim = 2 * (brakePipe_FiniteElements.size() + 1);
	systemMatrix = new double* [systemMatrixDim];
	for (int i = 0; i < systemMatrixDim; i++) {
		systemMatrix[i] = new double[systemMatrixDim];
	}
	// Allocate space for global forcing vector
	forcingVector = new double[systemMatrixDim];
	// Update 'systemMatrixAndForcingVectorBoolean' variable
	systemMatrixAndForcingVectorBoolean = true;
}


void BrakePipe_Cumulative::calc_brakePipe_FiniteElements() {
	int numberOfRailVehicles = (railVehicles[railVehicles.size() - 1]->positionInTrainConsist - railVehicles[0]->positionInTrainConsist) + 1;
	int remainingNumberOfRailVehicles = numberOfRailVehicles;
	int railVehiclesUsed = 0;
	int numberOfFiniteElements = 0;
	do {
		if ((numberOfRailVehicles <= BrakePipe_FiniteElement::MAX_RAIL_VEHICLES_PER_BRAKE_PIPE_FE) ||
			(remainingNumberOfRailVehicles > (2 * BrakePipe_FiniteElement::MAX_RAIL_VEHICLES_PER_BRAKE_PIPE_FE))) {
			int FESize;
			if (numberOfRailVehicles <= BrakePipe_FiniteElement::MAX_RAIL_VEHICLES_PER_BRAKE_PIPE_FE) {
				FESize = numberOfRailVehicles;
			}
			else {
				FESize = BrakePipe_FiniteElement::MAX_RAIL_VEHICLES_PER_BRAKE_PIPE_FE;
			}
			numberOfFiniteElements++;
			brakePipe_FiniteElements.push_back(new BrakePipe_FiniteElement(this, numberOfFiniteElements - 1));
			int i_upper_limit = railVehiclesUsed + FESize;
			for (int i = railVehiclesUsed; i < i_upper_limit; i++) {
				railVehiclesUsed++;
				brakePipe_FiniteElements[numberOfFiniteElements - 1]->railVehicles.push_back(railVehicles[i]);
			}
			remainingNumberOfRailVehicles = numberOfRailVehicles - railVehiclesUsed;
		}
		else {
			int FESizes[2];
			if (remainingNumberOfRailVehicles % 2 == 0) {
				FESizes[0] = remainingNumberOfRailVehicles / 2;
				FESizes[1] = remainingNumberOfRailVehicles / 2;
			}
			else {
				FESizes[0] = (int)ceil((double)remainingNumberOfRailVehicles / 2);
				FESizes[1] = (int)floor((double)remainingNumberOfRailVehicles / 2);
			}
			for (int j = 0; j < 2; j++) {
				numberOfFiniteElements++;
				brakePipe_FiniteElements.push_back(new BrakePipe_FiniteElement(this, numberOfFiniteElements - 1));
				int i_upper_limit = railVehiclesUsed + FESizes[j];
				for (int i = railVehiclesUsed; i < i_upper_limit; i++) {
					railVehiclesUsed++;
					brakePipe_FiniteElements[numberOfFiniteElements - 1]->railVehicles.push_back(railVehicles[i]);
				}
				remainingNumberOfRailVehicles = numberOfRailVehicles - railVehiclesUsed;
			}
		}
	} while (remainingNumberOfRailVehicles > 0);
}


void BrakePipe_Cumulative::calc_systemMatrix() {
	for (int i = 0; i < systemMatrixDim; i++) {
		for (int j = 0; j < systemMatrixDim; j++) {
			systemMatrix[i][j] = 0.0;
		}
	}
	double timeStep = trainConsist->inputFileReader_Simulation->userDefinedSimulations[0]->IMPLICIT_SOLVER_FIXED_TIME_STEP;
	for (int i = 0; i < systemMatrixDim; i++) {
		if (i == 0) {
			if (brakePipe_FiniteElements[0]->railVehicleTypes[0] == 1) {
				systemMatrix[i][0] = 1.0;
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					systemMatrix[i][0] = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 3.0);
					systemMatrix[i][1] = -(brakePipe_FiniteElements[0]->nodeAreas[0] / 2.0);
					systemMatrix[i][2] = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 6.0);
					systemMatrix[i][3] = brakePipe_FiniteElements[0]->nodeAreas[1] / 2.0;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					systemMatrix[i][0] = 1.0;
				}
			}
		}
		else if (i == 1) {
			if (brakePipe_FiniteElements[0]->railVehicleTypes[0] == 1) {
				systemMatrix[i][0] = -(brakePipe_FiniteElements[0]->effArea / 2.0);
				systemMatrix[i][1] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 3.0)) - ((brakePipe_FiniteElements[0]->nodeVelocities[0] * brakePipe_FiniteElements[0]->nodeAreas[0]) / 2.0);
				systemMatrix[i][2] = brakePipe_FiniteElements[0]->effArea / 2.0;
				systemMatrix[i][3] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 6.0)) + ((brakePipe_FiniteElements[0]->nodeVelocities[1] * brakePipe_FiniteElements[0]->nodeAreas[1]) / 2.0);
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					systemMatrix[i][1] = 1.0;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					systemMatrix[i][0] = -(brakePipe_FiniteElements[0]->effArea / 2.0);
					systemMatrix[i][1] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 3.0)) - ((brakePipe_FiniteElements[0]->nodeVelocities[0] * brakePipe_FiniteElements[0]->nodeAreas[0]) / 2.0);
					systemMatrix[i][2] = brakePipe_FiniteElements[0]->effArea / 2.0;
					systemMatrix[i][3] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 6.0)) + ((brakePipe_FiniteElements[0]->nodeVelocities[1] * brakePipe_FiniteElements[0]->nodeAreas[1]) / 2.0);
				}
			}
		}
		else if (i == systemMatrixDim - 2) {
			int numFE = brakePipe_FiniteElements.size();
			int numRV = brakePipe_FiniteElements[numFE - 1]->railVehicles.size();
			if (brakePipe_FiniteElements[numFE - 1]->railVehicleTypes[numRV - 1] == 1) {
				systemMatrix[i][systemMatrixDim - 2] = 1.0;
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[numFE - 1]->railVehicles[numRV - 1])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					systemMatrix[i][systemMatrixDim - 4] = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 6.0);
					systemMatrix[i][systemMatrixDim - 3] = -(brakePipe_FiniteElements[numFE - 1]->nodeAreas[0] / 2.0);
					systemMatrix[i][systemMatrixDim - 2] = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 3.0);
					systemMatrix[i][systemMatrixDim - 1] = brakePipe_FiniteElements[numFE - 1]->nodeAreas[1] / 2.0;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					systemMatrix[i][systemMatrixDim - 2] = 1.0;
				}
			}
		}
		else if (i == systemMatrixDim - 1) {
			int numFE = brakePipe_FiniteElements.size();
			int numRV = brakePipe_FiniteElements[numFE - 1]->railVehicles.size();
			if (brakePipe_FiniteElements[numFE - 1]->railVehicleTypes[numRV - 1] == 1) {
				systemMatrix[i][systemMatrixDim - 4] = -(brakePipe_FiniteElements[numFE - 1]->effArea / 2.0);
				systemMatrix[i][systemMatrixDim - 3] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 6.0)) - ((brakePipe_FiniteElements[numFE - 1]->nodeVelocities[0] * brakePipe_FiniteElements[numFE - 1]->nodeAreas[0]) / 2.0);
				systemMatrix[i][systemMatrixDim - 2] = brakePipe_FiniteElements[numFE - 1]->effArea / 2.0;
				systemMatrix[i][systemMatrixDim - 1] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 3.0)) + ((brakePipe_FiniteElements[numFE - 1]->nodeVelocities[1] * brakePipe_FiniteElements[numFE - 1]->nodeAreas[1]) / 2.0);
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[numFE - 1]->railVehicles[numRV - 1])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					systemMatrix[i][systemMatrixDim - 1] = 1.0;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					systemMatrix[i][systemMatrixDim - 4] = -(brakePipe_FiniteElements[numFE - 1]->effArea / 2.0);
					systemMatrix[i][systemMatrixDim - 3] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 6.0)) - ((brakePipe_FiniteElements[numFE - 1]->nodeVelocities[0] * brakePipe_FiniteElements[numFE - 1]->nodeAreas[0]) / 2.0);
					systemMatrix[i][systemMatrixDim - 2] = brakePipe_FiniteElements[numFE - 1]->effArea / 2.0;
					systemMatrix[i][systemMatrixDim - 1] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 3.0)) + ((brakePipe_FiniteElements[numFE - 1]->nodeVelocities[1] * brakePipe_FiniteElements[numFE - 1]->nodeAreas[1]) / 2.0);
				}
			}
		}
		else {
			int currFE1 = (int)floor(i / 2) - 1;
			int currFE2 = (int)floor(i / 2);
			if (i % 2 == 0) {
				systemMatrix[i][i - 2] = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 6.0);
				systemMatrix[i][i - 1] = -(brakePipe_FiniteElements[currFE1]->nodeAreas[0] / 2.0);
				systemMatrix[i][i] = ((1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 3.0)) + ((1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 3.0));
				systemMatrix[i][i + 1] = (brakePipe_FiniteElements[currFE1]->nodeAreas[1] / 2.0) - (brakePipe_FiniteElements[currFE2]->nodeAreas[0] / 2.0);
				systemMatrix[i][i + 2] = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 6.0);
				systemMatrix[i][i + 3] = brakePipe_FiniteElements[currFE2]->nodeAreas[1] / 2.0;
			}
			else {
				systemMatrix[i][i - 3] = -(brakePipe_FiniteElements[currFE1]->effArea / 2.0);
				systemMatrix[i][i - 2] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 6.0)) - ((brakePipe_FiniteElements[currFE1]->nodeVelocities[0] * brakePipe_FiniteElements[currFE1]->nodeAreas[0]) / 2.0);
				systemMatrix[i][i - 1] = (brakePipe_FiniteElements[currFE1]->effArea / 2.0) - (brakePipe_FiniteElements[currFE2]->effArea / 2.0);
				systemMatrix[i][i] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 3.0)) + ((1.0 / timeStep) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 3.0)) + ((brakePipe_FiniteElements[currFE1]->nodeVelocities[1] * brakePipe_FiniteElements[currFE1]->nodeAreas[1]) / 2.0) - ((brakePipe_FiniteElements[currFE2]->nodeVelocities[0] * brakePipe_FiniteElements[currFE2]->nodeAreas[0]) / 2.0);
				systemMatrix[i][i + 1] = brakePipe_FiniteElements[currFE2]->effArea / 2.0;
				systemMatrix[i][i + 2] = ((1.0 / timeStep) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 6.0)) + ((brakePipe_FiniteElements[currFE2]->nodeVelocities[1] * brakePipe_FiniteElements[currFE2]->nodeAreas[1]) / 2.0);
			}
		}
	}
}


void BrakePipe_Cumulative::calc_forcingVector() {
	double timeStep = trainConsist->inputFileReader_Simulation->userDefinedSimulations[0]->IMPLICIT_SOLVER_FIXED_TIME_STEP;
	for (int i = 0; i < systemMatrixDim; i++) {
		if (i == 0) {
			if (brakePipe_FiniteElements[0]->railVehicleTypes[0] == 1) {
				forcingVector[i] = ((Locomotive*)brakePipe_FiniteElements[0]->railVehicles[0])->idealizedRelayValvePressure;
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					double term1 = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 3.0) * brakePipe_FiniteElements[0]->nodePressures[0];
					double term2 = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[0]->nodePressures[1];
					double term3 = brakePipe_FiniteElements[0]->leakage / 2.0;
					forcingVector[i] = term1 + term2 + term3;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					forcingVector[i] = ((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->pressureAfterTwoWayEOTActivation;
				}
			}
		}
		else if (i == 1) {
			if (brakePipe_FiniteElements[0]->railVehicleTypes[0] == 1) {
				double term1 = (1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 3.0) * brakePipe_FiniteElements[0]->nodeMValues[0];
				double term2 = (1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[0]->nodeMValues[1];
				double term3 = (brakePipe_FiniteElements[0]->effCValue * brakePipe_FiniteElements[0]->brakePipeFELength) / 2.0;
				forcingVector[i] = term1 + term2 - term3;
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					forcingVector[i] = 0.0;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					double term1 = (1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 3.0) * brakePipe_FiniteElements[0]->nodeMValues[0];
					double term2 = (1.0 / timeStep) * ((brakePipe_FiniteElements[0]->effArea * brakePipe_FiniteElements[0]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[0]->nodeMValues[1];
					double term3 = (brakePipe_FiniteElements[0]->effCValue * brakePipe_FiniteElements[0]->brakePipeFELength) / 2.0;
					forcingVector[i] = term1 + term2 - term3;
				}
			}
		}
		else if (i == systemMatrixDim - 2) {
			int numFE = brakePipe_FiniteElements.size();
			int numRV = brakePipe_FiniteElements[numFE - 1]->railVehicles.size();
			if (brakePipe_FiniteElements[numFE - 1]->railVehicleTypes[numRV - 1] == 1) {
				forcingVector[i] = ((Locomotive*)brakePipe_FiniteElements[numFE - 1]->railVehicles[numRV - 1])->idealizedRelayValvePressure;
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[numFE - 1]->railVehicles[numRV - 1])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					double term1 = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[numFE - 1]->nodePressures[0];
					double term2 = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 3.0) * brakePipe_FiniteElements[numFE - 1]->nodePressures[1];
					double term3 = brakePipe_FiniteElements[numFE - 1]->leakage / 2.0;
					forcingVector[i] = term1 + term2 + term3;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					forcingVector[i] = ((Car*)brakePipe_FiniteElements[numFE - 1]->railVehicles[numRV - 1])->endOfTrainDevice->pressureAfterTwoWayEOTActivation;
				}
			}
		}
		else if (i == systemMatrixDim - 1) {
			int numFE = brakePipe_FiniteElements.size();
			int numRV = brakePipe_FiniteElements[numFE - 1]->railVehicles.size();
			if (brakePipe_FiniteElements[numFE - 1]->railVehicleTypes[numRV - 1] == 1) {
				double term1 = (1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[numFE - 1]->nodeMValues[0];
				double term2 = (1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 3.0) * brakePipe_FiniteElements[numFE - 1]->nodeMValues[1];
				double term3 = (brakePipe_FiniteElements[numFE - 1]->effCValue * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 2.0;
				forcingVector[i] = term1 + term2 - term3;
			}
			else {
				if ((trainConsist->eotDeviceCapability == 1) ||
					((trainConsist->eotDeviceCapability == 2) &&
						(((Car*)brakePipe_FiniteElements[numFE - 1]->railVehicles[numRV - 1])->endOfTrainDevice->twoWayEOTActivatedBool == false))) {
					forcingVector[i] = 0.0;
				}
				else {  // (trainConsist->eotDeviceCapability == 2) && (((Car*)brakePipe_FiniteElements[0]->railVehicles[0])->endOfTrainDevice->twoWayEOTActivatedBool == true)
					double term1 = (1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[numFE - 1]->nodeMValues[0];
					double term2 = (1.0 / timeStep) * ((brakePipe_FiniteElements[numFE - 1]->effArea * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 3.0) * brakePipe_FiniteElements[numFE - 1]->nodeMValues[1];
					double term3 = (brakePipe_FiniteElements[numFE - 1]->effCValue * brakePipe_FiniteElements[numFE - 1]->brakePipeFELength) / 2.0;
					forcingVector[i] = term1 + term2 - term3;
				}
			}
		}
		else {
			int currFE1 = (int)floor(i / 2) - 1;
			int currFE2 = (int)floor(i / 2);
			for (int j = 0; j < systemMatrixDim; j++) {
				if (i % 2 == 0) {
					double term1 = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[currFE1]->nodePressures[0];
					double term2 = (((1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 3.0)) + ((1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 3.0))) * brakePipe_FiniteElements[currFE2]->nodePressures[0];
					double term3 = (1.0 / (TrainConsist::GAS_CONSTANT_AIR * trainConsist->airTemperature * timeStep)) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[currFE2]->nodePressures[1];
					double term4 = brakePipe_FiniteElements[currFE1]->leakage / 2.0;
					double term5 = brakePipe_FiniteElements[currFE2]->leakage / 2.0;
					forcingVector[i] = term1 + term2 + term3 + term4 + term5;
				}
				else {
					double term1 = (1.0 / timeStep) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[currFE1]->nodeMValues[0];
					double term2 = (((1.0 / timeStep) * ((brakePipe_FiniteElements[currFE1]->effArea * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 3.0)) + ((1.0 / timeStep) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 3.0))) * brakePipe_FiniteElements[currFE2]->nodeMValues[0];
					double term3 = (1.0 / timeStep) * ((brakePipe_FiniteElements[currFE2]->effArea * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 6.0) * brakePipe_FiniteElements[currFE2]->nodeMValues[1];
					double term4 = (brakePipe_FiniteElements[currFE1]->effCValue * brakePipe_FiniteElements[currFE1]->brakePipeFELength) / 2.0;
					double term5 = (brakePipe_FiniteElements[currFE2]->effCValue * brakePipe_FiniteElements[currFE2]->brakePipeFELength) / 2.0;
					forcingVector[i] = term1 + term2 + term3 - term4 - term5;
				}
			}
		}
	}
}


void BrakePipe_Cumulative::updatePressureAndVelocityAtBrakePipeFiniteElementNodes() {
	int numFE = brakePipe_FiniteElements.size();
	double* tempPressureVector = new double[numFE + 1];
	double* tempMVector = new double[numFE + 1];
	for (int j = 0; j < systemMatrixDim; j++) {
		if (j % 2 == 0) {
			tempPressureVector[(int)floor(j / 2.0)] = linearSystem->stateSpaceVector[j];
		}
		else {
			tempMVector[(int)floor(j / 2.0)] = linearSystem->stateSpaceVector[j];
		}
	}
	for (int j = 0; j < numFE; j++) {
		brakePipe_FiniteElements[j]->nodePressures[0] = tempPressureVector[j];
		brakePipe_FiniteElements[j]->nodePressures[1] = tempPressureVector[j + 1];
		brakePipe_FiniteElements[j]->nodeMValues[0] = tempMVector[j];
		brakePipe_FiniteElements[j]->nodeMValues[1] = tempMVector[j + 1];
	}
	delete[] tempPressureVector;
	delete[] tempMVector;
}

