//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Simulation.h"
#include "AuxiliaryReservoir.h"
#include "BrakeCylinder.h"
#include "BrakePipe_Cumulative.h"
#include "BrakePipe_FiniteElement.h"
#include "Car.h"
#include "ControlValve_Car.h"
#include "Coupler.h"
#include "CouplingSystem.h"
#include "EmergencyReservoir.h"
#include "EndOfTrainDevice.h"
#include "Function.h"
#include "InputFileReader_ForcedSpeed.h"
#include "InputFileReader_Simulation.h"
#include "Interval.h"
#include "LinearSystem.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Point.h"
#include "ResultsWriter.h"
#include "Track.h"
#include "TrainConsist.h"


Simulation::Simulation(InputFileReader_Simulation* inputFileReader_Simulation) : UserDefinedRRComponent(inputFileReader_Simulation, 20, 0) {
	componentType = 6;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables.push_back(new Function(this, i));
	}
	// User-inputted physical constants
	// 0	-->	Index of car to be saved
	// 1	-->	Index of car to be saved
	// 2	-->	Index of car to be saved
	// 3	-->	Index of car to be saved
	// 4	-->	Index of car to be saved
	// 5	-->	Index of car to be saved
	// 6	-->	Index of car to be saved
	// 7	-->	Index of car to be saved
	// 8	-->	Index of car to be saved
	// 9	-->	Index of car to be saved
	// 10	--> Index of car to be saved
	// 11	-->	Index of car to be saved
	// 12	-->	Index of car to be saved
	// 13	-->	Index of car to be saved
	// 14	-->	Index of car to be saved
	// 15	-->	Index of car to be saved
	// 16	-->	Index of car to be saved
	// 17	-->	Index of car to be saved
	// 18	-->	Index of car to be saved
	// 19	-->	Index of car to be saved
	this->inputFileReader_Simulation = inputFileReader_Simulation;
	START_UDRRC_STRING = "Simulation_";
	END_UDRRC_STRING = "_Simulation";
	railVehiclesToSaveCreatedBool = false;
	explicitSolverNumSteps = 0;
	// Physical constants US units
	// Indices 0-19		--> Rail vehicle numbers (position in train consist)
	for (int i = 0; i < physicalConstantsSize; i++) {
		PCSTR_US[i] = "";
	}
	// Physical constants integer boolean
	// Indices 0-19		--> Rail vehicle numbers (position in train consist)
	for (int i = 0; i < physicalConstantsSize; i++) {
		PCINT[i] = true;
	}
	// Physical constant minimum threshold values in US units
	// Indices 0-19		--> Rail vehicle numbers (position in train consist)
	for (int i = 0; i < physicalConstantsSize; i++) {
		PCMIN_US[i] = 1.0;
	}
	// Physical constant minimum threshold values in SI units
	// Indices 0-19		--> Rail vehicle numbers (position in train consist)
	for (int i = 0; i < physicalConstantsSize; i++) {
		PCMIN_SI[i] = PCMIN_US[i];
	}
	// Physical constant maximum threshold values in US units
	// Indices 0-19		--> Rail vehicle numbers (position in train consist)
	for (int i = 0; i < physicalConstantsSize; i++) {
		if (inputFileReader_Simulation->userDefinedTrainConsists.size() > 0) {
			PCMAX_US[i] = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size();
		}
	}
	// Physical constant maximum threshold values in SI units
	// Indices 0-19		--> Rail vehicle numbers (position in train consist)
	for (int i = 0; i < physicalConstantsSize; i++) {
		PCMAX_SI[i] = PCMAX_US[i];
	}
}


Simulation::~Simulation() {
	if (railVehiclesToSaveCreatedBool == true) {
		delete[] railVehiclesToSave;
	}
	delete resultsWriter_BrakePipes;
	delete resultsWriter_CouplerForces;
	delete resultsWriter_CouplerDisplacements;
}


void Simulation::simulate(InputFileReader_ForcedSpeed* inputFileReader_ForcedSpeed) {
	std::cout << "Simulation running..." << std::endl;
	std::cout << std::endl;
	calc_explicitSolverNumSteps();
	explicitSolverStepIndex = 0;
	explicitSolverTime = 0.0;
	explicitSolverTimeStep = EXPLICIT_SOLVER_FIXED_TIME_STEP;
	implicitSolverTime = 0.0;
	velocityApproxZeroBool = false;
	startTimeVelocityApproxZero = 2.0 * MAX_NUMBER_OF_SIMULATED_SECONDS;
	double implicitSolverTimeOfPreviousProgressUpdate = implicitSolverTime;
	double explicitSolverTimeLastWrittenResults = explicitSolverTime;
	bool headerLabelsWrittenBool = false;  // header labels written boolean
	bool numericalInstabilityBool = false;  // numerical instability boolean
	bool excessiveCouplerDisplacementBool = false;  // excessive coupler displacement boolean
	bool consistInsideTrackBoundariesBool = true;  // consist inside track boundaries boolean
	bool consistMovingForwardBool = true;  // consist moving forward boolean
	bool trainSpeedUnderMaxSpeedBool = true;  // train speed under maximum allowable speed boolean
	// Calculate rail vehicle types for train consist
	inputFileReader_Simulation->userDefinedTrainConsists[0]->calc_railVehicleTypes();
	// Calculate vector of cumulative brake pipes for train consist
	inputFileReader_Simulation->userDefinedTrainConsists[0]->calc_brakePipes_Cumulative();
	// Calculate rail vehicle types for each cumulative brake pipe
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->calc_railVehicleTypes();
	}
	// Calculate vector of brake pipe finite elements for each cumulative brake pipe
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->calc_brakePipe_FiniteElements();
	}
	// Initialize system matrix and forcing vector for cumulative brake pipes
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->initializeSystemMatrixAndForcingVector();
	}
	// Calculate rail vehicle types for each brake pipe finite element
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_railVehicleTypes();
		}
	}
	// Calculate vector of locomotives for train consist
	inputFileReader_Simulation->userDefinedTrainConsists[0]->calc_locomotives();
	// Initialize end-of-train device for first and last car in train consist, if needed
	if (inputFileReader_Simulation->userDefinedTrainConsists[0]->eotDeviceCapability == 2) {
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[0] == 0) {
			((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0])->initialize_endOfTrainDevice();
		}
		int numRailVehicles = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size();
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[numRailVehicles - 1] == 0) {
			((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[numRailVehicles - 1])->initialize_endOfTrainDevice();
		}
	}
	// Convert forced speed file to SI units
	if (inputFileReader_ForcedSpeed->inputFileExistsBool == true) {
		inputFileReader_ForcedSpeed->convertToSI();
	}
	// Convert track and train consist to SI units
	convertToSI();
	// Calculate track length
	inputFileReader_Simulation->userDefinedTracks[0]->calc_trackLength();
	// Calculate air viscosity
	inputFileReader_Simulation->userDefinedTrainConsists[0]->calc_airViscosity();
	// Calculate brake rigging leverage ratios
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_brakeRiggingLeverageRatio();
	}
	// Set diameter of first node and second node for brake pipe finite elements
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			for (int k = 0; k < BrakePipe_FiniteElement::NUMBER_OF_NODES_PER_BRAKE_PIPE_FE; k++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->nodeDiameters[k] = BrakePipe_FiniteElement::DIAMETER;
			}
		}
	}
	// Calculate effective diameter for each brake pipe finite element (which is the harmonic mean of the diameter at the two nodes)
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_effDiameter();
		}
	}
	// Calculate area of first node and second node for brake pipe finite elements
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeAreas();
		}
	}
	// Calculate effective area for each brake pipe finite element (which is the harmonic mean of the area at the two nodes)
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_effArea();
		}
	}
	// Calculate rail vehicle brake pipe lengths
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_brakePipeLength();
	}
	// Calculate brake pipe finite element lengths
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_brakePipeFELength();
		}
	}
	// Calculate X location of rail vehicle along respective brake pipe finite element
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_railVehicleXLocations();
		}
	}
	// Calculate locomotive automatic brake setting and idealized relay valve pressure
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentAutomaticBrakeValveSetting(false);
		inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_idealizedRelayValvePressure();
	}
	// Set initial pressures and 'm' values for brake pipe finite element nodes
	inputFileReader_Simulation->userDefinedTrainConsists[0]->calculateInitialPressuresAndMValuesForBPFENodes();
	// Calculate brake pipe air density, brake pipe air velocity, and car brake pipe pressure values
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
		for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeDensities();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeVelocities();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calculateBrakePipePressureForRailVehicles();
		}
	}
	// Configure coupling systems
	inputFileReader_Simulation->userDefinedTrainConsists[0]->configureCouplingSystems();
	// Place train consist on track
	placeTrainConsistOnTrack();
	// Calculate angle between top of rail and coupler for each rail vehicle
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_angleBetweenTopOfRailAndCoupler();
	}
	// Calculate angle between top of rail and center of gravity for each rail vehicle
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_angleBetweenTopOfRailAndCenterOfGravity();
	}
	// Calculate first square root term used in L/V calculation
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_firstSquareRootTermForLOverVCalc();
	}
	// Calculate second square root term used in L/V calculation
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_secondSquareRootTermForLOverVCalc();
	}
	// Initialize output results file writers
	initializeResultsWriters();
	// Set current index in forced speed file
	if ((inputFileReader_ForcedSpeed->inputFileExistsBool == true) && (inputFileReader_ForcedSpeed->totalNumberOfForcedSpeeds > 0)) {
		for (int i = 0; i < inputFileReader_ForcedSpeed->totalNumberOfForcedSpeeds; i++) {
			if (inputFileReader_ForcedSpeed->DBTBIFS == 0) {
				if (inputFileReader_Simulation->userDefinedTrainConsists[0]->locationOnTrack > inputFileReader_ForcedSpeed->fsiv[i]) {
					inputFileReader_ForcedSpeed->curind = i;
				}
			}
			else if (inputFileReader_ForcedSpeed->DBTBIFS == 1) {
				if (explicitSolverTime > inputFileReader_ForcedSpeed->fsiv[i]) {
					inputFileReader_ForcedSpeed->curind = i;
				}
			}
		}
	}
	// First iteration boolean
	bool firstIterBool = true;
	// Simulation loop
	do {
		// Calculate train consist position
		inputFileReader_Simulation->userDefinedTrainConsists[0]->calc_locationOnTrack();
		// Check for forced speed
		if ((inputFileReader_ForcedSpeed->inputFileExistsBool == true) && (inputFileReader_ForcedSpeed->totalNumberOfForcedSpeeds > 0)) {
			if (inputFileReader_ForcedSpeed->curind < (inputFileReader_ForcedSpeed->totalNumberOfForcedSpeeds - 1)) {
				if (inputFileReader_ForcedSpeed->DBTBIFS == 0) {
					if (inputFileReader_Simulation->userDefinedTrainConsists[0]->locationOnTrack > inputFileReader_ForcedSpeed->fsiv[inputFileReader_ForcedSpeed->curind + 1]) {
						inputFileReader_ForcedSpeed->curind = inputFileReader_ForcedSpeed->curind + 1;
						for (size_t j = 0; j <= (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); j++) {
							inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[j]->ssv[1] = inputFileReader_ForcedSpeed->fsdv[inputFileReader_ForcedSpeed->curind];
						}
					}
				}
				else if (inputFileReader_ForcedSpeed->DBTBIFS == 1) {
					if (explicitSolverTime > inputFileReader_ForcedSpeed->fsiv[inputFileReader_ForcedSpeed->curind + 1]) {
						inputFileReader_ForcedSpeed->curind = inputFileReader_ForcedSpeed->curind + 1;
						for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); j++) {
							inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[j]->ssv[1] = inputFileReader_ForcedSpeed->fsdv[inputFileReader_ForcedSpeed->curind];
						}
					}
				}
			}
		}
		// Calculate control valve operating mode for every car in train consist
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				((ControlValve_Car*)((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i])->controlValve)->calc_currentOperatingMode();
			}
		}
		// Calculate control valve mass flow rates
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				((ControlValve_Car*)((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i])->controlValve)->calc_mdot();
			}
		}
		// Calculate auxiliary reservoir pressures and emergency reservoir pressures
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i])->auxiliaryReservoir->calc_pressure();
				((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i])->emergencyReservoir->calc_pressure();
			}
		}
		// Calculate brake cylinder pressures
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->brakeCylinder->calc_pressure(firstIterBool);
		}
		// Calculate brake pipe leakage
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
			for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_leakage();
			}
		}
		// Calculate locomotive automatic brake setting and idealized relay valve pressure
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentAutomaticBrakeValveSetting(true);
			inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_idealizedRelayValvePressure();
		}
		// Calculate boundary condition for first and last rail vehicle in train consist (if first and/or last rail vehicles are cars)
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->eotDeviceCapability == 2) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[0] == 0) {
				((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0])->endOfTrainDevice->calc_pressureAfterTwoWayEOTActivation();
			}
			int numberOfRailVehicles = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size();
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[numberOfRailVehicles - 1] == 0) {
				((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[numberOfRailVehicles - 1])->endOfTrainDevice->calc_pressureAfterTwoWayEOTActivation();
			}
		}
		// Calculate reynolds number and wall friction factor for brake pipe nodes
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
			for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeReynoldsNumbers();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeWallFrictionFactors();
			}
		}
		// Calculate node 'C' values and effective 'C' value (this is the term related to the wall friction factor)
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
			for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeCValues();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_effCValue();
			}
		}
		// Integration step for brake pipes
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
			// Calculate system matrix
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->calc_systemMatrix();
			// Calculate forcing vector
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->calc_forcingVector();
			// Calculate brake pipe pressures and 'm' values for next time step
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->linearSystem->
				gaussElimination(inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->systemMatrix,
					inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->forcingVector,
					inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->systemMatrixDim);
			// Update pressure and velocity values at brake pipe finite element nodes
			inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->updatePressureAndVelocityAtBrakePipeFiniteElementNodes();
		}
		// Calculate brake pipe air density, brake pipe air velocity, and car brake pipe pressure values
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative.size(); i++) {
			for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements.size(); j++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeDensities();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calc_nodeVelocities();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->brakePipes_Cumulative[i]->brakePipe_FiniteElements[j]->calculateBrakePipePressureForRailVehicles();
			}
		}
		// Calculate reactive centrifugal force
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_globalReactiveCentrifugalForce();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_globalVector_globalReactiveCentrifugalForce();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calcLocal3DVecFromGlobalReactiveCentrifugalForce();
		}
		// Calculate gravitational force
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			if (firstIterBool == true) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_globalGravitationalForce();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_globalVector_globalGravitationalForce();
			}
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calcLocal3DVecFromGlobalGravitationalForce();
		}
		// Calculate sum of external tangential forces (due to reactive centrifugal force, gravity, propulsion resistance, and curving resistance) for each rail vehicle
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_sumExternalTangentialForces();
		}
		// Calculate locomotive throttle setting
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentThrottleSetting();
		}
		// Calculate throttle force of each rail vehicle (note that throttle force is set to zero for cars)
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_throttleForce();
		}
		// Calculate locomotive independent brake and dynamic brake settings
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentIndependentBrakeValveSetting();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentDynamicBrakeSetting();
		}
		// Calculate brake cylinder piston force, normal force between brake shoe and wheel, and retarding brake force
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->brakeCylinder->calc_pistonForce();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->brakeCylinder->calc_normalShoeWheelForce();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->brakeCylinder->calc_retardingBrakeForce();
		}
		// Calculate braking force of each rail vehicle
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_brakingForce();
		}
		// Calculate angle (rotation about z axis) of each rail vehicle
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_theta();
		}
		// Calculate angle between current rail vehicle and adjacent leading rail vehicle
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_alpha_leadingRailVehicle();
		}
		// Calculate angle between current rail vehicle and adjacent trailing rail vehicle
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_alpha_trailingRailVehicle();
		}
		// Update implicit solver time
		implicitSolverTime = implicitSolverTime + IMPLICIT_SOLVER_FIXED_TIME_STEP;
		// Explicit solver loop
		do {
			// Calculate force on leading rail vehicle for each coupling system
			for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems.size(); i++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_centerToCenterDistance(true);
				inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_displacement();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_totalForceOnLeadingRailVehicle();  // also updates coupler displacements
			}
			// Calculate next time step results
			numericalInstabilityBool = explicitSolverIntegrationStep();
			// Depending on boolean returned by 'explicitSolverIntegrationStep' method, either terminate simulation or update progress bar
			if (numericalInstabilityBool == true) {
				closeResultsWriters(true);
				std::cout << "Simulation has been terminated due to the integration time step being too small." << std::endl;
				std::cout << std::endl;
				std::cout << "Press Enter to end program";
				std::cin.ignore();
				std::exit(EXIT_SUCCESS);
			}
			else {
				// Calculate force on leading rail vehicle for each coupling system
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_centerToCenterDistance(true);
					inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_displacement();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_totalForceOnLeadingRailVehicle();  // also updates coupler displacements
				}
				// Calculate tangential component of leading and trailing coupler for each rail vehicle
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_tangentialForceDueToLeadingCoupler();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_tangentialForceDueToTrailingCoupler();
				}
				// Calculate lateral component of leading and trailing coupler for each rail vehicle
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lateralForceDueToLeadingCoupler();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lateralForceDueToTrailingCoupler();
				}
				// Calculate force of leading truck on track
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_forceOnTrackDueToLeadingTruck();
				}
				// Calculate force of trailing truck on track
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_forceOnTrackDueToTrailingTruck();
				}
				// Calculate vertical force on each side of leading truck and trailing truck
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_verticalForceRightSideLeadingTruck();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_verticalForceLeftSideLeadingTruck();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_verticalForceRightSideTrailingTruck();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_verticalForceLeftSideTrailingTruck();
				}
				// Calculate L/V ratio for each side of leading truck and trailing truck
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lOverVForRightSideOfLeadingTruck();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lOverVForLeftSideOfLeadingTruck();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lOverVForRightSideOfTrailingTruck();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lOverVForLeftSideOfTrailingTruck();
				}
				// Calculate maximum L/V ratio
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_lOverVMax();
				}
				// Calculate locomotive automatic brake setting (if locomotive operator is distance-based), independent brake setting, 
				// dynamic brake setting, and throttle setting
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentAutomaticBrakeValveSetting(false);
					inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentIndependentBrakeValveSetting();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentDynamicBrakeSetting();
					inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[i]->calc_currentThrottleSetting();
				}
				// Write single time step results, if necessary
				if ((explicitSolverTime - explicitSolverTimeLastWrittenResults) >= (1.0 / sampleRate)) {
					explicitSolverTimeLastWrittenResults = explicitSolverTime;
					if (headerLabelsWrittenBool == false) {
						for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
							if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->printResultsBool == true) {
								inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->results(this, true);
							}
						}
						writeResults_BrakePipePressures(true);
						writeResults_AuxiliaryReservoirPressures(true);
						writeResults_EmergencyReservoirPressures(true);
						writeResults_CouplerForces(true);
						writeResults_CouplerDisplacements(true);
						headerLabelsWrittenBool = true;
					}
					else {
						for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
							if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->printResultsBool == true) {
								inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->results(this, false);
							}
						}
						writeResults_BrakePipePressures(false);
						writeResults_AuxiliaryReservoirPressures(false);
						writeResults_EmergencyReservoirPressures(false);
						writeResults_CouplerForces(false);
						writeResults_CouplerDisplacements(false);
						headerLabelsWrittenBool = true;
					}
				}
			}
		} while (explicitSolverTime < implicitSolverTime);
		// Update progress
		if ((implicitSolverTime - implicitSolverTimeOfPreviousProgressUpdate) >= ELAPSED_TIME_FOR_UPDATE_PROGRESS) {
			int progress = (int)((implicitSolverTime / MAX_NUMBER_OF_SIMULATED_SECONDS) * 100.0);
			std::cout << progress << "% complete" << std::endl;
			std::cout << std::endl;
			implicitSolverTimeOfPreviousProgressUpdate = implicitSolverTime;
		}
		// Check if any coupler has excessive displacement
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems.size(); i++) {
			for (size_t j = 0; j < inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->couplers.size(); j++) {
				int num_intervals = inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->couplers[j]->physicalVariables[0]->intervals.size();
				if ((inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->couplers[j]->displacement > 
					inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->couplers[j]->physicalVariables[0]->intervals[num_intervals - 1]->points[1]->x) || 
					(inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->couplers[j]->displacement <
						inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->couplers[j]->physicalVariables[0]->intervals[0]->points[0]->x)) {
					excessiveCouplerDisplacementBool = true;
					break;
				}
			}
			if (excessiveCouplerDisplacementBool == true) {
				break;
			}
		}
		if (excessiveCouplerDisplacementBool == true) {
			closeResultsWriters(true);
			std::cout << "Simulation has been terminated due to excessive coupler displacement." << std::endl;
			std::cout << std::endl;
			std::cout << "Press Enter to end program";
			std::cin.ignore();
			std::exit(EXIT_SUCCESS);
		}
		// Check if train consist is within track boundary limits
		consistInsideTrackBoundariesBool = checkTrainConsistIsOnTrack();
		if (consistInsideTrackBoundariesBool == false) {
			closeResultsWriters(true);
			std::cout << "Simulation has been terminated due to train consist reaching end of track." << std::endl;
			std::cout << std::endl;
			std::cout << "Press Enter to end program";
			std::cin.ignore();
			std::exit(EXIT_SUCCESS);
		}
		// Check if train consist has had zero or negative velocity for 30 seconds or more
		consistMovingForwardBool = checkTrainConsistIsMoving();
		if (consistMovingForwardBool == false) {
			closeResultsWriters(true);
			std::cout << "Simulation has been terminated due to train consist speed being below " << APPROX_ZERO_VELOCITY_US << " miles per hour for at least " << MAX_WAIT_POSITIVE_VELOCITY << " seconds." << std::endl;
			std::cout << std::endl;
			std::cout << "Press Enter to end program";
			std::cin.ignore();
			std::exit(EXIT_SUCCESS);
		}
		// Check if train speed exceeds maximum allowable train speed
		trainSpeedUnderMaxSpeedBool = checkTrainSpeedIsUnderMaximumAllowableSpeed();
		if (trainSpeedUnderMaxSpeedBool == false) {
			closeResultsWriters(true);
			std::cout << "Simulation has been terminated due to train consist speed exceeding maximum allowable speed of " << MAX_ALLOWABLE_TRAIN_SPEED_US << " miles per hour." << std::endl;
			std::cout << std::endl;
			std::cout << "Press Enter to end program";
			std::cin.ignore();
			std::exit(EXIT_SUCCESS);
		}
		// Update first iteration boolean
		firstIterBool = false;
	} while (implicitSolverTime < MAX_NUMBER_OF_SIMULATED_SECONDS);
	// Close result writers and print message to console that simulation was completed successfully
	closeResultsWriters(true);
	std::cout << "Simulation has been terminated due to the maximum number of simulated seconds (" << MAX_NUMBER_OF_SIMULATED_SECONDS << " seconds) being exceeded." << std::endl;
	std::cout << std::endl;
	std::cout << "Press Enter to end program";
	std::cin.ignore();
	std::exit(EXIT_SUCCESS);
}


bool Simulation::explicitSolverIntegrationStep() {
	double tcs = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0]->ssv[1];  // train consist speed
	double curr_est_err = 0.0;  // current estimated error
	// Loop through RK4 or RKF45 algorithm steps and loop through state space components
	do {
		if ((explicitSolverTimeStep < EXPLICIT_SOLVER_MIN_TIME_STEP)) {
			return true;
		}
		for (int l = 0; l < explicitSolverNumSteps; l++) {
			// Update explicit solver step index
			explicitSolverStepIndex = l;
			// Calculate state space variables
			for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_ssvApp(inputFileReader_Simulation);
			}
			// Calculate force on leading rail vehicle
			for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems.size(); i++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_centerToCenterDistance(false);
				inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_displacement();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[i]->calc_totalForceOnLeadingRailVehicle();  // also updates coupler displacements
			}
			// Calculate tangential component of leading and trailing coupler for each rail vehicle
			for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_tangentialForceDueToLeadingCoupler();
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_tangentialForceDueToTrailingCoupler();
			}
			// Calculate state space variable rates of change
			for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_ssvDot();
			}
			// Calculate k values for current algorithm step (to be used in next algorithm step)
			for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_kVals();
			}
			// Calculate result and error
			if (l == (explicitSolverNumSteps - 1)) {
				for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
					inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_res4();
					if (explicitSolverType == 1) {
						// Rail vehicle error
						inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_res5();
						curr_est_err = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->calc_ee();
						if (curr_est_err > EXPLICIT_SOLVER_MAX_ERROR_THRESHOLD) {
							break;
						}
					}
				}
			}
		}
		// Calculate revised time step, if necessary
		if ((explicitSolverType == 1) && (curr_est_err > EXPLICIT_SOLVER_MAX_ERROR_THRESHOLD)) {
			explicitSolverTimeStep = explicitSolverTimeStep * 0.75;
		}
	} while (curr_est_err > EXPLICIT_SOLVER_MAX_ERROR_THRESHOLD);
	// Update current time
	explicitSolverTime = explicitSolverTime + explicitSolverTimeStep;
	// Update state space variables
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->update_ssv();
	}
	// Calculate time step for next integration iteration
	if (explicitSolverType == 1) {
		explicitSolverTimeStep = explicitSolverTimeStep * 1.25;
		if (explicitSolverTimeStep > EXPLICIT_SOLVER_MAX_TIME_STEP) {
			explicitSolverTimeStep = EXPLICIT_SOLVER_MAX_TIME_STEP;
		}
	}
	else {
		explicitSolverTimeStep = EXPLICIT_SOLVER_FIXED_TIME_STEP;
	}
	// Return 'false'
	return false;
}


void Simulation::calc_explicitSolverNumSteps() {
	if (explicitSolverType == 0) {
		explicitSolverNumSteps = 4;
	}
	else {
		explicitSolverNumSteps = 6;
	}
}


std::string Simulation::load() {
	// Load method of integration
	bool mintlb = false;  // method of integration loaded boolean
	do {
		inputFileReader_Simulation->nextLine();
		if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
			std::vector<std::string> strvec = InputFileReader::split_string(inputFileReader_Simulation->currentLine, ',');
			if (strvec.size() != 1) {
				return std::string("Wrong number of conditions entered.  An integer representing integration method expected.");
			}
			size_t numcommas = std::count(inputFileReader_Simulation->currentLine.begin(), inputFileReader_Simulation->currentLine.end(), ',');
			if (numcommas != strvec.size() - 1) {
				return std::string("Wrong number of commas entered.  No commas expected.");
			}
			for (size_t i = 0; i < strvec.size(); i++) {
				try {
					explicitSolverType = stoi(strvec[i]);
					double explicitSolverType_check = stod(strvec[i]);
					if (explicitSolverType != explicitSolverType_check) {
						return std::string("Integration method could not be parse into integer.  Integration method value must be equal to '0' for ") +
							std::string("fixed time step integration or equal to '1' for variable time step integration.");
					}
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Integration method could not be parse into integer.  Integration method value must be equal to '0' for ") +
						std::string("fixed time step integration or equal to '1' for variable time step integration.");
				}
				if ((explicitSolverType != 0) && (explicitSolverType != 1)) {
					return std::string("Integration method value must be equal to '0' for fixed time step integration or equal to '1' for variable ") +
						std::string("time step integration.");
				}
			}
			mintlb = true;
		}
	} while (mintlb == false);
	// Load sampling rate
	bool samplratlb = false;  // sampling rate loaded boolean
	do {
		inputFileReader_Simulation->nextLine();
		if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
			std::vector<std::string> strvec = InputFileReader::split_string(inputFileReader_Simulation->currentLine, ',');
			if (strvec.size() != 1) {
				return std::string("Wrong number of conditions entered.  An integer representing sampling rate expected.");
			}
			size_t numcommas = std::count(inputFileReader_Simulation->currentLine.begin(), inputFileReader_Simulation->currentLine.end(), ',');
			if (numcommas != strvec.size() - 1) {
				return std::string("Wrong number of commas entered.  No commas expected.");
			}
			for (size_t i = 0; i < strvec.size(); i++) {
				try {
					sampleRate = stoi(strvec[i]);
					double sampleRate_check = stod(strvec[i]);
					if (sampleRate != sampleRate_check) {
						return std::string("Sampling rate could not be parsed into integer.  Sampling rate value must be an integer greater than or ") +
							std::string("equal to ") + std::to_string(MINIMUM_SAMPLING_RATE) + std::string(" hertz and less than or equal to ") +
							std::to_string(MAXIMUM_SAMPLING_RATE) + std::string(" hertz.");
					}
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Sampling rate could not be parsed into integer.  Sampling rate value must be an integer greater than or ") +
						std::string("equal to ") + std::to_string(MINIMUM_SAMPLING_RATE) + std::string(" hertz and less than or equal to ") +
						std::to_string(MAXIMUM_SAMPLING_RATE) + std::string(" hertz.");
				}
				if ((sampleRate < MINIMUM_SAMPLING_RATE) || (sampleRate > MAXIMUM_SAMPLING_RATE)) {
					return std::string("Sampling rate value must be an integer greater than or equal to ") + std::to_string(MINIMUM_SAMPLING_RATE) +
						std::string(" hertz and less than or equal to ") + std::to_string(MAXIMUM_SAMPLING_RATE) + std::string(" hertz.");
				}
			}
			samplratlb = true;
		}
	} while (samplratlb == false);
	// Load physical constants (index of rail vehicles to be saved)
	if (physicalConstantsSize > 0) {
		bool physicalConstants_Loaded = false;
		do {
			inputFileReader_Simulation->nextLine();
			if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
				std::vector<std::string> strvec = InputFileReader::split_string(inputFileReader_Simulation->currentLine, ',');
				if (strvec.size() < 1) {
					return std::string("At least one rail vehicle index expected.");
				}
				if ((int)strvec.size() > physicalConstantsSize) {
					return std::string("Too many rail vehicle indices entered.  A maximum number of ") + std::to_string(physicalConstantsSize) + std::string(" can be saved.");
				}
				size_t numcommas = std::count(inputFileReader_Simulation->currentLine.begin(), inputFileReader_Simulation->currentLine.end(), ',');
				if (numcommas != strvec.size() - 1) {
					return std::string("Wrong number of commas entered.");
				}
				for (size_t i = 0; i < strvec.size(); i++) {
					try {
						if (PCINT[i] == true) {
							physicalConstants[i] = stoi(strvec[i]);
							double y_check = stod(strvec[i]);
							if (physicalConstants[i] != y_check) {
								return std::string("Rail vehicle index could not be parsed into integer.");
							}
						}
						else {
							physicalConstants[i] = stod(strvec[i]);
						}
					}
					catch (const std::invalid_argument& ia) {
						if (PCINT[i] == true) {
							return std::string("Rail vehicle index could not be parsed into integer.");
						}
						else {
							return std::string("Rail vehicle index could not be parsed into number.");
						}
					}
					if ((physicalConstants[i] < PCMIN_US[i]) || (physicalConstants[i] > PCMAX_US[i])) {
						return std::string("Rail vehicle index values must be greater than or equal to 1 and less than or equal to total number of rail vehicles (") +
							std::to_string(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size()) + std::string(") in train consist.");
					}
				}
				physicalConstants_Loaded = true;
				if (railVehiclesToSaveCreatedBool == true) {
					delete[] railVehiclesToSave;
				}
				railVehiclesToSaveSize = strvec.size();
				railVehiclesToSave = new int[railVehiclesToSaveSize];
				for (int i = 0; i < railVehiclesToSaveSize; i++) {
					physicalConstants[i] = physicalConstants[i] - 1;
				}
			}
		} while (physicalConstants_Loaded == false);
	}
	// Load physical constants alternative names
	loadPhysicalConstantAlternateNames();
	// Sets print results booleans for rail vehicles
	for (int i = 0; i < railVehiclesToSaveSize; i++) {
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[railVehiclesToSave[i]]->printResultsBool = true;
	}
	// Read next line
	inputFileReader_Simulation->nextLine();
	// Find end-of-component string
	do {
		if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
			return std::string("Comment line or line indicating end of component expected.");
		}
		inputFileReader_Simulation->nextLine();
	} while (inputFileReader_Simulation->currentLine.compare(END_UDRRC_STRING) != 0);
	return InputFileReader::VALID_INPUT_STRING;
}


void Simulation::convertToSI() {
	inputFileReader_Simulation->userDefinedTracks[0]->convertToSI();
	inputFileReader_Simulation->userDefinedTrainConsists[0]->convertToSI();
}


void Simulation::loadPhysicalConstantAlternateNames() {
	for (int i = 0; i < railVehiclesToSaveSize; i++) {
		railVehiclesToSave[i] = (int)physicalConstants[i];
	}
}


void Simulation::placeTrainConsistOnTrack() {
	double cp = 0.0;  // current position
	for (int i = (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
		if (i == inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1) {
			int numrvs; // number of rail vehicles
			numrvs = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size();
			double trvlen;  // trailing rail vehicle length
			trvlen = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[numrvs - 1]->length;
			cp = Simulation::TRACK_SPATIAL_CUSHION + (trvlen / 2.0);
		}
		else {
			cp = cp + (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i + 1]->length / 2.0) +
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i + 1]->couplers[0]->displacement +
				inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->couplers[1]->displacement +
				(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->length / 2.0);
		}
		inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->ssv[0] = cp;
	}
}


bool Simulation::checkTrainConsistIsOnTrack() {
	double tsl;  // track start location
	tsl = 0.0;
	double tel;  // track end location
	tel = inputFileReader_Simulation->userDefinedTracks[0]->trackLength;
	double lvloc;  // leading vehicle location
	lvloc = inputFileReader_Simulation->userDefinedTrainConsists[0]->locationOnTrack;
	double lvlen;  // leading vehicle length
	lvlen = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0]->length;
	int nrvs = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size();
	double tvloc;  // trailing vehicle location
	tvloc = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[nrvs - 1]->ssv[0];
	double tvlen;  // trailing vehicle length
	tvlen = inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[nrvs - 1]->length;
	if (((lvloc + (lvlen / 2.0)) > tel) || ((tvloc - (tvlen / 2.0)) < tsl)) {
		// Update progress
		int progress = 100;
		std::cout << progress << "% complete" << std::endl;
		std::cout << std::endl;
		return false;
	}
	else {
		return true;
	}
}


bool Simulation::checkTrainConsistIsMoving() {
	if ((velocityApproxZeroBool == true) && (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0]->ssv[1] < APPROX_ZERO_VELOCITY)) {
		if ((explicitSolverTime - startTimeVelocityApproxZero) > MAX_WAIT_POSITIVE_VELOCITY) {
			// Update progress
			int progress = 100;
			std::cout << progress << "% complete" << std::endl;
			std::cout << std::endl;
			return false;
		}
	}
	else if ((velocityApproxZeroBool == true) && (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0]->ssv[1] > APPROX_ZERO_VELOCITY)) {
		velocityApproxZeroBool = false;
		startTimeVelocityApproxZero = 2.0 * MAX_NUMBER_OF_SIMULATED_SECONDS;
	}
	else if ((velocityApproxZeroBool == false) && (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0]->ssv[1] < APPROX_ZERO_VELOCITY)) {
		velocityApproxZeroBool = true;
		startTimeVelocityApproxZero = explicitSolverTime;
	}
	return true;
}


bool Simulation::checkTrainSpeedIsUnderMaximumAllowableSpeed() {
	if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[0]->ssv[1] > MAX_ALLOWABLE_TRAIN_SPEED) {
		// Update progress
		int progress = 100;
		std::cout << progress << "% complete" << std::endl;
		std::cout << std::endl;
		return false;
	}
	else {
		return true;
	}
}


void Simulation::writeResults_BrakePipePressures(bool whb) {
	if (whb == true) {
		std::vector<std::string> headers_BrakePipes;
		headers_BrakePipes.push_back("Time (s)");
		for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				headers_BrakePipes.push_back(std::to_string(i + 1) + std::string(". Car brake pipe pressure (psi)"));
			}
			else {
				headers_BrakePipes.push_back(std::to_string(i + 1) + std::string(". Locomotive brake pipe pressure (psi)"));
			}
		}
		resultsWriter_BrakePipes->writeLine(headers_BrakePipes);
	}
	std::vector<std::string> results_BrakePipes;
	results_BrakePipes.push_back(std::to_string(explicitSolverTime));
	double temp_var;
	for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
		temp_var = UnitConverter::pa_To_Psi(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->brakePipeAirPressure);
		results_BrakePipes.push_back(std::to_string(temp_var));
	}
	resultsWriter_BrakePipes->writeLine(results_BrakePipes);
}


void Simulation::writeResults_AuxiliaryReservoirPressures(bool whb) {
	if (whb == true) {
		std::vector<std::string> headers_AuxiliaryReservoirs;
		headers_AuxiliaryReservoirs.push_back("Time (s)");
		for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				headers_AuxiliaryReservoirs.push_back(std::to_string(i + 1) + std::string(". Car auxiliary reservoir pressure (psi)"));
			}
			else {
				headers_AuxiliaryReservoirs.push_back(std::to_string(i + 1) + std::string(". Locomotive auxiliary reservoir pressure (psi)"));
			}
		}
		resultsWriter_AuxiliaryReservoirs->writeLine(headers_AuxiliaryReservoirs);
	}
	std::vector<std::string> results_AuxiliaryReservoirs;
	results_AuxiliaryReservoirs.push_back(std::to_string(explicitSolverTime));
	double temp_var;
	for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
			temp_var = UnitConverter::pa_To_Psi(((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i])->auxiliaryReservoir->pressure);
		}
		else {
			temp_var = UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE);
		}
		results_AuxiliaryReservoirs.push_back(std::to_string(temp_var));
	}
	resultsWriter_AuxiliaryReservoirs->writeLine(results_AuxiliaryReservoirs);
}


void Simulation::writeResults_EmergencyReservoirPressures(bool whb) {
	if (whb == true) {
		std::vector<std::string> headers_EmergencyReservoirs;
		headers_EmergencyReservoirs.push_back("Time (s)");
		for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				headers_EmergencyReservoirs.push_back(std::to_string(i + 1) + std::string(". Car emergency reservoir pressure (psi)"));
			}
			else {
				headers_EmergencyReservoirs.push_back(std::to_string(i + 1) + std::string(". Locomotive emergency reservoir pressure (psi)"));
			}
		}
		resultsWriter_EmergencyReservoirs->writeLine(headers_EmergencyReservoirs);
	}
	std::vector<std::string> results_EmergencyReservoirs;
	results_EmergencyReservoirs.push_back(std::to_string(explicitSolverTime));
	double temp_var;
	for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
			temp_var = UnitConverter::pa_To_Psi(((Car*)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i])->emergencyReservoir->pressure);
		}
		else {
			temp_var = UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE);
		}
		results_EmergencyReservoirs.push_back(std::to_string(temp_var));
	}
	resultsWriter_EmergencyReservoirs->writeLine(results_EmergencyReservoirs);
}


void Simulation::writeResults_CouplerForces(bool whb) {
	if (whb == true) {
		std::vector<std::string> headers_CouplerForces;
		headers_CouplerForces.push_back("Time (s)");
		for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				headers_CouplerForces.push_back(std::to_string(i + 1) + std::string(". Car trailing coupler force (pounds)"));
			}
			else {
				headers_CouplerForces.push_back(std::to_string(i + 1) + std::string(". Locomotive trailing coupler force (pounds)"));
			}
		}
		resultsWriter_CouplerForces->writeLine(headers_CouplerForces);
	}
	std::vector<std::string> results_CouplerForces;
	results_CouplerForces.push_back(std::to_string(explicitSolverTime));
	double temp_var;
	for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
		if (i == (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1)) {
			results_CouplerForces.push_back("0.0");
		}
		else {
			temp_var = pow(pow(UnitConverter::n_To_Lb(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->tangentialForceDueToTrailingCoupler), 2) +
				pow(UnitConverter::n_To_Lb(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->lateralForceDueToTrailingCoupler), 2), 0.5);
			results_CouplerForces.push_back(std::to_string(temp_var));
		}
	}
	resultsWriter_CouplerForces->writeLine(results_CouplerForces);
}


void Simulation::writeResults_CouplerDisplacements(bool whb) {
	if (whb == true) {
		std::vector<std::string> headers_CouplerDisplacements;
		headers_CouplerDisplacements.push_back("Time (s)");
		for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicleTypes[i] == 0) {
				headers_CouplerDisplacements.push_back(std::to_string(i + 1) + std::string(". Car trailing coupler displacement (inches)"));
			}
			else {
				headers_CouplerDisplacements.push_back(std::to_string(i + 1) + std::string(". Locomotive trailing coupler displacement (inches)"));
			}
		}
		resultsWriter_CouplerDisplacements->writeLine(headers_CouplerDisplacements);
	}
	std::vector<std::string> results_CouplerDisplacements;
	results_CouplerDisplacements.push_back(std::to_string(explicitSolverTime));
	double temp_var;
	for (int i = ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i >= 0; i--) {
		if (i == (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1)) {
			results_CouplerDisplacements.push_back("0.0");
		}
		else {
			temp_var = UnitConverter::m_To_In(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->couplers[1]->displacement);
			results_CouplerDisplacements.push_back(std::to_string(temp_var));
		}
	}
	resultsWriter_CouplerDisplacements->writeLine(results_CouplerDisplacements);
}


void Simulation::initializeResultsWriters() {
	for (size_t i = 0; i <= (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1); i++) {
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->printResultsBool == true) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->initialize_resultsWriter(this);
		}
	}
	resultsWriter_BrakePipes = new ResultsWriter(inputFileReader_Simulation->inputFileDirectoryPath + "/" + inputFileReader_Simulation->inputFileName + "_brake_pipe_pressures.csv", false);
	resultsWriter_AuxiliaryReservoirs = new ResultsWriter(inputFileReader_Simulation->inputFileDirectoryPath + "/" + inputFileReader_Simulation->inputFileName + "_auxiliary_reservoir_pressures.csv", false);
	resultsWriter_EmergencyReservoirs = new ResultsWriter(inputFileReader_Simulation->inputFileDirectoryPath + "/" + inputFileReader_Simulation->inputFileName + "_emergency_reservoir_pressures.csv", false);
	resultsWriter_CouplerForces = new ResultsWriter(inputFileReader_Simulation->inputFileDirectoryPath + "/" + inputFileReader_Simulation->inputFileName + "_coupler_forces.csv", false);
	resultsWriter_CouplerDisplacements = new ResultsWriter(inputFileReader_Simulation->inputFileDirectoryPath + "/" + inputFileReader_Simulation->inputFileName + "_coupler_displacements.csv", false);
}


void Simulation::closeResultsWriters(bool simulationSuccessfulBool) {
	for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
		if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->printResultsBool == true) {
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->resultsWriter->ofs->flush();
			inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->resultsWriter->ofs->close();
		}
	}
	resultsWriter_BrakePipes->ofs->flush();
	resultsWriter_BrakePipes->ofs->close();
	resultsWriter_AuxiliaryReservoirs->ofs->flush();
	resultsWriter_AuxiliaryReservoirs->ofs->close();
	resultsWriter_EmergencyReservoirs->ofs->flush();
	resultsWriter_EmergencyReservoirs->ofs->close();
	resultsWriter_CouplerForces->ofs->flush();
	resultsWriter_CouplerForces->ofs->close();
	resultsWriter_CouplerDisplacements->ofs->flush();
	resultsWriter_CouplerDisplacements->ofs->close();
	if (simulationSuccessfulBool == false) {
		for (size_t i = 0; i < inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size(); i++) {
			if (inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->printResultsBool == true) {
				remove(inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[i]->resultsWriter->fp.c_str());
			}
		}
		remove(resultsWriter_BrakePipes->fp.c_str());
		remove(resultsWriter_AuxiliaryReservoirs->fp.c_str());
		remove(resultsWriter_EmergencyReservoirs->fp.c_str());
		remove(resultsWriter_CouplerForces->fp.c_str());
		remove(resultsWriter_CouplerDisplacements->fp.c_str());
	}
}

